"""Process entry point: FastAPI + rclpy + bag rotator, one binary.

Design:
  * The rclpy node is owned by a MultiThreadedExecutor spun on a daemon
    thread, so subscriber callbacks and the TF listener keep draining
    independently of HTTP activity.
  * FastAPI/uvicorn runs on the main asyncio loop. HTTP handlers call
    blocking rclpy methods through ``asyncio.to_thread``.
  * The BagRotator is started before uvicorn so recording begins the
    moment the process is up.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import threading
from typing import Optional

import rclpy
import uvicorn
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.executors import MultiThreadedExecutor

from .api.rest import build_app
from .bag_rotator import BagRotator
from .patrol_controller import PatrolController
from .ros_bridge import RosBridge

LOG = logging.getLogger('typego_web_gateway')


def _locate_frontend() -> Optional[str]:
    try:
        share = get_package_share_directory('typego_web_gateway')
    except PackageNotFoundError:
        return None
    candidate = os.path.join(share, 'frontend')
    return candidate if os.path.isdir(candidate) else None


def _parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog='gateway_node')
    parser.add_argument('--host', default=os.environ.get('TYPEGO_GATEWAY_HOST', '0.0.0.0'))
    parser.add_argument('--port', type=int,
                        default=int(os.environ.get('TYPEGO_GATEWAY_PORT', '8080')))
    parser.add_argument('--robot-namespace',
                        default=os.environ.get('TYPEGO_ROBOT_NS', ''))
    parser.add_argument('--bag-dir',
                        default=os.environ.get(
                            'TYPEGO_BAG_DIR',
                            os.path.expanduser('~/.typego/bags'),
                        ))
    parser.add_argument('--bag-chunk-seconds', type=int,
                        default=int(os.environ.get('TYPEGO_BAG_CHUNK_SEC', '600')))
    parser.add_argument('--bag-retain', type=int,
                        default=int(os.environ.get('TYPEGO_BAG_RETAIN', '6')))
    parser.add_argument('--no-bag', action='store_true',
                        help='disable always-on rosbag recording')
    # Ignore unknown args so the ros2 launcher can pass --ros-args etc.
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    args = _parse_args(argv)

    rclpy.init(args=argv)
    bridge = RosBridge(robot_namespace=args.robot_namespace)
    patrol = PatrolController(bridge)

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    ros_thread = threading.Thread(
        target=executor.spin, name='rclpy_executor', daemon=True,
    )
    ros_thread.start()

    bag_rotator: Optional[BagRotator] = None
    if not args.no_bag:
        bag_rotator = BagRotator(
            bag_dir=args.bag_dir,
            chunk_seconds=args.bag_chunk_seconds,
            retain=args.bag_retain,
        )
        bag_rotator.start()

    frontend_dir = _locate_frontend()
    if frontend_dir:
        LOG.info('serving frontend from %s', frontend_dir)
    else:
        LOG.info('frontend bundle not found; API only')

    app = build_app(bridge, patrol, bag_rotator, frontend_dir=frontend_dir)

    config = uvicorn.Config(
        app, host=args.host, port=args.port,
        log_level='info', loop='asyncio', lifespan='on',
    )
    server = uvicorn.Server(config)

    stop_event = threading.Event()

    def _handle_signal(signum, _frame):
        LOG.info('received signal %s, shutting down', signum)
        server.should_exit = True
        stop_event.set()

    # uvicorn installs its own handlers; we still want clean bag teardown.
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handle_signal)
        except ValueError:
            pass

    try:
        asyncio.run(server.serve())
    finally:
        if bag_rotator is not None:
            bag_rotator.stop()
        executor.shutdown()
        try:
            bridge.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
