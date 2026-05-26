#!/usr/bin/env python3
"""Save the current SLAM map to src/typego_sdk/resource/Map-<name>/.

Supports both autonomy modes:
  full: ARISE SLAM           (/save_slam_map + /save_map)
  base: slam_toolbox         (/slam_toolbox/serialize_map, writes + init_pose.json)

Paths are host-relative: the SLAM node (wherever it's running — host via
`make launch`, or inside `typego-sdk` container via `make docker_open`)
must see the destination directory at the same path. Uses rclpy clients
instead of `ros2 service call`, which hangs on long-running services under
FastDDS+iceoryx.

Usage:
  python3 scripts/save_map.py <name>
  python3 scripts/save_map.py <name> --mode full
  python3 scripts/save_map.py <name> --mode base --robot-id 1
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import shutil
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MAP_RESOURCE_DIR = PROJECT_ROOT / "src/typego_sdk/resource"
INSTALL_MAP_RESOURCE_DIR = PROJECT_ROOT / "install/typego_sdk/share/typego_sdk/resource"
RUNTIME_ENV = Path("/tmp/typego-runtime.env")
SERVICE_TIMEOUT_S = 120.0
TF_TIMEOUT_S = 5.0
VGRAPH_TIMEOUT_S = 30.0


def _load_runtime_env_if_missing() -> None:
    """Fallback: pull AUTONOMY_TYPE/ROBOT_ID/etc. from /tmp/typego-runtime.env."""
    if os.environ.get("AUTONOMY_TYPE"):
        return
    if not RUNTIME_ENV.is_file():
        return
    for raw in RUNTIME_ENV.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        # The file is emitted with shlex.quote, so shlex parses it back cleanly.
        parsed = shlex.split(value) or [""]
        os.environ.setdefault(key, parsed[0])


def _ns_prefix(robot_id: str | None) -> str:
    return f"/robot{robot_id}" if robot_id else ""


def _write_init_pose(node: Node, map_dir: Path) -> None:
    """Look up map→base_link TF and dump (x, y, yaw) to init_pose.json.

    Only needed for base autonomy — slam_launch.py reads init_pose.json to
    seed slam_toolbox's localization pose. Full autonomy's ARISE SLAM writes
    its own start_pose.txt inside the save handler.
    """
    buffer_ = Buffer()
    TransformListener(buffer_, node)

    deadline = time.monotonic() + TF_TIMEOUT_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer_.can_transform("map", "base_link", rclpy.time.Time()):
            break
    try:
        tf = buffer_.lookup_transform("map", "base_link", rclpy.time.Time())
    except TransformException as e:
        raise SystemExit(f"Could not look up map→base_link within {TF_TIMEOUT_S:.0f}s: {e}")

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    pose = {"x": t.x, "y": t.y, "yaw": yaw}
    (map_dir / "init_pose.json").write_text(json.dumps(pose, indent=4))
    print(f"=> init_pose.json: x={t.x:.3f}, y={t.y:.3f}, yaw={yaw:.3f}")


def _call_service(node: Node, srv_type, name: str, request, label: str):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=10.0):
        raise SystemExit(f"Service {name} not advertised — is the SLAM stack running?")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_S)
    if not future.done():
        raise SystemExit(f"{label}: no response within {SERVICE_TIMEOUT_S:.0f}s.")
    return future.result()


def _wait_for_file(path: Path, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file() and path.stat().st_size > 0:
            return True
        time.sleep(0.1)
    return path.is_file() and path.stat().st_size > 0


def _save_full(node: Node, name: str, map_dir: Path) -> None:
    from arise_slam_mid360_msgs.srv import SaveSlamMap
    from visualization_tools.srv import SaveMap

    slam_prefix = str(map_dir / name)

    resp = _call_service(node, SaveSlamMap, "/save_slam_map",
                         SaveSlamMap.Request(file_path=slam_prefix),
                         label="/save_slam_map")
    print(f"=> /save_slam_map: success={resp.success} ({resp.message})")
    if not resp.success:
        raise SystemExit(1)

    resp = _call_service(node, SaveMap, "/save_map",
                         SaveMap.Request(file_path=slam_prefix,
                                         voxel_size=0.0,
                                         save_vgraph=True),
                         label="/save_map")
    print(f"=> /save_map: success={resp.success} ({resp.message})")
    if not resp.success:
        raise SystemExit(1)
    if resp.vgraph_path:
        vgraph_path = Path(resp.vgraph_path)
        if _wait_for_file(vgraph_path, VGRAPH_TIMEOUT_S):
            print(f"=> FAR vgraph: wrote {vgraph_path}")
        else:
            raise SystemExit(
                f"FAR vgraph was requested at {vgraph_path}, but it did not "
                f"appear within {VGRAPH_TIMEOUT_S:.0f}s. Check that FAR has "
                "initialized a non-empty visibility graph."
            )
    else:
        print("=> FAR vgraph: skipped (graph_decoder was not subscribed)")

    # Full autonomy ships a stock waypoints.csv with the empty_map; reuse it.
    src_wp = INSTALL_MAP_RESOURCE_DIR / "Map-empty_map/waypoints.csv"
    if src_wp.is_file():
        shutil.copy(src_wp, map_dir / "waypoints.csv")

    # Make the new map visible to launch without a rebuild.
    install_map = INSTALL_MAP_RESOURCE_DIR / f"Map-{name}"
    install_map.mkdir(parents=True, exist_ok=True)
    for entry in map_dir.iterdir():
        if entry.is_file():
            shutil.copy2(entry, install_map / entry.name)
    print(f"=> Synced map to {install_map}")


def _save_base(node: Node, name: str, map_dir: Path, robot_id: str | None) -> None:
    """Base autonomy: call slam_toolbox/serialize_map with a host path.

    slam_toolbox writes <prefix>.posegraph + <prefix>.data on whatever
    filesystem the slam_toolbox process sees — which is the host when SLAM
    is launched via `make launch`. If you instead run the stack inside the
    docker container (`make docker_open`), run this script from inside the
    container too so the paths resolve the same way.
    """
    from slam_toolbox.srv import SerializePoseGraph

    service = f"{_ns_prefix(robot_id)}/slam_toolbox/serialize_map"
    target_prefix = str(map_dir / name)

    resp = _call_service(node, SerializePoseGraph, service,
                         SerializePoseGraph.Request(filename=target_prefix),
                         label=service)
    if resp.result != SerializePoseGraph.Response.RESULT_SUCCESS:
        raise SystemExit(
            f"{service}: result={resp.result} (RESULT_FAILED_TO_WRITE_FILE). "
            f"Check that {map_dir} is writable by the slam_toolbox process."
        )
    print(f"=> {service}: wrote {target_prefix}.posegraph + .data")

    # Stock waypoints ship with Map-empty_map.
    src_wp = INSTALL_MAP_RESOURCE_DIR / "Map-empty_map/waypoints.csv"
    if src_wp.is_file():
        shutil.copy(src_wp, map_dir / "waypoints.csv")


def main() -> int:
    _load_runtime_env_if_missing()

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("name", help="Map name (→ src/typego_sdk/resource/Map-<name>/)")
    parser.add_argument(
        "--mode",
        choices=("base", "full"),
        default=os.environ.get("AUTONOMY_TYPE"),
        help="Autonomy mode. Defaults to $AUTONOMY_TYPE from /tmp/typego-runtime.env.",
    )
    parser.add_argument(
        "--robot-id",
        default=os.environ.get("ROBOT_ID") or None,
        help="Robot ID for namespacing (defaults to $ROBOT_ID).",
    )
    args = parser.parse_args()

    if args.mode not in ("base", "full"):
        parser.error("--mode is required (base|full), or set AUTONOMY_TYPE in the environment.")

    map_dir = MAP_RESOURCE_DIR / f"Map-{args.name}"
    map_dir.mkdir(parents=True, exist_ok=True)
    print(f"=> Saving '{args.name}' ({args.mode} autonomy) to {map_dir}")

    rclpy.init()
    try:
        node = Node("save_map_client", namespace=_ns_prefix(args.robot_id))
        try:
            if args.mode == "full":
                _save_full(node, args.name, map_dir)
            else:
                _write_init_pose(node, map_dir)
                _save_base(node, args.name, map_dir, args.robot_id)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

    print(f"=> Map '{args.name}' saved.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
