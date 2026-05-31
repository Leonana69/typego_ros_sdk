"""Thin async-friendly facade over rclpy.

The rclpy node is spun on its own thread by a MultiThreadedExecutor. FastAPI
handlers live on a separate asyncio loop and reach ROS through the methods
on RosBridge. Blocking rclpy calls are wrapped with asyncio.to_thread so
they do not stall the event loop.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer, TransformListener
try:
    from tf2_ros import TransformException
except ImportError:  # older tf2_ros layouts keep it in the submodule
    from tf2_ros.exceptions import TransformException

try:
    from typego_interface.msg import PlaceGraph, WayPointArray
    from typego_interface.srv import SetSpeed
    _TYPEGO_INTERFACE = True
except ImportError:
    _TYPEGO_INTERFACE = False

from std_srvs.srv import Trigger


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    stamp: float


@dataclass
class MapSnapshot:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    stamp: float
    data: bytes  # raw int8 grid, row-major (width * height bytes)


class RosBridge(Node):
    def __init__(self, robot_namespace: str = ''):
        super().__init__('typego_web_gateway')
        self._lock = threading.RLock()
        self._ns = robot_namespace.strip('/')
        self._prefix = f'/{self._ns}' if self._ns else ''

        self._map: Optional[MapSnapshot] = None
        self._waypoints: List[Dict] = []
        self._place_graph: Dict = {'map_version': '', 'places': []}
        self._pose: Optional[Pose2D] = None
        self._events: Deque[Dict] = deque(maxlen=500)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, f'{self._prefix}/map' if self._prefix else '/map',
            self._on_map, latched,
        )
        self._rosout_sub = self.create_subscription(
            Log, '/rosout', self._on_rosout, 50,
        )

        if _TYPEGO_INTERFACE:
            self._waypoints_sub = self.create_subscription(
                WayPointArray, 'waypoints', self._on_waypoints, 10,
            )
            self._place_graph_sub = self.create_subscription(
                PlaceGraph, 'place_graph', self._on_place_graph, latched,
            )
            self._speed_client = self.create_client(
                SetSpeed, 'typego/set_speed',
            )
        else:
            self._waypoints_sub = None
            self._place_graph_sub = None
            self._speed_client = None

        # Optional: typego_config runs separately and may not be up. The
        # client is created eagerly but calls short-timeout so a missing
        # config service produces a clean None response.
        self._config_client = self.create_client(
            Trigger, 'typego_config/get_config',
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        action_name = f'{self._prefix}/navigate_to_pose' if self._prefix else '/navigate_to_pose'
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._active_goal_handle = None
        self._action_name = action_name

        self._pose_frame = f'{self._ns}/base_link' if self._ns else 'base_link'
        self._map_frame = f'{self._ns}/map' if self._ns else 'map'

        self._pose_timer = self.create_timer(0.1, self._update_pose)

    # ── subscriber callbacks ────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid) -> None:
        snapshot = MapSnapshot(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            origin_yaw=_quat_to_yaw(
                msg.info.origin.orientation.z,
                msg.info.origin.orientation.w,
            ),
            frame_id=msg.header.frame_id,
            stamp=time.time(),
            data=bytes(b & 0xFF for b in msg.data),
        )
        with self._lock:
            self._map = snapshot

    def _on_waypoints(self, msg) -> None:
        with self._lock:
            self._waypoints = [
                {'id': int(wp.id), 'x': float(wp.x), 'y': float(wp.y),
                 'label': str(wp.label),
                 'yaw': float(getattr(wp, 'yaw', 0.0)),
                 'semantic_context': str(getattr(wp, 'semantic_context', '')),
                 'confidence': float(getattr(wp, 'confidence', 0.0)),
                 'source': str(getattr(wp, 'source', '')),
                 'clearance_m': float(getattr(wp, 'clearance_m', 0.0)),
                 'generation_reason': str(getattr(wp, 'generation_reason', '')),
                 'place_id': str(getattr(wp, 'place_id', '')),
                 'waypoint_role': str(getattr(wp, 'waypoint_role', ''))}
                for wp in msg.waypoints
            ]

    def _on_place_graph(self, msg) -> None:
        places = []
        for p in msg.places:
            places.append({
                'place_id': str(p.place_id),
                'kind': str(p.kind),
                'semantic_label': str(getattr(p, 'semantic_label', '')),
                'instance_label': str(getattr(p, 'instance_label', '')),
                'operator_label': str(getattr(p, 'operator_label', '')),
                'label_locked': bool(getattr(p, 'label_locked', False)),
                'confidence': float(getattr(p, 'confidence', 0.0)),
                'source': str(getattr(p, 'source', '')),
                'objects': [str(o) for o in getattr(p, 'objects', [])],
                'affordances': [str(a) for a in getattr(p, 'affordances', [])],
                'summary': str(getattr(p, 'summary', '')),
                'centroid': {'x': float(p.centroid_x), 'y': float(p.centroid_y)},
                'peak': {'x': float(p.peak_x), 'y': float(p.peak_y)},
                'area_m2': float(p.area_m2),
                'clearance_m': float(p.clearance_m),
                'frontier_ratio': float(p.frontier_ratio),
                'elongation': float(getattr(p, 'elongation', 1.0)),
                'extent_long_m': float(getattr(p, 'extent_long_m', 0.0)),
                'extent_short_m': float(getattr(p, 'extent_short_m', 0.0)),
                'provisional': bool(p.provisional),
                'stale': bool(p.stale),
                'approach_yaw': (float(p.approach_yaw)
                                 if p.has_approach_yaw else None),
                'width_m': float(p.width_m) if p.has_width_m else None,
                'adjacent_place_ids': [str(a) for a in p.adjacent_place_ids],
                'member_waypoint_ids': [int(i) for i in p.member_waypoint_ids],
            })
        with self._lock:
            self._place_graph = {
                'map_version': str(msg.map_version),
                'places': places,
            }

    def _on_rosout(self, msg: Log) -> None:
        # rcl_interfaces/Log defines level as `byte`. In rclpy Humble the
        # deserialized field comes back as int while the class-level
        # constants (Log.WARN etc.) are bytes — a direct `<` comparison
        # raises TypeError. Compare against the numeric literal instead.
        level = int.from_bytes(msg.level, 'little') if isinstance(msg.level, (bytes, bytearray)) else int(msg.level)
        if level < 30:  # below WARN
            return
        with self._lock:
            self._events.append({
                'type': 'rosout',
                'level': level,
                'name': msg.name,
                'msg': msg.msg,
                'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            })

    def _update_pose(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._pose_frame,
                RclpyTime(), timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        pose = Pose2D(
            x=float(t.x), y=float(t.y),
            yaw=_quat_to_yaw(q.z, q.w),
            stamp=time.time(),
        )
        with self._lock:
            self._pose = pose

    # ── readers ──────────────────────────────────────────────────────────
    def get_pose(self) -> Optional[Pose2D]:
        with self._lock:
            return self._pose

    def get_map(self) -> Optional[MapSnapshot]:
        with self._lock:
            return self._map

    def get_waypoints(self) -> List[Dict]:
        with self._lock:
            return list(self._waypoints)

    def get_place_graph(self) -> Dict:
        with self._lock:
            return {
                'map_version': self._place_graph.get('map_version', ''),
                'places': list(self._place_graph.get('places', [])),
            }

    def get_events(self, limit: int = 100) -> List[Dict]:
        with self._lock:
            return list(self._events)[-limit:]

    def record_event(self, event: Dict) -> None:
        event.setdefault('stamp', time.time())
        with self._lock:
            self._events.append(event)

    def status(self) -> Dict:
        with self._lock:
            has_map = self._map is not None
            has_pose = self._pose is not None
            waypoints = len(self._waypoints)
        return {
            'action_name': self._action_name,
            'robot_namespace': self._ns,
            'has_map': has_map,
            'has_pose': has_pose,
            'waypoint_count': waypoints,
            'nav_server_ready': self._nav_client.server_is_ready(),
            'speed_service_ready': (
                self._speed_client.service_is_ready()
                if self._speed_client is not None else False
            ),
        }

    # ── commands (blocking; call via asyncio.to_thread) ──────────────────
    def send_goal(self, x: float, y: float, yaw_deg: float = 0.0,
                  timeout_s: float = 2.0) -> Tuple[bool, str]:
        """Dispatch a NavigateToPose goal. Returns (accepted, message).

        The action runs asynchronously on the ROS executor; completion is
        surfaced via the events ring buffer.
        """
        if not self._nav_client.wait_for_server(timeout_sec=timeout_s):
            return False, f'action server not ready ({self._action_name})'

        yaw = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal)
        # Wait for accept/reject using an event set by the done callback —
        # cannot spin_until_future_complete here, the executor thread does it.
        event = threading.Event()
        send_future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s + 1.0):
            return False, 'goal accept timeout'

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.record_event({'type': 'nav', 'status': 'rejected',
                               'x': x, 'y': y, 'yaw_deg': yaw_deg})
            return False, 'goal rejected'

        with self._lock:
            self._active_goal_handle = handle
        self.record_event({'type': 'nav', 'status': 'accepted',
                           'x': x, 'y': y, 'yaw_deg': yaw_deg})

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)
        return True, 'accepted'

    def _on_goal_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:  # pragma: no cover
            self.record_event({'type': 'nav', 'status': 'error', 'msg': str(exc)})
            return
        mapping = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_ABORTED: 'aborted',
            GoalStatus.STATUS_CANCELED: 'canceled',
        }
        self.record_event({
            'type': 'nav',
            'status': mapping.get(status, f'status_{status}'),
        })
        with self._lock:
            self._active_goal_handle = None

    def cancel_goal(self, timeout_s: float = 2.0) -> Tuple[bool, str]:
        with self._lock:
            handle = self._active_goal_handle
        if handle is None:
            return False, 'no active goal'
        future = handle.cancel_goal_async()
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s):
            return False, 'cancel timeout'
        self.record_event({'type': 'nav', 'status': 'cancel_requested'})
        return True, 'cancel requested'

    def wait_goal_result(self, timeout_s: float = 600.0) -> Optional[int]:
        """Block until the active goal finishes. Returns GoalStatus or None."""
        with self._lock:
            handle = self._active_goal_handle
        if handle is None:
            return None
        fut = handle.get_result_async()
        event = threading.Event()
        fut.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s):
            return None
        try:
            return fut.result().status
        except Exception:
            return None

    def set_speed(self, max_linear: float = -1.0, max_autonomy: float = -1.0,
                  max_reverse: float = -1.0, max_angular: float = -1.0,
                  timeout_s: float = 3.0) -> Tuple[bool, str]:
        if self._speed_client is None:
            return False, 'typego_interface not available'
        if not self._speed_client.wait_for_service(timeout_sec=timeout_s):
            return False, 'set_speed service not ready'
        req = SetSpeed.Request()
        req.max_speed = float(max_linear)
        req.max_autonomy_speed = float(max_autonomy)
        req.max_reverse_speed = float(max_reverse)
        req.max_angular_speed = float(max_angular)
        fut = self._speed_client.call_async(req)
        event = threading.Event()
        fut.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s + 1.0):
            return False, 'set_speed call timeout'
        resp = fut.result()
        if resp is None:
            return False, 'empty response'
        return bool(resp.success), str(resp.message)

    def get_robot_config(self, timeout_s: float = 1.0) -> Optional[Dict]:
        """Call typego_config/get_config; return None if service is down."""
        import json as _json

        if not self._config_client.wait_for_service(timeout_sec=timeout_s):
            return None
        fut = self._config_client.call_async(Trigger.Request())
        event = threading.Event()
        fut.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_s + 0.5):
            return None
        resp = fut.result()
        if resp is None or not resp.success:
            return None
        try:
            return _json.loads(resp.message)
        except Exception:
            return None


def _quat_to_yaw(qz: float, qw: float) -> float:
    """Flat-ground quaternion → yaw (radians)."""
    return 2.0 * math.atan2(qz, qw)
