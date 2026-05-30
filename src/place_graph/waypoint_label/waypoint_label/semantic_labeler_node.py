"""Opportunistic VLM region labeler.

Buffers pose-tagged camera frames, buckets them to place_ids via the C++
ResolvePlaceAtPoint service (a forward-ray point ~2 m ahead, since Python has no
access to the place-assignment grid), and once a region has enough diverse views
asks a VLM for a structured label and writes it back via SetPlaceLabel. The
place_graph node owns the label and propagates it across map refreshes.

The slow VLM/service work runs on a background thread so the rclpy executor
keeps spinning (and completing service futures) on the main thread.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:  # older tf2_ros layouts
    from tf2_ros.exceptions import TransformException

from typego_interface.msg import PlaceGraph
from typego_interface.srv import ResolvePlaceAtPoint, SetPlaceLabel

from waypoint_label.frame_buffer import (Frame, FrameRing, blur_score,
                                           encode_jpeg_b64, forward_point,
                                           image_to_rgb, select_diverse,
                                           yaw_from_quat)
from waypoint_label.vlm_client import VlmClient

# Only these place kinds get a room label; connectors/frontier are skipped.
ROOM_KINDS = {'room', 'corridor', 'open_area'}
# A place is a labeling candidate while its label is geometry-only or a prior
# low-confidence VLM pass that could still improve.
RELABELABLE_SOURCES = {'', 'map', 'vlm_lowconf'}


class SemanticLabeler(Node):
    def __init__(self):
        super().__init__('semantic_labeler')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.camera_topic = p('camera_topic', '/camera/color/image_raw')
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        base_url = p('vlm_base_url', 'http://localhost:8000/v1')
        model = p('vlm_model', 'Qwen2.5-VL-7B-Instruct')
        api_key = p('vlm_api_key', 'EMPTY')
        structured = bool(p('vlm_structured', True))
        self.max_image_px = int(p('max_image_px', 1024))
        self.frame_hz = float(p('frame_hz', 2.0))
        self.min_frames = int(p('min_frames', 3))
        self.max_views = int(p('max_views', 4))
        self.forward_dist_m = float(p('forward_dist_m', 2.0))
        self.conf_threshold = float(p('conf_threshold', 0.4))
        self.label_period_s = float(p('label_period_s', 5.0))
        self.min_blur = float(p('min_blur', 40.0))
        self.cooldown_s = float(p('cooldown_s', 60.0))

        self.vlm = VlmClient(base_url, model, api_key, structured,
                             logger=self.get_logger())
        self.ring = FrameRing(maxlen=400)
        self._graph_lock = threading.Lock()
        self._places: dict = {}
        self._attempted: dict = {}     # place_id -> last attempt (monotonic)
        self._last_frame_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(PlaceGraph, 'place_graph', self._on_graph,
                                 latched)
        self.create_subscription(Image, self.camera_topic, self._on_image, 10)

        self.resolve_cli = self.create_client(
            ResolvePlaceAtPoint, 'typego/resolve_place_at_point')
        self.label_cli = self.create_client(
            SetPlaceLabel, 'typego/set_place_label')

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._label_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'semantic_labeler up: camera={self.camera_topic} '
            f'vlm={base_url} model={model}')

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

    # ----- subscriptions (run on the executor thread) -----
    def _on_graph(self, msg):
        places = {}
        for pl in msg.places:
            places[str(pl.place_id)] = {
                'place_id': str(pl.place_id),
                'kind': str(pl.kind),
                'source': str(pl.source),
                'label_locked': bool(pl.label_locked),
                'provisional': bool(pl.provisional),
                'area_m2': float(pl.area_m2),
            }
        with self._graph_lock:
            self._places = places

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self._last_frame_t < 1.0 / max(self.frame_hz, 0.1):
            return
        pose = self._lookup_pose()
        if pose is None:
            return
        rgb = image_to_rgb(msg)
        if rgb is None:
            return
        self._last_frame_t = now
        x, y, yaw = pose
        self.ring.add(Frame(img=rgb, x=x, y=y, yaw=yaw, t=now,
                            blur=blur_score(rgb)))

    def _lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    # ----- labeling worker (background thread) -----
    def _label_loop(self):
        while not self._stop.is_set():
            self._stop.wait(self.label_period_s)
            if self._stop.is_set():
                break
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001 - never kill the worker
                self.get_logger().warning(f'label tick error: {e}')

    def _tick(self):
        frames = self.ring.snapshot()
        if len(frames) < self.min_frames:
            return
        with self._graph_lock:
            places = dict(self._places)
        cand = [pid for pid, pl in places.items()
                if pl['kind'] in ROOM_KINDS
                and not pl['label_locked']
                and not pl['provisional']
                and pl['source'] in RELABELABLE_SOURCES]
        if not cand:
            return
        buckets = self._bucket(frames)
        now = time.monotonic()
        for pid in cand:
            views = buckets.get(pid, [])
            if len(views) < self.min_frames:
                continue
            if now - self._attempted.get(pid, -1e9) < self.cooldown_s:
                continue
            self._attempted[pid] = now
            self._label_place(pid, places[pid], views)
            return  # at most one VLM call per tick

    def _bucket(self, frames):
        """Group frames by the place their forward-ray point falls in, via a
        single ResolvePlaceAtPoint call."""
        req = ResolvePlaceAtPoint.Request()
        req.xs = []
        req.ys = []
        for f in frames:
            px, py = forward_point(f.x, f.y, f.yaw, self.forward_dist_m)
            req.xs.append(px)
            req.ys.append(py)
        res = self._call(self.resolve_cli, req, timeout=5.0)
        buckets: dict = {}
        if res is None:
            return buckets
        for f, pid in zip(frames, res.place_ids):
            if pid:
                buckets.setdefault(pid, []).append(f)
        return buckets

    def _label_place(self, pid, place, frames):
        views = select_diverse(frames, self.max_views, self.min_blur)
        if not views:
            return
        images = [encode_jpeg_b64(f.img, self.max_image_px) for f in views]
        hint = (f"Map geometry: kind={place['kind']}, "
                f"area={place['area_m2']:.0f} m^2.")
        self.get_logger().info(f'labeling {pid} from {len(images)} views...')
        result = self.vlm.label_region(images, hint)
        if not result:
            self.get_logger().warning(f'no VLM result for {pid}')
            return
        conf = float(result.get('confidence', 0.0) or 0.0)
        req = SetPlaceLabel.Request()
        req.place_id = pid
        req.semantic_label = str(result.get('semantic_label', 'unknown'))
        req.instance_label = str(result.get('instance_label', ''))
        req.objects = [str(o) for o in result.get('objects', []) if str(o)]
        req.affordances = [str(a) for a in result.get('affordances', [])
                           if str(a)]
        req.summary = str(result.get('summary', ''))
        req.confidence = conf
        req.source = 'vlm' if conf >= self.conf_threshold else 'vlm_lowconf'
        req.lock = False
        res = self._call(self.label_cli, req, timeout=8.0)
        if res is None:
            self.get_logger().warning(f'SetPlaceLabel call failed for {pid}')
        elif res.success:
            self.get_logger().info(
                f'labeled {pid} = {req.semantic_label} '
                f'({req.source}, conf {conf:.2f})')
        else:
            self.get_logger().warning(
                f'SetPlaceLabel rejected {pid}: {res.message}')

    def _call(self, client, req, timeout):
        """Call a service from the worker thread, blocking on the future that
        the main-thread executor completes."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(f'service {client.srv_name} unavailable')
            return None
        fut = client.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self.get_logger().warning(f'service {client.srv_name} timed out')
            return None
        return fut.result()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLabeler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
