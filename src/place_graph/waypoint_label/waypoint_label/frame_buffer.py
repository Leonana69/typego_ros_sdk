"""Pose-tagged camera-frame buffer and view selection for region labeling.

ROS-free helpers (the node owns the subscriptions / TF / services). A frame is
an RGB image stamped with the robot's world pose so it can later be attributed
to a place via a forward-ray point and sent to a VLM.
"""
from __future__ import annotations

import io
import math
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np
from PIL import Image as PILImage


@dataclass
class Frame:
    img: np.ndarray   # H x W x 3, uint8, RGB
    x: float
    y: float
    yaw: float
    t: float          # monotonic seconds
    blur: float       # variance-of-Laplacian sharpness (higher = sharper)


def image_to_rgb(msg) -> np.ndarray | None:
    """Convert a sensor_msgs/Image to an H x W x 3 uint8 RGB array.

    Handles the common encodings without cv_bridge. Returns None for anything
    unsupported or malformed.
    """
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h <= 0 or w <= 0:
        return None
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    try:
        if enc in ('rgb8', 'bgr8'):
            rows = data.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
            return rows[:, :, ::-1].copy() if enc == 'bgr8' else rows.copy()
        if enc in ('rgba8', 'bgra8'):
            rows = data.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
            rgb = rows[:, :, :3]
            return rgb[:, :, ::-1].copy() if enc == 'bgra8' else rgb.copy()
        if enc == 'mono8':
            g = data.reshape(h, step)[:, :w]
            return np.repeat(g[:, :, None], 3, axis=2)
    except ValueError:
        return None
    return None


def blur_score(rgb: np.ndarray) -> float:
    """Variance of a 4-neighbour Laplacian on greyscale; higher = sharper."""
    g = rgb.astype(np.float32).mean(axis=2)
    lap = (-4.0 * g
           + np.roll(g, 1, 0) + np.roll(g, -1, 0)
           + np.roll(g, 1, 1) + np.roll(g, -1, 1))
    return float(lap[1:-1, 1:-1].var()) if lap.size > 4 else 0.0


def encode_jpeg(rgb: np.ndarray, max_px: int, quality: int = 85) -> bytes:
    """Downscale to `max_px` longest edge and return raw JPEG bytes."""
    im = PILImage.fromarray(np.ascontiguousarray(rgb), mode='RGB')
    w, h = im.size
    scale = max_px / float(max(w, h))
    if scale < 1.0:
        im = im.resize((max(1, int(w * scale)), max(1, int(h * scale))))
    buf = io.BytesIO()
    im.save(buf, format='JPEG', quality=quality)
    return buf.getvalue()


def montage_jpeg(rgbs: list, max_px: int, quality: int = 85) -> bytes | None:
    """Tile several views into one JPEG (longest edge ~`max_px`).

    The gateway accepts exactly one image, so multiple diverse viewpoints are
    composited into a single grid. A lone view is returned full-size; otherwise
    each tile is aspect-preserved and centred on a black cell of an even grid.
    Returns None if there is nothing to encode.
    """
    views = [r for r in rgbs if r is not None and getattr(r, 'size', 0)]
    if not views:
        return None
    if len(views) == 1:
        return encode_jpeg(views[0], max_px, quality)
    cols = math.ceil(math.sqrt(len(views)))
    rows = math.ceil(len(views) / cols)
    cell = max(1, max_px // cols)
    canvas = PILImage.new('RGB', (cols * cell, rows * cell), (0, 0, 0))
    for i, rgb in enumerate(views):
        tile = PILImage.fromarray(np.ascontiguousarray(rgb), mode='RGB')
        tile.thumbnail((cell, cell))   # aspect-preserving, fits within the cell
        r, c = divmod(i, cols)
        ox = c * cell + (cell - tile.size[0]) // 2
        oy = r * cell + (cell - tile.size[1]) // 2
        canvas.paste(tile, (ox, oy))
    buf = io.BytesIO()
    canvas.save(buf, format='JPEG', quality=quality)
    return buf.getvalue()


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def forward_point(x: float, y: float, yaw: float, dist: float) -> tuple:
    """A point `dist` metres ahead of the robot along its heading — the region
    the camera is most likely looking at, rather than where the robot stands."""
    return (x + dist * math.cos(yaw), y + dist * math.sin(yaw))


class FrameRing:
    """Thread-safe bounded ring of pose-tagged frames."""

    def __init__(self, maxlen: int = 400):
        self._dq: deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, f: Frame) -> None:
        with self._lock:
            self._dq.append(f)

    def snapshot(self) -> list:
        with self._lock:
            return list(self._dq)


def select_diverse(frames: list, k: int, min_blur: float,
                   min_pos_sep: float = 0.5, min_yaw_sep: float = 0.5) -> list:
    """Drop blurry frames, then greedily pick up to `k` spread in (x, y, yaw)
    so the VLM sees distinct viewpoints instead of near-duplicates."""
    cand = sorted((f for f in frames if f.blur >= min_blur),
                  key=lambda f: f.blur, reverse=True)
    picked: list = []
    for f in cand:
        too_close = False
        for g in picked:
            dpos = math.hypot(f.x - g.x, f.y - g.y)
            dyaw = abs(math.atan2(math.sin(f.yaw - g.yaw),
                                  math.cos(f.yaw - g.yaw)))
            if dpos < min_pos_sep and dyaw < min_yaw_sep:
                too_close = True
                break
        if not too_close:
            picked.append(f)
        if len(picked) >= k:
            break
    if not picked and cand:          # everything was similar: take the sharpest
        picked = cand[:k]
    return picked
