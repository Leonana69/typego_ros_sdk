"""OccupancyGrid → PNG rendering for the /api/map endpoint."""
from __future__ import annotations

import io

import numpy as np
from PIL import Image


def render_occupancy_grid(data: bytes, width: int, height: int) -> bytes:
    """Render a ROS OccupancyGrid payload to a PNG.

    Convention: -1 = unknown (gray), 0 = free (white), 100 = occupied
    (black). Values in between are interpolated linearly. The image is
    flipped vertically so y+ in map frame points up in the PNG, matching
    RViz's convention.
    """
    if width <= 0 or height <= 0:
        raise ValueError('empty grid')
    arr = np.frombuffer(data, dtype=np.int8).astype(np.int16)
    if arr.size != width * height:
        raise ValueError(
            f'grid data size mismatch: expected {width * height}, got {arr.size}'
        )
    grid = arr.reshape((height, width))

    out = np.empty_like(grid, dtype=np.uint8)
    # unknown → mid gray
    unknown = grid < 0
    out[unknown] = 205
    known = ~unknown
    # occupied probability 0..100 → white..black
    out[known] = 255 - (grid[known].clip(0, 100) * 255 // 100).astype(np.uint8)

    img = Image.fromarray(np.flipud(out), mode='L')
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True)
    return buf.getvalue()
