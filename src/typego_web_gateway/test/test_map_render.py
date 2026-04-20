"""Sanity tests for the OccupancyGrid -> PNG renderer."""
import io

from PIL import Image

from typego_web_gateway.map_render import render_occupancy_grid


def test_render_basic_grid():
    # 4x3 grid with one unknown cell, one occupied, rest free.
    data = bytes([255 if v == -1 else v for v in  # -1 becomes 0xFF in int8
                  (-1, 0, 100, 0,
                   0, 50, 0, 100,
                   -1, 0, 0, 0)])
    png = render_occupancy_grid(data, width=4, height=3)
    assert png.startswith(b'\x89PNG\r\n\x1a\n')
    img = Image.open(io.BytesIO(png))
    assert img.size == (4, 3)
    assert img.mode == 'L'


def test_render_rejects_size_mismatch():
    import pytest

    with pytest.raises(ValueError):
        render_occupancy_grid(b'\x00' * 10, width=4, height=3)
