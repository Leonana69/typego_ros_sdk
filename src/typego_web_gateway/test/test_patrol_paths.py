"""Verify waypoint JSON resolution honours the documented search order."""
import os

import pytest

from typego_web_gateway.patrol_controller import (
    _resolve_waypoints_path,
    load_waypoints,
)


def test_override_env_takes_precedence(tmp_path, monkeypatch):
    d = tmp_path / 'Map-demo'
    d.mkdir()
    (d / 'waypoints.json').write_text(
        '{"waypoints":[{"id":1,"x":1.0,"y":2.0},{"id":2,"x":3.5,"y":4.0}]}'
    )
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    path = _resolve_waypoints_path('demo')
    assert path == str(d / 'waypoints.json')


def test_load_waypoints_parses_json(tmp_path, monkeypatch):
    d = tmp_path / 'Map-demo'
    d.mkdir()
    (d / 'waypoints.json').write_text(
        '{"waypoints":[{"id":1,"x":1.0,"y":2.0},{"id":2,"x":3.5,"y":4.0}]}'
    )
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    wp = load_waypoints('demo')
    assert wp == {1: (1.0, 2.0), 2: (3.5, 4.0)}


def test_missing_map_raises(tmp_path, monkeypatch):
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    with pytest.raises(FileNotFoundError):
        load_waypoints('nope')
