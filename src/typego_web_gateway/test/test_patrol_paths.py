"""Verify waypoint CSV resolution honours the documented search order."""
import os

import pytest

from typego_web_gateway.patrol_controller import (
    _resolve_waypoints_path,
    load_waypoints,
)


def test_override_env_takes_precedence(tmp_path, monkeypatch):
    d = tmp_path / 'Map-demo'
    d.mkdir()
    (d / 'waypoints.csv').write_text('1 1.0 2.0 start\n2 3.5 4.0 end\n')
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    path = _resolve_waypoints_path('demo')
    assert path == str(d / 'waypoints.csv')


def test_load_waypoints_parses_csv(tmp_path, monkeypatch):
    d = tmp_path / 'Map-demo'
    d.mkdir()
    (d / 'waypoints.csv').write_text('1 1.0 2.0 start\n2 3.5 4.0 end\n')
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    wp = load_waypoints('demo')
    assert wp == {1: (1.0, 2.0), 2: (3.5, 4.0)}


def test_missing_map_raises(tmp_path, monkeypatch):
    monkeypatch.setenv('TYPEGO_MAPS_DIR', str(tmp_path))
    with pytest.raises(FileNotFoundError):
        load_waypoints('nope')
