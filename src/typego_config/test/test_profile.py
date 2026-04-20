"""Profile overlay behaviour."""
import pytest

from typego_config.loader import ConfigError, _deep_merge, load


def test_deep_merge_overrides_scalars():
    base = {'robot': {'type': 'go2', 'name': 'base'}}
    overlay = {'robot': {'name': 'overlay'}}
    merged = _deep_merge(base, overlay)
    assert merged == {'robot': {'type': 'go2', 'name': 'overlay'}}


def test_deep_merge_adds_keys():
    base = {'a': 1}
    overlay = {'b': 2, 'a': 3}
    assert _deep_merge(base, overlay) == {'a': 3, 'b': 2}


def test_deep_merge_replaces_lists():
    base = {'dynamic': ['x', 'y']}
    overlay = {'dynamic': ['z']}
    assert _deep_merge(base, overlay) == {'dynamic': ['z']}


def test_load_with_shipped_go2_indoor_profile():
    cfg = load(profile='go2_indoor')
    assert cfg.robot.type == 'go2'
    assert cfg.autonomy.type == 'base'


def test_load_with_shipped_kami_warehouse_profile():
    cfg = load(profile='kami_warehouse')
    assert cfg.robot.type == 'kami'
    assert cfg.autonomy.type == 'base'


def test_missing_profile_rejected():
    with pytest.raises(ConfigError):
        load(profile='no_such_profile')
