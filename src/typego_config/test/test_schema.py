"""Pydantic schema contract tests for robot.yaml."""
import pytest
import yaml
from pydantic import ValidationError

from typego_config.loader import flatten, load_raw, to_json_dict
from typego_config.schema import RobotConfig


def test_defaults_construct_cleanly():
    cfg = RobotConfig()
    assert cfg.robot.type == 'go2'
    assert cfg.autonomy.type == 'base'
    assert cfg.network.ros_domain_id == 0
    assert cfg.network.rmw.startswith('rmw_')
    assert cfg.web_gateway.port == 8080


def test_shipped_robot_yaml_loads(tmp_path):
    import typego_config
    pkg_dir = __import__('pathlib').Path(typego_config.__file__).parent.parent
    yml = (pkg_dir / 'config' / 'robot.yaml').read_text()
    data = yaml.safe_load(yml)
    cfg = RobotConfig.model_validate(data)
    assert cfg.robot.type in ('go2', 'kami')
    assert 1 <= cfg.network.edge_service.port <= 65535
    assert 'web_gateway.bag_retain' in cfg.dynamic


def test_invalid_domain_rejected():
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({'network': {'ros_domain_id': 300}})


def test_invalid_autonomy_rejected():
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({'autonomy': {'type': 'quantum'}})


def test_invalid_robot_type_rejected():
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({'robot': {'type': 'spot'}})


def test_unknown_top_level_key_rejected():
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({'mystery': 42})


def test_flatten_produces_dotted_keys():
    cfg = RobotConfig()
    flat = flatten(cfg)
    assert 'network.ros_domain_id' in flat
    assert 'network.edge_service.port' in flat
    assert 'profiles.nav2_params_file' in flat
    assert not any(k.startswith('dynamic') for k in flat)


def test_flatten_values_are_primitives():
    flat = flatten(RobotConfig())
    for key, value in flat.items():
        assert isinstance(value, (int, float, str, bool)), (
            f'{key}={value!r} must be primitive for rcl_interfaces'
        )


def test_to_json_dict_is_serializable():
    import json
    cfg = RobotConfig()
    blob = json.dumps(to_json_dict(cfg))
    assert '"robot"' in blob


def test_load_raw_rejects_top_level_list(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('- 1\n- 2\n')
    with pytest.raises(Exception) as excinfo:
        load_raw(path)
    assert 'mapping' in str(excinfo.value).lower()
