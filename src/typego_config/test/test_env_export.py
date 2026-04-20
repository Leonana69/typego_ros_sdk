"""Snapshot tests for env.py — the Docker / Makefile contract."""
import shlex

from typego_config.env import _env_pairs, render_env_lines, render_env_text
from typego_config.loader import load


def test_default_env_pairs_cover_dotenv_vars():
    cfg = load()
    pairs = _env_pairs(cfg)
    required = {
        'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTDDS_BUILTIN_TRANSPORTS',
        'ROBOT_ID', 'ROBOT_TYPE', 'ROBOT_IP',
        'EDGE_SERVICE_IP', 'EDGE_SERVICE_PORT',
        'GSTREAMER_INTERFACE', 'GSTREAMER_RGB_PORT',
        'GSTREAMER_DEPTH_PORT', 'GSTREAMER_AUDIO_PORT',
        'AUTONOMY_TYPE',
    }
    assert not (required - pairs.keys())


def test_render_env_lines_shipped_values():
    cfg = load()
    text = render_env_text(cfg)
    assert 'EDGE_SERVICE_PORT=50049' in text
    assert 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' in text
    for line in text.splitlines():
        if not line:
            continue
        key, sep, _ = line.partition('=')
        assert sep == '=' and key.isupper()


def test_quote_special_characters():
    from typego_config.schema import RobotConfig
    cfg = RobotConfig.model_validate({
        'network': {
            'robot_ip': '192.168.1.1',
            'gstreamer': {'interface': 'eno 1'},
        },
    })
    lines = render_env_lines(cfg)
    ip_line = next(l for l in lines if l.startswith('ROBOT_IP='))
    assert shlex.split(ip_line.split('=', 1)[1])[0] == '192.168.1.1'
    iface_line = next(l for l in lines if l.startswith('GSTREAMER_INTERFACE='))
    value = iface_line.split('=', 1)[1]
    assert shlex.split(value) == ['eno 1']


def test_empty_values_emit_empty():
    from typego_config.schema import RobotConfig
    cfg = RobotConfig()
    lines = render_env_lines(cfg)
    assert any(line == 'ROBOT_ID=' for line in lines)
