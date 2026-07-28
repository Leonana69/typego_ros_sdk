import yaml

from typego_config.loader import load
from typego_config.render import render_config_node_params, write_render_bundle


def test_config_node_params_have_flat_keys():
    cfg = load()
    text = render_config_node_params(cfg)
    doc = yaml.safe_load(text)
    assert '/**' in doc
    params = doc['/**']['ros__parameters']
    assert 'network.ros_domain_id' in params
    assert 'profiles.nav2_params_file' in params


def test_write_render_bundle_creates_files(tmp_path):
    cfg = load()
    files = write_render_bundle(cfg, tmp_path / 'out')
    assert files
    for f in files:
        assert f.is_file()
        assert f.read_text().strip() != ''
