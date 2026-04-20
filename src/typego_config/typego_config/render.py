"""Emit ROS 2 parameter files from robot.yaml."""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import yaml

from .loader import flatten
from .schema import RobotConfig


CONFIG_NODE_NAME = 'typego_config'


def render_config_node_params(cfg: RobotConfig,
                              node_name: str = CONFIG_NODE_NAME) -> str:
    flat = flatten(cfg)
    params: Dict[str, object] = dict(flat)
    params['dynamic'] = list(cfg.dynamic)
    doc = {'/**': {'ros__parameters': params}}
    del node_name
    return yaml.safe_dump(doc, sort_keys=True, default_flow_style=False)


def write_render_bundle(cfg: RobotConfig, out_dir: Path) -> List[Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    files: List[Path] = []
    node_params = out_dir / 'typego_config.params.yaml'
    node_params.write_text(render_config_node_params(cfg))
    files.append(node_params)
    return files
