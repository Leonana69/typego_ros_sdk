"""Render the env-var subset of robot.yaml for shell/Docker consumers."""
from __future__ import annotations

import shlex
from typing import Dict, List

from .schema import RobotConfig


def _env_pairs(cfg: RobotConfig) -> Dict[str, str]:
    return {
        'ROS_DOMAIN_ID': str(cfg.network.ros_domain_id),
        'RMW_IMPLEMENTATION': cfg.network.rmw,
        'FASTDDS_BUILTIN_TRANSPORTS': cfg.network.fastdds_builtin_transports,
        'ROBOT_ID': cfg.robot.id,
        'ROBOT_TYPE': cfg.robot.type,
        'ROBOT_IP': cfg.network.robot_ip,
        'EDGE_SERVICE_IP': cfg.network.edge_service.ip,
        'EDGE_SERVICE_PORT': str(cfg.network.edge_service.port),
        'GSTREAMER_INTERFACE': cfg.network.gstreamer.interface,
        'GSTREAMER_RGB_PORT': str(cfg.network.gstreamer.rgb_port),
        'GSTREAMER_DEPTH_PORT': str(cfg.network.gstreamer.depth_port),
        'GSTREAMER_AUDIO_PORT': str(cfg.network.gstreamer.audio_port),
        'AUTONOMY_TYPE': cfg.autonomy.type,
    }


def render_env_lines(cfg: RobotConfig) -> List[str]:
    out: List[str] = []
    for key, val in _env_pairs(cfg).items():
        if val == '':
            out.append(f'{key}=')
            continue
        out.append(f'{key}={shlex.quote(val)}')
    return out


def render_env_text(cfg: RobotConfig) -> str:
    return '\n'.join(render_env_lines(cfg)) + '\n'
