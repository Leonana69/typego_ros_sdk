"""Pydantic schema for robot.yaml — the single source of truth.

Every field here is one of:
  * an identity / network knob that used to live in docker/.env, or
  * a top-level pointer to a per-tool YAML profile (Nav2, SLAM, planners).

Deliberately does NOT attempt to swallow Nav2 / SLAM / planner tuning
YAMLs — those keep their tool-native formats and are referenced by
`profiles.*`.
"""
from __future__ import annotations

from typing import List, Literal

from pydantic import BaseModel, ConfigDict, Field

RobotType = Literal['go2', 'kami']
AutonomyType = Literal['base', 'full']
Rmw = Literal[
    'rmw_cyclonedds_cpp',
    'rmw_fastrtps_cpp',
    'rmw_connextdds',
]


class RobotIdentity(BaseModel):
    model_config = ConfigDict(extra='forbid')

    id: str = Field(
        default='',
        description='Namespace suffix; "" = no namespace. "1" becomes /robot1.',
    )
    type: RobotType = Field(
        default='go2',
        description='Selects which <type>_sdk package launches.',
    )
    name: str = Field(
        default='robot-alpha',
        description='Human-readable label shown in the web UI.',
    )


class AutonomyConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    type: AutonomyType = Field(
        default='base',
        description='"base" = SLAM Toolbox + Nav2; "full" = ARISE + TARE/FAR.',
    )


class EdgeServiceConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    ip: str = Field(default='127.0.0.1')
    port: int = Field(default=50049, ge=1, le=65535)


class GstreamerConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    interface: str = Field(default='wlo1')
    rgb_port: int = Field(default=1722, ge=1, le=65535)
    depth_port: int = Field(default=1723, ge=1, le=65535)
    audio_port: int = Field(default=1724, ge=1, le=65535)


class NetworkConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    ros_domain_id: int = Field(
        default=0, ge=0, le=232,
        description='ROS 2 DDS domain. 0..232 per ROS 2 spec.',
    )
    rmw: Rmw = Field(
        default='rmw_cyclonedds_cpp',
        description='DDS implementation set as RMW_IMPLEMENTATION.',
    )
    fastdds_builtin_transports: str = Field(default='UDPv4')
    robot_ip: str = Field(
        default='',
        description='Robot LAN IP. Consumed by go2_sdk / kami_sdk C++ clients.',
    )
    edge_service: EdgeServiceConfig = Field(default_factory=EdgeServiceConfig)
    gstreamer: GstreamerConfig = Field(default_factory=GstreamerConfig)


class WebGatewayConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    enabled: bool = True
    host: str = Field(default='0.0.0.0')
    port: int = Field(default=8080, ge=1, le=65535)
    bag_dir: str = Field(default='~/.typego/bags')
    bag_chunk_seconds: int = Field(default=600, ge=30, le=24 * 3600)
    bag_retain: int = Field(default=6, ge=1, le=1000)


class MapConfig(BaseModel):
    model_config = ConfigDict(extra='forbid')

    slam_map_name: str = Field(
        default='empty_map',
        description='Selects resource/Map-<name>/ under typego_sdk share.',
    )
    init_pose_from_file: bool = Field(
        default=True,
        description='If true, seed AMCL/SLAM pose from Map-<name>/init_pose.json.',
    )


class ProfilesConfig(BaseModel):
    """Filenames of per-tool YAMLs, resolved against their ament shares."""
    model_config = ConfigDict(extra='forbid')

    nav2_params_file: str = Field(
        default='nav2_params.yaml',
        description='Under typego_sdk/config/.',
    )
    slam_params_file: str = Field(
        default='slam.yaml',
        description='Under typego_sdk/config/.',
    )
    local_planner_profile: str = Field(
        default='dog',
        description='Picks local_planner/config/<name>.yaml.',
    )
    far_planner_profile: str = Field(
        default='outdoor',
        description='Picks far_planner/config/<name>.yaml.',
    )
    tare_planner_profile: str = Field(
        default='indoor_small',
        description='Picks tare_planner/config/<name>.yaml.',
    )


class RobotConfig(BaseModel):
    """Root model validated against robot.yaml."""
    model_config = ConfigDict(extra='forbid')

    robot: RobotIdentity = Field(default_factory=RobotIdentity)
    autonomy: AutonomyConfig = Field(default_factory=AutonomyConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    web_gateway: WebGatewayConfig = Field(default_factory=WebGatewayConfig)
    map: MapConfig = Field(default_factory=MapConfig)
    profiles: ProfilesConfig = Field(default_factory=ProfilesConfig)

    dynamic: List[str] = Field(
        default_factory=lambda: [
            'web_gateway.bag_retain',
            'web_gateway.bag_chunk_seconds',
            'map.slam_map_name',
        ],
        description=(
            'Dotted keys that the config service may mutate at runtime. '
            'All other fields are read-only after launch.'
        ),
    )
