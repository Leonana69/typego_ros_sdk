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

from pydantic import BaseModel, ConfigDict, Field, model_validator

RobotType = Literal['go2', 'kami']
# "2d" = SLAM Toolbox + Nav2 (2D occupancy grid).
# "3d" = LIO + terrain analysis + local planner (3D point cloud).
# Renamed from "base"/"full", which named nothing; see _MIGRATED_AUTONOMY_TYPES.
AutonomyType = Literal['2d', '3d']

#: Retired spellings, mapped to their replacement so the error can say what
#: to change instead of just listing the permitted values.
_MIGRATED_AUTONOMY_TYPES = {'base': '2d', 'full': '3d'}
SlamBackend = Literal['arise', 'lightning']
RoutePlannerBackend = Literal['far', 'pcd_grid']
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
        default='2d',
        description='"2d" = SLAM Toolbox + Nav2; '
                    '"3d" = LIO + terrain analysis + local planner.',
    )
    slam_backend: SlamBackend = Field(
        default='arise',
        description=(
            'LIO backend when type=3d. "arise" = arise_slam_mid360 (default). '
            '"lightning" = vendored lightning-lm. Ignored when type=2d.'
        ),
    )

    @model_validator(mode='before')
    @classmethod
    def _reject_legacy_type(cls, data):
        """Turn the old spellings into a migration instruction.

        Bare Literal validation would only say "Input should be '2d' or '3d'",
        which does not tell someone holding a pre-rename robot.yaml what to do.
        """
        if isinstance(data, dict):
            old = data.get('type')
            new = _MIGRATED_AUTONOMY_TYPES.get(old)
            if new is not None:
                raise ValueError(
                    f"autonomy.type: {old!r} was renamed to {new!r}. "
                    f"Change 'type: {old}' to 'type: {new}' in robot.yaml "
                    f"(and in any profile under config/profiles/)."
                )
        return data


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
    camera_topic: str = Field(
        default='camera/color/image_raw',
        description='Image topic the web gateway subscribes to. Relative => global '
        '/camera/color/image_raw (the go2 camera node is not namespaced); absolute '
        '(e.g. /robot1/camera/color/image_raw) for a namespaced camera; empty '
        'string => camera disabled.',
    )


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
    route_planner_backend: RoutePlannerBackend = Field(
        default='far',
        description='Global route backend for full_mode=1.',
    )
    far_planner_profile: str = Field(
        default='outdoor',
        description='Picks far_planner/config/<name>.yaml.',
    )
    tare_planner_profile: str = Field(
        default='indoor_small',
        description='Picks tare_planner/config/<name>.yaml.',
    )


class VehicleConfig(BaseModel):
    """Physical footprint of the robot, consumed by the local planner."""
    model_config = ConfigDict(extra='forbid')

    length: float = Field(
        default=0.7, gt=0.0, le=5.0,
        description='Footprint length (m). -> local_planner vehicleLength.',
    )
    width: float = Field(
        default=0.3, gt=0.0, le=5.0,
        description='Footprint width (m). -> local_planner vehicleWidth.',
    )


class SensorsConfig(BaseModel):
    """Sensor mounting offsets from base_link."""
    model_config = ConfigDict(extra='forbid')

    lidar_offset_x: float = Field(
        default=0.05, ge=-2.0, le=2.0,
        description='LiDAR X offset from base_link (m). -> sensorOffsetX.',
    )
    lidar_offset_y: float = Field(
        default=0.0, ge=-2.0, le=2.0,
        description='LiDAR Y offset from base_link (m). -> sensorOffsetY.',
    )
    camera_offset_z: float = Field(
        default=0.25, ge=-2.0, le=2.0,
        description='Camera Z offset from base_link (m). -> cameraOffsetZ.',
    )


class MotionConfig(BaseModel):
    """Speed / yaw limits. Two distinct linear knobs on purpose:
    ``max_speed`` caps the autonomy planner; ``cmd_vel_max_linear`` clamps
    the command actually sent to the robot by cmd_vel_controller.
    """
    model_config = ConfigDict(extra='forbid')

    max_speed: float = Field(
        default=1.375, gt=0.0, le=5.0,
        description='Local-planner top linear speed (m/s). -> local_planner maxSpeed.',
    )
    autonomy_speed: float = Field(
        default=0.875, gt=0.0, le=5.0,
        description='Local-planner autonomy-mode cruise speed (m/s). -> autonomySpeed.',
    )
    max_accel: float = Field(
        default=2.0, gt=0.0, le=20.0,
        description='Local-planner acceleration ramp (m/s^2). -> local_planner maxAccel.',
    )
    max_decel: float = Field(
        default=2.0, gt=0.0, le=20.0,
        description='Local-planner deceleration / braking ramp (m/s^2). -> local_planner maxDecel.',
    )
    cmd_vel_max_linear: float = Field(
        default=0.8, gt=0.0, le=5.0,
        description='cmd_vel_controller linear clamp (m/s, vx and vy).',
    )
    cmd_vel_max_angular: float = Field(
        default=1.0, gt=0.0, le=10.0,
        description='cmd_vel_controller angular clamp (rad/s).',
    )
    cmd_vel_min_angular: float = Field(
        default=0.2, ge=0.0, le=10.0,
        description='cmd_vel_controller angular deadband floor (rad/s).',
    )

    @model_validator(mode='after')
    def _check_bounds(self) -> 'MotionConfig':
        if self.cmd_vel_min_angular > self.cmd_vel_max_angular:
            raise ValueError(
                'motion.cmd_vel_min_angular must be <= motion.cmd_vel_max_angular'
            )
        if self.autonomy_speed > self.max_speed:
            raise ValueError(
                'motion.autonomy_speed must be <= motion.max_speed'
            )
        return self


class RobotConfig(BaseModel):
    """Root model validated against robot.yaml."""
    model_config = ConfigDict(extra='forbid')

    robot: RobotIdentity = Field(default_factory=RobotIdentity)
    autonomy: AutonomyConfig = Field(default_factory=AutonomyConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    web_gateway: WebGatewayConfig = Field(default_factory=WebGatewayConfig)
    map: MapConfig = Field(default_factory=MapConfig)
    profiles: ProfilesConfig = Field(default_factory=ProfilesConfig)
    vehicle: VehicleConfig = Field(default_factory=VehicleConfig)
    sensors: SensorsConfig = Field(default_factory=SensorsConfig)
    motion: MotionConfig = Field(default_factory=MotionConfig)

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
