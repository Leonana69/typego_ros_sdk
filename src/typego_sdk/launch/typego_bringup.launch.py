import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─── Defaults from robot.yaml ────────────────────────────────────────────────
# robot.yaml is validated with pydantic here, at launch-description time, and
# an invalid config raises rather than falling back. Previously this path
# parsed raw YAML and swallowed every error into a table of hardcoded
# fallbacks -- so a missing or malformed file silently started the stack on a
# different robot geometry than the one the collision table was baked for.
#
# The old workaround comment claimed typego_config could not be imported
# because "ros2 launch runs under /usr/bin/python3 without pydantic". That is
# not true of this workspace or the Docker image; both have it. Discovery and
# merging still come from typego_config.bootstrap, which is dependency-free,
# so the import chain does not require pydantic until validation itself.
#
# CLI overrides (`key:=value`) still win: launch.substitutions prefers the
# CLI-provided value over the declared default.
from typego_config.bootstrap import load_merged            # noqa: E402
from typego_config.loader import ConfigError, load         # noqa: E402


# Launch-arg name -> dotted path in the validated config.
_ARG_SOURCES = {
    'robot_id': ('robot', 'id'),
    'robot_type': ('robot', 'type'),
    'autonomy_type': ('autonomy', 'type'),
    'planner': ('autonomy', 'planner'),
    'slam_backend': ('autonomy', 'slam_backend'),
    'slam_map_name': ('map', 'slam_map_name'),
    'launch_web_gateway': ('web_gateway', 'enabled'),
    'web_gateway_port': ('web_gateway', 'port'),
    'robot_ip': ('network', 'robot_ip'),
    'nav2_params_file': ('profiles', 'nav2_params_file'),
    'slam_params_file': ('profiles', 'slam_params_file'),
    'local_planner_profile': ('profiles', 'local_planner_profile'),
    'far_planner_profile': ('profiles', 'far_planner_profile'),
    'tare_planner_profile': ('profiles', 'tare_planner_profile'),
    'sensor_offset_x': ('sensors', 'lidar_offset_x'),
    'sensor_offset_y': ('sensors', 'lidar_offset_y'),
    'camera_offset_z': ('sensors', 'camera_offset_z'),
    'vehicle_length': ('vehicle', 'length'),
    'vehicle_width': ('vehicle', 'width'),
    'max_speed': ('motion', 'max_speed'),
    'autonomy_speed': ('motion', 'autonomy_speed'),
    'max_accel': ('motion', 'max_accel'),
    'max_decel': ('motion', 'max_decel'),
    'cmd_vel_max_linear': ('motion', 'cmd_vel_max_linear'),
    'cmd_vel_max_angular': ('motion', 'cmd_vel_max_angular'),
    'cmd_vel_min_angular': ('motion', 'cmd_vel_min_angular'),
}

_ROBOT_YAML_RAW = {}


def _dig(data, *keys, default=None):
    cur = data
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k, default)
    return cur


def _as_launch_str(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _load_defaults():
    """Validated config -> launch-argument defaults. Raises on a bad config."""
    global _ROBOT_YAML_RAW
    try:
        cfg = load()
    except ConfigError as exc:
        raise RuntimeError(
            f'\n{"=" * 66}\n'
            f'typego_bringup: robot.yaml is invalid — refusing to launch.\n'
            f'{exc}\n'
            f'{"-" * 66}\n'
            f'Launching on built-in defaults would silently run the stack on a\n'
            f'different vehicle geometry than local_planner\'s collision table\n'
            f'was baked for, so this is fatal rather than a warning.\n'
            f'Check the file with:  make config_validate\n'
            f'{"=" * 66}'
        ) from exc

    # The raw merged dict still backs the few consumers that read sections the
    # launch args do not cover (edge_service, web_gateway extras).
    try:
        _ROBOT_YAML_RAW, _ = load_merged()
    except Exception:
        _ROBOT_YAML_RAW = {}

    dumped = cfg.model_dump(mode='json')
    return {
        arg: _as_launch_str(_dig(dumped, *path))
        for arg, path in _ARG_SOURCES.items()
    }


_DEFAULTS = _load_defaults()


ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_id',
        default_value=_DEFAULTS['robot_id'],
        description='Robot ID suffix (e.g. "1" → namespace robot1). Leave empty for no namespace.'
    ),
    DeclareLaunchArgument(
        'robot_type',
        default_value=_DEFAULTS['robot_type'],
        description='Robot type; selects which <robot_type>_sdk package to launch.'
    ),
    DeclareLaunchArgument(
        'autonomy_type',
        default_value=_DEFAULTS['autonomy_type'],
        description='Autonomy type; selects which autonomy type to launch.'
    ),
    DeclareLaunchArgument(
        'slam_backend',
        default_value=_DEFAULTS['slam_backend'],
        description='LIO backend for full autonomy: "arise" or "lightning". '
                    'Ignored when autonomy_type=2d.'
    ),
    DeclareLaunchArgument(
        'slam_map_name',
        default_value=_DEFAULTS['slam_map_name'],
        description='Pre-existing SLAM map name to load (passed to slam_launch.py).'
    ),
    DeclareLaunchArgument(
        'planner',
        default_value=_DEFAULTS['planner'],
        description='3D-autonomy planner (ignored when autonomy_type=2d): '
                    'local = local planner only, '
                    'far = + FAR route planner, '
                    'pcd_grid = + PCD grid planner, '
                    'exploration = + TARE exploration planner.'
    ),
    DeclareLaunchArgument(
        # Retired. Declared only so `full_mode:=N` fails with a migration
        # message instead of ros2 launch's generic unknown-argument error.
        'full_mode',
        default_value='',
        description='RETIRED -- use planner:=local|far|pcd_grid|exploration.'
    ),
    DeclareLaunchArgument(
        'launch_web_gateway',
        default_value=_DEFAULTS['launch_web_gateway'],
        description='If "true", launches the typego_web_gateway HMI on :8080.'
    ),
    DeclareLaunchArgument(
        'web_gateway_port',
        default_value=_DEFAULTS['web_gateway_port'],
        description='TCP port for the web gateway HTTP+WebSocket server.'
    ),
    DeclareLaunchArgument(
        'robot_ip',
        default_value=_DEFAULTS['robot_ip'],
        description='Robot LAN IP (network.robot_ip).'
    ),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=_DEFAULTS['nav2_params_file'],
        description='Nav2 parameter file (profiles.nav2_params_file in robot.yaml).'
    ),
    DeclareLaunchArgument(
        'slam_params_file',
        default_value=_DEFAULTS['slam_params_file'],
        description='SLAM parameter file (profiles.slam_params_file in robot.yaml).'
    ),
    DeclareLaunchArgument(
        'local_planner_profile',
        default_value=_DEFAULTS['local_planner_profile'],
        description='local_planner config profile (profiles.local_planner_profile): '
                    'dog | omniDir | standard.'
    ),
    DeclareLaunchArgument(
        'far_planner_profile',
        default_value=_DEFAULTS['far_planner_profile'],
        description='FAR route-planner profile (profiles.far_planner_profile): '
                    'indoor | outdoor. Used only when planner=far.'
    ),
    DeclareLaunchArgument(
        'tare_planner_profile',
        default_value=_DEFAULTS['tare_planner_profile'],
        description='TARE exploration profile (profiles.tare_planner_profile): '
                    'indoor_small | indoor_large | outdoor. Used only when planner=exploration.'
    ),
    DeclareLaunchArgument(
        'sensor_offset_x',
        default_value=_DEFAULTS['sensor_offset_x'],
        description='LiDAR X offset from base_link, m (sensors.lidar_offset_x).'
    ),
    DeclareLaunchArgument(
        'sensor_offset_y',
        default_value=_DEFAULTS['sensor_offset_y'],
        description='LiDAR Y offset from base_link, m (sensors.lidar_offset_y).'
    ),
    DeclareLaunchArgument(
        'camera_offset_z',
        default_value=_DEFAULTS['camera_offset_z'],
        description='Camera Z offset from base_link, m (sensors.camera_offset_z).'
    ),
    DeclareLaunchArgument(
        'vehicle_length',
        default_value=_DEFAULTS['vehicle_length'],
        description='Vehicle footprint length, m (vehicle.length).'
    ),
    DeclareLaunchArgument(
        'vehicle_width',
        default_value=_DEFAULTS['vehicle_width'],
        description='Vehicle footprint width, m (vehicle.width).'
    ),
    DeclareLaunchArgument(
        'max_speed',
        default_value=_DEFAULTS['max_speed'],
        description='Local-planner top linear speed, m/s (motion.max_speed).'
    ),
    DeclareLaunchArgument(
        'autonomy_speed',
        default_value=_DEFAULTS['autonomy_speed'],
        description='Local-planner autonomy cruise speed, m/s (motion.autonomy_speed).'
    ),
    DeclareLaunchArgument(
        'max_accel',
        default_value=_DEFAULTS['max_accel'],
        description='Local-planner acceleration ramp, m/s^2 (motion.max_accel).'
    ),
    DeclareLaunchArgument(
        'max_decel',
        default_value=_DEFAULTS['max_decel'],
        description='Local-planner deceleration ramp / braking, m/s^2 (motion.max_decel).'
    ),
    DeclareLaunchArgument(
        'cmd_vel_max_linear',
        default_value=_DEFAULTS['cmd_vel_max_linear'],
        description='cmd_vel_controller linear clamp, m/s (motion.cmd_vel_max_linear).'
    ),
    DeclareLaunchArgument(
        'cmd_vel_max_angular',
        default_value=_DEFAULTS['cmd_vel_max_angular'],
        description='cmd_vel_controller angular clamp, rad/s (motion.cmd_vel_max_angular).'
    ),
    DeclareLaunchArgument(
        'cmd_vel_min_angular',
        default_value=_DEFAULTS['cmd_vel_min_angular'],
        description='cmd_vel_controller angular deadband floor, rad/s '
                    '(motion.cmd_vel_min_angular).'
    ),
    DeclareLaunchArgument(
        'launch_config_service',
        default_value='true',
        description='If "true", spawns the typego_config service node for '
                    'runtime introspection of robot.yaml.'
    ),
    DeclareLaunchArgument(
        'launch_place_graph_viz',
        default_value='false',
        description='If "true", spawns place_graph_ros/place_graph_node, '
                    'which republishes the place-graph segmentation as RViz '
                    'markers (debug visualization on /place_graph_node/markers).'
    ),
    DeclareLaunchArgument(
        'launch_semantics',
        default_value='true',
        description='If "true", spawns waypoint_label/semantic_labeler_node, '
                    'which labels place-graph regions via the qwenvl VLM '
                    '(endpoint from robot.yaml network.edge_service).'
    ),
]


def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        robot_id = context.perform_substitution(LaunchConfiguration('robot_id'))
        robot_type = context.perform_substitution(LaunchConfiguration('robot_type'))
        autonomy_type = context.perform_substitution(LaunchConfiguration('autonomy_type'))
        slam_backend = context.perform_substitution(LaunchConfiguration('slam_backend'))
        slam_map_name = context.perform_substitution(LaunchConfiguration('slam_map_name'))
        planner = context.perform_substitution(LaunchConfiguration('planner'))
        launch_web_gateway = context.perform_substitution(
            LaunchConfiguration('launch_web_gateway')).lower() == 'true'
        web_gateway_port = context.perform_substitution(
            LaunchConfiguration('web_gateway_port'))
        robot_ip = context.perform_substitution(
            LaunchConfiguration('robot_ip'))
        nav2_params_file = context.perform_substitution(
            LaunchConfiguration('nav2_params_file'))
        slam_params_file = context.perform_substitution(
            LaunchConfiguration('slam_params_file'))
        local_planner_profile = context.perform_substitution(
            LaunchConfiguration('local_planner_profile'))
        legacy_full_mode = context.perform_substitution(
            LaunchConfiguration('full_mode'))
        if legacy_full_mode:
            raise RuntimeError(
                f"full_mode:={legacy_full_mode} was replaced by "
                f"planner:=local|far|pcd_grid|exploration "
                f"(0->local, 1->far or pcd_grid, 2->exploration). "
                f"Set autonomy.planner in robot.yaml, or pass planner:=<name>."
            )
        far_planner_profile = context.perform_substitution(
            LaunchConfiguration('far_planner_profile'))
        tare_planner_profile = context.perform_substitution(
            LaunchConfiguration('tare_planner_profile'))
        sensor_offset_x = context.perform_substitution(
            LaunchConfiguration('sensor_offset_x'))
        sensor_offset_y = context.perform_substitution(
            LaunchConfiguration('sensor_offset_y'))
        camera_offset_z = context.perform_substitution(
            LaunchConfiguration('camera_offset_z'))
        vehicle_length = context.perform_substitution(
            LaunchConfiguration('vehicle_length'))
        vehicle_width = context.perform_substitution(
            LaunchConfiguration('vehicle_width'))
        max_speed = context.perform_substitution(
            LaunchConfiguration('max_speed'))
        autonomy_speed = context.perform_substitution(
            LaunchConfiguration('autonomy_speed'))
        max_accel = context.perform_substitution(
            LaunchConfiguration('max_accel'))
        max_decel = context.perform_substitution(
            LaunchConfiguration('max_decel'))
        cmd_vel_max_linear = context.perform_substitution(
            LaunchConfiguration('cmd_vel_max_linear'))
        cmd_vel_max_angular = context.perform_substitution(
            LaunchConfiguration('cmd_vel_max_angular'))
        cmd_vel_min_angular = context.perform_substitution(
            LaunchConfiguration('cmd_vel_min_angular'))
        launch_config_service = context.perform_substitution(
            LaunchConfiguration('launch_config_service')).lower() == 'true'
        launch_place_graph_viz = context.perform_substitution(
            LaunchConfiguration('launch_place_graph_viz')).lower() == 'true'
        launch_semantics = context.perform_substitution(
            LaunchConfiguration('launch_semantics')).lower() == 'true'

        map_name = slam_map_name[4:] if slam_map_name.startswith('Map-') else slam_map_name
        robot_index = f'robot{robot_id}' if robot_id else ''
        tf_prefix = f'/{robot_index}' if robot_index else ''

        typego_sdk_pkg = get_package_share_directory('typego_sdk')
        robot_sdk_pkg = get_package_share_directory(f'{robot_type}_sdk')

        # --- iceoryx router daemon ---
        iox_roudi = ExecuteProcess(
            cmd=['iox-roudi'],
            output='screen',
            name='iox_roudi',
        )

        # --- Robot-type SDK ---
        robot_sdk_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_sdk_pkg, 'launch', f'{robot_type}_sdk.launch.py')
            ),
            launch_arguments={
                'autonomy_type': autonomy_type,
                'robot_id': robot_id,
                'robot_ip': robot_ip,
                # motion.* — cmd_vel_controller clamps (applies to base + full)
                'cmd_vel_max_linear': cmd_vel_max_linear,
                'cmd_vel_max_angular': cmd_vel_max_angular,
                'cmd_vel_min_angular': cmd_vel_min_angular,
            }.items(),
        )

        # --- place_graph_waypoints_node ---
        waypoints_remappings = []
        if robot_index:
            waypoints_remappings = [
                ('/tf', f'{tf_prefix}/tf'),
                ('/tf_static', f'{tf_prefix}/tf_static'),
            ]

        waypoints_node = Node(
            package='typego_sdk',
            executable='place_graph_waypoints_node',
            output='screen',
            remappings=waypoints_remappings,
            parameters=[{
                'slam_map_name': map_name,
                'map_topic': 'map',
            }],
        )

        # --- place_graph_node (debug RViz visualization of the segmentation) ---
        place_graph_viz_node = Node(
            package='place_graph_ros',
            executable='place_graph_node',
            output='screen',
            parameters=[{
                'map_topic': f'{tf_prefix}/map' if tf_prefix else '/map',
            }],
        )

        if autonomy_type == '3d':
            # --- Full autonomy (vehicle simulator) ---
            planner_launch_files = {
                'local': 'system_real_robot.launch.py',
                'far': 'system_real_robot_with_route_planner.launch.py',
                'pcd_grid': 'system_real_robot_with_route_planner.launch.py',
                'exploration':
                    'system_real_robot_with_exploration_planner.launch.py',
            }
            full_launch_file = planner_launch_files.get(
                planner, 'system_real_robot.launch.py')
            autonomy_pkg = get_package_share_directory('vehicle_simulator')
            full_map_prefix = os.path.join(
                typego_sdk_pkg, 'resource', f'Map-{map_name}', map_name)
            full_map_pcd = f'{full_map_prefix}.pcd' if map_name != 'empty_map' else ''
            # Common args every system_real_robot*.launch.py declares.
            full_launch_args = {
                'map_dir': full_map_pcd,
                'slam_backend': slam_backend,
                'sensorOffsetX': sensor_offset_x,
                'sensorOffsetY': sensor_offset_y,
                'cameraOffsetZ': camera_offset_z,
                'vehicleLength': vehicle_length,
                'vehicleWidth': vehicle_width,
                'maxSpeed': max_speed,
                'autonomySpeed': autonomy_speed,
                'maxAccel': max_accel,
                'maxDecel': max_decel,
                'local_planner_config': local_planner_profile,
            }
            # Mode-specific args — only pass an arg the chosen file declares.
            if planner in ('far', 'pcd_grid'):
                # Both route planners share one launch file; the planner name
                # is the backend name it expects.
                full_launch_args['route_planner_backend'] = planner
                full_launch_args['route_planner_config'] = far_planner_profile
                full_launch_args['vgraph_dir'] = (
                    f'{full_map_prefix}.vgh' if map_name != 'empty_map' else ''
                )
            elif planner == 'exploration':
                full_launch_args['exploration_planner_config'] = tare_planner_profile
            autonomy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(autonomy_pkg, 'launch', full_launch_file)
                ),
                launch_arguments=full_launch_args.items(),
            )
        else:
            # --- Base autonomy (SLAM, waypoints service, Nav2) ---
            autonomy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('autonomy_2d'),
                                 'launch', 'autonomy_2d.launch.py')
                ),
                launch_arguments={
                    'robot_id': robot_id,
                    'slam_map_name': map_name,
                    'nav2_params_file': nav2_params_file,
                    'slam_params_file': slam_params_file,
                }.items(),
            )

        actions = [
            LogInfo(msg=f'Robot namespace: "{robot_index or "<none>"}", SLAM map: "{map_name}"'),
            iox_roudi,
            robot_sdk_launch,
            autonomy_launch,
            waypoints_node,
        ]

        if launch_place_graph_viz:
            actions.append(place_graph_viz_node)

        # --- semantic_labeler_node (VLM region labeling) ---
        if launch_semantics:
            try:
                sem_pkg = get_package_share_directory('waypoint_label')
            except Exception as exc:  # package not built yet — skip with a log
                actions.append(LogInfo(
                    msg=f'waypoint_label share dir not found ({exc}); '
                        'skipping semantic labeler.'
                ))
            else:
                # qwenvl /process endpoint from robot.yaml network.edge_service;
                # an empty string lets the node fall back to $EDGE_SERVICE_IP /
                # localhost (make console/launch export those from robot.yaml).
                edge = _dig(_ROBOT_YAML_RAW, 'network', 'edge_service') or {}
                vlm_ip = str((edge.get('ip') if isinstance(edge, dict)
                              else '') or '').strip()
                vlm_port = str((edge.get('port') if isinstance(edge, dict)
                                else '') or '50049').strip()
                vlm_endpoint = (
                    f'http://{vlm_ip}:{vlm_port}/process' if vlm_ip else '')
                sem_cfg = os.path.join(sem_pkg, 'config', 'semantics.yaml')
                actions.append(Node(
                    package='waypoint_label',
                    executable='semantic_labeler_node',
                    name='semantic_labeler',
                    output='screen',
                    remappings=waypoints_remappings,
                    # The YAML file sets defaults; the inline dict applied after
                    # overrides vlm_endpoint with the robot.yaml-derived value.
                    parameters=[sem_cfg, {
                        'vlm_endpoint': vlm_endpoint,
                    }],
                ))

        # --- typego_config service node (runtime introspection of robot.yaml) ---
        if launch_config_service:
            try:
                config_pkg = get_package_share_directory('typego_config')
            except Exception as exc:
                actions.append(LogInfo(
                    msg=f'typego_config share dir not found ({exc}); skipping config service.'
                ))
            else:
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(config_pkg, 'launch', 'config_service.launch.py')
                    ),
                    launch_arguments={'config_path': ''}.items(),
                ))

        if launch_web_gateway:
            try:
                web_pkg = get_package_share_directory('typego_web_gateway')
            except Exception as exc:  # package not built yet — skip with a log
                actions.append(LogInfo(
                    msg=f'typego_web_gateway share dir not found ({exc}); skipping HMI.'
                ))
            else:
                web_args = {
                    'robot_id': robot_id,
                    'gateway_port': web_gateway_port,
                }
                web_cfg = _dig(_ROBOT_YAML_RAW, 'web_gateway') or {}
                for k_yaml, k_arg in (
                    ('host', 'gateway_host'),
                    ('bag_dir', 'bag_dir'),
                    ('bag_chunk_seconds', 'bag_chunk_seconds'),
                    ('bag_retain', 'bag_retain'),
                    ('camera_topic', 'camera_topic'),
                ):
                    val = web_cfg.get(k_yaml) if isinstance(web_cfg, dict) else None
                    if val is not None:
                        web_args[k_arg] = str(val)
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(web_pkg, 'launch', 'web_gateway.launch.py')
                    ),
                    launch_arguments=web_args.items(),
                ))

        return actions

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
