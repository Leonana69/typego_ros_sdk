import os
import sys

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
# Read robot.yaml at import time so DeclareLaunchArgument defaults reflect
# the single source of truth. CLI overrides (`key:=value`) still win because
# launch.substitutions prefers the CLI-provided value over the declared
# default.
#
# We deliberately avoid importing ``typego_config`` here — ros2 launch runs
# under /usr/bin/python3 by default, and typego_config depends on pydantic
# which typically lives in a conda env. Use plain PyYAML (which rosdep
# provides on Humble) with a tolerant lookup; full schema validation
# happens later in the typego_config service node.
_DEFAULT_FALLBACKS = {
    'robot_id': '',
    'robot_type': 'go2',
    'autonomy_type': '2d',
    'slam_backend': 'arise',
    'slam_map_name': 'empty_map',
    'launch_web_gateway': 'true',
    'web_gateway_port': '8080',
    'robot_ip': '',
    'nav2_params_file': 'nav2_params.yaml',
    'slam_params_file': 'slam.yaml',
    # profiles.* — which per-tool tuning YAML each planner loads
    'local_planner_profile': 'dog',
    'planner': 'local',
    'far_planner_profile': 'outdoor',
    'tare_planner_profile': 'indoor_small',
    # sensors.* — sensor mounting offsets
    'sensor_offset_x': '0.05',
    'sensor_offset_y': '0.0',
    'camera_offset_z': '0.25',
    # vehicle.* — physical footprint
    'vehicle_length': '0.7',
    'vehicle_width': '0.3',
    # motion.* — speed / yaw limits
    'max_speed': '1.375',
    'autonomy_speed': '0.875',
    'max_accel': '2.0',
    'max_decel': '2.0',
    'cmd_vel_max_linear': '0.8',
    'cmd_vel_max_angular': '1.0',
    'cmd_vel_min_angular': '0.2',
}


def _locate_robot_yaml():
    env = os.environ.get('TYPEGO_CONFIG')
    if env and os.path.isfile(env):
        return env
    try:
        share = get_package_share_directory('typego_config')
    except Exception:
        return None
    candidate = os.path.join(share, 'config', 'robot.yaml')
    return candidate if os.path.isfile(candidate) else None


def _dig(data, *keys, default=None):
    cur = data
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k, default)
    return cur


def _deep_merge_dict(base, overlay):
    out = {}
    for key in set(base) | set(overlay):
        if key in overlay and isinstance(overlay[key], dict) \
                and isinstance(base.get(key), dict):
            out[key] = _deep_merge_dict(base[key], overlay[key])
        elif key in overlay:
            out[key] = overlay[key]
        else:
            out[key] = base[key]
    return out


_ROBOT_YAML_RAW = {}


def _warn_fallback(reason):
    """Never fail silently: the built-in fallbacks disagree with the shipped
    robot.yaml on physical geometry (footprint 0.7x0.3 vs 0.75x0.4, lidar
    offset 0.05 vs 0.30), and local_planner's collision table is baked for
    the robot.yaml values. Launching on the fallbacks is a real hazard, so
    say so loudly rather than proceeding quietly.
    """
    sys.stderr.write(
        '\n'
        '========================================================\n'
        'typego_bringup: FALLING BACK TO BUILT-IN DEFAULTS\n'
        f'  reason: {reason}\n'
        '  consequence: vehicle/sensor/motion parameters will NOT\n'
        '    match robot.yaml. The local_planner collision table is\n'
        '    baked for the robot.yaml footprint; these defaults differ.\n'
        '  fix: repair the file above, or set TYPEGO_CONFIG to a valid\n'
        '    robot.yaml, then relaunch. Check it with:\n'
        '      make config_validate\n'
        '========================================================\n\n'
    )


def _load_defaults():
    global _ROBOT_YAML_RAW
    path = _locate_robot_yaml()
    if not path:
        _warn_fallback(
            'robot.yaml not found ($TYPEGO_CONFIG unset or missing, and no '
            'config/robot.yaml in the typego_config share directory)'
        )
        return dict(_DEFAULT_FALLBACKS)
    try:
        import yaml
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        # Honour $TYPEGO_PROFILE so setting it pre-launch reshapes the
        # defaults the same way `profile:=...` would at launch time.
        profile = os.environ.get('TYPEGO_PROFILE')
        if profile:
            p_path = os.path.join(
                os.path.dirname(path), 'profiles', f'{profile}.yaml'
            )
            if os.path.isfile(p_path):
                with open(p_path, 'r') as f:
                    overlay = yaml.safe_load(f) or {}
                if isinstance(overlay, dict):
                    data = _deep_merge_dict(
                        data if isinstance(data, dict) else {}, overlay)
            else:
                # A requested profile that does not exist is a typo, not a
                # no-op. Name it rather than silently using base defaults.
                sys.stderr.write(
                    f'typego_bringup: WARNING TYPEGO_PROFILE={profile!r} '
                    f'requested but {p_path} does not exist; continuing '
                    f'without the overlay.\n'
                )
    except Exception as exc:
        _warn_fallback(f'{path} could not be read or parsed: '
                       f'{type(exc).__name__}: {exc}')
        return dict(_DEFAULT_FALLBACKS)
    _ROBOT_YAML_RAW = data if isinstance(data, dict) else {}

    out = dict(_DEFAULT_FALLBACKS)
    mapping = {
        'robot_id': ('robot', 'id'),
        'robot_type': ('robot', 'type'),
        'autonomy_type': ('autonomy', 'type'),
        'slam_backend': ('autonomy', 'slam_backend'),
        'slam_map_name': ('map', 'slam_map_name'),
        'launch_web_gateway': ('web_gateway', 'enabled'),
        'web_gateway_port': ('web_gateway', 'port'),
        'robot_ip': ('network', 'robot_ip'),
        'nav2_params_file': ('profiles', 'nav2_params_file'),
        'slam_params_file': ('profiles', 'slam_params_file'),
        'local_planner_profile': ('profiles', 'local_planner_profile'),
        'planner': ('autonomy', 'planner'),
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
    for key, path_keys in mapping.items():
        value = _dig(data, *path_keys)
        if value is None:
            continue
        if isinstance(value, bool):
            out[key] = 'true' if value else 'false'
        else:
            out[key] = str(value)
    return out


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
