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
    'autonomy_type': 'base',
    'slam_backend': 'arise',
    'slam_map_name': 'empty_map',
    'launch_web_gateway': 'true',
    'web_gateway_port': '8080',
    'robot_ip': '',
    'nav2_params_file': 'nav2_params.yaml',
    'slam_params_file': 'slam.yaml',
    # profiles.* — which per-tool tuning YAML each planner loads
    'local_planner_profile': 'dog',
    'route_planner_backend': 'far',
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


def _load_defaults():
    global _ROBOT_YAML_RAW
    path = _locate_robot_yaml()
    if not path:
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
    except Exception:
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
        'route_planner_backend': ('profiles', 'route_planner_backend'),
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
                    'Ignored when autonomy_type=base.'
    ),
    DeclareLaunchArgument(
        'slam_map_name',
        default_value=_DEFAULTS['slam_map_name'],
        description='Pre-existing SLAM map name to load (passed to slam_launch.py).'
    ),
    DeclareLaunchArgument(
        'full_mode',
        default_value='0',
        description='Full-autonomy sub-mode (ignored when autonomy_type=base): '
                    '0 = plain vehicle_simulator, '
                    '1 = + FAR route planner, '
                    '2 = + TARE exploration planner.'
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
        'route_planner_backend',
        default_value=_DEFAULTS['route_planner_backend'],
        description='Route-planner backend for full_mode 1: far | pcd_grid.'
    ),
    DeclareLaunchArgument(
        'far_planner_profile',
        default_value=_DEFAULTS['far_planner_profile'],
        description='FAR route-planner profile (profiles.far_planner_profile): '
                    'indoor | outdoor. Used only in full_mode 1.'
    ),
    DeclareLaunchArgument(
        'tare_planner_profile',
        default_value=_DEFAULTS['tare_planner_profile'],
        description='TARE exploration profile (profiles.tare_planner_profile): '
                    'indoor_small | indoor_large | outdoor. Used only in full_mode 2.'
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
        full_mode = context.perform_substitution(LaunchConfiguration('full_mode'))
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
        route_planner_backend = context.perform_substitution(
            LaunchConfiguration('route_planner_backend'))
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

        if autonomy_type == 'full':
            # --- Full autonomy (vehicle simulator) ---
            full_mode_launch_files = {
                '0': 'system_real_robot.launch.py',
                '1': 'system_real_robot_with_route_planner.launch.py',
                '2': 'system_real_robot_with_exploration_planner.launch.py',
            }
            full_launch_file = full_mode_launch_files.get(
                full_mode, 'system_real_robot.launch.py')
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
                'local_planner_config': local_planner_profile,
            }
            # Mode-specific args — only pass an arg the chosen file declares.
            if full_mode == '1':
                full_launch_args['route_planner_backend'] = route_planner_backend
                full_launch_args['route_planner_config'] = far_planner_profile
                full_launch_args['vgraph_dir'] = (
                    f'{full_map_prefix}.vgh' if map_name != 'empty_map' else ''
                )
            elif full_mode == '2':
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
                    os.path.join(typego_sdk_pkg, 'launch', 'base_autonomy.launch.py')
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
