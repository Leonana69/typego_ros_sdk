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
    'nav2_params_file': 'nav2_params.yaml',
    'slam_params_file': 'slam.yaml',
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
        'nav2_params_file': ('profiles', 'nav2_params_file'),
        'slam_params_file': ('profiles', 'slam_params_file'),
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
        'launch_config_service',
        default_value='true',
        description='If "true", spawns the typego_config service node for '
                    'runtime introspection of robot.yaml.'
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
        nav2_params_file = context.perform_substitution(
            LaunchConfiguration('nav2_params_file'))
        slam_params_file = context.perform_substitution(
            LaunchConfiguration('slam_params_file'))
        launch_config_service = context.perform_substitution(
            LaunchConfiguration('launch_config_service')).lower() == 'true'

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
            }.items(),
        )

        # --- waypoints_service_node ---
        waypoints_remappings = []
        if robot_index:
            waypoints_remappings = [
                ('/tf', f'{tf_prefix}/tf'),
                ('/tf_static', f'{tf_prefix}/tf_static'),
            ]

        waypoints_node = Node(
            package='typego_sdk',
            executable='waypoints_service_node',
            output='screen',
            remappings=waypoints_remappings,
            parameters=[{
                'slam_map_name': slam_map_name,
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
            autonomy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(autonomy_pkg, 'launch', full_launch_file)
                ),
                launch_arguments={
                    'map_dir': os.path.join(typego_sdk_pkg, 'resource', f'Map-{slam_map_name}', f'{slam_map_name}.pcd') if slam_map_name != 'empty_map' else '',
                    'slam_backend': slam_backend,
                }.items(),
            )
        else:
            # --- Base autonomy (SLAM, waypoints service, Nav2) ---
            autonomy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(typego_sdk_pkg, 'launch', 'base_autonomy.launch.py')
                ),
                launch_arguments={
                    'robot_id': robot_id,
                    'slam_map_name': slam_map_name,
                    'nav2_params_file': nav2_params_file,
                    'slam_params_file': slam_params_file,
                }.items(),
            )

        actions = [
            LogInfo(msg=f'Robot namespace: "{robot_index or "<none>"}", SLAM map: "{slam_map_name}"'),
            iox_roudi,
            robot_sdk_launch,
            autonomy_launch,
            waypoints_node,
        ]

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
