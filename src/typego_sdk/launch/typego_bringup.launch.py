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

ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='Robot ID suffix (e.g. "1" → namespace robot1). Leave empty for no namespace.'
    ),
    DeclareLaunchArgument(
        'robot_type',
        default_value='go2',
        description='Robot type; selects which <robot_type>_sdk package to launch.'
    ),
    DeclareLaunchArgument(
        'autonomy_type',
        default_value='base',
        description='Autonomy type; selects which autonomy type to launch.'
    ),
    DeclareLaunchArgument(
        'slam_map_name',
        default_value='empty_map',
        description='Pre-existing SLAM map name to load (passed to slam_launch.py).'
    ),
    DeclareLaunchArgument(
        'launch_web_gateway',
        default_value='true',
        description='If "true", launches the typego_web_gateway HMI on :8080.'
    ),
    DeclareLaunchArgument(
        'web_gateway_port',
        default_value='8080',
        description='TCP port for the web gateway HTTP+WebSocket server.'
    ),
]


def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        robot_id = context.perform_substitution(LaunchConfiguration('robot_id'))
        robot_type = context.perform_substitution(LaunchConfiguration('robot_type'))
        autonomy_type = context.perform_substitution(LaunchConfiguration('autonomy_type'))
        slam_map_name = context.perform_substitution(LaunchConfiguration('slam_map_name'))
        launch_web_gateway = context.perform_substitution(
            LaunchConfiguration('launch_web_gateway')).lower() == 'true'
        web_gateway_port = context.perform_substitution(
            LaunchConfiguration('web_gateway_port'))

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
            autonomy_pkg = get_package_share_directory('vehicle_simulator')
            autonomy_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(autonomy_pkg, 'launch', 'system_real_robot.launch.py')
                    # os.path.join(autonomy_pkg, 'launch', 'system_real_robot_with_route_planner.launch.py')
                ),
                launch_arguments={
                    'map_dir': os.path.join(typego_sdk_pkg, 'resource', f'Map-{slam_map_name}', f'{slam_map_name}.pcd') if slam_map_name != 'empty_map' else '',
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
                }.items(),
            )

        actions = [
            LogInfo(msg=f'Robot namespace: "{robot_index or "<none>"}", SLAM map: "{slam_map_name}"'),
            iox_roudi,
            robot_sdk_launch,
            autonomy_launch,
            waypoints_node,
        ]

        if launch_web_gateway:
            try:
                web_pkg = get_package_share_directory('typego_web_gateway')
            except Exception as exc:  # package not built yet — skip with a log
                actions.append(LogInfo(
                    msg=f'typego_web_gateway share dir not found ({exc}); skipping HMI.'
                ))
            else:
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(web_pkg, 'launch', 'web_gateway.launch.py')
                    ),
                    launch_arguments={
                        'robot_id': robot_id,
                        'gateway_port': web_gateway_port,
                    }.items(),
                ))

        return actions

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
