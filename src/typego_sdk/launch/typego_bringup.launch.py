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
]


def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        robot_id = context.perform_substitution(LaunchConfiguration('robot_id'))
        robot_type = context.perform_substitution(LaunchConfiguration('robot_type'))
        autonomy_type = context.perform_substitution(LaunchConfiguration('autonomy_type'))
        slam_map_name = context.perform_substitution(LaunchConfiguration('slam_map_name'))

        robot_index = f'robot{robot_id}' if robot_id else ''
        tf_prefix = f'/{robot_index}' if robot_index else ''

        typego_sdk_pkg = get_package_share_directory('typego_sdk')
        robot_sdk_pkg = get_package_share_directory(f'{robot_type}_sdk')
        autonomy_pkg = get_package_share_directory('vehicle_simulator')

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

        # --- Base autonomy (SLAM, waypoints service, Nav2) ---
        base_autonomy_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(typego_sdk_pkg, 'launch', 'base_autonomy.launch.py')
            ),
            launch_arguments={
                'robot_id': robot_id,
                'slam_map_name': slam_map_name,
            }.items(),
        )

        # --- Full autonomy (vehicle simulator) ---
        full_autonomy_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_pkg, 'launch', 'system_real_robot.launch.py')
            ),
            launch_arguments={
                'map_dir': os.path.join(typego_sdk_pkg, 'resource', f'Map-{slam_map_name}', f'{slam_map_name}.pcd') if slam_map_name != 'empty_map' else '',
            }.items(),
        )

        autonomy_launch = base_autonomy_launch if autonomy_type == 'base' else full_autonomy_launch

        return [
            LogInfo(msg=f'Robot namespace: "{robot_index or "<none>"}", SLAM map: "{slam_map_name}"'),
            iox_roudi,
            robot_sdk_launch,
            autonomy_launch,
            waypoints_node,
        ]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
