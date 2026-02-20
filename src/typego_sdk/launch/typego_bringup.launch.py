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

        return [
            LogInfo(msg=f'Robot namespace: "{robot_index or "<none>"}", SLAM map: "{slam_map_name}"'),
            iox_roudi,
            robot_sdk_launch,
            base_autonomy_launch,
        ]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
