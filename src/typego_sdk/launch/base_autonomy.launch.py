import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='Robot ID suffix (e.g. "1" → namespace robot1). Leave empty for no namespace.'
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
        slam_map_name = context.perform_substitution(LaunchConfiguration('slam_map_name'))

        if robot_id:
            robot_index = f'robot{robot_id}'
            map_topic = f'/{robot_index}/map'
        else:
            robot_index = ''
            map_topic = '/map'

        typego_sdk_pkg = get_package_share_directory('typego_sdk')

        # --- SLAM ---
        slam_args = {'existing_map': slam_map_name}
        if robot_index:
            slam_args['robot_namespace'] = robot_index

        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(typego_sdk_pkg, 'launch', 'slam_launch.py')
            ),
            launch_arguments=slam_args.items(),
        )

        # --- Wait until the map topic is published before starting downstream nodes ---
        wait_for_map = ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'until ros2 topic list | grep -q "^{map_topic}$"; do '
                f'echo "Waiting for {map_topic}..."; sleep 1; done',
            ],
            output='screen',
            name='wait_for_map',
        )

        # --- Nav2 ---
        nav2_args = {}
        if robot_index:
            nav2_args['robot_namespace'] = robot_index

        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(typego_sdk_pkg, 'launch', 'nav2_launch.py')
            ),
            launch_arguments=nav2_args.items(),
        )

        # Start waypoints_service_node and Nav2 only after the map topic is ready
        post_map_actions = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_map,
                on_exit=[
                    LogInfo(msg=f'Map topic {map_topic} is available — starting Nav2.'),
                    nav2_launch,
                ],
            )
        )

        return [
            slam_launch,
            wait_for_map,
            post_map_actions,
        ]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
