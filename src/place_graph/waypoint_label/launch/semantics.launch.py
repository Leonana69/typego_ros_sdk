import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('waypoint_label'),
        'config', 'semantics.yaml')
    return LaunchDescription([
        Node(
            package='waypoint_label',
            executable='semantic_labeler_node',
            name='semantic_labeler',
            output='screen',
            parameters=[cfg],
        ),
    ])
