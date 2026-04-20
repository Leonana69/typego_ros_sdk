"""Launch the typego_config service node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value='',
            description='Absolute path to robot.yaml. Empty = default '
                        '($TYPEGO_CONFIG or shipped config).',
        ),
        Node(
            package='typego_config',
            executable='config_service_node',
            name='typego_config',
            output='screen',
            arguments=['--config', config_path],
        ),
    ])
