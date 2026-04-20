"""Launch the web gateway alongside the rest of the robot stack."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('gateway_host')
    port = LaunchConfiguration('gateway_port')
    robot_id = LaunchConfiguration('robot_id')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_chunk = LaunchConfiguration('bag_chunk_seconds')
    bag_retain = LaunchConfiguration('bag_retain')
    disable_bag = LaunchConfiguration('disable_bag')

    # /data/bags is the convention inside the Docker image (which creates
    # that directory at build time). On host systems that path usually
    # doesn't exist and isn't writable, so fall back to a per-user dir.
    default_bag_dir = (
        '/data/bags' if os.path.isdir('/data/bags')
        else os.path.expanduser('~/.typego/bags')
    )

    return LaunchDescription([
        DeclareLaunchArgument('gateway_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('gateway_port', default_value='8080'),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('bag_dir', default_value=default_bag_dir),
        DeclareLaunchArgument('bag_chunk_seconds', default_value='600'),
        DeclareLaunchArgument('bag_retain', default_value='6'),
        DeclareLaunchArgument(
            'disable_bag', default_value='false',
            description='set to "true" to skip the always-on rosbag recorder',
        ),
        Node(
            package='typego_web_gateway',
            executable='gateway_node',
            name='typego_web_gateway',
            output='screen',
            arguments=[
                '--host', host,
                '--port', port,
                '--robot-namespace', robot_id,
                '--bag-dir', bag_dir,
                '--bag-chunk-seconds', bag_chunk,
                '--bag-retain', bag_retain,
            ],
            parameters=[{
                'use_sim_time': False,
            }],
        ),
    ])
