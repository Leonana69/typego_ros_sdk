"""Launch the web gateway alongside the rest of the robot stack."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('gateway_host')
    port = LaunchConfiguration('gateway_port')
    robot_id = LaunchConfiguration('robot_id')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_chunk = LaunchConfiguration('bag_chunk_seconds')
    bag_retain = LaunchConfiguration('bag_retain')
    disable_bag = LaunchConfiguration('disable_bag')
    camera_topic = LaunchConfiguration('camera_topic')
    foxglove_enabled = LaunchConfiguration('enable_foxglove_bridge')
    foxglove_port = LaunchConfiguration('foxglove_bridge_port')

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
        DeclareLaunchArgument(
            'camera_topic', default_value='camera/color/image_raw',
            description='Image topic the gateway subscribes to for the camera '
                        'view; empty string disables the camera.',
        ),
        DeclareLaunchArgument(
            'enable_foxglove_bridge', default_value='false',
            description='run foxglove_bridge alongside the gateway for '
                        'Foxglove Studio / Lichtblick clients',
        ),
        DeclareLaunchArgument(
            'foxglove_bridge_port', default_value='8765',
            description='WebSocket port for foxglove_bridge',
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
                '--camera-topic', camera_topic,
            ],
            parameters=[{
                'use_sim_time': False,
            }],
        ),
        # Generic ROS ↔ WebSocket bridge. Foxglove Studio / Lichtblick can
        # connect to ws://<host>:8765 for the 3D panel, plots, image feeds,
        # topic graph, etc. — complements the curated HMI on :8080.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(
                PythonExpression(['"', foxglove_enabled, '".lower() == "true"'])
            ),
            parameters=[{
                'port': foxglove_port,
                'address': '0.0.0.0',
                'use_sim_time': False,
            }],
        ),
    ])
