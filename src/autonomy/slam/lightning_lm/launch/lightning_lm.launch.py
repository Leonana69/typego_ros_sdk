"""Launch lightning-lm as a drop-in replacement for arise_slam_mid360.

Mirrors `arise_slam.launch.py`'s public launch arguments so the parent
`system_real_robot.launch.py` can include either backend interchangeably.
Lightning-lm itself ships no launch files; this wrapper starts the
`run_slam_online` executable with a workspace-tuned YAML config.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("lightning"),
        "config",
        "livox_mid360.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    map_dir = LaunchConfiguration("map_dir")

    declare_config = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to lightning-lm YAML config.",
    )
    declare_map_dir = DeclareLaunchArgument(
        "map_dir",
        default_value="",
        description=(
            "Optional path passed through for parity with arise_slam.launch.py. "
            "lightning-lm reads map_path from the YAML config; this arg is "
            "currently unused but kept so callers can include either backend "
            "with the same launch_arguments dict."
        ),
    )

    # Match arise_slam.launch.py's other declarations so callers don't error
    # when passing the full set of launch args.
    declare_odom_topic = DeclareLaunchArgument("odom_topic", default_value="integrated_to_init")
    declare_world_frame = DeclareLaunchArgument("world_frame", default_value="map")
    declare_world_frame_rot = DeclareLaunchArgument("world_frame_rot", default_value="map_rot")
    declare_sensor_frame = DeclareLaunchArgument("sensor_frame", default_value="sensor")
    declare_sensor_frame_rot = DeclareLaunchArgument("sensor_frame_rot", default_value="sensor_rot")

    run_slam = Node(
        package="lightning",
        executable="run_slam_online",
        name="lightning_slam",
        output="screen",
        arguments=["-config", config_file],
    )

    # Bridge typego_interface/CustomMsg -> livox_ros_driver2/CustomMsg so
    # lightning's CustomMsg subscriber can consume the workspace's Livox driver.
    custom_msg_bridge = Node(
        package="lightning",
        executable="typego_to_livox_bridge.py",
        name="lightning_livox_bridge",
        output="screen",
        arguments=[
            "--input", "/livox/lidar_custom",
            "--output", "/lightning/livox_custom",
        ],
    )

    return LaunchDescription([
        declare_config,
        declare_map_dir,
        declare_odom_topic,
        declare_world_frame,
        declare_world_frame_rot,
        declare_sensor_frame,
        declare_sensor_frame_rot,
        custom_msg_bridge,
        run_slam,
    ])
