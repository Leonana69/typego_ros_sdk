from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
import json


def generate_launch_description():

    # --- Arguments ---
    ARGUMENTS = [
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace for this robot (topics only, not TF frames).'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value='slam.yaml',
            description='SLAM configuration YAML.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Launch RViz?'
        ),
        DeclareLaunchArgument(
            'existing_map',
            default_value='empty_map',
            description='Start SLAM with the given pre-existing map file.'
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        pkg_dir = FindPackageShare("typego_sdk").find("typego_sdk")

        ns = context.perform_substitution(LaunchConfiguration("robot_namespace"))
        slam_yaml = context.perform_substitution(LaunchConfiguration("slam_params_file"))
        rviz_flag = LaunchConfiguration("rviz")
        existing_map = context.perform_substitution(LaunchConfiguration("existing_map"))
        if existing_map != 'empty_map':
            use_existing_map = True
        else:
            use_existing_map = False

        slam_params_path = os.path.join(pkg_dir, "config", slam_yaml)
        rviz_config_path = os.path.join(pkg_dir, "config", "slam.rviz")
        map_file_path = os.path.join(pkg_dir, f"resource/Map-{existing_map}/{existing_map}")
        init_pose_path = os.path.join(pkg_dir, f"resource/Map-{existing_map}/init_pose.json")

        # ------------------------------------------------------------
        # 🔥 FRAME IDs MUST MATCH WHAT OTHER NODES PUBLISH
        # ------------------------------------------------------------
        # If namespace exists, prefix all frame IDs
        if ns:
            scan_topic = f"/{ns}/scan"  # Relative to namespace
            tf_prefix = f"/{ns}"
        else:
            scan_topic = "/scan"
            tf_prefix = ""

        print(f"🤖 SLAM Namespace: '{ns}'")

        # Common SLAM parameters
        slam_params = {
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "scan_topic": scan_topic,
        }

        # Read initial pose from init_pose.json if it exists
        map_start_pose = [0.0, 0.0, 0.0]  # Default values
        if os.path.exists(init_pose_path):
            try:
                with open(init_pose_path, 'r') as f:
                    init_pose_data = json.load(f)
                    map_start_pose = [
                        init_pose_data.get("x", 0.0),
                        init_pose_data.get("y", 0.0),
                        init_pose_data.get("yaw", 0.0)
                    ]
                print(f"📍 Loaded initial pose from {init_pose_path}: {map_start_pose}")
            except (json.JSONDecodeError, KeyError) as e:
                print(f"⚠️  Failed to parse {init_pose_path}: {e}. Using default pose.")
        else:
            print(f"⚠️  {init_pose_path} not found. Using default pose: {map_start_pose}")

        map_params = {
            "map_file_name": map_file_path,
            "map_start_pose": map_start_pose
        }

        nodes = []

        # ------------------------------------------------------------
        # 🔵 SLAM Toolbox Node (start with existing map or fresh)
        # ------------------------------------------------------------
        if use_existing_map:
            # Start with existing map
            nodes.append(
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    namespace=ns,      # only topics are namespaced
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params_path, slam_params, map_params],
                    remappings=[
                        ("/tf", f"{tf_prefix}/tf"),
                        ("/tf_static", f"{tf_prefix}/tf_static"),
                        ("/map", "map"),
                        ("/map_metadata", "map_metadata"),
                        ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                        ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
                    ]
                )
            )
        else:
            # Start fresh, no map
            nodes.append(
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    namespace=ns,
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params_path, slam_params],
                    remappings=[
                        ("/tf", f"{tf_prefix}/tf"),
                        ("/tf_static", f"{tf_prefix}/tf_static"),
                        ("/map", "map"),
                        ("/map_metadata", "map_metadata"),
                        ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                        ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
                    ]
                )
            )

        # ------------------------------------------------------------
        # 🟢 RViz2 Visualization
        # ------------------------------------------------------------
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=ns,
                name="rviz2",
                arguments=["-d", rviz_config_path],
                output="screen",
                condition=IfCondition(rviz_flag)
            )
        )

        return nodes

    return LaunchDescription(ARGUMENTS + [
        OpaqueFunction(function=launch_setup)
    ])