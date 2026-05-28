import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('pcd_grid_planner')
    default_config = os.path.join(pkg, 'config', 'default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicleLength',
            default_value='0.7',
            description='Vehicle footprint length, m.',
        ),
        DeclareLaunchArgument(
            'vehicleWidth',
            default_value='0.3',
            description='Vehicle footprint width, m.',
        ),
        LogInfo(msg='PCD grid planner: building 2D map live from terrain analysis.'),
        Node(
            package='pcd_grid_planner',
            executable='pcd_grid_planner',
            name='pcd_grid_planner',
            output='screen',
            parameters=[
                default_config,
                {
                    'vehicle_length': LaunchConfiguration('vehicleLength'),
                    'vehicle_width': LaunchConfiguration('vehicleWidth'),
                },
            ],
        ),
    ])
