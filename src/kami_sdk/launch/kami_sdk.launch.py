import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value=os.environ.get('ROBOT_ID', ''),
        description='Robot ID used to derive the robot namespace (e.g. 1 -> /robot1)'
    )

    def launch_setup(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)

        if robot_id:
            robot_name = f'robot{robot_id}'
            robot_ns = f'/{robot_name}'
        else:
            robot_ns = ''

        print(f'Kami🤖 Using robot namespace: {robot_ns if robot_ns else "<none>"}')

        # Create remapping list for tf topics
        tf_remappings = []
        if robot_ns:
            tf_remappings = [
                ('/tf', f'{robot_ns}/tf'),
                ('/tf_static', f'{robot_ns}/tf_static')
            ]

        # Define nodes
        tf_client_node = Node(
            package='kami_sdk',
            executable='tf_client_node',
            name='tf_client_node',
            remappings=tf_remappings,
            output='screen'
        )

        lidar_client_node = Node(
            package='kami_sdk',
            executable='lidar_client_node',
            name='lidar_client_node',
            remappings=tf_remappings,
            output='screen'
        )

        cmd_vel_controller_node = Node(
            package='kami_sdk',
            executable='cmd_vel_controller_node',
            name='cmd_vel_controller_node',
            parameters=[{
                'accept_cmd_vel': True,
                'cmd_vel_type': 'Twist'  # Use 'Twist' or 'TwistStamped'
            }],
            output='screen'
        )

        return [
            tf_client_node,
            lidar_client_node,
            cmd_vel_controller_node,
        ]

    return LaunchDescription([
        robot_id_arg,
        OpaqueFunction(function=launch_setup),
    ])
