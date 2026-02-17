import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='Robot ID used to derive the robot namespace (e.g. 1 -> /robot1)'
    )
    video_type_arg = DeclareLaunchArgument(
        'video_type',
        default_value='binocular',
        description='Video type: "binocular" (Binocular) or "dual_d435i" (Dual D435i)'
    )
    autonomy_type_arg = DeclareLaunchArgument(
        'autonomy_type',
        default_value='base',
        description='Autonomy type: "base" (Twist) or other (TwistStamped + CustomMsg)'
    )

    def launch_setup(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)
        autonomy_type = LaunchConfiguration('autonomy_type').perform(context)
        video_type = LaunchConfiguration('video_type').perform(context)
        if robot_id:
            robot_name = f'robot{robot_id}'
            robot_ns = f'/{robot_name}'
        else:
            robot_ns = ''
        
        print(f'Go2🤖 Using robot namespace: {robot_ns if robot_ns else "<none>"}')

        if autonomy_type == 'base':
            # nav2 use Twist
            cmd_vel_type = 'Twist'
            xfer_format = 'None'
        else:
            # full autonomy use TwistStamped
            cmd_vel_type = 'TwistStamped'
            xfer_format = 'CustomMsg'

        # Create remapping list for tf topics
        tf_remappings = []
        if robot_ns:
            tf_remappings = [
                ('/tf', f'{robot_ns}/tf'),
                ('/tf_static', f'{robot_ns}/tf_static')
            ]

        # Define nodes
        tf_client_node = Node(
            package='go2_sdk',
            executable='tf_client_node',
            name='tf_client_node',
            remappings=tf_remappings,
            output='screen'
        )

        lidar_client_node = Node(
            package='go2_sdk',
            executable='lidar_client_node',
            name='lidar_client_node',
            parameters=[{
                'xfer_format': xfer_format,  # Use 'CustomMsg' or 'PointCloud2' or 'None'
                'publish_rate': 10.0,        # Hz
                'scan_timeout_ms': 200       # milliseconds
            }],
            remappings=tf_remappings,
            output='screen'
        )

        if video_type == 'binocular':
            video_client_node = Node(
                package='go2_sdk',
                executable='video_client_binocular_node',
                name='video_client_binocular_node',
                output='screen'
            )
        elif video_type == 'dual_d435i':
            video_client_node = Node(
                package='go2_sdk',
                executable='video_client_dual_d435i_node',
                name='video_client_dual_d435i_node',
                output='screen'
            )

        cmd_vel_controller_node = Node(
            package='go2_sdk',
            executable='cmd_vel_controller_node',
            name='cmd_vel_controller_node',
            parameters=[{
                'accept_cmd_vel': True,
                'cmd_vel_type': cmd_vel_type  # Use 'Twist' or 'TwistStamped'
            }],
            output='screen'
        )

        return [
            tf_client_node,
            lidar_client_node,
            video_client_node,
            cmd_vel_controller_node,
        ]

    return LaunchDescription([
        robot_id_arg,
        autonomy_type_arg,
        video_type_arg,
        OpaqueFunction(function=launch_setup),
    ])
