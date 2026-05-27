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
        description='Autonomy type: "base" (Twist) or "full" (TwistStamped + CustomMsg)'
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='',
        description='Robot LAN IP. If empty, nodes fall back to $ROBOT_IP.'
    )
    cmd_vel_max_linear_arg = DeclareLaunchArgument(
        'cmd_vel_max_linear', default_value='0.8',
        description='cmd_vel linear clamp, m/s (robot.yaml motion.cmd_vel_max_linear)'
    )
    cmd_vel_max_angular_arg = DeclareLaunchArgument(
        'cmd_vel_max_angular', default_value='1.0',
        description='cmd_vel angular clamp, rad/s (robot.yaml motion.cmd_vel_max_angular)'
    )
    cmd_vel_min_angular_arg = DeclareLaunchArgument(
        'cmd_vel_min_angular', default_value='0.2',
        description='cmd_vel angular deadband floor, rad/s (robot.yaml motion.cmd_vel_min_angular)'
    )

    def launch_setup(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)
        autonomy_type = LaunchConfiguration('autonomy_type').perform(context)
        video_type = LaunchConfiguration('video_type').perform(context)
        robot_ip = LaunchConfiguration('robot_ip').perform(context)
        cmd_vel_max_linear = float(LaunchConfiguration('cmd_vel_max_linear').perform(context))
        cmd_vel_max_angular = float(LaunchConfiguration('cmd_vel_max_angular').perform(context))
        cmd_vel_min_angular = float(LaunchConfiguration('cmd_vel_min_angular').perform(context))
        if robot_id:
            robot_name = f'robot{robot_id}'
            robot_ns = f'/{robot_name}'
        else:
            robot_ns = ''
        
        print(f'Go2🤖 Using robot namespace: {robot_ns if robot_ns else "<none>"}')

        # Create remapping list for tf topics
        tf_remappings = []
        if robot_ns:
            tf_remappings = [
                ('/tf', f'{robot_ns}/tf'),
                ('/tf_static', f'{robot_ns}/tf_static')
            ]

        # Define nodes
        nodes = []
        if autonomy_type == 'base':
            xfer_format = 'PointCloud2'
            tf_client_node = Node(
                package='go2_sdk',
                executable='tf_client_node',
                name='tf_client_node',
                parameters=[{
                    'robot_ip': robot_ip,
                }],
                remappings=tf_remappings,
                output='screen'
            )
            nodes.append(tf_client_node)
        elif autonomy_type == 'full':
            xfer_format = 'CustomMsg'
        else:
            raise ValueError(f'Invalid autonomy type: {autonomy_type}')
        

        lidar_client_node = Node(
            package='go2_sdk',
            executable='lidar_client_node',
            name='lidar_client_node',
            parameters=[{
                'robot_ip': robot_ip,
                'xfer_format': xfer_format,  # Use 'CustomMsg' or 'PointCloud2' or 'None'
                'publish_rate': 10.0,        # Hz
                'scan_timeout_ms': 200       # milliseconds
            }],
            remappings=tf_remappings,
            output='screen'
        )
        nodes.append(lidar_client_node)

        if video_type == 'binocular' or video_type == 'dual_d435i':
            video_client_node = Node(
                package='go2_sdk',
                executable='video_client_' + video_type + '_node',
                name='video_client_' + video_type + '_node',
                output='screen'
            )
            nodes.append(video_client_node)
        else:
            raise ValueError(f'Invalid video type: {video_type}')

        cmd_vel_controller_node = Node(
            package='go2_sdk',
            executable='cmd_vel_controller_node',
            name='cmd_vel_controller_node',
            parameters=[{
                'robot_ip': robot_ip,
                'accept_cmd_vel': True,
                'cmd_vel_type': 'Twist',  # Use 'Twist' or 'TwistStamped'
                'cmd_vel_max_linear': cmd_vel_max_linear,
                'cmd_vel_max_angular': cmd_vel_max_angular,
                'cmd_vel_min_angular': cmd_vel_min_angular,
            }],
            output='screen'
        )
        nodes.append(cmd_vel_controller_node)

        return nodes

    return LaunchDescription([
        robot_id_arg,
        autonomy_type_arg,
        video_type_arg,
        robot_ip_arg,
        cmd_vel_max_linear_arg,
        cmd_vel_max_angular_arg,
        cmd_vel_min_angular_arg,
        OpaqueFunction(function=launch_setup),
    ])
