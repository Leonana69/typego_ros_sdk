import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get robot namespace from ROBOT_ID environment variable
    robot_id = os.environ.get('ROBOT_ID', '')
    autonomy_type = os.environ.get('AUTONOMY_TYPE', 'base')
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
    
    video_client_node = Node(
        package='go2_sdk',
        executable='video_client_node',
        name='video_client_node',
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
    
    return LaunchDescription([
        tf_client_node,
        lidar_client_node,
        video_client_node,
        cmd_vel_controller_node,
    ])
