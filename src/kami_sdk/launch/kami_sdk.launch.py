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
    autonomy_type_arg = DeclareLaunchArgument(
        'autonomy_type',
        default_value='base',
        description='Autonomy type: "base" (Twist) or "full" (TwistStamped + CustomMsg)'
    )

    def launch_setup(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)
        autonomy_type = LaunchConfiguration('autonomy_type').perform(context)
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
        nodes = []
        if autonomy_type == 'base':
            cmd_vel_type = 'Twist'
            xfer_format = 'None'
            tf_client_node = Node(
                package='kami_sdk',
                executable='tf_client_node',
                name='tf_client_node',
                remappings=tf_remappings,
                output='screen'
            )
            nodes.append(tf_client_node)
        elif autonomy_type == 'full':
            cmd_vel_type = 'TwistStamped'
            xfer_format = 'CustomMsg'
        else:
            raise ValueError(f'Invalid autonomy type: {autonomy_type}')

        lidar_client_node = Node(
            package='kami_sdk',
            executable='lidar_client_node',
            name='lidar_client_node',
            parameters=[{
                'xfer_format': xfer_format,  # Use 'CustomMsg' or 'PointCloud2' or 'None'
                'publish_scan': False,
            }],
            remappings=tf_remappings,
            output='screen'
        )
        nodes.append(lidar_client_node)

        cmd_vel_controller_node = Node(
            package='kami_sdk',
            executable='cmd_vel_controller_node',
            name='cmd_vel_controller_node',
            parameters=[{
                'accept_cmd_vel': True,
                'cmd_vel_type': cmd_vel_type  # Use 'Twist' or 'TwistStamped'
            }],
            output='screen'
        )
        nodes.append(cmd_vel_controller_node)

        return nodes

    return LaunchDescription([
        robot_id_arg,
        autonomy_type_arg,
        OpaqueFunction(function=launch_setup),
    ])
