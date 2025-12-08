from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import yaml
import os

ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for the robot (empty for no namespace)'
    ),
]

def modify_nav2_params(context):
    robot_namespace = context.launch_configurations['robot_namespace']
    
    # Create scan topic based on namespace
    if robot_namespace:
        scan_topic = f'/{robot_namespace}/scan'
    else:
        scan_topic = '/scan'
    
    # Load original parameters
    pkg_typego_sdk = get_package_share_directory('typego_sdk')
    original_config_path = os.path.join(pkg_typego_sdk, 'config', 'nav2_params.yaml')
    
    with open(original_config_path, 'r') as f:
        params = yaml.safe_load(f)
    
    # Modify the scan topic in the parameters
    params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = scan_topic
    params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = scan_topic

    # Save modified parameters to a temporary file
    temp_dir = '/tmp'
    os.makedirs(temp_dir, exist_ok=True)
    modified_config_path = os.path.join(temp_dir, f'nav2_params_modified_{os.getpid()}.yaml')
    
    with open(modified_config_path, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    
    print(f"Modified Nav2 parameters saved to: {modified_config_path}")
    print(f"Scan topic set to: {scan_topic}")
    
    return modified_config_path

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    def launch_setup(context, *args, **kwargs):
        modified_config_path = modify_nav2_params(context)
        
        # Wrap Nav2 launch in a group with namespace and TF remappings
        nav2_group = GroupAction([
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('nav2_bringup'),
                    '/launch/navigation_launch.py'
                ]),
                launch_arguments={
                    'namespace': robot_namespace,
                    'use_sim_time': 'false',
                    'params_file': modified_config_path,  # Use the modified config
                    'remappings': "[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/map', 'map')]"
                }.items()
            )
        ])
        
        return [nav2_group]

    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])