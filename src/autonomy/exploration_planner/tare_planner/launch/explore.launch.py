import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_tare_node(context, scenario):
    scenario_str = str(scenario.perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        # namespace='sensor_coverage_planner',
        parameters=[
            get_package_share_directory('tare_planner')+'/' + scenario_str + '.yaml',
            {'use_sim_time': use_sim_time},
        ])
    return [tare_planner_node]


def generate_launch_description():
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    workspace_root = None
    if ament_prefix_path:
        first_prefix = ament_prefix_path.split(':')[0]
        workspace_root = os.path.dirname(first_prefix)
    if workspace_root is None or workspace_root == '':
        workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))

    ortools_lib_dir = os.path.join(
        workspace_root,
        'src',
        'autonomy',
        'exploration_planner',
        'tare_planner',
        'or-tools',
        'lib',
    )
    existing_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    if existing_ld_library_path:
        ld_library_path_value = f"{ortools_lib_dir}:{existing_ld_library_path}"
    else:
        ld_library_path_value = ortools_lib_dir
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    scenario = LaunchConfiguration('scenario')

    declare_scenario = DeclareLaunchArgument(
        'scenario',
        default_value='outdoor',
        description='description for scenario argument')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_scenario,
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld_library_path_value),
        OpaqueFunction(function=launch_tare_node, args=[scenario])
    ])
