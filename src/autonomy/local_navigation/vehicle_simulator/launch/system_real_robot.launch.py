import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  map_dir = LaunchConfiguration('map_dir')
  slam_backend = LaunchConfiguration('slam_backend')
  local_planner_config = LaunchConfiguration('local_planner_config')
  vehicleLength = LaunchConfiguration('vehicleLength')
  vehicleWidth = LaunchConfiguration('vehicleWidth')
  maxSpeed = LaunchConfiguration('maxSpeed')
  autonomySpeed = LaunchConfiguration('autonomySpeed')
  maxAccel = LaunchConfiguration('maxAccel')
  maxDecel = LaunchConfiguration('maxDecel')

  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')
  declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.05', description='')
  declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.25', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  declare_map_dir = DeclareLaunchArgument('map_dir', default_value='', description='Path to the map PCD file')
  declare_slam_backend = DeclareLaunchArgument(
    'slam_backend',
    default_value='arise',
    description='LIO backend: "arise" (arise_slam_mid360) or "lightning" (lightning-lm).'
  )
  # Operator knobs sourced from robot.yaml via typego_bringup. Defaults match
  # the values that used to be hardcoded here, so standalone launch is unchanged.
  declare_local_planner_config = DeclareLaunchArgument('local_planner_config', default_value='dog', description='local_planner config profile')
  declare_vehicleLength = DeclareLaunchArgument('vehicleLength', default_value='0.7', description='Vehicle footprint length (m)')
  declare_vehicleWidth = DeclareLaunchArgument('vehicleWidth', default_value='0.3', description='Vehicle footprint width (m)')
  declare_maxSpeed = DeclareLaunchArgument('maxSpeed', default_value='1.375', description='Local-planner top linear speed (m/s)')
  declare_autonomySpeed = DeclareLaunchArgument('autonomySpeed', default_value='0.875', description='Local-planner autonomy cruise speed (m/s)')
  declare_maxAccel = DeclareLaunchArgument('maxAccel', default_value='2.0', description='Local-planner acceleration ramp (m/s^2)')
  declare_maxDecel = DeclareLaunchArgument('maxDecel', default_value='2.0', description='Local-planner deceleration ramp / braking (m/s^2)')

  use_arise = IfCondition(PythonExpression(["'", slam_backend, "' == 'arise'"]))
  use_lightning = IfCondition(PythonExpression(["'", slam_backend, "' == 'lightning'"]))
  
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'sensorOffsetX': sensorOffsetX,
      'sensorOffsetY': sensorOffsetY,
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
      'config': local_planner_config,
      'maxSpeed': maxSpeed,
      'autonomySpeed': autonomySpeed,
      'maxAccel': maxAccel,
      'maxDecel': maxDecel,
      'vehicleLength': vehicleLength,
      'vehicleWidth': vehicleWidth,
    }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    )
  )

  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': checkTerrainConn,
    }.items()
  )

  start_sensor_scan_generation = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
    )
  )

  start_arise_slam = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('arise_slam_mid360'), 'launch', 'arise_slam.launch.py')
    ),
    launch_arguments={
      'map_dir': map_dir,
    }.items(),
    condition=use_arise,
  )

  # `lightning` is an optional SLAM backend. Resolving its share directory
  # eagerly aborts description construction on an arise-only build that never
  # built it -- even though `condition` would have skipped the include anyway.
  # IncludeLaunchDescription loads its source lazily in execute(), and
  # Action.visit() gates execute() on the condition, so this sentinel path can
  # only surface when lightning is genuinely selected.
  try:
    lightning_launch_file = os.path.join(
      get_package_share_directory('lightning'), 'launch', 'lightning_lm.launch.py')
  except Exception:
    lightning_launch_file = '<lightning-package-not-built>/lightning_lm.launch.py'

  start_lightning_lm = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(lightning_launch_file),
    launch_arguments={
      'map_dir': map_dir,
    }.items(),
    condition=use_lightning,
  )

  start_visualization_tools = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
    ),
    launch_arguments={
      'world_name': world_name,
    }.items()
  )

  start_joy = Node(
    package='joy', 
    executable='joy_node',
    name='ps3_joy',
    output='screen',
    parameters=[{
                'dev': "/dev/input/js0",
                'deadzone': 0.12,
                'autorepeat_rate': 0.0,
  		}]
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_world_name)
  ld.add_action(declare_sensorOffsetX)
  ld.add_action(declare_sensorOffsetY)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_checkTerrainConn)
  ld.add_action(declare_map_dir)
  ld.add_action(declare_slam_backend)
  ld.add_action(declare_local_planner_config)
  ld.add_action(declare_vehicleLength)
  ld.add_action(declare_vehicleWidth)
  ld.add_action(declare_maxSpeed)
  ld.add_action(declare_autonomySpeed)
  ld.add_action(declare_maxAccel)
  ld.add_action(declare_maxDecel)

  ld.add_action(start_local_planner)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_sensor_scan_generation)
  ld.add_action(start_arise_slam)
  ld.add_action(start_lightning_lm)
  ld.add_action(start_visualization_tools)
  ld.add_action(start_joy)

  return ld
