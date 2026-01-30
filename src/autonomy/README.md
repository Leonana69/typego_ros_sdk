# Autonomy
This package is copied from [Jizhang's Project](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform). It requires a Livox Mid-360 lidar to work.

## Workflow (system_real_robot.launch.py)
This launch file orchestrates the real-robot pipeline. It wires up sensing, mapping, terrain analysis, planning, visualization, and manual control input. The high-level data flow is:
sensor input -> scan generation -> SLAM/pose -> terrain analysis -> local planning -> control output and visualization.

### Inputs (data and configuration)
- **Lidar point cloud**: Provided by the real robot. The MID-360 driver is intentionally not started in this launch file.
- **Joystick**: `joy_node` reads `/dev/input/js0` for manual input and safety override.
- **Launch arguments**: `world_name`, sensor/camera offsets, initial `vehicleX/Y`, and `checkTerrainConn` tune downstream modules.

### Mapping and state estimation
- **ARISE SLAM** (`arise_slam_mid360`): Consumes lidar data to produce a map and robot pose estimates for downstream modules.
- **Sensor scan generation** (`sensor_scan_generation`): Converts raw lidar into a scan format used by the terrain modules and planner.

### Terrain understanding
- **Terrain analysis** (`terrain_analysis` + `terrain_analysis_ext`): Builds traversability information, optionally checking connectivity via `checkTerrainConn`. These layers inform planning constraints.

### Planning and control
- **Local planner** (`local_planner`): Computes feasible local trajectories using terrain outputs and robot state. The `realRobot` flag is set to true and offsets are forwarded so the planner operates in the real robot frame.

### Outputs (control, visualization)
- **Control commands**: The planner generates drive commands consumed by the robot controller (outside this launch file).
- **Visualization tools** (`visualization_tools`): Publishes overlays and debugging views for maps, paths, and robot state.

## Data Flow (system_real_robot_with_exploration_planner.launch.py)
This launch file adds the exploration planner to the real-robot pipeline. The detailed topic-level data flow below is based on the default configurations in:
- `arise_slam_mid360/config/livox_mid360.yaml`
- `tare_planner/config/indoor_small.yaml` (default `exploration_planner_config`)

### 1) Data sources
- **Livox Mid-360 driver** (external to this launch file):
  - Common raw topics are `/livox/lidar` (PointCloud2 or Livox CustomMsg) and `/livox/imu` (Imu).
  - `arise_slam_mid360` expects `/lidar/scan` and `/imu/data` by default, so remap if needed.
- **Joystick** (`joy_node`):
  - Publishes `/joy` for manual control and safety overrides.
- **Exploration trigger** (optional):
  - `/start_exploration` can be used to start exploration if `kAutoStart` is false.

### 2) SLAM and state estimation (arise_slam_mid360)
ProjectName is empty in `livox_mid360.yaml`, so SLAM topics are rooted at `/`.
- **feature_extraction_node**
  - Subscribes: `/lidar/scan` (Livox CustomMsg) and `/imu/data` (Imu)
  - Publishes: `/feature_info` (LaserFeature), `/planner_points`, `/edge_points`
- **laser_mapping_node**
  - Subscribes: `/feature_info`, `/integrated_to_init5`
  - Publishes: `/registered_scan` (PointCloud2), `/laser_odometry` (Odometry), `/laser_cloud_map`
- **imu_preintegration_node**
  - Subscribes: `/imu/data`, `/laser_odometry`
  - Publishes: `/state_estimation` (Odometry), `/state_estimation_health`, `/imuodom_path`

### 3) Sensor processing (sensor_scan_generation)
- **sensorScanGeneration**
  - Subscribes: `/state_estimation`, `/registered_scan`
  - Publishes: `/sensor_scan` (PointCloud2 in `sensor_at_scan` frame),
    `/state_estimation_at_scan` (Odometry synced to scan)
  - Broadcasts TF: `map -> sensor_at_scan`

### 4) Terrain analysis
- **terrain_analysis**
  - Subscribes: `/state_estimation`, `/registered_scan`, `/joy`, `/map_clearing`
  - Publishes: `/terrain_map`
- **terrain_analysis_ext**
  - Subscribes: `/state_estimation`, `/registered_scan`, `/terrain_map`, `/joy`, `/cloud_clearing`
  - Publishes: `/terrain_map_ext`

### 5) Exploration planning (tare_planner)
Configured by `exploration_planner_config` (default `indoor_small`):
- **tare_planner_node**
  - Subscribes: `/state_estimation_at_scan`, `/registered_scan`, `/terrain_map`, `/terrain_map_ext`,
    `/joy`, `/start_exploration`, `/navigation_boundary`,
    `/sensor_coverage_planner/coverage_boundary`, `/sensor_coverage_planner/nogo_boundary`,
    `/reset_waypoint`
  - Publishes: `/way_point`, `/runtime`, `runtime_breakdown`, `exploration_finish`
- **navigationBoundary** (only if `use_boundary` is true)
  - Publishes: `/navigation_boundary`

### 6) Local planning and path following (local_planner)
- **localPlanner**
  - Subscribes: `/state_estimation`, `/registered_scan`, `/terrain_map`, `/joy`, `/way_point`,
    `/speed`, `/navigation_boundary`, `/added_obstacles`, `/check_obstacle`
  - Publishes: `/path`, `/slow_down`, `/free_paths`
- **pathFollower**
  - Subscribes: `/state_estimation`, `/path`, `/joy`, `/speed`, `/slow_down`, `/stop`
  - Publishes: `/cmd_vel` (TwistStamped) -> robot motor controller

### Summary diagram
Livox LiDAR + IMU
  -> /lidar/scan, /imu/data
  -> arise_slam_mid360
       -> /registered_scan, /state_estimation
       -> sensor_scan_generation -> /sensor_scan, /state_estimation_at_scan
       -> terrain_analysis -> /terrain_map -> terrain_analysis_ext -> /terrain_map_ext
       -> tare_planner -> /way_point
       -> localPlanner -> /path -> pathFollower -> /cmd_vel