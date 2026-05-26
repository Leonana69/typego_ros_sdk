# Autonomy
This package is copied from [Jizhang's Project](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform). It requires a Livox Mid-360 lidar to work.

## Workflow (system_real_robot.launch.py)
This launch file orchestrates the real-robot pipeline. It wires up sensing, mapping, terrain analysis, planning, visualization, and manual control input. The high-level data flow is:
sensor input -> scan generation -> SLAM/pose -> terrain analysis -> local planning -> control output and visualization.

### Inputs (data and configuration)
- **Lidar point cloud**: Provided by the real robot. The MID-360 driver is intentionally not started in this launch file.
- **Joystick**: `joy_node` reads `/dev/input/js0` for manual input and safety override.
- **Launch arguments**: `world_name`, sensor/camera offsets, initial `vehicleX/Y`, `checkTerrainConn`, and `slam_backend` tune downstream modules.

### Mapping and state estimation
The LIO (lidar-inertial odometry) backend is selectable via the `slam_backend` launch argument (default `arise`). Both backends publish `/state_estimation` and `/registered_scan`, so downstream modules are unaffected by the choice.
- **ARISE SLAM** (`arise_slam_mid360`, `slam_backend:=arise`): Feature-based LIO in the LOAM lineage. Consumes lidar data to produce a map and robot pose estimates for downstream modules.
- **Lightning-LM** (`lightning`, `slam_backend:=lightning`): Direct LIO (FasterLIO over an iVox neighbourhood index) with built-in real-time loop closure — an alternative backend that does not rely on extractable geometric features. See the repository root `README.md` for the full backend trade-offs.
- **Sensor scan generation** (`sensor_scan_generation`): Converts raw lidar into a scan format used by the terrain modules and planner.

### Terrain understanding
- **Terrain analysis** (`terrain_analysis` + `terrain_analysis_ext`): Builds traversability information, optionally checking connectivity via `checkTerrainConn`. These layers inform planning constraints.

### Planning and control
- **Local planner** (`local_planner`): Computes feasible local trajectories using terrain outputs and robot state. The `realRobot` flag is set to true and offsets are forwarded so the planner operates in the real robot frame.

### Outputs (control, visualization)
- **Control commands**: The planner generates drive commands consumed by the robot controller (outside this launch file).
- **Visualization tools** (`visualization_tools`): Publishes overlays and debugging views for maps, paths, and robot state.

### Saving a global map for re-use (FAR planner, etc.)
`visualization_tools` exposes `/save_map` (`visualization_tools/srv/SaveMap`), which writes a one-shot snapshot of the accumulated `/registered_scan` cloud as a binary PCD in the `map` frame and, when FAR is running, asks `graph_decoder` to dump the current visibility graph next to the PCD. `scripts/save_map.py` uses this service for full-autonomy saves, so `make save_map FILE=<name>` stores both the SLAM localization map and the FAR graph when the route planner is active.

```
ros2 service call /save_map visualization_tools/srv/SaveMap \
  "{file_path: '/tmp/floor_scan', voxel_size: 0.1, save_vgraph: true}"
```

Outputs (assuming everything is connected):
- `/tmp/floor_scan.pcd` — global point cloud map, suitable as FAR planner's `/scan_cloud` (`/terrain_map`) input.
- `/tmp/floor_scan_gravity.txt` — IMU gravity roll/pitch offsets, when received.
- `/tmp/floor_scan.vgh` — FAR planner visibility graph dumped by `graph_decoder`. Each row encodes one node's id, free_direct, position, surface dirs, the `is_covered` / `is_frontier` / `is_navpoint` / `is_boundary` flags, and four `|`-separated neighbor lists (connect / poly / contour / trajectory). The **freespace_vgraph** is implicit: it is the poly-edge subset between `is_covered` nodes, so one `.vgh` save captures both the full vgraph and the freespace vgraph.

The vgraph trigger publishes the destination path on `/save_file_dir` (consumed by `graph_decoder` from `far_planner.launch`). If FAR planner isn't running, only the PCD is written and the `vgraph_path` field in the response stays empty. Pass `save_vgraph: false` to skip the vgraph entirely; pass `voxel_size: 0.0` to fall back to the launch-time default; leave `file_path` empty to use the launch-time `exploredAreaFile` path with a timestamp suffix.

To reload on a later route-planner run, choose the saved map name as usual. `typego_bringup.launch.py` passes `Map-<name>/<name>.pcd` to SLAM for localization and passes `Map-<name>/<name>.vgh` to `farVgraphLoader`, which publishes `/read_file_dir` after FAR and `graph_decoder` are up. If the `.vgh` file is absent, the loader logs a warning and FAR falls back to building a fresh graph from live terrain.

The legacy `/save_explored_areas` service is preserved for backward compatibility but does not re-filter the snapshot and does not trigger the vgraph save.

## Data Flow (system_real_robot_with_exploration_planner.launch.py)
This launch file adds the exploration planner to the real-robot pipeline. The detailed topic-level data flow below is based on the default configurations in:
- `arise_slam_mid360/config/livox_mid360.yaml`
- `tare_planner/config/indoor_small.yaml` (default `exploration_planner_config`)

### 1) Data sources
- **Livox Mid-360 driver** (external to this launch file):
  - Common raw topics are `/livox/lidar` (PointCloud2 or Livox CustomMsg) and `/livox/imu` (Imu).
  - `arise_slam_mid360` expects `/livox/lidar` and `/livox/imu` by default, so remap if needed.
- **Joystick** (`joy_node`):
  - Publishes `/joy` for manual control and safety overrides.
- **Exploration trigger** (optional):
  - `/start_exploration` can be used to start exploration if `kAutoStart` is false.

### 2) SLAM and state estimation (arise_slam_mid360)
ProjectName is empty in `livox_mid360.yaml`, so SLAM topics are rooted at `/`.
- **feature_extraction_node**
  - Subscribes: `/livox/lidar` (Livox CustomMsg) and `/livox/imu` (Imu)
  - Publishes: `/feature_info` (LaserFeature), `/planner_points`, `/edge_points`
- **laser_mapping_node**
  - Subscribes: `/feature_info`, `/integrated_to_init5`
  - Publishes: `/registered_scan` (PointCloud2), `/laser_odometry` (Odometry), `/laser_cloud_map`
- **imu_preintegration_node**
  - Subscribes: `/livox/imu`, `/laser_odometry`
  - Publishes: `/state_estimation` (Odometry), `/state_estimation_health`, `/imuodom_path`

When `slam_backend:=lightning`, the three nodes above are replaced by a single `lightning` node (`run_slam_online`) that subscribes to `/livox/lidar` (PointCloud2) and `/livox/imu`, runs loop closure internally, and publishes the same `/registered_scan` and `/state_estimation` consumed downstream.

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
  -> /livox/lidar, /livox/imu
  -> SLAM backend (arise_slam_mid360 or lightning)
       -> /registered_scan, /state_estimation
       -> sensor_scan_generation -> /sensor_scan, /state_estimation_at_scan
       -> terrain_analysis -> /terrain_map -> terrain_analysis_ext -> /terrain_map_ext
       -> tare_planner -> /way_point
       -> localPlanner -> /path -> pathFollower -> /cmd_vel
