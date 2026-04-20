# TypeGo ROS SDK

A ROS 2 (Humble) SDK providing standardized interfaces and a full autonomy stack for TypeGo robots. Supports the **Unitree Go2** and **Kami** platforms with SLAM, navigation, route planning, and autonomous exploration.

## Architecture

```
typego_bringup.launch.py
├── Robot SDK (go2_sdk / kami_sdk)
│   ├── Lidar driver         → /scan
│   ├── Camera driver         → /camera/color/image_raw, /camera/depth/image_raw
│   ├── TF publisher          → odom → base_link → lidar_link
│   └── Command handler       → /cmd_vel
│
├── Base Autonomy (autonomy_type:=base)
│   ├── SLAM Toolbox          → /map
│   └── Nav2 (MPPI controller)
│
└── Full Autonomy (autonomy_type:=full)
    ├── ARISE SLAM (LOAM-based, Livox Mid360)
    ├── Terrain Analysis
    ├── Local Planner
    ├── FAR Route Planner / TARE Exploration Planner
    └── Vehicle Simulator (orchestrator)
```

## Standard Interfaces

All robot SDKs expose these topics. If `ROBOT_ID` is set, topics are namespaced under `/robot${ROBOT_ID}/`.

| Data Type | Topic | Frame ID |
|-----------|-------|----------|
| RGB Image | `/camera/color/image_raw` | - |
| Depth Image | `/camera/depth/image_raw` | - |
| 2D Lidar | `/scan` | `lidar_link` |
| Transforms | `/tf` | `base_link` |

**TF tree:** `odom` -> `base_link` -> `lidar_link`

## Packages

### Core

| Package | Description |
|---------|-------------|
| `typego_interface` | Custom ROS 2 message and service definitions |
| `typego_sdk` | Main SDK: launch orchestration, SLAM/Nav2 integration, waypoint service |
| `go2_sdk` | Unitree Go2 driver (lidar, camera, TF, commands) |
| `kami_sdk` | Kami robot driver (lidar, TF, commands) |

### Autonomy

| Package | Description |
|---------|-------------|
| `arise_slam_mid360` | LOAM-based SLAM for Livox Mid360 lidar |
| `terrain_analysis` / `terrain_analysis_ext` | Ground/slope/traversability classification |
| `local_planner` | Trajectory selection with obstacle avoidance |
| `far_planner` | FAR dynamic route planner (indoor/outdoor configs) |
| `tare_planner` | TARE frontier-based autonomous exploration |
| `vehicle_simulator` | Full autonomy orchestrator |

### Utilities

| Package | Description |
|---------|-------------|
| `waypoint_rviz_plugin` / `goalpoint_rviz_plugin` | RViz plugins for setting waypoints and goals |
| `teleop_rviz_plugin_plus` | RViz teleoperation widget |
| `teleop_keyboard_controller` | Keyboard teleoperation |
| `domain_bridge` | DDS domain bridging for multi-robot setups |
| `ROS-TCP-Endpoint` | TCP bridge for external clients (e.g. Unity) |

## Quick Start

### Configuration

One file owns the deployment: `src/typego_config/config/robot.yaml`. Edit it, relaunch the stack. `docker/.env` is no longer read.

```yaml
robot:
  id: ""
  type: go2           # go2 | kami
  name: robot-alpha

autonomy:
  type: base          # base | full

network:
  ros_domain_id: 1
  robot_ip: 192.168.0.163
  edge_service: { ip: 192.168.0.104, port: 50049 }

map:
  slam_map_name: 4th-partial

web_gateway:
  port: 8080
```

Validate without building:

```bash
make config_validate
```

### Docker (Recommended)

```bash
# Build and start the container (picks up AUTONOMY_TYPE from robot.yaml)
make docker_build

# Open a shell in the running container
make docker_open

# View in RViz (run on host)
make rviz
```

### Native Build

```bash
# Source ROS
source /opt/ros/humble/local_setup.bash

# For full autonomy, install SLAM dependencies first
make setup

# Build (also renders /tmp/typego-runtime.env from robot.yaml)
make build
```

### Launch

Every default is read from `robot.yaml`, so the no-args command Just Works:

```bash
# Source middleware env (ROS_DOMAIN_ID / RMW must be set before rclcpp starts)
make robot_env && set -a && source /tmp/typego-runtime.env && set +a

# Launch — uses robot.yaml for robot_type, autonomy_type, slam_map_name, …
ros2 launch typego_sdk typego_bringup.launch.py
```

CLI args still override any single field for one-off runs:

```bash
# Swap the map without editing robot.yaml
ros2 launch typego_sdk typego_bringup.launch.py slam_map_name:=empty_map

# Apply a shipped profile overlay (see src/typego_config/config/profiles/)
TYPEGO_PROFILE=go2_indoor ros2 launch typego_sdk typego_bringup.launch.py

# Point at a deployment-specific (e.g. untracked) yaml
TYPEGO_CONFIG=/abs/path/my-robot.yaml ros2 launch typego_sdk typego_bringup.launch.py

# Multi-robot
ros2 launch typego_sdk typego_bringup.launch.py robot_id:=1
ros2 launch typego_sdk typego_bringup.launch.py robot_id:=2 robot_type:=kami
```

Once up:

- **Web HMI:** <http://localhost:8080> — map, pose, goal form, patrol, event feed, bag download, live `robot name · type · autonomy` pill.
- **Live config introspection:** `ros2 service call /typego_config/get_config std_srvs/srv/Trigger` or `curl http://localhost:8080/api/config`.
- **Dry-run / schema / env dumps:**

  ```bash
  ros2 run typego_config typego-config dryrun      # resolved parameter tree
  ros2 run typego_config typego-config schema      # JSON schema
  ros2 run typego_config typego-config env         # KEY=VALUE lines
  ```

## Map Management

Maps are stored in `src/typego_sdk/resource/Map-<name>/` and contain SLAM data files, a pose graph, waypoints, and an initial pose.

```bash
# Save map (base autonomy, from host)
make save_map_base_autonomy FILE=my-map

# Save map (full autonomy)
make save_map_full_autonomy FILE=my-map

# Load a saved map at launch
ros2 launch typego_sdk typego_bringup.launch.py slam_map_name:=my-map ...
```

## Navigation

Send goals via any of these methods:

- **RViz:** Use the goal pose or waypoint RViz plugins
- **Nav2 action:** `navigate_to_pose` action server
- **Waypoint service:** Custom waypoints service node
- **Patrol script:** `python3 scripts/patrol.py --map_name <name> --patrol_list 0,1,2`
- **Keyboard:** `teleop_keyboard_controller`

## Makefile Reference

| Target | Description |
|--------|-------------|
| `make build` | Build all packages (skips autonomy packages when `AUTONOMY_TYPE=base`) |
| `make config_validate` | Validate `src/typego_config/config/robot.yaml` against the pydantic schema |
| `make robot_env` | Render `/tmp/typego-runtime.env` from `robot.yaml` for shell/Docker consumption |
| `make setup` | Install SLAM dependencies (Sophus, gtsam) and OR-Tools for full autonomy |
| `make docker_build` | Build Docker image and start container |
| `make docker_start` | Start container from existing image |
| `make docker_open` | Open interactive shell in container |
| `make docker_stop` | Stop and remove container |
| `make rviz` | Launch RViz with appropriate config and namespace remapping |
| `make save_map_base_autonomy FILE=<name>` | Save current SLAM map (base mode) |
| `make save_map_full_autonomy FILE=<name>` | Save current SLAM map (full mode) |
| `make iox_reset` | Clean up iceoryx shared memory segments |

## Key Dependencies

- **ROS 2 Humble** with `slam_toolbox`, `navigation2`, `pcl_ros`, `rviz2`
- **CycloneDDS** (configured as default RMW)
- **GStreamer** for camera streaming
- **PCL**, **OpenCV**, **Eigen** for perception
- **GTSAM**, **Sophus** for SLAM factor graph optimization (full autonomy)
- **OR-Tools** for exploration planning (full autonomy, auto-downloads arm64 variant)

## Project Structure

```
src/
├── typego_interface/          # Custom messages/services
├── typego_sdk/                # Main SDK, launch files, configs, maps
├── go2_sdk/                   # Unitree Go2 robot driver
├── kami_sdk/                  # Kami robot driver
└── autonomy/
    ├── base_autonomy/         # Terrain analysis, local planner, vehicle simulator
    ├── slam/                  # ARISE SLAM + dependencies (Sophus, gtsam)
    ├── route_planner/         # FAR planner, boundary/graph handlers
    ├── exploration_planner/   # TARE planner + OR-Tools
    └── utilities/             # RViz plugins, teleop, domain bridge
scripts/                       # Patrol, goal sending, sensor test utilities
docker/                        # Dockerfile and .env configuration
```
