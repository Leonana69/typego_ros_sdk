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

### Python host dependencies

Even the Docker path needs a working host Python: `make docker_build` (and every other top-level target) first runs `typego-config env` on the host to render `/tmp/typego-runtime.env` from `robot.yaml`. That CLI imports **PyYAML** and **pydantic**, so both must be present on the interpreter `make` resolves to.

The repo ships a `pyproject.toml` + `uv.lock` covering every pip-installed Python dep (web gateway runtime + config CLI). Use `uv`:

```bash
uv sync                 # runtime: PyYAML, pydantic, fastapi, uvicorn, numpy, pillow, aiofiles, setuptools
uv sync --group dev     # + black, pre-commit, pytest, pytest-cov
```

Or pip, if you prefer:

```bash
pip install pyyaml pydantic   # minimum to run `make docker_build` / `make build`
```

ROS 2 Python bindings (`rclpy`, etc.) come from apt, not PyPI, and are not in the lockfile.

### Docker (production / reproducible deploy)

```bash
# Build and start the container (picks up AUTONOMY_TYPE from robot.yaml)
make docker_build

# Open a shell in the running container, then run ros2 launch inside
make docker_open

# RViz still runs on the host
make rviz
```

> **Host prerequisite:** `make docker_build` renders `/tmp/typego-runtime.env` on the host *before* invoking Docker, so the host Python must have **PyYAML** and **pydantic** installed (see above). Without them the build fails in the `$(ROBOT_ENV)` step, before any Docker commands run.

For day-to-day local work, prefer the native path + `make console` below — it skips the container build and gives you an interactive menu + log pane.

### Native (recommended for local development)

```bash
# One-time
source /opt/ros/humble/local_setup.bash
uv sync                      # host Python deps (PyYAML, pydantic, …); or: pip install pyyaml pydantic
pip install textual          # used by `make console` + `make rviz`
make setup                   # only needed for full autonomy (Sophus, OR-Tools)
make build                   # builds + renders /tmp/typego-runtime.env from robot.yaml

# Every time: launch the interactive console
make console
```

`make console` is a TUI that:

1. Asks for autonomy mode (`base` / `full`).
2. For full autonomy, asks which layer — `0` plain, `1` FAR route planner, `2` TARE exploration planner (matches `src/autonomy/real_robot.sh`'s `MODE`).
3. Lists the maps on disk filtered by mode (base: `.posegraph`; full: `.pcd`) plus **new map** to start from empty.
4. Spawns `ros2 launch typego_sdk typego_bringup.launch.py …` with the right args and streams its logs in the top pane.
5. In a new-map session, type `s` → enter a name → map is saved and the console returns to the menu; the new map is ready to pick on the next run.

Bottom-pane shortcuts while running: `q` quit, `m` back to menu, `s` save (new-map only).

### RViz

```bash
make rviz
```

Same TUI shape: asks for mode, for full mode asks which config (vehicle simulator / FAR / TARE), then streams rviz logs. `q` to quit, `m` to pick a different config.

### Headless / scripted launch

When you don't want the TUI (CI, remote session, scripted deploys):

```bash
# Defaults from robot.yaml
make launch

# One-off overrides
make launch ARGS="slam_map_name:=empty_map"
make launch ARGS="autonomy_type:=full full_mode:=2"    # full + TARE
```

Or call `ros2 launch` directly (remember to source the env first):

```bash
make robot_env && set -a && source /tmp/typego-runtime.env && set +a

ros2 launch typego_sdk typego_bringup.launch.py
ros2 launch typego_sdk typego_bringup.launch.py autonomy_type:=full full_mode:=1
ros2 launch typego_sdk typego_bringup.launch.py robot_id:=1 robot_type:=kami

# Apply a shipped profile overlay (see src/typego_config/config/profiles/)
TYPEGO_PROFILE=go2_indoor ros2 launch typego_sdk typego_bringup.launch.py

# Point at a deployment-specific (e.g. untracked) yaml
TYPEGO_CONFIG=/abs/path/my-robot.yaml ros2 launch typego_sdk typego_bringup.launch.py
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

The easiest path is through `make console` — pick **new map**, walk the robot, then type `s` and enter a name. The console handles the save + shutdown and drops you back at the menu so you can pick the new map on the next run.

Headless equivalent:

```bash
# Save the current SLAM map (picks base/full from robot.yaml's AUTONOMY_TYPE)
make save_map FILE=my-map

# Load a saved map
make launch ARGS="slam_map_name:=my-map"
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
| `make console` | Interactive TUI: pick mode + full-autonomy layer + map, stream logs, save-and-return |
| `make launch` | Headless launch with `robot.yaml` defaults; `ARGS="..."` for one-off overrides |
| `make rviz` | Interactive rviz launcher (pick layer: vehicle_simulator / FAR / TARE) |
| `make save_map FILE=<name>` | Save current SLAM map (mode picked from `AUTONOMY_TYPE`) |
| `make docker_build` | Build Docker image and start container |
| `make docker_start` | Start container from existing image |
| `make docker_open` | Open interactive shell in container |
| `make docker_stop` | Stop and remove container |
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
