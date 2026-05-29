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
    ├── LIO/SLAM (slam_backend:=arise | lightning)
    │   ├── ARISE SLAM (LOAM-based, Livox Mid360)
    │   └── Lightning-LM (FasterLIO + loop closure, Livox Mid360)
    ├── Terrain Analysis
    ├── Local Planner
    ├── FAR Route Planner / TARE Exploration Planner
    └── Vehicle Simulator (orchestrator)
```

Both LIO backends honour the same downstream contract — odometry on `/state_estimation`, dense undistorted cloud on `/registered_scan`, `map → sensor` TF — so terrain analysis, local planner, FAR, and TARE see identical inputs regardless of which backend is selected.

### LIO Backends

Both backends are LIO (lidar–inertial odometry) for the Livox Mid-360 with built-in loop closure. They differ in algorithm class and engineering trade-offs:

**ARISE SLAM** — *feature-based* LIO in the LOAM lineage. Three nodes (`feature_extraction_node` → `laser_mapping_node` ← `imu_preintegration_node`); GTSAM/ISAM2 factor graph; PCL-ICP loop closure. Publishes `/state_estimation` at IMU rate (~50 Hz).

- **Pick when:** the scene has rich geometric structure (corners, edges, planes); the controller needs IMU-rate state feedback (near-zero output lag).
- **Avoid when:** open or feature-poor environments — empty plazas, long blank corridors, parking lots, tunnel-like halls — where edge/planar feature extraction degenerates.
- **Cost:** lower CPU (single-feature pipeline parallelised across 3 procs) but higher RSS than Lightning.

**Lightning-LM** — *direct* LIO using FasterLIO over an iVox neighbourhood index (no explicit feature extraction; registers against the full undistorted cloud). Single `run_slam_online` process; built-in real-time loop closure. Publishes `/state_estimation` at lidar rate (~10 Hz).

- **Pick when:** feature-poor / unstructured scenes — the direct registration uses the full cloud and does not rely on extractable features. Also when accuracy and low RAM matter more than odometry latency.
- **Avoid when:** the controller demands IMU-rate state feedback, or CPU is the bottleneck (roughly 2–3× the CPU footprint of ARISE on the same bag).
- **Cost:** ~10× higher output lag than ARISE; higher CPU; lower RSS.

A comparison harness (`scripts/slam_bench/run_bench.sh`) replays a bag through each backend and emits a side-by-side `summary.md` covering output rate, lag, CPU/RSS, and closed-loop drift. Runs land in `logs/slam_bench/<timestamp>/`.

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
| `arise_slam_mid360` | LOAM-based SLAM for Livox Mid360 lidar (default `slam_backend`) |
| `lightning` (lightning-lm) | FasterLIO + real-time loop closure for Livox Mid360; switchable via `slam_backend:=lightning`. Consumes `/livox/lidar` + `/livox/imu` directly and publishes `/state_estimation` + `/registered_scan` to match ARISE's contract. |
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
  slam_backend: arise # arise | lightning  (only used when type=full)

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

For a key-by-key map of every `robot.yaml` setting — what it controls, the launch
arg / ROS param / file it drives, and when it takes effect — see
[`doc/CONFIG.md`](doc/CONFIG.md).

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
2. For full autonomy, asks which layer — `0` plain, `1` FAR route planner, `2` PCD roadmap planner, `3` TARE exploration planner.
3. Lists the maps on disk filtered by mode (base: `.posegraph`; full: `.pcd`) plus **new map** to start from empty. The PCD roadmap option only allows saved PCD maps because it plans from an existing map.
4. Spawns `ros2 launch typego_sdk typego_bringup.launch.py …` with the right args and streams its logs in the top pane.
5. In a new-map session, type `s` → enter a name → map is saved and the console returns to the menu; the new map is ready to pick on the next run.

Bottom-pane shortcuts while running: `q` quit, `m` back to menu, `s` save (new-map only).

### RViz

```bash
make rviz
```

Same TUI shape: asks for mode, for full mode asks which config (vehicle simulator / FAR / TARE / PCD roadmap), then streams rviz logs. `q` to quit, `m` to pick a different config.

### Headless / scripted launch

When you don't want the TUI (CI, remote session, scripted deploys):

```bash
# Defaults from robot.yaml
make launch

# One-off overrides
make launch ARGS="slam_map_name:=empty_map"
make launch ARGS="autonomy_type:=full full_mode:=2"    # full + TARE
make launch ARGS="autonomy_type:=full slam_backend:=lightning"  # swap ARISE → lightning-lm
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
- **Waypoints:** Place-graph waypoints node (`place_graph_waypoints_node`)
- **Patrol script:** `python3 scripts/patrol.py --map_name <name> --patrol_list 0,1,2`
- **Keyboard:** `teleop_keyboard_controller`

### Place-Graph Waypoints

`place_graph_waypoints_node` builds a geometry-only **place graph** from the live occupancy
grid — segmenting it into rooms, corridors, open areas, portals, junctions, and frontier
regions — and emits LLM-selectable navigation targets per place. Waypoints are *fixed
anchors*: once minted, a waypoint keeps its integer `id` and its frozen pose across map
refreshes until that pose becomes invalid, so a patrol index or an object attached to
`(id, x, y)` stays stable as the map grows.

Per place it generates: an `entrance` just inside each adjacent doorway; interval `coverage`
waypoints (every `waypoint_spacing_m`, default 2.0 m) for corridors and large rooms; a single
`center` for small rooms; a `doorway`/`transition`/`junction` waypoint at each connector; and a
volatile `frontier` target in unexplored regions. Each `typego_interface/msg/WayPoint` carries
`place_id` and `waypoint_role` alongside `yaw`, `semantic_context`, `clearance_m`, and
`source`. The full place graph is published on the latched `place_graph` topic and exposed via
the gateway's `GET /api/places`; waypoints and the graph persist together in `waypoints.json`
(schema v3). Semantic (VLM/LLM) labeling is deferred — labels here are geometry-derived.

Useful `place_graph_waypoints_node` parameters include `waypoint_spacing_m`,
`coverage_min_area_m2`, `entrance_offset_m`, `min_anchor_clearance_m`, `min_refresh_interval_s`,
`map_change_cell_threshold`, `occupancy_threshold`, and `stale_anchor_retire_refreshes`.

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
