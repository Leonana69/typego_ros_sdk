# Configuration Map

`src/typego_config/config/robot.yaml` is the single file an operator edits. Everything
else is either deep tuning (referenced *by* `robot.yaml`) or internal launch plumbing.
Edit `robot.yaml`, relaunch the stack — `make launch` / `make console`.

```
make config_validate     # validate robot.yaml against the schema, no build
```

---

## How a value reaches a node

```
                       src/typego_config/config/robot.yaml
                                     │
        ┌────────────────────────────┼────────────────────────────┐
        ▼                            ▼                            ▼
  Makefile                   typego_bringup.launch.py       config_service node
  `typego-config env`        reads robot.yaml (PyYAML),      publishes every key as
        │                    declares launch args,          a read-only ROS param
        ▼                    threads them to child            on /typego_config
  /tmp/typego-runtime.env    launches
  (env-var subset only:            │
   ROS_DOMAIN_ID, ROBOT_IP,        ├─► {robot_type}_sdk.launch.py ─► cmd_vel_controller
   AUTONOMY_TYPE, …)               │        (motion.cmd_vel_* clamps)
        │                          │
        ▼                          └─► system_real_robot*.launch.py
  Docker build / shell                      ├─► local_planner.launch (profile, dims, speeds)
                                            ├─► far_planner.launch   (far profile)
                                            └─► tare explore         (tare profile)
```

A CLI override (`ros2 launch … key:=value`, or `make launch ARGS="key:=value"`) always
wins over the `robot.yaml`-derived default.

---

## Key-by-key map

### `robot:` — identity
| Key | Controls | Effective at |
|-----|----------|--------------|
| `id` | Namespace suffix (`1` → `/robot1`) | launch |
| `type` | Which `<type>_sdk` package launches (`go2` \| `kami`) | launch |
| `name` | Label shown in the web UI | runtime |

### `autonomy:`
| Key | Controls | Effective at |
|-----|----------|--------------|
| `type` | `base` (SLAM Toolbox + Nav2) vs `full` (ARISE/lightning + local planner) | build + launch |
| `slam_backend` | LIO backend when `type=full`: `arise` \| `lightning` | launch |

### `network:`
| Key | Drives | Effective at |
|-----|--------|--------------|
| `ros_domain_id`, `rmw`, `fastdds_builtin_transports` | DDS env vars via `/tmp/typego-runtime.env` | launch |
| `robot_ip` | `go2_sdk` / `kami_sdk` C++ clients (`config_utils.hpp`) | launch |
| `edge_service.*`, `gstreamer.*` | Edge service + video stream endpoints | launch |

### `web_gateway:`
| Key | Controls | Effective at |
|-----|----------|--------------|
| `enabled`, `host`, `port` | HMI server | launch |
| `bag_dir`, `bag_chunk_seconds`, `bag_retain` | Bag recording (last two runtime-mutable) | launch / runtime |

### `map:`
| Key | Controls | Effective at |
|-----|----------|--------------|
| `slam_map_name` | Which `Map-<name>/` resource loads (runtime-mutable) | launch / runtime |
| `init_pose_from_file` | Seed pose from `Map-<name>/init_pose.json` | launch |

### `profiles:` — selects *which* tuning YAML each tool loads
| Key | Picks | Choices |
|-----|-------|---------|
| `nav2_params_file` | `typego_sdk/config/<file>` | any file there |
| `slam_params_file` | `typego_sdk/config/<file>` | any file there |
| `local_planner_profile` | `local_planner/config/<name>.yaml` | `dog` \| `omniDir` \| `standard` |
| `far_planner_profile` | `far_planner/config/<name>.yaml` (full_mode 1) | `indoor` \| `outdoor` |
| `tare_planner_profile` | `tare_planner/<name>.yaml` (full_mode 2) | `indoor_small` \| `indoor_large` \| `outdoor` |

### `vehicle:` — physical footprint
| Key | Drives | Node param |
|-----|--------|-----------|
| `length` | local_planner `vehicleLength` | `/localPlanner vehicleLength` |
| `width` | local_planner `vehicleWidth` | `/localPlanner vehicleWidth` |

### `sensors:` — mounting offsets from `base_link`
| Key | Drives |
|-----|--------|
| `lidar_offset_x` | `sensorOffsetX` (local_planner, path follower) |
| `lidar_offset_y` | `sensorOffsetY` |
| `camera_offset_z` | `cameraOffsetZ` |

### `motion:` — speed / yaw limits
| Key | Drives | Note |
|-----|--------|------|
| `max_speed` | local_planner `maxSpeed` | Planner top speed |
| `autonomy_speed` | local_planner `autonomySpeed` | Autonomy-mode cruise speed (≤ `max_speed`) |
| `cmd_vel_max_linear` | `cmd_vel_controller` linear clamp (vx, vy) | Clamps the command sent to the robot |
| `cmd_vel_max_angular` | `cmd_vel_controller` yaw clamp | |
| `cmd_vel_min_angular` | `cmd_vel_controller` yaw deadband floor | Nonzero yaw below this snaps up to it |

**Two linear-speed knobs, on purpose.** `motion.max_speed` (default 1.375 m/s) caps the
*autonomy planner*. `motion.cmd_vel_max_linear` (default 0.8 m/s) clamps the *command
actually sent to the robot*. They are independent; today the planner can ask for more than
the controller will pass. Both are now visible here so you can reconcile them if you want a
single effective limit.

---

## Intentionally NOT in `robot.yaml`

`robot.yaml` *selects* a tuning profile; it does not contain the tuning itself. Deep
parameters live in their tool-native YAMLs and are edited there:

- Nav2 / SLAM tuning — `src/typego_sdk/config/{nav2_params,slam}.yaml`
- Local-planner tuning — `local_planner/config/{dog,omniDir,standard}.yaml`
- FAR / TARE tuning — their `config/` directories
- Terrain analysis — `terrain_analysis*/launch/*.launch`

This keeps `robot.yaml` short and operator-focused.

---

## Runtime-mutable keys

Keys listed under `dynamic:` in `robot.yaml` may be changed at runtime via
`ros2 param set /typego_config <key> <value>` and the `/typego_config/reload` service.
Everything else is read-only after launch.

## Override without editing `robot.yaml`

```bash
make launch ARGS="local_planner_profile:=omniDir max_speed:=1.0"   # one-off CLI override
TYPEGO_PROFILE=go2_indoor make launch                              # apply a profile overlay
TYPEGO_CONFIG=/abs/path/other-robot.yaml make launch               # use a different file
```

Profile overlays live in `src/typego_config/config/profiles/`.
