# Improvement Plan

Generated 2026-03-26 after line-by-line verification of `doc/TODO.md` against the codebase.

Legend: **V** = verified, **X** = not confirmed, **P** = partially confirmed (details adjusted)

---

## Phase 1 — Critical Bug Fixes (do first, minimal risk) ✅ COMPLETED

| # | Item | Status | Notes |
|---|------|--------|-------|
| 1 | `pathFollower.cpp:104` — `double stopInitTime = false;` → `= 0.0` | **DONE** | One-line fix. |
| 2 | `PI` truncation (`3.1415926`) → `M_PI` | **DONE** | 9 files affected. Replaced `#include <math.h>` with `#include <cmath>`, removed `const double PI`, replaced all uses with `M_PI`. |
| 3 | `arise_slam_mid360/package.xml` — `amend_cmake` → `ament_cmake` | **DONE** | One-character typo on line 18. |
| 4 | `visualizationTools.cpp` — unchecked `fopen` return before `fprintf` | **DONE** | Added NULL guards that disable saving flag on fopen failure. |
| 5 | `arize_slam.launch.py` filename typo → `arise_slam.launch.py` | **DONE** | Renamed file + updated 6 launch file references. |
| 6 | Signed/unsigned mismatch (`int = cloud->points.size()`) | **DONE** | Changed 36 declarations to `size_t` across 8 files, fixed for-loop variables, added casts for mixed comparisons. All packages build warning-free. |

**Dropped from Phase 1:**
- `localPlanner.cpp:968` unbounded array index via `correspondences` — **X** not confirmed. Bounds analysis shows max index = `343*35 + 342 = 12347`, within the `36*343 = 12348` element array. Safe as-is.

---

## Phase 2 — Safety & Data Integrity

| # | Item | Status | Notes |
|---|------|--------|-------|
| 7 | Thread-safety of global state | **V** | `vehicleX/Y/Z`, `odomTime`, flags written by callbacks, read by `spin_some` loop. Zero `std::atomic`/`std::mutex` in `pathFollower.cpp` or `localPlanner.cpp`. Add `std::mutex` for pose bundles, `std::atomic` for flags like `newLaserCloud`. |
| 8 | Sensor data staleness watchdog | **V** | Neither `pathFollower` nor `localPlanner` check if odometry/laser data is fresh. Add a 0.5 s timeout that zeroes `cmd_vel` when no new data arrives. |
| 9 | Parameter validation at startup | **V** | `maxSpeed`, `obstacleHeightThre`, `vehicleHeight` etc. loaded but never range-checked (`localPlanner.cpp:631-712`). Add checks for safety-critical params. |
| 10 | Replace `exit(1)` with error codes in file I/O | **V** | `localPlanner.cpp` (lines 385-524, 5 functions) and `waypointExample.cpp` (lines 58-220, 9 calls) all `exit(1)` on read errors. Return error codes or throw so the launch system can respawn. |
| 11 | Configure QoS profiles | **V** | `/registered_scan` subscriber uses default QoS (depth=5). Should be `SensorDataQoS` (best-effort). `/cmd_vel` publisher should use deadline policy. |
| 12 | No `use_sim_time` propagation in base_autonomy XML launch files | **V** | All 6 XML launch files (`local_planner.launch`, `terrain_analysis.launch`, etc.) lack `use_sim_time`. Causes TF extrapolation errors with rosbags. |
| 13 | `realRobot` parameter silently ignored | **V** | `system_real_robot.launch.py:34` passes `'realRobot': 'true'` to `local_planner.launch`, which never declares or reads it. Just remove. |

---

## Phase 3 — Performance (hot-path optimizations)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 14 | Replace `sqrt` with squared-distance comparisons | **V** | `localPlanner.cpp` lines 835/850/865/941: `sqrt(x*x+y*y) < threshold` in inner loops (~36x343 iterations). Compare squared distances instead. |
| 15 | Pre-compute sin/cos lookup in `localPlanner` | **V** | Lines 944-976: `cos(rotAng)`/`sin(rotAng)` recomputed per point for each of 36 rotations. Build a 36-element table outside the point loop. |
| 16 | Avoid redundant point struct copies | **V** | `terrainAnalysis.cpp:164`: `point = laserCloud->points[i]` copies full struct, then reads `point.x/y/z`. Access fields directly. |
| 17 | `reserve()` before `push_back` on point clouds | **V** | `laserCloudCrop`, `plannerCloudCrop`, `terrainCloudElev` grow without `reserve()`. Sizes bounded by input cloud — reserve to avoid reallocations. |

---

## Phase 4 — Launch & Deployment Hardening

| # | Item | Status | Notes |
|---|------|--------|-------|
| 18 | Add `respawn=True` to critical nodes | **V** | Only 1 launch file has `respawn` (set to `False`). Add `respawn=True` + `respawn_delay=2.0` for SLAM, local_planner, terrain_analysis. |
| 19 | Parameterize joystick device path | **V** | 9 launch files hardcode `/dev/input/js0`. Add `DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')`. |
| 20 | Fix `nav2_launch.py` temp file write | **V** | Lines 40-45 write to `/tmp/nav2_params_modified_<pid>.yaml`. Use `RewrittenYaml` from `nav2_common` or pass overrides via `parameters`. |
| 21 | Standardize launch file format (XML → Python) | **V** | 11 packages use XML `.launch`, rest use Python `.launch.py`. Migrate XML launch files to Python for consistency and programmatic argument passing. |
| 22 | Consistent namespace handling | **V** | `slam_launch.py` uses `namespace=` param + remappings. `nav2_launch.py` uses `GroupAction` + `PushRosNamespace`. Pick one pattern workspace-wide. |
| 23 | Fix hardcoded `/opt/ros/humble/include` in tare_planner | **V** | `tare_planner/CMakeLists.txt:39`. Replace with `ament_target_dependencies()`. |
| 24 | Fix `tare_planner` `LD_LIBRARY_PATH` hack | **V** | `explore.launch.py:78` dynamically computes and sets `LD_LIBRARY_PATH` to find vendored or-tools. Install or-tools libs properly via CMake. |
| 25 | Validate `ROBOT_ID` environment variable | **V** | `kami_sdk.launch.py:12` accepts empty/malformed `ROBOT_ID` without validation. Add check with clear error message. |
| 26 | Add a default deployment profile | **V** | `typego_bringup.launch.py` requires `robot_type`/`autonomy_type`/`slam_map` args. Add a `go2_outdoor.launch.py` that works out of the box. |

**Adjusted:**
- Vehicle config profile selection (TODO §R2-3.4) — **P**. The mechanism exists (`local_planner.launch` has `<arg name="config" default="omniDir"/>`), but `system_real_robot.launch.py` hardcodes `'config': 'dog'`. Fix: expose `vehicle_type` as a top-level launch argument.

**User Comment**
- Ignore 22, 23, 24, 25, and 26.
---

## Phase 5 — Code Quality & Maintainability

| # | Item | Status | Notes |
|---|------|--------|-------|
| 27 | Remove `using namespace std;` | **V** | All 8 base_autonomy `.cpp` files. Qualify `std::` explicitly. |
| 28 | Replace C-style I/O with `std::ifstream` | **V** | `localPlanner.cpp` lines 377-579: 5 functions use `fscanf`/`fopen`. Use `std::ifstream` with RAII. |
| 29 | Replace `#define PLOTPATHSET` with `constexpr bool` | **V** | `localPlanner.cpp:47` and 6 `#if` guards. Use `constexpr bool` or CMake option. |
| 30 | Add header files (`.hpp`) for node logic | **V** | All node logic in single `.cpp` files, no headers. Extract function declarations for unit-testability. |
| 31 | Name magic numbers as `constexpr` | **V** | `36` (rotations), `343` (paths), `7` (groups), `129` (mid-group), `30` (pointSkipNum), `100` (spin rate) — all bare literals in `localPlanner.cpp`. |
| 32 | Replace `NULL` with `nullptr` | **V** | `far_planner.cpp:87`, `graph_planner.cpp` (7 instances), `contour_graph.cpp` (27 instances). |

---

## Phase 6 — Architecture Refactoring (larger scope)

| # | Item | Status | Notes |
|---|------|--------|-------|
| 33 | Merge `terrain_analysis` + `terrain_analysis_ext` | **V** | 693 + 568 lines, ~60% identical. Only config differs: local 21x21@1.0m vs extended 41x41@2.0m. Refactor into one package with parameterized config. |
| 34 | Extract shared coordinate transform to `autonomy_common` | **V** | Same cos/sin yaw rotation copy-pasted in `terrainAnalysis.cpp:523-538`, `localPlanner.cpp:819-846` (5+ occurrences). Also de-duplicates PI, voxel indexing, parameter helpers. |
| 35 | Split `localPlanner.cpp` monolith | **V** | 1228 lines: file I/O (5 funcs), action server, joystick handler, 7 subscription callbacks, path scoring, obstacle checking, visualization. Split into: path library loader, path scorer, node shell. |
| 36 | Unify parameter handling | **V** | 95 lines of boilerplate (lines 618-712) for 47 declare+get pairs. Same pattern in all nodes. Use a helper template or struct-based approach. |
| 37 | Convert to ROS 2 component nodes | **V** | All 4 primary nodes use `make_shared` + `spin_some`. Convert to `rclcpp::Node` subclasses for composable deployment (zero-copy intra-process). |
| 38 | Add lifecycle node support | **V** | No `LifecycleNode` usage anywhere. `local_planner` can publish `cmd_vel` before SLAM is ready. Add managed startup sequencing. |

** User Comment**
- Ignore 33.
---

## Phase 7 — Build System Cleanup

| # | Item | Status | Notes |
|---|------|--------|-------|
| 39 | Externalize vendored or-tools (137 MB) and gtsam (246 MB) | **V** | Total ~383 MB in-tree. Use `rosdep` keys or CMake `FetchContent`/`ExternalProject`. |
| 40 | Add `COLCON_IGNORE` for or-tools | **P** | gtsam already has `COLCON_IGNORE` in build subdirectories; or-tools has none. Add marker to prevent colcon from scanning it. |
| 41 | Document TARE planner parameters | **P** | ~87 parameters per config (not 130+ as claimed). `kKeyposeCloudDwzFilterLeafSize` etc. have section headers but no per-parameter docs. Add inline comments with units and valid ranges. |

**Dropped from plan:**
- Missing `<exec_depend>` entries in base_autonomy `package.xml` — **X** not confirmed. All packages use `<depend>` tags which cover both build and runtime in ROS 2.

---

## Phase 8 — Testing & Observability

| # | Item | Status | Notes |
|---|------|--------|-------|
| 42 | Unit tests for voxel indexing | **V** | Voxel-to-index conversion (with `< 0` offset correction) used in 6+ places. Test boundary cases. |
| 43 | Integration test: `terrain_analysis` → `local_planner` pipeline | **V** | Replay rosbag, assert terrain_map published, valid path selected, cmd_vel within bounds. |
| 44 | Action server goal lifecycle test | **V** | Test `navigate_to_pose` goal → feedback → succeed/abort, including joystick override cancel. |
| 45 | Launch test for `typego_bringup` | **V** | Only 1 launch test exists (domain_bridge). Use `launch_testing` for full graph startup validation. |
| 46 | Add `/diagnostics` publishing | **V** | Zero `diagnostic_msgs` usage. Add status for terrain map freshness, path selection rate, SLAM health. |
| 47 | Add parameter schema files | **V** | No schemas exist. Use ROS 2 parameter descriptor constraints to catch bad config at startup. |
| 48 | Add CI build matrix (x86_64 + aarch64) | **V** | No CI configuration at repo root. Add GitHub Actions with Jetson Docker image for cross-arch testing. |

---

## Verification Summary

| Category | Confirmed | Not Confirmed | Partially | Total |
|----------|-----------|---------------|-----------|-------|
| Round 1 Bug Fixes | 4 | 0 | 0 | 4 |
| Round 1 Architecture | 6 | 0 | 0 | 6 |
| Round 1 Safety | 4 | 0 | 0 | 4 |
| Round 1 Performance | 4 | 0 | 0 | 4 |
| Round 1 Code Quality | 6 | 0 | 0 | 6 |
| Round 1 Testing | 3 | 0 | 0 | 3 |
| Round 2 Bug Fixes | 3 | **1** | 0 | 4 |
| Round 2 Launch System | 9 | 0 | 0 | 9 |
| Round 2 Config & Deploy | 2 | 0 | **2** | 4 |
| Round 2 Build System | 2 | **1** | **1** | 4 |
| Round 2 Cross-Package | 3 | 0 | 0 | 3 |
| Round 2 Testing | 4 | 0 | 0 | 4 |
| **Total** | **50** | **2** | **3** | **55** |

**Items dropped (not confirmed):**
1. `localPlanner.cpp:968` unbounded array index — bounds analysis shows it is safe
2. Missing `<exec_depend>` in `package.xml` — all use `<depend>` which covers runtime

**Items adjusted (partially confirmed):**
1. TARE params count: ~87, not 130+
2. Vehicle config selection: mechanism exists but hardcoded in practice
3. `COLCON_IGNORE`: gtsam has markers, or-tools does not
