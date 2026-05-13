# lightning-lm (vendored)

Upstream: https://github.com/gaoxiang12/lightning-lm
Commit:   1325fed8fa97a2506878360fc23427ea00a767da  (2026-04-07)
Fetched:  2026-05-13

## License

Upstream `package.xml` says `TODO: License declaration` and there is no
`LICENSE` file in the repository. This vendored copy is therefore kept for
internal/exploratory use only — do **not** redistribute downstream without
first clarifying the license with the upstream author (Gao Xiang,
`gao.xiang.thu@gmail.com`).

## Local patches relative to upstream

1. **`package.xml`** — removed two undeclared deps the upstream package
   inherits from an internal build that are never referenced in source:
   `scrubber_common`, `agibot_robot`.

2. **`src/core/lio/laser_mapping.h`** — added two public getters:
   `GetScanDownWorld()` exposes the world-frame downsampled scan the
   LIO front-end already produces each tick (used for debugging /
   visualisation), and `GetScanUndistWorld()` returns the full
   undistorted scan transformed into the world frame (this is what gets
   published on `/registered_scan` — it matches arise_slam's
   `laserCloudFullRes` density so terrain_analysis sees the same
   obstacle data under either backend).

3. **`src/core/system/slam.h` / `slam.cc`** — added ROS 2 publishers
   for `nav_msgs/Odometry` (default topic `/state_estimation` — the
   topic terrain_analysis / local_planner / sensor_scan_generation
   actually subscribe to; arise_slam's `/integrated_to_init` is an
   internal topic of the LIO front-end, not what downstream consumes),
   `sensor_msgs/PointCloud2` (default `/registered_scan`), and a
   `tf2_ros::TransformBroadcaster` for `map → sensor`. Topic names
   and frame ids are read from a new optional `outputs:` block in the
   YAML config; defaults match what the autonomy stack
   (terrain_analysis, local_planner, far/tare_planner) actually
   subscribes to, so the rest of the stack does not need to be aware
   of the backend switch. Published from two callbacks: `ProcessLidar`
   (full: odom + TF + dense scan, ~10 Hz, via `lio_->GetState()` and
   `lio_->GetScanUndistWorld()`) and `ProcessIMU` (light: odom + TF
   only at IMU rate, via `lio_->GetIMUState()` from lightning's
   existing IMU-rate ESKF). The IMU-rate publish mirrors arise_slam's
   `imu_preintegration` node — without it, local_planner sees a stale
   pose between lidar ticks and the robot can drive past obstacles
   before reacting. Also: skip creating
   `cloud_sub_` / `livox_sub_` when their YAML topic name is empty
   (rclcpp rejects empty topic strings with `InvalidTopicNameError`).

4. **`doc/`** — removed (162 MB of demo GIFs/images, none referenced by
   the build).

5. **`config/livox_mid360.yaml`** — added (workspace-tuned profile for
   the go2 / kami Livox MID360 + IMU setup; sets the `outputs:` block to
   match arise_slam topics).

6. **`launch/lightning_lm.launch.py`** — added (no upstream launch
   files exist; we wrap the `run_slam_online` executable so the parent
   `system_real_robot.launch.py` can include it the same way it includes
   `arise_slam.launch.py`).

7. **`thirdparty/livox_ros_driver/CMakeLists.txt`** — set
   `LIVOX_ROS_DRIVER2_VERSION` to `1.0.0` before the `project(...)` call.
   Upstream references `${LIVOX_ROS_DRIVER2_VERSION}` but never defines
   it (the `include(cmake/version.cmake)` line is commented out), so
   ament_cmake_python emits `setup.py` with `version=''`. Modern
   setuptools (≥ 68, common in conda envs) rejects empty versions with
   `packaging.version.InvalidVersion`.

8. **`src/app/run_slam_online.cc`** — split argv before handing it to
   gflags. `ros2 launch` always appends `--ros-args ...` after our own
   args; gflags rejects unknown flags by default and aborts. We
   truncate argc at the first `--ros-args` for gflags, then pass the
   original (full) argv to `rclcpp::init` so node remapping still works.

9. **`scripts/typego_to_livox_bridge.py`** — added (helper node).
   Lightning's PointCloud2 preprocessor only handles Ouster / Velodyne /
   RoboSense; for Livox it requires `livox_ros_driver2::msg::CustomMsg`.
   The go2 / kami driver publishes Livox CustomMsg under the
   `typego_interface` namespace, and DDS treats the two messages as
   distinct types despite field-identical layouts. This Python node
   subscribes to `typego_interface/CustomMsg` and republishes the same
   payload as `livox_ros_driver2/CustomMsg`. Installed via
   `install(PROGRAMS ...)` in the lightning CMakeLists, started from
   `launch/lightning_lm.launch.py`.

When pulling new upstream commits, re-apply patches 1–3 (and 8 if
`run_slam_online.cc` changes upstream).
