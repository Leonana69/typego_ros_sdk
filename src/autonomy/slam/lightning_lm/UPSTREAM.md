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

2. **`src/core/lio/laser_mapping.h`** — added a public getter
   `GetScanDownWorld()` exposing the world-frame downsampled scan that
   the LIO front-end already produces each tick.

3. **`src/core/system/slam.h` / `slam.cc`** — added ROS 2 publishers
   for `nav_msgs/Odometry` (default topic `integrated_to_init`),
   `sensor_msgs/PointCloud2` (default `registered_scan`), and a
   `tf2_ros::TransformBroadcaster` for `map → sensor`. Driven by
   `lio_->GetState()` and `lio_->GetScanDownWorld()` after each
   `ProcessLidar()` tick. Topic names and frame ids are read from a new
   optional `outputs:` block in the YAML config; defaults match the
   `arise_slam_mid360` interface contract so the rest of the autonomy
   stack (terrain_analysis, local_planner, far/tare_planner) does not
   need to be aware of the backend switch.

4. **`doc/`** — removed (162 MB of demo GIFs/images, none referenced by
   the build).

5. **`config/livox_mid360.yaml`** — added (workspace-tuned profile for
   the go2 / kami Livox MID360 + IMU setup; sets the `outputs:` block to
   match arise_slam topics).

6. **`launch/lightning_lm.launch.py`** — added (no upstream launch
   files exist; we wrap the `run_slam_online` executable so the parent
   `system_real_robot.launch.py` can include it the same way it includes
   `arise_slam.launch.py`).

When pulling new upstream commits, re-apply patches 1–3.
