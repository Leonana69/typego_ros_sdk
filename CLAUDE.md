# CLAUDE.md

## Build

Source ROS using `local_setup.bash` (not `setup.bash`) to avoid sourcing the workspace overlay before it exists:

```bash
bash -c 'source /opt/ros/humble/local_setup.bash && colcon build --packages-select <package_name>'
```
