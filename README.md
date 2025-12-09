# TypeGo ROS SDK
This project provides all the ROS interfaced required for TypeGo to work.

## Interfaces
The `${ROBOT_TYPE}_sdk` should expose the following topics:
| Data Type | Topic Name | Frame ID |
|-----------|------------|----------|
| RGB Image | `/camera/color/image_raw` | x |
| Depth Image | `/camera/depth/image_raw` | x |
| Lidar Scan (2D) | `/scan` | `lidar_link` |
| Robot Position | `/tf` | `base_link` |

If `$ROBOT_ID` is defined, each node should started with a namespace `robot${ROBOT_ID}` (e.g., `/robot2/scan`).

### TF
`odom` -> `base_link` -> `lidar_link`
