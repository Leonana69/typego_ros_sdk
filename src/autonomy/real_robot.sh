#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash

# 0: system_real_robot.launch.py
# 1: system_real_robot_with_route_planner.launch.py
# 2: system_real_robot_with_exploration_planner.launch.py
MODE="${MODE:-0}"

case "$MODE" in
  0)
    ros2 launch vehicle_simulator system_real_robot.launch.py &
    ;;
  1)
    ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py &
    ;;
  2)
    ros2 launch vehicle_simulator system_real_robot_with_exploration_planner.launch.py &
    ;;
  *)
    echo "Unknown MODE: $MODE (expected 0, 1, or 2)" >&2
    exit 1
    ;;
esac

sleep 1
if [ "${RUN_RVIZ:-1}" -eq 1 ]; then
  case "$MODE" in
    0)
      ros2 run rviz2 rviz2 -d $SCRIPT_DIR/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz
      ;;
    1)
      ros2 run rviz2 rviz2 -d $SCRIPT_DIR/route_planner/far_planner/rviz/default.rviz
      ;;
    2)
      ros2 run rviz2 rviz2 -d $SCRIPT_DIR/exploration_planner/tare_planner/rviz/tare_planner_ground.rviz
      ;;
    *)
      echo "Unknown MODE: $MODE (expected 0, 1, or 2)" >&2
      exit 1
      ;;
  esac
fi