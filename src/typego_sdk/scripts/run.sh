#!/bin/bash

# Get robot namespace from ROBOT_ID environment variable
ROBOT_ID="${ROBOT_ID:-}"
if [ -n "$ROBOT_ID" ]; then
    ROBOT_NAME="robot${ROBOT_ID}"
    ROBOT_NS="/${ROBOT_NAME}"
    MAP_TOPIC="${ROBOT_NS}/map"
else
    ROBOT_NS=""
    MAP_TOPIC="/map"
fi

ROBOT_TYPE="${ROBOT_TYPE:-go2}"

SLAM_MAP_NAME="${SLAM_MAP_NAME:-empty_map}"

echo "🤖 Using robot namespace: ${ROBOT_NS:-<none>}, SLAM map name: ${SLAM_MAP_NAME:-<none>}"

# Source the ROS 2 setup file and run the nodes in the background
iox-roudi &

ros2 run ${ROBOT_TYPE}_sdk tf_service_node \
    --ros-args -r /tf:=${ROBOT_NS}/tf -r /tf_static:=${ROBOT_NS}/tf_static &
ros2 run ${ROBOT_TYPE}_sdk lidar_service_node \
    --ros-args -r /tf:=${ROBOT_NS}/tf -r /tf_static:=${ROBOT_NS}/tf_static &
ros2 run ${ROBOT_TYPE}_sdk video_service_node &

# Launch SLAM with robot namespace (works with empty string too)

if [ -n "$ROBOT_NS" ]; then
	ros2 launch typego_sdk slam_launch.py robot_namespace:=$ROBOT_NAME existing_map:=$SLAM_MAP_NAME &
else
	ros2 launch typego_sdk slam_launch.py existing_map:=$SLAM_MAP_NAME &
fi

# Wait until /map or /<namespace>/map topic is available
echo "⏳ Waiting for ${MAP_TOPIC} topic to be available..."
while ! ros2 topic list | grep -q "${MAP_TOPIC}"; do
	echo "🚧 ${MAP_TOPIC} not available yet. Retrying in 1 second..."
	sleep 1
done

ros2 run typego_sdk waypoints_service_node \
    --ros-args -r /tf:=${ROBOT_NS}/tf -r /tf_static:=${ROBOT_NS}/tf_static &

# Launch Nav2 with robot namespace (works with empty string too)
if [ -n "$ROBOT_NS" ]; then
    ros2 launch typego_sdk nav2_launch.py robot_namespace:=$ROBOT_NAME &
else
    ros2 launch typego_sdk nav2_launch.py &
fi

echo "All services launched. Waiting for them to exit..."
wait