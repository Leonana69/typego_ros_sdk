#!/bin/bash

# Get robot namespace from ROBOT_ID environment variable
ROBOT_ID="${ROBOT_ID:-}"
if [ -n "$ROBOT_ID" ]; then
    ROBOT_NAME="robot${ROBOT_ID}"
    ROBOT_NS="/${ROBOT_NAME}"
else
    ROBOT_NS=""
fi

echo "Kami 🤖 Using robot namespace: ${ROBOT_NS:-<none>}"

ros2 run kami_sdk tf_service_node \
    --ros-args -r /tf:=${ROBOT_NS}/tf -r /tf_static:=${ROBOT_NS}/tf_static &
ros2 run kami_sdk lidar_service_node \
    --ros-args -r /tf:=${ROBOT_NS}/tf -r /tf_static:=${ROBOT_NS}/tf_static &