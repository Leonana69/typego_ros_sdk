#!/bin/bash
# Get robot namespace from ROBOT_ID environment variable
ROBOT_ID="${ROBOT_ID:-}"
if [ -n "$ROBOT_ID" ]; then
    ROBOT_NS="robot${ROBOT_ID}"
    MAP_TOPIC="/${ROBOT_NS}/map"
else
    ROBOT_NS=""
    MAP_TOPIC="/map"
fi
echo "🤖 Using robot namespace: ${ROBOT_NS:-<none>}"

# Source the ROS 2 setup file and run the nodes in the background
iox-roudi &
source /workspace/install/setup.bash && ros2 run go2_sdk tf_service_node &
source /workspace/install/setup.bash && ros2 run go2_sdk livox_udp_receiver_node &
source /workspace/install/setup.bash && ros2 run go2_sdk gstreamer_receiver_node