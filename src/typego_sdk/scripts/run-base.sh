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

# Start the IOX router
iox-roudi &

# Run the script in $(ROBOT_TYPE)_sdk/scripts/run.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"
chmod +x "${WORKSPACE_SRC}/${ROBOT_TYPE}_sdk/scripts/run.sh"
"${WORKSPACE_SRC}/${ROBOT_TYPE}_sdk/scripts/run.sh"