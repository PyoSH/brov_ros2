#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source /workspace/brov_ros2/docker/ros_env.sh

cd "${BROV_ROS_WS}"
colcon build --symlink-install --event-handlers console_direct+

set +u
# shellcheck disable=SC1091
source "${BROV_ROS_WS}/install/setup.bash"
set -u

for package in brov_base brov_control brov_perception brov_viz brov_bringup; do
    ros2 pkg prefix "${package}" >/dev/null
done

echo "BROV ROS 2 workspace ready: ${BROV_ROS_WS}/install"
