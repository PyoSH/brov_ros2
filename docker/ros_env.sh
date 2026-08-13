#!/usr/bin/env bash

_brov_ros_setup="/opt/ros/humble/setup.bash"
_brov_overlay_setup="/workspace/brov_ros2/install/setup.bash"
_brov_restore_nounset=false

case "$-" in
    *u*)
        _brov_restore_nounset=true
        set +u
        ;;
esac

if [[ -f "${_brov_ros_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${_brov_ros_setup}"
fi

if [[ -f "${_brov_overlay_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${_brov_overlay_setup}"
fi

export BROV_ROS_WS="/workspace/brov_ros2"
export BROV_DATA_DIR="${BROV_DATA_DIR:-/workspace/brov_ros2/runtime}"

if [[ "${_brov_restore_nounset}" == "true" ]]; then
    set -u
fi

unset _brov_ros_setup _brov_overlay_setup _brov_restore_nounset
