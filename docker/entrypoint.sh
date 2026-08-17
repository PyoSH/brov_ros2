#!/usr/bin/env bash
set -e

if [[ -f /usr/local/bin/brov-ros-env ]]; then
    # shellcheck disable=SC1091
    source /usr/local/bin/brov-ros-env
else
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

exec "$@"
