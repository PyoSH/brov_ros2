#!/usr/bin/env bash
# 리눅스 네이티브 실행 환경. `source env_native.sh` 로 쓴다.
#
# conda(python3.13)가 PATH 앞에 있으면 rclpy가 깨진다:
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# ROS Humble은 /usr/bin/python3 (3.10)로만 동작하므로 conda를 걷어낸다.

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | grep -v anaconda | paste -sd:)"
unset PYTHONHOME
unset CONDA_PREFIX

source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"          # colcon (pip --user 설치)

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$_here/install/setup.bash" ] && source "$_here/install/setup.bash"

export BROV_DATA_DIR="$_here/runtime"          # drag_test.json 저장 위치
mkdir -p "$BROV_DATA_DIR"
unset _here
