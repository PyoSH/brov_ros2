#!/usr/bin/env bash
# 리눅스 네이티브 실행 환경. `source env_native.sh` 로 쓴다.
#
# conda(python3.13)가 PATH 앞에 있으면 rclpy가 깨진다:
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# ROS Humble은 /usr/bin/python3 (3.10)로만 동작하므로 conda를 걷어낸다.

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | grep -v anaconda | paste -sd:)"
unset PYTHONHOME
unset CONDA_PREFIX

# 인터프리터에 file capability가 붙어 있으면 rclpy가 같은 증상으로 깨진다:
#   ImportError: librcl_action.so: cannot open shared object file
# capability가 붙은 바이너리는 glibc의 secure-execution mode(AT_SECURE=1)로
# 뜨고, 그 모드에서 ld.so는 LD_LIBRARY_PATH를 **환경에서 지운다.** 아래에서
# 아무리 제대로 세팅해도 소용이 없다 -- AMENT_PREFIX_PATH는 살아남아서 ROS가
# 패키지는 찾는데 .so만 못 여는, 원인을 짚기 어려운 형태가 된다.
#
# 2026-01-04에 libfranka 실시간 제어용으로 `setcap cap_sys_nice=eip`를 걸어
# 실제로 겪었다. libfranka에는 필요 없다 -- SCHED_FIFO는 limits.conf의
# `@realtime rtprio 99`만으로 되고, 그쪽은 AT_SECURE를 세우지 않는다.
if command -v getcap >/dev/null 2>&1; then
    _brov_py="$(readlink -f "$(command -v python3)")"
    if [ -n "$(getcap "$_brov_py" 2>/dev/null)" ]; then
        echo "경고: $_brov_py 에 capability가 붙어 있다 —" >&2
        echo "      $(getcap "$_brov_py")" >&2
        echo "      LD_LIBRARY_PATH가 무시되어 rclpy import가 실패한다." >&2
        echo "      해제: sudo setcap -r $_brov_py" >&2
        echo "      음수 nice가 필요하면 /etc/security/limits.conf에" >&2
        echo "      '@realtime soft nice -20' / '@realtime hard nice -20'을 쓸 것." >&2
    fi
    unset _brov_py
fi

source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"          # colcon (pip --user 설치)

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$_here/install/setup.bash" ] && source "$_here/install/setup.bash"

export BROV_DATA_DIR="$_here/runtime"          # drag_test.json 저장 위치
mkdir -p "$BROV_DATA_DIR"
unset _here
