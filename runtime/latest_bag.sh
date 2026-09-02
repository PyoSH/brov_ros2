#!/usr/bin/env bash
# 같은 이름으로 두 번 기록하면 launch 가 뒤에 시각을 붙인다. 가장 최근 것을 찾는다.
# 사용: ros2 run brov_base diag_loop_delay "$(./runtime/latest_bag.sh a2_yaw)" --axis yaw --open-loop --seconds 35
ls -dt ~/BROV/brov_ros2/runtime/bags/"${1:?prefix}"* 2>/dev/null | head -1
