#!/bin/bash
# Sim2Swim 논문 Fig.4 (a)(b)(c) — Gazebo SITL 재현.
#
# IsaacLab 의 test_policy.py --test {straight_line,square_ballast,
# square_random_attitude} 와 **같은 기하/heading/속도**로 돈다. 차이는 깊이
# 기준뿐이다: IsaacLab 은 절대 depth 5.0 의 평면 경로, 여기는 mission frame
# z=0(= start 깊이)의 같은 평면 경로다. 양쪽 다 수직 leg 이 없다.
#
# 사각 waypoint 의 y 부호에 주의. IsaacLab 은 Z-up 이라 +y 인데 mission frame 은
# NED 라 부호가 반대다. 숫자를 그대로 옮기면 **거울상**이 되고, 밸러스트가
# 좌현이라 선회 방향이 물리적으로 달라진다.
#
# depth_source=pressure 를 명시하는 이유는 docs/DEPTH_SOURCE.md 참조.
# 기본값(mavlink_ekf)으로 돌리면 EKF 수직 위치가 얼어붙어 기체가 수면까지
# 떠오른다 — 미션 중 총 상승 1.77 m vs 0.19 m.
#
# usage: run_fig4_sitl.sh <a|b|c> [send_pwm] [arm]
set -u
CASE=$1; PWM=${2:-true}; ARM=${3:-true}
REPO=${BROV_ROS2_DIR:-$HOME/brov_ros2}
POLICY=$REPO/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt

case "$CASE" in
  a) WPS="0,0,0;5.0,0,0";                       HEAD=align ;;
  b) WPS="0,0,0;5.0,0,0;5.0,-5.0,0;0,-5.0,0";   HEAD=upright ;;
  c) WPS="0,0,0;5.0,0,0;5.0,-5.0,0;0,-5.0,0";   HEAD=random_at_waypoint ;;
  *) echo "usage: run_fig4_sitl.sh <a|b|c> [send_pwm] [arm]"; exit 1 ;;
esac

# case (b) 는 600 g 밸러스트가 필요하다. IsaacLab 은 질량이 아니라 부피 결손으로
# 근사하므로(0.6 kg / 1000 = 6.0e-4 m^3, CoB 측방 -0.0114 m), Gazebo 에서도 같은
# 값을 collision box 로 옮긴다. 적용/원복 후 sim 재시작이 선행돼야 한다.
[ "$CASE" = "b" ] && echo "  주의: case (b) 는 밸러스트 적용 후 sim 재시작이 선행돼야 한다"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
exec ros2 launch brov_bringup split_stack.launch.py \
  connection:=udpin:0.0.0.0:14552 \
  thruster_reversal_profile:=edo_sitl_identity \
  thruster_model:=gazebo_linear \
  policy_path:="$POLICY" \
  vehicle_model_path:="$REPO/brov_base/brov_base/vendor/brov2_heavy.yaml" \
  "waypoints:=$WPS" \
  depth_source:=pressure \
  waypoint_frame:=start_heading heading_mode:=$HEAD loop:=true \
  max_segment_length_m:=50.0 \
  cruise_speed:=0.50 lookahead_dist:=1.0 reach_threshold:=0.30 \
  send_pwm:=$PWM arm:=$ARM
