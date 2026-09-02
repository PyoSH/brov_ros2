#!/usr/bin/env bash
# 실험 A1 — 정책 이득 배율 주행. 사용: ./runtime/a1_gain.sh 0.5   (또는 1.0)
# 바닥에 놓고 시작한다 -- rise_m:=0.4 로 정책이 0.4 m 띄운 뒤 왕복. 1.0 과 0.5 각 60 초. 떨림이 0.5 에서 사라지면
# "지연+세기" 기전, 남으면 deadband/chatter.
set -e
GAIN="${1:?사용법: a1_gain.sh <0.5|1.0>}"
cd ~/BROV/brov_ros2 && source env_native.sh
TAG="$(echo "$GAIN" | tr -d .)"
ros2 launch brov_bringup pool_demo_a.launch.py \
  frame:=start_heading rise_m:=0.5 \
  wrench_gain:="$GAIN" \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=$PWD/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \
  vehicle_model_path:=$PWD/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=$PWD/runtime/bags/a1_gain${TAG} \
  cruise_speed:=0.25 leg_m:=1.5 \
  depth_source:=mavlink_ekf dvl:=false \
  send_pwm:=true arm:=true
