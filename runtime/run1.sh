#!/usr/bin/env bash
# AHRS(start_heading) 주행 — 마커 정렬 없음.
set -e
cd ~/BROV/brov_ros2
source env_native.sh

ros2 launch brov_bringup pool_demo_a.launch.py \
  frame:=start_heading \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=$PWD/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \
  vehicle_model_path:=$PWD/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=$PWD/runtime/bags/run1 \
  cruise_speed:=0.25 leg_m:=2.5 \
  depth_source:=mavlink_ekf \
  send_pwm:=true arm:=true
