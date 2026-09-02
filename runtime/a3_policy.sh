#!/usr/bin/env bash
# 실험 A3 — 정책 A/B 주행. 사용: ./runtime/a3_policy.sh <delayA|fixplant>
#
# 같은 프로토콜(A1 gain 1.0)로 두 정책을 각각 60 s 돌려, 지연 DR 재학습이
# 2 Hz limit cycle 을 없앴는지 본다. 기존 A1 bag 은 leg 1.0 이라 비교 대상이
# 아니다 -- **이번 세션에서 둘 다 새로 돈다.**
#
# 바닥에 놓고 start (rise_m 이 0.5 m 띄운다). 가까운 벽에서 0.5 m, 기수 +x.
set -e
case "${1:?사용법: a3_policy.sh <delayA|fixplant>}" in
  delayA)   BUNDLE=sim2swim_delayA_wa0017_mk2_s42_i299 ;;
  fixplant) BUNDLE=sim2swim_fixplant_wa0017_mk2_s42_i299 ;;
  *) echo "delayA 또는 fixplant"; exit 1 ;;
esac
cd ~/BROV/brov_ros2 && source env_native.sh
echo "정책: $BUNDLE"
ros2 launch brov_bringup pool_demo_a.launch.py \
  frame:=start_heading rise_m:=0.5 \
  wrench_gain:=1.0 \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=$PWD/artifacts/policies/$BUNDLE/policy_raw_flu_mk2.pt \
  vehicle_model_path:=$PWD/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=$PWD/runtime/bags/a3_$1 \
  cruise_speed:=0.25 leg_m:=1.5 \
  depth_source:=mavlink_ekf dvl:=false \
  send_pwm:=true arm:=true
