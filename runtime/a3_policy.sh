#!/usr/bin/env bash
# 실험 A3 — 정책 A/B 주행. 사용: ./runtime/a3_policy.sh [delayA|fixplant] [telemetry_hz]
#   telemetry_hz 기본 25. G1 배포 시험은 50 (bag 에 _t50 이 붙는다).
#
# 같은 프로토콜(A1 gain 1.0)로 두 정책을 각각 60 s 돌려, 지연 DR 재학습이
# 2 Hz limit cycle 을 없앴는지 본다. 기존 A1 bag 은 leg 1.0 이라 비교 대상이
# 아니다 -- **이번 세션에서 둘 다 새로 돈다.**
#
# 바닥에 놓고 start (rise_m 이 0.5 m 띄운다). 가까운 벽에서 0.5 m, 기수 +x.
set -e
# 2026-09-03 사용자 결정: delayA 가 기본 배포 정책으로 승격. 단 fixplant
# (지연 없이 학습된 정책)는 **대조군으로 유지** — 비교 주행 시 같은 자리에서
# 짝으로 돌린다 (9-03 세션의 55 s 창 A/B 방식).
case "${1:-delayA}" in
  delayA)   BUNDLE=sim2swim_delayA_wa0017_mk2_s42_i299 ;;
  fixplant) BUNDLE=sim2swim_fixplant_wa0017_mk2_s42_i299 ;;
  *) echo "delayA 또는 fixplant"; exit 1 ;;
esac
TELEM="${2:-25}"
# heading_mode: HEADING=straight ./runtime/a3_policy.sh delayA 50  (기본 align)
HEADING="${HEADING:-align}"
cd ~/BROV/brov_ros2 && source env_native.sh
echo "정책: $BUNDLE  telemetry ${TELEM} Hz"
ros2 launch brov_bringup pool_demo_a.launch.py \
  frame:=start_heading rise_m:=0.5 telemetry_rate_hz:=$TELEM heading_mode:=$HEADING \
  wrench_gain:=1.0 \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=$PWD/artifacts/policies/$BUNDLE/policy_raw_flu_mk2.pt \
  vehicle_model_path:=$PWD/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=$PWD/runtime/bags/a3_${1:-delayA}_${HEADING}_t$TELEM \
  cruise_speed:=0.25 leg_m:=1.5 \
  depth_source:=mavlink_ekf dvl:=false \
  send_pwm:=true arm:=true
