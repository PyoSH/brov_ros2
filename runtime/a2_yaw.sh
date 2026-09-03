#!/usr/bin/env bash
# 실험 A2 — 역전 없는 지연 측정 (yaw). 기체는 제자리에서 ~47 deg/s 로 돈다.
# bias 1.0 + amplitude 0.5 N*m -> 수평 추진기 PWM 0.10~0.16, 0 을 넘지 않음.
# 바닥에 놓고 start. 노드가 0.4 m 띄운 뒤 그 깊이를 스스로 지킨다 (무게 불필요).
#
# 사용: ./runtime/a2_yaw.sh [telemetry_hz]   기본 25. G1 시험은 50 (bag: a2_yaw_t50).
set -e
TELEM="${1:-25}"
# G4 ③ 전용 endpoint A/B: CONN=udpout:192.168.2.2:14560 ./runtime/a2_yaw.sh 50
CONN="${CONN:-udpout:192.168.2.2:14550}"
# Step 4b 진단 전용: BACKEND=do_set_servo ./runtime/a2_yaw.sh 50  (미션에 쓰지 말 것)
BACKEND="${BACKEND:-rc_override}"
# τ 정밀화: KIND=chirp ./runtime/a2_yaw.sh 50  (0.5→8 Hz, 09-03 chirp 과 같은 조건)
KIND="${KIND:-square}"
cd ~/BROV/brov_ros2 && source env_native.sh
ros2 launch brov_bringup deadtime_test.launch.py \
  axis:=yaw kind:=$KIND bias:=1.0 amplitude:=0.5 period_s:=1.0 duration_s:=40 \
  rise_m:=0.4 telemetry_rate_hz:=$TELEM actuation_backend:=$BACKEND \
  chirp_f0_hz:=0.5 chirp_f1_hz:=8.0 \
  connection:=$CONN \
  bag_path:=$PWD/runtime/bags/a2_yaw_t${TELEM}$([ "$BACKEND" = rc_override ] || echo _$BACKEND)$([ "$KIND" = square ] || echo _$KIND) \
  send_pwm:=true arm:=true
