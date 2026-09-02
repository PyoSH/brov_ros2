#!/usr/bin/env bash
# 실험 A2 — 역전 없는 지연 측정 (yaw). 기체는 제자리에서 ~47 deg/s 로 돈다.
# bias 1.0 + amplitude 0.5 N*m -> 수평 추진기 PWM 0.10~0.16, 0 을 넘지 않음.
# 바닥에 놓고 start. 노드가 0.4 m 띄운 뒤 그 깊이를 스스로 지킨다 (무게 불필요).
set -e
cd ~/BROV/brov_ros2 && source env_native.sh
ros2 launch brov_bringup deadtime_test.launch.py \
  axis:=yaw kind:=square bias:=1.0 amplitude:=0.5 period_s:=1.0 duration_s:=40 \
  rise_m:=0.4 \
  connection:=udpout:192.168.2.2:14550 \
  bag_path:=$PWD/runtime/bags/a2_yaw \
  send_pwm:=true arm:=true
