#!/usr/bin/env bash
# M1/M2 — 링크 지연 하한과 MAVLink 왕복. 스택을 띄우지 않는다 (추력 없음).
# LATENCY_DECOMPOSITION_PLAN.md §2.
cd ~/BROV/brov_ros2 && source env_native.sh
echo "=== M1: 랩톱 → 로봇 RPi ping x50 ==="
ping -c 50 -i 0.1 192.168.2.2 | tail -3
echo
echo "=== M2: 랩톱 ↔ FC MAVLink 왕복 (라우터 포함) x50 ==="
ros2 run brov_base diag_link_rtt --rounds 50
