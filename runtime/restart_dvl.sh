#!/usr/bin/env bash
# BlueOS 의 Water Linked DVL extension 만 재시작한다 (로봇 재부팅 대신).
# 언제: ./runtime/check_ekf.sh 가 "DVL extension 이 멈춰 있다" 를 낼 때.
# 원인: dvl_record_node 가 A50 의 TCP 슬롯을 차지하면 extension 이 떨어지고
#       스스로 회복하지 않는다 (2026-09-02). 정책 주행에 DVL 기록을 켜지 말 것.
set -e
curl -s -m 10 -X POST "http://192.168.2.2/kraken/v2.0/extension/restart?extension_identifier=bluerobotics.water-linked-dvl" \
  -H "accept: application/json" && echo
echo "재시작 요청 보냄. 60~90 s 뒤:  ./runtime/check_ekf.sh"
