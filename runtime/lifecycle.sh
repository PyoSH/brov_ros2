#!/usr/bin/env bash
# prepare -> arm -> start. prepare 가 실패하면 **사유를 보여주고** 재시도한다.
# "telemetry 없음" 이면 heartbeat 만 오는 링크일 가능성이 크다 -- 재실행 대신
# /brov/request_streams 로 스트림을 다시 요청한 뒤 재시도한다 (2026-09-02 실기).
cd ~/BROV/brov_ros2 && source env_native.sh
for i in $(seq 1 10); do
  out=$(ros2 service call /brov/prepare_control std_srvs/srv/Trigger 2>&1)
  if echo "$out" | grep -q "success=True"; then echo "prepare OK"; break; fi
  msg=$(echo "$out" | grep -o "message='[^']*'")
  echo "prepare 실패 $i/10: $msg"
  if echo "$msg" | grep -q "telemetry"; then
    ros2 service call /brov/request_streams std_srvs/srv/Trigger 2>&1 | grep -o "message='[^']*'"
  fi
  sleep 2
done
echo "$out" | grep -q "success=True" || { echo "prepare 를 못 넘겼다. 위 사유를 볼 것."; exit 1; }
ros2 service call /brov/arm_control   std_srvs/srv/Trigger | grep -o "message='[^']*'"
ros2 service call /brov/start_control std_srvs/srv/Trigger | grep -o "message='[^']*'"
echo "제어 중. 끝나면:  ./runtime/stop.sh"
