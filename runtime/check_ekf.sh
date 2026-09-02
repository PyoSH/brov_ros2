#!/usr/bin/env bash
# EKF 가 위치를 갖고 있는가 — LOCAL_POSITION_NED 가 안 올 때 원인을 한 줄로.
# 읽기 전용 (mavlink2rest HTTP). MAVLink 클라이언트를 열지 않는다.
H=http://192.168.2.2:6040/v1/mavlink/vehicles
if ! curl -s -m 3 -o /dev/null "$H/1/components/1/messages/HEARTBEAT"; then
  echo "BlueOS(192.168.2.2:6040) 응답 없음 — 부팅 중이거나 테더 문제. 잠시 뒤 다시."; exit 2; fi
c() { curl -s -m 3 "$1" | python3 -c "import sys,json; d=json.load(sys.stdin); t=d['status']['time']; print(t.get('counter'), (t.get('last_update') or '')[11:19])" 2>/dev/null || echo "none none"; }
read v1 t1 <<< "$(c $H/255/components/0/messages/VISION_POSITION_DELTA)"
read l1 tl1 <<< "$(c $H/1/components/1/messages/LOCAL_POSITION_NED)"
sleep 1.5
read v2 t2 <<< "$(c $H/255/components/0/messages/VISION_POSITION_DELTA)"
read l2 tl2 <<< "$(c $H/1/components/1/messages/LOCAL_POSITION_NED)"
flags=$(curl -s -m 3 $H/1/components/1/messages/EKF_STATUS_REPORT | python3 -c "import sys,json; print(json.load(sys.stdin)['message']['flags']['bits'])" 2>/dev/null); flags=${flags:-0}
st() { if [ "$1" = "none" ]; then echo "메시지 없음(아직 한 번도 안 옴)"; elif [ "$1" != "$2" ]; then echo "흐름"; else echo "멈춤(마지막 $3)"; fi; }
dvl=$(st "$v1" "$v2" "$t2"); pos=$(st "$l1" "$l2" "$tl2")
hz=$(( (flags & 8) != 0 )); cp=$(( (flags & 128) != 0 ))
echo "DVL extension VISION_POSITION_DELTA: $dvl"
echo "FC LOCAL_POSITION_NED            : $pos"
echo "EKF flags $flags: POS_HORIZ_REL=$hz CONST_POS_MODE=$cp"
if [ "$pos" = "흐름" ] && [ "$hz" = 1 ]; then echo "→ OK. prepare 가 통과할 상태다."
elif [ "$dvl" != "흐름" ]; then echo "→ DVL extension 이 멈춰 있다. ./runtime/restart_dvl.sh (안 되면 로봇 재부팅)."
else echo "→ extension 은 흐르는데 EKF 가 아직 위치를 못 잡았다. 기체를 바닥에서 20~30 cm 띄우고 10 초 뒤 다시."; fi
