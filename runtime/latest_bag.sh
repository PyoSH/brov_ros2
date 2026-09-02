#!/usr/bin/env bash
# 같은 이름으로 두 번 기록하면 launch 가 뒤에 시각을 붙인다. 가장 최근 것을 찾는다.
# 못 찾으면 **빈 문자열 대신 실패**한다 -- 빈 인자가 분석기까지 흘러가 'no such
# table: topics' 로 죽는 것보다 여기서 멈추는 게 낫다 (2026-09-03).
set -u
p="${1:?사용법: latest_bag.sh <prefix>}"
d=$(ls -dt ~/BROV/brov_ros2/runtime/bags/"$p"* 2>/dev/null | head -1)
if [ -z "$d" ]; then
  echo "latest_bag.sh: '$p' 로 시작하는 bag 이 없다 — 그 주행이 아직 안 돌았다." >&2
  exit 1
fi
echo "$d"
