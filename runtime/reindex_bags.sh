#!/usr/bin/env bash
# launch 를 Ctrl+C 로 내리면 기록기가 metadata.yaml 을 못 쓰고 죽을 때가 있다.
# diag_loop_delay 는 sqlite 직접 읽기로 폴백하지만 `ros2 bag info`/plotjuggler 는
# 못 연다. metadata 가 없는 bag 을 전부 복구한다 (내용은 건드리지 않는다).
cd ~/BROV/brov_ros2 && source env_native.sh
for d in runtime/bags/*/; do
  [ -f "$d/metadata.yaml" ] && continue
  ls "$d"/*.db3 >/dev/null 2>&1 || continue
  echo "reindex: $d"; ros2 bag reindex "$d" sqlite3 >/dev/null 2>&1 || ros2 bag reindex "$d"
done
echo "done"
