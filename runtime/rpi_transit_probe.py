#!/usr/bin/env python3
"""G4 분해 ④ — RPi 관측점. **로봇의 BlueOS 터미널에서** 실행한다.

44 ms 잔차가 FC→RPi(직렬)에 있는지 RPi→랩톱(라우터·이더넷)에 있는지 가른다:
같은 메시지의 (FC 시계 도장, RPi 도착 시각)을 여기서 기록하고, 랩톱 bag 의
(FC 도장, 랩톱 도착)과 대조하면

    FC→RPi 몫 = d(RPi)        RPi→랩톱 몫 = d(랩톱) − d(RPi)

이 답이 곧 **온보드 이전의 가치 판정**이다 — 잔차가 FC→RPi 쪽이면 온보드로
못 줄인다(그 구간을 그대로 지난다).

준비: BlueOS → MAVLink Endpoints 에 **UDP Client → 127.0.0.1:14560** 을 추가한다
(라우터가 이 주소로 보내고, 프로브가 udpin 으로 듣는다. UDP Server 로 127.0.0.1 은
BlueOS 가 거부한다). 127.0.0.1 이 안 되면 192.168.2.2 로 하고 --conn 도 맞춘다.
pymavlink 가 없으면 `pip install pymavlink`.

사용 (BlueOS 터미널):
    python3 rpi_transit_probe.py --conn udpin:127.0.0.1:14560 --seconds 60 \\
        --out /tmp/rpi_transit.csv
끝나면 CSV 를 랩톱으로 가져와 `runtime/analysis/transit_compare.py <bag> <csv>
--offset-laptop <diag_link_rtt 값>` 으로 같은 메시지끼리 맞춰 두 구간을 가른다.
첫 줄에 RPi↔FC 시계 offset 을 적어 두므로(TIMESYNC), 랩톱 offset 만 있으면 된다.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--conn", default="udpin:127.0.0.1:14560")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="rpi_transit.csv")
    args = ap.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink 가 없다: pip install pymavlink")

    m = mavutil.mavlink_connection(args.conn)
    print(f"{args.conn} 대기 (heartbeat)...")
    m.wait_heartbeat(timeout=30)

    # RPi 시계 − FC 시계 offset. 이것이 없으면 d(RPi) 는 상수 offset 을 품어
    # 랩톱의 d 와 뺄 수 없다 (랩톱과 RPi 의 wall clock 은 동기가 아니다).
    # 랩톱 diag_link_rtt 와 같은 방식(TIMESYNC, 최소-RTT 표본)이고 부호도 같다:
    # 출력값 = wall − FC, 즉 d − offset 이 절대 편도. 보내는 것은 TIMESYNC 뿐
    # (override 없음), 전용 localhost endpoint 라 랩톱 경로와 경쟁하지 않는다.
    rtts, offs = [], []
    for _ in range(20):
        ts1 = time.monotonic_ns(); wall = time.time_ns()
        m.mav.timesync_send(0, ts1)
        deadline = time.time() + 1.0
        while time.time() < deadline:
            r = m.recv_match(type="TIMESYNC", blocking=True, timeout=0.2)
            if r is None:
                continue
            if int(r.tc1) != 0 and int(r.ts1) == ts1:
                rtt = time.monotonic_ns() - ts1
                rtts.append(rtt / 1e6)
                offs.append((wall + rtt / 2 - int(r.tc1)) / 1e9)   # wall − FC
                break
        time.sleep(0.05)
    if len(offs) < 5:
        sys.exit(f"TIMESYNC 응답 부족({len(offs)}/20) — endpoint 가 FC 와 양방향인지 확인")
    best = min(range(len(rtts)), key=lambda i: rtts[i])
    offset = offs[best]
    print(f"RPi↔FC TIMESYNC RTT 최소 {rtts[best]:.2f} ms  → offset(RPi wall − FC) {offset:+.6f} s")
    print(f"기록 시작 — {args.seconds:.0f} s → {args.out}")

    rows = []
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        msg = m.recv_match(
            type=["SERVO_OUTPUT_RAW", "ATTITUDE_QUATERNION", "ATTITUDE"],
            blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        # FC 시계 도장: SERVO 는 time_usec[µs], ATTITUDE 계열은 time_boot_ms[ms]
        fc_s = (msg.time_usec / 1e6 if t == "SERVO_OUTPUT_RAW"
                else msg.time_boot_ms / 1e3)
        rows.append((t, f"{fc_s:.6f}", f"{time.time():.6f}"))

    with open(args.out, "w", newline="") as f:
        f.write(f"# rpi_offset_s {offset:.6f}\n")     # transit_compare.py 가 읽는다
        w = csv.writer(f)
        w.writerow(["msg_type", "fc_stamp_s", "rpi_wall_s"])
        w.writerows(rows)
    print(f"완료 — {len(rows)} 표본. 랩톱에서: d(RPi) = rpi_wall − fc_stamp 분포를 "
          "diag_loop_delay --mode transit 의 d(랩톱)과 대조할 것.")


if __name__ == "__main__":
    main()
