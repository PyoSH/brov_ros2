#!/usr/bin/env python3
"""G4 분해 ④ — RPi 관측점. **로봇의 BlueOS 터미널에서** 실행한다.

44 ms 잔차가 FC→RPi(직렬)에 있는지 RPi→랩톱(라우터·이더넷)에 있는지 가른다:
같은 메시지의 (FC 시계 도장, RPi 도착 시각)을 여기서 기록하고, 랩톱 bag 의
(FC 도장, 랩톱 도착)과 대조하면

    FC→RPi 몫 = d(RPi)        RPi→랩톱 몫 = d(랩톱) − d(RPi)

이 답이 곧 **온보드 이전의 가치 판정**이다 — 잔차가 FC→RPi 쪽이면 온보드로
못 줄인다(그 구간을 그대로 지난다).

준비: BlueOS → MAVLink Endpoints 에 localhost 용 udpin 서버를 하나 추가
(예: udpin 127.0.0.1:14560). pymavlink 가 없으면 `pip install pymavlink`.

사용 (BlueOS 터미널):
    python3 rpi_transit_probe.py --conn udpin:127.0.0.1:14560 --seconds 60 \\
        --out /tmp/rpi_transit.csv
끝나면 CSV 를 랩톱으로 가져와 랩톱 bag 의 transit 결과와 나란히 놓는다.
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
        w = csv.writer(f)
        w.writerow(["msg_type", "fc_stamp_s", "rpi_wall_s"])
        w.writerows(rows)
    print(f"완료 — {len(rows)} 표본. 랩톱에서: d(RPi) = rpi_wall − fc_stamp 분포를 "
          "diag_loop_delay --mode transit 의 d(랩톱)과 대조할 것.")


if __name__ == "__main__":
    main()
