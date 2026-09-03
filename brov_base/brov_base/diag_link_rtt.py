#!/usr/bin/env python3
"""M2 — 랩톱 ↔ FC MAVLink 왕복시간 (라우터 포함). ROS 불필요, pymavlink 단독.

지연 분해(`docs/LATENCY_DECOMPOSITION_PLAN.md`)의 링크 측정이다:

    τ_total(80 ms) ≈ τ_link(이 도구) + τ_FC스케줄+양자화(잔차) + τ_actuator(M4)

TIMESYNC(tc1=0) 를 보내면 ArduPilot 이 응답한다 — 왕복이 곧 랩톱↔FC RTT 다.
TIMESYNC 무응답 기체를 위해 PARAM_REQUEST_READ 왕복 측정을 폴백으로 갖는다.
ICMP ping(M1, `ping 192.168.2.2`)과의 차 = 라우터+FC 수신처리 몫.

사용:
    ros2 run brov_base diag_link_rtt                       # 기본 실기 주소
    ros2 run brov_base diag_link_rtt --conn udpin:0.0.0.0:14552   # SITL
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np


def _summary(name: str, rtts_ms: list[float]) -> None:
    a = np.array(rtts_ms)
    print(f"{name}: n={len(a)}  중앙 {np.median(a):6.2f} ms  "
          f"p10 {np.percentile(a, 10):6.2f}  p90 {np.percentile(a, 90):6.2f}  "
          f"최소 {a.min():6.2f}  최대 {a.max():6.2f}")
    if np.percentile(a, 90) > 3 * np.median(a):
        print("  ** p90 이 중앙의 3배 초과 — 링크 jitter 가 크다. 평균만 인용하지 말 것. **")


def measure_timesync(m, rounds: int):
    """MAVLink TIMESYNC 왕복. ts1 에코가 우리 것일 때만 채택한다.

    응답의 tc1 = FC 가 요청을 받은 순간의 **FC 시계**[ns]다. 대칭 링크 가정으로
        offset ≈ tc1 − (ts1 + rtt/2)
    를 표본마다 얻고, **최소-RTT 표본**의 값을 추정치로 쓴다(큐잉이 안 낀
    왕복일수록 대칭 가정이 잘 선다). 이 offset 이 있어야 transit 분해
    (`diag_loop_delay --mode transit --offset`)가 상대 분포를 절대 하행
    시간으로 바꿀 수 있다. 주의: FC 시계는 boot 기준이라 offset 은 크고
    무의미해 보이는 수지만, 상수라는 것만 중요하다.
    """
    rtts, offsets = [], []
    for _ in range(rounds):
        ts1 = time.monotonic_ns()
        # offset 은 **wall clock** 기준으로 잰다 -- transit 분해가 대조할 bag
        # 도착 시각이 wall 이기 때문. RTT 는 monotonic 으로(시계 점프 무관).
        wall_send = time.time_ns()
        m.mav.timesync_send(0, ts1)
        t_deadline = time.time() + 1.0
        while time.time() < t_deadline:
            msg = m.recv_match(type="TIMESYNC", blocking=True, timeout=0.3)
            if msg is None:
                continue
            # 응답은 tc1!=0 이고 ts1 이 우리가 보낸 값 그대로다.
            if int(msg.tc1) != 0 and int(msg.ts1) == ts1:
                rtt_ns = time.monotonic_ns() - ts1
                rtts.append(rtt_ns / 1e6)
                offsets.append((int(msg.tc1) - (wall_send + rtt_ns / 2)) / 1e9)
                break
        time.sleep(0.05)
    return rtts, offsets


def measure_param(m, rounds: int, name: str = "RC_SPEED") -> list[float]:
    rtts = []
    for _ in range(rounds):
        t0 = time.monotonic_ns()
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode(), -1)
        deadline = time.time() + 1.0
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if msg is not None and msg.param_id.strip("\x00") == name:
                rtts.append((time.monotonic_ns() - t0) / 1e6)
                break
        time.sleep(0.05)
    return rtts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--conn", default="udpout:192.168.2.2:14550",
                    help="MAVLink 연결 문자열 (기본: 실기 BlueOS)")
    ap.add_argument("--rounds", type=int, default=50)
    args = ap.parse_args()

    from pymavlink import mavutil

    print(f"연결 {args.conn} ...")
    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat(timeout=20)
    print(f"heartbeat OK (sys {m.target_system})")

    ts, offs = measure_timesync(m, args.rounds)
    if len(ts) >= args.rounds // 2:
        _summary("TIMESYNC RTT", ts)
        best = int(np.argmin(ts))
        unc = (np.percentile(ts, 90) - ts[best]) / 2.0
        print(f"시계 offset (FC boot − 랩톱 wall): {offs[best]:+.6f} s "
              f"± {unc:.1f} ms  [최소-RTT 표본 기준]")
        print(f"  → diag_loop_delay --mode transit --offset {-offs[best]:.6f}")
    else:
        print(f"TIMESYNC 응답 부족({len(ts)}/{args.rounds}) — param 왕복으로 폴백")
        pr = measure_param(m, args.rounds)
        if not pr:
            sys.exit("param 왕복도 실패 — 링크를 확인할 것")
        _summary("PARAM RTT (폴백; TIMESYNC 보다 처리 몫이 더 낀다)", pr)

    print("\n해석: 이 값이 τ_link(왕복). ICMP ping 과의 차 = 라우터+FC 수신처리.")
    print("      τ_total(교차상관 80 ms) − τ_link − M4 = FC 스케줄+양자화 잔차.")


if __name__ == "__main__":
    main()
