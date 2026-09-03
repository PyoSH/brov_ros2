#!/usr/bin/env python3
"""G4 분해 ④ — 같은 메시지의 (FC 도장, RPi 도착, 랩톱 도착) 을 맞춰 편도를 두 구간으로 가른다.

    FC→RPi   = (rpi_wall − fc) − offset_rpi
    RPi→랩톱 = (lap_wall − fc) − offset_lap − FC→RPi
    (offset = wall − FC 시계; RPi 것은 CSV 첫 줄, 랩톱 것은 diag_link_rtt 출력의 --offset 값)

사용:
    python3 runtime/analysis/transit_compare.py <bag> <rpi_transit.csv> --offset-laptop <s>

두 wall clock 은 각각 FC 시계에 TIMESYNC 로 묶여 있으므로 서로 동기일 필요가 없다.
60 s 기록에서 크리스털 drift(수십 ppm)는 수 ms — 구간 판정(수십 ms 등급)에 무관하다.
매칭: servo 는 time_usec(µs) 정확히, ahrs 는 time_boot_ms(ms) 정확히.
"""
import argparse
import csv
import sys

import numpy as np

from brov_base.diag_loop_delay import read_bag_fc


def _load_csv(path):
    offset = None
    rows = {"SERVO_OUTPUT_RAW": [], "ATTITUDE": [], "ATTITUDE_QUATERNION": []}
    with open(path) as f:
        first = f.readline()
        if first.startswith("# rpi_offset_s"):
            offset = float(first.split()[2])
        else:
            f.seek(0)
        for r in csv.DictReader(f):
            rows.setdefault(r["msg_type"], []).append(
                (float(r["fc_stamp_s"]), float(r["rpi_wall_s"])))
    return offset, {k: np.array(v) for k, v in rows.items() if v}


def _match(bag_arr, rpi_arr, tol_s):
    """bag (도착, fc) 와 rpi (fc, 도착) 를 fc 도장으로 맞춘다."""
    fc_r = rpi_arr[:, 0]
    order = np.argsort(fc_r)
    fc_sorted = fc_r[order]
    idx = np.searchsorted(fc_sorted, bag_arr[:, 1])
    idx = np.clip(idx, 0, len(fc_sorted) - 1)
    ok = np.abs(fc_sorted[idx] - bag_arr[:, 1]) <= tol_s
    return bag_arr[ok], rpi_arr[order][idx[ok]]


def _stats(x_ms):
    return (f"중앙 {np.median(x_ms):6.1f}  p10 {np.percentile(x_ms, 10):6.1f}  "
            f"p90 {np.percentile(x_ms, 90):6.1f} ms  (n={len(x_ms)})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("csv")
    ap.add_argument("--offset-laptop", type=float, required=True,
                    help="랩톱 wall − FC [s]; diag_link_rtt 가 '--offset' 뒤에 찍는 값")
    ap.add_argument("--offset-rpi", type=float, default=None,
                    help="CSV 첫 줄에 없을 때만")
    args = ap.parse_args()

    off_rpi, rpi = _load_csv(args.csv)
    if args.offset_rpi is not None:
        off_rpi = args.offset_rpi
    if off_rpi is None:
        sys.exit("RPi offset 이 없다 — 새 프로브(TIMESYNC 포함)로 다시 기록하거나 --offset-rpi")
    _wr, sv, gy = read_bag_fc(args.bag)

    print(f"=== transit 두 구간 분해   bag={args.bag}  csv={args.csv}")
    print(f"    offset  랩톱−FC {args.offset_laptop:+.6f} s   RPi−FC {off_rpi:+.6f} s")
    pairs = [("servo", sv, rpi.get("SERVO_OUTPUT_RAW"), 2e-6)]
    att = rpi.get("ATTITUDE_QUATERNION")
    if att is None:
        att = rpi.get("ATTITUDE")
    pairs.append(("ahrs", gy, att, 6e-4))
    for name, bag_arr, rpi_arr, tol in pairs:
        if rpi_arr is None:
            print(f"  {name}: CSV 에 해당 메시지 없음"); continue
        b, r = _match(bag_arr, rpi_arr, tol)
        if len(b) < 50:
            print(f"  {name}: 맞춘 표본 {len(b)} — 같은 주행의 bag/CSV 인지, FC 재부팅이 없었는지 확인")
            continue
        fc = b[:, 1]
        d_rpi = (r[:, 1] - fc - off_rpi) * 1000
        d_lap = (b[:, 0] - fc - args.offset_laptop) * 1000
        hop = d_lap - d_rpi
        print(f"  {name:6s} FC→RPi   {_stats(d_rpi)}")
        print(f"         RPi→랩톱 {_stats(hop)}")
        print(f"         전체     {_stats(d_lap)}")
        share = np.median(d_rpi) / max(np.median(d_lap), 1e-9)
        where = "FC→RPi (직렬/스케줄 — 온보드 이전으로 못 줄인다)" if share > 0.5 \
            else "RPi→랩톱 (라우터/이더넷 — 온보드 이전·전용 endpoint 가 먹는다)"
        print(f"         → 편도의 {share*100:.0f} % 가 FC→RPi. 상수의 소재: {where}")


if __name__ == "__main__":
    main()
