#!/usr/bin/env python3
"""깊이 게이트 — 어느 baro 가 물속 센서이고, EKF 수직 위치를 믿을 수 있는가.

무엇을 왜 재는가
================
두 가지를 한 번에 판정한다.

1. **어느 SCALED_PRESSURE 가 depth sensor 인가.** ArduSub 는 `BARO_TYPE_WATER`
   인 첫 instance 를 primary 로 잡는데(ArduSub/system.cpp:108), 그 인덱스는
   probe 순서에 달렸고 SITL 과 실기가 다르다. **추론하지 않고 셋 다 재서**
   수직 이동에 반응한 쪽을 물속 센서로 선언한다.

2. **EKF 수직 위치가 실제 깊이를 따라오는가.** 2026-08-29 SITL 에서
   `LOCAL_POSITION_NED.z` 가 초기값에 얼어붙었다 -- 기체가 GT 기준 5.8 m
   상승하는 동안 ±0.1 m 를 보고했다. 같은 메시지의 `vz` 는 상승을 정확히
   알고 있었으므로 속도는 맞고 위치만 적분되지 않는 상태였다. guidance 의
   수직 LOS 항이 그 값으로 오차를 계산하므로 보정이 전혀 안 나갔고, 폐루프가
   부력 드리프트를 7.8 배 증폭해 기체가 1.77 m 떠올랐다. **수조 깊이 여유는
   0.7 m 다.** 같은 증상이면 수초 만에 수면 또는 바닥에 닿는다.

왜 "정확히 1 m" 를 요구하지 않는가
==================================
수조의 z 안전 영역이 0.20~0.90 m 다. **1 m 를 내릴 수가 없고**, 0.5 m 를 손으로
정확히 재는 것도 현실적이지 않다.

그럴 필요가 없다. 재려는 것은 "EKF 가 진짜 깊이를 따라오는가" 이고, 진짜 깊이의
기준자는 이미 손에 있다 -- **압력이다.** ArduSub 자신의 변환식(AP_Baro.cpp:888)
으로 물속 baro 의 압력 변화를 미터로 바꾸고, `depth_ekf` 를 그것에 대해
**회귀**한다.

    depth_ekf = a * depth_baro + b

`a ~ 1` 이고 잔차가 작으면 EKF 가 따라오는 것이다. `a ~ 0` 이면 SITL 에서 본
얼어붙음이다. `a` 가 1 에서 멀면 배율이 틀린 것이고, `a < 0` 이면 부호가
뒤집힌 것이다. **셋 다 다른 고장이고, 두 점만으로는 구분되지 않는다.**

이 방식이 검증하지 **못하는** 것은 압력 자체의 절대 배율이다. 그건 `--drop` 으로
알려진 거리를 줬을 때만 선다(§두 지점 방식). 다만 논문 5.2 도, `depth_source:=
pressure` 도 그 변환식을 그대로 쓰므로, 실제로 필요한 판정은 위의 상대 비교다.

사용법
======
주행 스택이 떠 있는 상태에서 돌린다. `base_node` 가 MAVLink 를 단독 소유하므로
이 도구는 **토픽만 읽는다.**

    ros2 run brov_base diag_depth_gate

Enter 를 누르고, 기체를 안전 영역 안에서 **위아래로 천천히 몇 번** 움직인다
(왕복 2~3 회면 충분하다. 절대 위치는 아무래도 좋고 폭만 있으면 된다).

알려진 거리를 낼 수 있다면 압력의 절대 배율까지 함께 검증한다::

    ros2 run brov_base diag_depth_gate --drop 0.50
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import threading

# ArduSub 자신의 변환식(AP_Baro.cpp:888)이 쓰는 상수.
#     altitude = (ground_pressure - pressure) / 9800 / SPEC_GRAV   [Pa]
# 담수(SPEC_GRAV=1.0)에서 1 m 당 9800 Pa = 98.0 hPa 다.
_PA_PER_M_FRESH = 9800.0

# 내부(기체 안) baro 는 수심에 거의 반응하지 않는다. 물속 센서와의 차이가
# 자릿수로 벌어지므로 문턱은 느슨해도 된다 -- 애매하면 그렇다고 말한다.
_WATER_MIN_FRACTION = 0.5     # 기대 변화량의 절반 이상이면 물속으로 본다
_DRY_MAX_FRACTION = 0.1       # 10% 미만이면 내부로 본다

# sweep 방식에서 "움직였다" 고 볼 최소 압력 폭. 5 cm = 490 Pa 이고, Bar30 의
# 잡음은 그보다 자릿수가 작다. 이보다 작으면 수직 이동이 부족했던 것이다.
_MIN_SPAN_PA = 400.0


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


# --------------------------------------------------------------- 순수 판정
def span(values: list[float]) -> float:
    """peak-to-peak. 표본이 부족하면 0."""
    return (max(values) - min(values)) if len(values) >= 2 else 0.0


def linear_fit(x: list[float], y: list[float]) -> dict:
    """``y = a x + b`` 최소제곱. numpy 없이 -- 시험이 가벼워진다."""
    n = len(x)
    if n < 3 or n != len(y):
        return {"slope": None, "intercept": None, "r2": None, "rms_residual": None}
    mx, my = statistics.fmean(x), statistics.fmean(y)
    sxx = sum((v - mx) ** 2 for v in x)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    if sxx <= 0.0:
        return {"slope": None, "intercept": None, "r2": None, "rms_residual": None}
    slope = sxy / sxx
    intercept = my - slope * mx
    residuals = [b - (slope * a + intercept) for a, b in zip(x, y)]
    ss_res = sum(r * r for r in residuals)
    ss_tot = sum((b - my) ** 2 for b in y)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else None
    return {"slope": slope, "intercept": intercept, "r2": r2,
            "rms_residual": math.sqrt(ss_res / n)}


def identify_responding_instances(
    traces: dict[int, list[float]], min_span_pa: float = _MIN_SPAN_PA
) -> dict:
    """수직 이동에 반응한 instance 를 고른다.

    절대 거리를 모르므로 기울기로는 판정할 수 없다. 그래도 **물속 센서와 내부
    baro 는 폭이 자릿수로 갈린다** -- 0.5 m 만 움직여도 물속은 4900 Pa, 내부는
    수 Pa 다. 그 분리만 쓴다.
    """
    spans = {i: span(v) for i, v in traces.items()}
    responding = sorted(i for i, s in spans.items() if s >= min_span_pa)
    return {"spans_pa": spans, "responding": responding,
            "received": sorted(i for i, v in traces.items() if v)}


def pressure_to_depth_m(
    pressure_pa: list[float], spec_grav: float
) -> list[float]:
    """압력을 **상대** 깊이로. 기준은 평균이라 아래가 +다 (NED 와 같은 부호)."""
    if not pressure_pa:
        return []
    reference = statistics.fmean(pressure_pa)
    scale = _PA_PER_M_FRESH * spec_grav
    return [(p - reference) / scale for p in pressure_pa]


def analyse_sweep(
    depth_baro_m: list[float],
    depth_ekf_m: list[float],
    *,
    slope_tolerance: float = 0.15,
    min_r2: float = 0.90,
    min_span_m: float = 0.05,
) -> dict:
    """EKF 깊이를 압력 깊이에 회귀해 판정한다.

    셋을 구분한다: 따라옴 / 얼어붙음 / 배율·부호 오류. 두 점 방식은 이 셋을
    구분하지 못한다.
    """
    fit = linear_fit(depth_baro_m, depth_ekf_m)
    baro_span = span(depth_baro_m)
    result = {**fit, "baro_span_m": baro_span, "ekf_span_m": span(depth_ekf_m)}
    if fit["slope"] is None:
        result["verdict"] = "표본 부족"
        return result
    if baro_span < min_span_m:
        # 움직이지 않았으면 어떤 판정도 근거가 없다. 통과시키지 않는다.
        result["verdict"] = (
            f"판정 불가 — 수직 이동이 {baro_span * 100:.1f} cm 뿐이다 "
            f"(최소 {min_span_m * 100:.0f} cm)")
        return result
    slope, r2 = fit["slope"], fit["r2"]
    if abs(slope) < 0.2:
        result["verdict"] = "FAIL — EKF 가 깊이를 따라오지 않는다 (SITL 과 같은 얼어붙음)"
    elif slope < 0.0:
        result["verdict"] = "FAIL — EKF 깊이의 **부호가 반대**다"
    elif r2 is not None and r2 < min_r2:
        result["verdict"] = f"FAIL — 상관이 낮다 (R^2 {r2:.3f})"
    elif abs(slope - 1.0) > slope_tolerance:
        result["verdict"] = f"FAIL — 배율이 틀리다 (기울기 {slope:.3f})"
    else:
        result["verdict"] = "PASS — mavlink_ekf 유지 가능"
    return result


def analyse_instance(
    shallow: list[float], deep: list[float], drop_m: float, spec_grav: float
) -> dict:
    """두 지점 방식 -- 알려진 거리가 있을 때만 쓴다. 압력의 절대 배율을 본다."""
    expected = _PA_PER_M_FRESH * spec_grav * drop_m
    shallow_mean, deep_mean = _mean(shallow), _mean(deep)
    if shallow_mean is None or deep_mean is None:
        return {"verdict": "수신 없음", "delta_pa": None, "slope_pa_per_m": None,
                "shallow_pa": shallow_mean, "deep_pa": deep_mean,
                "expected_pa": expected, "noise_pa": None}
    delta = deep_mean - shallow_mean
    slope = delta / drop_m if drop_m else float("nan")
    ratio = abs(delta) / expected if expected else 0.0
    if ratio >= _WATER_MIN_FRACTION:
        verdict = "WATER (깊이센서)"
    elif ratio <= _DRY_MAX_FRACTION:
        verdict = "dry/internal"
    else:
        # 중간값은 판정하지 않는다. 애매한 것을 물속이라고 부르면 그 오차가
        # 깊이 전체에 실린다.
        verdict = "판정 불가"
    noise = None
    if len(shallow) > 1 and len(deep) > 1:
        noise = max(statistics.pstdev(shallow), statistics.pstdev(deep))
    return {"verdict": verdict, "delta_pa": delta, "slope_pa_per_m": slope,
            "shallow_pa": shallow_mean, "deep_pa": deep_mean,
            "expected_pa": expected, "noise_pa": noise}


def resample(
    source_t: list[float], source_v: list[float], target_t: list[float]
) -> list[float]:
    """`source` 를 `target` 시각으로 선형 보간한다.

    압력과 depth_ekf 는 서로 다른 토픽·주기로 온다. 짝을 짓지 않고 회귀하면
    두 신호의 위상차가 기울기에 그대로 섞인다.
    """
    if len(source_t) < 2 or not target_t:
        return []
    out = []
    j = 0
    for t in target_t:
        while j + 2 < len(source_t) and source_t[j + 1] < t:
            j += 1
        t0, t1 = source_t[j], source_t[j + 1]
        v0, v1 = source_v[j], source_v[j + 1]
        if t1 == t0:
            out.append(v0)
        else:
            w = (t - t0) / (t1 - t0)
            out.append(v0 + w * (v1 - v0))
    return out


# ------------------------------------------------------------------ 보고
class Trace:
    """시각이 붙은 표본."""

    def __init__(self) -> None:
        self.pressure_t: dict[int, list[float]] = {0: [], 1: [], 2: []}
        self.pressure_v: dict[int, list[float]] = {0: [], 1: [], 2: []}
        self.depth_t: list[float] = []
        self.depth_v: list[float] = []


def _report_sweep(trace: Trace, spec_grav: float, args) -> int:
    found = identify_responding_instances(trace.pressure_v)
    print("\n  instance  토픽                        표본   압력 폭 [Pa]   깊이 폭 [cm]   판정")
    for i in range(3):
        topic = f"/brov/sensor/pressure{i}"
        n = len(trace.pressure_v[i])
        if not n:
            print(f"  {i:8d}  {topic:26s} {0:6d} {'':>14} {'':>14}   수신 없음")
            continue
        s = found["spans_pa"][i]
        cm = 100.0 * s / (_PA_PER_M_FRESH * spec_grav)
        verdict = ("WATER (깊이센서)" if i in found["responding"]
                   else "dry/internal")
        print(f"  {i:8d}  {topic:26s} {n:6d} {s:14.1f} {cm:14.1f}   {verdict}")

    responding = found["responding"]
    if len(responding) != 1:
        print("\n  결론")
        if not responding:
            print(f"    - 반응한 instance 가 없다. 수직 이동이 부족했거나"
                  f" (최소 {_MIN_SPAN_PA / _PA_PER_M_FRESH * 100:.0f} cm 필요)"
                  f" 센서가 물에 잠기지 않았다.")
        else:
            print(f"    - instance {responding} 가 모두 반응했다. 판별이 안 된다"
                  " (SITL 이 그랬다). BARO_PRIMARY 를 근거로 삼을 것.")
        return 1

    water = responding[0]
    baro_t = trace.pressure_t[water]
    baro_depth = pressure_to_depth_m(trace.pressure_v[water], spec_grav)
    ekf_depth = resample(trace.depth_t, trace.depth_v, baro_t)
    if not ekf_depth:
        print("\n  /brov/sensor/depth_ekf 를 받지 못했다 — EKF 판정 불가")
        return 1
    ekf_ref = statistics.fmean(ekf_depth)
    ekf_depth = [v - ekf_ref for v in ekf_depth]

    r = analyse_sweep(baro_depth, ekf_depth,
                      slope_tolerance=args.slope_tolerance, min_r2=args.min_r2)
    print(f"\n  EKF 깊이 vs 압력 깊이 (instance {water} 를 기준자로)")
    print(f"    표본 {len(baro_depth)},  압력 깊이 폭 {r['baro_span_m'] * 100:.1f} cm"
          f",  EKF 깊이 폭 {r['ekf_span_m'] * 100:.1f} cm")
    if r["slope"] is not None:
        print(f"    depth_ekf = {r['slope']:+.3f} * depth_baro {r['intercept']:+.3f}"
              f"   R^2 {r['r2']:.4f}   잔차 RMS {r['rms_residual'] * 100:.1f} cm")
    print(f"    {r['verdict']}")

    print("\n  결론")
    print(f"    - 물속 센서는 instance {water} 다. FC 의 BARO_PRIMARY 가 같은 값인지"
          " 반드시 대조할 것 —")
    print("      `depth_source:=pressure` 로 launch 하고 `/brov/prepare_control` 을"
          " 부르면 로그에 확정값이 찍힌다.")
    if r["verdict"].startswith("PASS"):
        print("    - EKF 수직 위치가 압력을 따라온다. `depth_source:=mavlink_ekf` 유지.")
        return 0
    print("    - EKF 수직 위치를 믿을 수 없다. `depth_source:=pressure` 로 넘길 것"
          " (docs/DEPTH_SOURCE.md).")
    return 1


def _report_two_station(shallow: Trace, deep: Trace, drop_m: float,
                        spec_grav: float, args) -> int:
    expected = _PA_PER_M_FRESH * spec_grav * drop_m
    print(f"\n하강 {drop_m:.2f} m,  SPEC_GRAV {spec_grav:.3f} "
          f"-> 물속 센서 기대 변화 {expected:8.0f} Pa "
          f"({_PA_PER_M_FRESH * spec_grav / 100.0:.1f} hPa/m)")
    print("\n  instance  토픽                        dP [Pa]   기울기 [Pa/m]   잡음     판정")
    water = []
    for i in range(3):
        r = analyse_instance(shallow.pressure_v[i], deep.pressure_v[i],
                             drop_m, spec_grav)
        topic = f"/brov/sensor/pressure{i}"
        if r["delta_pa"] is None:
            print(f"  {i:8d}  {topic:26s} {'':>9} {'':>15} {'':>8}  {r['verdict']}")
            continue
        noise = f"{r['noise_pa']:7.1f}" if r["noise_pa"] is not None else "      -"
        print(f"  {i:8d}  {topic:26s} {r['delta_pa']:9.0f} "
              f"{r['slope_pa_per_m']:15.0f} {noise}  {r['verdict']}")
        if r["verdict"].startswith("WATER"):
            water.append(i)

    shallow_ekf = _mean(shallow.depth_v)
    deep_ekf = _mean(deep.depth_v)
    print("\n  EKF 수직 위치 (/brov/sensor/depth_ekf, NED — 아래가 +)")
    if shallow_ekf is None or deep_ekf is None:
        print("    수신 없음")
        return 1
    delta = deep_ekf - shallow_ekf
    print(f"    얕음 {shallow_ekf:+.3f} m  ->  깊음 {deep_ekf:+.3f} m"
          f"   변화 {delta:+.3f} m  (기대 {drop_m:+.3f} m)")
    ok = abs(delta - drop_m) <= args.ekf_tolerance
    print(f"    {'PASS — mavlink_ekf 유지 가능' if ok else 'FAIL'}")

    print("\n  결론")
    if len(water) == 1:
        print(f"    - 물속 센서는 instance {water[0]} 다. "
              "BARO_PRIMARY 와 대조할 것.")
    elif not water:
        print("    - 물속으로 판정된 instance 가 없다. 하강 거리를 확인할 것.")
    else:
        print(f"    - instance {water} 가 모두 반응했다. BARO_PRIMARY 를 근거로 삼을 것.")
    if not ok:
        print("    - EKF 수직 위치를 믿을 수 없다. `depth_source:=pressure` 로 "
              "넘길 것 (docs/DEPTH_SOURCE.md).")
        return 1
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--drop", type=float, default=None,
        help="두 지점 방식으로 전환한다. 실제로 내린 거리 [m]. 생략하면 "
             "거리를 모르는 채로 압력을 기준자로 삼는 sweep 방식이다.")
    parser.add_argument("--spec-grav", type=float, default=1.0,
                        help="BARO_SPEC_GRAV. 담수 1.0 / 해수 1.024")
    parser.add_argument("--seconds", type=float, default=40.0,
                        help="sweep 방식에서 기록할 시간 [s]")
    parser.add_argument("--station-seconds", type=float, default=6.0,
                        help="두 지점 방식에서 지점마다 모을 시간 [s]")
    parser.add_argument("--slope-tolerance", type=float, default=0.15,
                        help="회귀 기울기가 1 에서 이만큼 안이면 통과")
    parser.add_argument("--min-r2", type=float, default=0.90,
                        help="회귀 R^2 최소값")
    parser.add_argument("--ekf-tolerance", type=float, default=0.15,
                        help="두 지점 방식에서 EKF 깊이 변화 허용 오차 [m]")
    args = parser.parse_args()
    if args.drop is not None and (args.drop <= 0.0 or not math.isfinite(args.drop)):
        raise SystemExit("--drop 은 양수여야 한다")

    import time

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import FluidPressure
    from std_msgs.msg import Float32

    rclpy.init()
    node = Node("brov_diag_depth_gate")
    current: Trace | None = None
    lock = threading.Lock()

    def on_pressure(index):
        def callback(message: FluidPressure) -> None:
            with lock:
                if current is not None:
                    current.pressure_t[index].append(time.monotonic())
                    current.pressure_v[index].append(float(message.fluid_pressure))
        return callback

    def on_depth(message: Float32) -> None:
        with lock:
            if current is not None:
                current.depth_t.append(time.monotonic())
                current.depth_v.append(float(message.data))

    for i in range(3):
        node.create_subscription(
            FluidPressure, f"/brov/sensor/pressure{i}", on_pressure(i), 50)
    node.create_subscription(Float32, "/brov/sensor/depth_ekf", on_depth, 50)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def collect(prompt: str, seconds: float) -> Trace:
        nonlocal current
        input(f"\n{prompt} Enter: ")
        trace = Trace()
        with lock:
            current = trace
        print(f"  {seconds:.0f} s 기록 중 ...")
        time.sleep(seconds)
        with lock:
            current = None
        counts = ", ".join(f"p{i}={len(trace.pressure_v[i])}" for i in range(3))
        print(f"  표본 {counts}, depth_ekf={len(trace.depth_v)}")
        return trace

    try:
        print("깊이 게이트 — /brov/sensor/* 만 읽는다 (MAVLink 를 열지 않는다).")
        if args.drop is None:
            print("sweep 방식: 압력을 기준자로 삼는다. 거리를 잴 필요가 없다.")
            trace = collect(
                f"기록을 시작하면 {args.seconds:.0f} s 동안 기체를 안전 영역 안에서\n"
                "  위아래로 천천히 2~3 회 움직이십시오. 준비되면", args.seconds)
            status = _report_sweep(trace, args.spec_grav, args)
        else:
            shallow = collect("기체를 얕은 기준 깊이에 정지시키고",
                              args.station_seconds)
            deep = collect(f"정확히 {args.drop:.2f} m 더 깊게 내리고",
                           args.station_seconds)
            status = _report_two_station(shallow, deep, args.drop,
                                         args.spec_grav, args)
    except (KeyboardInterrupt, EOFError):
        status = 130
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(status)


if __name__ == "__main__":
    main()
