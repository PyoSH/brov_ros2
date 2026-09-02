#!/usr/bin/env python3
"""배포 루프의 dead time 측정 — 명령이 효과를 내기까지 걸리는 시간.

무엇을 왜 재는가
================
정책은 IsaacLab 에서 학습됐고 거기엔 dead time 이 **구조적으로 없다**. 환경이
같은 프로세스 안에서 action 을 물리에 적용하고 그 직후 관측을 읽으므로,
`obs_{t+1}` 에는 `a_t` 의 효과가 이미 들어 있다. 배포는 다르다:

    base_node -> MAVLink -> (BlueOS 라우팅) -> ArduSub -> servo
      -> 추진기 -> 기체 -> telemetry -> base_node

이 왕복 시간이 dead time 이다. 순수 시간지연의 전달함수는 ``e^(-s*tau)`` 로
**크기는 모든 주파수에서 정확히 1** 이고 위상만 ``-w*tau`` 로 커진다. 저역통과
같은 lag 은 위상을 더하는 대신 이득도 깎지만, dead time 은 이득을 하나도 깎지
않고 위상만 더한다. 그래서 위험하다.

폐루프가 어떤 주파수에서 스스로 진동하려면 그 주파수에서 루프 이득이 1 이상이고
총 위상이 -180도 여야 한다. 위상 예산은:

    기체(관성 지배 영역)   -90도            (힘 -> 속도는 적분이다)
    dead time              -360*f*tau
    ZOH (제어 주기 T)      -360*f*T/2
    컨트롤러               정책은 실측 -1~-5도 (거의 비례 제어)

기체의 -90도는 주파수와 무관하게 고정이고 dead time 과 ZOH 만 주파수에 비례해
커지므로, 총합이 -180도 에 닿는 주파수가 하나 정해진다:

    f = (90 - |phi_c|) / (360 * (tau + T/2))

2026-08-31 Gazebo SITL 측정: tau=60ms 에서 예측 2.99 Hz, 실측 3.00 Hz.
tau=80ms(mavproxy 경유)에서는 실측 1.98 Hz 였다. **dead time 을 줄이면 진동
주파수가 올라간다** -- 위상으로 제한된 루프의 서명이다.

이득 조건은 고주파에서 기체 크기가 ``1/(m_eff*w)`` 이므로

    |L(w)| = |K_p| * WRENCH_SCALE / (m_eff * w)

이고, ``|L| = 1`` 이 되는 문턱이 ``|K_p| = m_eff*w/SCALE`` 이다. 학습된 정책의
실측 이득은 K_p ~ 4.5 로, tau=60ms 의 문턱 4.4 에 걸쳐 있다.

jitter 도 함께 본다
===================
평균 지연만큼이나 **지연이 매번 얼마나 흔들리는지**가 중요하다. SITL 에서
mavproxy 를 거칠 때 교차상관 최대값이 0.487 이었고 직결에서 0.934 였다 --
평균 지연은 80->60ms 로 조금 줄었을 뿐인데 상관이 두 배가 됐다. 낮은 상관은
"고정 지연으로 설명되지 않는 성분이 크다", 즉 timing jitter 를 뜻한다.
실기도 BlueOS 라우팅을 거치므로 같은 서명이 나올 수 있다. 그래서 이 도구는
지연 피크와 상관계수를 **함께** 보고한다. 하나만 보면 오독한다.

사용법
======
주행을 bag 으로 남기고(``split_stack.launch.py record_bag:=true``) 분석한다::

    ros2 run brov_base diag_loop_delay <bag_경로>
    ros2 run brov_base diag_loop_delay <bag_경로> --axis surge --m-eff 21.0
    ros2 run brov_base diag_loop_delay <bag_경로> --axis yaw --open-loop

각축(roll/pitch/yaw)은 ``wrench.torque`` 와 ``state.angular_velocity`` 를 읽는다.
각속도는 자이로에서 **직접** 오므로 EKF 속도 융합의 필터 지연이 섞이지 않는다 --
yaw 여기(``deadtime_test.launch.py axis:=yaw bias:=1.0 amplitude:=0.5``)로 재면
통신+추진기 지연만 깨끗하게 남고, 추진기가 0 을 넘지 않아 ESC 역전 지연도 빠진다.

기록에 ``/brov/cmd/wrench`` 와 ``/brov/state`` 가 **같은 시계로** 있어야 한다.
둘 다 같은 프로세스 계열에서 발행되므로 bag 의 수신 시각을 쓴다.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

# 유효질량 = 강체질량 + added mass (brov2_heavy.yaml). 축별로 다르다.
# 각축은 유효관성 = 강체 관성 + added inertia [kg*m^2]. 토크 -> 각가속도도
# 적분 하나라 위상 예산의 -90도가 그대로 성립한다.
_M_EFF = {
    "surge": 14.635 + 6.36, "sway": 14.635 + 7.12, "heave": 14.635 + 13.5,
    "roll": 0.289 + 0.189, "pitch": 0.329 + 0.135, "yaw": 0.337 + 0.222,
}
_WRENCH_SCALE = {"surge": 85.0, "sway": 85.0, "heave": 120.0,
                 "roll": 26.0, "pitch": 14.0, "yaw": 22.0}
# 각축은 wrench.torque / state.angular_velocity 의 같은 인덱스를 읽는다.
_AXIS_INDEX = {"surge": 0, "sway": 1, "heave": 2, "roll": 0, "pitch": 1, "yaw": 2}
_ANGULAR = {"roll", "pitch", "yaw"}


def _iter_messages(path: str):
    """(topic, raw_bytes, stamp_ns) 를 내놓는다.

    rosbag2 를 먼저 쓰되, 실패하면 **sqlite3 로 직접 읽는다.** 기록기가 정상
    종료되지 못하면 ``metadata.yaml`` 이 없고 파일이 잠겨 있어 rosbag2 가 열지
    못한다 -- 현장에서 정전·크래시로 충분히 생기는 상황이고, 그렇다고 측정을
    통째로 버릴 이유는 없다. db3 안에는 필요한 것이 다 들어 있다.
    """
    try:
        import rosbag2_py

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        yield types
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            yield topic, data, stamp
        return
    except Exception as exc:                      # noqa: BLE001 -- 폴백이 목적
        print(f"  (rosbag2 로 열지 못해 sqlite3 직접 읽기로 폴백: {exc})")

    import glob
    import os
    import sqlite3

    db = path
    if os.path.isdir(path):
        cand = sorted(glob.glob(os.path.join(path, "*.db3")))
        if not cand:
            raise SystemExit(f"{path} 안에 .db3 가 없다")
        db = cand[0]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    types = {name: typ for _id, name, typ in
             con.execute("select id, name, type from topics")}
    ids = {tid: name for tid, name, _ in
           con.execute("select id, name, type from topics")}
    yield types
    for tid, stamp, data in con.execute(
            "select topic_id, timestamp, data from messages order by timestamp"):
        yield ids[tid], bytes(data), stamp


def read_bag(path: str):
    """bag 에서 (t, wrench_force, t, body_velocity, t, control_active) 를 뽑는다.

    시각은 **bag 의 수신 시각**을 쓴다. 두 토픽이 같은 시계에서 기록되므로
    상대 시차가 보존되고, 그것이 교차상관에 필요한 전부다.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    it = _iter_messages(path)
    types = next(it)
    for need in ("/brov/cmd/wrench", "/brov/state"):
        if need not in types:
            raise SystemExit(f"bag 에 {need} 가 없다. 기록된 토픽: {sorted(types)}")

    wr_t, wr_f, st_t, st_v, act_t, act_v = [], [], [], [], [], []
    for topic, data, stamp in it:
        t = stamp * 1e-9
        if topic == "/brov/cmd/wrench":
            m = deserialize_message(data, get_message(types[topic]))
            wr_t.append(t)
            # [force(3), torque(3)] -- 각축 분석이 뒤의 셋을 읽는다.
            wr_f.append([m.force.x, m.force.y, m.force.z,
                         m.torque.x, m.torque.y, m.torque.z])
        elif topic == "/brov/state":
            m = deserialize_message(data, get_message(types[topic]))
            st_t.append(t)
            # [linear(3), angular(3)]. 각속도는 자이로에서 직접 오므로 EKF 의
            # 속도 융합 지연이 섞이지 않는다 -- yaw 로 재면 통신+추진기 지연만 남는다.
            st_v.append([m.linear_velocity.x, m.linear_velocity.y, m.linear_velocity.z,
                         m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        elif topic == "/brov/control_active":
            m = deserialize_message(data, get_message(types[topic]))
            act_t.append(t)
            act_v.append(bool(m.data))
    return (np.array(wr_t), np.array(wr_f),
            np.array(st_t), np.array(st_v),
            np.array(act_t), np.array(act_v, dtype=bool))


def read_bag_fc(path: str):
    """M3/M4 용 원시 신호를 뽑는다.

    반환:
      wrench:  (t_arrival, torque_z)
      servo:   (t_arrival, t_fc, pwm[8])   -- t_fc = header.stamp (FC boot 시계)
      gyro:    (t_arrival, t_fc, omega[3])
    servo/ahrs 의 header.stamp 는 base_node 가 FC boot 시계로 찍는다
    (SERVO_OUTPUT_RAW.time_usec / ATTITUDE.time_boot_ms). 같은 시계이므로 둘의
    header 끼리 교차상관(M4)하면 링크 지연이 전혀 안 낀다.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    it = _iter_messages(path)
    types = next(it)
    need = ("/brov/cmd/wrench", "/brov/sensor/servo_out", "/brov/sensor/ahrs")
    for topic in need:
        if topic not in types:
            raise SystemExit(
                f"bag 에 {topic} 가 없다 (M3/M4 는 2026-09-02 servo 배선 이후의 "
                f"bag 이 필요하다). 기록된 토픽: {sorted(types)}")

    wr, sv, gy = [], [], []
    for topic, data, stamp in it:
        t = stamp * 1e-9
        if topic == "/brov/cmd/wrench":
            m = deserialize_message(data, get_message(types[topic]))
            wr.append([t, m.torque.z])
        elif topic == "/brov/sensor/servo_out":
            m = deserialize_message(data, get_message(types[topic]))
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            sv.append([t, hdr, *list(m.position)[:8]])
        elif topic == "/brov/sensor/ahrs":
            m = deserialize_message(data, get_message(types[topic]))
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            gy.append([t, hdr, m.angular_velocity.x, m.angular_velocity.y,
                       m.angular_velocity.z])
    return np.array(wr), np.array(sv), np.array(gy)


def xcorr_delay(t_x, x, t_y, y, *, max_lag_s=0.30, grid_dt=0.005):
    """y 가 x 보다 얼마나 늦는지 — 공통 격자 리샘플 후 교차상관.

    반환 (lag_s, r, lags_s, r_profile). 단위시험이 이 함수를 직접 검증한다.
    """
    t0 = max(t_x[0], t_y[0])
    t1 = min(t_x[-1], t_y[-1])
    if t1 - t0 < 3.0:
        raise SystemExit(f"공통 구간이 너무 짧다: {t1 - t0:.2f}s")
    grid = np.arange(t0, t1, grid_dt)
    xs = np.interp(grid, t_x, x); xs = xs - xs.mean()
    ys = np.interp(grid, t_y, y); ys = ys - ys.mean()
    lags = np.arange(0, int(round(max_lag_s / grid_dt)) + 1)
    cc = []
    for L in lags:
        a = xs[: len(xs) - L] if L else xs
        b = ys[L:]
        cc.append(np.corrcoef(a, b)[0, 1] if len(a) > 10 else np.nan)
    cc = np.array(cc)
    best = int(np.nanargmax(np.abs(cc)))
    return lags[best] * grid_dt, float(cc[best]), lags * grid_dt, cc


def _pick_servo_channel(pwm):
    """여기(excitation)를 실은 채널 = 분산 최대 채널."""
    return int(np.argmax(pwm.std(axis=0)))


def analyse_m3(wr, sv, skip_s=5.0):
    """M3: 명령(wrench 도착) -> 서보 출력(도착). τ_up + τ_FC + τ_down."""
    t0 = max(wr[0, 0], sv[0, 0]) + skip_s
    w = wr[wr[:, 0] >= t0]
    v = sv[sv[:, 0] >= t0]
    ch = _pick_servo_channel(v[:, 2:10])
    lag, r, lags, cc = xcorr_delay(w[:, 0], w[:, 1], v[:, 0], v[:, 2 + ch])
    print(f"M3  명령→서보출력 도착   lag = {lag*1000:5.1f} ms,  r = {r:+.3f}"
          f"   (채널 servo{ch+1}, n={len(w)}/{len(v)})")
    for L, c in zip(lags[:: max(1, len(lags)//10)], cc[:: max(1, len(lags)//10)]):
        print(f"    {L*1000:6.1f} ms : r = {c:+.3f}")
    return lag, r


def analyse_m4(sv, gy, skip_s=5.0):
    """M4: 서보 출력 -> 자이로 각가속 — **FC 시계**. 링크가 전혀 안 낀다.

    τ_actuator(+자이로 샘플링)의 확정 측정이다. 신호는 분산 최대 서보 채널과
    yaw 각가속(자이로 z 미분) — A2-yaw 프로토콜(역전 없음)에 맞춘 선택이다.
    """
    t0f = max(sv[0, 1], gy[0, 1]) + skip_s
    v = sv[sv[:, 1] >= t0f]
    g = gy[gy[:, 1] >= t0f]
    if np.any(np.diff(v[:, 1]) < 0) or np.any(np.diff(g[:, 1]) < 0):
        print("  ** FC 시계 되감김 감지 — FC 재부팅이 낀 bag 이다. 구간을 나눠 볼 것. **")
    dt_ms = float(np.median(np.diff(v[:, 1]))) * 1000.0
    if dt_ms > 60.0:
        print(f"  ** SERVO 스트림이 저속이다 ({dt_ms:.0f} ms 간격) — M4 분해능 "
              f"±{dt_ms/2:.0f} ms 로 측정 불가. **")
        print("     원인: 경로 위의 mavproxy/GCS 가 streamrate 를 덮어쓴 것 "
              "(2026-09-02 SITL 에서 mavproxy 기본 4 Hz 로 확인). 실기에서는 "
              "QGC/Cockpit 을 끊고 재시도할 것.")
    ch = _pick_servo_channel(v[:, 2:10])
    yaw_rate = g[:, 4]
    yaw_acc = np.gradient(yaw_rate, g[:, 1])
    lag, r, lags, cc = xcorr_delay(v[:, 1], v[:, 2 + ch], g[:, 1], yaw_acc)
    print(f"M4  서보→자이로 (FC 시계)  lag = {lag*1000:5.1f} ms,  r = {r:+.3f}"
          f"   (채널 servo{ch+1}, n={len(v)}/{len(g)})")
    for L, c in zip(lags[:: max(1, len(lags)//10)], cc[:: max(1, len(lags)//10)]):
        print(f"    {L*1000:6.1f} ms : r = {c:+.3f}")
    print("  해석: 이 값이 τ_actuator. 학습 주입값 = τ_total − 이 값"
          " (DELAY_TRAINING_PLAN §1-4).")
    return lag, r


def analyse(wr_t, wr_f, st_t, st_v, act_t, act_v, axis: str, m_eff: float,
            control_dt: float, skip_s: float, seconds: float | None = None,
            open_loop: bool = False):
    angular = axis in _ANGULAR
    # 3열 배열(옛 시험·옛 bag 형식)은 선형축만 담는다. 6열이면 뒤의 셋이 각축.
    i = _AXIS_INDEX[axis] + (3 if angular and wr_f.shape[1] == 6 else 0)
    if angular and (wr_f.shape[1] != 6 or st_v.shape[1] != 6):
        raise SystemExit(f"{axis} 축은 torque/angular_velocity 가 필요한데 배열이 3열이다")
    scale = _WRENCH_SCALE[axis]
    unit_m = "kg*m^2" if angular else "kg"

    # control_active 가 True 인 구간만 본다. 그 밖은 명령이 나가지 않는다.
    if len(act_t):
        on = act_t[act_v] if act_v.any() else np.array([])
        if not len(on):
            raise SystemExit("control_active 가 True 인 구간이 없다")
        t0 = on[0] + skip_s
    else:
        t0 = max(wr_t[0], st_t[0]) + skip_s
    t1 = min(wr_t[-1], st_t[-1])
    if seconds is not None:
        # 여기(excitation)가 duration_s 뒤 스스로 멈춰도 control_active 는 계속
        # true 다. 그 뒤의 **명령이 0 인 구간**을 같이 넣으면 상관에 기여하지
        # 않으면서 표본만 늘려 r 을 희석한다. 여기를 넣은 시간만큼 자른다.
        t1 = min(t1, t0 + seconds)
    if t1 - t0 < 5.0:
        raise SystemExit(f"분석 구간이 너무 짧다: {t1 - t0:.1f}s")

    print(f"분석 구간 {t1 - t0:.1f}s  (control_active 후 {skip_s:.0f}s 제외"
          + (f", 이후 {seconds:.0f}s 만 사용" if seconds is not None else "") + ")")
    for name, t in (("/brov/cmd/wrench", wr_t), ("/brov/state", st_t)):
        m = (t >= t0) & (t <= t1)
        dt = np.diff(t[m])
        print(f"  {name:20s} n={m.sum():5d}  dt 중앙 {np.median(dt)*1000:5.1f} ms "
              f"({1/np.median(dt):5.1f} Hz)  p90 {np.percentile(dt, 90)*1000:5.1f} ms")

    # 공통 격자로 리샘플한 뒤 가속도를 만든다. 지연은 명령과 **가속도** 사이에
    # 있다 -- 속도는 그 적분이라 위상이 90도 더 밀려 피크가 흐려진다.
    grid = np.arange(t0, t1, control_dt / 2)
    f = np.interp(grid, wr_t, wr_f[:, i])
    v = np.interp(grid, st_t, st_v[:, i])
    a = np.gradient(v, grid)
    f = f - f.mean()
    a = a - a.mean()

    step = grid[1] - grid[0]
    lags = np.arange(0, int(round(0.30 / step)) + 1)
    cc = []
    for L in lags:
        x = f[: len(f) - L] if L else f
        y = a[L:]
        cc.append(np.corrcoef(x, y)[0, 1] if len(x) > 10 else np.nan)
    cc = np.array(cc)
    best = int(np.nanargmax(np.abs(cc)))
    tau = lags[best] * step

    print(f"\ndead time (명령 -> 가속도 교차상관)")
    print(f"  피크 lag = {tau*1000:5.1f} ms,  r = {cc[best]:+.3f}")
    if abs(cc[best]) < 0.5:
        print("  ** r < 0.5: 고정 지연으로 설명되지 않는 성분이 크다(timing jitter). **")
        print("     평균 지연만 인용하면 오독한다. 라우팅 홉/버퍼링을 의심할 것.")
    print("  lag 프로파일:")
    for L in lags[:: max(1, len(lags) // 10)]:
        k = list(lags).index(L)
        print(f"    {L*step*1000:6.1f} ms : r = {cc[k]:+.3f}")

    # 지배 주파수
    def dominant(x):
        X = np.fft.rfft(x)
        fr = np.fft.rfftfreq(len(x), step)
        P = np.abs(X) ** 2
        if len(P) < 3:
            return float("nan"), 0.0
        j = 1 + int(np.argmax(P[1:]))
        return fr[j], P[j] / max(P[1:].sum(), 1e-12)

    ff, pf = dominant(f)
    fa, pa = dominant(a)
    print(f"\n지배 주파수   명령 {ff:5.2f} Hz (전력비 {100*pf:4.1f}%)   "
          f"응답 {fa:5.2f} Hz (전력비 {100*pa:4.1f}%)")

    # 폐루프 주행의 명령 스펙트럼은 미션 주기(0.04 Hz 같은 저주파)가 지배해
    # 위 줄이 진동을 놓친다 -- 2026-09-02 A1 에서 실제로 그랬다. 진동대(1~5 Hz)
    # 안의 지배 주파수와 **절대** RMS 를 따로 낸다. 전력비는 이득을 바꾸면 분모도
    # 바뀌어 비교가 안 된다.
    def band(x, lo, hi):
        X = np.fft.rfft(x)
        fr = np.fft.rfftfreq(len(x), step)
        m = (fr >= lo) & (fr <= hi)
        if not m.any():
            return float("nan"), 0.0
        P = np.abs(X[m]) ** 2
        rms = math.sqrt(2.0 * P.sum()) / len(x)
        return fr[m][int(np.argmax(P))], rms
    fb, rb = band(f, 1.0, 5.0)
    ab, rab = band(a, 1.0, 5.0)
    unit_f = "N*m" if axis in _ANGULAR else "N"
    unit_a = "rad/s^2" if axis in _ANGULAR else "m/s^2"
    print(f"진동대 1~5 Hz  명령 {fb:5.2f} Hz  RMS {rb:6.2f} {unit_f}   "
          f"응답 {ab:5.2f} Hz  RMS {rab:6.3f} {unit_a}")
    ff_cmp = fb if (not math.isfinite(ff) or ff < 0.2) and math.isfinite(fb) else ff
    if open_loop:
        # 개루프 여기 주행에서는 지배 주파수가 곧 **우리가 넣은 신호**다.
        # 아래 폐루프 비교를 그대로 인용하면 없는 불일치를 보고하게 된다.
        print("  (개루프 — 이 값은 주입한 여기의 주파수이지 폐루프 진동이 아니다)")
        print(f"  지연 분해능의 한계: 여기가 {ff:.2f} Hz 협대역이면 교차상관 "
              f"봉우리 반폭이 ~{1000 / (4 * ff):.0f} ms 다.")
        print("  봉우리가 평평하면 jitter 이기 전에 **대역폭 부족**을 먼저 의심할 것 "
              "— period_s 를 줄이거나 kind:=chirp 로 다시 잰다.")

    # 위상 예산과 예측 진동 주파수
    print(f"\n위상 예산  ({'I_eff' if angular else 'm_eff'} = {m_eff:.3g} {unit_m}, "
          f"제어주기 {control_dt*1000:.0f} ms)")
    phi_c = 4.0                      # 정책 실측 -1~-5도. 보수적으로 4도.
    f_pred = (90.0 - phi_c) / (360.0 * (tau + control_dt / 2))
    print(f"  -180도 교차 예측 주파수 = {f_pred:5.2f} Hz")
    if open_loop:
        print("  (개루프이므로 실측 지배 주파수와의 비교는 하지 않는다 — "
              "폐루프 진동이 아니다)")
    elif math.isfinite(ff_cmp) and ff_cmp > 0.2:
        print(f"  실측 지배 주파수        = {ff_cmp:5.2f} Hz"
              f"   ({'일치' if abs(ff_cmp - f_pred) < 0.5 * f_pred else '불일치 — 다른 위상원 의심'})"
              + ("  [진동대 1~5 Hz 기준]" if ff_cmp != ff else ""))
    w = 2 * math.pi * f_pred
    kp_thresh = m_eff * w / scale
    print(f"  이 주파수에서 |L|=1 이 되는 이득 문턱 K_p = {kp_thresh:5.2f}")
    print(f"    (학습된 정책 실측 K_p ~ 4.5 (surge/heave, 순항 관측) -- 넘으면 진동 조건 성립)")
    for tag, phase in (("기체(관성)", -90.0),
                       ("dead time", -360.0 * f_pred * tau),
                       ("ZOH", -360.0 * f_pred * control_dt / 2),
                       ("정책", -phi_c)):
        print(f"    {tag:12s} {phase:7.1f}도")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag", help="rosbag2 디렉토리")
    ap.add_argument("--axis", default="heave", choices=sorted(_AXIS_INDEX))
    ap.add_argument("--m-eff", type=float, default=None,
                    help="유효질량 [kg] 또는 각축이면 유효관성 [kg*m^2]. "
                         "미지정 시 축별 기본값(강체+added)")
    ap.add_argument("--control-dt", type=float, default=0.04, help="제어 주기 [s]")
    ap.add_argument("--skip", type=float, default=5.0,
                    help="control_active 후 제외할 초기 과도 [s]")
    ap.add_argument("--seconds", type=float, default=None,
                    help="skip 이후 사용할 길이 [s]. deadtime_test 의 duration_s "
                         "에서 --skip 을 뺀 값을 주면, 여기가 끝난 뒤의 "
                         "명령 0 구간이 r 을 희석하는 것을 막는다")
    ap.add_argument("--mode", default="closed", choices=["closed", "m3", "m4"],
                    help="closed=기존 명령→가속 분석. m3=명령→서보출력(도착 시계), "
                         "m4=서보→자이로(FC 시계, 링크 무관) — 지연 분해용 "
                         "(LATENCY_DECOMPOSITION_PLAN.md)")
    ap.add_argument("--open-loop", action="store_true",
                    help="주입한 여기로 잰 주행. 폐루프 진동 전제의 비교를 끄고 "
                         "대신 여기 대역폭이 정하는 지연 분해능을 보고한다")
    args = ap.parse_args()

    m_eff = args.m_eff if args.m_eff is not None else _M_EFF[args.axis]
    if args.mode in ("m3", "m4"):
        print(f"=== 지연 분해 {args.mode.upper()}   bag={args.bag}")
        wr, sv, gy = read_bag_fc(args.bag)
        if args.mode == "m3":
            analyse_m3(wr, sv, skip_s=args.skip)
        else:
            analyse_m4(sv, gy, skip_s=args.skip)
        return
    print(f"=== dead time 진단   bag={args.bag}   축={args.axis}")
    data = read_bag(args.bag)
    analyse(*data, axis=args.axis, m_eff=m_eff,
            control_dt=args.control_dt, skip_s=args.skip,
            seconds=args.seconds, open_loop=args.open_loop)


if __name__ == "__main__":
    sys.exit(main())
