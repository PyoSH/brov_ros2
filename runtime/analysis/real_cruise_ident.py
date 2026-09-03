#!/usr/bin/env python3
"""실기 청정 bag 에서 (1) 순항점 Jacobian, (2) 2 Hz 스펙트럼 기준선.

POOL_EXPERIMENTS_20260902-03.md §6 "부족한 것" 두 항목을 닫는다:
- 순항점 Jacobian 을 실기 bag 으로 잰 적이 없다 — 지금까지의 K_p(문턱 대비)는
  IsaacLab/SITL 관측 위의 값이었다. 이득은 동작점 의존이 실측돼 있으므로
  (정지 −1.9 vs 순항 −4.5 사례) **실기 관측 분포 위에서** 재야 확정이다.
- 2 Hz 스펙트럼 기준선 — 다음 재학습(지연 DR 확대)의 전후 비교 기준.

방법
====
Jacobian: bag 의 /brov/observation(16-D)을 header 시각으로 /brov/cmd/wrench 와
짝짓고, 기록 action(= wrench / (SCALE·T6), runtime clip ±1 반영)이 정책 재생과
일치하는지 검증한 뒤(재생 게이트), 순항 관측 표본에서 autograd 로
K_p = ∂a_i/∂v_e_i 를 잰다. 정책은 배포 번들 그대로 로드한다.

스펙트럼: a1_band 와 같은 수학(1.8~2.6 Hz 대역 절대 RMS, 50 Hz 격자) —
명령(힘/토크)과 응답(속도/각속도의 미분) 6 축.

사용 (bluerov2_sitl 컨테이너, ROS+torch):
    python3 runtime/analysis/real_cruise_ident.py \\
        artifacts/policies/sim2swim_delayA_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \\
        runtime/bags/<bag> [<bag2> ...]
"""
import sys

import numpy as np
import torch

sys.path.insert(0, "brov_base")
from brov_base.diag_loop_delay import _iter_messages  # noqa: E402

WRENCH_SCALE = np.array([85.0, 85.0, 120.0, 26.0, 14.0, 22.0])
T6 = np.array([1.0, -1.0, -1.0, 1.0, -1.0, -1.0])
# 축별 유효질량/관성 (a1_band 와 동일 출처: 강체 + added mass)
AX = [("surge", 0), ("sway", 1), ("heave", 2), ("roll", 3), ("pitch", 4), ("yaw", 5)]
KP_THRESH_80MS = 3.52          # τ=80 ms 문턱 (heave 기준 정규화 이득)


def read_bag(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    it = _iter_messages(path)
    types = next(it)
    obs, wr, st, act = [], [], [], []
    for topic, data, stamp in it:
        t = stamp * 1e-9
        if topic == "/brov/observation":
            m = deserialize_message(data, get_message(types[topic]))
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            obs.append([hdr, t, float(m.valid), *list(m.data)[:16]])
        elif topic == "/brov/cmd/wrench":
            m = deserialize_message(data, get_message(types[topic]))
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            wr.append([hdr, t, m.force.x, m.force.y, m.force.z,
                       m.torque.x, m.torque.y, m.torque.z])
        elif topic == "/brov/state":
            m = deserialize_message(data, get_message(types[topic]))
            st.append([t, m.linear_velocity.x, m.linear_velocity.y,
                       m.linear_velocity.z, m.angular_velocity.x,
                       m.angular_velocity.y, m.angular_velocity.z])
        elif topic == "/brov/control_active":
            m = deserialize_message(data, get_message(types[topic]))
            act.append([t, float(m.data)])
    return (np.array(obs), np.array(wr), np.array(st), np.array(act))


def band_rms(x, fs, lo=1.8, hi=2.6):
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    m = (f >= lo) & (f <= hi)
    return float(np.sqrt(2 * np.sum(np.abs(X[m]) ** 2) / len(x) ** 2))


def main():
    pol = torch.jit.load(sys.argv[1], map_location="cpu").eval()
    for bag in sys.argv[2:]:
        obs, wr, st, act = read_bag(bag)
        on = act[act[:, 1] > 0.5, 0]
        t0, t1 = on[0] + 10.0, act[act[:, 1] > 0.5, 0][-1]
        print("=" * 74)
        print(f"bag: {bag}   활성 {t1 - t0:.0f}s (앞 10 s 제외)")

        # ── 재생 게이트: obs↔wrench 를 header 로 짝짓고 clip 반영해 대조 ──
        O = obs[(obs[:, 1] >= t0) & (obs[:, 1] <= t1) & (obs[:, 2] > 0.5)]
        W = wr[(wr[:, 1] >= t0) & (wr[:, 1] <= t1)]
        A = W[:, 2:8] / (WRENCH_SCALE * T6)
        j = np.searchsorted(O[:, 0], W[:, 0])
        j = np.clip(j, 1, len(O) - 1)
        pick = np.where(np.abs(O[j - 1, 0] - W[:, 0]) <= np.abs(O[j, 0] - W[:, 0]),
                        j - 1, j)
        dt = np.abs(O[pick, 0] - W[:, 0])
        good = dt < 0.021
        Om, Am = O[pick[good], 3:19], A[good]
        with torch.no_grad():
            R = pol(torch.tensor(Om, dtype=torch.float32)).numpy()
        err = np.abs(np.clip(R, -1, 1) - Am).max()
        clip_frac = float(np.mean(np.abs(R) > 1.0))
        print(f"재생 게이트: n={len(Om)}  max|dA|={err:.2e}  raw clip {100*clip_frac:.1f}%"
              f"   {'통과' if err < 1e-4 else '** 실패 — 이하 무효 **'}")

        # ── 순항점 Jacobian (autograd) ──
        step = max(1, len(Om) // 400)
        X = torch.tensor(Om[::step], dtype=torch.float32)
        Js = []
        for i in range(len(X)):
            x = X[i:i + 1].clone().requires_grad_(True)
            J = torch.autograd.functional.jacobian(
                lambda z: pol(z).squeeze(0), x, vectorize=True)
            Js.append(J.squeeze().detach().numpy())
        J = np.stack(Js)
        print(f"순항점 Jacobian ({len(J)} 표본)   [τ=80 ms 문턱 {KP_THRESH_80MS}]")
        print(f"{'축':>6} {'K_p 중앙':>9} {'IQR':>17} {'K_i(z_v)':>9} {'문턱 대비':>9}")
        for a in range(3):
            kp = J[:, a, 4 + a]
            ki = J[:, a, 10 + a]
            med = np.median(kp)
            print(f"{AX[a][0]:>6} {med:9.3f} [{np.percentile(kp,25):7.2f},"
                  f"{np.percentile(kp,75):6.2f}] {np.median(ki):9.3f} "
                  f"{100*(abs(med)/KP_THRESH_80MS-1):+8.0f}%")

        # ── 2 Hz 스펙트럼 기준선 (a1_band 수학) ──
        fs = 50.0
        grid = np.arange(t0, t1, 1 / fs)
        Wt = wr[:, 1]
        Sv = st[(st[:, 0] >= t0 - 1) & (st[:, 0] <= t1 + 1)]
        print(f"2 Hz 대역 (1.8~2.6) 절대 RMS — 명령 / 응답")
        for name, i in AX:
            cmd = np.interp(grid, Wt, wr[:, 2 + i])
            resp_v = np.interp(grid, Sv[:, 0], Sv[:, 1 + i])
            resp_a = np.gradient(resp_v, grid)
            unit_c = "N" if i < 3 else "N·m"
            unit_r = "m/s²" if i < 3 else "rad/s²"
            print(f"  {name:6s} {band_rms(cmd, fs):6.2f} {unit_c:4s}"
                  f"   {band_rms(resp_a, fs):6.3f} {unit_r}")


if __name__ == "__main__":
    main()
