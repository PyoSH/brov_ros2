#!/usr/bin/env python3
"""실기 종단속도(v_max) 및 surge 항력계수 측정.

무엇을 왜 재는가
================
Sim2Swim 보상 Eq.(8) ``r_a = w_a e^(-||a||)``은 **정상상태에서 절대 0이 되지
않는 행동 비용**을 만든다. 속도항 Eq.(6)은 ``e^(-||v_e||^2)``로 오차 0 근방에서
기울기가 0이므로, 두 항의 균형점이 곧 정책이 수렴할 정상상태 속도가 된다.
그 균형점을 결정하는 값은 딱 하나다:

    A = drag(V_d) / K_surge        (V_d를 유지하는 데 드는 정규화 추력)

    A = 0.50 → 보상최적 추종률 19%      A = 0.11 → 60%
    A = 0.20 → 45%                      A = 0.05 → 77%

현재 시뮬레이션은 von Benzon 계수(Xu=13.7, Xuu=141, K=85 N)를 쓰고 있고 이는
A=0.495, 즉 최고속도 0.73 m/s를 함의한다. 제조사 BlueROV2 사양은 1.5 m/s다.
**이 스크립트는 그 2배 불일치를 실측으로 끝낸다.** 결과에 따라 갈린다:

* v_max ≈ 0.73 m/s로 확인 → plant가 맞다. 논문 보상으로는 V_d=0.5를 학습할 수
  없으므로 Eq.(8)을 바꿔야 한다(어떤 V_d를 골라도 11~19%다 — 선형항력 비용
  ``w_a·Xu/K``가 V_d와 무관하게 남기 때문).
* v_max ≈ 1.5 m/s로 확인 → Xuu가 5배 과대다. 계수를 고치는 게 먼저다.

측정 원리
=========
정상상태에서 추력과 항력이 같다. 여러 추력 수준에서 종단속도를 재고

    tau_x = Xu * u + Xuu * u * |u|

를 최소자승 적합한다(원점 통과, 2 파라미터). tau_x는 명령값이 아니라 **실제
전달 추력**을 쓴다 — 명령 wrench를 할당행렬로 8개 추력기에 분배하고, PWM으로
역변환한 뒤, T200 실측 테이블에서 그 PWM/전압이 내는 추력을 다시 읽어 합산한다.
deadband와 clamp가 반영된다.

측정되는 항력은 **전진비(advance ratio) 손실을 포함한 유효 항력**이다. 정지
추력 테이블로 나눈 값이기 때문이다. 시뮬레이션도 정지 추력 테이블을 쓰므로
자기일관적이고, 이 목적(A 결정)에는 오히려 이게 맞다. 전진비를 따로 모델링할
거라면 그때는 분리해서 다시 재야 한다.

속도 출처
=========
MAVLink ``LOCAL_POSITION_NED``의 vel_ned를 body frame으로 회전해 쓴다. 이 값은
ArduSub EKF3 출력이고 DVL이 ``EK3_SRC1_VELXY``로 융합돼 있다. **A50 DVL body
velocity 직결 경로는 아직 미구현이며(layer 1) 그쪽이 논문과 같은 경로다.**
정상상태 직진에서는 EKF 속도가 DVL을 잘 따라가지만, 교차검증으로 같은 구간의
위치 차분 속도도 함께 기록해서 두 값이 어긋나면 경고한다.

안전
====
``diag_thruster_map.py``와 같은 게이트를 쓴다 — MANUAL mode, 명시적 arm,
``--confirm-run``. 추가로 매 스텝 거리/깊이/자세/시간 한계를 검사하고 위반 시
즉시 중립 정지한다. 수평 4기는 open-loop surge + yaw PID, 수직 4기는 깊이/
roll/pitch PID가 잡는다(수평/수직 추력기가 담당 DOF가 겹치지 않는 BlueROV2
Heavy 배치를 그대로 이용).

사용 예
=======
    # 로직만 확인 (기체 연결/무장 없음)
    python3 -m brov_base.diag_terminal_velocity --dry-run

    # 실측. 전압은 시험 직전 실측값을 넣을 것 (T200 추력은 전압 의존)
    python3 -m brov_base.diag_terminal_velocity \\
        --voltage 15.6 --depth 1.0 --levels 0.3 0.45 0.6 0.75 0.9 1.0 \\
        --dwell 8 --max-run-distance 7 --confirm-run --out vmax_run1.json

    # 기록만 다시 적합
    python3 -m brov_base.diag_terminal_velocity --fit-from vmax_run1.json
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time

import torch

from brov_base import math_utils as mu
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.t200_table import T200ThrustTable
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix


# 시뮬레이션 현재값 — 실측 결과와 나란히 보여주기 위한 참조.
_SIM_XU, _SIM_XUU = 13.7, 141.0
_SIM_K_SURGE = 85.0
_PAPER_VD = 0.5

_MANUAL_CUSTOM_MODE = 19


# ----------------------------------------------------------------------
class _PID:
    """자세/깊이 유지용 최소 PID. 적분은 출력 한계에 맞춰 clamp한다."""

    def __init__(self, kp: float, ki: float, kd: float, out_limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = abs(out_limit)
        self._i = 0.0
        self._prev: float | None = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev = None

    def __call__(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        # 적분 한계를 ki로 환산해두면 적분항 단독으로는 출력 한계를 못 넘는다.
        if self.ki > 0.0:
            self._i = max(-self.out_limit / self.ki,
                          min(self.out_limit / self.ki, self._i + error * dt))
        out = self.kp * error + self.ki * self._i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, out))


def _roll_pitch_yaw(q: torch.Tensor) -> tuple[float, float, float]:
    """[w,x,y,z] -> (roll, pitch, yaw) rad. R = Rz(yaw)Ry(pitch)Rx(roll) 규약."""
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ----------------------------------------------------------------------
class _Allocator:
    """명령 wrench -> PWM, 그리고 그 PWM이 실제로 내는 wrench를 되돌려준다."""

    def __init__(self, voltage: float, dt: float = 0.04):
        # thruster_pos_dir_ned()는 list를 반환한다. build_allocation_matrix()는
        # tensor를 요구하므로, model_based_controller.py와 같은 순서로
        # 모델을 먼저 만들고 그 텐서(_pos/_dir)로 할당행렬을 만든다 —
        # 할당행렬과 추력기 모델이 같은 기하를 쓰는 것을 구조적으로 보장한다.
        pos, dir_ = thruster_pos_dir_ned(load_brov2_yaml())
        self.model = BROV2ThrusterModel(
            num_envs=1, dt=dt, device="cpu", pos=pos, dir=dir_, voltage=voltage,
        )
        self.B = build_allocation_matrix(self.model._pos, self.model._dir)
        self.B_pinv = torch.linalg.pinv(self.B)
        self.table = T200ThrustTable(device="cpu")
        self.voltage = torch.full((1,), float(voltage))
        self.max_reverse_n, self.max_forward_n = self.model.force_limits_n
        # 순수 surge로 실제 전달 가능한 최대 추력. level의 기준값이자 v_max
        # 계산의 tau_max이므로 생성 시점에 한 번 확정한다.
        self.surge_max_n = self._measure_surge_capability()
        self.linear_max_n = self._measure_linear_regime()

    def _measure_surge_capability(self, ceiling_n: float = 400.0,
                                  steps: int = 801) -> float:
        """순수 surge로 실제 전달 가능한 최대 추력 [N].

        추력기 한계에서 해석적으로 역산하면 틀린다 — ``force_limits_n``은 **전압
        무관 전역 극값**이라 지금 전압에서 실현 불가능한 값을 준다(15.6V에서
        해석값 139.7 N vs 실제 전달 124.7 N). deadband/PWM 역변환/테이블 비선형도
        더해진다. 그래서 명령을 실제로 할당→역변환→테이블 왕복시켜 전달값의
        최대를 찾는다.
        """
        best = 0.0
        wrench = torch.zeros(6)
        for tau in torch.linspace(0.0, ceiling_n, steps):
            wrench[0] = float(tau)
            _, delivered = self.apply(wrench)
            best = max(best, float(delivered[0]))
        return best

    def _measure_linear_regime(self, margin: float = 0.98) -> float:
        """PWM이 포화하지 않는 구간의 최대 전달 surge [N].

        이 지점을 넘으면 두 가지가 동시에 깨진다. ① 수평 4기가 PWM 한계에
        붙어서 **yaw PID가 권한을 잃는다** — 직진 유지가 안 되면 측정 자체가
        무의미하다. ② clamp_thrust가 분배를 왜곡해 순수 surge가 아니게 된다
        (15.6V level 0.9에서 Fz로 -0.78 N 누설). 그래서 기본 level은 이 구간
        안에 두고, v_max는 적합된 항력곡선을 full thrust까지 외삽해서 얻는다.
        """
        best = 0.0
        wrench = torch.zeros(6)
        for tau in torch.linspace(0.0, self.surge_max_n * 1.2, 481):
            wrench[0] = float(tau)
            pwm, delivered = self.apply(wrench)
            if float(pwm.abs().max()) >= margin:
                break
            best = max(best, float(delivered[0]))
        return best

    def surge_command_for(self, delivered_target: float) -> float:
        """전달 추력이 ``delivered_target``이 되게 하는 명령 wrench [N].

        명령→전달은 항등이 아니다. ``clamp_thrust``와 PWM 역변환/양자화 때문에
        포화 근처에서 어긋난다 — 15.6V에서 125.65 N을 명령하면 117.73 N만 나온다.
        level이 '실제로 낸 추력의 비율'을 뜻하게 하려고 역으로 푼다(단조 구간
        이분법).
        """
        lo, hi = 0.0, 400.0
        wrench = torch.zeros(6)
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            wrench[0] = mid
            _, delivered = self.apply(wrench)
            if float(delivered[0]) < delivered_target:
                lo = mid
            else:
                hi = mid
        return hi

    def apply(self, wrench: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """wrench(6, SNAME/FRD) -> (pwm(8), 실제 전달 wrench(6))."""
        desired = self.B_pinv @ wrench
        desired = self.model.clamp_thrust(desired)
        pwm = self.model.inverse_thrust(desired).clamp(-1.0, 1.0)
        delivered_f = self.table.force(pwm, self.voltage).reshape(-1)
        return pwm, self.B @ delivered_f


# ----------------------------------------------------------------------
def fit_drag(samples: list[dict]) -> dict:
    """tau_x = Xu*u + Xuu*u*|u| 최소자승 (원점 통과)."""
    usable = [s for s in samples if s.get("steady") and abs(s["u_mps"]) > 1e-3]
    if len(usable) < 2:
        return {"ok": False, "reason": f"정상상태 표본 {len(usable)}개 (최소 2개 필요)"}

    u = torch.tensor([s["u_mps"] for s in usable], dtype=torch.float64)
    tau = torch.tensor([s["tau_x_n"] for s in usable], dtype=torch.float64)
    A = torch.stack([u, u * u.abs()], dim=1)
    coef = torch.linalg.lstsq(A, tau.unsqueeze(1)).solution.reshape(-1)
    Xu, Xuu = float(coef[0]), float(coef[1])

    resid = tau - A @ coef
    ss_res = float((resid ** 2).sum())
    ss_tot = float(((tau - tau.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    tau_max = max(s["tau_x_max_n"] for s in usable)
    if Xuu > 1e-9:
        v_max = (-Xu + math.sqrt(max(0.0, Xu * Xu + 4.0 * Xuu * tau_max))) / (2.0 * Xuu)
    elif Xu > 1e-9:
        v_max = tau_max / Xu
    else:
        v_max = float("nan")

    drag_vd = Xu * _PAPER_VD + Xuu * _PAPER_VD ** 2
    return {
        "ok": True,
        "Xu": Xu,
        "Xuu": Xuu,
        "r2": r2,
        "rms_residual_n": math.sqrt(ss_res / len(usable)),
        "n_samples": len(usable),
        "tau_x_max_n": tau_max,
        "v_max_mps": v_max,
        "drag_at_paper_vd_n": drag_vd,
        "A_ratio": drag_vd / tau_max if tau_max > 0 else float("nan"),
    }


def reward_optimal_tracking(Xu: float, Xuu: float, tau_max: float,
                            vd: float = _PAPER_VD,
                            w_v: float = 0.2, w_a: float = 0.3) -> float:
    """논문 보상(Eq.6 제곱 / Eq.8 비제곱)의 정상상태 최적 추종률 [0,1]."""
    if tau_max <= 0.0:
        return float("nan")
    best_u, best_r = 0.0, -1e9
    for i in range(4001):
        u = vd * i / 4000.0
        a = (Xu * u + Xuu * u * u) / tau_max
        r = w_v * math.exp(-(vd - u) ** 2) + w_a * math.exp(-abs(a))
        if r > best_r:
            best_r, best_u = r, u
    return best_u / vd


def report(fit: dict) -> None:
    if not fit["ok"]:
        print(f"[적합 실패] {fit['reason']}")
        return
    print("\n" + "=" * 68)
    print("실측 surge 항력")
    print("=" * 68)
    print(f"  Xu  = {fit['Xu']:8.2f}  N/(m/s)      (시뮬레이션 현재값 {_SIM_XU})")
    print(f"  Xuu = {fit['Xuu']:8.2f}  N/(m/s)^2    (시뮬레이션 현재값 {_SIM_XUU})")
    print(f"  R^2 = {fit['r2']:8.4f}   RMS 잔차 {fit['rms_residual_n']:.2f} N"
          f"   표본 {fit['n_samples']}개")
    print(f"\n  최대 surge 추력 {fit['tau_x_max_n']:.1f} N  →  v_max = {fit['v_max_mps']:.3f} m/s")
    print(f"  (시뮬레이션 현재값이 함의하는 v_max = "
          f"{(-_SIM_XU + math.sqrt(_SIM_XU**2 + 4*_SIM_XUU*_SIM_K_SURGE))/(2*_SIM_XUU):.3f} m/s,"
          f"  제조사 사양 1.5 m/s)")

    A = fit["A_ratio"]
    track = reward_optimal_tracking(fit["Xu"], fit["Xuu"], fit["tau_x_max_n"])
    sim_track = reward_optimal_tracking(_SIM_XU, _SIM_XUU, _SIM_K_SURGE)
    print("\n" + "-" * 68)
    print(f"V_d = {_PAPER_VD} m/s 유지에 필요한 정규화 추력  A = {A:.3f}"
          f"   (항력 {fit['drag_at_paper_vd_n']:.1f} N)")
    print(f"논문 보상의 정상상태 최적 추종률 = {100*track:.0f}%"
          f"   (현재 시뮬레이션 값으로는 {100*sim_track:.0f}%)")
    print("-" * 68)
    if track < 0.5:
        print("→ 이 plant에서는 논문 보상(Eq.8)으로 V_d=0.5를 학습할 수 없다.")
        print("  V_d를 낮춰도 안 된다 — 선형항력 비용 w_a*Xu/K = "
              f"{0.3*fit['Xu']/fit['tau_x_max_n']:.3f} 가 V_d와 무관하게 남는다.")
        print("  Eq.8을 바꿔야 한다(행동 '변화율' 페널티 권장).")
    else:
        print("→ 논문 보상 그대로 학습 가능한 영역. 현재 시뮬레이션 계수가 과대평가다.")
    print("=" * 68 + "\n")


# ----------------------------------------------------------------------
def _run_level(iface, alloc, args, level: float, target_depth: float,
               target_yaw: float, log: list) -> dict:
    """추력 수준 하나를 유지하며 종단속도를 잰다."""
    depth_pid = _PID(args.depth_kp, args.depth_ki, args.depth_kd, args.heave_limit_n)
    roll_pid = _PID(args.att_kp, args.att_ki, args.att_kd, args.moment_limit_nm)
    pitch_pid = _PID(args.att_kp, args.att_ki, args.att_kd, args.moment_limit_nm)
    yaw_pid = _PID(args.att_kp, args.att_ki, args.att_kd, args.moment_limit_nm)

    tau_fwd_max = alloc.surge_max_n
    # level은 '실제 낸 추력의 비율'이다. 명령값은 그렇게 되도록 역산해 고정한다.
    tau_target = level * tau_fwd_max
    surge_cmd = alloc.surge_command_for(tau_target)
    period = 1.0 / args.rate_hz
    t0 = time.monotonic()
    prev = t0
    start_pos = None
    pwm_peak = 0.0
    leak_peak = 0.0
    series: list[tuple[float, float, float]] = []      # (t, u_body, tau_x_delivered)
    abort = None

    while True:
        now = time.monotonic()
        dt = now - prev
        if dt < period:
            time.sleep(period - dt)
            continue
        prev = now
        elapsed = now - t0

        snap = iface.snapshot()
        if snap is None:
            abort = "telemetry 없음"
            break
        if max(snap["att_age_s"], snap["pos_age_s"]) > args.max_telemetry_age:
            abort = f"telemetry 지연 {max(snap['att_age_s'], snap['pos_age_s']):.2f}s"
            break

        q = snap["att_quat_ned"].reshape(4)
        pos = snap["pos_ned"].reshape(3)
        vel_ned = snap["vel_ned"].reshape(3)
        if start_pos is None:
            start_pos = pos.clone()

        v_body = mu.quat_apply(mu.quat_conjugate(q.unsqueeze(0)),
                               vel_ned.unsqueeze(0)).reshape(3)
        u = float(v_body[0])
        roll, pitch, yaw = _roll_pitch_yaw(q)
        depth = float(pos[2])                          # NED: +z = 아래
        travelled = float(torch.linalg.vector_norm(pos - start_pos))

        # ── 안전 한계 ──
        if travelled > args.max_run_distance:
            abort = f"거리 한계 {travelled:.2f} m"
            break
        if abs(depth - target_depth) > args.max_depth_error:
            abort = f"깊이 이탈 {depth - target_depth:+.2f} m"
            break
        if max(abs(roll), abs(pitch)) > math.radians(args.max_tilt_deg):
            abort = f"자세 이탈 roll {math.degrees(roll):+.0f}° pitch {math.degrees(pitch):+.0f}°"
            break
        if elapsed > args.dwell + args.accel_grace:
            break

        # ── wrench 조립: surge는 open-loop, 나머지는 PID ──
        wrench = torch.zeros(6)
        wrench[0] = surge_cmd
        wrench[2] = depth_pid(depth - target_depth, dt)          # NED +z 아래
        wrench[3] = roll_pid(_wrap_pi(0.0 - roll), dt)
        wrench[4] = pitch_pid(_wrap_pi(0.0 - pitch), dt)
        wrench[5] = yaw_pid(_wrap_pi(target_yaw - yaw), dt)

        pwm, delivered = alloc.apply(wrench)
        if not args.dry_run:
            iface.send_pwm(pwm)

        series.append((elapsed, u, float(delivered[0])))
        pwm_peak = max(pwm_peak, float(pwm.abs().max()))
        leak_peak = max(leak_peak, float(delivered[1:3].abs().max()))
        log.append({
            "level": level, "t": elapsed, "u": u, "depth": depth,
            "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw), "travelled": travelled,
            "tau_x_cmd": float(wrench[0]), "tau_x_delivered": float(delivered[0]),
        })

    if not args.dry_run:
        iface.neutral_stop()

    # ── 정상상태 판정: 마지막 settle_window 구간의 |du/dt|와 표준편차 ──
    tail = [s for s in series if s[0] >= max(0.0, series[-1][0] - args.settle_window)] if series else []
    result = {"level": level, "abort": abort, "n": len(series), "steady": False,
              "pwm_peak": pwm_peak, "wrench_leak_peak_n": leak_peak}
    if len(tail) >= 5:
        ts = [s[0] for s in tail]
        us = [s[1] for s in tail]
        taus = [s[2] for s in tail]
        span = ts[-1] - ts[0]
        slope = (us[-1] - us[0]) / span if span > 0 else float("inf")
        sd = statistics.pstdev(us)
        steady = abs(slope) < args.settle_slope and sd < args.settle_sd
        result.update({
            "u_mps": statistics.fmean(us),
            "u_sd": sd,
            "du_dt": slope,
            "tau_x_n": statistics.fmean(taus),
            "tau_x_max_n": tau_fwd_max,
            "pwm_saturated": pwm_peak >= 0.98,
            "steady": steady,
            "window_s": span,
        })
    return result


def _turnaround(iface, alloc, args, target_depth: float, target_yaw: float) -> None:
    """다음 구간을 위해 제자리에서 180° 선회한다(항상 전진 추력만 쓰기 위함).

    T200은 역추력이 정추력보다 약해서(테이블 기준 -51.5N vs +64.1N) 전/후진을
    섞으면 적합이 오염된다. 그래서 되돌아올 때도 뱃머리를 돌려 전진한다.
    """
    if args.dry_run:
        return
    yaw_pid = _PID(args.att_kp, args.att_ki, args.att_kd, args.moment_limit_nm)
    depth_pid = _PID(args.depth_kp, args.depth_ki, args.depth_kd, args.heave_limit_n)
    period = 1.0 / args.rate_hz
    deadline = time.monotonic() + args.turnaround_timeout
    settled_since = None
    while time.monotonic() < deadline:
        snap = iface.snapshot()
        if snap is None:
            break
        q = snap["att_quat_ned"].reshape(4)
        _, _, yaw = _roll_pitch_yaw(q)
        err = _wrap_pi(target_yaw - yaw)
        wrench = torch.zeros(6)
        wrench[2] = depth_pid(float(snap["pos_ned"].reshape(3)[2]) - target_depth, period)
        wrench[5] = yaw_pid(err, period)
        pwm, _ = alloc.apply(wrench)
        iface.send_pwm(pwm)
        if abs(err) < math.radians(args.turnaround_tol_deg):
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since > args.turnaround_hold:
                break
        else:
            settled_since = None
        time.sleep(period)
    iface.neutral_stop()


# ----------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="실기 종단속도/surge 항력 측정",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--fit-from", type=str, default=None,
                   help="기록된 JSON에서 적합만 다시 수행하고 종료")
    p.add_argument("--dry-run", action="store_true",
                   help="기체 연결/무장 없이 상태기계 로직만 확인")
    p.add_argument("--confirm-run", action="store_true",
                   help="실제 추력을 내는 시험을 승인. 없으면 실행 거부")

    p.add_argument("--voltage", type=float, default=None,
                   help="시험 직전 실측 배터리 전압 [V]. T200 추력이 전압에 의존하므로 필수")
    p.add_argument("--levels", type=float, nargs="+",
                   default=[0.25, 0.40, 0.55, 0.70, 0.85],
                   help="surge 추력 수준 — '실제 전달 추력 / 최대 전달 추력' 비율. "
                        "기본값은 PWM 비포화 구간 안에 있다(포화하면 yaw 권한 상실). "
                        "v_max는 적합된 항력곡선의 외삽으로 얻으므로 1.0까지 갈 필요 없다.")
    p.add_argument("--depth", type=float, default=1.0, help="유지 깊이 [m, NED +아래]")
    p.add_argument("--dwell", type=float, default=8.0, help="수준별 유지 시간 [s]")
    p.add_argument("--accel-grace", type=float, default=2.0,
                   help="가속에 허용하는 추가 시간 [s]")
    p.add_argument("--settle-window", type=float, default=3.0,
                   help="정상상태 판정에 쓰는 마지막 구간 [s]")
    p.add_argument("--settle-slope", type=float, default=0.02,
                   help="정상상태 판정 |du/dt| 한계 [m/s^2]")
    p.add_argument("--settle-sd", type=float, default=0.05,
                   help="정상상태 판정 속도 표준편차 한계 [m/s]")
    p.add_argument("--rate-hz", type=float, default=25.0, help="제어 주기 [Hz]")

    p.add_argument("--max-run-distance", type=float, default=7.0,
                   help="수준별 최대 주행 거리 [m] — 수조 길이보다 작게")
    p.add_argument("--max-depth-error", type=float, default=0.5, help="깊이 이탈 한계 [m]")
    p.add_argument("--max-tilt-deg", type=float, default=30.0, help="roll/pitch 한계 [deg]")
    p.add_argument("--max-telemetry-age", type=float, default=0.5,
                   help="telemetry 최대 지연 [s]")

    p.add_argument("--depth-kp", type=float, default=60.0)
    p.add_argument("--depth-ki", type=float, default=8.0)
    p.add_argument("--depth-kd", type=float, default=20.0)
    p.add_argument("--heave-limit-n", type=float, default=60.0)
    p.add_argument("--att-kp", type=float, default=8.0)
    p.add_argument("--att-ki", type=float, default=1.0)
    p.add_argument("--att-kd", type=float, default=3.0)
    p.add_argument("--moment-limit-nm", type=float, default=12.0)

    p.add_argument("--turnaround-timeout", type=float, default=20.0)
    p.add_argument("--turnaround-tol-deg", type=float, default=10.0)
    p.add_argument("--turnaround-hold", type=float, default=1.0)

    p.add_argument("--out", type=str, default="terminal_velocity.json")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.fit_from:
        with open(args.fit_from, encoding="utf-8") as fh:
            data = json.load(fh)
        report(fit_drag(data["levels"]))
        return 0

    if not args.dry_run:
        if not args.confirm_run:
            print("추력을 내는 시험이다. --confirm-run 없이는 실행하지 않는다.", file=sys.stderr)
            return 2
        if args.voltage is None:
            print("--voltage 필수: T200 추력은 전압에 크게 의존한다 "
                  "(같은 PWM에서 10V와 20V의 추력이 2배 이상 차이).", file=sys.stderr)
            return 2
        if any(not 0.0 < lv <= 1.0 for lv in args.levels):
            print("--levels는 (0, 1] 범위여야 한다.", file=sys.stderr)
            return 2

    alloc = _Allocator(args.voltage if args.voltage is not None else 14.8,
                       dt=1.0 / args.rate_hz)
    tau_fwd_max = alloc.surge_max_n
    linear_level = alloc.linear_max_n / tau_fwd_max if tau_fwd_max > 0 else 0.0
    print(f"[설정] 전압 {alloc.voltage.item():.1f} V")
    print(f"       순수 surge 최대 전달 추력  {tau_fwd_max:6.1f} N  (= level 1.00)")
    print(f"       PWM 비포화 한계            {alloc.linear_max_n:6.1f} N  "
          f"(= level {linear_level:.2f})")
    print(f"       수준 {args.levels}")
    hot = [lv for lv in args.levels if lv > linear_level]
    if hot:
        print(f"       [경고] level {hot}은 PWM 포화 구간이다 — 수평 4기가 한계에 붙어")
        print(f"              yaw PID가 권한을 잃고 wrench 누설이 생긴다. 직진 유지가")
        print(f"              안 되면 측정이 무의미하므로 level {linear_level:.2f} 이하를 권한다.")

    if args.dry_run:
        print("[dry-run] 기체 연결 없이 종료. 상태기계/적합은 --fit-from으로 확인할 것.")
        return 0

    from brov_base.mavlink_interface import RealRobotInterface

    log: list[dict] = []
    results: list[dict] = []
    with RealRobotInterface() as iface:
        iface.connect()
        ctrl = iface.control_snapshot()
        if ctrl["custom_mode"] != _MANUAL_CUSTOM_MODE:
            print(f"MANUAL mode가 아니다 (custom_mode={ctrl['custom_mode']}). 중단.",
                  file=sys.stderr)
            return 3
        iface.enable_passthrough()
        iface.arm()

        snap = iface.snapshot()
        if snap is None:
            print("telemetry 없음. 중단.", file=sys.stderr)
            iface.neutral_stop()
            return 3
        _, _, yaw0 = _roll_pitch_yaw(snap["att_quat_ned"].reshape(4))
        target_yaw = yaw0

        try:
            for i, level in enumerate(args.levels):
                print(f"\n[{i+1}/{len(args.levels)}] level {level:.2f} "
                      f"→ 목표 전달 {level*tau_fwd_max:.1f} N, "
                      f"yaw 목표 {math.degrees(target_yaw):+.0f}°")
                res = _run_level(iface, alloc, args, level, args.depth, target_yaw, log)
                results.append(res)
                if res.get("steady"):
                    print(f"    정상상태 u = {res['u_mps']:+.3f} m/s "
                          f"(sd {res['u_sd']:.3f}, du/dt {res['du_dt']:+.3f}), "
                          f"전달 추력 {res['tau_x_n']:.1f} N")
                    if res.get("pwm_saturated"):
                        print(f"    [경고] PWM 포화 (peak {res['pwm_peak']:.3f}), "
                              f"wrench 누설 {res['wrench_leak_peak_n']:.2f} N — "
                              f"yaw 권한이 없었을 수 있다")
                else:
                    reason = res.get("abort") or "정상상태 미도달"
                    print(f"    사용 불가 — {reason}")
                if res.get("abort") and "telemetry" in res["abort"]:
                    break
                if i + 1 < len(args.levels):
                    target_yaw = _wrap_pi(target_yaw + math.pi)
                    print(f"    선회 → {math.degrees(target_yaw):+.0f}°")
                    _turnaround(iface, alloc, args, args.depth, target_yaw)
        finally:
            iface.neutral_stop()
            iface.disarm()

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump({
            "voltage_v": float(alloc.voltage.item()),
            "tau_x_max_n": tau_fwd_max,
            "pwm_saturated": pwm_peak >= 0.98,
            "args": vars(args),
            "levels": results,
            "timeseries": log,
        }, fh, indent=2)
    print(f"\n기록 저장: {args.out}")
    report(fit_drag(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
