"""drag_test 순수 로직 시험 — rclpy/기체 없이 돈다."""

import math
import statistics
import random

import pytest

from brov_control.drag_test import (
    Limits,
    SteadyDetector,
    build_level_plans,
    coast_fit,
    lsq_slope,
    recirculation_check,
    wrap_pi,
)


# 물리 상수 — brov2_heavy.yaml
MASS_EFF_SURGE = 14.635 + 6.36      # mass + added_mass[0]
SIM_XU, SIM_XUU = 13.7, 141.0
TAU_MAX = 123.0                      # 15.3 V 실측


def _limits(**overrides):
    base = dict(
        run_x_min=0.50, run_x_max=2.60, lane_y=0.85,
        max_cross_track_m=0.35, z_min=0.20, z_max=0.90,
        target_z=0.58, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )
    base.update(overrides)
    return Limits(**base)


# ---------------------------------------------------------------- lsq_slope
@pytest.mark.parametrize("true_slope", [0.0, 0.05, -0.13, 0.4])
def test_lsq_slope_recovers_noiseless_ramp(true_slope):
    ts = [i / 25.0 for i in range(30)]
    us = [0.5 + true_slope * t for t in ts]
    assert lsq_slope(ts, us) == pytest.approx(true_slope, abs=1e-9)


def test_lsq_slope_degenerate_inputs():
    assert lsq_slope([], []) == float("inf")
    assert lsq_slope([1.0], [0.5]) == float("inf")
    # 모든 t가 같으면 기울기를 정의할 수 없다
    assert lsq_slope([2.0, 2.0, 2.0], [0.1, 0.2, 0.3]) == float("inf")


def test_lsq_slope_beats_endpoint_difference_under_noise():
    """짧은 창에서 최소자승이 오기각을 크게 줄이는 것이 이 선택의 근거다."""
    ts = [i / 25.0 for i in range(30)]
    span = ts[-1] - ts[0]
    sigma, thresh, trials = 0.02, 0.03, 3000
    rng = random.Random(20260827)

    rej_endpoint = rej_lsq = 0
    for _ in range(trials):
        us = [0.5 + rng.gauss(0.0, sigma) for _ in ts]
        if abs((us[-1] - us[0]) / span) >= thresh:
            rej_endpoint += 1
        if abs(lsq_slope(ts, us)) >= thresh:
            rej_lsq += 1

    # 진짜 정상상태인데 기각한 비율
    assert rej_lsq / trials < 0.05
    assert rej_endpoint / trials > 4 * (rej_lsq / trials)


# ----------------------------------------------------------- SteadyDetector
def _feed(det, u_of_t, duration=3.0, rate=25.0, tau=40.0, noise=0.0, seed=1):
    rng = random.Random(seed)
    n = int(duration * rate)
    for i in range(n):
        t = i / rate
        det.add(t, u_of_t(t) + rng.gauss(0.0, noise), tau)


def test_steady_detector_accepts_settled_velocity():
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    _feed(det, lambda t: 0.48, noise=0.015)
    out = det.evaluate()
    assert out["steady"]
    assert out["u_mps"] == pytest.approx(0.48, abs=0.01)
    assert out["tau_x_n"] == pytest.approx(40.0)
    assert out["window_s"] == pytest.approx(1.16, abs=0.05)


def test_steady_detector_rejects_still_accelerating():
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    _feed(det, lambda t: 0.2 * t)          # 계속 가속
    out = det.evaluate()
    assert not out["steady"]
    assert "du/dt" in out["reason"]


def test_steady_detector_rejects_noisy_hold():
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    _feed(det, lambda t: 0.5, noise=0.12, seed=7)
    out = det.evaluate()
    assert not out["steady"]
    assert "sd" in out["reason"]


def test_steady_detector_needs_minimum_tail():
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    det.add(0.0, 0.5, 40.0)
    det.add(0.04, 0.5, 40.0)
    out = det.evaluate()
    assert not out["steady"]
    assert "표본 부족" in out["reason"]


def test_steady_detector_uses_only_the_tail_window():
    """가속 구간이 앞에 있어도 마지막 창이 정착했으면 유효한 표본이다.

    거리 한계로 중단된 주행이 그대로 쓸 수 있는 근거.
    """
    det = SteadyDetector(window_s=1.0, max_slope=0.03, max_sd=0.05)
    _feed(det, lambda t: min(0.6, 0.6 * (1.0 - math.exp(-t / 0.15))),
          duration=3.0, noise=0.01)
    out = det.evaluate()
    assert out["steady"]
    assert out["u_mps"] == pytest.approx(0.6, abs=0.02)


# ------------------------------------------------------------------ Limits
def test_limits_pass_inside_box():
    assert _limits().violation(1.5, 0.85, 0.58, 0.0, 0.0) is None


@pytest.mark.parametrize("x,y,z,roll,pitch,needle", [
    (0.40, 0.85, 0.58, 0.0, 0.0, "주행축"),
    (2.70, 0.85, 0.58, 0.0, 0.0, "주행축"),
    (1.50, 1.40, 0.58, 0.0, 0.0, "차선"),
    (1.50, 0.85, 0.15, 0.0, 0.0, "깊이 한계"),
    (1.50, 0.85, 0.20, 0.0, 0.0, "깊이 이탈"),
    (1.50, 0.85, 0.58, math.radians(40), 0.0, "자세"),
    (1.50, 0.85, 0.58, 0.0, math.radians(-35), "자세"),
])
def test_limits_catch_each_violation(x, y, z, roll, pitch, needle):
    reason = _limits().violation(x, y, z, roll, pitch)
    assert reason is not None and needle in reason


def test_depth_box_matches_115cm_tank():
    """수심 1.15 m에서 중앙 배치 시 상하 여유가 실제로 남는가."""
    half_height = 0.254 / 2
    lim = _limits(target_z=0.575, max_z_error_m=0.30, z_min=0.20, z_max=0.90)
    # 한계까지 갔을 때 바닥/수면 여유
    lowest = lim.target_z - lim.max_z_error_m - half_height
    highest = lim.target_z + lim.max_z_error_m + half_height
    assert lowest > 0.10          # 바닥에서 10 cm 이상
    assert highest < 1.15 - 0.10  # 수면에서 10 cm 이상


# ------------------------------------------------------------------ wrap_pi
@pytest.mark.parametrize("raw,expected", [
    (0.0, 0.0), (math.pi / 2, math.pi / 2),
    (3 * math.pi, -math.pi), (-3 * math.pi, -math.pi),
    (2 * math.pi + 0.3, 0.3),
])
def test_wrap_pi(raw, expected):
    assert wrap_pi(raw) == pytest.approx(expected, abs=1e-9)


# -------------------------------------------------------------- level plans
def test_level_plans_alternate_direction_and_always_go_forward():
    """역추력이 약해 전/후진을 섞으면 적합이 오염된다 — 항상 뱃머리를 돌린다."""
    lim = _limits()
    plans = build_level_plans([0.10, 0.20, 0.32], lim, axis_heading=0.0,
                              margin_m=0.15)
    assert [p.forward for p in plans] == [True, False, True]
    assert plans[0].heading == pytest.approx(0.0)
    assert abs(wrap_pi(plans[1].heading - math.pi)) < 1e-9
    # 정방향은 낮은 x에서, 역방향은 높은 x에서 출발
    assert plans[0].start_x == pytest.approx(lim.run_x_min + 0.15)
    assert plans[1].start_x == pytest.approx(lim.run_x_max - 0.15)


def test_level_plans_start_inside_limits():
    lim = _limits()
    for p in build_level_plans([0.1, 0.2, 0.32, 0.45, 0.6], lim, 0.0, 0.15):
        assert lim.run_x_min <= p.start_x <= lim.run_x_max


# -------------------------------------------------------------- coast_fit
def _simulate_coast(xu, xuu, u0, sample_dt=0.04, duration=3.0,
                    noise=0.0, seed=3):
    """중립 추력에서의 감속 궤적.

    적분 스텝은 샘플 주기와 분리한다. 가설 A의 시상수가 0.083 s라 dt=0.04의
    전진오일러로 적분하면 **시뮬레이터 자체가** 20% 넘게 틀린다 — 적합이 아니라
    시험 도구가 만드는 오차다. 기체는 연속계이므로 25 Hz는 샘플링에만 쓴다.
    """
    rng = random.Random(seed)
    dt_int = 2e-4
    u, t, out, acc = u0, 0.0, [], 0.0
    for _ in range(int(duration / dt_int)):
        if acc <= 1e-9:
            out.append((t, u + rng.gauss(0.0, noise)))
            acc = sample_dt
        u += dt_int * (-(xu * u + xuu * u * abs(u))) / MASS_EFF_SURGE
        t += dt_int
        acc -= dt_int
    return out


def test_coast_fit_recovers_xuu_with_known_xu():
    """정상상태 적합의 Xu를 넣으면 Xuu가 정확히 복원된다 — 권장 경로."""
    fit = coast_fit(_simulate_coast(SIM_XU, SIM_XUU, u0=0.85),
                    MASS_EFF_SURGE, xu_known=SIM_XU)
    assert fit["ok"]
    assert fit["Xuu"] == pytest.approx(SIM_XUU, rel=0.05)
    assert fit["xu_identifiable"] is False
    assert fit["xu_source"] == "steady_state"


def test_coast_fit_recovers_hypothesis_b_xuu():
    fit = coast_fit(_simulate_coast(13.7, 46.4, u0=1.30),
                    MASS_EFF_SURGE, xu_known=13.7)
    assert fit["ok"]
    assert fit["Xuu"] == pytest.approx(46.4, rel=0.05)


def test_coast_fit_xuu_survives_realistic_velocity_noise():
    """EKF 속도잡음 σ=0.02에서도 Xuu는 쓸 수 있어야 한다 — 이게 교차검증의 값어치."""
    for xu, xuu, u0 in [(SIM_XU, SIM_XUU, 0.85), (13.7, 46.4, 1.30)]:
        errs = []
        for seed in range(1, 16):
            fit = coast_fit(
                _simulate_coast(xu, xuu, u0, noise=0.02, seed=seed),
                MASS_EFF_SURGE, xu_known=xu)
            assert fit["ok"]
            errs.append(abs(fit["Xuu"] - xuu) / xuu)
        assert statistics.fmean(errs) < 0.08


def test_coast_fit_separates_the_two_hypotheses_under_noise():
    """타행만으로도 141 vs 46이 갈려야 독립 교차검증이라 할 수 있다."""
    a = coast_fit(_simulate_coast(SIM_XU, SIM_XUU, 0.85, noise=0.02),
                  MASS_EFF_SURGE, xu_known=SIM_XU)
    b = coast_fit(_simulate_coast(13.7, 46.4, 1.30, noise=0.02),
                  MASS_EFF_SURGE, xu_known=13.7)
    assert a["Xuu"] > 100.0
    assert b["Xuu"] < 70.0


def test_coast_fit_flags_xu_as_unidentifiable_when_solved_for():
    """xu_known 없이 부르면 Xu를 내주되 믿지 말라고 표시해야 한다."""
    fit = coast_fit(_simulate_coast(SIM_XU, SIM_XUU, u0=0.85, noise=0.02),
                    MASS_EFF_SURGE)
    assert fit["ok"]
    assert fit["xu_identifiable"] is True
    assert fit["xu_source"] == "coast_2param"
    # 실제로 못 믿는다는 것 자체를 고정해 둔다 — 이 값을 보고에 쓰면 안 된다.
    assert abs(fit["Xu"] - SIM_XU) / SIM_XU > 0.20


def test_coast_fit_rejects_too_few_samples():
    assert not coast_fit([(0.0, 0.5), (0.04, 0.4)], MASS_EFF_SURGE)["ok"]


def test_coast_fit_rejects_near_zero_velocity_only():
    """u<=0.05는 버려진다 — 잡음이 신호를 압도하는 구간."""
    samples = [(i * 0.04, 0.01) for i in range(50)]
    assert not coast_fit(samples, MASS_EFF_SURGE)["ok"]


# ------------------------------------------------------ recirculation_check
def test_recirculation_check_needs_a_repeat():
    out = recirculation_check([
        {"level": 0.10, "steady": True, "u_mps": 0.25},
        {"level": 0.20, "steady": True, "u_mps": 0.37},
    ])
    assert not out["checked"]


def test_recirculation_check_passes_on_consistent_repeat():
    out = recirculation_check([
        {"level": 0.10, "steady": True, "u_mps": 0.250},
        {"level": 0.20, "steady": True, "u_mps": 0.370},
        {"level": 0.10, "steady": True, "u_mps": 0.254},
    ])
    assert out["checked"] and not out["biased"]


def test_recirculation_check_flags_drift():
    out = recirculation_check([
        {"level": 0.10, "steady": True, "u_mps": 0.250},
        {"level": 0.10, "steady": True, "u_mps": 0.290},   # +16%
    ])
    assert out["checked"] and out["biased"]
    assert out["worst_level"] == pytest.approx(0.10)


def test_recirculation_check_ignores_unsteady_samples():
    out = recirculation_check([
        {"level": 0.10, "steady": True, "u_mps": 0.250},
        {"level": 0.10, "steady": False, "u_mps": 0.900},
    ])
    assert not out["checked"]


# ------------------------------------------- 적합 정본과의 연결 (회귀 방지)
def test_shared_fit_recovers_simulation_coefficients():
    """brov_base의 fit_drag가 적합의 단일 정본임을 확인한다."""
    from brov_base.diag_terminal_velocity import fit_drag

    samples = []
    for level in (0.10, 0.20, 0.32, 0.45, 0.60):
        tau = level * TAU_MAX
        u = (-SIM_XU + math.sqrt(SIM_XU ** 2 + 4 * SIM_XUU * tau)) / (2 * SIM_XUU)
        samples.append({"steady": True, "u_mps": u, "tau_x_n": tau,
                        "tau_x_max_n": TAU_MAX})
    fit = fit_drag(samples)
    assert fit["ok"]
    assert fit["Xu"] == pytest.approx(SIM_XU, rel=1e-6)
    assert fit["Xuu"] == pytest.approx(SIM_XUU, rel=1e-6)
    assert fit["v_max_mps"] == pytest.approx(0.884, abs=0.01)
    assert fit["A_ratio"] == pytest.approx(0.340, abs=0.005)


def test_shared_fit_separates_the_two_hypotheses():
    """이 시험이 답해야 하는 판정이 실제로 갈리는지."""
    from brov_base.diag_terminal_velocity import fit_drag

    def synth(xu, xuu):
        out = []
        for level in (0.10, 0.20, 0.32, 0.45, 0.60):
            tau = level * TAU_MAX
            u = (-xu + math.sqrt(xu ** 2 + 4 * xuu * tau)) / (2 * xuu)
            out.append({"steady": True, "u_mps": u, "tau_x_n": tau,
                        "tau_x_max_n": TAU_MAX})
        return fit_drag(out)

    a = synth(SIM_XU, SIM_XUU)
    b = synth(13.7, 46.4)
    assert a["v_max_mps"] < 0.95        # 가설 A 판정 경계
    assert b["v_max_mps"] > 1.15        # 가설 B 판정 경계
