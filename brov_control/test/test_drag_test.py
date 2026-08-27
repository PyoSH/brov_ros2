"""drag_test 순수 로직 시험 — rclpy/기체 없이 돈다."""

import math
import statistics
import random

import pytest

import json

import numpy as np

from brov_control.dvl_reader import DvlReader
from brov_control.drag_test import (
    Limits,
    Phase,
    SteadyDetector,
    build_level_plans,
    coast_fit,
    lsq_slope,
    recirculation_check,
    transient_fit,
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


# ── ASCEND 단계: 바닥에서 이륙해 호버링한 뒤 주행한다 ──────────────────

def _floor_limits() -> Limits:
    """바닥을 원점으로 잡은 odom-상대 한계 (drag_test_odom.yaml과 같은 규약)."""
    return Limits(
        run_x_min=-0.15, run_x_max=2.45,
        lane_y=0.0, max_cross_track_m=0.35,
        z_min=-0.35, z_max=0.35,
        target_z=0.0, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )


def test_ascend_phase_exists_and_is_ordered_before_approach():
    assert Phase.ASCEND.value == "ASCEND"
    assert list(Phase)[0] is Phase.ASCEND


def test_depth_check_blocks_takeoff_from_the_floor():
    """바닥(z=0)에서 목표가 +0.5면 깊이 두 항 모두 위반한다 — 그래서 끈다."""
    limits = _floor_limits()
    # 상승 목표에 해당하는 z를 그대로 넣으면 z_max와 목표오차 둘 다 걸린다.
    assert limits.violation(0.0, 0.0, 0.50, 0.0, 0.0) is not None
    assert limits.violation(0.0, 0.0, 0.50, 0.0, 0.0,
                            check_depth=False) is None


def test_check_depth_false_still_enforces_lane_and_attitude():
    """깊이만 끄는 것이지 차선·자세·주행축까지 푸는 게 아니다."""
    limits = _floor_limits()
    assert "차선 이탈" in limits.violation(
        0.0, 0.90, 0.50, 0.0, 0.0, check_depth=False)
    assert "자세 이탈" in limits.violation(
        0.0, 0.0, 0.50, math.radians(45.0), 0.0, check_depth=False)
    assert "주행축 한계" in limits.violation(
        3.00, 0.0, 0.50, 0.0, 0.0, check_depth=False)


def test_depth_check_is_enforced_once_reanchored():
    """상승이 끝나고 기준을 다시 잡으면 호버 깊이가 z=0이고 한계가 되살아난다."""
    limits = _floor_limits()
    assert limits.violation(0.0, 0.0, 0.0, 0.0, 0.0) is None
    assert "깊이 이탈" in limits.violation(0.0, 0.0, 0.32, 0.0, 0.0)
    assert "깊이 한계" in limits.violation(0.0, 0.0, 0.40, 0.0, 0.0)


def test_ascent_corridor_brackets_the_target():
    """노드의 상승 회랑은 [-max_z_error, 목표+max_z_error]다."""
    limits = _floor_limits()
    target, band = 0.5, limits.max_z_error_m
    lo, hi = -band, target + band
    assert lo <= 0.0 <= hi          # 바닥 출발점
    assert lo <= target <= hi       # 목표 호버 깊이
    assert not (lo <= -0.40 <= hi)  # 바닥 아래로 가라앉음
    assert not (lo <= 0.90 <= hi)   # 목표를 넘어 수면으로 솟음


# ── 왕복 기하: 출발점과 여유가 한계 안에 들어가는가 ────────────────────

def test_odom_relative_geometry_leaves_room_both_ways():
    """drag_test_odom.yaml의 기하로 전/후진 전 구간이 한계 안에 있어야 한다.

    run_x_min == -run_start_margin_m 으로 두면 전진 출발점이 정확히 0(=prepare
    자리)이 되고, margin 이 그대로 뒤쪽 여유가 된다. 0.15로 잡았을 때
    start_hold 30초 동안 밀려 하한을 밟은 적이 있어 여유를 명시적으로 검사한다.
    """
    run_x_min, run_x_max, margin = -0.35, 2.60, 0.35
    run_m, coast_m = 1.80, 0.62
    limits = Limits(
        run_x_min=run_x_min, run_x_max=run_x_max,
        lane_y=0.0, max_cross_track_m=0.35,
        z_min=-0.35, z_max=0.35, target_z=0.0, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )
    plans = build_level_plans([0.10, 0.20], limits, 0.0, margin)

    forward = plans[0]
    assert forward.forward is True
    assert forward.start_x == pytest.approx(0.0)   # prepare 자리에서 출발
    assert limits.violation(
        forward.start_x + run_m + coast_m, 0.0, 0.0, 0.0, 0.0) is None

    reverse = plans[1]
    assert reverse.forward is False
    assert limits.violation(
        reverse.start_x - run_m - coast_m, 0.0, 0.0, 0.0, 0.0) is None


def test_start_hold_needs_backward_room_behind_the_start_point():
    """출발점과 하한 사이 여유가 start_hold 중 밀림을 흡수해야 한다."""
    margin = 0.35
    limits = Limits(
        run_x_min=-margin, run_x_max=2.60,
        lane_y=0.0, max_cross_track_m=0.35,
        z_min=-0.35, z_max=0.35, target_z=0.0, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )
    start_x = build_level_plans([0.10], limits, 0.0, margin)[0].start_x
    # 예전 0.15 여유는 26초 만에 소진됐다. 0.35는 같은 밀림률에서 60초를 버틴다.
    drift_rate_mps = 0.15 / 26.0
    assert limits.violation(start_x - drift_rate_mps * 55.0,
                            0.0, 0.0, 0.0, 0.0) is None
    assert limits.violation(start_x - margin - 0.01,
                            0.0, 0.0, 0.0, 0.0) is not None


# ── 단발 운용: 수준마다 운용자가 자리를 잡고 prepare→start 한다 ────────

def test_single_shot_plans_do_not_alternate_direction():
    """왕복을 끄면 모든 수준이 같은 방향·같은 출발점을 쓴다.

    매 회차 출발점이 '방금 prepare한 자리'가 되므로 여유 공간이 회차마다
    같아진다. 왕복 계획에서는 출발점이 run_x_min 쪽과 run_x_max 쪽을
    번갈아 쓰기 때문에 그렇지 않다.
    """
    margin = 0.35
    limits = Limits(
        run_x_min=-margin, run_x_max=2.20,
        lane_y=0.0, max_cross_track_m=0.35,
        z_min=-0.35, z_max=0.35, target_z=0.0, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )
    levels = [0.10, 0.20, 0.32]

    single = build_level_plans(levels, limits, 0.0, margin,
                               alternate_direction=False)
    assert [p.forward for p in single] == [True, True, True]
    assert all(p.start_x == pytest.approx(0.0) for p in single)
    assert all(p.heading == pytest.approx(0.0) for p in single)

    sweep = build_level_plans(levels, limits, 0.0, margin)
    assert [p.forward for p in sweep] == [True, False, True]
    assert len({round(p.start_x, 6) for p in sweep}) == 2


def test_single_shot_preserves_level_order():
    """회차마다 다음 수준으로 넘어가되 순서와 개수는 그대로여야 한다."""
    margin = 0.35
    limits = Limits(
        run_x_min=-margin, run_x_max=2.20,
        lane_y=0.0, max_cross_track_m=0.35,
        z_min=-0.35, z_max=0.35, target_z=0.0, max_z_error_m=0.30,
        max_tilt_rad=math.radians(30.0),
    )
    levels = [0.10, 0.20, 0.32, 0.10]     # repeat_first_level 포함 형태
    plans = build_level_plans(levels, limits, 0.0, margin,
                              alternate_direction=False)
    assert [p.level for p in plans] == levels
    assert len(plans) == len(levels)


# ── 정상상태 조기 종료: 언제 끊고, 언제 끊지 말아야 하는가 ─────────────

def _first_steady_time(series, *, window_s=1.2, max_slope=0.03, max_sd=0.05,
                       min_run_s=2.0):
    """노드의 조기 종료 조건을 그대로 재현해 처음 걸리는 시각을 돌려준다."""
    det = SteadyDetector(window_s=window_s, max_slope=max_slope, max_sd=max_sd)
    for t, u in series:
        det.add(t, u, 0.0)
        if t < min_run_s:
            continue
        out = det.evaluate()
        if out.get("steady") and out.get("window_s", 0.0) >= window_s * 0.9:
            return t, out
    return None, None


def test_early_exit_does_not_latch_on_the_vehicle_at_rest():
    """정지 상태는 기울기도 편차도 0이라 정상상태를 통과한다 — min_run_s가 막는다."""
    at_rest = [(i * 0.04, 0.0) for i in range(50)]      # 0~2.0s, u=0
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    for t, u in at_rest:
        det.add(t, u, 0.0)
    assert det.evaluate()["steady"] is True             # 가드가 없으면 통과한다
    t_stop, _ = _first_steady_time(at_rest, min_run_s=2.0)
    assert t_stop is None                                # 가드가 있으면 안 걸린다


def test_early_exit_does_not_trigger_during_acceleration():
    """가속 중에는 기울기가 살아 있어 걸리지 않는다."""
    ramp = [(i * 0.04, 0.60 * (1.0 - math.exp(-i * 0.04 / 2.5)))
            for i in range(150)]                         # 0~6s, 시정수 2.5s
    t_stop, _ = _first_steady_time(ramp, min_run_s=2.0)
    assert t_stop is None or t_stop > 4.0


def test_early_exit_stops_before_a_late_disturbance_contaminates_the_tail():
    """실측 level 0.32 형태 — 0.46까지 갔다가 뒤에 꺾인다.

    조기 종료가 없으면 꼬리 창이 감속 구간에 걸려 정상상태 속도를 크게
    낮춰 잡는다. 실제로 그렇게 u=0.253/-0.092 같은 값이 나왔다.
    """
    plateau = [(t, u) for t, u in (
        (0.5, 0.29), (1.0, 0.36), (1.5, 0.41), (2.0, 0.44), (2.4, 0.44),
        (2.9, 0.43), (3.4, 0.43), (3.9, 0.44), (4.4, 0.44), (4.8, 0.44),
    )]
    # 0.04s 간격으로 채워 넣는다 (창 하나에 표본이 충분해야 한다).
    dense = []
    for i in range(len(plateau) - 1):
        (t0, u0), (t1, u1) = plateau[i], plateau[i + 1]
        n = max(1, int((t1 - t0) / 0.04))
        for k in range(n):
            f = k / n
            dense.append((t0 + k * 0.04, u0 + f * (u1 - u0)))
    disturbed = dense + [(4.8 + i * 0.04, 0.44 - 0.20 * i * 0.04)
                         for i in range(1, 75)]          # 뒤늦은 감속

    t_stop, out = _first_steady_time(disturbed, min_run_s=2.0)
    assert t_stop is not None and t_stop < 4.8           # 교란 전에 끊는다
    assert out["u_mps"] == pytest.approx(0.44, abs=0.03)

    # 끝까지 달리면 꼬리가 감속 구간이라 값이 무너진다.
    det = SteadyDetector(window_s=1.2, max_slope=0.03, max_sd=0.05)
    for t, u in disturbed:
        det.add(t, u, 0.0)
    assert det.evaluate()["u_mps"] < 0.35


# ── 과도구간 적합: 정상상태 없이 가속 곡선만으로 계수를 되찾는다 ────────

def _synth_transient(noise_sd=0.004, *, m_eff=21.0, xu=13.7, xuu=141.0,
                     coast=True, seed=0):
    """알려진 계수로 궤적을 만든다. -1s 전기록과 타행까지 포함한다."""
    rng = np.random.default_rng(seed)
    rows = []
    for level, tau in ((0.10, 12.4), (0.20, 24.8), (0.32, 39.6),
                       (0.45, 55.8), (0.60, 74.3)):
        u, dt = 0.0, 0.04
        for i in range(-25, 150):                    # -1.0s ~ +6.0s
            tau_now = 0.0 if i < 0 else tau
            rows.append({"level": level, "t": i * dt,
                         "u": u + rng.normal(0.0, noise_sd),
                         "tau_x_delivered": tau_now})
            u += dt * (tau_now - xu * u - xuu * u * abs(u)) / m_eff
        if coast:
            for i in range(75):                      # 타행 3s
                rows.append({"level": level, "t": (150 + i) * dt,
                             "u": u + rng.normal(0.0, noise_sd),
                             "tau_x_delivered": 0.0})
                u += dt * (0.0 - xu * u - xuu * u * abs(u)) / m_eff
    return rows


def test_transient_fit_recovers_xuu_without_any_steady_state():
    """정상상태 창을 전혀 쓰지 않고도 Xuu를 1% 안으로 되찾는다."""
    for noise in (0.0, 0.004, 0.010):
        out = transient_fit(_synth_transient(noise))
        assert out["ok"] is True
        assert out["Xuu"] == pytest.approx(141.0, rel=0.02), noise
        assert out["Xu"] == pytest.approx(13.7, rel=0.05), noise
        assert out["rms_relative"] < 0.05, noise


def test_transient_fit_m_eff_is_attenuated_by_velocity_noise():
    """m_eff는 du/dt의 잡음 때문에 낮게 치우친다 — 보고에 쓰지 말라는 근거."""
    clean = transient_fit(_synth_transient(0.0))["m_eff_kg"]
    noisy = transient_fit(_synth_transient(0.010))["m_eff_kg"]
    assert noisy < clean < 21.0            # 둘 다 참값보다 낮고, 잡음이 더 낮춘다
    assert clean == pytest.approx(21.0, rel=0.15)


def test_transient_fit_does_not_differentiate_across_level_boundaries():
    """수준마다 t가 되감기므로 합쳐서 미분하면 발산한다 — 나눠 미분해야 한다."""
    rows = _synth_transient(0.004)
    assert min(r["t"] for r in rows) < 0.0          # 전기록이 음수 t로 들어있다
    assert len({r["level"] for r in rows}) == 5
    out = transient_fit(rows)
    assert out["n_levels"] == 5
    assert math.isfinite(out["Xuu"]) and out["Xuu"] > 0.0


def test_transient_fit_rejects_too_few_samples():
    assert transient_fit([])["ok"] is False
    assert transient_fit([{"level": 0.1, "t": i * 0.04, "u": 0.1,
                           "tau_x_delivered": 12.0}
                          for i in range(10)])["ok"] is False


# ── DVL 직결 파서: 논문이 쓰는 body 속도 경로 ──────────────────────────

_DVL_VELOCITY = json.dumps({
    "time": 111.4, "vx": 0.231, "vy": -0.004, "vz": 0.002,
    "fom": 0.00015, "altitude": 0.743,
    "transducers": [{"id": i, "beam_valid": i != 3} for i in range(4)],
    "velocity_valid": True, "status": 0,
    "format": "json_v3", "type": "velocity",
})


def test_dvl_parser_extracts_body_velocity_and_beam_count():
    out = DvlReader.parse_line(_DVL_VELOCITY)
    assert out is not None
    assert (out["vx"], out["vy"], out["vz"]) == (0.231, -0.004, 0.002)
    assert out["velocity_valid"] is True
    assert out["beams_valid"] == 3            # 4개 중 1개 무효
    assert out["altitude"] == 0.743


def test_dvl_parser_ignores_other_report_types_and_garbage():
    """A50은 position_local/dead_reckoning도 같은 소켓으로 보낸다."""
    assert DvlReader.parse_line(json.dumps(
        {"type": "position_local", "x": 1.0, "y": 2.0})) is None
    assert DvlReader.parse_line(b"not json") is None
    assert DvlReader.parse_line("") is None
    assert DvlReader.parse_line(json.dumps({"type": "velocity"})) is None


def test_dvl_status_nonzero_marks_velocity_invalid():
    """velocity_valid 만 보면 안 된다 — status != 0 이면 못 쓴다."""
    bad = json.loads(_DVL_VELOCITY)
    bad["status"] = 1
    assert DvlReader.parse_line(json.dumps(bad))["velocity_valid"] is False


def test_dvl_sample_is_all_none_when_no_data_has_arrived():
    """DVL이 없거나 끊겨도 측정을 막지 않는다 — 값만 비운다."""
    sample = DvlReader("203.0.113.1").sample()
    assert sample["dvl_vx"] is None
    assert sample["dvl_valid"] is False
    assert sample["dvl_connected"] is False


# ── 조기 종료 확인 창: 짧은 창의 오인을 긴 창이 걸러낸다 ────────────────

def test_confirmation_window_rejects_a_plateau_that_is_still_rising():
    """가속 뒤에 짧은 평탄부가 붙은 궤적 — 짧은 창은 속고 긴 창은 안 속는다.

    실측 level 0.20이 이 모양이었다. t=2.09에 du/dt 0.0142로 문턱 0.015를
    통과했지만 그 뒤로도 계속 올랐고, 0.243 m/s라는 이른 값이 채택됐다.
    """
    det = SteadyDetector(window_s=1.2, max_slope=0.015, max_sd=0.05)
    dt = 0.04
    t = 0.0
    for i in range(60):                     # 2.4s 동안 0 → 0.30 으로 상승
        det.add(t, 0.30 * (i / 60.0), 22.5)
        t += dt
    for _ in range(30):                     # 이어서 1.2s 평탄
        det.add(t, 0.300, 22.5)
        t += dt

    short = det.evaluate()                  # 마지막 1.2s = 평탄부만
    assert short["steady"] is True

    long = det.evaluate(2.4)                # 상승 절반이 들어온다
    assert long["steady"] is False
    assert "du/dt" in long["reason"]


def test_confirmation_window_still_accepts_a_real_plateau():
    """진짜 평탄부는 긴 창에서도 통과해야 한다 — 과하게 막으면 안 된다."""
    det = SteadyDetector(window_s=1.2, max_slope=0.015, max_sd=0.05)
    for i in range(200):                            # 8초 내내 0.40 근방
        t = i * 0.04
        det.add(t, 0.400 + 0.002 * math.sin(t * 3.0), 40.0)
    assert det.evaluate()["steady"] is True
    assert det.evaluate(2.4)["steady"] is True


def test_evaluate_defaults_to_the_configured_window():
    det = SteadyDetector(window_s=1.2, max_slope=0.015, max_sd=0.05)
    for i in range(200):
        det.add(i * 0.04, 0.3, 30.0)
    assert det.evaluate()["window_s"] == pytest.approx(
        det.evaluate(1.2)["window_s"])
    assert det.evaluate(2.4)["window_s"] > det.evaluate()["window_s"]
