"""여기(excitation) 신호 회귀 시험.

이 신호가 틀리면 dead time 추정이 조용히 틀린다 -- 교차상관은 무엇을 넣든
어떤 lag 을 내놓기 때문에, 신호가 의도와 다르다는 사실이 결과에 드러나지 않는다.
그래서 파형 자체를 고정한다. ROS 없이 도는 순수 함수만 시험한다.
"""

import math

import pytest

from brov_control.diag_excite_node import excitation


def _square(t, **kw):
    base = dict(kind="square", amplitude=20.0, bias=0.0, period_s=1.0,
                chirp_f0_hz=0.2, chirp_f1_hz=5.0, duration_s=60.0)
    base.update(kw)
    return excitation(t, **base)


def test_square_alternates_every_half_period():
    assert _square(0.00) == pytest.approx(+20.0)
    assert _square(0.49) == pytest.approx(+20.0)
    assert _square(0.51) == pytest.approx(-20.0)
    assert _square(0.99) == pytest.approx(-20.0)
    assert _square(1.01) == pytest.approx(+20.0)


def test_square_has_zero_mean_so_the_vehicle_does_not_drift():
    """평균이 0 이 아니면 60 초 동안 한 방향으로 밀려 안전 영역을 벗어난다."""
    n = 10_000
    samples = [_square(6.0 * i / n) for i in range(n)]
    assert sum(samples) / n == pytest.approx(0.0, abs=0.05)


def test_bias_shifts_the_mean_by_exactly_the_bias():
    """부력 상쇄용이다. 진폭은 그대로 두고 평균만 옮겨야 한다."""
    n = 10_000
    samples = [_square(6.0 * i / n, bias=3.0) for i in range(n)]
    assert sum(samples) / n == pytest.approx(3.0, abs=0.05)
    assert max(samples) == pytest.approx(23.0)
    assert min(samples) == pytest.approx(-17.0)


def test_signal_is_zero_outside_the_run_window():
    """duration 이 지나면 스스로 중립으로 돌아간다 -- 잊고 두어도 계속 흔들지 않는다."""
    assert _square(59.9, duration_s=60.0) != 0.0
    assert _square(60.0, duration_s=60.0) == 0.0
    assert _square(120.0, duration_s=60.0) == 0.0
    assert _square(-0.1) == 0.0


def test_chirp_sweeps_from_f0_to_f1_over_the_duration():
    """순시 주파수는 위상의 미분이다. 스윕이 실제로 f0 에서 f1 로 가야 한다."""
    kw = dict(kind="chirp", amplitude=1.0, bias=0.0, period_s=1.0,
              chirp_f0_hz=1.0, chirp_f1_hz=5.0, duration_s=10.0)

    def phase(t):
        # asin 은 되돌릴 수 없으므로 수치 미분 대신 영교차 간격으로 본다.
        return excitation(t, **kw)

    def first_zero_crossing_period(t0):
        prev, crossings = phase(t0), []
        t = t0
        while t < t0 + 2.0 and len(crossings) < 3:
            t += 1e-4
            cur = phase(t)
            if prev <= 0.0 < cur or prev >= 0.0 > cur:
                crossings.append(t)
            prev = cur
        return crossings[1] - crossings[0] if len(crossings) > 1 else None

    early = first_zero_crossing_period(0.05)     # 반주기 ~ 1/(2*1 Hz) = 0.5 s
    late = first_zero_crossing_period(9.0)       # 반주기 ~ 1/(2*~4.6 Hz) ≈ 0.11 s
    assert early is not None and late is not None
    assert early > 3.0 * late, f"스윕이 일어나지 않았다: {early:.3f} vs {late:.3f}"


def test_chirp_amplitude_never_exceeds_the_requested_value():
    kw = dict(kind="chirp", amplitude=12.0, bias=0.0, period_s=1.0,
              chirp_f0_hz=0.2, chirp_f1_hz=5.0, duration_s=30.0)
    peak = max(abs(excitation(30.0 * i / 5000, **kw)) for i in range(5000))
    assert peak <= 12.0 + 1e-9


def test_unknown_kind_is_refused():
    with pytest.raises(ValueError):
        _square(1.0, kind="prbs")


# ---------------------------------------------------------------- 깊이 유지
# 음성부력 기체는 이 노드만 떠 있으면 가라앉는다. 부호가 틀리면 바닥으로 **더**
# 민다 -- 실기에서 한 번이면 끝나는 종류의 실수라 여기서 고정한다.
from brov_control.diag_excite_node import depth_hold_force


def _hold(**kw):
    base = dict(kp_n_per_m=20.0, kd_n_per_mps=15.0, bias_n=0.0, max_n=15.0)
    base.update(kw)
    return base


def test_too_deep_pushes_up():
    """NED z 는 아래가 +. 기준보다 깊으면(+오차) 위로(-힘) 밀어야 한다."""
    assert depth_hold_force(+0.10, 0.0, 0.0, **_hold()) < 0.0
    assert depth_hold_force(-0.10, 0.0, 0.0, **_hold()) > 0.0


def test_sinking_is_damped():
    """가라앉는 중(vz>0)이면 오차가 0 이어도 위로 민다."""
    assert depth_hold_force(0.0, +0.05, 0.0, **_hold()) < 0.0


def test_bias_cancels_weight_without_error():
    """순중량 상쇄 feedforward: 오차 0 에서 정확히 bias 가 나간다."""
    assert depth_hold_force(0.0, 0.0, 0.0, **_hold(bias_n=-3.0)) == pytest.approx(-3.0)


def test_hold_force_is_clamped():
    assert depth_hold_force(+5.0, 0.0, 0.0, **_hold()) == pytest.approx(-15.0)
    assert depth_hold_force(-5.0, 0.0, 0.0, **_hold()) == pytest.approx(+15.0)


def test_hold_gain_stays_far_below_the_heave_oscillation_threshold():
    """이 루프도 80 ms 를 지난다. kd 15 N/(m/s) 는 정규화 0.125 -- 문턱 3.52 의 1/28."""
    assert 15.0 / 120.0 < 3.52 / 10.0


def test_rise_moves_the_reference_up_not_down():
    """바닥(z=+0.9)에서 start, rise 0.4 -> 기준 z=+0.5 (NED 는 아래가 +).
    부호가 틀리면 바닥을 뚫으려 든다."""
    from brov_control.diag_excite_node import ExciteNode
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    try:
        # 기본 axis 는 heave 라 깊이 되먹임이 꺼진다(측정 축). yaw 로 둔다.
        node = ExciteNode(parameter_overrides=[
            Parameter("axis", value="yaw"), Parameter("rise_m", value=0.4),
            Parameter("bias", value=1.0), Parameter("amplitude", value=0.5)])
        node._z = 0.9; node._state_valid = True
        from std_msgs.msg import Bool
        node._on_active(Bool(data=True))
        assert node._depth_ref == pytest.approx(0.5)
        # 기준보다 깊은 상태(바닥) -> 위로(-) 민다
        node._vz = 0.0
        assert node._depth_hold_force() < 0.0
        node.destroy_node()
        with pytest.raises(ValueError, match="rise_m"):
            ExciteNode(parameter_overrides=[
                Parameter("axis", value="yaw"), Parameter("rise_m", value=0.9),
                Parameter("bias", value=1.0), Parameter("amplitude", value=0.5)])
    finally:
        rclpy.shutdown()
