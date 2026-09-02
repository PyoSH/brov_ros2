"""깊이 게이트 판정 로직 회귀 시험.

이 판정이 틀리면 **틀린 센서로 깊이를 만들거나**, 얼어붙은 EKF 수직 위치를
정상이라고 부르게 된다. 후자는 2026-08-29 SITL 에서 기체를 1.77 m 띄웠고,
수조 깊이 여유는 0.7 m 뿐이다. ROS 없이 도는 순수 함수만 시험한다.
"""

import pytest

from brov_base.diag_depth_gate import (
    analyse_instance,
    analyse_sweep,
    identify_responding_instances,
    linear_fit,
    pressure_to_depth_m,
    resample,
)


def _samples(value: float, n: int = 60, noise: float = 0.0) -> list[float]:
    return [value + (noise if i % 2 else -noise) for i in range(n)]


def test_water_sensor_is_identified_by_the_expected_pressure_slope():
    """담수 1 m = 9800 Pa. ArduSub 자신의 변환식 상수와 같은 값이어야 한다."""
    result = analyse_instance(
        _samples(101_325.0), _samples(101_325.0 + 9800.0), 1.0, 1.0
    )
    assert result["verdict"] == "WATER (깊이센서)"
    assert result["slope_pa_per_m"] == pytest.approx(9800.0)


def test_internal_baro_barely_moves():
    result = analyse_instance(
        _samples(99_800.0), _samples(99_812.0), 1.0, 1.0
    )
    assert result["verdict"] == "dry/internal"


def test_ambiguous_response_is_not_called_water():
    """애매한 것을 물속이라고 부르면 그 오차가 깊이 전체에 실린다."""
    result = analyse_instance(
        _samples(101_325.0), _samples(101_325.0 + 2_500.0), 1.0, 1.0
    )
    assert result["verdict"] == "판정 불가"


def test_seawater_specific_gravity_changes_the_expectation():
    """SPEC_GRAV 가 틀리면 깊이에 2.4% 스케일 오차가 난다."""
    delta = 9800.0 * 1.024
    assert analyse_instance(
        _samples(101_325.0), _samples(101_325.0 + delta), 1.0, 1.024
    )["verdict"] == "WATER (깊이센서)"


def test_partial_drop_is_scaled_by_the_actual_distance():
    """0.5 m 만 내렸으면 4900 Pa 가 정상이다 -- 거리로 나눠야 판정이 선다."""
    result = analyse_instance(
        _samples(101_325.0), _samples(101_325.0 + 4900.0), 0.5, 1.0
    )
    assert result["verdict"] == "WATER (깊이센서)"
    assert result["slope_pa_per_m"] == pytest.approx(9800.0)


def test_missing_instance_reports_no_reception():
    result = analyse_instance([], [], 1.0, 1.0)
    assert result["verdict"] == "수신 없음"
    assert result["delta_pa"] is None


# --------------------------------------------------------------- sweep 방식
# 수조의 z 안전 영역이 0.20~0.90 m 다. **1 m 를 내릴 수가 없고** 0.5 m 를 손으로
# 정확히 재는 것도 현실적이지 않다. 그래서 거리를 모르는 채로, 압력을 기준자로
# 삼아 EKF 를 회귀한다. 두 점으로는 "얼어붙음 / 배율 오류 / 부호 반전" 이 서로
# 구분되지 않는다 -- 셋 다 다른 고장이다.
def _sweep(amplitude_m: float, n: int = 200) -> list[float]:
    import math as _m
    return [amplitude_m * _m.sin(2 * _m.pi * 2.0 * i / n) for i in range(n)]


def test_water_instance_is_the_one_that_moves():
    """물속과 내부 baro 는 폭이 자릿수로 갈린다. 그 분리만 쓴다."""
    traces = {
        0: [99_800.0 + 0.4 * (i % 3) for i in range(200)],      # 내부
        1: [101_325.0 + 9800.0 * d for d in _sweep(0.30)],      # 물속 ±30 cm
        2: [],                                                   # 미수신
    }
    found = identify_responding_instances(traces)
    assert found["responding"] == [1]
    assert found["received"] == [0, 1]


def test_no_vertical_motion_is_not_mistaken_for_a_dry_sensor():
    """움직이지 않았으면 물속 센서도 조용하다. 아무것도 반응 안 하면 그렇게 말한다."""
    traces = {0: [99_800.0] * 100, 1: [101_325.0] * 100, 2: []}
    assert identify_responding_instances(traces)["responding"] == []


def test_pressure_converts_to_depth_with_ardusub_constant():
    """9800 Pa/m 은 ArduSub 자신의 변환식 상수다. 다르면 깊이가 통째로 어긋난다."""
    depth = pressure_to_depth_m([101_325.0, 101_325.0 + 9800.0], 1.0)
    assert depth[1] - depth[0] == pytest.approx(1.0)


def test_seawater_specific_gravity_scales_the_ruler():
    depth = pressure_to_depth_m([101_325.0, 101_325.0 + 9800.0 * 1.024], 1.024)
    assert depth[1] - depth[0] == pytest.approx(1.0)


def test_tracking_ekf_passes_the_regression():
    baro = _sweep(0.30)
    ekf = [1.002 * d + 0.4 for d in baro]      # 오프셋은 무관하다 -- 상대 깊이다
    result = analyse_sweep(baro, ekf)
    assert result["verdict"].startswith("PASS")
    assert result["slope"] == pytest.approx(1.002, abs=1e-3)


def test_frozen_ekf_is_distinguished_from_a_scale_error():
    """SITL 증상: 실제로 움직이는데 EKF 값이 얼어붙는다."""
    baro = _sweep(0.30)
    frozen = analyse_sweep(baro, [0.0 for _ in baro])
    assert frozen["verdict"].startswith("FAIL")
    assert "얼어붙음" in frozen["verdict"]

    scaled = analyse_sweep(baro, [0.55 * d for d in baro])
    assert scaled["verdict"].startswith("FAIL")
    assert "배율" in scaled["verdict"]


def test_sign_inversion_is_named_as_such():
    """2026-08-28 수조에서 EKF heave 와 DVL vz 의 부호가 반대였다."""
    baro = _sweep(0.30)
    result = analyse_sweep(baro, [-1.0 * d for d in baro])
    assert result["verdict"].startswith("FAIL")
    assert "부호" in result["verdict"]


def test_noisy_uncorrelated_ekf_fails_even_with_slope_near_one():
    """기울기만 보면 통과처럼 보이는 경우가 있다. R^2 이 그것을 잡는다."""
    import random
    rng = random.Random(11)
    baro = _sweep(0.30)
    ekf = [d + rng.uniform(-0.5, 0.5) for d in baro]
    result = analyse_sweep(baro, ekf)
    assert result["verdict"].startswith("FAIL")
    assert "상관" in result["verdict"]


def test_insufficient_vertical_travel_refuses_to_judge():
    """2 cm 움직여 놓고 통과시키면 게이트가 아무것도 막지 않는다."""
    baro = _sweep(0.01)
    result = analyse_sweep(baro, [1.0 * d for d in baro])
    assert result["verdict"].startswith("판정 불가")
    assert "수직 이동" in result["verdict"]


def test_resample_aligns_two_topics_before_regression():
    """압력과 depth_ekf 는 주기가 다르다. 짝을 안 지으면 위상차가 기울기에 섞인다."""
    source_t = [0.0, 1.0, 2.0]
    source_v = [0.0, 10.0, 20.0]
    assert resample(source_t, source_v, [0.5, 1.5]) == pytest.approx([5.0, 15.0])


def test_linear_fit_reports_none_when_the_input_cannot_support_a_fit():
    assert linear_fit([1.0, 1.0, 1.0], [1.0, 2.0, 3.0])["slope"] is None
    assert linear_fit([1.0], [1.0])["slope"] is None
