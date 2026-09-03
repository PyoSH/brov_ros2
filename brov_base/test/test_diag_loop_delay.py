"""diag_loop_delay 의 교차상관 수학이 실제로 지연을 되찾는지 확인한다.

bag 읽기는 rosbag2 의존이라 여기서 시험하지 않는다. 시험하는 것은 **알려진
지연을 넣은 합성 신호에서 그 값을 되찾는가** 다 -- 그게 이 도구의 전부이고,
틀리면 실기 측정값이 통째로 틀린다.
"""
import numpy as np
import pytest

from brov_base.diag_loop_delay import analyse

CONTROL_DT = 0.04
FS = 200.0                      # 합성 신호 표본율


def _synth(tau_s, duration=40.0, freq=2.0, noise=0.0, seed=0):
    """명령 정현파와, 그것을 tau 만큼 늦춰 적분한 속도를 만든다.

    가속도가 명령에 비례하고(a = F/m) 속도가 그 적분이라는 실제 관계를 따른다.
    도구는 명령과 **가속도** 사이의 지연을 재므로 tau 를 되찾아야 한다.
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration, 1.0 / FS)
    f = 100.0 * np.sin(2 * np.pi * freq * t)
    lag = int(round(tau_s * FS))
    a = np.concatenate([np.zeros(lag), f[: len(f) - lag]]) / 28.1
    if noise:
        a = a + rng.normal(0.0, noise, size=a.shape)
    v = np.cumsum(a) / FS
    wr_f = np.zeros((len(t), 3)); wr_f[:, 2] = f
    st_v = np.zeros((len(t), 3)); st_v[:, 2] = v
    act_t = np.array([t[0]])
    act_v = np.array([True])
    return t, wr_f, t, st_v, act_t, act_v


@pytest.mark.parametrize("tau", [0.02, 0.04, 0.06, 0.08, 0.12])
def test_recovers_known_delay(tau, capsys):
    analyse(*_synth(tau), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=1.0)
    out = capsys.readouterr().out
    line = next(l for l in out.splitlines() if "피크 lag" in l)
    got_ms = float(line.split("=")[1].split("ms")[0])
    # 격자 해상도가 control_dt/2 = 20 ms 이므로 그 이내면 맞다.
    assert abs(got_ms - tau * 1000) <= 20.0, f"{line} (기대 {tau*1000:.0f} ms)"


def test_jitter_lowers_correlation_for_broadband_input():
    """지연이 매번 흔들리면 r 이 떨어진다 -- 단, 입력이 광대역일 때만.

    정현파 하나에 표본별 무작위 지연을 걸면 평균이 다시 같은 주파수의 정현파가
    되어(진폭만 줄고 위상만 밀림) 상관이 높게 남는다. 물리적으로 맞는 거동이다.
    광대역 입력에서는 주파수 성분마다 다르게 흐트러져 상관이 실제로 떨어진다.
    실제 명령도 정현파가 아니라 광대역이므로 이쪽이 맞는 시험이다.

    r 은 눈금이 매겨진 jitter 척도가 아니라 **지표**다. 그래서 절대 문턱을
    단언하지 않고 깨끗한 고정지연 대비 얼마나 낮아지는지만 본다.
    2026-08-31 SITL: mavproxy 경유 r=0.487, 직결 r=0.934.
    """
    import contextlib
    import io as _io

    def run(jitter):
        rng = np.random.default_rng(2)
        t = np.arange(0.0, 60.0, 1.0 / FS)
        # 광대역 명령: 백색잡음을 저역통과해 만든다
        raw = rng.normal(0.0, 1.0, size=len(t))
        k = np.ones(int(FS / 20)) / (FS / 20)
        f = 100.0 * np.convolve(raw, k, mode="same")
        base = int(round(0.06 * FS))
        a = np.zeros_like(f)
        for n in range(len(f)):
            lag = base + (int(rng.integers(-base, base + 1)) if jitter else 0)
            if 0 <= n - lag < len(f):
                a[n] = f[n - lag] / 28.1
        v = np.cumsum(a) / FS
        wr_f = np.zeros((len(t), 3)); wr_f[:, 2] = f
        st_v = np.zeros((len(t), 3)); st_v[:, 2] = v
        buf = _io.StringIO()
        with contextlib.redirect_stdout(buf):
            analyse(t, wr_f, t, st_v, np.array([t[0]]), np.array([True]),
                    axis="heave", m_eff=28.1, control_dt=CONTROL_DT, skip_s=1.0)
        line = next(l for l in buf.getvalue().splitlines() if "피크 lag" in l)
        # "피크 lag = X ms,  r = +0.9  (부그리드 보정 ...)" -- r 만 떼어낸다.
        return abs(float(line.split("r =")[1].split("(")[0].strip()))

    r_clean = run(jitter=False)
    r_jitter = run(jitter=True)
    assert r_clean > 0.5, f"고정 지연인데 r 이 낮다: {r_clean:.3f}"
    assert r_jitter < r_clean - 0.1, (
        f"jitter 를 넣었는데 r 이 거의 안 떨어졌다: {r_clean:.3f} -> {r_jitter:.3f}")


def test_predicted_crossover_matches_the_sitl_measurement(capsys):
    """SITL 실측과 예측식이 맞는지 -- tau=60ms 에서 약 3 Hz 가 나와야 한다.

    2026-08-31 Gazebo SITL 직결 측정: 지연 60 ms, 실측 지배 주파수 3.00 Hz.
    """
    analyse(*_synth(0.06, freq=3.0), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=1.0)
    out = capsys.readouterr().out
    line = next(l for l in out.splitlines() if "교차 예측 주파수" in l)
    f_pred = float(line.split("=")[1].split("Hz")[0])
    assert 2.5 <= f_pred <= 3.5, line


def test_seconds_truncates_the_analysis_window(capsys):
    """여기가 끝난 뒤의 명령 0 꼬리를 잘라낸다.

    deadtime_test 는 duration_s 뒤 스스로 중립으로 돌아가지만 control_active 는
    계속 true 다. 2026-09-02 실기에서 30 s 여기 + 25.7 s 꼬리가 한 창에 들어갔다.
    """
    analyse(*_synth(0.08, duration=60.0), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=5.0, seconds=25.0)
    out = capsys.readouterr().out
    line = next(l for l in out.splitlines() if l.startswith("분석 구간"))
    assert line.startswith("분석 구간 25.0s")
    assert "이후 25s 만 사용" in line


def test_seconds_still_refuses_a_window_that_is_too_short():
    with pytest.raises(SystemExit):
        analyse(*_synth(0.08, duration=60.0), axis="heave", m_eff=28.1,
                control_dt=CONTROL_DT, skip_s=5.0, seconds=2.0)


def test_open_loop_drops_the_closed_loop_frequency_comparison(capsys):
    """개루프 여기 주행에서 지배 주파수는 **넣은 신호**다. 폐루프 비교를 그대로
    인용하면 없는 '불일치' 를 보고한다 -- 2026-09-02 첫 분석이 그랬다."""
    analyse(*_synth(0.08, freq=1.0), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=1.0, open_loop=True)
    out = capsys.readouterr().out
    assert "불일치" not in out
    assert "개루프" in out
    # 1 Hz 협대역이면 봉우리 반폭 ~250 ms 라는 분해능 한계를 같이 보고한다.
    assert "반폭이 ~250 ms" in out


def test_closed_loop_default_keeps_the_frequency_comparison(capsys):
    analyse(*_synth(0.08, freq=1.0), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=1.0)
    out = capsys.readouterr().out
    assert "실측 지배 주파수" in out
    assert "개루프" not in out


def _synth6(tau_s, axis_col, inertia, duration=40.0, freq=2.0):
    """6열 형식([force3, torque3] / [linear3, angular3])으로 한 축에만 신호."""
    t = np.arange(0.0, duration, 1.0 / FS)
    f = 5.0 * np.sin(2 * np.pi * freq * t)
    lag = int(round(tau_s * FS))
    a = np.concatenate([np.zeros(lag), f[: len(f) - lag]]) / inertia
    v = np.cumsum(a) / FS
    wr = np.zeros((len(t), 6)); wr[:, axis_col] = f
    st = np.zeros((len(t), 6)); st[:, axis_col] = v
    return t, wr, t, st, np.array([t[0]]), np.array([True])


@pytest.mark.parametrize("tau", [0.04, 0.08])
def test_yaw_axis_recovers_known_delay_from_torque_and_gyro(tau, capsys):
    """yaw 는 통신+추진기 지연만 재는 축이다 -- 자이로 직접, ESC 역전 없음."""
    analyse(*_synth6(tau, 5, 0.559), axis="yaw", m_eff=0.559,
            control_dt=CONTROL_DT, skip_s=1.0, open_loop=True)
    out = capsys.readouterr().out
    line = next(l for l in out.splitlines() if "피크 lag" in l)
    got_ms = float(line.split("=")[1].split("ms")[0])
    assert abs(got_ms - tau * 1000) <= 20.0, line
    assert "I_eff" in out and "kg*m^2" in out


def test_six_column_arrays_still_analyse_linear_axes(capsys):
    """6열 bag 에서 heave 를 고르면 예전과 같은 열(2)을 읽어야 한다."""
    analyse(*_synth6(0.08, 2, 28.1), axis="heave", m_eff=28.1,
            control_dt=CONTROL_DT, skip_s=1.0)
    line = next(l for l in capsys.readouterr().out.splitlines() if "피크 lag" in l)
    assert abs(float(line.split("=")[1].split("ms")[0]) - 80.0) <= 20.0


def test_angular_axis_refuses_three_column_arrays():
    with pytest.raises(SystemExit):
        analyse(*_synth(0.08), axis="yaw", m_eff=0.559,
                control_dt=CONTROL_DT, skip_s=1.0)


def test_closed_loop_comparison_uses_the_oscillation_band_when_mission_dominates(capsys):
    """명령이 저주파(미션 주기)에 눌려 있어도 1~5 Hz 진동을 잡아 예측과 비교한다.
    2026-09-02 A1 에서 0.04 Hz 가 지배해 '실측' 줄이 통째로 빠졌다."""
    t = np.arange(0.0, 40.0, 1.0 / FS)
    slow = 60.0 * np.sin(2 * np.pi * 0.04 * t)          # 미션 주기
    osc = 8.0 * np.sin(2 * np.pi * 2.3 * t)              # 진동
    f = slow + osc
    lag = int(round(0.08 * FS))
    a = np.concatenate([np.zeros(lag), f[: len(f) - lag]]) / 28.1
    v = np.cumsum(a) / FS
    wr = np.zeros((len(t), 3)); wr[:, 2] = f
    st = np.zeros((len(t), 3)); st[:, 2] = v
    analyse(t, wr, t, st, np.array([t[0]]), np.array([True]),
            axis="heave", m_eff=28.1, control_dt=CONTROL_DT, skip_s=1.0)
    out = capsys.readouterr().out
    assert "진동대 1~5 Hz" in out
    line = next(l for l in out.splitlines() if "실측 지배 주파수" in l)
    assert "2.3" in line and "일치" in line and "진동대" in line


def test_xcorr_delay_recovers_known_fc_clock_lag():
    """M4 핵심: FC 시계 위 두 신호의 알려진 지연을 되찾는가.

    서보(사각파)와 자이로 각가속(같은 파형이 τ 늦게 + 잡음)을 FC 시계로 만들어
    xcorr_delay 가 τ 를 격자 해상도(5 ms) 안에서 복원해야 한다. 두 신호의
    표본율이 달라도(서보 25 Hz, 자이로 50 Hz) 성립해야 한다 — 실제 bag 이
    그렇다.
    """
    from brov_base.diag_loop_delay import xcorr_delay

    rng = np.random.default_rng(7)
    tau = 0.045
    t_sv = np.arange(0.0, 40.0, 1 / 25)          # 서보 25 Hz
    t_gy = np.arange(0.0, 40.0, 1 / 50)          # 자이로 50 Hz
    sq = lambda t: np.sign(np.sin(2 * np.pi * 1.0 * t))
    sv = 1500 + 120 * sq(t_sv)
    gy = 3.0 * sq(t_gy - tau) + rng.normal(0, 0.2, len(t_gy))

    lag, r, _, _ = xcorr_delay(t_sv, sv, t_gy, gy)
    assert abs(lag - tau) <= 0.005 + 1e-9, f"lag {lag*1000:.1f} ms (기대 45)"
    assert r > 0.9, f"r={r:.3f}"


def test_servo_out_carries_the_fc_clock_not_wall_clock():
    """servo_out 의 header.stamp 는 FC boot 시계여야 하고, 같은 seq 는 재발행하지
    않아야 한다.

    M4 는 servo↔ahrs 의 header 끼리 교차상관한다. 벽시계가 섞이면 '링크 무관
    측정'이라는 존재 이유가 사라진다.

    **DDS 를 쓰지 않는다.** 구판은 구독으로 수신을 세었는데, 전체 스위트에서
    discovery 타이밍에 따라 어긋났고, 실패가 rclpy context 누수(try/finally
    부재)로 이어져 뒤 파일들의 rclpy.init() 을 전부 ERROR 로 만들었다.
    publisher.publish 를 직접 캡처하면 결정론적이다.
    """
    import rclpy

    from brov_base.base_node import BaseNode

    class _Iface:
        def __init__(self):
            self.sent = []
            self._servo_seq = 1

        def bump(self):
            self._servo_seq += 1

        def snapshot(self):
            import torch
            return {
                "att_quat_ned": torch.tensor([1.0, 0, 0, 0]),
                "pos_ned": torch.zeros(3), "vel_ned": torch.zeros(3),
                "body_rates_ned": torch.zeros(3),
                "att_age_s": 0.01, "pos_age_s": 0.01,
                "att_seq": 1, "pos_seq": 1,
                "att_time_boot_ms": 123456,
                "press_abs_hpa": [None, None, None],
                "press_age_s": [float("inf")] * 3,
                "press_seq": [0, 0, 0],
                "ekf_vel_variance": None,
            }

        def control_snapshot(self):
            import torch
            return {
                "heartbeat_age_s": 0.1, "custom_mode": 19, "armed": False,
                "servo_output_us": torch.tensor(
                    [1500, 1500, 1500, 1500, 1600, 1500, 1500, 1500],
                    dtype=torch.int32),
                "servo_time_usec": 123_456_789,   # FC boot 123.456789 s
                "servo_seq": self._servo_seq,
            }

        def send_pwm(self, pwm): self.sent.append(pwm)
        def neutral_stop(self): pass
        def enable_passthrough(self): pass
        def arm(self): pass
        def disarm(self): pass
        def close(self, send_stop=True): pass
        def get_parameter(self, n, timeout=5.0): return None

    we_inited = not rclpy.ok()
    if we_inited:
        rclpy.init()
    node = None
    try:
        node = BaseNode(interface=_Iface())
        node._publish_sensor_topics = True

        class _CapturePub:
            def __init__(self):
                self.msgs = []

            def publish(self, msg):
                self.msgs.append(msg)

        cap = _CapturePub()
        node._pub_servo_out = cap
        node._last_servo_seq = None

        node._publish_sensor_sample(node._interface.snapshot())
        assert len(cap.msgs) == 1, "servo_out 이 발행되지 않았다"
        m = cap.msgs[-1]
        fc = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        assert abs(fc - 123.456789) < 1e-6, f"stamp 가 FC 시계가 아니다: {fc}"
        assert list(m.position)[4] == 1600.0

        # seq 를 올리지 않고 다시 = FC 가 새 표본을 안 준 상황 → 재발행 금지
        node._publish_sensor_sample(node._interface.snapshot())
        assert len(cap.msgs) == 1, "같은 servo_seq 를 중복 발행했다"

        node._interface.bump()
        node._publish_sensor_sample(node._interface.snapshot())
        assert len(cap.msgs) == 2, "새 seq 인데 발행되지 않았다"
    finally:
        if node is not None:
            node.destroy_node()
        if we_inited:
            rclpy.shutdown()


# ---------------------------------------------------------------- 봉우리 품질
# 1 Hz 사각파는 봉우리가 ~250 ms 반폭으로 흐리고, chirp 은 뾰족하다. 그 차이를
# 숫자로 내지 못하면 "80 ms" 가 얼마나 믿을 값인지 말할 수 없다 (2026-09-03).
from brov_base.diag_loop_delay import peak_quality


def test_peak_quality_refines_between_grid_points():
    """격자 20 ms 인데 참값이 85 ms 면 포물선 보간이 그쪽으로 당겨야 한다."""
    lags = np.arange(0, 8)
    step = 0.020
    true_tau = 0.085
    cc = np.exp(-((lags * step - true_tau) / 0.04) ** 2)
    q = peak_quality(cc, lags, step)
    assert q["tau"] == pytest.approx(0.080)          # 격자 위 최댓값
    assert abs(q["refined"] - true_tau) < 0.006      # 보간이 참값에 더 가깝다
    assert abs(q["refined"] - true_tau) < abs(q["tau"] - true_tau)


def test_sharp_peak_has_smaller_half_width_than_flat_one():
    """chirp(뾰족) vs 사각파(평평) 를 반폭과 대비로 구분한다.

    실측 프로파일을 그대로 쓴다 -- 2026-09-03 a2_yaw(사각파) 와 a2_chirp.
    """
    step = 0.020
    square = np.array([0.743, 0.762, 0.780, 0.795, 0.809, 0.790, 0.706, 0.574,
                       0.411, 0.246, 0.105, -0.016, -0.122, -0.214, -0.292, -0.361])
    chirp = np.array([-0.205, -0.026, 0.271, 0.616, 0.833, 0.766, 0.483, 0.066,
                      -0.305, -0.468, -0.465, -0.345, -0.210, -0.154, -0.141, -0.135])
    lags = np.arange(len(square))
    qs, qc = peak_quality(square, lags, step), peak_quality(chirp, lags, step)
    assert qs["tau"] == pytest.approx(0.080) and qc["tau"] == pytest.approx(0.080)
    assert qc["half_width"] < qs["half_width"], (
        f"chirp 이 더 뾰족해야 한다: {qc['half_width']:.3f} vs {qs['half_width']:.3f}")
    assert qc["contrast"] > qs["contrast"]


def test_peak_quality_survives_a_peak_at_the_edge():
    """M4 처럼 0 ms 에서 최대인 경우에도 죽지 않는다."""
    lags = np.arange(0, 8); step = 0.020
    cc = np.linspace(0.33, -0.21, len(lags))
    q = peak_quality(cc, lags, step)
    assert q["tau"] == pytest.approx(0.0)
    assert q["refined"] == pytest.approx(0.0)


def test_transit_separates_queueing_from_constant_delay(capsys):
    """G4 분해 ①의 수학 — 큐잉과 상수 지연을 실제로 구분하는가.

    상수 transit(모든 메시지 같은 지연)이면 `d − min(d)` 가 0 근처에 몰려야
    하고, burst 큐잉(메시지마다 대기 시간이 다름)이면 p90 이 그 대기 폭만큼
    벌어져야 한다. offset(시계 차)이 얼마든 결과가 같아야 한다 — 분해가
    offset-free 라는 주장 자체의 검증이다.
    """
    from brov_base.diag_loop_delay import analyse_transit

    rng = np.random.default_rng(11)
    n = 500
    t_fc = np.arange(n) * 0.04
    CLOCK_OFFSET = 12345.678            # 임의의 시계 차 — 결과에 영향 없어야 함

    DRIFT_PPM = 3000e-6                 # SITL 실측급 0.3% drift — 결과 불변이어야 함

    def run(extra_wait):
        # 선형 drift + 느린 wobble(±8 ms, 20 s 주기 — SITL RTF 요동 모사)까지
        # 걸어도 큐잉 판정이 불변이어야 한다.
        wobble = 0.008 * np.sin(2 * np.pi * t_fc / 20.0)
        arrival = t_fc * (1 + DRIFT_PPM) + CLOCK_OFFSET + 0.020 + wobble + extra_wait
        sv = np.column_stack([arrival, t_fc] + [np.full(n, 1500.0)] * 8)
        gy = np.column_stack([arrival + 0.002, t_fc, *(np.zeros((3, n)))])
        analyse_transit(sv, gy)
        out = capsys.readouterr().out
        line = next(l for l in out.splitlines() if l.startswith("  servo"))
        p90 = float(line.split("p90")[1].split()[0])
        return p90, out

    # 상수 지연: p90 ≈ 0
    p90_const, out_const = run(np.zeros(n))
    # ±8 ms wobble 이 2 s 창을 조금 새어나와(수 ms) 0 은 아니다 — 판정 문턱
    # (15 ms)과 큐잉 케이스(>25 ms)에서 충분히 떨어져 있으면 된다.
    assert p90_const < 8.0, f"상수 지연인데 큐잉으로 읽힘: p90={p90_const}"
    assert "평평하다" in out_const

    # burst 큐잉: 메시지마다 0~35 ms 대기 (115200 baud burst 시나리오)
    p90_queue, out_queue = run(rng.uniform(0.0, 0.035, n))
    assert 25.0 < p90_queue < 36.0, f"큐잉 폭 복원 실패: p90={p90_queue}"
    assert "대기" in out_queue


def test_transit_early_outliers_are_not_queueing(capsys):
    """09-03 실기 a2_yaw bag 의 함정: servo 의 3 % 가 한 telemetry 주기(40 ms)
    먼저 도착하면 min 이 그 이상치가 되어 'min 대비' 가 40 ms 로 뜬다. 판정은
    덩어리의 폭(p90−p10)으로 해야 '평평' 이 나온다. 그리고 토픽 간 중앙 차는
    원시 d 로 재야 servo 가 ahrs 보다 늦게 오는 것이 보인다(잔차로 재면 항상 0)."""
    from brov_base.diag_loop_delay import analyse_transit

    rng = np.random.default_rng(3)
    n = 800
    t_fc = np.arange(n) * 0.04
    arrival = t_fc + 500.0 + 0.060
    early = rng.random(n) < 0.03
    arrival_sv = arrival - np.where(early, 0.040, 0.0)
    sv = np.column_stack([arrival_sv, t_fc] + [np.full(n, 1500.0)] * 8)
    gy = np.column_stack([arrival - 0.012, t_fc, *(np.zeros((3, n)))])
    analyse_transit(sv, gy)
    out = capsys.readouterr().out
    assert "평평하다" in out, out
    assert "큐잉이 아니다" in out, out
    line = next(l for l in out.splitlines() if "토픽 간 중앙 차" in l)
    dm = float(line.split(":")[1].split()[0])
    assert 10.0 < dm < 14.0, line
