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
        return abs(float(line.split("r =")[1]))

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
