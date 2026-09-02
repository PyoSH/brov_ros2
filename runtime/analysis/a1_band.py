"""A1: 1.8~2.6 Hz 대역의 **절대** 진폭 (명령·응답), 이득 1.0 vs 0.5."""
import sys, numpy as np
from brov_base.diag_loop_delay import read_bag
import os
# 분석 구간 상한 [s] — 환경변수 A1_SECONDS. EKF 가 발산한 꼬리를 잘라
# 두 주행을 같은 길이로 비교할 때 쓴다 (2026-09-03 delayA 78 s 이후 발산).
_LIMIT = float(os.environ.get("A1_SECONDS", "0")) or None

AX = {"surge": (0, 0, 20.995), "sway": (1, 1, 21.755), "heave": (2, 2, 28.135), "yaw": (5, 5, 0.559)}
def band_rms(x, fs, lo, hi):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1/fs)
    m = (f >= lo) & (f <= hi)
    return np.sqrt(2 * np.sum(np.abs(X[m])**2) / len(x)**2), f[m][np.argmax(np.abs(X[m]))] if m.any() else np.nan
def run(path):
    wr_t, wr_f, st_t, st_v, act_t, act_v = read_bag(path)
    t0 = act_t[act_v][0] + 5.0; t1 = min(wr_t[-1], st_t[-1]);  t1 = min(t1, t0 + _LIMIT) if _LIMIT else t1
    fs = 50.0; grid = np.arange(t0, t1, 1/fs)
    out = {}
    for ax, (ci, vi, m) in AX.items():
        f = np.interp(grid, wr_t, wr_f[:, ci]); v = np.interp(grid, st_t, st_v[:, vi]); a = np.gradient(v, grid)
        cb, cf = band_rms(f, fs, 1.8, 2.6); ab, af = band_rms(a, fs, 1.8, 2.6)
        out[ax] = dict(cmd_band=cb, cmd_f=cf, acc_band=ab, acc_f=af, cmd_mean=f.mean(), cmd_rms=f.std(), acc_rms=a.std())
    return out, t1 - t0
# 라벨은 argv[3], argv[4] 로 바꿀 수 있다 (정책 A/B 에도 그대로 쓴다).
L1 = sys.argv[3] if len(sys.argv) > 3 else "A(기준)"
L2 = sys.argv[4] if len(sys.argv) > 4 else "B(비교)"
r10, d10 = run(sys.argv[1]); r05, d05 = run(sys.argv[2])
print(f"분석 구간: {L1} {d10:.0f}s, {L2} {d05:.0f}s   (1.8~2.6 Hz 대역 RMS)\n")
print(f"{'축':6s} {'':8s} {'명령 대역 RMS':>14s} {'@Hz':>5s} {'응답 대역 RMS':>14s} {'@Hz':>5s} {'명령 평균':>9s} {'명령 전체RMS':>11s}")
for ax in AX:
    u = "N*m" if ax == "yaw" else "N"; ua = "rad/s²" if ax == "yaw" else "m/s²"
    for lbl, r in ((L1, r10), (L2, r05)):
        d = r[ax]
        print(f"{ax:6s} {lbl:8s} {d['cmd_band']:10.2f} {u:>3s} {d['cmd_f']:5.2f} {d['acc_band']:9.3f} {ua:>6s} {d['acc_f']:5.2f} {d['cmd_mean']:+9.2f} {d['cmd_rms']:11.2f}")
    a, b = r10[ax], r05[ax]
    print(f"{'':6s} 비(B/A)       명령 대역 x{b['cmd_band']/max(a['cmd_band'],1e-9):.2f}   응답 대역 x{b['acc_band']/max(a['acc_band'],1e-9):.2f}\n")
# heave 추진기 동작점: 수직 추진기 4개, deadband ~0.45 N/추진기
for lbl, r in ((L1, r10), (L2, r05)):
    print(f"heave 명령 평균 {lbl}: {r['heave']['cmd_mean']:+.2f} N  -> 수직 추진기당 {r['heave']['cmd_mean']/4:+.2f} N  (deadband 가장자리 ~0.45 N)")
