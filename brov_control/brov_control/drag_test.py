"""실기 surge 항력 측정의 순수 로직 — ROS/MAVLink 의존 없음.

무엇을 왜 재는가
================
Sim2Swim 보상 Eq.(8) ``r_a = w_a e^(-||a||)``은 정상상태에서 0이 되지 않는 행동
비용을 만들고, 속도항 Eq.(6)은 오차 0 근방에서 기울기가 0이다. 두 항의 균형점을
결정하는 값은 하나다:

    A = drag(0.5 m/s) / 최대 surge 추력

    가설 A (sim 계수 Xu=13.7, Xuu=141):  v_max 0.88 m/s,  A 0.340
    가설 B (제조사 사양):                v_max 1.48 m/s,  A 0.149

이 모듈은 그 판정에 필요한 것만 담는다 — 정상상태 판정, 상태기계, 안전 한계.
적합(``fit_drag``)과 추력 할당(``_Allocator``)은 이미 검증된
``brov_base.diag_terminal_velocity``의 것을 그대로 쓴다.

ROS와 분리한 이유는 시험 가능성이다. 상태 전이와 판정은 물 없이, rclpy 없이
전부 검증할 수 있어야 한다.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class Phase(str, Enum):
    """수준 하나를 처리하는 동안의 단계."""

    ASCEND = "ASCEND"            # 바닥에서 이륙 — 목표 깊이까지 상승 후 호버
    APPROACH = "APPROACH"        # 주행 시점으로 이동
    SETTLE_YAW = "SETTLE_YAW"    # 방위 정렬 대기
    RUN = "RUN"                  # open-loop surge, 종단속도 측정
    COAST = "COAST"              # 중립 타행 — 추력테이블 무관 교차검증
    TURNAROUND = "TURNAROUND"    # 180° 선회
    WAIT = "WAIT"                # 재순환 안정화
    DONE = "DONE"


@dataclass(frozen=True)
class Limits:
    """pool 프레임 절대좌표 기준 안전 한계.

    기존 스크립트는 시작점 기준 상대 주행거리를 썼는데, 그러면 벽까지 남은
    거리를 모른다. 여기서는 절대 위치로 판정한다.
    """

    run_x_min: float
    run_x_max: float
    lane_y: float
    max_cross_track_m: float
    z_min: float
    z_max: float
    target_z: float
    max_z_error_m: float
    max_tilt_rad: float

    def violation(self, x: float, y: float, z: float,
                  roll: float, pitch: float,
                  *, check_depth: bool = True) -> str | None:
        """한계를 벗어났으면 사유 문자열, 아니면 None.

        ``check_depth=False``는 깊이 두 항(``z_min``/``z_max`` 절대 한계와
        목표 깊이 오차)을 함께 끈다. 바닥에서 이륙하는 ASCEND 단계는 정의상
        두 항 모두를 위반한 상태로 시작하므로 첫 틱에 중단된다. 그 구간의
        깊이는 노드가 별도 상승 회랑으로 감시한다. 차선과 자세 한계는 끄지
        않는다.
        """
        if not (self.run_x_min <= x <= self.run_x_max):
            return f"주행축 한계 x={x:.2f} (허용 {self.run_x_min:.2f}~{self.run_x_max:.2f})"
        if abs(y - self.lane_y) > self.max_cross_track_m:
            return f"차선 이탈 {y - self.lane_y:+.2f} m"
        if check_depth and not (self.z_min <= z <= self.z_max):
            return f"깊이 한계 z={z:.2f} (허용 {self.z_min:.2f}~{self.z_max:.2f})"
        if check_depth and abs(z - self.target_z) > self.max_z_error_m:
            return f"깊이 이탈 {z - self.target_z:+.2f} m"
        if max(abs(roll), abs(pitch)) > self.max_tilt_rad:
            return (f"자세 이탈 roll {math.degrees(roll):+.0f}° "
                    f"pitch {math.degrees(pitch):+.0f}°")
        return None


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lsq_slope(ts: list[float], us: list[float]) -> float:
    """최소자승 du/dt.

    끝점차분(``(u[-1]-u[0])/span``)을 쓰면 안 된다. 양 끝 두 샘플만 쓰므로 잡음이
    ``sigma*sqrt(2)/span``인데, 짧은 수조가 요구하는 창 1.2 s / EKF 속도잡음
    0.02 m/s에서 0.024가 되어 판정 문턱(0.02~0.03)을 넘는다. 몬테카를로에서
    진짜 정상상태를 41.5% 오기각했다. 최소자승은 ``sigma*sqrt(12/(N*span^2))``로
    같은 조건에서 0.011이고 오기각 6.0%다.
    """
    n = len(ts)
    if n < 2:
        return float("inf")
    tbar = statistics.fmean(ts)
    ubar = statistics.fmean(us)
    sxx = sum((t - tbar) ** 2 for t in ts)
    if sxx <= 0.0:
        return float("inf")
    return sum((t - tbar) * (u - ubar) for t, u in zip(ts, us)) / sxx


@dataclass
class SteadyDetector:
    """마지막 ``window_s`` 구간으로 정상상태를 판정한다.

    거리 한계로 주행이 중단돼도 이 판정은 그대로 수행한다 — 중단 직전 창이
    종단속도에 들어와 있으면 유효한 표본이다. 짧은 수조를 성립시키는 지점이다.
    """

    window_s: float
    max_slope: float
    max_sd: float
    min_samples: int = 5
    _series: list[tuple[float, float, float]] = field(default_factory=list)

    def add(self, t: float, u: float, tau_x: float) -> None:
        self._series.append((t, u, tau_x))

    @property
    def samples(self) -> int:
        return len(self._series)

    def tail(self, window_s: float | None = None) -> list[tuple[float, float, float]]:
        if not self._series:
            return []
        span = self.window_s if window_s is None else float(window_s)
        cutoff = self._series[-1][0] - span
        return [s for s in self._series if s[0] >= cutoff]

    def evaluate(self, window_s: float | None = None) -> dict:
        """정상상태 여부와 그 구간의 평균 속도/전달 추력.

        ``window_s``로 창을 바꿔 볼 수 있다. 짧은 창은 가속 중의 일시적
        평탄부(살짝 내려갔다 다시 오르는 구간)를 정상상태로 오인한다 —
        실측에서 level 0.20이 t=2.09에 u=0.243으로 통과했지만 그 뒤로도
        계속 올랐다. 긴 창으로 한 번 더 확인하는 용도다.
        """
        tail = self.tail(window_s)
        if len(tail) < self.min_samples:
            return {"steady": False, "n_tail": len(tail),
                    "reason": f"표본 부족 {len(tail)}/{self.min_samples}"}
        ts = [s[0] for s in tail]
        us = [s[1] for s in tail]
        taus = [s[2] for s in tail]
        slope = lsq_slope(ts, us)
        sd = statistics.pstdev(us)
        span = ts[-1] - ts[0]
        steady = abs(slope) < self.max_slope and sd < self.max_sd
        reason = ""
        if not steady:
            parts = []
            if abs(slope) >= self.max_slope:
                parts.append(f"|du/dt| {abs(slope):.4f} >= {self.max_slope}")
            if sd >= self.max_sd:
                parts.append(f"sd {sd:.4f} >= {self.max_sd}")
            reason = ", ".join(parts)
        return {
            "steady": steady,
            "u_mps": statistics.fmean(us),
            "u_sd": sd,
            "du_dt": slope,
            "tau_x_n": statistics.fmean(taus),
            "window_s": span,
            "n_tail": len(tail),
            "reason": reason,
        }


def coast_fit(samples: list[tuple[float, float]], mass_eff: float,
              xu_known: float | None = None) -> dict:
    """중립 타행 감속곡선에서 Xuu를 적합한다 — 추력 테이블과 무관한 교차검증.

    ``M du/dt = -(Xu u + Xuu u|u|)``. 추력이 0인 구간이므로 T200 정지추력
    테이블의 정확도가 **전혀 개입하지 않는다.** 주행 구간의 적합은 정지추력으로
    나눈 '유효 항력'(전진비 손실 포함)인 반면 이쪽은 순수 유체력이라, 둘을
    비교하면 테이블 배율 오차를 잡아낼 수 있다. 거리는 0.3~0.8 m만 쓴다.

    추정 방식 — 미분이 아니라 누적적분
    ==================================
    측정 속도를 미분하면 안 된다. 25 Hz에서 시상수가 0.08 s(가설 A)까지 짧아
    중앙차분의 잡음이 신호를 덮는다. 대신 구간 [t0, tk]를 적분한 형태

        -M (u_k - u_0) = Xu * ∫u dt + Xuu * ∫u^2 dt

    를 k마다 한 행으로 쌓는다. 누적적분은 잡음이 √N으로 줄고 좌변 Δu가 크게
    자라므로 SNR이 훨씬 낫다.

    **Xu는 타행에서 식별되지 않는다.** 관측 속도 범위에서 u와 u^2이 거의
    공선이라, 속도잡음 σ=0.02 m/s에서 2-파라미터 적합의 Xu 오차가 -44~-108%까지
    간다. 그래서 ``xu_known``(정상상태 적합에서 나온 Xu — 거기서는 5개의 분리된
    작동점이 있어 잘 식별된다)을 받아 **Xuu만** 푼다. 이때 Xuu 오차는 σ=0.02에서
    -1.3~-3.7%다. ``xu_known``이 None이면 둘 다 풀되 ``xu_identifiable=False``로
    표시한다 — 그 Xu는 보고에 쓰지 말 것.

    쟁점이 Xuu(141이냐 46이냐)이므로 이 한 파라미터만으로 판정에 충분하다.
    """
    usable = [(t, u) for t, u in samples if u > 0.05]
    if len(usable) < 6:
        return {"ok": False, "reason": f"타행 표본 {len(usable)}개 (최소 6개)"}

    u0 = usable[0][1]
    i1 = i2 = 0.0
    rows: list[tuple[float, float, float]] = []
    for i in range(len(usable) - 1):
        (ta, ua), (tb, ub) = usable[i], usable[i + 1]
        dt = tb - ta
        if dt <= 0.0:
            continue
        i1 += 0.5 * (ua + ub) * dt              # ∫u dt   (사다리꼴)
        i2 += 0.5 * (ua * ua + ub * ub) * dt    # ∫u^2 dt
        rows.append((i1, i2, -mass_eff * (ub - u0)))
    if len(rows) < 4:
        return {"ok": False, "reason": f"적분 구간 {len(rows)}개 (최소 4개)"}

    if xu_known is not None:
        xu = float(xu_known)
        num = sum(b * (c - xu * a) for a, b, c in rows)
        den = sum(b * b for _, b, _ in rows)
        if den <= 1e-12:
            return {"ok": False, "reason": "∫u^2 이 0에 가깝다 — 타행 구간이 짧다"}
        xuu = num / den
        identifiable = False
    else:
        s11 = sum(a * a for a, _, _ in rows)
        s12 = sum(a * b for a, b, _ in rows)
        s22 = sum(b * b for _, b, _ in rows)
        t1 = sum(a * c for a, _, c in rows)
        t2 = sum(b * c for _, b, c in rows)
        det = s11 * s22 - s12 * s12
        if abs(det) < 1e-12:
            return {"ok": False, "reason": "정규방정식이 특이 — 속도 범위가 좁다"}
        xu = (t1 * s22 - t2 * s12) / det
        xuu = (s11 * t2 - s12 * t1) / det
        identifiable = True

    resid = [c - (xu * a + xuu * b) for a, b, c in rows]
    ss_res = sum(r * r for r in resid)
    mean_c = statistics.fmean([c for _, _, c in rows])
    ss_tot = sum((c - mean_c) ** 2 for _, _, c in rows)
    return {
        "ok": True,
        "Xu": xu,
        "Xuu": xuu,
        "xu_identifiable": identifiable,
        "xu_source": "coast_2param" if identifiable else "steady_state",
        "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
        "n_samples": len(rows),
        "u_start": u0,
        "u_end": usable[-1][1],
    }


def recirculation_check(results: list[dict], tolerance: float = 0.05) -> dict:
    """같은 level의 반복 측정을 비교해 재순환 편향을 검사한다.

    물 7 m^3에 제트를 반복 주입하면 벽 반사류가 생기고, 기체가 자기가 만든
    흐름을 타면 종단속도가 편향된다. 3~4 m 수조에서 가장 큰 계통오차이며
    수준 간 대기시간으로만 완화된다. 최저 수준을 처음과 마지막에 두 번 재서
    비교하는 것이 이를 잡는 유일한 방법이다.
    """
    by_level: dict[float, list[float]] = {}
    for r in results:
        if r.get("steady") and "u_mps" in r:
            by_level.setdefault(round(float(r["level"]), 4), []).append(
                float(r["u_mps"])
            )
    repeats = {lv: us for lv, us in by_level.items() if len(us) >= 2}
    if not repeats:
        return {"checked": False,
                "reason": "반복 측정된 수준이 없다 — 최저 수준을 두 번 넣을 것"}
    worst_level, worst_rel = None, 0.0
    for lv, us in repeats.items():
        lo, hi = min(us), max(us)
        rel = abs(hi - lo) / hi if hi > 1e-9 else float("inf")
        if rel > worst_rel:
            worst_level, worst_rel = lv, rel
    return {
        "checked": True,
        "biased": worst_rel > tolerance,
        "worst_level": worst_level,
        "worst_relative_spread": worst_rel,
        "detail": {lv: us for lv, us in repeats.items()},
    }


@dataclass
class LevelPlan:
    """한 수준의 주행 계획 — 어느 방향으로, 어디서 어디까지."""

    level: float
    heading: float          # pool 프레임 목표 방위 [rad]
    start_x: float
    forward: bool           # True면 pool +X 방향


def build_level_plans(levels: list[float], limits: Limits,
                      axis_heading: float, margin_m: float,
                      alternate_direction: bool = True) -> list[LevelPlan]:
    """수준마다 번갈아 방향을 바꾸는 왕복 계획을 만든다.

    T200은 역추력(-51.5 N)이 정추력(+64.1 N)보다 약해서 전/후진을 섞으면 적합이
    오염된다. 그래서 되돌아올 때도 뱃머리를 돌려 항상 전진한다.

    ``alternate_direction=False``는 왕복을 끄고 모든 수준을 같은 방향·같은
    출발점으로 만든다. 수준마다 운용자가 기체를 원하는 자리로 옮기고 다시
    prepare하는 단발 운용(single_level_per_start)에서 쓴다 — 그때 출발점은
    항상 '방금 prepare한 자리'이고, 선회는 노드가 하지 않는다.
    """
    plans: list[LevelPlan] = []
    forward = True
    for level in levels:
        start_x = (limits.run_x_min + margin_m if forward
                   else limits.run_x_max - margin_m)
        plans.append(LevelPlan(
            level=float(level),
            heading=wrap_pi(axis_heading if forward else axis_heading + math.pi),
            start_x=start_x,
            forward=forward,
        ))
        if alternate_direction:
            forward = not forward
    return plans


def _transient_blocks(samples: list[dict], smooth: int):
    """수준별로 나눠 평활·미분한다.

    수준마다 t가 -pre_run_s에서 다시 시작하므로, 전부 합쳐 미분하면 수준
    경계에서 시간이 되감기며 du/dt가 발산한다. 반드시 나눠서 미분한 뒤
    설계행렬만 쌓아야 한다.
    """
    groups: dict[object, list[tuple[float, float, float]]] = {}
    for entry in samples:
        if entry.get("t") is None or entry.get("u") is None:
            continue
        if entry.get("tau_x_delivered") is None:
            continue
        key = entry.get("level")
        groups.setdefault(key, []).append(
            (float(entry["t"]), float(entry["u"]),
             float(entry["tau_x_delivered"]))
        )

    k = smooth if smooth % 2 else smooth + 1
    kernel = np.ones(k) / k
    blocks = []
    for rows in groups.values():
        rows.sort(key=lambda r: r[0])
        # 같은 시각이 중복되면 미분이 0으로 나눈다 — 앞의 것만 남긴다.
        dedup = [rows[0]]
        for row in rows[1:]:
            if row[0] > dedup[-1][0]:
                dedup.append(row)
        if len(dedup) < max(20, k * 2):
            continue
        ts = np.array([r[0] for r in dedup])
        us = np.array([r[1] for r in dedup])
        taus = np.array([r[2] for r in dedup])
        u_s = np.convolve(us, kernel, mode="valid")
        half = k // 2
        t_s, tau_s = ts[half:len(ts) - half], taus[half:len(taus) - half]
        n = min(len(u_s), len(t_s), len(tau_s))
        u_s, t_s, tau_s = u_s[:n], t_s[:n], tau_s[:n]
        if n < 20 or float(np.ptp(t_s)) <= 0.0:
            continue
        # 추력이 계단으로 바뀌는 지점(인가·차단)은 평활 창이 뭉개서 du/dt를
        # 왜곡한다. 그대로 두면 m_eff가 15%쯤 높게 치우친다. 계단 주변
        # 창 하나만큼을 버린다 — 나머지 구간은 그대로 쓴다.
        # 실제 tau는 매 틱 조금씩 흔들린다(할당 왕복·PID 교차결합). 실측에서
        # |dtau| 중앙값이 0.0005 N인데 계단은 11~22 N이었다. 절대 문턱 1e-6을
        # 쓰면 지터의 69%가 계단으로 잡혀 블록이 통째로 비워진다 — 실제로
        # 과도구간 적합이 "쓸 수 있는 수준이 없다"로 죽었다. 추력 크기에
        # 비례한 문턱으로 인가·차단만 골라낸다.
        step_threshold = max(1.0, 0.2 * float(np.ptp(tau_s)))
        keep = np.ones(n, dtype=bool)
        steps = np.nonzero(np.abs(np.diff(tau_s)) > step_threshold)[0]
        for idx in steps:
            lo, hi = max(0, idx - k), min(n, idx + k + 1)
            keep[lo:hi] = False
        if int(keep.sum()) < 20:
            continue
        blocks.append((np.gradient(u_s, t_s)[keep], u_s[keep], tau_s[keep]))
    return blocks


def transient_fit(samples: list[dict], *, xu_known: float | None = None,
                  smooth: int = 9) -> dict:
    """가속 구간 전체로 (m_eff, Xu, Xuu)를 푼다 — 정상상태가 필요 없다.

    운동방정식은 매 표본에서 성립한다::

        tau = m_eff * du/dt + Xu * u + Xuu * u|u|

    정상상태 적합은 꼬리 창 한 개(약 30표본)만 쓰고 나머지를 버린다. 짧은
    수조에서 종단속도에 도달하지 못하는 수준도 가속 곡선은 온전히 남고, 그
    곡선이 같은 계수를 구속하므로 여기서 살려 쓴다. 추력이 0인 타행 구간까지
    함께 넣으면 m_eff와 항력항의 공선성이 크게 풀린다.

    부수적으로 ``m_eff``(질량+부가질량)를 **측정**한다. ``coast_fit``은 이 값을
    외부에서 받아 선형으로 곱하므로, 가정값을 쓰면 그만큼 타행 Xuu가 통째로
    치우친다. 여기서 얻은 값을 넣으면 그 의존이 끊긴다.

    한계는 ``coast_fit``과 같다 — 관측 속도 범위에서 ``u``와 ``u|u|``가 거의
    공선이라 Xu와 Xuu가 잘 분리되지 않는다. ``xu_known``을 주면 Xu를 고정하고
    나머지 둘만 푼다. ``xu_identifiable``이 False면 Xu를 보고에 쓰지 말 것.

    ``m_eff``는 **낮게 치우친다**. 회귀변수인 ``du/dt``가 미분으로 증폭된 잡음을
    품고 있어 그 계수가 0쪽으로 끌리는 감쇠편향(errors-in-variables)이다. 합성
    왕복에서 속도잡음 sd 0.004에 -11%, sd 0.010에 -20%였다. 반면 ``Xuu``는 같은
    조건에서 오차 1% 안쪽이다 — **Xuu는 그대로 쓰고, m_eff는 자릿수 확인용으로만
    볼 것.** ``coast_fit``에 넣을 때는 이 편향이 타행 Xuu를 같은 비율로 낮춘다.
    """

    blocks = _transient_blocks(samples, smooth)
    if not blocks:
        return {"ok": False, "reason": "쓸 수 있는 수준이 없다 (수준당 최소 20표본)"}

    dudt = np.concatenate([b[0] for b in blocks])
    u_s = np.concatenate([b[1] for b in blocks])
    tau_s = np.concatenate([b[2] for b in blocks])
    quad = u_s * np.abs(u_s)

    if xu_known is None:
        design = np.column_stack([dudt, u_s, quad])
        target = tau_s
    else:
        design = np.column_stack([dudt, quad])
        target = tau_s - float(xu_known) * u_s

    solution, *_ = np.linalg.lstsq(design, target, rcond=None)
    residual = target - design @ solution
    rms = float(np.sqrt(np.mean(residual ** 2)))

    if xu_known is None:
        m_eff, xu, xuu = (float(v) for v in solution)
    else:
        m_eff, xuu = (float(v) for v in solution)
        xu = float(xu_known)

    if float(np.std(u_s)) > 0.0 and float(np.std(quad)) > 0.0:
        collinearity = abs(float(np.corrcoef(u_s, quad)[0, 1]))
    else:
        collinearity = 1.0

    # tau는 수준 안에서 상수라 R^2가 무의미하다. 잔차를 추력 크기로 정규화한다.
    scale = float(np.mean(np.abs(tau_s))) or 1.0
    return {
        "ok": True,
        "m_eff_kg": m_eff,
        "Xu": xu,
        "Xuu": xuu,
        "rms_residual_n": rms,
        "rms_relative": rms / scale,
        "n_samples": int(len(u_s)),
        "n_levels": len(blocks),
        "u_range": [float(u_s.min()), float(u_s.max())],
        "collinearity": collinearity,
        "xu_identifiable": xu_known is None and collinearity < 0.98,
        "m_eff_plausible": 5.0 < m_eff < 60.0,
    }
