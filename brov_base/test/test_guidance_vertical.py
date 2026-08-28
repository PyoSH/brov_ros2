"""BF LOS의 수직축 거동 회귀 시험.

`test_guidance_depth_hold.py`를 대체한다. 구 depth-hold P 제어기는 두 가지를
겸하고 있었고, BF 전환 후 그중 하나만 남는다:

  (1) 수직 보정 잠식 우회  → BF의 독립 vertical-track 축(υ_d)이 대체. **제거됨**
  (2) 상승/하강 속도 제한  → BF가 대체하지 않는다. **`depth_speed_limit`로 유지**

(2)의 적용 방식이 바뀐 것을 여기서 고정한다 — 수직 성분만 clamp하면 명령
방향이 경로에서 틀어지므로, 이제는 **방향을 보존하고 전체 크기를 줄인다.**
"""

import math

import torch

from brov_base.guidance import LOSGuidance


_Q_ID = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
# NED. [0,0,0] → 0.5 m 하강 → 수평 1 m 전진.
_WP = torch.tensor([[[0.0, 0.0, 0.0], [0.0, 0.0, -0.5], [1.0, 0.0, -0.5]]])


def _guidance(**kw):
    kw.setdefault("depth_speed_limit", 0.1)
    g = LOSGuidance(_WP, "cpu", cruise_speed=0.1, reach_threshold=0.15,
                    heading_mode="straight", loop=False, **kw)
    g.reset(torch.tensor([0]), initial_quat=_Q_ID)
    return g


def test_vertical_track_error_is_corrected_on_a_horizontal_segment():
    """수평 구간에서도 경로 위/아래 이탈이 독립적으로 복원된다."""
    g = _guidance(lookahead_vert=0.2)
    v, _ = g.compute(torch.tensor([[0.0, 0.0, -0.4]]), _Q_ID)
    assert int(g._wp_idx[0]) == 1
    # NED에서 로봇(z=-0.4)이 경로(z=-0.5)보다 얕다 → 더 깊이(-z) 가야 한다.
    assert float(v[0, 2]) < 0.0
    # h=+0.1, υ_d = atan(-0.1/0.2), 수직 성분이 한계(0.1) 안이라 스케일 없음.
    expected = 0.1 * math.sin(math.atan(-0.1 / 0.2))
    assert abs(float(v[0, 2]) - expected) < 1e-6


def test_vertical_speed_limit_preserves_direction_and_scales_magnitude():
    """순수 수직 구간: BF는 전속을 수직에 싣고, 한계가 크기를 줄인다.

    구 구현은 v_d_world[2]만 clamp해서 다른 축과의 비율이 깨졌다. 이제는
    같은 비율로 전체를 줄이므로 방향이 보존된다.
    """
    g = _guidance(depth_speed_limit=0.05)
    v, _ = g.compute(torch.tensor([[0.0, 0.0, -0.05]]), _Q_ID)   # 하강 구간 위
    assert int(g._wp_idx[0]) == 0
    assert abs(float(v[0, 2]) + 0.05) < 1e-6      # 한계까지만
    assert abs(float(v[0, 0])) < 1e-6 and abs(float(v[0, 1])) < 1e-6


def test_vertical_limit_is_inactive_when_within_bound():
    """한계 안에서는 ||v_d|| = cruise_speed가 그대로 보존된다."""
    g = _guidance(depth_speed_limit=1.0)          # 사실상 무제한
    for p in ([0.0, 0.0, -0.05], [0.3, 0.2, -0.4], [0.9, -0.4, -0.6]):
        g._wp_idx[:] = 0 if p[2] > -0.4 else 1
        v, _ = g.compute(torch.tensor([p]), _Q_ID)
        assert abs(float(v.norm()) - 0.1) < 1e-6


def test_course_depends_on_cross_track_only_not_on_along_track_progress():
    """BF의 구조적 성질 — 조향각은 (e, h)만의 함수다.

    구 구현은 lookahead '지점'을 세그먼트 끝에 clamp했기 때문에 같은 이탈량이라도
    진행률에 따라 조향이 달라졌다.
    """
    g = _guidance(lookahead_vert=0.2, depth_speed_limit=1.0)
    g._loop = True
    g._wp_idx[:] = 1
    a, _ = g.compute(torch.tensor([[0.1, 0.2, -0.4]]), _Q_ID)
    g._wp_idx[:] = 1
    b, _ = g.compute(torch.tensor([[0.9, 0.2, -0.4]]), _Q_ID)
    assert torch.allclose(a, b, atol=1e-6)


def test_terminal_completion_continues_position_hold():
    g = _guidance()
    g.compute(torch.tensor([[0.0, 0.0, -0.5]]), _Q_ID)    # idx 0 → 1
    g.compute(torch.tensor([[0.95, 0.0, -0.5]]), _Q_ID)   # final 도달
    assert bool(g.mission_complete[0])
    v, _ = g.compute(torch.tensor([[0.8, 0.0, -0.2]]), _Q_ID)
    assert float(v[0, 0]) > 0.0                            # final waypoint로 복귀
    assert float(v[0, 2]) < 0.0                            # 다시 깊이로
