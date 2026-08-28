"""Case-A one-shot takeoff followed by a two-point horizontal loop."""

import math

import pytest
import torch

from brov_base import math_utils as mu
from brov_base.guidance import LOSGuidance


def _guidance() -> LOSGuidance:
    # Guidance coordinates are start-heading/NED-like: negative z is upward.
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, -0.5], [0.2, 0.0, -0.5]]],
        dtype=torch.float32,
    )
    guidance = LOSGuidance(
        waypoints,
        "cpu",
        cruise_speed=0.1,
        lookahead_dist=0.4,
        reach_threshold=0.15,
        heading_mode="takeoff_then_align",
        loop=True,
    )
    initial = mu.quat_from_euler_xyz(
        torch.tensor([0.0]),
        torch.tensor([0.0]),
        torch.tensor([0.4]),
    )
    guidance.reset(torch.tensor([0]), initial_quat=initial)
    return guidance


def test_takeoff_holds_initial_level_heading_then_enters_loop() -> None:
    guidance = _guidance()
    initial = guidance._straight_q_d.clone()

    _, takeoff_q = guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]), initial, advance_waypoint=False
    )
    assert torch.allclose(takeoff_q, initial, atol=1e-6)
    assert int(guidance._wp_idx[0]) == 0

    guidance.compute(
        torch.tensor([[0.0, 0.0, -0.4]]), initial, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 0

    _, outbound_q = guidance.compute(
        torch.tensor([[0.0, 0.0, -0.5]]), initial, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 1
    assert abs(float(mu.yaw_from_quat(outbound_q)[0])) < 1e-5

    guidance.compute(
        torch.tensor([[0.2, 0.0, -0.5]]), outbound_q, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 2
    _, target = guidance._current_and_next(guidance._wp_idx)
    assert torch.allclose(target[0], guidance._wp[0, 1])

    _, return_q = guidance.compute(
        torch.tensor([[0.2, 0.0, -0.5]]), outbound_q, advance_waypoint=False
    )
    assert abs(abs(float(mu.yaw_from_quat(return_q)[0])) - math.pi) < 1e-5

    guidance.compute(
        torch.tensor([[0.0, 0.0, -0.5]]), return_q, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 1


@pytest.mark.parametrize("loop,count", [(False, 2), (False, 4), (True, 2), (True, 4)])
def test_takeoff_mode_requires_one_prefix_plus_two_loop_points(
    loop: bool, count: int
) -> None:
    with pytest.raises(
        ValueError, match="requires exactly three waypoints"
    ):
        LOSGuidance(
            torch.zeros((1, count, 3)),
            "cpu",
            heading_mode="takeoff_then_align",
            loop=loop,
        )


def test_takeoff_can_finish_one_horizontal_leg_without_reversal() -> None:
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.2], [5.0, 0.0, 0.2]]],
        dtype=torch.float32,
    )
    guidance = LOSGuidance(
        waypoints,
        "cpu",
        cruise_speed=0.5,
        lookahead_dist=1.0,
        reach_threshold=0.5,
        heading_mode="takeoff_then_align",
        loop=False,
        depth_speed_limit=0.05,
    )
    identity = mu.identity_quat(1, "cpu")
    guidance.reset(torch.tensor([0]), initial_quat=identity)

    takeoff_v, _ = guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]), identity, advance_waypoint=False
    )
    # 절대허용오차 명시: BF LOS는 방향을 삼각함수로 만들므로 수직 구간에서
    # 수평 성분이 정확한 0이 아니다 — cos(pi/2)가 float32에서 -4.4e-08이다.
    # 구 구현은 [0,0,dz]를 정규화해 정확한 0이 나왔다. 남는 값은 2e-09로
    # 신호(0.05)보다 7자리 작아 물리적 의미가 없다.
    assert takeoff_v[0].tolist() == pytest.approx([0.0, 0.0, 0.05], abs=1e-6)

    outbound_v, _ = guidance.compute(
        torch.tensor([[0.0, 0.0, 0.2]]), identity, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 1
    assert outbound_v[0].tolist() == pytest.approx([0.5, 0.0, 0.0])

    hold_v, _ = guidance.compute(
        torch.tensor([[5.0, 0.0, 0.2]]), identity, advance_waypoint=True
    )
    assert bool(guidance.mission_complete[0])
    assert int(guidance._wp_idx[0]) == 1
    assert hold_v[0].tolist() == pytest.approx([0.0, 0.0, 0.0])


def test_cruise_speed_per_leg_uses_different_speed_per_horizontal_leg() -> None:
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.2], [2.0, 0.0, 0.2]]],
        dtype=torch.float32,
    )
    guidance = LOSGuidance(
        waypoints,
        "cpu",
        cruise_speed=0.5,
        cruise_speed_per_leg=[0.5, 0.25, 0.5],
        lookahead_dist=0.4,
        reach_threshold=0.15,
        heading_mode="takeoff_then_align",
        loop=True,
    )
    identity = mu.identity_quat(1, "cpu")
    guidance.reset(torch.tensor([0]), initial_quat=identity)

    guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]), identity, advance_waypoint=True
    )
    outbound_v, _ = guidance.compute(
        torch.tensor([[0.0, 0.0, 0.2]]), identity, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 1
    assert float(outbound_v[0, :2].norm()) == pytest.approx(0.25, abs=1e-5)

    return_v, _ = guidance.compute(
        torch.tensor([[2.0, 0.0, 0.2]]), identity, advance_waypoint=True
    )
    assert int(guidance._wp_idx[0]) == 2
    assert float(return_v[0, :2].norm()) == pytest.approx(0.5, abs=1e-5)


def test_cruise_speed_per_leg_rejects_wrong_length() -> None:
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.2], [2.0, 0.0, 0.2]]],
        dtype=torch.float32,
    )
    with pytest.raises(ValueError, match="expected one per waypoint"):
        LOSGuidance(
            waypoints,
            "cpu",
            cruise_speed_per_leg=[0.5, 0.25],
            heading_mode="takeoff_then_align",
        )


def test_cruise_speed_per_leg_rejects_non_positive() -> None:
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.2], [2.0, 0.0, 0.2]]],
        dtype=torch.float32,
    )
    with pytest.raises(ValueError, match="must be positive"):
        LOSGuidance(
            waypoints,
            "cpu",
            cruise_speed_per_leg=[0.5, 0.0, 0.5],
            heading_mode="takeoff_then_align",
        )


def test_takeoff_waypoint_advances_after_overshoot() -> None:
    """이륙 waypoint 를 지나쳐도 전환된다 (5 cm 창을 놓쳐도).

    2026-08-28 Gazebo SITL 회귀: takeoff 분기가 along-track 통과 판정을 버려
    0.20 m 이륙 구간을 0.5 m/s 로 지나친 기체가 하강 명령을 계속 받아
    해저(-9.99 m)까지 내려갔다. 40 m 미션 4 회 중 3 회가 이 창을 놓쳤다.
    """
    waypoints = torch.tensor(
        [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.20], [40.0, 0.0, 0.20]]],
        dtype=torch.float32,
    )
    guidance = LOSGuidance(
        waypoints,
        "cpu",
        cruise_speed=0.5,
        lookahead_dist=1.0,
        reach_threshold=0.30,
        heading_mode="takeoff_then_align",
        loop=False,
    )
    q0 = mu.quat_from_euler_xyz(
        torch.tensor([0.0]), torch.tensor([0.0]), torch.tensor([0.0])
    )
    guidance.reset(torch.tensor([0]), initial_quat=q0)

    # 5 cm 창을 완전히 건너뛴다 -- 0.0 m 에서 0.35 m 로 한 번에 이동.
    guidance.compute(torch.tensor([[0.0, 0.0, 0.0]]), q0)
    guidance.compute(torch.tensor([[0.0, 0.0, 0.35]]), q0)
    assert int(guidance._wp_idx[0]) == 1, (
        f"이륙 waypoint 를 0.35 m 까지 지났는데 전환되지 않았다 "
        f"(wp_idx={int(guidance._wp_idx[0])})"
    )

    # 전환 뒤 명령은 하강이 아니라 전진이어야 한다.
    v_d_b, _ = guidance.compute(torch.tensor([[0.0, 0.0, 0.35]]), q0)
    assert abs(float(v_d_b[0, 0])) > abs(float(v_d_b[0, 2])), (
        f"전환 뒤에도 수직 성분이 지배적이다: {v_d_b[0].tolist()}"
    )
