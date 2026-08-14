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


@pytest.mark.parametrize("loop,count", [(False, 3), (True, 2), (True, 4)])
def test_takeoff_mode_requires_one_prefix_plus_two_loop_points(
    loop: bool, count: int
) -> None:
    with pytest.raises(
        ValueError, match="requires loop=true and exactly three waypoints"
    ):
        LOSGuidance(
            torch.zeros((1, count, 3)),
            "cpu",
            heading_mode="takeoff_then_align",
            loop=loop,
        )
