import math

import pytest
import torch

from brov_base import math_utils as mu
from brov_base.mission import (
    odom_waypoints_to_mission,
    parse_waypoints,
    pool_to_mission_quaternion,
    validate_waypoint_bounds,
)


def test_parse_waypoints_returns_expected_shape_and_values():
    result = parse_waypoints("0, 0, 0; 3.0,0,0")

    assert result.shape == (1, 2, 3)
    assert result.dtype == torch.float32
    assert torch.allclose(result[0, 1], torch.tensor([3.0, 0.0, 0.0]))


@pytest.mark.parametrize(
    "specification",
    ("", "0,0,0", "0,0;1,0,0", "0,0,0;nan,0,0", "0,0,0;bad,0,0"),
)
def test_parse_waypoints_rejects_invalid_input(specification):
    with pytest.raises(ValueError):
        parse_waypoints(specification)


def test_fixed_axis_bounds_accept_tank_line():
    waypoints = parse_waypoints("0,0,0;3,0,0")

    validate_waypoint_bounds(
        waypoints,
        enabled=True,
        minimum_xyz=[0.0, 0.0, 0.0],
        maximum_xyz=[3.0, 0.0, 0.0],
    )


def test_bounds_reject_first_outside_axis_with_actionable_error():
    waypoints = parse_waypoints("0,0,0;0.6,0.7,0")

    with pytest.raises(ValueError, match=r"waypoint\[1\]\.y=0.7.*\[0, 0.6\]"):
        validate_waypoint_bounds(
            waypoints,
            enabled=True,
            minimum_xyz=[0.0, 0.0, 0.0],
            maximum_xyz=[0.6, 0.6, 0.0],
        )


def test_bounds_reject_inverted_axis():
    with pytest.raises(ValueError, match="minimum > maximum on axis: x"):
        validate_waypoint_bounds(
            parse_waypoints("0,0,0;1,0,0"),
            enabled=True,
            minimum_xyz=[1.0, 0.0, 0.0],
            maximum_xyz=[0.0, 0.0, 0.0],
        )


def test_disabled_bounds_preserve_general_missions():
    validate_waypoint_bounds(
        parse_waypoints("-100,20,3;100,-20,-3"),
        enabled=False,
        minimum_xyz=[0.0, 0.0, 0.0],
        maximum_xyz=[0.0, 0.0, 0.0],
    )


def test_odom_waypoints_restore_absolute_ned_then_subtract_start():
    points_odom = torch.tensor([[1.0, -2.0, -3.0], [2.0, -4.0, -5.0]])
    position_ned = torch.tensor([1.0, 2.0, 3.0])
    attitude = torch.tensor([1.0, 0.0, 0.0, 0.0])

    result = odom_waypoints_to_mission(
        points_odom, position_ned, attitude, "ned"
    )

    assert result.shape == (1, 2, 3)
    assert torch.allclose(
        result[0], torch.tensor([[0.0, 0.0, 0.0], [1.0, 2.0, 2.0]])
    )


def test_odom_waypoints_apply_start_heading_yaw_after_basis_conversion():
    half_pi = torch.tensor(torch.pi / 2.0)
    attitude = mu.quat_from_euler_xyz(
        torch.tensor(0.0), torch.tensor(0.0), half_pi
    )
    # odom +Y corresponds to NED -Y. With a +90 degree initial NED yaw,
    # that NED -Y direction becomes mission -X.
    points_odom = torch.tensor([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    result = odom_waypoints_to_mission(
        points_odom, torch.zeros(3), attitude, "start_heading"
    )

    assert torch.allclose(
        result[0, 1], torch.tensor([-1.0, 0.0, 0.0]), atol=1e-6
    )


@pytest.mark.parametrize("frame", ["pool", "map", ""])
def test_odom_waypoint_transform_rejects_unknown_legacy_frame(frame):
    with pytest.raises(ValueError, match="waypoint_frame"):
        odom_waypoints_to_mission(
            torch.zeros(2, 3), torch.zeros(3),
            torch.tensor([1.0, 0.0, 0.0, 0.0]), frame
        )


def test_pool_attitude_transform_matches_identity_physical_pose() -> None:
    # pool == odom.  A level North-facing FLU body is identity in pool, while
    # the corresponding NED/FRD desired attitude is identity in mission.
    q_mission_pool = pool_to_mission_quaternion(
        [0.0, 0.0, 0.0, 1.0],
        torch.tensor([1.0, 0.0, 0.0, 0.0]),
        "start_heading",
    )
    q_body_basis = torch.tensor([0.0, 1.0, 0.0, 0.0])
    q_internal = mu.quat_unique(
        mu.quat_mul(
            mu.quat_mul(
                q_mission_pool, torch.tensor([1.0, 0.0, 0.0, 0.0])
            ),
            q_body_basis,
        )
    )
    assert torch.allclose(
        q_internal, torch.tensor([1.0, 0.0, 0.0, 0.0]), atol=1e-6
    )


def test_pool_north_target_becomes_minus_ninety_in_east_start_heading() -> None:
    attitude_east_ned = mu.quat_from_euler_xyz(
        torch.tensor(0.0), torch.tensor(0.0), torch.tensor(math.pi / 2.0)
    )
    q_mp = pool_to_mission_quaternion(
        [0.0, 0.0, 0.0, 1.0], attitude_east_ned, "start_heading"
    )
    q_internal = mu.quat_unique(
        mu.quat_mul(
            mu.quat_mul(q_mp, torch.tensor([1.0, 0.0, 0.0, 0.0])),
            torch.tensor([0.0, 1.0, 0.0, 0.0]),
        )
    )
    expected = mu.quat_unique(
        mu.quat_from_euler_xyz(
            torch.tensor(0.0),
            torch.tensor(0.0),
            torch.tensor(-math.pi / 2.0),
        )
    )
    assert abs(float(torch.dot(q_internal, expected))) == pytest.approx(
        1.0, abs=1e-6
    )


def test_pool_attitude_transform_round_trip_under_full_rotation() -> None:
    q_pool_odom = mu.quat_from_euler_xyz(
        torch.tensor(0.2), torch.tensor(-0.1), torch.tensor(0.4)
    )
    attitude = mu.quat_from_euler_xyz(
        torch.tensor(-0.3), torch.tensor(0.15), torch.tensor(0.8)
    )
    q_pool_target = mu.quat_from_euler_xyz(
        torch.tensor(0.4), torch.tensor(-0.5), torch.tensor(1.1)
    )
    q_mp = pool_to_mission_quaternion(
        [q_pool_odom[1], q_pool_odom[2], q_pool_odom[3], q_pool_odom[0]],
        attitude,
        "start_heading",
    )
    q_x = torch.tensor([0.0, 1.0, 0.0, 0.0])
    q_internal = mu.quat_mul(mu.quat_mul(q_mp, q_pool_target), q_x)
    recovered = mu.quat_mul(
        mu.quat_mul(mu.quat_conjugate(q_mp), q_internal),
        mu.quat_conjugate(q_x),
    )
    recovered = recovered / recovered.norm()
    assert abs(float(torch.dot(recovered, q_pool_target))) == pytest.approx(
        1.0, abs=1e-6
    )
