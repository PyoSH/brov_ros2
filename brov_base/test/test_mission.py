import pytest
import torch

from brov_base.mission import parse_waypoints, validate_waypoint_bounds


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
