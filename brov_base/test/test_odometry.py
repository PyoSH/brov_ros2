"""NED/FRD to odom Z-up/FLU conversion regression tests."""

import pytest
import torch

from brov_base.odometry import ned_frd_to_odom_flu


def _rotation_matrix_wxyz(quaternion: torch.Tensor) -> torch.Tensor:
    quaternion = quaternion / quaternion.norm()
    w, x, y, z = quaternion
    return torch.tensor(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=quaternion.dtype,
    )


def test_identity_pose_changes_ned_frd_axes_to_zup_flu():
    converted = ned_frd_to_odom_flu(
        [1.0, 2.0, 3.0],
        [1.0, 0.0, 0.0, 0.0],
        [4.0, 5.0, 6.0],
        [0.1, 0.2, 0.3],
    )

    assert torch.allclose(converted.position_odom, torch.tensor([1.0, -2.0, -3.0]))
    assert torch.allclose(converted.orientation_xyzw, torch.tensor([0.0, 0.0, 0.0, 1.0]))
    assert torch.allclose(
        converted.linear_velocity_body_flu, torch.tensor([4.0, -5.0, -6.0])
    )
    assert torch.allclose(
        converted.angular_velocity_body_flu, torch.tensor([0.1, -0.2, -0.3])
    )


def test_arbitrary_pose_matches_matrix_contract_and_body_twist():
    position_ned = torch.tensor([0.7, -1.2, 2.4], dtype=torch.float64)
    quaternion_ned_frd = torch.tensor([0.7, 0.2, -0.3, 0.6], dtype=torch.float64)
    quaternion_ned_frd = quaternion_ned_frd / quaternion_ned_frd.norm()
    velocity_ned = torch.tensor([0.4, -0.5, 0.8], dtype=torch.float64)
    angular_velocity_frd = torch.tensor([0.11, -0.22, 0.33], dtype=torch.float64)
    signs = torch.diag(torch.tensor([1.0, -1.0, -1.0], dtype=torch.float64))

    converted = ned_frd_to_odom_flu(
        position_ned,
        quaternion_ned_frd,
        velocity_ned,
        angular_velocity_frd,
    )

    orientation_wxyz = converted.orientation_xyzw[(3, 0, 1, 2),]
    rotation_ned_frd = _rotation_matrix_wxyz(quaternion_ned_frd)
    rotation_odom_flu = _rotation_matrix_wxyz(orientation_wxyz)
    assert torch.allclose(converted.position_odom, signs @ position_ned, atol=1e-12)
    assert torch.allclose(
        rotation_odom_flu,
        signs @ rotation_ned_frd @ signs,
        atol=1e-12,
    )
    assert torch.allclose(
        converted.linear_velocity_body_flu,
        signs @ rotation_ned_frd.T @ velocity_ned,
        atol=1e-12,
    )
    assert torch.allclose(
        converted.angular_velocity_body_flu,
        signs @ angular_velocity_frd,
        atol=1e-12,
    )


def test_conversion_supports_matching_batches():
    converted = ned_frd_to_odom_flu(
        torch.tensor([[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]),
        torch.zeros((2, 3)),
        torch.zeros((2, 3)),
    )

    assert converted.position_odom.shape == (2, 3)
    assert torch.allclose(
        converted.position_odom,
        torch.tensor([[1.0, -2.0, -3.0], [-1.0, 2.0, 3.0]]),
    )
    assert converted.orientation_xyzw.shape == (2, 4)


@pytest.mark.parametrize(
    "position, quaternion, velocity, rates",
    [
        ([0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3),
        ([0.0] * 3, [0.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3),
        ([0.0] * 3, [1.0, 0.0, 0.0, 0.0], [float("nan"), 0.0, 0.0], [0.0] * 3),
        (
            [[0.0, 0.0, 0.0]],
            [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            [[0.0, 0.0, 0.0]],
            [[0.0, 0.0, 0.0]],
        ),
    ],
)
def test_conversion_rejects_invalid_geometry(position, quaternion, velocity, rates):
    with pytest.raises(ValueError):
        ned_frd_to_odom_flu(position, quaternion, velocity, rates)
