import math

import numpy as np
import pytest

from brov_perception.geometry import (
    matrix_from_xyz_quaternion,
    matrix_from_xyz_rpy,
    quaternion_from_matrix,
)


def test_identity_rotation_is_identity_quaternion() -> None:
    np.testing.assert_allclose(
        quaternion_from_matrix(np.eye(3)), [0.0, 0.0, 0.0, 1.0], atol=1e-12
    )


def test_yaw_rotation_and_translation() -> None:
    transform = matrix_from_xyz_rpy([1.0, 2.0, 3.0], [0.0, 0.0, math.pi / 2.0])
    np.testing.assert_allclose(transform[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(
        transform[:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-12,
    )
    quaternion = quaternion_from_matrix(transform[:3, :3])
    np.testing.assert_allclose(
        np.abs(quaternion),
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
        atol=1e-12,
    )


def test_transform_rejects_bad_vector_lengths() -> None:
    with pytest.raises(ValueError):
        matrix_from_xyz_rpy([0.0, 0.0], [0.0, 0.0, 0.0])


def test_quaternion_rejects_bad_matrix_shape() -> None:
    with pytest.raises(ValueError):
        quaternion_from_matrix(np.eye(4))


def test_xyz_quaternion_builds_surveyed_pool_marker_transform() -> None:
    transform = matrix_from_xyz_quaternion(
        [3.8, 0.85, 0.24], [-0.5, -0.5, 0.5, 0.5]
    )
    np.testing.assert_allclose(transform[:3, 3], [3.8, 0.85, 0.24])
    np.testing.assert_allclose(
        transform[:3, :3],
        [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "quaternion",
    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 2.0], [0.0, 0.0, 0.0, float("nan")]),
)
def test_xyz_quaternion_rejects_invalid_quaternion(quaternion) -> None:
    with pytest.raises(ValueError):
        matrix_from_xyz_quaternion([0.0, 0.0, 0.0], quaternion)
