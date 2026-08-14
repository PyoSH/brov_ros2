import math

import numpy as np
import pytest

from brov_localization.math3d import (
    invert_transform,
    make_transform,
    markley_quaternion_mean_xyzw,
    matrix_to_quaternion_xyzw,
    quaternion_angular_distance_rad,
    quaternion_xyzw_to_matrix,
    robust_average_transforms,
    rotate_pose_covariance,
    rotation_rpy_rad,
)


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


@pytest.mark.parametrize(
    "rpy",
    [
        (0.0, 0.0, 0.0),
        (math.pi, 0.0, 0.0),
        (0.2, -0.3, 1.1),
        (-1.2, 0.7, -2.8),
    ],
)
def test_quaternion_matrix_round_trip(rpy) -> None:
    quaternion = _quaternion_from_rpy(*rpy)
    rotation = quaternion_xyzw_to_matrix(quaternion)
    recovered = matrix_to_quaternion_xyzw(rotation)
    assert quaternion_angular_distance_rad(quaternion, recovered) < 1.0e-7


def test_transform_inverse_is_exact_for_full_se3() -> None:
    transform = make_transform(
        [1.2, -0.4, 2.7], _quaternion_from_rpy(0.3, -0.2, 1.0)
    )
    assert transform @ invert_transform(transform) == pytest.approx(np.eye(4))
    assert invert_transform(transform) @ transform == pytest.approx(np.eye(4))


def test_markley_mean_is_invariant_to_quaternion_sign() -> None:
    expected = _quaternion_from_rpy(0.1, -0.05, 0.7)
    mean = markley_quaternion_mean_xyzw([expected, -expected, expected, -expected])
    assert quaternion_angular_distance_rad(mean, expected) < 1.0e-7


def test_robust_transform_average_rejects_translation_and_rotation_outliers() -> None:
    expected = make_transform(
        [1.0, 2.0, 0.4], _quaternion_from_rpy(0.04, -0.03, 0.5)
    )
    samples = []
    offsets = [
        (-0.010, 0.002, 0.001, -0.010),
        (-0.004, -0.003, 0.000, -0.005),
        (0.000, 0.000, -0.002, 0.000),
        (0.003, 0.004, 0.001, 0.004),
        (0.011, -0.002, 0.000, 0.009),
    ]
    for dx, dy, dz, dyaw in offsets:
        perturbation = make_transform(
            [dx, dy, dz], _quaternion_from_rpy(0.0, 0.0, dyaw)
        )
        samples.append(perturbation @ expected)
    samples.append(
        make_transform([8.0, -5.0, 3.0], _quaternion_from_rpy(1.5, 0.4, -2.0))
    )

    estimate = robust_average_transforms(
        samples,
        max_translation_residual_m=0.08,
        max_rotation_residual_rad=math.radians(5.0),
        min_inliers=5,
    )
    assert np.count_nonzero(estimate.inlier_mask) == 5
    assert estimate.transform[:3, 3] == pytest.approx(expected[:3, 3], abs=0.012)
    estimated_q = matrix_to_quaternion_xyzw(estimate.transform[:3, :3])
    expected_q = matrix_to_quaternion_xyzw(expected[:3, :3])
    assert quaternion_angular_distance_rad(estimated_q, expected_q) < math.radians(1.0)


def test_robust_transform_average_fails_when_gate_has_too_few_inliers() -> None:
    samples = [
        make_transform([float(index), 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        for index in range(4)
    ]
    with pytest.raises(ValueError, match="inliers"):
        robust_average_transforms(
            samples,
            max_translation_residual_m=0.1,
            max_rotation_residual_rad=0.1,
            min_inliers=3,
        )


def test_pose_covariance_rotates_both_translation_and_small_angles() -> None:
    rotation = quaternion_xyzw_to_matrix(_quaternion_from_rpy(0.0, 0.0, math.pi / 2))
    covariance = np.diag([1.0, 4.0, 9.0, 16.0, 25.0, 36.0])
    rotated = rotate_pose_covariance(covariance, rotation)
    assert np.diag(rotated) == pytest.approx([4.0, 1.0, 9.0, 25.0, 16.0, 36.0])


def test_rpy_recovers_nonzero_roll_and_pitch() -> None:
    expected = (0.21, -0.17, 0.92)
    rotation = quaternion_xyzw_to_matrix(_quaternion_from_rpy(*expected))
    assert rotation_rpy_rad(rotation) == pytest.approx(expected)
