"""
Small, ROS-independent SE(3) helpers.

The public quaternion convention in this module is ``[x, y, z, w]`` to match
``geometry_msgs``.  A transform named ``transform_a_b`` maps coordinates from
frame B into frame A.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


_EPS = 1.0e-12


def normalize_quaternion_xyzw(quaternion: Iterable[float]) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion contains non-finite values")
    norm = float(np.linalg.norm(q))
    if norm <= _EPS:
        raise ValueError("quaternion norm is zero")
    return q / norm


def quaternion_xyzw_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("rotation contains non-finite values")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-6):
        raise ValueError("rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1.0e-6):
        raise ValueError("rotation determinant is not +1")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    q = normalize_quaternion_xyzw([x, y, z, w])
    return -q if q[3] < 0.0 else q


def make_transform(
    translation: Iterable[float], quaternion_xyzw: Iterable[float]
) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    position = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(position)):
        raise ValueError("translation contains non-finite values")
    transform[:3, :3] = quaternion_xyzw_to_matrix(quaternion_xyzw)
    transform[:3, 3] = position
    return transform


def validate_transform(transform: np.ndarray) -> np.ndarray:
    value = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    if not np.all(np.isfinite(value)):
        raise ValueError("transform contains non-finite values")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-9):
        raise ValueError("transform has an invalid homogeneous row")
    # This also validates the SO(3) block.
    matrix_to_quaternion_xyzw(value[:3, :3])
    return value


def invert_transform(transform_a_b: np.ndarray) -> np.ndarray:
    value = validate_transform(transform_a_b)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = value[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ value[:3, 3]
    return inverse


def quaternion_angular_distance_rad(
    quaternion_a: Iterable[float], quaternion_b: Iterable[float]
) -> float:
    a = normalize_quaternion_xyzw(quaternion_a)
    b = normalize_quaternion_xyzw(quaternion_b)
    dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def markley_quaternion_mean_xyzw(quaternions: Iterable[Iterable[float]]) -> np.ndarray:
    values = np.asarray(list(quaternions), dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4 or len(values) == 0:
        raise ValueError("at least one quaternion is required")
    normalized = np.vstack([normalize_quaternion_xyzw(value) for value in values])
    reference = normalized[0]
    normalized = np.vstack(
        [value if np.dot(value, reference) >= 0.0 else -value for value in normalized]
    )
    accumulator = normalized.T @ normalized
    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    result = normalize_quaternion_xyzw(eigenvectors[:, int(np.argmax(eigenvalues))])
    if np.dot(result, reference) < 0.0:
        result = -result
    return result


def _quaternion_medoid(quaternions: np.ndarray) -> np.ndarray:
    normalized = np.vstack([normalize_quaternion_xyzw(value) for value in quaternions])
    dots = np.clip(np.abs(normalized @ normalized.T), 0.0, 1.0)
    costs = np.sum(2.0 * np.arccos(dots), axis=1)
    return normalized[int(np.argmin(costs))]


def rotation_rpy_rad(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    matrix_to_quaternion_xyzw(matrix)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def rotate_pose_covariance(covariance: Iterable[float], rotation_a_b: np.ndarray) -> np.ndarray:
    """
    Rotate a ROS pose covariance from frame B into frame A.

    The 6-vector ordering is ``x,y,z,rot_x,rot_y,rot_z`` and both translation
    and small-angle errors are assumed to be expressed in the pose header frame.
    """
    covariance_matrix = np.asarray(covariance, dtype=np.float64).reshape(6, 6)
    rotation = np.asarray(rotation_a_b, dtype=np.float64).reshape(3, 3)
    matrix_to_quaternion_xyzw(rotation)
    if not np.all(np.isfinite(covariance_matrix)):
        raise ValueError("pose covariance contains non-finite values")
    jacobian = np.zeros((6, 6), dtype=np.float64)
    jacobian[:3, :3] = rotation
    jacobian[3:, 3:] = rotation
    result = jacobian @ covariance_matrix @ jacobian.T
    return 0.5 * (result + result.T)


@dataclass(frozen=True)
class RobustTransformEstimate:
    transform: np.ndarray
    inlier_mask: np.ndarray
    translation_residual_m: np.ndarray
    rotation_residual_rad: np.ndarray


def _residuals(
    translations: np.ndarray,
    quaternions: np.ndarray,
    center_translation: np.ndarray,
    center_quaternion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    translation_residual = np.linalg.norm(translations - center_translation, axis=1)
    rotation_residual = np.array(
        [
            quaternion_angular_distance_rad(value, center_quaternion)
            for value in quaternions
        ],
        dtype=np.float64,
    )
    return translation_residual, rotation_residual


def robust_average_transforms(
    transforms: Iterable[np.ndarray],
    *,
    max_translation_residual_m: float,
    max_rotation_residual_rad: float,
    min_inliers: int,
) -> RobustTransformEstimate:
    """
    Estimate one rigid transform while rejecting gross sample outliers.

    A component-wise translation median and quaternion medoid seed the gate.
    The accepted rotations are then averaged with the normalized Markley
    eigenvector method.  One final re-gate/re-fit prevents the preliminary
    center from biasing the reported result.
    """
    values = [validate_transform(value).copy() for value in transforms]
    if min_inliers <= 0:
        raise ValueError("min_inliers must be positive")
    if len(values) < min_inliers:
        raise ValueError(f"need {min_inliers} transforms, got {len(values)}")
    if max_translation_residual_m <= 0.0 or max_rotation_residual_rad <= 0.0:
        raise ValueError("residual thresholds must be positive")

    translations = np.vstack([value[:3, 3] for value in values])
    quaternions = np.vstack(
        [matrix_to_quaternion_xyzw(value[:3, :3]) for value in values]
    )
    center_translation = np.median(translations, axis=0)
    center_quaternion = _quaternion_medoid(quaternions)

    for _ in range(2):
        translation_residual, rotation_residual = _residuals(
            translations, quaternions, center_translation, center_quaternion
        )
        inliers = (translation_residual <= max_translation_residual_m) & (
            rotation_residual <= max_rotation_residual_rad
        )
        if int(np.count_nonzero(inliers)) < min_inliers:
            raise ValueError(
                "residual gate left "
                f"{int(np.count_nonzero(inliers))}/{min_inliers} required inliers"
            )
        center_translation = np.median(translations[inliers], axis=0)
        center_quaternion = markley_quaternion_mean_xyzw(quaternions[inliers])

    translation_residual, rotation_residual = _residuals(
        translations, quaternions, center_translation, center_quaternion
    )
    inliers = (translation_residual <= max_translation_residual_m) & (
        rotation_residual <= max_rotation_residual_rad
    )
    if int(np.count_nonzero(inliers)) < min_inliers:
        raise ValueError(
            "final residual gate left "
            f"{int(np.count_nonzero(inliers))}/{min_inliers} required inliers"
        )
    center_translation = np.median(translations[inliers], axis=0)
    center_quaternion = markley_quaternion_mean_xyzw(quaternions[inliers])
    transform = make_transform(center_translation, center_quaternion)
    translation_residual, rotation_residual = _residuals(
        translations, quaternions, center_translation, center_quaternion
    )
    inliers = (translation_residual <= max_translation_residual_m) & (
        rotation_residual <= max_rotation_residual_rad
    )
    if int(np.count_nonzero(inliers)) < min_inliers:
        raise ValueError(
            "reported estimate left "
            f"{int(np.count_nonzero(inliers))}/{min_inliers} required inliers"
        )
    return RobustTransformEstimate(
        transform=transform,
        inlier_mask=inliers,
        translation_residual_m=translation_residual,
        rotation_residual_rad=rotation_residual,
    )
