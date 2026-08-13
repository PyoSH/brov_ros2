"""Small, ROS-independent rigid-transform helpers used by ArUco pose output."""

from __future__ import annotations

import math

import numpy as np


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to ROS quaternion ``[x, y, z, w]``."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0 or not np.isfinite(norm):
        raise ValueError("rotation produced an invalid quaternion")
    return quaternion / norm


def matrix_from_xyz_rpy(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Create ``T_parent_child`` using XYZ metres and fixed-axis RPY radians."""

    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("xyz and rpy must each contain three values")
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def matrix_from_xyz_quaternion(
    xyz: list[float], quaternion_xyzw: list[float]
) -> np.ndarray:
    """Create ``T_parent_child`` from XYZ metres and ROS ``[x,y,z,w]``."""

    if len(xyz) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("xyz and quaternion_xyzw must contain 3 and 4 values")
    translation = np.asarray(xyz, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if not np.all(np.isfinite(translation)) or not np.all(
        np.isfinite(quaternion)
    ):
        raise ValueError("xyz and quaternion_xyzw must be finite")
    norm = float(np.linalg.norm(quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("quaternion_xyzw must have unit norm")

    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform
