"""Pure geometry helpers shared by the pool visualization and its tests."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def finite_vector(values: Iterable[float], length: int, name: str) -> tuple:
    """Return a finite float tuple of the requested length."""
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def normalized_quaternion(values: Iterable[float]) -> Quaternion:
    """Validate and normalize an xyzw quaternion."""
    x, y, z, w = finite_vector(values, 4, "quaternion_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        raise ValueError("quaternion_xyzw norm must be positive")
    return x / norm, y / norm, z / norm, w / norm


def quaternion_rotation_matrix(values: Iterable[float]) -> tuple[Vector3, ...]:
    """Return the active rotation matrix for an xyzw quaternion."""
    x, y, z, w = normalized_quaternion(values)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_vector(
    quaternion_xyzw: Iterable[float], vector: Iterable[float]
) -> Vector3:
    """Rotate a vector using an xyzw quaternion."""
    matrix = quaternion_rotation_matrix(quaternion_xyzw)
    vx, vy, vz = finite_vector(vector, 3, "vector")
    return tuple(
        row[0] * vx + row[1] * vy + row[2] * vz for row in matrix
    )


def add_vectors(left: Sequence[float], right: Sequence[float]) -> Vector3:
    """Add two 3-D vectors."""
    lhs = finite_vector(left, 3, "left")
    rhs = finite_vector(right, 3, "right")
    return tuple(a + b for a, b in zip(lhs, rhs))


def pool_corners(size_xyz: Iterable[float]) -> tuple[Vector3, ...]:
    """Return the eight corners of a positive-octant rectangular pool."""
    sx, sy, sz = finite_vector(size_xyz, 3, "pool_size_xyz")
    if min(sx, sy, sz) <= 0.0:
        raise ValueError("pool_size_xyz entries must be positive")
    return (
        (0.0, 0.0, 0.0),
        (sx, 0.0, 0.0),
        (sx, sy, 0.0),
        (0.0, sy, 0.0),
        (0.0, 0.0, sz),
        (sx, 0.0, sz),
        (sx, sy, sz),
        (0.0, sy, sz),
    )


def pool_edge_segments(
    size_xyz: Iterable[float],
) -> tuple[tuple[Vector3, Vector3], ...]:
    """Return the twelve line segments forming the pool boundary."""
    corners = pool_corners(size_xyz)
    edge_indices = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    return tuple((corners[start], corners[end]) for start, end in edge_indices)


def inside_pool(position: Iterable[float], size_xyz: Iterable[float]) -> bool:
    """Return whether a point lies inside the nominal pool bounds."""
    point = finite_vector(position, 3, "position")
    size = finite_vector(size_xyz, 3, "pool_size_xyz")
    return all(0.0 <= value <= limit for value, limit in zip(point, size))
