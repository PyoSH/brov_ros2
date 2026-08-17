"""Pure NED/FRD to ROS odometry-frame conversion helpers.

The MAVLink navigation state uses a NED world frame and an FRD body frame.
ROS odometry uses a Z-up world frame and an FLU body frame.  Both basis
changes are the same proper rotation::

    S = diag(1, -1, -1)

For a position ``p_N`` and body-to-world rotation ``R_ND`` the converted pose
is therefore ``p_O = S p_N`` and ``R_OB = S R_ND S``.  ``nav_msgs/Odometry``
twist is expressed in its child/body frame, so the linear velocity is first
rotated from NED world into FRD body and then changed to FLU.

This module intentionally has no ROS or pymavlink dependency.  It can be
unit-tested and reused by a ROS message adapter without coupling geometry to
transport code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import torch

from brov_base import math_utils as mu


_S_DIAGONAL = (1.0, -1.0, -1.0)


@dataclass(frozen=True)
class OdomKinematics:
    """One converted pose and body twist, ready for ROS message fields.

    ``orientation_xyzw`` follows the ROS quaternion order.  The other vectors
    use XYZ component order.  The two twist vectors are expressed in the FLU
    body frame, as required for an odometry message whose child is
    ``base_link``.
    """

    position_odom: torch.Tensor
    orientation_xyzw: torch.Tensor
    linear_velocity_body_flu: torch.Tensor
    angular_velocity_body_flu: torch.Tensor


def _floating_tensor(value, *, like: torch.Tensor | None = None) -> torch.Tensor:
    if like is None:
        result = torch.as_tensor(value)
        if not result.is_floating_point():
            result = result.to(dtype=torch.get_default_dtype())
        return result
    return torch.as_tensor(value, dtype=like.dtype, device=like.device)


def _require_vector(value: torch.Tensor, size: int, name: str) -> None:
    if value.ndim == 0 or value.shape[-1] != size:
        raise ValueError(f"{name} must have shape (..., {size})")
    if not torch.isfinite(value).all():
        raise ValueError(f"{name} must contain only finite values")


def ned_frd_to_odom_flu(
    position_ned: torch.Tensor | Sequence[float],
    attitude_quat_ned_frd_wxyz: torch.Tensor | Sequence[float],
    linear_velocity_ned: torch.Tensor | Sequence[float],
    angular_velocity_frd: torch.Tensor | Sequence[float],
) -> OdomKinematics:
    """Convert MAVLink NED/FRD pose and velocity to odom Z-up/FLU.

    Parameters accept either one state or matching leading batch dimensions.
    ``attitude_quat_ned_frd_wxyz`` is the body-FRD to world-NED quaternion in
    MAVLink/Isaac ``[w, x, y, z]`` order.  The returned quaternion is in ROS
    ``[x, y, z, w]`` order.
    """

    quaternion = _floating_tensor(attitude_quat_ned_frd_wxyz)
    position = _floating_tensor(position_ned, like=quaternion)
    linear_velocity = _floating_tensor(linear_velocity_ned, like=quaternion)
    angular_velocity = _floating_tensor(angular_velocity_frd, like=quaternion)

    _require_vector(position, 3, "position_ned")
    _require_vector(quaternion, 4, "attitude_quat_ned_frd_wxyz")
    _require_vector(linear_velocity, 3, "linear_velocity_ned")
    _require_vector(angular_velocity, 3, "angular_velocity_frd")
    leading_shape = quaternion.shape[:-1]
    if any(
        value.shape[:-1] != leading_shape
        for value in (position, linear_velocity, angular_velocity)
    ):
        raise ValueError("pose and twist inputs must have matching leading dimensions")

    norm = torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True)
    if (norm <= torch.finfo(quaternion.dtype).eps).any():
        raise ValueError("attitude quaternion norm must be positive")
    quaternion = quaternion / norm

    signs = quaternion.new_tensor(_S_DIAGONAL)
    position_odom = position * signs

    # Quaternion conjugation by the pi rotation about X implements
    # R_OB = S R_ND S.  In wxyz components this is [w, x, -y, -z].
    orientation_wxyz = quaternion * quaternion.new_tensor((1.0, 1.0, -1.0, -1.0))
    orientation_xyzw = orientation_wxyz[..., (1, 2, 3, 0)]

    linear_velocity_body_frd = mu.quat_apply(
        mu.quat_conjugate(quaternion), linear_velocity
    )
    linear_velocity_body_flu = linear_velocity_body_frd * signs
    angular_velocity_body_flu = angular_velocity * signs

    return OdomKinematics(
        position_odom=position_odom,
        orientation_xyzw=orientation_xyzw,
        linear_velocity_body_flu=linear_velocity_body_flu,
        angular_velocity_body_flu=angular_velocity_body_flu,
    )
