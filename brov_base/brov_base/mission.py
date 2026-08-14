"""Mission waypoint parsing and fail-closed input-bound validation."""

from __future__ import annotations

from collections.abc import Sequence

import torch

from brov_base import math_utils as mu


_AXES = ("x", "y", "z")


def parse_waypoints(specification: str) -> torch.Tensor:
    """Parse ``x,y,z;x,y,z`` into a finite ``(1, N, 3)`` float tensor."""

    if not isinstance(specification, str) or not specification.strip():
        raise ValueError("waypoints must be a non-empty 'x,y,z;...' string")

    rows: list[list[float]] = []
    for index, waypoint_text in enumerate(specification.split(";")):
        fields = [field.strip() for field in waypoint_text.split(",")]
        if len(fields) != 3 or any(not field for field in fields):
            raise ValueError(
                f"waypoint[{index}] must contain exactly three values: x,y,z"
            )
        try:
            rows.append([float(field) for field in fields])
        except ValueError as exception:
            raise ValueError(
                f"waypoint[{index}] contains a non-numeric value"
            ) from exception

    if len(rows) < 2:
        raise ValueError("at least two waypoints are required")

    waypoints = torch.tensor(rows, dtype=torch.float32).unsqueeze(0)
    if not torch.isfinite(waypoints).all():
        raise ValueError("waypoints must not contain NaN or infinity")
    return waypoints


def _xyz_tensor(name: str, values: Sequence[float]) -> torch.Tensor:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{name} must contain exactly three numeric values")
    try:
        raw_values = list(values)
    except (TypeError, ValueError) as exception:
        raise ValueError(
            f"{name} must contain exactly three numeric values"
        ) from exception
    if len(raw_values) != 3:
        raise ValueError(f"{name} must contain exactly three numeric values")
    try:
        result = torch.tensor(
            [float(value) for value in raw_values], dtype=torch.float32
        )
    except (TypeError, ValueError) as exception:
        raise ValueError(
            f"{name} must contain exactly three numeric values"
        ) from exception
    if not torch.isfinite(result).all():
        raise ValueError(f"{name} must not contain NaN or infinity")
    return result


def validate_waypoint_bounds(
    waypoints: torch.Tensor,
    *,
    enabled: bool,
    minimum_xyz: Sequence[float],
    maximum_xyz: Sequence[float],
    tolerance_m: float = 1e-6,
) -> None:
    """Reject configured waypoints outside an inclusive mission-frame box.

    Equal lower/upper values intentionally constrain an axis to a fixed value,
    which is useful for constant-depth tank demonstrations. This validates
    command input only; it is not a runtime geofence for the measured vehicle
    position.
    """

    if (
        waypoints.ndim != 3
        or waypoints.shape[0] != 1
        or waypoints.shape[2] != 3
    ):
        raise ValueError("waypoints must have shape (1, N, 3)")
    if (
        waypoints.shape[1] < 2
        or not torch.isfinite(waypoints).all()
    ):
        raise ValueError(
            "waypoints must contain at least two finite xyz points"
        )
    if not enabled:
        return
    if tolerance_m < 0.0:
        raise ValueError("waypoint bound tolerance must be non-negative")

    minimum = _xyz_tensor("waypoint_min_xyz", minimum_xyz)
    maximum = _xyz_tensor("waypoint_max_xyz", maximum_xyz)
    invalid_axes = (
        (minimum > maximum).nonzero(as_tuple=True)[0].tolist()
    )
    if invalid_axes:
        names = ", ".join(_AXES[index] for index in invalid_axes)
        raise ValueError(
            f"waypoint bounds have minimum > maximum on axis: {names}"
        )

    points = waypoints[0]
    outside = (
        (points < minimum - tolerance_m)
        | (points > maximum + tolerance_m)
    )
    if outside.any():
        point_index, axis_index = outside.nonzero(as_tuple=False)[0].tolist()
        value = float(points[point_index, axis_index])
        lower = float(minimum[axis_index])
        upper = float(maximum[axis_index])
        raise ValueError(
            f"waypoint[{point_index}].{_AXES[axis_index]}="
            f"{value:g} is outside "
            f"the allowed [{lower:g}, {upper:g}] m mission-frame range"
        )


def odom_waypoints_to_mission(
    waypoints_odom: torch.Tensor,
    position_ned: torch.Tensor,
    attitude_ned_frd: torch.Tensor,
    waypoint_frame: str,
) -> torch.Tensor:
    """Resolve immutable ROS Z-up ``odom`` points for legacy guidance.

    The local odometry contract is the fixed basis conversion
    ``p_odom = diag(1,-1,-1) p_ned``.  This function first restores absolute
    ArduSub NED points and then applies the exact origin/yaw convention used by
    :class:`brov_base.observation.ObservationBuilder` at control activation.
    It therefore changes only the outer mission representation; the deployed
    16-element policy observation remains unchanged.
    """

    points = torch.as_tensor(waypoints_odom, dtype=torch.float32)
    position = torch.as_tensor(position_ned, dtype=torch.float32)
    attitude = torch.as_tensor(attitude_ned_frd, dtype=torch.float32)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 3:
        raise ValueError("waypoints_odom must have shape (N,3) with N >= 2")
    if position.shape != (3,) or attitude.shape != (4,):
        raise ValueError("position_ned and attitude_ned_frd must have shapes (3,) and (4,)")
    if not all(torch.isfinite(value).all() for value in (points, position, attitude)):
        raise ValueError("mission transform inputs must be finite")
    if waypoint_frame not in {"ned", "start_heading"}:
        raise ValueError("waypoint_frame must be 'ned' or 'start_heading'")

    ned_absolute = points * points.new_tensor([1.0, -1.0, -1.0])
    relative_ned = ned_absolute - position.unsqueeze(0)
    if waypoint_frame == "start_heading":
        yaw = mu.yaw_from_quat(attitude)
        zero = torch.zeros_like(yaw)
        q_ned_to_mission = mu.quat_from_euler_xyz(zero, zero, -yaw)
        relative_ned = mu.quat_apply(
            q_ned_to_mission.expand(relative_ned.shape[0], -1),
            relative_ned,
        )
    return relative_ned.unsqueeze(0)


def pool_to_mission_quaternion(
    pool_to_odom_xyzw: Sequence[float],
    attitude_ned_frd: torch.Tensor,
    waypoint_frame: str,
) -> torch.Tensor:
    """Return ``^mission q_pool`` in wxyz order for attitude commands.

    ``pool_to_odom_xyzw`` is the rotation from ``odom`` coordinates into the
    surveyed Z-up ``pool`` frame.  Local ROS odometry and MAVLink NED are
    related by the proper basis rotation ``diag(1,-1,-1)``.  The optional
    start-heading rotation then removes the PREPARE-time NED yaw, exactly as
    :class:`ObservationBuilder` does for state and waypoint position.

    A pool-FLU desired body attitude can consequently be converted to the
    legacy guidance convention with::

        q_mission_body_frd = q_mission_pool * q_pool_body_flu * q_x180

    where the final ``q_x180`` changes only the body basis FLU -> FRD.
    """

    if waypoint_frame not in {"ned", "start_heading"}:
        raise ValueError("waypoint_frame must be 'ned' or 'start_heading'")
    try:
        raw_xyzw = list(pool_to_odom_xyzw)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "pool_to_odom_xyzw must contain four finite values"
        ) from error
    if len(raw_xyzw) != 4:
        raise ValueError("pool_to_odom_xyzw must contain four finite values")
    try:
        q_pool_odom = torch.tensor(
            [raw_xyzw[3], raw_xyzw[0], raw_xyzw[1], raw_xyzw[2]],
            dtype=torch.float32,
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            "pool_to_odom_xyzw must contain four finite values"
        ) from error
    attitude = torch.as_tensor(attitude_ned_frd, dtype=torch.float32)
    if attitude.shape != (4,) or not torch.isfinite(attitude).all():
        raise ValueError("attitude_ned_frd must be one finite wxyz quaternion")
    if not torch.isfinite(q_pool_odom).all():
        raise ValueError("pool_to_odom_xyzw must contain four finite values")
    pool_odom_norm = q_pool_odom.norm()
    attitude_norm = attitude.norm()
    if abs(float(pool_odom_norm) - 1.0) > 1e-3:
        raise ValueError("pool_to_odom quaternion norm is invalid")
    if abs(float(attitude_norm) - 1.0) > 1e-3:
        raise ValueError("attitude_ned_frd quaternion norm is invalid")
    q_pool_odom = q_pool_odom / pool_odom_norm
    attitude = attitude / attitude_norm

    # O <- N and N <- O are both the 180-degree X rotation.
    q_ned_odom = attitude.new_tensor([0.0, 1.0, 0.0, 0.0])
    q_odom_pool = mu.quat_conjugate(q_pool_odom)
    if waypoint_frame == "start_heading":
        yaw = mu.yaw_from_quat(attitude)
        zero = torch.zeros_like(yaw)
        q_mission_ned = mu.quat_from_euler_xyz(zero, zero, -yaw)
    else:
        q_mission_ned = attitude.new_tensor([1.0, 0.0, 0.0, 0.0])
    result = mu.quat_mul(
        mu.quat_mul(q_mission_ned, q_ned_odom), q_odom_pool
    )
    result = result / result.norm().clamp_min(1e-12)
    return mu.quat_unique(result)
