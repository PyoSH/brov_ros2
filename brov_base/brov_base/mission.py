"""Mission waypoint parsing and fail-closed input-bound validation."""

from __future__ import annotations

from collections.abc import Sequence

import torch


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
