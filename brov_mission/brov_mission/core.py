"""ROS-independent validation and transform helpers for pool missions."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import json
import math


Point3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

POOL_POSITION_MISSION_V1 = "brov_pool_position_mission_v1"
POOL_POSITION_MISSION_V2 = "brov_pool_position_mission_v2"
RANDOM_ATTITUDE_REFERENCE_FRAME = "pool_zup_flu"
RANDOM_ATTITUDE_GENERATOR_VERSION = "sha256_counter_uniform_rpy_v1"

CONTRACT_HEADING_MODES = {
    POOL_POSITION_MISSION_V1: frozenset(
        {"straight", "align", "takeoff_then_align"}
    ),
    POOL_POSITION_MISSION_V2: frozenset({"random_at_waypoint"}),
}


@dataclass(frozen=True)
class RandomAttitudeSettings:
    """Hashed, deterministic random-attitude contract for mission v2."""

    seed: int
    reference_frame: str
    generator_version: str
    rpy_min_rad: Point3
    rpy_max_rad: Point3
    max_slew_rate_rad_s: float
    attitude_tolerance_rad: float
    angular_speed_tolerance_rad_s: float
    dwell_time_s: float
    max_duration_s: float
    max_laps: int


@dataclass(frozen=True)
class MissionSettings:
    cruise_speed: float
    lookahead_dist: float
    reach_threshold: float
    heading_mode: str
    loop: bool
    random_attitude: RandomAttitudeSettings | None = None
    # Optional per-waypoint-index speed override (v1 contract only -- one
    # value per waypoint, indexed by the leg *departing* that index toward
    # its next waypoint). None/empty means every leg uses the scalar
    # cruise_speed above, unchanged from before this field existed.
    cruise_speed_per_leg: tuple[float, ...] | None = None


@dataclass(frozen=True)
class ValidationSettings:
    safe_min_xyz: Point3
    safe_max_xyz: Point3
    max_first_point_distance_m: float
    min_segment_length_m: float
    identity_orientation_tolerance: float
    allowed_heading_modes: tuple[str, ...]


def _finite_float(value, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def xyz(values: Sequence[float], name: str) -> Point3:
    if isinstance(values, (str, bytes)) or len(values) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    return tuple(
        _finite_float(value, f"{name}[{index}]")
        for index, value in enumerate(values)
    )


def quaternion(values: Sequence[float], name: str) -> Quaternion:
    if isinstance(values, (str, bytes)) or len(values) != 4:
        raise ValueError(f"{name} must contain exactly four values")
    return tuple(
        _finite_float(value, f"{name}[{index}]")
        for index, value in enumerate(values)
    )


def validate_identity_quaternion(
    value: Quaternion, tolerance: float
) -> None:
    """Require a valid quaternion representing the identity rotation.

    Both q and -q are accepted because they encode the same rotation.
    """

    tolerance = _finite_float(tolerance, "identity orientation tolerance")
    if tolerance <= 0.0:
        raise ValueError("identity orientation tolerance must be positive")
    norm = math.sqrt(sum(component * component for component in value))
    if abs(norm - 1.0) > tolerance:
        raise ValueError(f"invalid pose quaternion norm {norm:.6g}")
    x, y, z, w = (component / norm for component in value)
    if (
        abs(x) > tolerance
        or abs(y) > tolerance
        or abs(z) > tolerance
        or abs(abs(w) - 1.0) > tolerance
    ):
        raise ValueError(
            "non-identity waypoint orientation is unsupported; "
            "this mission contract is position-only"
        )


def validate_mission_settings(
    settings: MissionSettings,
    allowed_heading_modes: Sequence[str],
    *,
    contract_version: str = POOL_POSITION_MISSION_V1,
    max_cruise_speed: float | None = None,
    max_lookahead_dist: float | None = None,
    max_reach_threshold: float | None = None,
    num_waypoints: int | None = None,
) -> None:
    for name, value, maximum in (
        ("cruise_speed", settings.cruise_speed, max_cruise_speed),
        ("lookahead_dist", settings.lookahead_dist, max_lookahead_dist),
        ("reach_threshold", settings.reach_threshold, max_reach_threshold),
    ):
        numeric = _finite_float(value, name)
        if numeric <= 0.0:
            raise ValueError(f"{name} must be positive")
        if maximum is not None:
            maximum = _finite_float(maximum, f"max_{name}")
            if maximum <= 0.0:
                raise ValueError(f"max_{name} must be positive")
            if numeric > maximum:
                raise ValueError(
                    f"{name}={numeric:g} exceeds operational maximum {maximum:g}"
                )
    if settings.cruise_speed_per_leg is not None:
        if contract_version != POOL_POSITION_MISSION_V1:
            raise ValueError(
                "cruise_speed_per_leg is only defined for mission v1"
            )
        per_leg = settings.cruise_speed_per_leg
        if len(per_leg) == 0:
            raise ValueError("cruise_speed_per_leg must not be empty if set")
        if num_waypoints is not None and len(per_leg) != num_waypoints:
            raise ValueError(
                f"cruise_speed_per_leg has {len(per_leg)} values; expected "
                f"one per waypoint ({num_waypoints})"
            )
        for index, value in enumerate(per_leg):
            numeric = _finite_float(value, f"cruise_speed_per_leg[{index}]")
            if numeric <= 0.0:
                raise ValueError(f"cruise_speed_per_leg[{index}] must be positive")
            if max_cruise_speed is not None and numeric > max_cruise_speed:
                raise ValueError(
                    f"cruise_speed_per_leg[{index}]={numeric:g} exceeds "
                    f"operational maximum {max_cruise_speed:g}"
                )
    contract_version = str(contract_version).strip()
    contract_modes = CONTRACT_HEADING_MODES.get(contract_version)
    if contract_modes is None:
        raise ValueError(
            f"unsupported mission contract_version={contract_version!r}"
        )
    allowed = tuple(str(mode).strip() for mode in allowed_heading_modes)
    if not allowed or any(not mode for mode in allowed):
        raise ValueError("allowed_heading_modes must not be empty")
    if not set(allowed).issubset(contract_modes):
        raise ValueError(
            "allowed_heading_modes must be a subset of the selected "
            f"contract modes: {sorted(contract_modes)}"
        )
    if settings.heading_mode not in allowed:
        raise ValueError(
            f"heading_mode={settings.heading_mode!r} is not allowed; "
            f"expected one of {sorted(allowed)}"
        )
    if contract_version == POOL_POSITION_MISSION_V1:
        if settings.random_attitude is not None:
            raise ValueError("mission v1 must not carry random_attitude metadata")
        return

    if settings.heading_mode != "random_at_waypoint":
        raise ValueError("mission v2 requires heading_mode='random_at_waypoint'")
    if not settings.loop:
        raise ValueError("mission v2 requires loop=true with a finite max_laps")
    if settings.random_attitude is None:
        raise ValueError("mission v2 requires random_attitude metadata")
    validate_random_attitude_settings(settings.random_attitude)


def _positive_finite(value, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def validate_random_attitude_settings(
    settings: RandomAttitudeSettings,
) -> None:
    """Validate every safety-relevant field bound into the v2 plan hash."""

    if isinstance(settings.seed, bool) or not isinstance(settings.seed, int):
        raise ValueError("random_attitude.seed must be an integer")
    # ROS parameters are signed int64 even though the counter input is encoded
    # as an unsigned integer by the deterministic sampler.
    if settings.seed < 0 or settings.seed > (2**63 - 1):
        raise ValueError("random_attitude.seed must be in [0, 2^63-1]")
    if settings.reference_frame != RANDOM_ATTITUDE_REFERENCE_FRAME:
        raise ValueError(
            "random_attitude.reference_frame must be "
            f"{RANDOM_ATTITUDE_REFERENCE_FRAME!r}"
        )
    if settings.generator_version != RANDOM_ATTITUDE_GENERATOR_VERSION:
        raise ValueError(
            "random_attitude.generator_version must be "
            f"{RANDOM_ATTITUDE_GENERATOR_VERSION!r}"
        )

    minimum = xyz(settings.rpy_min_rad, "random_attitude.rpy_min_rad")
    maximum = xyz(settings.rpy_max_rad, "random_attitude.rpy_max_rad")
    legacy_minimum = (-math.pi / 2.0, -math.pi / 2.0, -math.pi)
    legacy_maximum = (math.pi / 2.0, math.pi / 2.0, math.pi)
    for axis, lower, upper, safe_lower, safe_upper in zip(
        "rpy", minimum, maximum, legacy_minimum, legacy_maximum
    ):
        if lower >= upper:
            raise ValueError(
                f"random_attitude {axis} minimum must be below maximum"
            )
        if lower < safe_lower or upper > safe_upper:
            raise ValueError(
                f"random_attitude {axis} bounds [{lower:g}, {upper:g}] "
                f"exceed the v2 limit [{safe_lower:g}, {safe_upper:g}]"
            )

    max_slew = _positive_finite(
        settings.max_slew_rate_rad_s,
        "random_attitude.max_slew_rate_rad_s",
    )
    attitude_tolerance = _positive_finite(
        settings.attitude_tolerance_rad,
        "random_attitude.attitude_tolerance_rad",
    )
    angular_speed_tolerance = _positive_finite(
        settings.angular_speed_tolerance_rad_s,
        "random_attitude.angular_speed_tolerance_rad_s",
    )
    dwell = _positive_finite(
        settings.dwell_time_s, "random_attitude.dwell_time_s"
    )
    duration = _positive_finite(
        settings.max_duration_s, "random_attitude.max_duration_s"
    )
    if dwell >= duration:
        raise ValueError(
            "random_attitude.dwell_time_s must be below max_duration_s"
        )
    if max_slew > math.pi:
        raise ValueError(
            "random_attitude.max_slew_rate_rad_s must not exceed pi"
        )
    if attitude_tolerance > math.pi:
        raise ValueError(
            "random_attitude.attitude_tolerance_rad must not exceed pi"
        )
    if angular_speed_tolerance > math.pi:
        raise ValueError(
            "random_attitude.angular_speed_tolerance_rad_s must not exceed pi"
        )
    if (
        isinstance(settings.max_laps, bool)
        or not isinstance(settings.max_laps, int)
        or settings.max_laps <= 0
        or settings.max_laps > (2**31 - 1)
    ):
        raise ValueError(
            "random_attitude.max_laps must be an integer in [1, 2^31-1]"
        )


def validate_draft_geometry(
    points: Sequence[Sequence[float]],
    orientations: Sequence[Sequence[float]],
    current_pool_position: Sequence[float],
    settings: ValidationSettings,
    *,
    min_waypoints: int = 2,
    max_waypoints: int | None = None,
    max_segment_length_m: float | None = None,
    loop: bool = False,
) -> tuple[Point3, ...]:
    """Validate a position-only path inside a convex pool safe box.

    A looping mission includes the closing edge from the last waypoint back to
    the first in every segment-length check. Callers therefore represent a
    closed polygon with unique vertices and must not duplicate the first point
    at the end.

    The safe box bounds authored waypoint centres, not the vehicle's measured
    start pose.  A vehicle may legitimately start just outside that box (for
    example, resting on the pool floor below the minimum cruising height).
    Entry remains bounded by ``max_first_point_distance_m``: the first
    waypoint must be inside the safe box and close to the measured start pose.
    """

    if not isinstance(loop, bool):
        raise ValueError("loop must be a boolean")
    if isinstance(min_waypoints, bool) or not isinstance(min_waypoints, int):
        raise ValueError("min_waypoints must be an integer")
    if min_waypoints < 2:
        raise ValueError("min_waypoints must be at least two")
    if max_waypoints is not None:
        if isinstance(max_waypoints, bool) or not isinstance(max_waypoints, int):
            raise ValueError("max_waypoints must be an integer")
        if max_waypoints < 2:
            raise ValueError("max_waypoints must be at least two")
        if min_waypoints > max_waypoints:
            raise ValueError("min_waypoints must not exceed max_waypoints")
    if len(points) < min_waypoints:
        raise ValueError(
            f"draft path has {len(points)} poses; minimum is {min_waypoints}"
        )
    if max_waypoints is not None:
        if len(points) > max_waypoints:
            raise ValueError(
                f"draft path has {len(points)} poses; maximum is {max_waypoints}"
            )
    if len(orientations) != len(points):
        raise ValueError("each draft point must have one orientation")

    minimum = xyz(settings.safe_min_xyz, "safe_min_xyz")
    maximum = xyz(settings.safe_max_xyz, "safe_max_xyz")
    if any(lower >= upper for lower, upper in zip(minimum, maximum)):
        raise ValueError("each safe_min_xyz value must be < safe_max_xyz")

    normalized: list[Point3] = []
    for index, (point_value, orientation_value) in enumerate(
        zip(points, orientations)
    ):
        point = xyz(point_value, f"waypoint[{index}]")
        orientation = quaternion(
            orientation_value, f"waypoint[{index}].orientation"
        )
        validate_identity_quaternion(
            orientation, settings.identity_orientation_tolerance
        )
        for axis, value, lower, upper in zip(
            "xyz", point, minimum, maximum
        ):
            if value < lower or value > upper:
                raise ValueError(
                    f"waypoint[{index}].{axis}={value:g} is outside "
                    f"the pool safe range [{lower:g}, {upper:g}]"
                )
        normalized.append(point)

    min_segment_length = _finite_float(
        settings.min_segment_length_m, "min_segment_length_m"
    )
    if min_segment_length <= 0.0:
        raise ValueError("min_segment_length_m must be positive")
    segment_pairs = list(zip(normalized, normalized[1:]))
    if loop:
        segment_pairs.append((normalized[-1], normalized[0]))
    for index, (start, end) in enumerate(segment_pairs):
        length = math.dist(start, end)
        if length < min_segment_length:
            raise ValueError(
                f"segment[{index}] length {length:.6g} m is below "
                f"{min_segment_length:g} m"
            )
        if max_segment_length_m is not None:
            maximum_segment = _finite_float(
                max_segment_length_m, "max_segment_length_m"
            )
            if maximum_segment <= 0.0:
                raise ValueError("max_segment_length_m must be positive")
            if length > maximum_segment:
                raise ValueError(
                    f"segment[{index}] length {length:.6g} m exceeds "
                    f"{maximum_segment:g} m"
                )
        # The safe region is a convex axis-aligned box. A straight segment is
        # contained exactly when both endpoints are contained. Check its axis
        # extrema explicitly so this invariant remains visible in the contract.
        for axis, start_value, end_value, lower, upper in zip(
            "xyz", start, end, minimum, maximum
        ):
            segment_min = min(start_value, end_value)
            segment_max = max(start_value, end_value)
            if segment_min < lower or segment_max > upper:
                raise ValueError(
                    f"segment[{index}].{axis} leaves the pool safe range"
                )

    # Validate that the live position is a finite XYZ value, but do not apply
    # the waypoint-centre safe box to it.  The robot can start on the floor or
    # otherwise just outside the conservative cruising box.  The distance gate
    # below still prevents an arbitrary jump into an unrelated mission.
    current = xyz(current_pool_position, "current pool position")
    maximum_distance = _finite_float(
        settings.max_first_point_distance_m,
        "max_first_point_distance_m",
    )
    if maximum_distance <= 0.0:
        raise ValueError("max_first_point_distance_m must be positive")
    first_distance = math.dist(current, normalized[0])
    if first_distance > maximum_distance:
        raise ValueError(
            f"first waypoint is {first_distance:.3f} m from the current "
            f"pool pose (limit {maximum_distance:.3f} m)"
        )
    return tuple(normalized)


def normalized_quaternion(value: Sequence[float]) -> Quaternion:
    raw = quaternion(value, "transform quaternion")
    norm = math.sqrt(sum(component * component for component in raw))
    if norm <= 1e-12:
        raise ValueError("transform quaternion norm is zero")
    return tuple(component / norm for component in raw)


def rotate_point(point: Point3, value: Quaternion) -> Point3:
    """Rotate a point with an xyzw quaternion without external dependencies."""

    x, y, z, w = normalized_quaternion(value)
    px, py, pz = point
    # R(q) p, expanded for ROS xyzw ordering.
    return (
        (1.0 - 2.0 * (y * y + z * z)) * px
        + 2.0 * (x * y - z * w) * py
        + 2.0 * (x * z + y * w) * pz,
        2.0 * (x * y + z * w) * px
        + (1.0 - 2.0 * (x * x + z * z)) * py
        + 2.0 * (y * z - x * w) * pz,
        2.0 * (x * z - y * w) * px
        + 2.0 * (y * z + x * w) * py
        + (1.0 - 2.0 * (x * x + y * y)) * pz,
    )


def transform_points(
    points: Sequence[Point3],
    translation: Sequence[float],
    rotation_xyzw: Sequence[float],
) -> tuple[Point3, ...]:
    offset = xyz(translation, "transform translation")
    rotation = normalized_quaternion(rotation_xyzw)
    transformed = []
    for point in points:
        rotated = rotate_point(xyz(point, "point"), rotation)
        transformed.append(
            tuple(value + delta for value, delta in zip(rotated, offset))
        )
    return tuple(transformed)


def invert_transform(
    translation: Sequence[float],
    rotation_xyzw: Sequence[float],
) -> tuple[Point3, Quaternion]:
    """Invert a rigid transform expressed as translation plus ROS xyzw.

    If the input maps child coordinates into parent coordinates, the returned
    transform maps parent coordinates into child coordinates.
    """

    offset = xyz(translation, "transform translation")
    x, y, z, w = normalized_quaternion(rotation_xyzw)
    inverse_rotation = (-x, -y, -z, w)
    inverse_translation = rotate_point(
        (-offset[0], -offset[1], -offset[2]), inverse_rotation
    )
    return inverse_translation, inverse_rotation


def canonical_plan_content(
    points: Sequence[Point3],
    settings: MissionSettings,
    *,
    frame_id: str = "pool",
    contract_version: str = POOL_POSITION_MISSION_V1,
) -> bytes:
    """Return stable content bytes; timestamps and localization are excluded."""

    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a non-empty string")
    canonical_frame = frame_id.strip()
    contract_version = str(contract_version).strip()
    points = list(points)
    validate_mission_settings(
        settings,
        CONTRACT_HEADING_MODES.get(contract_version, ()),
        contract_version=contract_version,
        num_waypoints=len(points),
    )

    payload = {
        "contract": contract_version,
        "frame_id": canonical_frame,
        "waypoints": [[float(value) for value in point] for point in points],
        "cruise_speed": float(settings.cruise_speed),
        "lookahead_dist": float(settings.lookahead_dist),
        "reach_threshold": float(settings.reach_threshold),
        "heading_mode": settings.heading_mode,
        "loop": bool(settings.loop),
    }
    if settings.cruise_speed_per_leg is not None:
        payload["cruise_speed_per_leg"] = [
            float(value) for value in settings.cruise_speed_per_leg
        ]
    if contract_version == POOL_POSITION_MISSION_V2:
        random = settings.random_attitude
        assert random is not None
        payload["random_attitude"] = {
            "seed": int(random.seed),
            "reference_frame": random.reference_frame,
            "generator_version": random.generator_version,
            "rpy_min_rad": [float(value) for value in random.rpy_min_rad],
            "rpy_max_rad": [float(value) for value in random.rpy_max_rad],
            "max_slew_rate_rad_s": float(random.max_slew_rate_rad_s),
            "attitude_tolerance_rad": float(random.attitude_tolerance_rad),
            "angular_speed_tolerance_rad_s": float(
                random.angular_speed_tolerance_rad_s
            ),
            "dwell_time_s": float(random.dwell_time_s),
            "max_duration_s": float(random.max_duration_s),
            "max_laps": int(random.max_laps),
        }
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")


def plan_hash(
    points: Sequence[Point3],
    settings: MissionSettings,
    *,
    frame_id: str = "pool",
    contract_version: str = POOL_POSITION_MISSION_V1,
) -> str:
    return hashlib.sha256(
        canonical_plan_content(
            points,
            settings,
            frame_id=frame_id,
            contract_version=contract_version,
        )
    ).hexdigest()
