import hashlib
import json
import math

import pytest

from brov_mission.core import (
    canonical_plan_content,
    invert_transform,
    MissionSettings,
    POOL_POSITION_MISSION_V1,
    POOL_POSITION_MISSION_V2,
    ValidationSettings,
    plan_hash,
    RANDOM_ATTITUDE_GENERATOR_VERSION,
    RANDOM_ATTITUDE_REFERENCE_FRAME,
    RandomAttitudeSettings,
    transform_points,
    validate_draft_geometry,
    validate_mission_settings,
    validate_random_attitude_settings,
)


MISSION = MissionSettings(
    cruise_speed=0.1,
    lookahead_dist=0.4,
    reach_threshold=0.15,
    heading_mode="straight",
    loop=False,
)

VALIDATION = ValidationSettings(
    safe_min_xyz=(0.0, 0.0, 0.0),
    safe_max_xyz=(4.0, 1.7, 1.1),
    max_first_point_distance_m=0.3,
    min_segment_length_m=0.05,
    identity_orientation_tolerance=1e-3,
    allowed_heading_modes=("straight", "align"),
)

RANDOM_ATTITUDE = RandomAttitudeSettings(
    seed=20260814,
    reference_frame=RANDOM_ATTITUDE_REFERENCE_FRAME,
    generator_version=RANDOM_ATTITUDE_GENERATOR_VERSION,
    rpy_min_rad=(-math.pi / 2.0, -math.pi / 2.0, -math.pi),
    rpy_max_rad=(math.pi / 2.0, math.pi / 2.0, math.pi),
    max_slew_rate_rad_s=0.35,
    attitude_tolerance_rad=math.radians(10.0),
    angular_speed_tolerance_rad_s=math.radians(5.0),
    dwell_time_s=2.0,
    max_duration_s=120.0,
    max_laps=1,
)

RANDOM_MISSION = MissionSettings(
    cruise_speed=0.05,
    lookahead_dist=0.15,
    reach_threshold=0.08,
    heading_mode="random_at_waypoint",
    loop=True,
    random_attitude=RANDOM_ATTITUDE,
)


def _reference_random_quaternion(metadata: dict, event_index: int):
    """Independent, dependency-free oracle for the documented v1 generator."""

    angles = []
    for axis_index, (lower, upper) in enumerate(
        zip(metadata["rpy_min_rad"], metadata["rpy_max_rad"])
    ):
        payload = (
            f"{metadata['generator_version']}:{metadata['seed']}:"
            f"{event_index}:{axis_index}"
        ).encode("ascii")
        integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
        uniform = integer / float(1 << 64)
        angles.append(lower + uniform * (upper - lower))

    roll, pitch, yaw = angles
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    normalized = tuple(value / norm for value in quaternion)
    if normalized[0] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized


def test_valid_position_only_pool_path() -> None:
    points = validate_draft_geometry(
        [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
        [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, -1.0)],
        (1.05, 0.85, 0.3),
        VALIDATION,
    )
    assert points == ((1.0, 0.85, 0.3), (2.0, 0.85, 0.3))


@pytest.mark.parametrize(
    "orientations, match",
    [
        (
            [(0.0, 0.0, 0.2, math.sqrt(0.96)), (0.0, 0.0, 0.0, 1.0)],
            "non-identity",
        ),
        (
            [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
            "quaternion norm",
        ),
    ],
)
def test_pose_orientation_fails_closed(orientations, match) -> None:
    with pytest.raises(ValueError, match=match):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
            orientations,
            (1.0, 0.85, 0.3),
            VALIDATION,
        )


def test_waypoint_outside_safe_box_is_rejected() -> None:
    with pytest.raises(ValueError, match="outside the pool safe range"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (4.01, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (1.0, 0.85, 0.3),
            VALIDATION,
        )


def test_short_segment_is_rejected() -> None:
    with pytest.raises(ValueError, match=r"segment\[0\] length"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (1.01, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (1.0, 0.85, 0.3),
            VALIDATION,
        )


def test_first_point_distance_gate() -> None:
    with pytest.raises(ValueError, match="first waypoint"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (1.0, 0.85, -0.4),
            VALIDATION,
        )


def test_nearby_bottom_start_below_safe_box_is_allowed() -> None:
    points = validate_draft_geometry(
        [(1.0, 0.85, 0.0), (2.0, 0.85, 0.0)],
        [(0.0, 0.0, 0.0, 1.0)] * 2,
        (1.0, 0.85, -0.025),
        VALIDATION,
    )

    assert points == ((1.0, 0.85, 0.0), (2.0, 0.85, 0.0))


@pytest.mark.parametrize("invalid_x", [math.nan, math.inf, -math.inf])
def test_current_position_must_remain_finite(invalid_x: float) -> None:
    with pytest.raises(
        ValueError,
        match=r"current pool position\[0\] must be finite",
    ):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (invalid_x, 0.85, 0.3),
            VALIDATION,
        )


def test_transform_points_uses_target_from_source_transform() -> None:
    # +90 degrees about Z, followed by translation.
    q = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    actual = transform_points([(1.0, 0.0, 0.0)], (2.0, 3.0, 4.0), q)
    assert actual[0] == pytest.approx((2.0, 4.0, 4.0), abs=1e-12)


def test_invert_transform_round_trip() -> None:
    q = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    translation = (2.0, 3.0, 4.0)
    point = (0.7, -0.2, 1.1)
    transformed = transform_points([point], translation, q)[0]
    inverse_translation, inverse_rotation = invert_transform(translation, q)
    recovered = transform_points(
        [transformed], inverse_translation, inverse_rotation
    )[0]
    assert recovered == pytest.approx(point, abs=1e-12)


def test_plan_hash_is_canonical_and_content_sensitive() -> None:
    points = ((1.0, 0.85, 0.3), (2.0, 0.85, 0.3))
    first = plan_hash(points, MISSION)
    second = plan_hash(tuple(tuple(v for v in p) for p in points), MISSION)
    changed = plan_hash(
        points,
        MissionSettings(0.11, 0.4, 0.15, "straight", False),
    )
    assert first == second
    assert len(first) == 64
    assert first != changed


def test_canonical_plan_frame_defaults_to_pool_and_affects_hash() -> None:
    points = ((1.0, 0.85, 0.3), (2.0, 0.85, 0.3))
    default_content = canonical_plan_content(points, MISSION)
    explicit_pool = canonical_plan_content(
        points, MISSION, frame_id="pool"
    )
    custom_content = canonical_plan_content(
        points, MISSION, frame_id="surveyed_pool"
    )

    assert default_content == explicit_pool
    assert json.loads(default_content)["frame_id"] == "pool"
    assert json.loads(custom_content)["frame_id"] == "surveyed_pool"
    assert plan_hash(points, MISSION) == hashlib.sha256(
        default_content
    ).hexdigest()
    assert plan_hash(
        points, MISSION, frame_id="surveyed_pool"
    ) == hashlib.sha256(custom_content).hexdigest()
    assert plan_hash(points, MISSION) != plan_hash(
        points, MISSION, frame_id="surveyed_pool"
    )


@pytest.mark.parametrize("frame_id", ["", "   ", None])
def test_canonical_plan_rejects_empty_frame_id(frame_id) -> None:
    with pytest.raises(ValueError, match="frame_id must be a non-empty string"):
        canonical_plan_content(
            ((0.0, 0.0, 0.0),), MISSION, frame_id=frame_id
        )


def test_cruise_speed_per_leg_included_in_canonical_content_when_set() -> None:
    points = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), (2.0, 0.0, 0.2))
    per_leg_mission = MissionSettings(
        cruise_speed=0.5,
        lookahead_dist=0.4,
        reach_threshold=0.15,
        heading_mode="takeoff_then_align",
        loop=True,
        cruise_speed_per_leg=(0.5, 0.25, 0.5),
    )
    with_per_leg = canonical_plan_content(points, per_leg_mission)
    without_per_leg = canonical_plan_content(
        points,
        MissionSettings(
            cruise_speed=0.5,
            lookahead_dist=0.4,
            reach_threshold=0.15,
            heading_mode="takeoff_then_align",
            loop=True,
        ),
    )
    payload = json.loads(with_per_leg)
    assert payload["cruise_speed_per_leg"] == [0.5, 0.25, 0.5]
    assert "cruise_speed_per_leg" not in json.loads(without_per_leg)
    assert with_per_leg != without_per_leg


def test_cruise_speed_per_leg_length_must_match_waypoint_count() -> None:
    points = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), (2.0, 0.0, 0.2))
    mission = MissionSettings(
        cruise_speed=0.5,
        lookahead_dist=0.4,
        reach_threshold=0.15,
        heading_mode="takeoff_then_align",
        loop=True,
        cruise_speed_per_leg=(0.5, 0.25),
    )
    with pytest.raises(ValueError, match="expected one per waypoint"):
        canonical_plan_content(points, mission)


def test_cruise_speed_per_leg_rejects_non_positive_and_over_max() -> None:
    with pytest.raises(ValueError, match="must be positive"):
        validate_mission_settings(
            MissionSettings(
                0.5, 0.4, 0.15, "takeoff_then_align", True,
                cruise_speed_per_leg=(0.5, 0.0, 0.5),
            ),
            ("takeoff_then_align",),
        )
    with pytest.raises(ValueError, match="exceeds operational maximum"):
        validate_mission_settings(
            MissionSettings(
                0.5, 0.4, 0.15, "takeoff_then_align", True,
                cruise_speed_per_leg=(0.5, 0.9, 0.5),
            ),
            ("takeoff_then_align",),
            max_cruise_speed=0.6,
        )


def test_cruise_speed_per_leg_rejected_for_v2_contract() -> None:
    with pytest.raises(ValueError, match="only defined for mission v1"):
        validate_mission_settings(
            MissionSettings(
                cruise_speed=0.05,
                lookahead_dist=0.15,
                reach_threshold=0.08,
                heading_mode="random_at_waypoint",
                loop=True,
                random_attitude=RANDOM_ATTITUDE,
                cruise_speed_per_leg=(0.05, 0.05, 0.05, 0.05),
            ),
            ("random_at_waypoint",),
            contract_version=POOL_POSITION_MISSION_V2,
        )


def test_heading_mode_allowlist() -> None:
    validate_mission_settings(MISSION, VALIDATION.allowed_heading_modes)
    with pytest.raises(ValueError, match="not allowed"):
        validate_mission_settings(
            MissionSettings(0.1, 0.4, 0.15, "random_at_waypoint", False),
            VALIDATION.allowed_heading_modes,
        )


def test_v2_random_contract_is_canonical_hashed_and_distinct_from_v1() -> None:
    points = ((1.0, 0.7, 0.4), (1.4, 0.7, 0.4))
    validate_mission_settings(
        RANDOM_MISSION,
        ("random_at_waypoint",),
        contract_version=POOL_POSITION_MISSION_V2,
    )

    content = canonical_plan_content(
        points,
        RANDOM_MISSION,
        contract_version=POOL_POSITION_MISSION_V2,
    )
    canonical = json.loads(content)
    assert canonical["contract"] == POOL_POSITION_MISSION_V2
    assert canonical["random_attitude"] == {
        "seed": 20260814,
        "reference_frame": "pool_zup_flu",
        "generator_version": "sha256_counter_uniform_rpy_v1",
        "rpy_min_rad": [-math.pi / 2.0, -math.pi / 2.0, -math.pi],
        "rpy_max_rad": [math.pi / 2.0, math.pi / 2.0, math.pi],
        "max_slew_rate_rad_s": 0.35,
        "attitude_tolerance_rad": math.radians(10.0),
        "angular_speed_tolerance_rad_s": math.radians(5.0),
        "dwell_time_s": 2.0,
        "max_duration_s": 120.0,
        "max_laps": 1,
    }
    assert plan_hash(
        points,
        RANDOM_MISSION,
        contract_version=POOL_POSITION_MISSION_V2,
    ) == hashlib.sha256(content).hexdigest()
    assert canonical["contract"] != POOL_POSITION_MISSION_V1


def test_v2_generator_golden_vector_from_canonical_metadata() -> None:
    """Keep the producer metadata interoperable with the consumer sampler."""

    canonical = json.loads(
        canonical_plan_content(
            ((1.0, 0.7, 0.4), (1.4, 0.7, 0.4)),
            RANDOM_MISSION,
            contract_version=POOL_POSITION_MISSION_V2,
        )
    )
    assert _reference_random_quaternion(
        canonical["random_attitude"], event_index=0
    ) == pytest.approx(
        (
            0.36995846,
            0.15418720,
            0.61874908,
            -0.67565274,
        ),
        abs=1e-7,
        rel=0.0,
    )


@pytest.mark.parametrize(
    "replacement",
    [
        {"seed": 20260815},
        {"rpy_min_rad": (-1.0, -1.0, -2.0)},
        {"rpy_max_rad": (1.0, 1.0, 2.0)},
        {"max_slew_rate_rad_s": 0.30},
        {"attitude_tolerance_rad": 0.15},
        {"angular_speed_tolerance_rad_s": 0.08},
        {"dwell_time_s": 3.0},
        {"max_duration_s": 90.0},
        {"max_laps": 2},
    ],
)
def test_v2_plan_hash_binds_every_variable_random_setting(replacement) -> None:
    points = ((1.0, 0.7, 0.4), (1.4, 0.7, 0.4))
    original = plan_hash(
        points,
        RANDOM_MISSION,
        contract_version=POOL_POSITION_MISSION_V2,
    )
    random_values = dict(RANDOM_ATTITUDE.__dict__)
    random_values.update(replacement)
    changed = MissionSettings(
        cruise_speed=RANDOM_MISSION.cruise_speed,
        lookahead_dist=RANDOM_MISSION.lookahead_dist,
        reach_threshold=RANDOM_MISSION.reach_threshold,
        heading_mode=RANDOM_MISSION.heading_mode,
        loop=RANDOM_MISSION.loop,
        random_attitude=RandomAttitudeSettings(**random_values),
    )
    assert plan_hash(
        points,
        changed,
        contract_version=POOL_POSITION_MISSION_V2,
    ) != original


def test_v1_is_frozen_and_v2_is_random_only() -> None:
    with pytest.raises(ValueError, match="selected contract modes"):
        validate_mission_settings(
            RANDOM_MISSION,
            ("random_at_waypoint",),
            contract_version=POOL_POSITION_MISSION_V1,
        )
    with pytest.raises(ValueError, match="selected contract modes"):
        validate_mission_settings(
            MISSION,
            ("straight",),
            contract_version=POOL_POSITION_MISSION_V2,
        )
    with pytest.raises(ValueError, match="requires loop=true"):
        validate_mission_settings(
            MissionSettings(
                0.05,
                0.15,
                0.08,
                "random_at_waypoint",
                False,
                RANDOM_ATTITUDE,
            ),
            ("random_at_waypoint",),
            contract_version=POOL_POSITION_MISSION_V2,
        )


@pytest.mark.parametrize(
    ("replacement", "match"),
    [
        ({"reference_frame": "start_heading"}, "reference_frame"),
        ({"generator_version": "torch_rand_v1"}, "generator_version"),
        ({"rpy_min_rad": (-2.0, -1.0, -1.0)}, "exceed the v2 limit"),
        ({"rpy_max_rad": (0.0, 1.0, 1.0), "rpy_min_rad": (0.0, -1.0, -1.0)}, "minimum"),
        ({"max_slew_rate_rad_s": 0.0}, "must be positive"),
        ({"dwell_time_s": 120.0}, "below max_duration_s"),
        ({"max_laps": 0}, "max_laps"),
    ],
)
def test_random_attitude_metadata_fails_closed(replacement, match) -> None:
    values = dict(RANDOM_ATTITUDE.__dict__)
    values.update(replacement)
    with pytest.raises(ValueError, match=match):
        validate_random_attitude_settings(RandomAttitudeSettings(**values))


def test_operational_setting_maxima_fail_closed() -> None:
    with pytest.raises(ValueError, match="exceeds operational maximum"):
        validate_mission_settings(
            MissionSettings(100.0, 0.4, 0.15, "straight", False),
            VALIDATION.allowed_heading_modes,
            max_cruise_speed=0.3,
        )


def test_waypoint_count_and_segment_maxima_fail_closed() -> None:
    with pytest.raises(ValueError, match="minimum is 4"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3), (3.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 3,
            (1.0, 0.85, 0.3),
            VALIDATION,
            min_waypoints=4,
            max_waypoints=4,
        )
    with pytest.raises(ValueError, match="must not exceed"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (1.0, 0.85, 0.3),
            VALIDATION,
            min_waypoints=3,
            max_waypoints=2,
        )
    with pytest.raises(ValueError, match="maximum is 2"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3), (3.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 3,
            (1.0, 0.85, 0.3),
            VALIDATION,
            max_waypoints=2,
        )
    with pytest.raises(ValueError, match="exceeds"):
        validate_draft_geometry(
            [(1.0, 0.85, 0.3), (2.0, 0.85, 0.3)],
            [(0.0, 0.0, 0.0, 1.0)] * 2,
            (1.0, 0.85, 0.3),
            VALIDATION,
            max_segment_length_m=0.5,
        )


def test_loop_rejects_duplicate_terminal_point_as_zero_closing_edge() -> None:
    points = [
        (1.0, 0.5, 0.3),
        (1.4, 0.5, 0.3),
        (1.4, 0.9, 0.3),
        (1.0, 0.9, 0.3),
        (1.0, 0.5, 0.3),
    ]
    with pytest.raises(ValueError, match=r"segment\[4\] length"):
        validate_draft_geometry(
            points,
            [(0.0, 0.0, 0.0, 1.0)] * len(points),
            points[0],
            VALIDATION,
            loop=True,
        )


def test_loop_closing_segment_obeys_maximum_length() -> None:
    points = [
        (1.0, 0.5, 0.3),
        (1.4, 0.5, 0.3),
        (1.8, 0.5, 0.3),
    ]
    orientations = [(0.0, 0.0, 0.0, 1.0)] * len(points)
    validate_draft_geometry(
        points,
        orientations,
        points[0],
        VALIDATION,
        max_segment_length_m=0.5,
        loop=False,
    )
    with pytest.raises(ValueError, match=r"segment\[2\].*exceeds"):
        validate_draft_geometry(
            points,
            orientations,
            points[0],
            VALIDATION,
            max_segment_length_m=0.5,
            loop=True,
        )
