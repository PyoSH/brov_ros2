"""In-process ROS test of validation and immutable mission resolution."""

import hashlib
import json
import math
import time
from types import SimpleNamespace

import pytest

try:
    import rclpy
    from brov_interfaces.msg import (
        AlignedOdometry,
        LocalizationStatus,
        ResolvedMission,
    )
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_srvs.srv import Trigger

    from brov_mission.mission_manager_node import MissionManagerNode
except ImportError:
    pytest.skip("ROS 2 runtime is unavailable", allow_module_level=True)


def _pump(executor, predicate, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("condition did not become true before timeout")


def _status(
    stamp, alignment_id: str, frame_id: str = "pool"
) -> LocalizationStatus:
    message = LocalizationStatus()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.state = LocalizationStatus.INITIALIZED
    message.epoch = 7
    message.odometry_session_id = "mavlink-boot-session-1:nav0"
    message.alignment_id = alignment_id
    message.output_valid = True
    message.sample_count = 12
    message.reason = "one-shot alignment initialized"

    # ^pool T_odom: translate by (1, 2, 3), then rotate +90 deg about Z.
    message.pool_to_odom.translation.x = 1.0
    message.pool_to_odom.translation.y = 2.0
    message.pool_to_odom.translation.z = 3.0
    message.pool_to_odom.rotation.z = math.sqrt(0.5)
    message.pool_to_odom.rotation.w = math.sqrt(0.5)
    return message


def _pool_odometry(stamp, frame_id: str = "pool") -> Odometry:
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 2.0
    message.pose.pose.position.y = 3.0
    message.pose.pose.position.z = 4.0
    message.pose.pose.orientation.w = 1.0
    return message


def _aligned_odometry(
    stamp, alignment_id: str, frame_id: str = "pool"
) -> AlignedOdometry:
    message = AlignedOdometry()
    message.odometry = _pool_odometry(stamp, frame_id)
    message.localization_epoch = 7
    message.odometry_session_id = "mavlink-boot-session-1:nav0"
    message.alignment_id = alignment_id
    return message


def _draft(stamp, frame_id: str = "pool") -> Path:
    message = Path()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    for x, y, z in ((2.0, 3.0, 4.0), (3.0, 3.0, 4.0)):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("localization_epoch", 8, "epoch does not match"),
        ("odometry_session_id", "other-session", "session does not match"),
        ("alignment_id", "other-alignment", "alignment does not match"),
    ],
)
def test_current_position_rejects_each_atomic_identity_mismatch(
    field, value, match
) -> None:
    """Require exact status identity before reading the enclosed position."""
    envelope = _aligned_odometry(
        Odometry().header.stamp,
        "alignment-a",
    )
    setattr(envelope, field, value)
    owner = SimpleNamespace(_aligned_odometry=envelope)
    localization = {
        "epoch": 7,
        "odometry_session_id": "mavlink-boot-session-1:nav0",
        "alignment_id": "alignment-a",
    }
    with pytest.raises(ValueError, match=match):
        MissionManagerNode._current_pool_position(owner, localization)


@pytest.mark.parametrize(
    "allowed_modes",
    [
        ["straight", "upright"],
        [""],
    ],
)
def test_constructor_rejects_heading_modes_outside_consumer_contract(
    allowed_modes,
) -> None:
    rclpy.init()
    try:
        override = Parameter(
            "allowed_heading_modes",
            Parameter.Type.STRING_ARRAY,
            allowed_modes,
        )
        with pytest.raises(ValueError, match="non-empty subset"):
            MissionManagerNode(parameter_overrides=[override])
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_constructor_accepts_only_versioned_random_contract() -> None:
    rclpy.init()
    try:
        v1_overrides = [
            Parameter(
                "heading_mode",
                Parameter.Type.STRING,
                "random_at_waypoint",
            ),
            Parameter(
                "allowed_heading_modes",
                Parameter.Type.STRING_ARRAY,
                ["random_at_waypoint"],
            ),
        ]
        with pytest.raises(ValueError, match="selected resolved mission"):
            MissionManagerNode(parameter_overrides=v1_overrides)

        v2_overrides = [
            Parameter(
                "contract_version",
                Parameter.Type.STRING,
                "brov_pool_position_mission_v2",
            ),
            Parameter(
                "heading_mode",
                Parameter.Type.STRING,
                "random_at_waypoint",
            ),
            Parameter("loop", Parameter.Type.BOOL, True),
            Parameter(
                "allowed_heading_modes",
                Parameter.Type.STRING_ARRAY,
                ["random_at_waypoint"],
            ),
            Parameter("random_attitude_seed", Parameter.Type.INTEGER, 7),
        ]
        node = MissionManagerNode(parameter_overrides=v2_overrides)
        try:
            assert node._contract_version == "brov_pool_position_mission_v2"
            assert node._mission_settings.heading_mode == "random_at_waypoint"
            assert node._mission_settings.random_attitude.seed == 7
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_constructor_rejects_waypoint_count_limits_in_wrong_order() -> None:
    rclpy.init()
    try:
        overrides = [
            Parameter("min_waypoints", Parameter.Type.INTEGER, 4),
            Parameter("max_waypoints", Parameter.Type.INTEGER, 3),
        ]
        with pytest.raises(ValueError, match="must not exceed"):
            MissionManagerNode(parameter_overrides=overrides)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize(
    ("contract_version", "heading_mode", "loop", "allowed_modes"),
    [
        (
            "brov_pool_position_mission_v1",
            "straight",
            False,
            "[straight,align]",
        ),
        (
            "brov_pool_position_mission_v2",
            "random_at_waypoint",
            True,
            "[random_at_waypoint]",
        ),
    ],
)
def test_validate_reject_changed_alignment_then_commit_resolved_contract(
    contract_version,
    heading_mode,
    loop,
    allowed_modes,
) -> None:
    """Bind validation to alignment identity and verify resolution."""
    pool_frame = "surveyed_pool"
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"pool_frame:={pool_frame}",
            "-p",
            "pool_safe_min_xyz:=[-10.0,-10.0,-10.0]",
            "-p",
            "pool_safe_max_xyz:=[10.0,10.0,10.0]",
            "-p",
            "localization_max_age_s:=3.0",
            "-p",
            "odometry_max_age_s:=3.0",
            "-p",
            f"contract_version:={contract_version}",
            "-p",
            f"heading_mode:={heading_mode}",
            "-p",
            f"loop:={'true' if loop else 'false'}",
            "-p",
            f"allowed_heading_modes:={allowed_modes}",
        ]
    )
    executor = SingleThreadedExecutor()
    mission = MissionManagerNode()
    harness = Node("mission_runtime_contract_harness")
    latched_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    status_publisher = harness.create_publisher(
        LocalizationStatus, "/brov/localization/status", latched_qos
    )
    odometry_publisher = harness.create_publisher(
        AlignedOdometry,
        "/brov/localization/odometry_pool_with_alignment",
        qos_profile_sensor_data,
    )
    draft_publisher = harness.create_publisher(
        Path, "/brov/mission/draft_path", 1
    )
    resolved_messages = []
    harness.create_subscription(
        ResolvedMission,
        "/brov/mission/resolved",
        resolved_messages.append,
        latched_qos,
    )
    validate_client = harness.create_client(
        Trigger, "/brov/mission/validate"
    )
    commit_client = harness.create_client(Trigger, "/brov/mission/commit")

    executor.add_node(mission)
    executor.add_node(harness)
    try:
        _pump(
            executor,
            lambda: status_publisher.get_subscription_count() == 1
            and odometry_publisher.get_subscription_count() == 1
            and draft_publisher.get_subscription_count() == 1
            and mission._pub_resolved.get_subscription_count() == 1
            and validate_client.service_is_ready()
            and commit_client.service_is_ready(),
        )

        stamp = harness.get_clock().now().to_msg()
        first_alignment = "11111111-1111-4111-8111-111111111111"
        second_alignment = "22222222-2222-4222-8222-222222222222"
        status_publisher.publish(
            _status(stamp, first_alignment, pool_frame)
        )
        odometry_publisher.publish(
            _aligned_odometry(stamp, first_alignment, pool_frame)
        )
        draft_publisher.publish(_draft(stamp, pool_frame))
        _pump(
            executor,
            lambda: mission._localization is not None
            and mission._aligned_odometry is not None
            and mission._draft_revision == 1,
        )

        validate_future = validate_client.call_async(Trigger.Request())
        _pump(executor, validate_future.done)
        assert validate_future.result().success
        assert "explicit commit required" in validate_future.result().message

        # The exact alignment identity is part of the validation snapshot.
        # A new one-shot solution between validate and commit must fail closed.
        status_publisher.publish(
            _status(
                harness.get_clock().now().to_msg(),
                second_alignment,
                pool_frame,
            )
        )
        _pump(
            executor,
            lambda: mission._localization.alignment_id == second_alignment,
        )
        rejected_future = commit_client.call_async(Trigger.Request())
        _pump(executor, rejected_future.done)
        assert not rejected_future.result().success
        assert "aligned odometry alignment does not match" in (
            rejected_future.result().message
        )
        assert not resolved_messages

        # Once the atomic pose carries the new identity too, revalidate and
        # resolve pool points using
        # inverse(^pool T_odom). Expected odom points are (1,-1,1),(1,-2,1).
        odometry_publisher.publish(
            _aligned_odometry(
                harness.get_clock().now().to_msg(),
                second_alignment,
                pool_frame,
            )
        )
        _pump(
            executor,
            lambda: mission._aligned_odometry.alignment_id
            == second_alignment,
        )
        validate_future = validate_client.call_async(Trigger.Request())
        _pump(executor, validate_future.done)
        assert validate_future.result().success
        commit_future = commit_client.call_async(Trigger.Request())
        _pump(executor, commit_future.done)
        assert commit_future.result().success
        _pump(executor, lambda: len(resolved_messages) == 1)

        resolved = resolved_messages[0]
        assert resolved.header.frame_id == "odom"
        assert resolved.contract_version == contract_version
        assert resolved.localization_epoch == 7
        assert resolved.odometry_session_id == "mavlink-boot-session-1:nav0"
        assert resolved.alignment_id == second_alignment
        assert resolved.mission_id
        assert len(resolved.plan_hash) == 64
        assert resolved.plan_hash == hashlib.sha256(
            resolved.canonical_plan_json.encode("ascii")
        ).hexdigest()

        canonical = json.loads(resolved.canonical_plan_json)
        expected_canonical = {
            "contract": contract_version,
            "frame_id": pool_frame,
            "waypoints": [[2.0, 3.0, 4.0], [3.0, 3.0, 4.0]],
            "cruise_speed": 0.1,
            "lookahead_dist": 0.4,
            "reach_threshold": 0.15,
            "heading_mode": heading_mode,
            "loop": loop,
        }
        if contract_version == "brov_pool_position_mission_v2":
            expected_canonical["random_attitude"] = {
                "seed": 0,
                "reference_frame": "pool_zup_flu",
                "generator_version": "sha256_counter_uniform_rpy_v1",
                "rpy_min_rad": [
                    -math.pi / 2.0,
                    -math.pi / 2.0,
                    -math.pi,
                ],
                "rpy_max_rad": [
                    math.pi / 2.0,
                    math.pi / 2.0,
                    math.pi,
                ],
                "max_slew_rate_rad_s": 0.35,
                "attitude_tolerance_rad": 0.1745329252,
                "angular_speed_tolerance_rad_s": 0.0872664626,
                "dwell_time_s": 2.0,
                "max_duration_s": 120.0,
                "max_laps": 1,
            }
        assert canonical == expected_canonical
        actual_points = [
            (point.x, point.y, point.z) for point in resolved.waypoints
        ]
        assert actual_points[0] == pytest.approx((1.0, -1.0, 1.0), abs=1e-6)
        assert actual_points[1] == pytest.approx((1.0, -2.0, 1.0), abs=1e-6)
    finally:
        executor.remove_node(mission)
        executor.remove_node(harness)
        mission.destroy_node()
        harness.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
