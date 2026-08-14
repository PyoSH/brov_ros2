"""Small in-process ROS test of the public localization lifecycle."""

import math
import time
import uuid

import pytest

try:
    import rclpy
    from brov_interfaces.msg import (
        AlignedOdometry,
        LocalizationStatus,
        OdometrySession,
    )
    from brov_interfaces.srv import InitializePool
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
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
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    from brov_localization.localization_node import PoolAlignmentNode
except ImportError:
    pytest.skip("ROS 2 runtime is unavailable", allow_module_level=True)


def _pump(executor, predicate, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("condition did not become true before timeout")


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _odometry(stamp) -> Odometry:
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "test_odom"
    message.child_frame_id = "test_base"
    message.pose.pose.orientation.w = 1.0
    return message


def _envelope(odometry: Odometry, session_id: str) -> OdometrySession:
    message = OdometrySession()
    message.odometry = odometry
    message.odometry_session_id = session_id
    return message


def test_atomic_session_initialize_publish_and_session_invalidation() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    harness = Node("pool_alignment_contract_harness")
    latched_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    prefix = "/brov_test_pool_alignment"

    overrides = [
        Parameter(
            "odometry_session_topic", value=f"{prefix}/odometry_session"
        ),
        Parameter("vision_topic", value=f"{prefix}/vision"),
        Parameter("visible_topic", value=f"{prefix}/visible"),
        Parameter(
            "aligned_odometry_topic", value=f"{prefix}/aligned_odometry"
        ),
        Parameter("pool_odometry_topic", value=f"{prefix}/odometry_pool"),
        Parameter("status_topic", value=f"{prefix}/status"),
        Parameter("valid_topic", value=f"{prefix}/valid"),
        Parameter("pool_frame", value="test_pool"),
        Parameter("odom_frame", value="test_odom"),
        Parameter("base_frame", value="test_base"),
        Parameter("default_min_samples", value=3),
        Parameter("max_message_age_s", value=2.0),
        Parameter("visible_timeout_s", value=2.0),
        Parameter("status_publish_period_s", value=0.1),
    ]
    localization = PoolAlignmentNode(parameter_overrides=overrides)

    odometry_session_publisher = harness.create_publisher(
        OdometrySession,
        f"{prefix}/odometry_session",
        qos_profile_sensor_data,
    )
    vision_publisher = harness.create_publisher(
        PoseStamped, f"{prefix}/vision", qos_profile_sensor_data
    )
    visible_publisher = harness.create_publisher(
        Bool, f"{prefix}/visible", qos_profile_sensor_data
    )
    statuses = []
    valid_values = []
    pool_odometry = []
    aligned_odometry = []
    harness.create_subscription(
        LocalizationStatus, f"{prefix}/status", statuses.append, latched_qos
    )
    harness.create_subscription(Bool, f"{prefix}/valid", valid_values.append, latched_qos)
    harness.create_subscription(
        Odometry,
        f"{prefix}/odometry_pool",
        pool_odometry.append,
        qos_profile_sensor_data,
    )
    harness.create_subscription(
        AlignedOdometry,
        f"{prefix}/aligned_odometry",
        aligned_odometry.append,
        qos_profile_sensor_data,
    )
    initialize_client = harness.create_client(
        InitializePool, "/brov/localization/initialize_pool"
    )
    reset_client = harness.create_client(Trigger, "/brov/localization/reset")
    tilt_client = harness.create_client(
        Trigger, "/brov/localization/confirm_camera_tilt_neutral"
    )

    executor.add_node(harness)
    executor.add_node(localization)
    try:
        _pump(
            executor,
            lambda: odometry_session_publisher.get_subscription_count() == 1
            and vision_publisher.get_subscription_count() == 1,
        )
        visible_publisher.publish(Bool(data=True))
        # A pre-confirmation pair must never enter the alignment sample set.
        pre_stamp = harness.get_clock().now().to_msg()
        pre_odometry = _odometry(pre_stamp)
        pre_vision = PoseStamped()
        pre_vision.header.stamp = pre_stamp
        pre_vision.header.frame_id = "test_pool"
        pre_vision.pose.orientation.w = 1.0
        odometry_session_publisher.publish(
            _envelope(pre_odometry, "session-a")
        )
        vision_publisher.publish(pre_vision)
        _pump(
            executor,
            lambda: statuses
            and statuses[-1].odometry_session_id == "session-a"
            and statuses[-1].sample_count == 0,
        )

        blocked_future = initialize_client.call_async(
            InitializePool.Request(min_samples=3)
        )
        _pump(executor, blocked_future.done)
        assert not blocked_future.result().success
        assert "tilt neutral is not confirmed" in blocked_future.result().message

        assert tilt_client.wait_for_service(timeout_sec=1.0)
        tilt_future = tilt_client.call_async(Trigger.Request())
        _pump(executor, tilt_future.done)
        assert tilt_future.result().success
        assert "all prior samples cleared" in tilt_future.result().message

        for _ in range(3):
            stamp = harness.get_clock().now().to_msg()
            odometry = _odometry(stamp)

            vision = PoseStamped()
            vision.header.stamp = stamp
            vision.header.frame_id = "test_pool"
            vision.pose.position.x = 1.2
            vision.pose.position.y = 0.7
            vision.pose.position.z = 0.4
            qx, qy, qz, qw = _quaternion_from_rpy(0.04, -0.03, 0.6)
            vision.pose.orientation.x = qx
            vision.pose.orientation.y = qy
            vision.pose.orientation.z = qz
            vision.pose.orientation.w = qw
            odometry_session_publisher.publish(
                _envelope(odometry, "session-a")
            )
            vision_publisher.publish(vision)
            _pump(
                executor,
                lambda: statuses
                and statuses[-1].sample_count >= _ + 1,
            )

        assert initialize_client.wait_for_service(timeout_sec=1.0)
        below_floor = initialize_client.call_async(
            InitializePool.Request(min_samples=2)
        )
        _pump(executor, below_floor.done)
        assert not below_floor.result().success
        assert "below configured safety floor 3" in below_floor.result().message

        request = InitializePool.Request()
        request.min_samples = 3
        future = initialize_client.call_async(request)
        _pump(executor, future.done)
        response = future.result()
        assert response.success
        assert response.epoch == 1
        _pump(
            executor,
            lambda: bool(pool_odometry)
            and bool(aligned_odometry)
            and valid_values[-1].data,
        )
        output = pool_odometry[-1]
        envelope = aligned_odometry[-1]
        assert envelope.odometry == output
        assert envelope.localization_epoch == response.epoch
        assert envelope.odometry_session_id == "session-a"
        assert envelope.alignment_id == statuses[-1].alignment_id
        assert output.header.frame_id == "test_pool"
        assert output.child_frame_id == "test_base"
        assert output.pose.pose.position.x == pytest.approx(1.2)
        assert output.pose.pose.position.y == pytest.approx(0.7)
        assert output.pose.pose.position.z == pytest.approx(0.4)
        assert statuses[-1].state == LocalizationStatus.INITIALIZED
        assert statuses[-1].output_valid is True
        assert valid_values[-1].data is statuses[-1].output_valid
        assert statuses[-1].sample_count == 3
        assert str(uuid.UUID(statuses[-1].alignment_id)) == statuses[-1].alignment_id
        assert statuses[-1].pool_to_odom.translation.x == pytest.approx(1.2)
        assert statuses[-1].pool_to_odom.translation.y == pytest.approx(0.7)
        assert statuses[-1].pool_to_odom.translation.z == pytest.approx(0.4)
        assert statuses[-1].pool_to_odom.rotation.x == pytest.approx(qx)
        assert statuses[-1].pool_to_odom.rotation.y == pytest.approx(qy)
        assert statuses[-1].pool_to_odom.rotation.z == pytest.approx(qz)
        assert statuses[-1].pool_to_odom.rotation.w == pytest.approx(qw)
        initialized_alignment_id = statuses[-1].alignment_id
        aligned_count = len(aligned_odometry)

        session_b_odometry = _odometry(harness.get_clock().now().to_msg())
        odometry_session_publisher.publish(
            _envelope(session_b_odometry, "session-b")
        )
        _pump(
            executor,
            lambda: statuses
            and statuses[-1].state == LocalizationStatus.INVALID
            and statuses[-1].odometry_session_id == "session-b"
            and statuses[-1].alignment_id == ""
            and valid_values
            and not valid_values[-1].data,
        )
        invalidated_epoch = statuses[-1].epoch
        assert statuses[-1].output_valid is False
        assert valid_values[-1].data is statuses[-1].output_valid
        assert invalidated_epoch > response.epoch
        assert statuses[-1].pool_to_odom.translation.x == 0.0
        assert statuses[-1].pool_to_odom.rotation.w == 1.0
        assert initialized_alignment_id
        assert len(aligned_odometry) == aligned_count

        session_blocked = initialize_client.call_async(
            InitializePool.Request(min_samples=3)
        )
        _pump(executor, session_blocked.done)
        assert not session_blocked.result().success
        assert "tilt neutral is not confirmed" in session_blocked.result().message

        assert reset_client.wait_for_service(timeout_sec=1.0)
        reset_future = reset_client.call_async(Trigger.Request())
        _pump(executor, reset_future.done)
        assert reset_future.result().success
        _pump(
            executor,
            lambda: statuses
            and statuses[-1].state == LocalizationStatus.UNINITIALIZED,
        )
        assert statuses[-1].epoch > invalidated_epoch
        reset_blocked = initialize_client.call_async(
            InitializePool.Request(min_samples=3)
        )
        _pump(executor, reset_blocked.done)
        assert not reset_blocked.result().success
        assert "tilt neutral is not confirmed" in reset_blocked.result().message
    finally:
        executor.remove_node(localization)
        executor.remove_node(harness)
        localization.destroy_node()
        harness.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
