#!/usr/bin/env python3
"""
Explicit, one-shot alignment between continuous odometry and the pool frame.

This node never opens MAVLink and never commands the vehicle.  It combines a
time-paired raw pool pose with local odometry only while the vehicle is still,
then freezes the accepted ``pool -> odom`` transform until reset or until the
odometry session identity changes.
"""

from __future__ import annotations

from collections import deque
import copy
from dataclasses import dataclass
import math

import numpy as np
import rclpy
from brov_interfaces.msg import (
    AlignedOdometry,
    LocalizationStatus,
    OdometrySession,
)
from brov_interfaces.srv import InitializePool
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .alignment import (
    AlignmentSampleBuffer,
    TimedOdometry,
    TimedVisionPose,
    make_alignment_sample,
)
from .identity import AlignmentIdGenerator
from .math3d import (
    make_transform,
    matrix_to_quaternion_xyzw,
    rotate_pose_covariance,
    rotation_rpy_rad,
    validate_transform,
)


@dataclass(frozen=True)
class _OdometryRecord:
    message: Odometry
    sample: TimedOdometry


@dataclass(frozen=True)
class _VisionRecord:
    message: PoseStamped
    sample: TimedVisionPose


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_s(stamp) -> float:
    return _stamp_ns(stamp) / 1.0e9


def _pose_to_transform(pose: Pose) -> np.ndarray:
    return make_transform(
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    )


def _fill_pose(pose: Pose, transform: np.ndarray) -> None:
    value = validate_transform(transform)
    quaternion = matrix_to_quaternion_xyzw(value[:3, :3])
    pose.position.x = float(value[0, 3])
    pose.position.y = float(value[1, 3])
    pose.position.z = float(value[2, 3])
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])


def _fill_transform(message, transform: np.ndarray) -> None:
    value = validate_transform(transform)
    quaternion = matrix_to_quaternion_xyzw(value[:3, :3])
    message.translation.x = float(value[0, 3])
    message.translation.y = float(value[1, 3])
    message.translation.z = float(value[2, 3])
    message.rotation.x = float(quaternion[0])
    message.rotation.y = float(quaternion[1])
    message.rotation.z = float(quaternion[2])
    message.rotation.w = float(quaternion[3])


def _twist_speeds(message: Odometry) -> tuple[float, float]:
    linear = message.twist.twist.linear
    angular = message.twist.twist.angular
    linear_speed = float(np.linalg.norm([linear.x, linear.y, linear.z]))
    angular_speed = float(np.linalg.norm([angular.x, angular.y, angular.z]))
    if not math.isfinite(linear_speed) or not math.isfinite(angular_speed):
        raise ValueError("odometry twist contains non-finite values")
    return linear_speed, angular_speed


class PoolAlignmentNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "brov_pool_alignment", parameter_overrides=parameter_overrides
        )

        self.declare_parameter(
            "odometry_session_topic", "/brov/odometry/local_with_session"
        )
        self.declare_parameter("vision_topic", "/brov/aruco/robot_pose_pool")
        self.declare_parameter("visible_topic", "/brov/aruco/visible")
        self.declare_parameter(
            "pool_odometry_topic", "/brov/localization/odometry_pool"
        )
        self.declare_parameter(
            "aligned_odometry_topic",
            "/brov/localization/odometry_pool_with_alignment",
        )
        self.declare_parameter("status_topic", "/brov/localization/status")
        self.declare_parameter("valid_topic", "/brov/localization/valid")
        self.declare_parameter("pool_frame", "pool")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("default_min_samples", 20)
        self.declare_parameter("max_buffer_samples", 200)
        self.declare_parameter("sample_retention_s", 8.0)
        self.declare_parameter("max_message_age_s", 0.5)
        self.declare_parameter("max_future_offset_s", 0.05)
        self.declare_parameter("visible_timeout_s", 0.5)
        self.declare_parameter("max_timestamp_skew_s", 0.08)
        self.declare_parameter("stationary_linear_speed_mps", 0.03)
        self.declare_parameter("stationary_angular_speed_rad_s", 0.05)
        self.declare_parameter("max_translation_residual_m", 0.15)
        self.declare_parameter("max_rotation_residual_deg", 10.0)
        self.declare_parameter("max_abs_alignment_roll_deg", 15.0)
        self.declare_parameter("max_abs_alignment_pitch_deg", 15.0)
        self.declare_parameter(
            "require_camera_tilt_neutral_confirmation", True
        )
        self.declare_parameter("status_publish_period_s", 0.5)

        self._pool_frame = str(self.get_parameter("pool_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        if not self._pool_frame or not self._odom_frame or not self._base_frame:
            raise ValueError("pool_frame, odom_frame and base_frame must be non-empty")
        if len({self._pool_frame, self._odom_frame, self._base_frame}) != 3:
            raise ValueError("pool_frame, odom_frame and base_frame must be distinct")

        self._default_min_samples = int(
            self.get_parameter("default_min_samples").value
        )
        self._max_buffer_samples = int(
            self.get_parameter("max_buffer_samples").value
        )
        self._sample_retention_s = self._positive_parameter("sample_retention_s")
        self._max_message_age_s = self._positive_parameter("max_message_age_s")
        self._max_future_offset_s = self._nonnegative_parameter(
            "max_future_offset_s"
        )
        self._visible_timeout_s = self._positive_parameter("visible_timeout_s")
        self._max_timestamp_skew_s = self._nonnegative_parameter(
            "max_timestamp_skew_s"
        )
        self._stationary_linear_speed_mps = self._nonnegative_parameter(
            "stationary_linear_speed_mps"
        )
        self._stationary_angular_speed_rad_s = self._nonnegative_parameter(
            "stationary_angular_speed_rad_s"
        )
        self._max_translation_residual_m = self._positive_parameter(
            "max_translation_residual_m"
        )
        self._max_rotation_residual_rad = math.radians(
            self._positive_parameter("max_rotation_residual_deg")
        )
        self._max_abs_alignment_roll_rad = math.radians(
            self._positive_parameter("max_abs_alignment_roll_deg")
        )
        self._max_abs_alignment_pitch_rad = math.radians(
            self._positive_parameter("max_abs_alignment_pitch_deg")
        )
        self._require_camera_tilt_neutral_confirmation = bool(
            self.get_parameter("require_camera_tilt_neutral_confirmation").value
        )
        status_period = self._positive_parameter("status_publish_period_s")
        if self._default_min_samples <= 0:
            raise ValueError("default_min_samples must be positive")
        if self._max_buffer_samples < self._default_min_samples:
            raise ValueError(
                "max_buffer_samples must be at least default_min_samples"
            )

        self._samples = AlignmentSampleBuffer(
            max_samples=self._max_buffer_samples,
            retention_s=self._sample_retention_s,
        )
        self._odometry: deque[_OdometryRecord] = deque(
            maxlen=self._max_buffer_samples
        )
        self._pending_vision: deque[_VisionRecord] = deque(
            maxlen=self._max_buffer_samples
        )
        self._session_id = ""
        self._alignment: np.ndarray | None = None
        self._alignment_id = ""
        self._alignment_ids = AlignmentIdGenerator()
        self._alignment_sample_count = 0
        self._epoch = 0
        self._state = LocalizationStatus.UNINITIALIZED
        self._reason = "waiting for odometry session and synchronized samples"
        self._visible = False
        self._visible_rx_s = float("-inf")
        self._last_odom: _OdometryRecord | None = None
        self._camera_tilt_neutral_confirmed = (
            not self._require_camera_tilt_neutral_confirmation
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_odometry = self.create_publisher(
            Odometry,
            str(self.get_parameter("pool_odometry_topic").value),
            qos_profile_sensor_data,
        )
        self._pub_aligned_odometry = self.create_publisher(
            AlignedOdometry,
            str(self.get_parameter("aligned_odometry_topic").value),
            qos_profile_sensor_data,
        )
        self._pub_status = self.create_publisher(
            LocalizationStatus,
            str(self.get_parameter("status_topic").value),
            latched_qos,
        )
        self._pub_valid = self.create_publisher(
            Bool,
            str(self.get_parameter("valid_topic").value),
            latched_qos,
        )
        self._tf = TransformBroadcaster(self)

        self.create_subscription(
            OdometrySession,
            str(self.get_parameter("odometry_session_topic").value),
            self._on_odometry_session,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("vision_topic").value),
            self._on_vision,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("visible_topic").value),
            self._on_visible,
            qos_profile_sensor_data,
        )
        self.create_service(
            InitializePool,
            "/brov/localization/initialize_pool",
            self._on_initialize,
        )
        self.create_service(
            Trigger,
            "/brov/localization/reset",
            self._on_reset,
        )
        self.create_service(
            Trigger,
            "/brov/localization/confirm_camera_tilt_neutral",
            self._on_confirm_camera_tilt_neutral,
        )
        self.create_timer(status_period, self._publish_status)
        self._publish_status()
        self.get_logger().info(
            "pool alignment ready — collection only while stationary; "
            "explicit /brov/localization/initialize_pool required"
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _stamp_is_fresh(self, stamp) -> tuple[bool, str]:
        stamp_value_ns = _stamp_ns(stamp)
        if stamp_value_ns <= 0:
            return False, "zero acquisition timestamp"
        age = self._now_s() - stamp_value_ns / 1.0e9
        if age < -self._max_future_offset_s:
            return False, f"timestamp is {-age:.3f}s in the future"
        if age > self._max_message_age_s:
            return False, f"message is stale by {age:.3f}s"
        return True, ""

    def _visible_is_fresh(self) -> bool:
        return self._visible and (
            self._now_s() - self._visible_rx_s <= self._visible_timeout_s
        )

    def _set_state(self, state: int, reason: str) -> None:
        changed = self._state != state or self._reason != reason
        self._state = state
        self._reason = reason
        if changed:
            self._publish_status()

    def _publish_status(self) -> None:
        self._samples.prune(self._now_s())
        now = self.get_clock().now().to_msg()
        status = LocalizationStatus()
        status.header.stamp = now
        status.header.frame_id = self._pool_frame
        status.state = int(self._state)
        status.epoch = int(self._epoch)
        status.odometry_session_id = self._session_id
        status.alignment_id = self._alignment_id
        _fill_transform(
            status.pool_to_odom,
            self._alignment if self._alignment is not None else np.eye(4),
        )
        status.output_valid = self._is_output_valid()
        status.sample_count = int(
            self._alignment_sample_count
            if self._state == LocalizationStatus.INITIALIZED
            else len(self._samples)
        )
        status.reason = self._status_reason()
        self._pub_status.publish(status)
        self._pub_valid.publish(Bool(data=status.output_valid))

    def _status_reason(self) -> str:
        if self._state == LocalizationStatus.INITIALIZED and not self._odom_is_fresh():
            return f"{self._reason}; local odometry is stale or unavailable"
        return self._reason

    def _odom_is_fresh(self) -> bool:
        if self._last_odom is None:
            return False
        return self._stamp_is_fresh(self._last_odom.message.header.stamp)[0]

    def _is_output_valid(self) -> bool:
        return (
            self._state == LocalizationStatus.INITIALIZED
            and self._alignment is not None
            and bool(self._alignment_id)
            and bool(self._session_id)
            and self._odom_is_fresh()
        )

    def _clear_measurements(self) -> None:
        self._samples.clear()
        self._odometry.clear()
        self._pending_vision.clear()
        self._last_odom = None
        self._alignment_sample_count = 0

    def _invalidate(self, reason: str, *, state: int) -> None:
        self._alignment = None
        self._alignment_id = ""
        self._camera_tilt_neutral_confirmed = (
            not self._require_camera_tilt_neutral_confirmation
        )
        self._clear_measurements()
        self._epoch += 1
        self._set_state(state, reason)

    def _accept_session_id(self, session_id: str) -> bool:
        session_id = session_id.strip()
        if not session_id:
            self._session_id = ""
            self._invalidate(
                "received an empty odometry session id",
                state=LocalizationStatus.INVALID,
            )
            return False
        if not self._session_id:
            self._session_id = session_id
            self._clear_measurements()
            requirement = (
                "; camera tilt-neutral confirmation required"
                if self._require_camera_tilt_neutral_confirmation
                and not self._camera_tilt_neutral_confirmed
                else ""
            )
            self._set_state(
                LocalizationStatus.UNINITIALIZED,
                f"odometry session {session_id!r} accepted{requirement}",
            )
            return True
        if session_id == self._session_id:
            return True
        previous = self._session_id
        self._session_id = session_id
        self._invalidate(
            f"odometry session changed from {previous!r} to {session_id!r}",
            state=LocalizationStatus.INVALID,
        )
        return True

    def _on_visible(self, message: Bool) -> None:
        self._visible = bool(message.data)
        self._visible_rx_s = self._now_s()
        if not self._visible:
            self._pending_vision.clear()
            return
        self._try_pair_pending()

    def _parse_odometry(self, message: Odometry) -> _OdometryRecord:
        if message.header.frame_id != self._odom_frame:
            raise ValueError(
                f"odometry frame is {message.header.frame_id!r}, "
                f"expected {self._odom_frame!r}"
            )
        if message.child_frame_id != self._base_frame:
            raise ValueError(
                f"odometry child is {message.child_frame_id!r}, "
                f"expected {self._base_frame!r}"
            )
        fresh, reason = self._stamp_is_fresh(message.header.stamp)
        if not fresh:
            raise ValueError(reason)
        transform = _pose_to_transform(message.pose.pose)
        linear_speed, angular_speed = _twist_speeds(message)
        if not np.all(np.isfinite(np.asarray(message.pose.covariance, dtype=float))):
            raise ValueError("odometry pose covariance contains non-finite values")
        if not np.all(np.isfinite(np.asarray(message.twist.covariance, dtype=float))):
            raise ValueError("odometry twist covariance contains non-finite values")
        return _OdometryRecord(
            message=copy.deepcopy(message),
            sample=TimedOdometry(
                stamp_s=_stamp_s(message.header.stamp),
                transform_odom_base=transform,
                linear_speed_mps=linear_speed,
                angular_speed_rad_s=angular_speed,
            ),
        )

    def _parse_vision(self, message: PoseStamped) -> _VisionRecord:
        if message.header.frame_id != self._pool_frame:
            raise ValueError(
                f"vision frame is {message.header.frame_id!r}, "
                f"expected {self._pool_frame!r}"
            )
        fresh, reason = self._stamp_is_fresh(message.header.stamp)
        if not fresh:
            raise ValueError(reason)
        return _VisionRecord(
            message=copy.deepcopy(message),
            sample=TimedVisionPose(
                stamp_s=_stamp_s(message.header.stamp),
                transform_pool_base=_pose_to_transform(message.pose),
            ),
        )

    def _on_odometry_session(self, message: OdometrySession) -> None:
        """Consume session identity and its odometry as one DDS sample."""

        if not self._accept_session_id(message.odometry_session_id):
            return
        try:
            record = self._parse_odometry(message.odometry)
        except ValueError as exception:
            self.get_logger().warning(
                f"atomic local odometry rejected: {exception}"
            )
            return
        self._last_odom = record
        self._odometry.append(record)
        self._prune_pending()
        if self._state == LocalizationStatus.INITIALIZED:
            self._publish_aligned_odometry(record)
        else:
            self._try_pair_pending()

    def _on_vision(self, message: PoseStamped) -> None:
        try:
            record = self._parse_vision(message)
        except ValueError as exception:
            self.get_logger().warning(f"vision pose rejected: {exception}")
            return
        if self._state == LocalizationStatus.INITIALIZED:
            # One-shot means vision cannot silently move pool->odom after approval.
            return
        if not self._camera_tilt_neutral_confirmed:
            return
        self._pending_vision.append(record)
        self._prune_pending()
        self._try_pair_pending()

    def _prune_pending(self) -> None:
        now = self._now_s()
        self._odometry = deque(
            (
                record
                for record in self._odometry
                if -self._max_future_offset_s
                <= now - record.sample.stamp_s
                <= self._max_message_age_s
            ),
            maxlen=self._max_buffer_samples,
        )
        self._pending_vision = deque(
            (
                record
                for record in self._pending_vision
                if -self._max_future_offset_s
                <= now - record.sample.stamp_s
                <= self._max_message_age_s
            ),
            maxlen=self._max_buffer_samples,
        )
        self._samples.prune(now)

    def _try_pair_pending(self) -> None:
        if (
            self._state == LocalizationStatus.INITIALIZED
            or not self._session_id
            or not self._camera_tilt_neutral_confirmed
            or not self._visible_is_fresh()
        ):
            return
        self._prune_pending()
        if not self._pending_vision or not self._odometry:
            return

        now = self._now_s()
        remaining: deque[_VisionRecord] = deque(maxlen=self._max_buffer_samples)
        accepted = 0
        for vision in self._pending_vision:
            stationary_odometry = [
                record
                for record in self._odometry
                if record.sample.linear_speed_mps
                <= self._stationary_linear_speed_mps
                and record.sample.angular_speed_rad_s
                <= self._stationary_angular_speed_rad_s
            ]
            if not stationary_odometry:
                remaining.append(vision)
                continue
            nearest = min(
                stationary_odometry,
                key=lambda record: abs(
                    record.sample.stamp_s - vision.sample.stamp_s
                ),
            )
            skew = abs(nearest.sample.stamp_s - vision.sample.stamp_s)
            if skew > self._max_timestamp_skew_s:
                remaining.append(vision)
                continue
            try:
                sample = make_alignment_sample(
                    vision.sample,
                    nearest.sample,
                    collected_at_s=now,
                    max_timestamp_skew_s=self._max_timestamp_skew_s,
                    max_linear_speed_mps=self._stationary_linear_speed_mps,
                    max_angular_speed_rad_s=self._stationary_angular_speed_rad_s,
                )
            except ValueError as exception:
                self.get_logger().warning(f"alignment pair rejected: {exception}")
                continue
            if self._samples.add(sample, now_s=now):
                accepted += 1
        self._pending_vision = remaining
        if accepted:
            self._set_state(
                LocalizationStatus.COLLECTING,
                f"collected {len(self._samples)} synchronized stationary samples",
            )

    def _on_initialize(self, request, response):
        requested = int(request.min_samples)
        required = requested if requested > 0 else self._default_min_samples
        response.epoch = int(self._epoch)
        if self._state == LocalizationStatus.INITIALIZED:
            response.success = False
            response.message = "already initialized; reset explicitly before re-alignment"
            return response
        if requested > 0 and requested < self._default_min_samples:
            response.success = False
            response.message = (
                f"min_samples={requested} is below configured safety floor "
                f"{self._default_min_samples}"
            )
            return response
        if not self._session_id:
            response.success = False
            response.message = "odometry session id is unavailable"
            return response
        if not self._camera_tilt_neutral_confirmed:
            response.success = False
            response.message = (
                "camera tilt neutral is not confirmed; call "
                "/brov/localization/confirm_camera_tilt_neutral"
            )
            return response
        if not self._visible_is_fresh():
            response.success = False
            response.message = "target marker is not currently visible"
            return response
        if self._last_odom is None or not self._odom_is_fresh():
            response.success = False
            response.message = "fresh local odometry is unavailable"
            return response
        if (
            self._last_odom.sample.linear_speed_mps
            > self._stationary_linear_speed_mps
            or self._last_odom.sample.angular_speed_rad_s
            > self._stationary_angular_speed_rad_s
        ):
            response.success = False
            response.message = "vehicle is not currently stationary"
            return response
        if required > self._max_buffer_samples:
            response.success = False
            response.message = (
                f"min_samples={required} exceeds max_buffer_samples="
                f"{self._max_buffer_samples}"
            )
            return response

        now = self._now_s()
        self._samples.prune(now)
        if len(self._samples) < required:
            self._set_state(
                LocalizationStatus.COLLECTING,
                f"waiting for samples: {len(self._samples)}/{required}",
            )
            response.success = False
            response.message = self._reason
            return response
        try:
            estimate = self._samples.estimate(
                now_s=now,
                min_samples=required,
                max_translation_residual_m=self._max_translation_residual_m,
                max_rotation_residual_rad=self._max_rotation_residual_rad,
            )
            roll, pitch, _ = rotation_rpy_rad(estimate.transform[:3, :3])
            if abs(roll) > self._max_abs_alignment_roll_rad:
                raise ValueError(
                    f"alignment roll {math.degrees(roll):.2f}deg exceeds gate"
                )
            if abs(pitch) > self._max_abs_alignment_pitch_rad:
                raise ValueError(
                    f"alignment pitch {math.degrees(pitch):.2f}deg exceeds gate"
                )
        except ValueError as exception:
            self._set_state(
                LocalizationStatus.COLLECTING,
                f"initialization rejected: {exception}",
            )
            response.success = False
            response.message = self._reason
            return response

        inliers = int(np.count_nonzero(estimate.inlier_mask))
        self._alignment = estimate.transform.copy()
        self._alignment_id = self._alignment_ids.new()
        self._alignment_sample_count = inliers
        self._samples.clear()
        self._pending_vision.clear()
        self._epoch += 1
        self._set_state(
            LocalizationStatus.INITIALIZED,
            f"full-SE3 pool->odom initialized from {inliers} inliers",
        )
        response.success = True
        response.message = self._reason
        response.epoch = int(self._epoch)
        if self._last_odom is not None and self._odom_is_fresh():
            self._publish_aligned_odometry(self._last_odom)
        return response

    def _on_confirm_camera_tilt_neutral(self, _request, response):
        if self._state == LocalizationStatus.INITIALIZED:
            response.success = False
            response.message = (
                "alignment is already initialized; reset before changing the "
                "tilt-neutral confirmation"
            )
            return response
        if not self._session_id:
            response.success = False
            response.message = "odometry session id is unavailable"
            return response
        self._alignment = None
        self._alignment_id = ""
        self._clear_measurements()
        self._camera_tilt_neutral_confirmed = True
        self._set_state(
            LocalizationStatus.UNINITIALIZED,
            "camera tilt neutral confirmed; all prior samples cleared",
        )
        response.success = True
        response.message = self._reason
        return response

    def _on_reset(self, _request, response):
        self._alignment = None
        self._alignment_id = ""
        self._camera_tilt_neutral_confirmed = (
            not self._require_camera_tilt_neutral_confirmation
        )
        self._clear_measurements()
        self._epoch += 1
        self._set_state(
            LocalizationStatus.UNINITIALIZED,
            (
                "manual reset; camera tilt-neutral confirmation required"
                if self._require_camera_tilt_neutral_confirmation
                else "manual reset; waiting for synchronized stationary samples"
            ),
        )
        response.success = True
        response.message = "pool alignment reset"
        return response

    def _publish_aligned_odometry(self, record: _OdometryRecord) -> None:
        if not self._is_output_valid():
            return
        transform_pool_base = self._alignment @ record.sample.transform_odom_base
        output = Odometry()
        output.header.stamp = record.message.header.stamp
        output.header.frame_id = self._pool_frame
        output.child_frame_id = self._base_frame
        _fill_pose(output.pose.pose, transform_pool_base)
        covariance = rotate_pose_covariance(
            record.message.pose.covariance, self._alignment[:3, :3]
        )
        output.pose.covariance = covariance.reshape(-1).tolist()
        # nav_msgs/Odometry defines twist in child_frame_id.  Since the child
        # remains base_link, both twist and twist covariance are unchanged.
        output.twist = copy.deepcopy(record.message.twist)

        envelope = AlignedOdometry()
        envelope.odometry = output
        envelope.localization_epoch = int(self._epoch)
        envelope.odometry_session_id = self._session_id
        envelope.alignment_id = self._alignment_id
        self._pub_aligned_odometry.publish(envelope)
        self._pub_odometry.publish(output)

        transform = TransformStamped()
        transform.header.stamp = record.message.header.stamp
        transform.header.frame_id = self._pool_frame
        transform.child_frame_id = self._odom_frame
        quaternion = matrix_to_quaternion_xyzw(self._alignment[:3, :3])
        transform.transform.translation.x = float(self._alignment[0, 3])
        transform.transform.translation.y = float(self._alignment[1, 3])
        transform.transform.translation.z = float(self._alignment[2, 3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoolAlignmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
