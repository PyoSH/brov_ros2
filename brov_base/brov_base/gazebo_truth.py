"""Gazebo ENU/FLU ground truth to MAVLink-style NED/FRD conversion.

The Edo Gazebo ``OdometryPublisher`` publishes pose in its world ENU frame
and twist in the ``base_link`` FLU frame.  ``ObservationBuilder`` consumes a
body-FRD to world-NED quaternion, world-NED linear velocity, and FRD angular
rates.  Keeping this conversion in a ROS-independent module makes the frame
contract directly testable before ground truth is ever allowed to drive the
policy.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Sequence

import torch

from brov_base import math_utils as mu


_SQRT_HALF = 2.0**-0.5
_Q_NED_FROM_ENU = torch.tensor(
    [0.0, _SQRT_HALF, _SQRT_HALF, 0.0], dtype=torch.float32
)
_Q_FLU_FROM_FRD = torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float32)
_FLU_TO_FRD = torch.tensor([1.0, -1.0, -1.0], dtype=torch.float32)


@dataclass(frozen=True)
class GazeboTruthKinematics:
    """One Gazebo truth sample expressed in MAVLink navigation conventions."""

    position_ned: torch.Tensor
    attitude_quat_ned_frd_wxyz: torch.Tensor
    linear_velocity_ned: torch.Tensor
    angular_rate_frd_proxy: torch.Tensor


class GazeboTruthBuffer:
    """Thread-safe, fail-closed buffer for bridged Gazebo Odometry messages.

    Receipt monotonic time is used only for freshness.  Gazebo's header stamp
    is retained as the policy integration time, so a slow or jittery bridge
    cannot silently change the simulated dynamics seen by the observation
    integrator.
    """

    def __init__(
        self,
        *,
        expected_frame: str = "odom",
        expected_child_frame: str = "base_link",
        quaternion_norm_tolerance: float = 0.1,
    ) -> None:
        self.expected_frame = expected_frame
        self.expected_child_frame = expected_child_frame
        if (
            not math.isfinite(quaternion_norm_tolerance)
            or quaternion_norm_tolerance <= 0.0
        ):
            raise ValueError("quaternion_norm_tolerance must be finite and positive")
        self.quaternion_norm_tolerance = float(quaternion_norm_tolerance)
        self._lock = threading.Lock()
        self._snapshot: dict | None = None
        self._last_source_time_s: float | None = None
        self._last_quaternion: torch.Tensor | None = None
        self._sequence = 0
        self._invalid_reason: str | None = None

    @property
    def invalid_reason(self) -> str | None:
        with self._lock:
            return self._invalid_reason

    def update(self, message, *, receive_time: float | None = None) -> bool:
        """Validate and store one ``nav_msgs/Odometry``-compatible message.

        Returns ``False`` for an exact duplicate source stamp.  A malformed
        frame or a backward source clock latches an invalid state; callers
        must restart the node rather than silently falling back to EKF input.
        """

        receive_time = time.monotonic() if receive_time is None else receive_time
        try:
            if message.header.frame_id != self.expected_frame:
                raise ValueError(
                    f"Gazebo truth frame {message.header.frame_id!r} != "
                    f"{self.expected_frame!r}"
                )
            if message.child_frame_id != self.expected_child_frame:
                raise ValueError(
                    f"Gazebo truth child frame {message.child_frame_id!r} != "
                    f"{self.expected_child_frame!r}"
                )
            stamp = message.header.stamp
            source_time_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            if not torch.isfinite(torch.tensor(source_time_s)) or source_time_s < 0.0:
                raise ValueError("Gazebo truth source stamp must be finite and non-negative")

            pose = message.pose.pose
            twist = message.twist.twist
            converted = gazebo_enu_flu_to_ned_frd(
                (pose.position.x, pose.position.y, pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                (twist.linear.x, twist.linear.y, twist.linear.z),
                (twist.angular.x, twist.angular.y, twist.angular.z),
                quaternion_norm_tolerance=self.quaternion_norm_tolerance,
            )
        except (AttributeError, TypeError, ValueError) as error:
            with self._lock:
                self._invalid_reason = str(error)
            raise

        with self._lock:
            if self._invalid_reason is not None:
                raise ValueError(self._invalid_reason)
            if self._last_source_time_s is not None:
                if source_time_s < self._last_source_time_s:
                    self._invalid_reason = (
                        "Gazebo truth source clock moved backward "
                        f"({source_time_s:.9f}s < {self._last_source_time_s:.9f}s)"
                    )
                    raise ValueError(self._invalid_reason)
                if source_time_s == self._last_source_time_s:
                    return False

            quaternion = converted.attitude_quat_ned_frd_wxyz
            if (
                self._last_quaternion is not None
                and float(torch.dot(quaternion, self._last_quaternion)) < 0.0
            ):
                quaternion = -quaternion
            previous_quaternion = self._last_quaternion
            previous_source_time_s = self._last_source_time_s
            self._last_quaternion = quaternion.clone()
            self._last_source_time_s = source_time_s
            self._sequence += 1
            if previous_quaternion is None or previous_source_time_s is None:
                return True
            body_rates_frd = body_angular_velocity_from_quaternions(
                previous_quaternion,
                quaternion,
                source_time_s - previous_source_time_s,
            )
            self._snapshot = {
                "att_quat_ned": quaternion.clone(),
                "body_rates_ned": body_rates_frd,
                "pos_ned": converted.position_ned.clone(),
                "vel_ned": converted.linear_velocity_ned.clone(),
                "att_rx_time": source_time_s,
                "pos_rx_time": source_time_s,
                "feedback_rx_monotonic": float(receive_time),
                "feedback_source_time_s": source_time_s,
                "att_seq": self._sequence,
                "pos_seq": self._sequence,
            }
            return True

    def snapshot(self, *, now: float | None = None) -> dict | None:
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._snapshot is None:
                return None
            result = {
                key: value.clone() if isinstance(value, torch.Tensor) else value
                for key, value in self._snapshot.items()
            }
        age = float(now) - float(result["feedback_rx_monotonic"])
        result["att_age_s"] = age
        result["pos_age_s"] = age
        return result


def _vector(value: Sequence[float] | torch.Tensor, size: int, name: str) -> torch.Tensor:
    result = torch.as_tensor(value, dtype=torch.float32)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not torch.isfinite(result).all():
        raise ValueError(f"{name} must contain only finite values")
    return result


def body_angular_velocity_from_quaternions(
    previous_wxyz: Sequence[float] | torch.Tensor,
    current_wxyz: Sequence[float] | torch.Tensor,
    dt: float,
) -> torch.Tensor:
    """Estimate body-frame angular velocity from two body-to-world poses."""

    previous = _vector(previous_wxyz, 4, "previous_wxyz")
    current = _vector(current_wxyz, 4, "current_wxyz")
    if not torch.isfinite(torch.tensor(dt)) or dt <= 0.0:
        raise ValueError("quaternion derivative dt must be finite and positive")
    previous_norm = torch.linalg.vector_norm(previous)
    current_norm = torch.linalg.vector_norm(current)
    if (
        float(previous_norm) <= torch.finfo(previous.dtype).eps
        or float(current_norm) <= torch.finfo(current.dtype).eps
    ):
        raise ValueError("quaternion derivative inputs must have positive norm")
    previous = previous / previous_norm
    current = current / current_norm
    delta = mu.quat_unique(mu.quat_mul(mu.quat_conjugate(previous), current))
    vector = delta[1:4]
    magnitude = torch.linalg.vector_norm(vector)
    if float(magnitude) <= torch.finfo(vector.dtype).eps:
        return 2.0 * vector / float(dt)
    angle = 2.0 * torch.atan2(magnitude, delta[0].clamp_min(0.0))
    return vector / magnitude * angle / float(dt)


def gazebo_enu_flu_to_ned_frd(
    position_enu: Sequence[float] | torch.Tensor,
    orientation_enu_flu_xyzw: Sequence[float] | torch.Tensor,
    linear_velocity_body_flu: Sequence[float] | torch.Tensor,
    angular_velocity_body_flu: Sequence[float] | torch.Tensor,
    *,
    quaternion_norm_tolerance: float = 0.1,
) -> GazeboTruthKinematics:
    """Convert one Gazebo odometry sample to NED-world / FRD-body state.

    The fixed world mapping is ``[N, E, D] = [y_gz, x_gz, -z_gz]``.  The
    body mapping is ``[forward, right, down] = [x, -y, -z]``.  Gazebo's
    odometry twist is expressed in the child/body frame, so its linear
    velocity is first rotated into ENU world and then mapped to NED.
    """

    position = _vector(position_enu, 3, "position_enu")
    orientation_xyzw = _vector(
        orientation_enu_flu_xyzw, 4, "orientation_enu_flu_xyzw"
    )
    linear_body = _vector(
        linear_velocity_body_flu, 3, "linear_velocity_body_flu"
    )
    angular_body = _vector(
        angular_velocity_body_flu, 3, "angular_velocity_body_flu"
    )

    orientation_wxyz = orientation_xyzw[[3, 0, 1, 2]]
    norm = torch.linalg.vector_norm(orientation_wxyz)
    if float(norm) <= torch.finfo(orientation_wxyz.dtype).eps:
        raise ValueError("orientation quaternion norm must be positive")
    if (
        not math.isfinite(quaternion_norm_tolerance)
        or quaternion_norm_tolerance <= 0.0
    ):
        raise ValueError("quaternion_norm_tolerance must be finite and positive")
    if abs(float(norm) - 1.0) > quaternion_norm_tolerance:
        raise ValueError(
            "orientation quaternion norm outside tolerance "
            f"({float(norm):.6f}, tolerance={quaternion_norm_tolerance:.6f})"
        )
    orientation_wxyz = orientation_wxyz / norm

    q_ned_from_enu = _Q_NED_FROM_ENU.to(orientation_wxyz)
    q_flu_from_frd = _Q_FLU_FROM_FRD.to(orientation_wxyz)
    attitude_ned_frd = mu.quat_unique(
        mu.quat_mul(
            mu.quat_mul(q_ned_from_enu, orientation_wxyz),
            q_flu_from_frd,
        )
    )

    world_velocity_enu = mu.quat_apply(orientation_wxyz, linear_body)
    position_ned = position[[1, 0, 2]] * position.new_tensor((1.0, 1.0, -1.0))
    linear_velocity_ned = world_velocity_enu[[1, 0, 2]] * position.new_tensor(
        (1.0, 1.0, -1.0)
    )
    # Gazebo 7's OdometryPublisher exposes differentiated RPY values in
    # ``twist.angular``, not a body gyro.  Preserve the basis-converted value
    # only as a diagnostic proxy; GazeboTruthBuffer derives the policy's body
    # angular velocity from consecutive quaternions and source timestamps.
    angular_rate_frd_proxy = angular_body * _FLU_TO_FRD.to(angular_body)

    return GazeboTruthKinematics(
        position_ned=position_ned,
        attitude_quat_ned_frd_wxyz=attitude_ned_frd,
        linear_velocity_ned=linear_velocity_ned,
        angular_rate_frd_proxy=angular_rate_frd_proxy,
    )
