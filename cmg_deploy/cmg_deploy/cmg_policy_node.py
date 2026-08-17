#!/usr/bin/env python3
"""CMG hover-policy node: OBS17 -> TorchScript -> ACTION8 -> /brov/thruster_pwm.

This node is deliberately computation-only, mirroring
``brov_control/policy_node.py``: it never opens MAVLink and never computes
a PWM microsecond value or a thruster reversal sign itself. By default
(``state_source: mavlink_ekf``) it consumes brov_base's already-validated
MAVLink/EKF odometry (``/brov/odometry/local_with_session``); see the
``gazebo_truth_diagnostic`` alternative below. Either way it publishes a
normalized ``[-1, 1]`` 8-vector to the shared ``/brov/thruster_pwm`` topic
only while
``/brov/control_active`` is true. brov_base's PWM gateway is the single
owner of PWM scaling, the real-vehicle T2/T3/T8 reversal mask, arming, and
the actual RC_CHANNELS_OVERRIDE transport -- exactly as it already is for
``policy_node``/``policy_node_mk2``/``model_based_controller_node``. Only
one of those controllers may run at a time; brov_base enforces exactly one
publisher on ``/brov/thruster_pwm``.

The hover target is self-contained (``TargetManager``, ``HOVER_ORIGIN`` by
default): it does not consume brov_mission's waypoint/guidance stack at
all, so no mission file or resolved mission is required for this
controller specifically (``base.launch.py`` still needs *some*
``mission_file`` to boot ``obs_node``, but its content is irrelevant here).

``state_source: gazebo_truth_diagnostic`` is a second, Gazebo-only state
input for isolating the policy from MAVLink/EKF quality: it reads the raw
Gazebo bridge Odometry (default ``/brov/sim/gazebo_odometry_raw``)
directly instead of ``/brov/odometry/local_with_session``.
``obs_node._publish_odometry`` always builds that topic from the
MAVLink/EKF snapshot regardless of ``feedback_source``
(``obs_node.py:2248``), so it cannot be used to separate "the policy is
wrong" from "the EKF estimate is wrong/slow" -- this second path exists
for exactly that isolation and nothing else. The Gazebo bridge publishes
pose in world ENU (Z-up, so no NED conversion is needed here the way
``obs_node``'s own ``GazeboTruthBuffer`` needs one to match MAVLink's
convention) and twist already in body-FLU, matching this policy's OBS17
contract directly. This path does not exist on the real vehicle and must
never be selected there; it requires the explicit
``i_understand_gazebo_truth_is_sim_only`` acknowledgement to start.
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from brov_interfaces.msg import OdometrySession

from .core.action_mapper import clip_action
from .core.contract import ACTION_DIM, POLICY_SHA256
from .core.obs_builder import VehicleState, build_observation
from .core.policy_runner import PolicyRunner
from .core.target_manager import TargetManager

_LOG_EVERY_N = 50


def _finite_limit_vector(values, *, length: int, name: str) -> np.ndarray:
    result = np.asarray(list(values), dtype=np.float32)
    if result.shape != (length,):
        raise ValueError(f"{name} must contain exactly {length} values")
    if not np.isfinite(result).all() or bool((result < 0.0).any()):
        raise ValueError(f"{name} must contain finite non-negative values")
    return result


def _limit_step(requested, previous, *, absolute_limit: float, max_delta):
    bounded = np.clip(requested, -absolute_limit, absolute_limit)
    if max_delta is None:
        return bounded
    delta = np.clip(bounded - previous, -max_delta, max_delta)
    return previous + delta


class CmgPolicyNode(Node):
    _NODE_NAME = "cmg_policy_node"

    def __init__(self, **node_kwargs):
        super().__init__(self._NODE_NAME, **node_kwargs)
        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_sha256", POLICY_SHA256)
        self.declare_parameter("hover_mode", "HOVER_ORIGIN")
        self.declare_parameter("relative_target_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("target_quaternion_mode", "INITIAL")
        self.declare_parameter("action_abs_limit", [1.0] * ACTION_DIM)
        self.declare_parameter("pwm_abs_limit", 1.0)
        self.declare_parameter("pwm_slew_rate_per_s", 0.0)
        self.declare_parameter("obs_rate_hz", 25.0)
        self.declare_parameter("log_every_n", _LOG_EVERY_N)
        self.declare_parameter("state_source", "mavlink_ekf")
        self.declare_parameter(
            "gazebo_truth_topic", "/brov/sim/gazebo_odometry_raw"
        )
        self.declare_parameter("i_understand_gazebo_truth_is_sim_only", False)

        policy_path = str(self.get_parameter("policy_path").value)
        if not policy_path:
            raise ValueError(
                "policy_path parameter is required and must point to policy.pt"
            )
        expected_sha256 = str(self.get_parameter("policy_sha256").value)
        hover_mode = str(self.get_parameter("hover_mode").value)
        if hover_mode not in ("HOVER_ORIGIN", "RELATIVE_TARGET"):
            raise ValueError("hover_mode must be HOVER_ORIGIN or RELATIVE_TARGET")
        target_quaternion_mode = str(
            self.get_parameter("target_quaternion_mode").value
        )
        if target_quaternion_mode not in ("INITIAL", "LEVEL"):
            raise ValueError("target_quaternion_mode must be INITIAL or LEVEL")
        self._relative_target_xyz = tuple(
            float(v)
            for v in self.get_parameter("relative_target_xyz").value
        )
        if len(self._relative_target_xyz) != 3:
            raise ValueError("relative_target_xyz must contain exactly 3 values")
        self._hover_mode = hover_mode
        self._target_quaternion_mode = target_quaternion_mode

        self._action_abs_limit = _finite_limit_vector(
            self.get_parameter("action_abs_limit").value,
            length=ACTION_DIM,
            name="action_abs_limit",
        )
        if bool((self._action_abs_limit > 1.0).any()):
            raise ValueError("action_abs_limit values must not exceed 1.0")
        self._pwm_abs_limit = float(self.get_parameter("pwm_abs_limit").value)
        if not np.isfinite(self._pwm_abs_limit) or not 0.0 < self._pwm_abs_limit <= 1.0:
            raise ValueError("pwm_abs_limit must be finite and in (0, 1]")
        pwm_slew_rate_per_s = float(self.get_parameter("pwm_slew_rate_per_s").value)
        if not np.isfinite(pwm_slew_rate_per_s) or pwm_slew_rate_per_s < 0.0:
            raise ValueError("pwm_slew_rate_per_s must be finite and non-negative")
        obs_rate_hz = float(self.get_parameter("obs_rate_hz").value)
        if obs_rate_hz <= 0.0:
            raise ValueError("obs_rate_hz must be positive")
        self._pwm_max_delta = (
            None if pwm_slew_rate_per_s == 0.0 else pwm_slew_rate_per_s / obs_rate_hz
        )
        self._log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        state_source = str(self.get_parameter("state_source").value)
        if state_source not in ("mavlink_ekf", "gazebo_truth_diagnostic"):
            raise ValueError(
                "state_source must be mavlink_ekf or gazebo_truth_diagnostic"
            )
        if state_source == "gazebo_truth_diagnostic" and not bool(
            self.get_parameter("i_understand_gazebo_truth_is_sim_only").value
        ):
            raise ValueError(
                "state_source=gazebo_truth_diagnostic requires the explicit "
                "i_understand_gazebo_truth_is_sim_only:=true acknowledgement; "
                "this path does not exist on the real vehicle"
            )
        self._state_source = state_source
        gazebo_truth_topic = str(self.get_parameter("gazebo_truth_topic").value)

        self.policy = PolicyRunner(policy_path)
        if expected_sha256 and self.policy.sha256 != expected_sha256:
            raise ValueError(
                "policy SHA-256 mismatch: expected "
                f"{expected_sha256}, loaded {self.policy.sha256} from {policy_path}"
            )
        self.get_logger().info(
            f"loaded CMG hover policy: {policy_path} sha256={self.policy.sha256} "
            f"hover_mode={hover_mode}"
        )

        self._control_active = False
        self._discard_next_active = False
        self._last_odometry_session_id = ""
        self._last_sent_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._target_manager = TargetManager(
            mode=self._hover_mode,
            relative_xyz=self._relative_target_xyz,
            target_q_mode=self._target_quaternion_mode,
        )
        self._obs_count = 0

        self.pub_action_raw = self.create_publisher(
            Float32MultiArray, "/cmg/policy/action_raw", 10
        )
        self.pub_action = self.create_publisher(
            Float32MultiArray, "/cmg/policy/action", 10
        )
        self.pub_preview = self.create_publisher(
            Float32MultiArray, "/cmg/policy/thruster_pwm_preview", 10
        )
        self.pub_target = self.create_publisher(
            Float32MultiArray, "/cmg/policy/target", 10
        )
        self.pub_pwm = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 10
        )

        if state_source == "mavlink_ekf":
            self.sub_odometry = self.create_subscription(
                OdometrySession,
                "/brov/odometry/local_with_session",
                self._on_odometry_session,
                1,
            )
        else:
            self.sub_odometry = self.create_subscription(
                Odometry, gazebo_truth_topic, self._on_gazebo_truth_odometry, 1
            )
            self.get_logger().warn(
                "DIAGNOSTIC-ONLY state_source=gazebo_truth_diagnostic active "
                f"(topic={gazebo_truth_topic}); sim-only, never use this on "
                "the real vehicle"
            )
        self.sub_active = self.create_subscription(
            Bool, "/brov/control_active", self._on_control_active, 1
        )
        self.get_logger().info(
            f"CMG hover policy ready (state_source={state_source}); awaiting "
            "odometry. Actual PWM waits for /brov/control_active=true"
        )

    def _on_control_active(self, message: Bool) -> None:
        active = bool(message.data)
        if active and not self._control_active:
            self._discard_next_active = True
            self._last_sent_action = np.zeros(ACTION_DIM, dtype=np.float32)
            # Re-latch the hover target at this instant rather than at node
            # startup, so shadow-mode motion before START does not become
            # the frozen setpoint.
            self._target_manager = TargetManager(
                mode=self._hover_mode,
                relative_xyz=self._relative_target_xyz,
                target_q_mode=self._target_quaternion_mode,
            )
        if not active:
            self._discard_next_active = False
            self._last_sent_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._control_active = active

    def _on_odometry_session(self, message: OdometrySession) -> None:
        session_id = message.odometry_session_id
        if (
            self._control_active
            and self._last_odometry_session_id
            and session_id != self._last_odometry_session_id
        ):
            self.get_logger().warn(
                "odometry session changed while active "
                f"({self._last_odometry_session_id} -> {session_id}); "
                "re-latching hover target defensively"
            )
            self._target_manager = TargetManager(
                mode=self._hover_mode,
                relative_xyz=self._relative_target_xyz,
                target_q_mode=self._target_quaternion_mode,
            )
        self._last_odometry_session_id = session_id
        self._process_state(message.odometry)

    def _on_gazebo_truth_odometry(self, message: Odometry) -> None:
        # No session concept on the raw Gazebo bridge topic; the
        # control-active edge re-latch in _on_control_active is the only
        # re-latch defense in this diagnostic path.
        self._process_state(message)

    def _process_state(self, odom: Odometry) -> None:
        position = np.array(
            [
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        orientation = odom.pose.pose.orientation
        quaternion_wxyz = np.array(
            [orientation.w, orientation.x, orientation.y, orientation.z],
            dtype=np.float32,
        )
        linear_velocity_body = np.array(
            [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z],
            dtype=np.float32,
        )
        angular_velocity_body = np.array(
            [
                odom.twist.twist.angular.x,
                odom.twist.twist.angular.y,
                odom.twist.twist.angular.z,
            ],
            dtype=np.float32,
        )
        stamp = odom.header.stamp
        state = VehicleState(
            position_world=position,
            quaternion_wxyz=quaternion_wxyz,
            linear_velocity_body=linear_velocity_body,
            angular_velocity_body=angular_velocity_body,
            timestamp_s=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        )

        try:
            target_position, target_quaternion = self._target_manager.update(
                position, quaternion_wxyz
            )
            observation = build_observation(target_position, target_quaternion, state)
            raw_action = self.policy.infer(observation)
        except ValueError as error:
            self.get_logger().error(f"ignoring invalid policy step: {error}")
            return

        limited_action = clip_action(
            np.clip(raw_action, -self._action_abs_limit, self._action_abs_limit)
        )
        pwm = _limit_step(
            limited_action,
            self._last_sent_action,
            absolute_limit=self._pwm_abs_limit,
            max_delta=self._pwm_max_delta,
        )

        self.pub_action_raw.publish(Float32MultiArray(data=raw_action.tolist()))
        self.pub_action.publish(Float32MultiArray(data=limited_action.tolist()))
        self.pub_preview.publish(Float32MultiArray(data=pwm.tolist()))
        self.pub_target.publish(
            Float32MultiArray(data=target_position.tolist() + target_quaternion.tolist())
        )

        if self._control_active and self._discard_next_active:
            self._discard_next_active = False
        elif self._control_active:
            self.pub_pwm.publish(Float32MultiArray(data=pwm.tolist()))
            self._last_sent_action = pwm.copy()

        self._obs_count += 1
        if self._obs_count % self._log_every_n == 0:
            locked = "locked" if self._target_manager.origin is not None else "unlocked"
            self.get_logger().info(
                f"target {locked} action_norm={float(np.linalg.norm(limited_action)):.2f} "
                f"active={self._control_active}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmgPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
