#!/usr/bin/env python3
"""
TorchScript policy inference node.

The node is deliberately computation-only: it subscribes to the observation,
publishes the six-axis policy action and a continuously inspectable PWM preview,
then forwards commands to the hardware-facing topic only while base control is
active. ``brov_base`` owns MAVLink and the actual PWM gateway.
"""

from __future__ import annotations

from pathlib import Path

import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from brov_base.vendor import params as vehicle_params
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import (
    BROV2ThrusterModel,
    build_allocation_matrix,
)

from .policy_runner import PolicyRunner
from .policy_contract import (
    LEGACY_ACTION_CONTRACT,
    WRENCH_SCALE,
    action_to_allocation_multiplier,
    resolve_policy_artifact_contract,
)


# Must remain identical to the policy training environment's wrench limits.
_WRENCH_SCALE = torch.tensor(WRENCH_SCALE)
_ACTION_LABELS = ["surge", "sway ", "heave", "roll ", "pitch", "yaw  "]
_BAR_WIDTH = 20


def _finite_limit_vector(values, *, length: int, name: str) -> torch.Tensor:
    """Validate one non-negative per-axis operational limit vector."""
    result = torch.as_tensor(list(values), dtype=torch.float32)
    if result.shape != (length,):
        raise ValueError(f"{name} must contain exactly {length} values")
    if not torch.isfinite(result).all() or bool((result < 0.0).any()):
        raise ValueError(f"{name} must contain finite non-negative values")
    return result


def _limit_pwm_step(
    requested: torch.Tensor,
    previous: torch.Tensor,
    *,
    absolute_limit: float,
    max_delta: float | None,
) -> torch.Tensor:
    """Apply the case profile's absolute and per-sample PWM envelope."""
    bounded = requested.clamp(-absolute_limit, absolute_limit)
    if max_delta is None:
        return bounded
    delta = (bounded - previous).clamp(-max_delta, max_delta)
    return previous + delta


def _action_bar(label: str, value: float, width: int = _BAR_WIDTH) -> str:
    """Render one normalized action around a centered zero marker."""
    clipped = max(-1.0, min(1.0, value))
    midpoint = width // 2
    filled = int(round(abs(clipped) * midpoint))
    characters = ["-"] * width
    characters[midpoint] = "|"
    if clipped >= 0.0:
        for index in range(filled):
            characters[midpoint + index] = "#"
    else:
        for index in range(filled):
            characters[midpoint - 1 - index] = "#"
    return f"{label} [{''.join(characters)}] {value:+.2f}"


class PolicyNode(Node):
    """Run the exported end-to-end policy for each valid observation."""

    _NODE_NAME = "brov_policy_node"
    _POLICY_ACTION_CONTRACT = LEGACY_ACTION_CONTRACT

    def __init__(self):
        super().__init__(self._NODE_NAME)
        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_metadata_path", "")
        self.declare_parameter("vehicle_config", "")
        self.declare_parameter("obs_rate_hz", 25.0)
        self.declare_parameter("vis_rate_hz", 2.0)
        self.declare_parameter("action_abs_limit", [1.0] * 6)
        self.declare_parameter("pwm_abs_limit", 1.0)
        # Zero preserves the exported policy's legacy behavior.  Safety-
        # critical profiles such as pool random-attitude v2 must explicitly
        # provide a positive rate.
        self.declare_parameter("pwm_slew_rate_per_s", 0.0)

        policy_path = str(self.get_parameter("policy_path").value)
        if not policy_path:
            raise ValueError(
                "policy_path parameter is required and must point to policy.pt"
            )
        metadata_path = str(self.get_parameter("policy_metadata_path").value)
        obs_rate_hz = float(self.get_parameter("obs_rate_hz").value)
        vis_rate_hz = float(self.get_parameter("vis_rate_hz").value)
        if obs_rate_hz <= 0.0 or vis_rate_hz <= 0.0:
            raise ValueError("obs_rate_hz and vis_rate_hz must be positive")
        self._action_abs_limit = _finite_limit_vector(
            self.get_parameter("action_abs_limit").value,
            length=6,
            name="action_abs_limit",
        )
        if bool((self._action_abs_limit > 1.0).any()):
            raise ValueError("action_abs_limit values must not exceed 1.0")
        self._pwm_abs_limit = float(
            self.get_parameter("pwm_abs_limit").value
        )
        self._pwm_slew_rate_per_s = float(
            self.get_parameter("pwm_slew_rate_per_s").value
        )
        if (
            not torch.isfinite(torch.tensor(self._pwm_abs_limit))
            or not 0.0 < self._pwm_abs_limit <= 1.0
        ):
            raise ValueError("pwm_abs_limit must be finite and in (0, 1]")
        if (
            not torch.isfinite(torch.tensor(self._pwm_slew_rate_per_s))
            or self._pwm_slew_rate_per_s < 0.0
        ):
            raise ValueError(
                "pwm_slew_rate_per_s must be finite and non-negative"
            )
        self._pwm_max_delta = (
            None
            if self._pwm_slew_rate_per_s == 0.0
            else self._pwm_slew_rate_per_s / obs_rate_hz
        )
        self._last_sent_pwm = torch.zeros(8, dtype=torch.float32)
        self._vis_every_n = max(1, round(obs_rate_hz / vis_rate_hz))
        self._obs_count = 0
        # Fail closed until an explicit current-session signal is received.
        # /brov/control_active is intentionally volatile, and base republishes
        # it continuously while running.
        self._control_active = False
        self._discard_next_active_observation = False

        vehicle_config = str(self.get_parameter("vehicle_config").value)
        vehicle_model_path = (
            vehicle_config
            if vehicle_config
            else str(Path(vehicle_params.__file__).with_name("brov2_heavy.yaml"))
        )
        self._policy_artifact = resolve_policy_artifact_contract(
            policy_path,
            requested_action_contract=self._POLICY_ACTION_CONTRACT,
            metadata_path=metadata_path,
            vehicle_model_path=vehicle_model_path,
        )
        self._policy_to_allocation = action_to_allocation_multiplier(
            self._POLICY_ACTION_CONTRACT
        )
        self.policy = PolicyRunner(policy_path, device="cpu")
        yaml_params = (
            load_brov2_yaml(vehicle_config)
            if vehicle_config
            else load_brov2_yaml()
        )
        thruster_pos, thruster_dir = thruster_pos_dir_ned(yaml_params)
        self.thruster = BROV2ThrusterModel(
            num_envs=1,
            dt=1.0 / obs_rate_hz,
            device="cpu",
            pos=thruster_pos,
            dir=thruster_dir,
        )
        self.allocation_matrix = build_allocation_matrix(
            self.thruster._pos, self.thruster._dir
        )
        self.allocation_pinv = torch.linalg.pinv(self.allocation_matrix)

        self.pub_action_raw = self.create_publisher(
            Float32MultiArray, "/brov/policy/action_raw", 10
        )
        self.pub_action = self.create_publisher(
            Float32MultiArray, "/brov/action", 10
        )
        self.pub_wrench_requested = self.create_publisher(
            Float32MultiArray, "/brov/policy/wrench_requested", 10
        )
        self.pub_thruster_force_requested = self.create_publisher(
            Float32MultiArray, "/brov/policy/thruster_force_requested", 10
        )
        self.pub_thruster_force_limited = self.create_publisher(
            Float32MultiArray, "/brov/policy/thruster_force_limited", 10
        )
        self.pub_wrench_after_thruster_limit = self.create_publisher(
            Float32MultiArray,
            "/brov/policy/wrench_after_thruster_limit",
            10,
        )
        self.pub_pwm_requested = self.create_publisher(
            Float32MultiArray, "/brov/policy/thruster_pwm_requested", 10
        )
        self.pub_pwm = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 10
        )
        self.pub_preview = self.create_publisher(
            Float32MultiArray,
            "/brov/policy/thruster_pwm_preview",
            10,
        )
        artifact_qos = QoSProfile(depth=1)
        artifact_qos.reliability = ReliabilityPolicy.RELIABLE
        artifact_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_artifact_contract = self.create_publisher(
            String,
            "/brov/policy/artifact_contract",
            artifact_qos,
        )
        self._artifact_contract_message = String(
            data=self._policy_artifact.to_json()
        )
        self._publish_artifact_contract()
        # Re-publish at a low rate as well as using transient-local durability.
        # This guarantees that generic rosbag/CLI volatile subscribers still
        # capture deployment provenance even if discovery completes after the
        # node's initial publication.
        self._artifact_contract_timer = self.create_timer(
            1.0, self._publish_artifact_contract
        )
        self.sub_obs = self.create_subscription(
            Float32MultiArray,
            "/brov/observation",
            self._on_observation,
            1,
        )
        self.sub_active = self.create_subscription(
            Bool,
            "/brov/control_active",
            self._on_control_active,
            1,
        )
        self.get_logger().info(
            f"policy loaded in preview mode: {policy_path}; profile="
            f"{self._policy_artifact.profile} action_contract="
            f"{self._policy_artifact.action_contract} metadata_verified="
            f"{self._policy_artifact.metadata_verified}; actual PWM waits for "
            "/brov/control_active=true"
        )

    def _publish_artifact_contract(self) -> None:
        self.pub_artifact_contract.publish(self._artifact_contract_message)

    def _on_control_active(self, message: Bool) -> None:
        active = bool(message.data)
        if active and not self._control_active:
            # Observation and enable are different ROS topics.  A depth-one
            # queue plus one discarded sample keeps an in-flight preview from
            # becoming a live command after the enable edge.
            self._discard_next_active_observation = True
            self._last_sent_pwm.zero_()
        if not active:
            self._discard_next_active_observation = False
            self._last_sent_pwm.zero_()
        self._control_active = active

    def _on_observation(self, message: Float32MultiArray) -> None:
        if len(message.data) != 16:
            self.get_logger().warn(
                f"ignoring observation dimension {len(message.data)}; expected 16"
            )
            return
        observation = torch.tensor(message.data, dtype=torch.float32)
        action = self.policy.act(observation)
        if action.shape != (6,) or not torch.isfinite(action).all():
            self.get_logger().error(
                f"ignoring invalid policy output shape {tuple(action.shape)}"
            )
            return

        limited_action = torch.maximum(
            torch.minimum(action, self._action_abs_limit),
            -self._action_abs_limit,
        )
        # Policy output is FLU/Z-up for MK2, while B/B+ is SNAME/FRD.  The
        # cached multiplier is identity only for the explicitly named legacy
        # model_299 contract.
        desired_wrench = (
            _WRENCH_SCALE * limited_action * self._policy_to_allocation
        )
        desired_force = self.allocation_pinv @ desired_wrench
        limited_force = self.thruster.clamp_thrust(desired_force)
        wrench_after_thruster_limit = self.allocation_matrix @ limited_force
        requested_pwm = self.thruster.inverse_thrust(
            desired_force.unsqueeze(0)
        ).squeeze(0)
        pwm = _limit_pwm_step(
            requested_pwm,
            self._last_sent_pwm,
            absolute_limit=self._pwm_abs_limit,
            max_delta=self._pwm_max_delta,
        )

        self.pub_action_raw.publish(Float32MultiArray(data=action.tolist()))
        self.pub_action.publish(Float32MultiArray(data=limited_action.tolist()))
        self.pub_wrench_requested.publish(
            Float32MultiArray(data=desired_wrench.tolist())
        )
        self.pub_thruster_force_requested.publish(
            Float32MultiArray(data=desired_force.tolist())
        )
        self.pub_thruster_force_limited.publish(
            Float32MultiArray(data=limited_force.tolist())
        )
        self.pub_wrench_after_thruster_limit.publish(
            Float32MultiArray(data=wrench_after_thruster_limit.tolist())
        )
        self.pub_pwm_requested.publish(
            Float32MultiArray(data=requested_pwm.tolist())
        )
        self.pub_preview.publish(Float32MultiArray(data=pwm.tolist()))
        if self._control_active and self._discard_next_active_observation:
            self._discard_next_active_observation = False
        elif self._control_active:
            self.pub_pwm.publish(Float32MultiArray(data=pwm.tolist()))
            self._last_sent_pwm = pwm.detach().clone()

        self._obs_count += 1
        if self._obs_count % self._vis_every_n == 0:
            lines = [
                _action_bar(label, value.item())
                for label, value in zip(_ACTION_LABELS, action)
            ]
            self.get_logger().info("action:\n" + "\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
