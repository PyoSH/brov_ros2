#!/usr/bin/env python3
"""TorchScript policy inference node.

The node is deliberately computation-only: it subscribes to the observation,
publishes the six-axis policy action and maps that action to eight normalized
thruster commands. ``brov_base`` owns MAVLink and the actual PWM gateway.
"""

from __future__ import annotations

import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import (
    BROV2ThrusterModel,
    build_allocation_matrix,
)

from .policy_runner import PolicyRunner


# Must remain identical to the policy training environment's wrench limits.
_WRENCH_SCALE = torch.tensor([85.0, 85.0, 120.0, 26.0, 14.0, 22.0])
_ACTION_LABELS = ["surge", "sway ", "heave", "roll ", "pitch", "yaw  "]
_BAR_WIDTH = 20


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

    def __init__(self):
        super().__init__("brov_policy_node")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("vehicle_config", "")
        self.declare_parameter("obs_rate_hz", 25.0)
        self.declare_parameter("vis_rate_hz", 2.0)

        policy_path = str(self.get_parameter("policy_path").value)
        if not policy_path:
            raise ValueError(
                "policy_path parameter is required and must point to policy.pt"
            )
        obs_rate_hz = float(self.get_parameter("obs_rate_hz").value)
        vis_rate_hz = float(self.get_parameter("vis_rate_hz").value)
        if obs_rate_hz <= 0.0 or vis_rate_hz <= 0.0:
            raise ValueError("obs_rate_hz and vis_rate_hz must be positive")
        self._vis_every_n = max(1, round(obs_rate_hz / vis_rate_hz))
        self._obs_count = 0

        self.policy = PolicyRunner(policy_path, device="cpu")
        vehicle_config = str(self.get_parameter("vehicle_config").value)
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
        allocation_matrix = build_allocation_matrix(
            self.thruster._pos, self.thruster._dir
        )
        self.allocation_pinv = torch.linalg.pinv(allocation_matrix)

        self.pub_action = self.create_publisher(
            Float32MultiArray, "/brov/action", 10
        )
        self.pub_pwm = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 10
        )
        self.sub_obs = self.create_subscription(
            Float32MultiArray,
            "/brov/observation",
            self._on_observation,
            10,
        )
        self.get_logger().info(f"policy loaded: {policy_path}")

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

        desired_wrench = _WRENCH_SCALE * action
        desired_force = self.allocation_pinv @ desired_wrench
        pwm = self.thruster.inverse_thrust(
            desired_force.unsqueeze(0)
        ).squeeze(0)

        self.pub_action.publish(Float32MultiArray(data=action.tolist()))
        self.pub_pwm.publish(Float32MultiArray(data=pwm.tolist()))

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
