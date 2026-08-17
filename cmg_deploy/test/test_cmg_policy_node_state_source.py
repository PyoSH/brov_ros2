"""The gazebo_truth_diagnostic state source must fail closed without the
explicit sim-only acknowledgement -- it exists only to isolate the policy
from MAVLink/EKF quality in Gazebo and must never silently activate."""

from pathlib import Path

import pytest
import rclpy
import torch

from cmg_deploy.cmg_policy_node import CmgPolicyNode


class _EightAxisPolicy(torch.nn.Module):
    def forward(self, observation):
        return torch.zeros(observation.shape[0], 8)


@pytest.fixture
def policy_path(tmp_path):
    path = tmp_path / "policy.pt"
    torch.jit.trace(_EightAxisPolicy(), torch.zeros(1, 17)).save(str(path))
    return str(path)


@pytest.fixture(autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _overrides(policy_path, **extra):
    values = {"policy_path": policy_path, "policy_sha256": ""}
    values.update(extra)
    return [
        rclpy.parameter.Parameter(name, value=value)
        for name, value in values.items()
    ]


def test_gazebo_truth_diagnostic_requires_explicit_ack(policy_path):
    with pytest.raises(ValueError, match="i_understand_gazebo_truth_is_sim_only"):
        node = CmgPolicyNode(
            parameter_overrides=_overrides(
                policy_path, state_source="gazebo_truth_diagnostic"
            )
        )
        node.destroy_node()


def test_gazebo_truth_diagnostic_starts_with_explicit_ack(policy_path):
    node = CmgPolicyNode(
        parameter_overrides=_overrides(
            policy_path,
            state_source="gazebo_truth_diagnostic",
            i_understand_gazebo_truth_is_sim_only=True,
        )
    )
    try:
        assert node._state_source == "gazebo_truth_diagnostic"
    finally:
        node.destroy_node()


def test_mavlink_ekf_is_the_default(policy_path):
    node = CmgPolicyNode(parameter_overrides=_overrides(policy_path))
    try:
        assert node._state_source == "mavlink_ekf"
    finally:
        node.destroy_node()


def test_invalid_state_source_rejected(policy_path):
    with pytest.raises(ValueError, match="state_source"):
        node = CmgPolicyNode(
            parameter_overrides=_overrides(policy_path, state_source="bogus")
        )
        node.destroy_node()
