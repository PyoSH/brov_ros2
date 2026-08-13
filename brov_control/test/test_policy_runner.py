"""TorchScript loading, shape, and command-boundary tests."""

from pathlib import Path

import pytest
import torch

from brov_control.policy_runner import PolicyRunner


class _SixAxisPolicy(torch.nn.Module):
    def forward(self, observation):
        return 2.0 * observation[:, :6]


def _export_policy(path: Path) -> None:
    model = torch.jit.trace(_SixAxisPolicy(), torch.zeros(1, 16))
    model.save(str(path))


def test_runner_loads_policy_and_clamps_single_observation(tmp_path):
    policy_path = tmp_path / "policy.pt"
    _export_policy(policy_path)
    runner = PolicyRunner(policy_path)
    observation = torch.zeros(16)
    observation[:6] = torch.tensor([1.0, -1.0, 0.25, -0.25, 0.0, 0.75])
    action = runner.act(observation)
    assert action.shape == (6,)
    assert torch.allclose(
        action,
        torch.tensor([1.0, -1.0, 0.5, -0.5, 0.0, 1.0]),
    )


def test_runner_preserves_batch_dimension(tmp_path):
    policy_path = tmp_path / "policy.pt"
    _export_policy(policy_path)
    action = PolicyRunner(policy_path).act(torch.zeros(3, 16))
    assert action.shape == (3, 6)


def test_missing_policy_fails_closed(tmp_path):
    with pytest.raises(FileNotFoundError):
        PolicyRunner(tmp_path / "missing.pt")

