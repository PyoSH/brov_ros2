"""TorchScript loading, shape, and clamp tests for the CMG OBS17->ACTION8 runner."""

from pathlib import Path

import numpy as np
import pytest
import torch

from cmg_deploy.core.policy_runner import PolicyRunner


class _EightAxisPolicy(torch.nn.Module):
    def forward(self, observation):
        return 2.0 * observation[:, :8]


def _export_policy(path: Path) -> None:
    model = torch.jit.trace(_EightAxisPolicy(), torch.zeros(1, 17))
    model.save(str(path))


def test_runner_loads_policy_and_clamps_single_observation(tmp_path):
    policy_path = tmp_path / "policy.pt"
    _export_policy(policy_path)
    runner = PolicyRunner(policy_path)
    observation = np.zeros(17, dtype=np.float32)
    observation[:8] = [1.0, -1.0, 0.25, -0.25, 0.0, 0.75, 0.9, -0.9]
    action = runner.infer(observation)
    assert action.shape == (8,)
    assert np.allclose(
        action,
        [1.0, -1.0, 0.5, -0.5, 0.0, 1.0, 1.0, -1.0],
    )


def test_runner_rejects_wrong_observation_shape(tmp_path):
    policy_path = tmp_path / "policy.pt"
    _export_policy(policy_path)
    runner = PolicyRunner(policy_path)
    with pytest.raises(ValueError):
        runner.infer(np.zeros(16, dtype=np.float32))


def test_missing_policy_fails_closed(tmp_path):
    with pytest.raises(FileNotFoundError):
        PolicyRunner(tmp_path / "missing.pt")
