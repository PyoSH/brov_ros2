"""Small TorchScript policy runner with no training-stack dependencies."""

from __future__ import annotations

from pathlib import Path

import torch


class PolicyRunner:
    """Load an exported TorchScript policy and run clamped inference."""

    def __init__(self, jit_path: str | Path, device: str = "cpu"):
        policy_path = Path(jit_path).expanduser()
        if not policy_path.is_file():
            raise FileNotFoundError(f"TorchScript policy not found: {policy_path}")
        self.device = device
        self._model = torch.jit.load(str(policy_path), map_location=device)
        self._model.eval()

    @torch.inference_mode()
    def act(self, observation: torch.Tensor) -> torch.Tensor:
        """Map one observation to a six-axis action clamped to ``[-1, 1]``."""
        if observation.dim() not in (1, 2):
            raise ValueError("observation must be a 1-D vector or a batch matrix")
        model_input = observation if observation.dim() == 2 else observation.unsqueeze(0)
        action = self._model(model_input.to(self.device))
        if action.dim() != 2 or action.shape[0] != model_input.shape[0]:
            raise ValueError(
                "policy output must be a batch matrix with the same batch size"
            )
        result = action.squeeze(0) if observation.dim() == 1 else action
        return result.clamp(-1.0, 1.0)
