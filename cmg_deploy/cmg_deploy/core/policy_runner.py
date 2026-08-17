"""Small TorchScript policy runner with no training-stack dependencies.

Numpy in, numpy out (unlike brov_control's torch-tensor ``PolicyRunner``)
so ``core/`` stays independently unit-testable without rclpy or torch
tensors leaking into the observation-builder/action-mapper contract.
"""
from pathlib import Path

import numpy as np

from .contract import ACTION_DIM, OBS_DIM


class PolicyRunner:
    def __init__(self, policy_path, device: str = "cpu"):
        import torch

        self.torch = torch
        self.path = Path(policy_path).expanduser()
        if not self.path.is_file():
            raise FileNotFoundError(f"TorchScript policy not found: {self.path}")
        import hashlib

        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.device = device
        self.model = torch.jit.load(str(self.path), map_location=device)
        self.model.eval()

    def infer(self, obs):
        a = np.asarray(obs, dtype=np.float32)
        if a.shape == (OBS_DIM,):
            a = a[None, :]
        if a.shape != (1, OBS_DIM) or not np.isfinite(a).all():
            raise ValueError(f"policy requires finite ({OBS_DIM},) or (1,{OBS_DIM})")
        with self.torch.inference_mode():
            out = self.model(self.torch.from_numpy(a).to(self.device))
        out = out.detach().cpu().numpy().astype(np.float32).reshape(-1)
        if out.shape != (ACTION_DIM,) or not np.isfinite(out).all():
            raise ValueError(f"policy did not return finite ACTION{ACTION_DIM}")
        return np.clip(out, -1.0, 1.0)
