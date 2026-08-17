"""Exact frozen G08R1 observation construction, reproduced from the CMG v3 controller."""
from dataclasses import dataclass

import numpy as np

from .contract import OBS_DIM
from .math_utils import normalize_quaternion_wxyz, quat_apply_inverse_wxyz


@dataclass
class VehicleState:
    position_world: np.ndarray
    quaternion_wxyz: np.ndarray
    linear_velocity_body: np.ndarray
    angular_velocity_body: np.ndarray
    timestamp_s: float


def build_observation(target_position_world, target_quaternion_wxyz, state: VehicleState):
    p = np.asarray(state.position_world, dtype=np.float32).reshape(3)
    q = normalize_quaternion_wxyz(state.quaternion_wxyz)
    tq = normalize_quaternion_wxyz(target_quaternion_wxyz)
    offset = quat_apply_inverse_wxyz(
        q, np.asarray(target_position_world, dtype=np.float32).reshape(3) - p
    )
    out = np.concatenate(
        (
            tq,
            offset,
            q,
            np.asarray(state.linear_velocity_body, dtype=np.float32).reshape(3),
            np.asarray(state.angular_velocity_body, dtype=np.float32).reshape(3),
        )
    ).astype(np.float32)
    if out.shape != (OBS_DIM,) or not np.isfinite(out).all():
        raise ValueError("invalid OBS17")
    return out


def observation_debug(obs):
    obs = np.asarray(obs, dtype=np.float32).reshape(OBS_DIM)
    return {
        "target_q_wxyz": obs[:4].tolist(),
        "target_offset_body_xyz": obs[4:7].tolist(),
        "current_q_wxyz": obs[7:11].tolist(),
        "body_linear_velocity_xyz": obs[11:14].tolist(),
        "body_angular_velocity_xyz": obs[14:17].tolist(),
    }
