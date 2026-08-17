"""Explicit model-based PI/PD controller for the 16-D BROV observation.

The observation order is ``q_e(4), v_e_b(3), omega_b(3), z_v(3), z_q(3)``.
Errors use ``current - desired`` and are therefore stabilized with negative
feedback. Observation/controller vectors use FLU (Z-up), while the thruster
allocation matrix uses SNAME/FRD (Z-down); the conversion is explicit below.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from brov_base.vendor.thruster import (
    BROV2ThrusterModel,
    build_allocation_matrix,
)


_WRENCH_ZUP_TO_SNAME = torch.tensor([1.0, -1.0, -1.0, 1.0, -1.0, -1.0])
_WRENCH_NORMALIZATION = torch.tensor([85.0, 85.0, 120.0, 26.0, 14.0, 22.0])


@dataclass(frozen=True)
class ControllerOutput:
    """Controller output and diagnostic vectors for one observation."""

    wrench_zup: torch.Tensor
    wrench_sname: torch.Tensor
    estimated_wrench_zup: torch.Tensor
    normalized_action_zup: torch.Tensor
    thruster_force: torch.Tensor
    pwm: torch.Tensor


def _vec3(value, name: str, *, positive: bool = False) -> torch.Tensor:
    result = torch.as_tensor(value, dtype=torch.float32)
    if result.shape != (3,) or not torch.isfinite(result).all():
        raise ValueError(f"{name} must contain three finite values")
    if positive and bool((result <= 0.0).any()):
        raise ValueError(f"all {name} values must be positive")
    return result


def quaternion_error_rotation_vector(q_error: torch.Tensor) -> torch.Tensor:
    """Convert a ``[w, x, y, z]`` error quaternion to its shortest rotvec."""
    quaternion = torch.as_tensor(q_error, dtype=torch.float32)
    if quaternion.shape != (4,) or not torch.isfinite(quaternion).all():
        raise ValueError("q_error must be a finite four-element quaternion")
    norm = quaternion.norm()
    if float(norm) < 1e-6:
        raise ValueError("q_error norm is too close to zero")
    quaternion = quaternion / norm

    # q and -q represent the same attitude. The w>=0 hemisphere always yields
    # the shortest rotation and avoids a discontinuity in the controller.
    if float(quaternion[0]) < 0.0:
        quaternion = -quaternion
    vector_norm = quaternion[1:4].norm()
    if float(vector_norm) < 1e-7:
        return 2.0 * quaternion[1:4]
    angle = 2.0 * torch.atan2(vector_norm, quaternion[0].clamp_min(0.0))
    return quaternion[1:4] * (angle / vector_norm)


class ModelBasedController:
    """Body-frame velocity PI plus attitude/rate PI-D wrench controller."""

    def __init__(
        self,
        thruster_pos,
        thruster_dir,
        *,
        linear_kp=(25.0, 25.0, 35.0),
        linear_ki=(0.0, 0.0, 0.0),
        attitude_kp=(3.0, 3.0, 3.0),
        attitude_ki=(0.0, 0.0, 0.0),
        angular_kd=(1.5, 1.5, 1.0),
        force_limit=(15.0, 15.0, 20.0),
        torque_limit=(3.0, 3.0, 3.0),
        minimum_active_pwm=0.10,
        thruster_force_activation=0.25,
        device="cpu",
    ):
        self.device = device
        self.linear_kp = _vec3(linear_kp, "linear_kp").to(device)
        self.linear_ki = _vec3(linear_ki, "linear_ki").to(device)
        self.attitude_kp = _vec3(attitude_kp, "attitude_kp").to(device)
        self.attitude_ki = _vec3(attitude_ki, "attitude_ki").to(device)
        self.angular_kd = _vec3(angular_kd, "angular_kd").to(device)
        self.force_limit = _vec3(
            force_limit, "force_limit", positive=True
        ).to(device)
        self.torque_limit = _vec3(
            torque_limit, "torque_limit", positive=True
        ).to(device)
        self.minimum_active_pwm = float(minimum_active_pwm)
        self.thruster_force_activation = float(thruster_force_activation)
        if not 0.075 < self.minimum_active_pwm <= 1.0:
            raise ValueError(
                "minimum_active_pwm must exceed the T200 deadband (0.075) "
                "and be no greater than 1"
            )
        if self.thruster_force_activation < 0.0:
            raise ValueError("thruster_force_activation must be non-negative")

        self.thruster = BROV2ThrusterModel(
            num_envs=1,
            dt=0.04,
            device=device,
            pos=thruster_pos,
            dir=thruster_dir,
        )
        self.allocation_matrix = build_allocation_matrix(
            self.thruster._pos, self.thruster._dir
        )
        self.allocation_pinv = torch.linalg.pinv(self.allocation_matrix)
        self._wrench_transform = _WRENCH_ZUP_TO_SNAME.to(device)
        self._normalization = _WRENCH_NORMALIZATION.to(device)

    def compute(self, observation: torch.Tensor) -> ControllerOutput:
        """Calculate the wrench, individual thrust, and normalized PWM."""
        obs = torch.as_tensor(
            observation, dtype=torch.float32, device=self.device
        )
        if obs.shape != (16,) or not torch.isfinite(obs).all():
            raise ValueError("observation must be a finite 16-element vector")

        q_error = obs[0:4]
        velocity_error = obs[4:7]
        angular_velocity = obs[7:10]
        velocity_integral = obs[10:13]
        attitude_integral = obs[13:16]
        rotation_error = quaternion_error_rotation_vector(q_error).to(
            self.device
        )

        force_zup = (
            -self.linear_kp * velocity_error
            - self.linear_ki * velocity_integral
        )
        torque_zup = (
            -self.attitude_kp * rotation_error
            - self.angular_kd * angular_velocity
            - self.attitude_ki * attitude_integral
        )
        force_zup = torch.maximum(
            torch.minimum(force_zup, self.force_limit), -self.force_limit
        )
        torque_zup = torch.maximum(
            torch.minimum(torque_zup, self.torque_limit), -self.torque_limit
        )
        wrench_zup = torch.cat((force_zup, torque_zup))

        wrench_sname = wrench_zup * self._wrench_transform
        desired_force = self.allocation_pinv @ wrench_sname
        # Keep diagnostics and commands inside the same realizable T200 range.
        desired_force = desired_force.clamp(-51.5, 64.1)
        pwm = self.thruster.inverse_thrust(
            desired_force.unsqueeze(0)
        ).squeeze(0)

        # The T200 model produces exactly zero thrust in |PWM|<=0.075. Suppress
        # numerical noise and lift active commands above that deadband.
        active = desired_force.abs() >= self.thruster_force_activation
        pwm_sign = torch.sign(desired_force)
        active_floor = torch.tensor(
            self.minimum_active_pwm, device=self.device
        )
        pwm = torch.where(
            active,
            pwm_sign * torch.maximum(pwm.abs(), active_floor),
            torch.zeros_like(pwm),
        ).clamp(-1.0, 1.0)

        estimated_force, estimated_torque = self.thruster.compute(
            pwm.unsqueeze(0)
        )
        estimated_wrench_zup = torch.cat(
            (estimated_force.squeeze(0), estimated_torque.squeeze(0))
        )

        return ControllerOutput(
            wrench_zup=wrench_zup,
            wrench_sname=wrench_sname,
            estimated_wrench_zup=estimated_wrench_zup,
            normalized_action_zup=(
                wrench_zup / self._normalization
            ).clamp(-1.0, 1.0),
            thruster_force=desired_force,
            pwm=pwm,
        )
