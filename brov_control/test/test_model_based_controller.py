"""Motor-free regression tests for the explicit controller mathematics."""

import math

import torch

from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_control.model_based_controller import (
    ModelBasedController,
    quaternion_error_rotation_vector,
)


def _controller() -> ModelBasedController:
    position, direction = thruster_pos_dir_ned(load_brov2_yaml())
    return ModelBasedController(position, direction)


def _identity_observation() -> torch.Tensor:
    observation = torch.zeros(16)
    observation[0] = 1.0
    return observation


def test_identity_error_produces_neutral():
    output = _controller().compute(_identity_observation())
    assert torch.allclose(output.wrench_zup, torch.zeros(6), atol=1e-6)
    assert torch.allclose(output.pwm, torch.zeros(8), atol=1e-6)


def test_velocity_error_uses_negative_feedback_and_active_pwm_floor():
    observation = _identity_observation()
    observation[4:7] = torch.tensor([-0.1, 0.2, -0.3])
    output = _controller().compute(observation)
    assert torch.allclose(
        output.wrench_zup[:3], torch.tensor([2.5, -5.0, 10.5])
    )
    active = output.pwm != 0.0
    assert bool(active.any())
    assert bool((output.pwm[active].abs() >= 0.10).all())


def test_pitch_error_and_frame_transform_have_expected_signs():
    observation = torch.zeros(16)
    angle = math.radians(5.0)
    observation[0] = math.cos(angle / 2.0)
    observation[2] = math.sin(angle / 2.0)
    output = _controller().compute(observation)
    assert output.wrench_zup[4] < 0.0
    assert output.wrench_sname[4] > 0.0


def test_quaternion_sign_represents_the_same_rotation_error():
    quaternion = torch.tensor([0.9, 0.1, -0.2, 0.3])
    assert torch.allclose(
        quaternion_error_rotation_vector(quaternion),
        quaternion_error_rotation_vector(-quaternion),
        atol=1e-6,
    )


def test_invalid_observation_fails_without_actuation():
    try:
        _controller().compute(torch.zeros(15))
    except ValueError as error:
        assert "16-element" in str(error)
    else:
        raise AssertionError("invalid observation was accepted")

