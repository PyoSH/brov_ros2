import numpy as np
import pytest

from cmg_deploy.core.obs_builder import VehicleState, build_observation


def test_identity_contract():
    state = VehicleState(
        position_world=np.zeros(3),
        quaternion_wxyz=np.array([1, 0, 0, 0]),
        linear_velocity_body=np.ones(3),
        angular_velocity_body=np.ones(3),
        timestamp_s=0.0,
    )
    observation = build_observation([1, 2, 3], [1, 0, 0, 0], state)
    assert np.allclose(
        observation,
        [1, 0, 0, 0, 1, 2, 3, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1],
    )
    assert observation.shape == (17,)


def test_rejects_non_finite_state():
    state = VehicleState(
        position_world=np.array([np.nan, 0, 0]),
        quaternion_wxyz=np.array([1, 0, 0, 0]),
        linear_velocity_body=np.zeros(3),
        angular_velocity_body=np.zeros(3),
        timestamp_s=0.0,
    )
    with pytest.raises(ValueError):
        build_observation([0, 0, 0], [1, 0, 0, 0], state)
