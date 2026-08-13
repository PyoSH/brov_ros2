"""Vehicle configuration and thruster API regression tests."""

import torch

from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix


def test_packaged_vehicle_yaml_has_eight_thrusters():
    params = load_brov2_yaml()
    positions, directions = thruster_pos_dir_ned(params)

    assert len(positions) == 8
    assert len(directions) == 8
    assert all(len(vector) == 3 for vector in positions + directions)


def test_packaged_thruster_configuration_builds_allocation_matrix():
    params = load_brov2_yaml()
    positions, directions = thruster_pos_dir_ned(params)
    allocation = build_allocation_matrix(
        torch.tensor(positions, dtype=torch.float32),
        torch.tensor(directions, dtype=torch.float32),
    )

    assert allocation.shape == (6, 8)
    assert torch.isfinite(allocation).all()
    assert torch.linalg.matrix_rank(allocation) == 6


def test_thruster_model_accepts_packaged_geometry():
    params = load_brov2_yaml()
    positions, directions = thruster_pos_dir_ned(params)
    model = BROV2ThrusterModel(
        num_envs=1,
        dt=0.04,
        device="cpu",
        pos=positions,
        dir=directions,
    )

    forces, torques = model.compute(torch.zeros((1, 8)))
    assert torch.equal(forces, torch.zeros((1, 3)))
    assert torch.equal(torques, torch.zeros((1, 3)))
