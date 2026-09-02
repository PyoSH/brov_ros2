"""Vehicle configuration and thruster API regression tests."""

import pytest
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


def test_thruster_force_clamp_matches_inverse_envelope():
    """클램프는 **현재 전압의 테이블 한계**를 따른다 — 전 전압 포락선이 아니다.

    2026-09-02 수조 세션에서 "추진기 한계 불일치" 로 기록된 혼선의 정리:
    세 숫자가 전부 실재하고 의미가 다르다.
      * ±51.5/64.1 N — **제거된 다항식 모델**의 포락선. 이 시험의 구판이
        단언하던 값이고, 어디에도 더는 존재하지 않는다.
      * −49.4/+65.9 N — ``force_limits_n``: 전 전압 테이블 최소/최대.
        보상 정규화 전용이며 실제 클램프가 아니다 (구 base_node 로그가
        이 값을 찍어 현장을 오도했다 — 로그는 고쳤다).
      * −36.7/+47.2 N @14.8 V — ``clamp_thrust`` 가 실제로 자르는 값.
        전압에 따라 움직인다 (12.6 V 에서 −30.3/+38.8, 16.8 V 에서 −41.9/+54.5).
    """
    model = BROV2ThrusterModel(num_envs=1, dt=0.04, device="cpu")

    # 실제 클램프 = 공칭 14.8 V 테이블 한계. 큰 요청은 여기서 잘려야 한다.
    requested = torch.tensor([[-100.0, -36.0, -1.0, 0.0, 1.0, 47.0, 100.0, 0.0]])
    limited = model.clamp_thrust(requested)
    assert limited.tolist()[0] == pytest.approx(
        [-36.66, -36.0, -1.0, 0.0, 1.0, 47.0, 47.21, 0.0], abs=0.01
    )

    # 전 전압 포락선(force_limits_n)은 클램프보다 넓다 — 둘을 혼동하면 안 된다.
    lo_env, hi_env = model.force_limits_n
    assert lo_env < float(limited.min()) and hi_env > float(limited.max())
    assert (lo_env, hi_env) == pytest.approx((-49.38, 65.92), abs=0.01)
