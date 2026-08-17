"""Motor-free tests for legacy/MK2 policy artifact separation."""

from __future__ import annotations

import hashlib
import json

import pytest
import torch

from brov_control.policy_contract import (
    LEGACY_ACTION_CONTRACT,
    MK2_ACTION_CONTRACT,
    action_to_allocation_multiplier,
    resolve_policy_artifact_contract,
)


def _sha(path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mk2_bundle(tmp_path):
    policy = tmp_path / "policy_raw_flu_mk2.pt"
    policy.write_bytes(b"deterministic-mk2-policy")
    vehicle = tmp_path / "brov2_heavy.yaml"
    vehicle.write_text("name: brov2_heavy\n", encoding="utf-8")
    metadata = {
        "schema": "brov_torchscript_policy_v2",
        "policy_sha256": _sha(policy),
        "checkpoint_contract_verified": True,
        "profile": "deploy_v2",
        "observation_contract": "brov_velocity_observation_v2",
        "action_contract": MK2_ACTION_CONTRACT,
        "input_dim": 16,
        "output_dim": 6,
        "action_order": ["surge", "sway", "heave", "roll", "pitch", "yaw"],
        "wrench_scale": [85.0, 85.0, 120.0, 26.0, 14.0, 22.0],
        "policy_frame": "body_flu_zup",
        "allocation_frame": "body_frd_sname",
        "vehicle_model_sha256": _sha(vehicle),
    }
    sidecar = tmp_path / "policy_raw_flu_mk2.pt.metadata.json"
    sidecar.write_text(json.dumps(metadata), encoding="utf-8")
    return policy, sidecar, vehicle


def test_action_contract_basis_vectors_are_exact() -> None:
    assert torch.equal(
        action_to_allocation_multiplier(LEGACY_ACTION_CONTRACT),
        torch.ones(6),
    )
    assert torch.equal(
        action_to_allocation_multiplier(MK2_ACTION_CONTRACT),
        torch.tensor([1.0, -1.0, -1.0, 1.0, -1.0, -1.0]),
    )


def test_mk2_bundle_verifies_hash_frames_and_vehicle(tmp_path) -> None:
    policy, sidecar, vehicle = _mk2_bundle(tmp_path)

    contract = resolve_policy_artifact_contract(
        policy,
        requested_action_contract=MK2_ACTION_CONTRACT,
        metadata_path=sidecar,
        vehicle_model_path=vehicle,
    )

    assert contract.metadata_verified
    assert contract.profile == "deploy_v2"
    assert contract.action_contract == MK2_ACTION_CONTRACT


def test_mk2_bundle_accepts_deploy_v3_profile(tmp_path) -> None:
    policy, sidecar, vehicle = _mk2_bundle(tmp_path)
    payload = json.loads(sidecar.read_text(encoding="utf-8"))
    payload["profile"] = "deploy_v3"
    sidecar.write_text(json.dumps(payload), encoding="utf-8")

    contract = resolve_policy_artifact_contract(
        policy,
        requested_action_contract=MK2_ACTION_CONTRACT,
        metadata_path=sidecar,
        vehicle_model_path=vehicle,
    )

    assert contract.metadata_verified
    assert contract.profile == "deploy_v3"


def test_mk2_bundle_accepts_deploy_v4_profile(tmp_path) -> None:
    policy, sidecar, vehicle = _mk2_bundle(tmp_path)
    payload = json.loads(sidecar.read_text(encoding="utf-8"))
    payload["profile"] = "deploy_v4"
    sidecar.write_text(json.dumps(payload), encoding="utf-8")

    contract = resolve_policy_artifact_contract(
        policy,
        requested_action_contract=MK2_ACTION_CONTRACT,
        metadata_path=sidecar,
        vehicle_model_path=vehicle,
    )

    assert contract.metadata_verified
    assert contract.profile == "deploy_v4"


def test_mk2_missing_metadata_fails_closed(tmp_path) -> None:
    policy = tmp_path / "policy.pt"
    policy.write_bytes(b"no-sidecar")
    with pytest.raises(FileNotFoundError, match="requires policy metadata"):
        resolve_policy_artifact_contract(
            policy,
            requested_action_contract=MK2_ACTION_CONTRACT,
            vehicle_model_path=tmp_path / "vehicle.yaml",
        )


@pytest.mark.parametrize(
    ("field", "bad_value", "message"),
    [
        ("action_contract", LEGACY_ACTION_CONTRACT, "contract mismatch"),
        ("policy_sha256", "0" * 64, "policy SHA256"),
        ("vehicle_model_sha256", "0" * 64, "vehicle-model SHA256"),
        ("profile", "legacy_exact", "requires profile in"),
        ("output_dim", 8, "contract mismatch"),
    ],
)
def test_mk2_metadata_mismatch_fails_closed(
    tmp_path, field, bad_value, message
) -> None:
    policy, sidecar, vehicle = _mk2_bundle(tmp_path)
    payload = json.loads(sidecar.read_text(encoding="utf-8"))
    payload[field] = bad_value
    sidecar.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        resolve_policy_artifact_contract(
            policy,
            requested_action_contract=MK2_ACTION_CONTRACT,
            metadata_path=sidecar,
            vehicle_model_path=vehicle,
        )


def test_legacy_executable_requires_matching_verified_metadata(tmp_path) -> None:
    policy = tmp_path / "policy.pt"
    policy.write_bytes(b"legacy-model-299")
    sidecar = tmp_path / "policy.pt.metadata.json"
    sidecar.write_text(
        json.dumps(
            {
                "schema": "brov_torchscript_policy_v2",
                "policy_sha256": _sha(policy),
                "profile": "legacy_exact",
                "observation_contract": "brov_velocity_observation_v1",
                "action_contract": LEGACY_ACTION_CONTRACT,
                "input_dim": 16,
                "output_dim": 6,
            }
        ),
        encoding="utf-8",
    )
    contract = resolve_policy_artifact_contract(
        policy,
        requested_action_contract=LEGACY_ACTION_CONTRACT,
    )
    assert contract.metadata_verified
    assert contract.profile == "legacy_exact"


def test_legacy_missing_metadata_also_fails_closed(tmp_path) -> None:
    policy = tmp_path / "policy.pt"
    policy.write_bytes(b"legacy-model-299")
    with pytest.raises(FileNotFoundError, match="requires policy metadata"):
        resolve_policy_artifact_contract(
            policy,
            requested_action_contract=LEGACY_ACTION_CONTRACT,
        )
