"""Fail-closed policy artifact and action-frame contracts.

The legacy ``model_299`` policy was trained and deployed without an explicit
FLU/Z-up to SNAME/FRD transform.  New MK2 policies use a named, metadata-bound
contract so those two incompatible conventions cannot be mixed silently.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import torch


LEGACY_ACTION_CONTRACT = "legacy_model_299_no_t6"
MK2_ACTION_CONTRACT = "explicit_flu_zup_to_sname_frd_v1"
SUPPORTED_ACTION_CONTRACTS = (
    LEGACY_ACTION_CONTRACT,
    MK2_ACTION_CONTRACT,
)
MK2_OBSERVATION_CONTRACT = "brov_velocity_observation_v2"
# Training profiles that share the MK2 observation/action contract (same
# 16-D observation, same wrench-scaled 6-D action) and are therefore safe to
# deploy through policy_node_mk2.  Profiles differ in desired-state curriculum
# and reward shaping only -- neither affects this runtime contract.  Extend
# this tuple (never the single deploy_v2 literal) when a new profile is added
# in step_2_BROV/envs/vel_env_cfg.py with the same observation/action pair.
MK2_ACCEPTED_PROFILES = ("deploy_v2", "deploy_v3", "deploy_v4")
ACTION_ORDER = ("surge", "sway", "heave", "roll", "pitch", "yaw")
WRENCH_SCALE = (85.0, 85.0, 120.0, 26.0, 14.0, 22.0)

_ACTION_TO_ALLOCATION = {
    LEGACY_ACTION_CONTRACT: (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
    MK2_ACTION_CONTRACT: (1.0, -1.0, -1.0, 1.0, -1.0, -1.0),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class PolicyArtifactContract:
    policy_path: str
    policy_sha256: str
    action_contract: str
    observation_contract: str
    profile: str
    vehicle_model_sha256: str | None
    metadata_path: str | None
    metadata_verified: bool

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


def action_to_allocation_multiplier(
    action_contract: str,
    *,
    dtype: torch.dtype = torch.float32,
    device: str | torch.device = "cpu",
) -> torch.Tensor:
    """Return the six-axis policy-to-SNAME sign vector for one contract."""

    try:
        values = _ACTION_TO_ALLOCATION[action_contract]
    except KeyError as exc:
        raise ValueError(
            f"unknown policy_action_contract={action_contract!r}; expected "
            f"{SUPPORTED_ACTION_CONTRACTS}"
        ) from exc
    return torch.tensor(values, dtype=dtype, device=device)


def resolve_policy_artifact_contract(
    policy_path: str | Path,
    *,
    requested_action_contract: str,
    metadata_path: str | Path | None = None,
    vehicle_model_path: str | Path | None = None,
) -> PolicyArtifactContract:
    """Validate the policy hash and its named action/observation contract.

    A sibling ``<policy>.metadata.json`` is discovered automatically.  Both
    legacy and MK2 executables require checksum-bound metadata; the executable
    fixes the requested contract and the sidecar must agree with it.
    """

    policy = Path(policy_path).expanduser().resolve()
    if not policy.is_file():
        raise FileNotFoundError(f"TorchScript policy not found: {policy}")
    action_to_allocation_multiplier(requested_action_contract)

    explicit_metadata = bool(str(metadata_path or "").strip())
    metadata = (
        Path(metadata_path).expanduser().resolve()
        if explicit_metadata
        else Path(str(policy) + ".metadata.json")
    )
    policy_sha = _sha256(policy)

    if not metadata.is_file():
        raise FileNotFoundError(
            f"{requested_action_contract} requires policy metadata: {metadata}"
        )

    try:
        payload = json.loads(metadata.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid policy metadata JSON: {metadata}") from exc
    if not isinstance(payload, dict):
        raise ValueError("policy metadata must contain one JSON object")

    required = {
        "schema": "brov_torchscript_policy_v2",
        "input_dim": 16,
        "output_dim": 6,
        "action_contract": requested_action_contract,
    }
    mismatches = {
        key: {"metadata": payload.get(key), "required": expected}
        for key, expected in required.items()
        if payload.get(key) != expected
    }
    if mismatches:
        raise ValueError(f"policy metadata contract mismatch: {mismatches}")
    if payload.get("policy_sha256") != policy_sha:
        raise ValueError("policy SHA256 does not match policy metadata")

    observation_contract = str(payload.get("observation_contract", ""))
    if (
        requested_action_contract == MK2_ACTION_CONTRACT
        and observation_contract != MK2_OBSERVATION_CONTRACT
    ):
        raise ValueError(
            "MK2 policy requires observation_contract="
            f"{MK2_OBSERVATION_CONTRACT!r}; got {observation_contract!r}"
        )
    profile = str(payload.get("profile", ""))
    if (
        requested_action_contract == MK2_ACTION_CONTRACT
        and profile not in MK2_ACCEPTED_PROFILES
    ):
        raise ValueError(
            f"MK2 deployment requires profile in {MK2_ACCEPTED_PROFILES!r}; "
            f"got {profile!r}"
        )

    vehicle_sha = None
    if requested_action_contract == MK2_ACTION_CONTRACT:
        if payload.get("action_order") != list(ACTION_ORDER):
            raise ValueError(
                f"MK2 policy action_order must be {list(ACTION_ORDER)!r}"
            )
        if payload.get("wrench_scale") != list(WRENCH_SCALE):
            raise ValueError(
                f"MK2 policy wrench_scale must be {list(WRENCH_SCALE)!r}"
            )
        if payload.get("policy_frame") != "body_flu_zup":
            raise ValueError("MK2 policy_frame must be 'body_flu_zup'")
        if payload.get("allocation_frame") != "body_frd_sname":
            raise ValueError("MK2 allocation_frame must be 'body_frd_sname'")
        if payload.get("checkpoint_contract_verified") is not True:
            raise ValueError("MK2 checkpoint contract is not verified")
        if vehicle_model_path is None:
            raise ValueError("MK2 contract validation requires vehicle_model_path")
        vehicle_path = Path(vehicle_model_path).expanduser().resolve()
        if not vehicle_path.is_file():
            raise FileNotFoundError(f"vehicle model not found: {vehicle_path}")
        vehicle_sha = _sha256(vehicle_path)
        if payload.get("vehicle_model_sha256") != vehicle_sha:
            raise ValueError("vehicle-model SHA256 does not match policy metadata")

    return PolicyArtifactContract(
        policy_path=str(policy),
        policy_sha256=policy_sha,
        action_contract=requested_action_contract,
        observation_contract=observation_contract,
        profile=profile,
        vehicle_model_sha256=vehicle_sha,
        metadata_path=str(metadata),
        metadata_verified=True,
    )
