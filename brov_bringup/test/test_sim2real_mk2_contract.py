"""Non-hardware contracts for the real-vehicle MK2 RL launch path."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "sim2real_demo.launch.py"
CONTROL_ROOT = PACKAGE_ROOT.parent / "brov_control"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("sim2real_demo_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _defaults(description) -> dict[str, str]:
    context = LaunchContext()
    declarations = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    return {
        name: perform_substitutions(context, declaration.default_value)
        for name, declaration in declarations.items()
    }


def _controller_choices(description) -> list[str]:
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == "controller":
            return list(entity.choices)
    raise AssertionError("controller argument not declared")


def test_controller_accepts_rl_mk2(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(PACKAGE_ROOT) if package == "brov_bringup"
        else str(CONTROL_ROOT) if package == "brov_control"
        else f"/share/{package}",
    )
    description = module.generate_launch_description()
    assert _controller_choices(description) == ["model", "rl", "rl_mk2"]


def test_rl_mk2_defaults_to_conservative_real_envelope(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(PACKAGE_ROOT) if package == "brov_bringup"
        else str(CONTROL_ROOT) if package == "brov_control"
        else f"/share/{package}",
    )
    defaults = _defaults(module.generate_launch_description())
    rl_mk2_config = Path(defaults["rl_mk2_config"])
    assert rl_mk2_config.name == "rl_controller_mk2_real_v1.yaml"
    assert rl_mk2_config != CONTROL_ROOT / "config" / "rl_controller_mk2_deploy_v2.yaml"


def test_rl_mk2_real_envelope_is_conservative() -> None:
    document = yaml.safe_load(
        (CONTROL_ROOT / "config" / "rl_controller_mk2_real_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    profile = document["brov_policy_node"]["ros__parameters"]

    limits = profile["action_abs_limit"]
    assert len(limits) == 6
    assert all(0.0 < float(value) < 1.0 for value in limits)
    assert profile["pwm_abs_limit"] == pytest.approx(0.35)
    assert 0.0 < profile["pwm_abs_limit"] < 1.0
    assert profile["pwm_slew_rate_per_s"] == pytest.approx(0.40)
    assert profile["pwm_slew_rate_per_s"] > 0.0


def test_rl_mk2_real_envelope_is_strictly_tighter_than_sitl_deploy_v2() -> None:
    real = yaml.safe_load(
        (CONTROL_ROOT / "config" / "rl_controller_mk2_real_v1.yaml").read_text(
            encoding="utf-8"
        )
    )["brov_policy_node"]["ros__parameters"]
    sitl = yaml.safe_load(
        (CONTROL_ROOT / "config" / "rl_controller_mk2_deploy_v2.yaml").read_text(
            encoding="utf-8"
        )
    )["brov_policy_node"]["ros__parameters"]

    assert all(
        float(r) < float(s)
        for r, s in zip(real["action_abs_limit"], sitl["action_abs_limit"])
    )
    assert float(real["pwm_abs_limit"]) < float(sitl["pwm_abs_limit"])
