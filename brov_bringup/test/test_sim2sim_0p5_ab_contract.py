"""Contracts for the Stage-1 Gazebo GT-vs-EKF comparison launch."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "sim2sim_0p5_ab.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("sim2sim_0p5_ab", LAUNCH_PATH)
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


def test_direct_stage1_mission_is_takeoff_then_single_0p5_leg() -> None:
    params = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mission_sim2sim_straight_0p5.yaml").read_text(
            encoding="utf-8"
        )
    )["brov_obs_node"]["ros__parameters"]

    assert params["waypoint_frame"] == "start_heading"
    assert params["heading_mode"] == "takeoff_then_align"
    assert params["loop"] is False
    assert params["cruise_speed"] == pytest.approx(0.50)
    assert params["depth_speed_limit"] == pytest.approx(0.05)
    assert params["waypoints"] == "0,0,0;0,0,0.20;5.0,0,0.20"
    assert params["waypoint_max_xyz"] == pytest.approx([5.0, 0.0, 0.20])


def test_launch_defaults_are_shadow_safe_and_mavlink_owned(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/share/{package}",
    )
    description = module.generate_launch_description()
    defaults = _defaults(description)

    assert defaults["feedback_source"] == "mavlink_ekf"
    assert defaults["connection"] == "udpin:0.0.0.0:14552"
    assert defaults["send_pwm"] == "false"
    assert defaults["arm"] == "false"

    bridge_nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and getattr(entity, "_Node__package", None) == "ros_gz_bridge"
    ]
    assert len(bridge_nodes) == 1
    assert getattr(bridge_nodes[0], "_Node__node_executable", None) == (
        "parameter_bridge"
    )


def test_sitl_vehicle_keeps_truth_as_an_explicit_non_default_source() -> None:
    vehicle = yaml.safe_load(
        (
            PACKAGE_ROOT.parent
            / "brov_base"
            / "config"
            / "vehicle_sitl.yaml"
        ).read_text(encoding="utf-8")
    )["brov_obs_node"]["ros__parameters"]

    assert vehicle["thruster_reversal_profile"] == "edo_sitl_identity"
    assert vehicle["feedback_source"] == "mavlink_ekf"
    assert vehicle["gazebo_truth_logging_enabled"] is False
    assert vehicle["send_pwm"] is False
    assert vehicle["arm"] is False
