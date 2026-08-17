"""Non-hardware contracts for pool-localized Sim2Swim bringup."""

from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "sim2swim_demo.launch.py"
RUNBOOK = PACKAGE_ROOT.parent / "docs" / "SIM2SWIM_DEMO.md"
BASE_SAFETY = PACKAGE_ROOT.parent / "brov_base" / "config" / "safety.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "sim2swim_demo_launch", LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _yaml_parameters(name: str, node_name: str) -> dict:
    document = yaml.safe_load(
        (PACKAGE_ROOT / "config" / name).read_text(encoding="utf-8")
    )
    return document[node_name]["ros__parameters"]


def _context(case: str, allow_case_c: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations["case"] = case
    context.launch_configurations["allow_case_c"] = allow_case_c
    return context


def _selected_include(module, case: str, allow_case_c: str):
    module.get_package_share_directory = lambda package: (
        str(PACKAGE_ROOT) if package == "brov_bringup" else f"/share/{package}"
    )
    actions = module._include_selected_case(_context(case, allow_case_c))
    assert len(actions) == 1
    include = actions[0]
    assert isinstance(include, IncludeLaunchDescription)
    return include


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


def test_case_a_uses_position_v1_takeoff_then_align_profile() -> None:
    profile = _yaml_parameters(
        "mission_manager_sim2swim_a.yaml", "brov_mission_manager"
    )

    assert profile["contract_version"] == "brov_pool_position_mission_v1"
    assert profile["heading_mode"] == "takeoff_then_align"
    assert profile["allowed_heading_modes"] == ["takeoff_then_align"]
    assert profile["loop"] is True
    assert profile["min_waypoints"] == 3
    assert profile["max_waypoints"] == 3
    assert profile["cruise_speed"] == pytest.approx(0.50)
    assert profile["max_cruise_speed"] == pytest.approx(0.60)
    assert profile["lookahead_dist"] == pytest.approx(0.40)
    assert profile["reach_threshold"] == pytest.approx(0.15)
    assert not any(key.startswith("random_attitude_") for key in profile)


def test_case_c_uses_bounded_deterministic_random_v2_profile() -> None:
    profile = _yaml_parameters(
        "mission_manager_sim2swim_c.yaml", "brov_mission_manager"
    )

    assert profile["contract_version"] == "brov_pool_position_mission_v2"
    assert profile["heading_mode"] == "random_at_waypoint"
    assert profile["allowed_heading_modes"] == ["random_at_waypoint"]
    assert profile["loop"] is True
    assert profile["min_waypoints"] == 4
    assert profile["max_waypoints"] == 4
    assert profile["random_attitude_reference_frame"] == "pool_zup_flu"
    assert (
        profile["random_attitude_generator_version"]
        == "sha256_counter_uniform_rpy_v1"
    )
    assert isinstance(profile["random_attitude_seed"], int)
    assert profile["random_attitude_seed"] >= 0

    minimum = profile["random_attitude_rpy_min_rad"]
    maximum = profile["random_attitude_rpy_max_rad"]
    assert len(minimum) == len(maximum) == 3
    assert all(
        math.isfinite(float(lower))
        and math.isfinite(float(upper))
        and float(lower) < float(upper)
        for lower, upper in zip(minimum, maximum)
    )
    assert minimum == pytest.approx(
        [-math.radians(15.0), -math.radians(15.0), -math.radians(30.0)]
    )
    assert maximum == pytest.approx(
        [math.radians(15.0), math.radians(15.0), math.radians(30.0)]
    )
    for key in (
        "random_attitude_max_slew_rate_rad_s",
        "random_attitude_tolerance_rad",
        "random_attitude_angular_speed_tolerance_rad_s",
        "random_attitude_dwell_time_s",
        "random_attitude_max_duration_s",
    ):
        assert math.isfinite(float(profile[key])) and float(profile[key]) > 0.0
    assert profile["random_attitude_max_slew_rate_rad_s"] == pytest.approx(
        math.radians(10.0)
    )
    assert profile["random_attitude_max_duration_s"] == pytest.approx(60.0)
    assert profile["random_attitude_max_laps"] == 1

    controller = _yaml_parameters(
        "rl_controller_sim2swim_c.yaml", "brov_policy_node"
    )
    limits = controller["action_abs_limit"]
    assert len(limits) == 6
    assert all(0.0 < float(value) < 1.0 for value in limits)
    assert controller["pwm_abs_limit"] == pytest.approx(0.35)
    assert controller["pwm_slew_rate_per_s"] == pytest.approx(0.40)


def test_case_c_has_benign_bootstrap_and_gateway_envelope() -> None:
    bootstrap = _yaml_parameters(
        "mission_sim2swim_bootstrap.yaml", "brov_obs_node"
    )
    assert bootstrap["heading_mode"] == "straight"
    assert bootstrap["loop"] is False
    assert bootstrap["cruise_speed"] <= 0.01
    assert "random" not in bootstrap["heading_mode"]

    general = yaml.safe_load(BASE_SAFETY.read_text(encoding="utf-8"))[
        "brov_obs_node"
    ]["ros__parameters"]
    case_c = _yaml_parameters("safety_sim2swim_c.yaml", "brov_obs_node")
    overridden = {
        "max_resolved_cruise_speed",
        "max_pwm_abs",
        "max_pwm_delta_per_s",
    }
    assert set(general) <= set(case_c)
    assert all(
        case_c[key] == value
        for key, value in general.items()
        if key not in overridden
    )
    assert case_c["max_pwm_abs"] == pytest.approx(0.35)
    assert case_c["max_pwm_delta_per_s"] == pytest.approx(0.50)
    assert general["max_resolved_cruise_speed"] == pytest.approx(0.60)
    assert case_c["max_resolved_cruise_speed"] == pytest.approx(0.30)
    controller_slew = _yaml_parameters(
        "rl_controller_sim2swim_c.yaml", "brov_policy_node"
    )["pwm_slew_rate_per_s"]
    assert float(controller_slew) < float(case_c["max_pwm_delta_per_s"])
    assert case_c["pwm_rate_first_command_dt_s"] == pytest.approx(0.04)
    assert case_c["max_random_mission_laps"] == 1


def test_launch_has_safe_fixed_pool_defaults(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/share/{package}",
    )
    defaults = _defaults(module.generate_launch_description())

    assert defaults["case"] == "a"
    assert defaults["allow_case_c"] == "false"
    assert defaults["send_pwm"] == "false"
    assert defaults["arm"] == "false"
    assert defaults["demo_orchestrator"] == "true"
    assert defaults["case_a_target_pool_z_m"] == "0.70"
    assert defaults["case_a_segment_length_m"] == "0.20"
    assert defaults["camera"] == "true"
    assert defaults["aruco"] == "true"
    assert defaults["require_pool_localization"] == "true"
    assert defaults["require_resolved_mission"] == "true"
    assert defaults["rviz"] == "false"


def test_case_c_is_fail_closed_without_explicit_opt_in(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: str(PACKAGE_ROOT),
    )

    with pytest.raises(RuntimeError, match="case c is fail-closed"):
        module._include_selected_case(_context("c", "false"))


@pytest.mark.parametrize(
    (
        "case",
        "allow_case_c",
        "bootstrap",
        "manager",
        "safety_config",
        "rl_config",
    ),
    [
        (
            "a",
            "false",
            "mission_sim2swim_a.yaml",
            "mission_manager_sim2swim_a.yaml",
            None,
            "/share/brov_control/config/rl_controller.yaml",
        ),
        (
            "c",
            "true",
            "mission_sim2swim_bootstrap.yaml",
            "mission_manager_sim2swim_c.yaml",
            str(PACKAGE_ROOT / "config" / "safety_sim2swim_c.yaml"),
            str(PACKAGE_ROOT / "config" / "rl_controller_sim2swim_c.yaml"),
        ),
    ],
)
def test_case_selects_rl_pool_profile(
    case: str,
    allow_case_c: str,
    bootstrap: str,
    manager: str,
    safety_config: str | None,
    rl_config: str,
) -> None:
    module = _load_launch_module()
    include = _selected_include(module, case, allow_case_c)
    arguments = dict(include.launch_arguments)

    assert arguments["controller"] == "rl"
    assert arguments["demo_case"] == case
    assert arguments["auto_generate_case_a_path"] == (
        "true" if case == "a" else "false"
    )
    assert perform_substitutions(
        LaunchContext(),
        arguments["case_a_target_pool_z_m"].variable_name,
    ) == "case_a_target_pool_z_m"
    assert perform_substitutions(
        LaunchContext(),
        arguments["case_a_segment_length_m"].variable_name,
    ) == "case_a_segment_length_m"
    assert perform_substitutions(
        LaunchContext(),
        arguments["require_pool_localization"].variable_name,
    ) == (
        "require_pool_localization"
    )
    assert perform_substitutions(
        LaunchContext(),
        arguments["require_resolved_mission"].variable_name,
    ) == (
        "require_resolved_mission"
    )
    assert Path(arguments["mission_file"]) == (
        PACKAGE_ROOT / "config" / bootstrap
    )
    assert Path(arguments["mission_manager_config"]) == (
        PACKAGE_ROOT / "config" / manager
    )
    if safety_config is None:
        assert perform_substitutions(
            LaunchContext(), arguments["safety_config"].variable_name
        ) == "safety_config"
    else:
        assert Path(arguments["safety_config"]) == Path(safety_config)
    assert Path(arguments["rl_config"]) == Path(rl_config)


def test_wrapper_reuses_pool_launch_and_has_no_automation() -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_names = {
        call.func.id
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
    }

    assert "pool_localized_demo.launch.py" in source
    assert "sim2real_demo.launch.py" not in source
    assert "Node" not in called_names
    assert "ExecuteProcess" not in called_names
    assert "TimerAction" not in called_names
    assert "EmitEvent" not in called_names


def test_runbook_records_resolved_pool_workflow_and_lifecycle() -> None:
    text = RUNBOOK.read_text(encoding="utf-8")
    for required in (
        "brov_pool_position_mission_v1",
        "brov_pool_position_mission_v2",
        "pool_zup_flu",
        "sha256_counter_uniform_rpy_v1",
        "require_pool_localization=true",
        "require_resolved_mission=true",
        "/brov/mission/draft_path",
        "/brov/localization/confirm_camera_tilt_neutral",
        "/brov/localization/initialize_pool",
        "/brov/mission/validate",
        "/brov/mission/commit",
        "/brov/prepare_control",
        "allow_case_c:=true",
        "16-D",
        "continuous fusion",
    ):
        assert required in text

    prepare = text.index("ros2 service call /brov/prepare_control")
    arm = text.index("ros2 service call /brov/arm_control", prepare)
    start = text.index("ros2 service call /brov/start_control", arm)
    stop = text.index("ros2 service call /brov/stop_control", start)
    disarm = text.index("ros2 service call /brov/disarm_control", stop)
    assert prepare < arm < start < stop < disarm
    assert "ros2 service call /brov/model_based/start" not in text
