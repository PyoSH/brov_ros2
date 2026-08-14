"""Static launch contracts for fail-closed pool-localized bringup."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCH = PACKAGE_ROOT / "launch" / "base.launch.py"
POOL_LAUNCH = PACKAGE_ROOT / "launch" / "pool_localized_demo.launch.py"
RUNBOOK = PACKAGE_ROOT.parent / "docs" / "POOL_LOCALIZATION_RUNBOOK.md"
VIZ_LAUNCH = (
    PACKAGE_ROOT.parent / "brov_viz" / "launch" / "pool_vision.launch.py"
)
LOCALIZATION_CONFIG = (
    PACKAGE_ROOT.parent / "brov_localization" / "config" / "localization.yaml"
)


def _load_launch_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _description(path: Path, name: str, monkeypatch):
    module = _load_launch_module(path, name)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/share/{package}",
    )
    return module.generate_launch_description()


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


def _node_literals(path: Path) -> list[tuple[str, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    result = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        values = {
            keyword.arg: keyword.value
            for keyword in call.keywords
            if keyword.arg is not None
        }
        package = values.get("package")
        executable = values.get("executable")
        assert isinstance(package, ast.Constant) and isinstance(
            package.value, str
        )
        assert isinstance(executable, ast.Constant) and isinstance(
            executable.value, str
        )
        result.append((package.value, executable.value))
    return result


def test_base_gate_arguments_default_off_and_reach_obs_node(monkeypatch) -> None:
    description = _description(BASE_LAUNCH, "base_launch", monkeypatch)
    defaults = _defaults(description)
    source = BASE_LAUNCH.read_text(encoding="utf-8")

    assert defaults["require_pool_localization"] == "false"
    assert defaults["require_resolved_mission"] == "false"
    assert '"require_pool_localization": ParameterValue(' in source
    assert '"require_resolved_mission": ParameterValue(' in source


def test_pool_profile_is_safe_and_forces_both_gates(monkeypatch) -> None:
    description = _description(POOL_LAUNCH, "pool_launch", monkeypatch)
    defaults = _defaults(description)
    include = next(
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    )
    forwarded = dict(include.launch_arguments)

    assert defaults["send_pwm"] == "false"
    assert defaults["arm"] == "false"
    assert defaults["require_pool_localization"] == "true"
    assert defaults["require_resolved_mission"] == "true"
    assert defaults["controller"] == "model"
    assert defaults["demo_orchestrator"] == "true"
    assert defaults["demo_case"] == "a"
    assert defaults["auto_generate_case_a_path"] == "true"
    assert defaults["rviz"] == "false"
    assert forwarded["require_pool_localization"] == "true"
    assert forwarded["require_resolved_mission"] == "true"


def test_pool_profile_has_required_nodes_and_exactly_two_controller_branches() -> None:
    nodes = _node_literals(POOL_LAUNCH)
    source = POOL_LAUNCH.read_text(encoding="utf-8")

    assert nodes.count(("brov_localization", "pool_alignment_node")) == 1
    assert nodes.count(("brov_mission", "mission_manager_node")) == 1
    assert nodes.count(("brov_perception", "camera_stream_node")) == 1
    assert nodes.count(("brov_perception", "aruco_pose_node")) == 1
    assert nodes.count(
        ("brov_bringup", "demo_orchestrator_node")
    ) == 1
    assert nodes.count(
        ("brov_control", "model_based_controller_node")
    ) == 1
    assert nodes.count(("brov_control", "policy_node")) == 1
    assert "condition=model_selected" in source
    assert "condition=rl_selected" in source
    assert "condition=IfCondition(demo_orchestrator)" in source


def test_launch_contains_no_automatic_service_or_process_action() -> None:
    tree = ast.parse(POOL_LAUNCH.read_text(encoding="utf-8"))
    called_names = {
        call.func.id
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
    }

    assert "ExecuteProcess" not in called_names
    assert "TimerAction" not in called_names
    assert "EmitEvent" not in called_names


def test_optional_rviz_include_is_visualization_only_and_default_off() -> None:
    pool_source = POOL_LAUNCH.read_text(encoding="utf-8")
    viz_nodes = _node_literals(VIZ_LAUNCH)

    assert 'condition=IfCondition(rviz)' in pool_source
    assert ("brov_viz", "pool_scene_node") in viz_nodes
    assert ("rviz2", "rviz2") in viz_nodes
    assert all(
        package not in {"brov_base", "brov_control", "brov_perception"}
        for package, _executable in viz_nodes
    )


def test_runbook_records_identity_observation_and_known_limits() -> None:
    text = RUNBOOK.read_text(encoding="utf-8")

    for required in (
        "full-SE(3)",
        "odometry session",
        "localization epoch",
        "alignment_id",
        "pool_to_odom",
        "output_valid=true",
        "canonical_plan_json",
        "contract_version",
        "16-D observation",
        "정지 초기화",
        "decode-time stamp",
        "neutral tilt",
        "nominal extrinsic",
        "continuous fusion 없음",
    ):
        assert required in text

    for service in (
        "/brov/localization/confirm_camera_tilt_neutral",
        "/brov/localization/initialize_pool",
        "/brov/mission/validate",
        "/brov/mission/commit",
        "/brov/prepare_control",
        "/brov/arm_control",
        "/brov/start_control",
        "/brov/disarm_control",
    ):
        assert f"ros2 service call {service}" in text

    assert 'InitializePool "{min_samples: 0}"' in text
    assert "localizer를 재시작" in text
    assert "PREPARE → ARM → START" in text

    actuation = text.split("### 7. 실제 actuation", 1)[1].split(
        "## Invalidation", 1
    )[0]
    prepare = actuation.index(
        "ros2 service call /brov/prepare_control"
    )
    arm = actuation.index("ros2 service call /brov/arm_control")
    start = actuation.index("ros2 service call /brov/start_control")
    controller = actuation.index(
        "ros2 service call /brov/model_based/start"
    )
    assert prepare < arm < start < controller

    stop = actuation.index("ros2 service call /brov/stop_control")
    controller_stop = actuation.index(
        "ros2 service call /brov/model_based/stop"
    )
    disarm = actuation.index("ros2 service call /brov/disarm_control")
    assert stop < controller_stop < disarm


def test_launch_documents_explicit_arm_and_keeps_operator_actions_manual() -> None:
    base_source = BASE_LAUNCH.read_text(encoding="utf-8")
    pool_source = POOL_LAUNCH.read_text(encoding="utf-8")

    assert "Permit the explicit arm_control service" in base_source
    assert "Permit the explicit arm_control service" in pool_source
    assert "launch construction never confirms camera" in pool_source
    assert "initializes alignment" in pool_source
    assert "commits a route" in pool_source


def test_pool_profile_dependency_requires_tilt_neutral_confirmation() -> None:
    document = yaml.safe_load(
        LOCALIZATION_CONFIG.read_text(encoding="utf-8")
    )
    parameters = document["brov_pool_alignment"]["ros__parameters"]

    assert parameters["require_camera_tilt_neutral_confirmation"] is True
    assert int(parameters["default_min_samples"]) > 1
