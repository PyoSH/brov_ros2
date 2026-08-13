"""Non-hardware contract tests for the Sim2Swim demo launch."""

from __future__ import annotations

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


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "sim2swim_demo_launch", LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _mission(name: str) -> dict:
    path = PACKAGE_ROOT / "config" / name
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data["brov_obs_node"]["ros__parameters"]


def _waypoints(parameters: dict) -> list[list[float]]:
    return [
        [float(component) for component in point.split(",")]
        for point in parameters["waypoints"].split(";")
    ]


def _assert_points_within_declared_bounds(parameters: dict) -> None:
    minimum = parameters["waypoint_min_xyz"]
    maximum = parameters["waypoint_max_xyz"]
    assert parameters["waypoint_bounds_enabled"] is True
    assert len(minimum) == len(maximum) == 3
    for point in _waypoints(parameters):
        assert len(point) == 3
        assert all(
            lower <= value <= upper
            for value, lower, upper in zip(point, minimum, maximum)
        )


def _context(case: str, allow_case_c: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations["case"] = case
    context.launch_configurations["allow_case_c"] = allow_case_c
    return context


def _selected_arguments(module, case: str, allow_case_c: str) -> dict:
    module.get_package_share_directory = lambda package: (
        str(PACKAGE_ROOT) if package == "brov_bringup" else f"/share/{package}"
    )
    context = _context(case, allow_case_c)
    actions = module._include_selected_case(context)
    assert len(actions) == 1
    include = actions[0]
    assert isinstance(include, IncludeLaunchDescription)
    # Keep forwarded LaunchConfiguration objects unresolved: resolving those
    # would require executing their declarations. The two structural values
    # selected by this launch (controller and mission_file) are plain strings.
    return dict(include.launch_arguments)


def test_case_a_exact_line_geometry_and_bounds() -> None:
    parameters = _mission("mission_sim2swim_a.yaml")
    points = _waypoints(parameters)

    assert points == [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
    assert parameters["waypoint_min_xyz"] == [0.0, 0.0, 0.0]
    assert parameters["waypoint_max_xyz"] == [2.0, 0.0, 0.0]
    assert math.dist(points[0], points[1]) == pytest.approx(2.0)
    _assert_points_within_declared_bounds(parameters)


def test_case_c_exact_square_geometry_and_bounds() -> None:
    parameters = _mission("mission_sim2swim_c.yaml")
    points = _waypoints(parameters)

    assert points == [
        [0.0, 0.0, 0.0],
        [0.4, 0.0, 0.0],
        [0.4, 0.4, 0.0],
        [0.0, 0.4, 0.0],
    ]
    assert parameters["waypoint_min_xyz"] == [0.0, 0.0, 0.0]
    assert parameters["waypoint_max_xyz"] == [0.4, 0.4, 0.0]
    closed_points = points + [points[0]]
    side_lengths = [
        math.dist(start, end)
        for start, end in zip(closed_points, closed_points[1:])
    ]
    assert side_lengths == pytest.approx([0.4, 0.4, 0.4, 0.4])
    _assert_points_within_declared_bounds(parameters)


def test_sim2swim_launch_has_safe_output_defaults(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: f"/share/{package}",
    )
    description = module.generate_launch_description()
    declarations = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()

    assert perform_substitutions(
        context, declarations["send_pwm"].default_value
    ) == "false"
    assert perform_substitutions(
        context, declarations["arm"].default_value
    ) == "false"


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
    ("case", "allow_case_c", "mission_name"),
    [
        ("a", "false", "mission_sim2swim_a.yaml"),
        ("c", "true", "mission_sim2swim_c.yaml"),
    ],
)
def test_structural_case_selection_without_launching_nodes(
    case: str,
    allow_case_c: str,
    mission_name: str,
) -> None:
    module = _load_launch_module()
    arguments = _selected_arguments(module, case, allow_case_c)

    assert arguments["controller"] == "rl"
    assert (
        Path(arguments["mission_file"])
        == PACKAGE_ROOT / "config" / mission_name
    )
