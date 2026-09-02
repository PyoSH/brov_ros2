"""`pool_demo_a.launch.py` 정적 계약.

이 launch 는 **실기가 물속에서 추력을 내는** 구성을 만든다. 그러므로 여기서
고정하는 것은 편의가 아니라 안전이다:

- 추력 기본값이 꺼져 있을 것
- launch 가 스스로 정렬·arm·start 하지 않을 것
- marker 프레임의 waypoint 가 수조 안전 영역 안일 것 (guidance 의 한계 검사는
  세그먼트 **길이**만 보지 경계 상자를 모른다)
- dead time 분석에 필요한 두 토픽이 기록 목록에 있을 것 (사후에 다시 못 잰다)
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / "launch" / "pool_demo_a.launch.py"


def _module(monkeypatch):
    spec = importlib.util.spec_from_file_location("pool_demo_a_launch", LAUNCH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda package: f"/share/{package}"
    )
    return module


def _declarations(description: LaunchDescription):
    return {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _defaults(description: LaunchDescription) -> dict[str, str]:
    context = LaunchContext()
    return {
        name: perform_substitutions(context, declaration.default_value)
        for name, declaration in _declarations(description).items()
    }


def _text(context: LaunchContext, value) -> str:
    """문자열이든 substitution 이든 하나의 텍스트로 만든다."""
    if isinstance(value, str):
        return value
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value)
    )


def _compose(module, **overrides):
    """`_compose` 를 실행해 launch 가 만드는 entity 목록과 그 context 를 얻는다."""
    description = module.generate_launch_description()
    context = LaunchContext()
    for name, value in {**_defaults(description), **overrides}.items():
        context.launch_configurations[name] = value
    return module._compose(context), context


def _included_arguments(composed) -> dict[str, str]:
    actions, context = composed
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            return {
                _text(context, key): _text(context, value)
                for key, value in action.launch_arguments
            }
    raise AssertionError("split_stack include 를 찾지 못했다")


def _nodes(composed) -> list[tuple[str, str]]:
    return [
        (action.node_package, action.node_executable)
        for action in composed[0]
        if isinstance(action, Node)
    ]


def _record_command(composed) -> list[str] | None:
    actions, context = composed
    for action in actions:
        # launch_ros 의 Node 는 ExecuteProcess 를 상속한다. 먼저 걸러내지 않으면
        # 첫 노드의 실행 명령을 기록기로 착각한다.
        if isinstance(action, ExecuteProcess) and not isinstance(action, Node):
            return [_text(context, part) for part in action.cmd]
    return None


def test_thrust_and_arm_default_to_off(monkeypatch):
    defaults = _defaults(_module(monkeypatch).generate_launch_description())
    assert defaults["send_pwm"] == "false"
    assert defaults["arm"] == "false"


def test_marker_frame_and_recording_are_the_defaults(monkeypatch):
    defaults = _defaults(_module(monkeypatch).generate_launch_description())
    assert defaults["frame"] == "marker"
    assert defaults["markers"] == "true"
    # 지연도 센서 편차도 사후에 다시 잴 방법이 없다.
    assert defaults["record_bag"] == "true"
    # DVL 기록은 기본 off. 2026-09-02 실기에서 이 노드가 BlueOS DVL extension 을
    # 밀어내 EKF 가 위치를 잃었다 -- 기록 전용 노드가 제어의 위치원을 걸면 안 된다.
    assert defaults["dvl"] == "false"
    # 게이트 통과 전에는 깊이 출처를 조용히 바꾸지 않는다.
    assert defaults["depth_source"] == "mavlink_ekf"


def test_marker_frame_uses_absolute_pool_waypoints(monkeypatch):
    module = _module(monkeypatch)
    arguments = _included_arguments(_compose(module))
    assert arguments["waypoint_frame"] == "pool"
    points = [
        [float(v) for v in chunk.split(",")]
        for chunk in arguments["waypoints"].split(";")
    ]
    assert points[0] == pytest.approx([0.60, 0.85, 0.70])
    assert points[1] == pytest.approx([3.10, 0.85, 0.70])
    for point in points:
        for axis, value in zip("xyz", point):
            low, high = module._POOL_ENVELOPE[axis]
            assert low <= value <= high


def test_start_heading_frame_matches_the_drag_test_convention(monkeypatch):
    module = _module(monkeypatch)
    arguments = _included_arguments(_compose(module, frame="start_heading"))
    assert arguments["waypoint_frame"] == "start_heading"
    assert arguments["waypoints"] in ("0,0,0.0000;2.5000,0,0.0000", "0,0,-0.0000;2.5000,0,-0.0000")


def test_waypoints_outside_the_pool_envelope_are_refused(monkeypatch):
    module = _module(monkeypatch)
    with pytest.raises(RuntimeError, match="안전 영역"):
        _compose(module, leg_m="3.2")
    with pytest.raises(RuntimeError, match="안전 영역"):
        _compose(module, lane_y_m="1.80")
    with pytest.raises(RuntimeError, match="안전 영역"):
        _compose(module, target_pool_z_m="1.20")


def test_start_heading_leg_is_not_envelope_checked(monkeypatch):
    """상대 프레임에는 절대 좌표가 없다. 검사하는 척하지 않는다."""
    module = _module(monkeypatch)
    arguments = _included_arguments(
        _compose(module, frame="start_heading", leg_m="3.2")
    )
    assert arguments["waypoints"] in ("0,0,0.0000;3.2000,0,0.0000", "0,0,-0.0000;3.2000,0,-0.0000")


def test_marker_frame_refuses_to_run_without_the_marker_pipeline(monkeypatch):
    module = _module(monkeypatch)
    with pytest.raises(RuntimeError, match="markers"):
        _compose(module, markers="false")


def test_marker_pipeline_composes_exactly_the_three_producers(monkeypatch):
    module = _module(monkeypatch)
    nodes = _nodes(_compose(module))
    assert ("brov_perception", "camera_stream_node") in nodes
    assert ("brov_perception", "aruco_pose_node") in nodes
    assert ("brov_localization", "pool_alignment_node") in nodes
    # 미션 스택은 띄우지 않는다. `/brov/cmd/wrench` 발행자는 정확히 하나여야 한다.
    assert ("brov_mission", "mission_manager_node") not in nodes
    assert ("brov_control", "model_based_controller_node") not in nodes


def test_markers_can_be_dropped_only_in_the_relative_frame(monkeypatch):
    module = _module(monkeypatch)
    nodes = _nodes(_compose(module, frame="start_heading", markers="false"))
    assert ("brov_perception", "camera_stream_node") not in nodes
    assert ("brov_localization", "pool_alignment_node") not in nodes


def test_dvl_is_record_only_and_off_unless_asked(monkeypatch):
    module = _module(monkeypatch)
    assert ("brov_control", "dvl_record_node") not in _nodes(_compose(module))
    assert ("brov_control", "dvl_record_node") in _nodes(
        _compose(module, dvl="true")
    )


def test_recorded_topics_cover_dead_time_and_both_depth_paths(monkeypatch):
    module = _module(monkeypatch)
    command = _record_command(_compose(module))
    assert command is not None and command[:3] == ["ros2", "bag", "record"]
    recorded = set(command)
    # diag_loop_delay 는 이 둘의 교차상관으로 dead time 을 낸다.
    assert {"/brov/cmd/wrench", "/brov/state"} <= recorded
    # `/brov/state` 가 고르지 않은 깊이 경로.
    assert {
        "/brov/sensor/depth_ekf",
        "/brov/sensor/pressure0",
        "/brov/sensor/pressure1",
        "/brov/sensor/pressure2",
    } <= recorded
    assert {"/brov/sensor/ahrs", "/brov/dvl/sample"} <= recorded
    # 카메라 영상은 분 단위로 GB 를 먹는다.
    assert not [topic for topic in recorded if "image" in topic]


def test_bag_path_never_collides_with_an_existing_recording(tmp_path, monkeypatch):
    """`ros2 bag record -o` 는 기존 디렉터리를 거절하고 죽는다. 나머지 스택은
    멀쩡히 돌기 때문에, 기록 없이 주행이 끝나고 그 사실을 사후에야 안다 --
    지연도 센서 편차도 다시 잴 방법이 없는데. 2026-09-02 실기에서 실제로 죽었다."""
    module = _module(monkeypatch)
    free = tmp_path / "run1"
    command = _record_command(_compose(module, bag_path=str(free)))
    assert command[command.index("-o") + 1] == str(free)

    free.mkdir()
    command = _record_command(_compose(module, bag_path=str(free)))
    chosen = command[command.index("-o") + 1]
    assert chosen != str(free)
    assert chosen.startswith(str(free) + "-")
    assert not os.path.exists(chosen)


def test_split_stack_recording_is_disabled_to_avoid_two_recorders(monkeypatch):
    module = _module(monkeypatch)
    assert _included_arguments(_compose(module))["record_bag"] == "false"
    assert _record_command(_compose(module, record_bag="false")) is None


def test_launch_never_aligns_arms_or_starts(monkeypatch):
    """정렬·arm·start 는 전부 운용자의 명시적 서비스 호출로만 일어난다."""
    module = _module(monkeypatch)
    command = _record_command(_compose(module)) or []
    forbidden = (
        "initialize_pool",
        "arm_control",
        "start_control",
        "prepare_control",
        "confirm_camera_tilt_neutral",
    )
    text = LAUNCH.read_text(encoding="utf-8")
    body = text.split('"""', 2)[2]           # 운용 안내가 있는 docstring 은 제외
    for name in forbidden:
        assert name not in body, f"launch 본문이 {name} 을 호출한다"
        assert not [part for part in command if name in part]


def test_wrench_gain_defaults_to_unity_and_is_forwarded(monkeypatch):
    """실험 A1 의 손잡이. 기본 1.0 이어야 평소 주행이 정책 그대로이고, 값이
    split_stack 까지 닿아야 실험이 실제로 정책에 걸린다."""
    module = _module(monkeypatch)
    assert _defaults(module.generate_launch_description())["wrench_gain"] == "1.0"
    assert _included_arguments(_compose(module))["wrench_gain"] == "1.0"
    assert _included_arguments(_compose(module, wrench_gain="0.5"))["wrench_gain"] == "0.5"


def test_rise_lets_a_negatively_buoyant_vehicle_start_from_the_floor(monkeypatch):
    """음성부력 기체는 start 전에 바닥에 있다. rise_m 만큼 정책이 띄운다.
    사용자는 양수 미터만 준다 -- NED 부호는 launch 가 뒤집는다."""
    module = _module(monkeypatch)
    arguments = _included_arguments(
        _compose(module, frame="start_heading", rise_m="0.4"))
    assert arguments["waypoints"] == "0,0,-0.4000;2.5000,0,-0.4000"
    # 기본은 0 -- start 깊이 유지 (기존 거동 그대로).
    assert _included_arguments(_compose(module, frame="start_heading"))["waypoints"] \
        == "0,0,-0.0000;2.5000,0,-0.0000" or \
        _included_arguments(_compose(module, frame="start_heading"))["waypoints"] \
        == "0,0,0.0000;2.5000,0,0.0000"
    with pytest.raises(RuntimeError, match="rise_m"):
        _compose(module, frame="start_heading", rise_m="0.9")
    with pytest.raises(RuntimeError, match="rise_m"):
        _compose(module, frame="start_heading", rise_m="-0.1")
