"""`waypoint_frame=pool` — 마커 정렬이 세운 절대 프레임 회귀 시험.

무엇을 지키는가
===============
1. **좌표 변환이 왕복한다.** 수조 좌표로 준 waypoint 가 실제로 그 수조 지점을
   가리켜야 한다. 틀리면 벽까지 남은 거리가 통째로 어긋나고, 게이트는 하나도
   걸리지 않는다 -- 경로 길이는 그대로이므로 guidance 의 한계 검사도 통과한다.
2. **정렬이 없으면 목표를 내지 않는다.** 절대 좌표 경로를 절대 프레임 없이
   따라가면 임의의 방향으로 2.5 m 를 달린다. 침묵하면 base watchdog 이
   0.25 s 안에 중립 정지시킨다.
3. **주행 중 정렬이 갈리면 멈춘다.** 새 정렬로 조용히 갈아타는 것은 같은 실패의
   느린 판본이다.
"""

from __future__ import annotations

import math

import pytest
import rclpy
import torch

from brov_interfaces.msg import BrovState, DesiredState, LocalizationStatus

from brov_base import math_utils as mu
from brov_base.guidance_node import GuidanceNode


_S = torch.tensor([1.0, -1.0, -1.0])


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _quat_from_yaw(yaw: float) -> torch.Tensor:
    return torch.tensor(
        [[math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]],
        dtype=torch.float32,
    )


def _make_node(**overrides):
    """파라미터를 override 한 GuidanceNode 를 만든다."""
    params = {
        "waypoints": "0.60,0.85,0.70;3.10,0.85,0.70",
        "waypoint_frame": "pool",
        "cruise_speed": 0.25,
        "heading_mode": "align",
        "lookahead_dist": 1.0,
        "reach_threshold": 0.30,
        "loop": True,
    }
    params.update(overrides)
    context_overrides = [
        rclpy.parameter.Parameter(name, value=value)
        for name, value in params.items()
    ]
    node = GuidanceNode(parameter_overrides=context_overrides)
    node._control_active = True
    return node


def _status(
    *,
    yaw_rad: float = 0.0,
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    state: int = LocalizationStatus.INITIALIZED,
    output_valid: bool = True,
    epoch: int = 3,
    alignment_id: str = "align-1",
) -> LocalizationStatus:
    """``pool_to_odom`` = ``T_pool_odom`` 을 담은 status 를 만든다."""
    status = LocalizationStatus()
    status.state = int(state)
    status.output_valid = bool(output_valid)
    status.epoch = int(epoch)
    status.alignment_id = alignment_id
    status.odometry_session_id = "session-1"
    status.pool_to_odom.rotation.w = math.cos(yaw_rad / 2.0)
    status.pool_to_odom.rotation.x = 0.0
    status.pool_to_odom.rotation.y = 0.0
    status.pool_to_odom.rotation.z = math.sin(yaw_rad / 2.0)
    status.pool_to_odom.translation.x = translation[0]
    status.pool_to_odom.translation.y = translation[1]
    status.pool_to_odom.translation.z = translation[2]
    return status


def _ned_for_pool(
    pool_xyz: tuple[float, float, float],
    *,
    yaw_rad: float,
    translation: tuple[float, float, float],
) -> torch.Tensor:
    """수조 좌표 하나를 그에 대응하는 NED 좌표로 되돌린다.

    ``p_pool = R_A (S p_ned) + t_A`` 의 역이다.
    """
    q_a = _quat_from_yaw(yaw_rad)
    t_a = torch.tensor([list(translation)], dtype=torch.float32)
    p_pool = torch.tensor([list(pool_xyz)], dtype=torch.float32)
    p_odom = mu.quat_apply(mu.quat_conjugate(q_a), p_pool - t_a)
    return p_odom * _S


def _state(pos_ned: torch.Tensor, yaw_ned: float = 0.0) -> BrovState:
    state = BrovState()
    state.valid = True
    state.attitude_age_s = 0.01
    state.position_age_s = 0.01
    state.position.x = float(pos_ned[0, 0])
    state.position.y = float(pos_ned[0, 1])
    state.position.z = float(pos_ned[0, 2])
    quat = _quat_from_yaw(yaw_ned)
    state.attitude.w = float(quat[0, 0])
    state.attitude.x = float(quat[0, 1])
    state.attitude.y = float(quat[0, 2])
    state.attitude.z = float(quat[0, 3])
    return state


def _collect(node) -> list[DesiredState]:
    published: list[DesiredState] = []
    node._pub.publish = published.append
    return published


def test_pool_frame_maps_state_back_to_the_pool_coordinate_it_came_from():
    """정렬이 회전과 평행이동을 모두 가져도 수조 좌표가 정확히 복원된다."""
    yaw, translation = math.radians(37.0), (1.30, -0.42, 0.55)
    node = _make_node()
    node._on_pool_status(_status(yaw_rad=yaw, translation=translation))

    pool_point = (1.75, 0.90, 0.65)
    pos_ned = _ned_for_pool(pool_point, yaw_rad=yaw, translation=translation)
    node._on_state(_state(pos_ned))

    # 미션 프레임은 NED 규약이므로 S 를 한 번 되돌리면 수조 좌표다.
    mission = mu.quat_apply(node._q_ned_to_mission, pos_ned - node._origin_ned)
    recovered = (mission * _S)[0].tolist()
    assert recovered == pytest.approx(list(pool_point), abs=1e-4)
    node.destroy_node()


def test_no_desired_state_without_an_initialized_alignment():
    """정렬 전에는 목표를 내지 않는다 — watchdog 이 중립 정지시킨다."""
    node = _make_node()
    published = _collect(node)
    node._on_state(_state(torch.zeros(1, 3)))
    assert published == []
    assert "미수신" in node._pool_gate_reason

    node._on_pool_status(_status(state=LocalizationStatus.COLLECTING))
    node._on_state(_state(torch.zeros(1, 3)))
    assert published == []
    assert "정렬 미완료" in node._pool_gate_reason

    node._on_pool_status(_status(output_valid=False))
    node._on_state(_state(torch.zeros(1, 3)))
    assert published == []
    assert "출력 무효" in node._pool_gate_reason

    node._on_pool_status(_status())
    node._on_state(_state(torch.zeros(1, 3)))
    assert len(published) == 1
    node.destroy_node()


def test_alignment_change_during_the_run_stops_the_desired_stream():
    """정렬이 갈리면 새 절대 좌표계로 조용히 갈아타지 않고 멈춘다."""
    node = _make_node()
    published = _collect(node)
    node._on_pool_status(_status(epoch=3, alignment_id="align-1"))
    node._on_state(_state(torch.zeros(1, 3)))
    assert len(published) == 1

    node._on_pool_status(_status(epoch=4, alignment_id="align-2"))
    node._on_state(_state(torch.zeros(1, 3)))
    assert len(published) == 1
    assert "주행 중 바뀌었다" in node._pool_gate_reason
    node.destroy_node()


def test_stale_alignment_status_stops_the_desired_stream():
    """status 가 갱신되지 않으면(노드가 죽으면) 목표를 내지 않는다."""
    node = _make_node(pool_status_max_age_s=1.0)
    published = _collect(node)
    node._on_pool_status(_status())
    node._on_state(_state(torch.zeros(1, 3)))
    assert len(published) == 1

    node._pool_status_wall -= 5.0
    node._on_state(_state(torch.zeros(1, 3)))
    assert len(published) == 1
    assert "갱신되지 않았다" in node._pool_gate_reason
    node.destroy_node()


def test_degenerate_alignment_rotation_stops_instead_of_crashing():
    """쿼터니언이 아닌 회전이 오면 노드가 죽지 않고 목표만 멈춘다.

    구독 콜백에서 예외를 던지면 노드가 통째로 죽고, 그러면 `/brov/desired` 도
    `/brov/state` 도 아닌 **아무 신호도** 남지 않아 원인 추적이 불가능해진다.
    """
    node = _make_node()
    published = _collect(node)
    status = _status()
    status.pool_to_odom.rotation.w = 0.0
    node._on_pool_status(status)
    node._on_state(_state(torch.zeros(1, 3)))
    assert published == []
    assert "쿼터니언" in node._pool_gate_reason
    node.destroy_node()


def test_start_heading_frame_still_needs_no_alignment():
    """상대 프레임 주행은 마커 없이도 그대로 돈다 — 회귀 방지."""
    node = _make_node(
        waypoints="0,0,0;2.5,0,0", waypoint_frame="start_heading"
    )
    published = _collect(node)
    node._on_state(_state(torch.zeros(1, 3), yaw_ned=math.radians(90.0)))
    assert len(published) == 1
    node.destroy_node()


def test_unknown_waypoint_frame_is_refused():
    with pytest.raises(ValueError):
        _make_node(waypoint_frame="pool_zup")
