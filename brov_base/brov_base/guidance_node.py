#!/usr/bin/env python3
"""brov_guidance_node — 경로를 목표로 바꾼다.

역할 분리에서 이 노드가 소유하는 것
====================================
**"어디로 가야 하는가"만** 소유한다. 로봇을 만지지 않고, 오차를 계산하지 않고,
적분을 갖지 않는다.

  sub  /brov/state      BrovState
  pub  /brov/desired    DesiredState  (v_d_b, q_d, waypoint_index, mission_complete)

**step_3에서 LOSGuidance가 NBV/커버리지 모듈로 교체되는 지점이 정확히 여기다.**
그때 바꾸는 것은 이 파일 하나이고, 관측·정책·base는 무변경이다. 그것이 유도를
별도 노드로 뽑는 가장 큰 이유다.

유도 법칙
=========
`brov_base/guidance.py`의 `LOSGuidance` — Sim2Swim이 인용한
Breivik & Fossen (2005) Sec. IV의 3D LOS다. 경로 고정 방위/앙각에 cross-track과
vertical-track 오차의 lookahead 보정을 **독립적으로** 더하므로
`||v_d_b|| = cruise_speed`가 정확히 보존된다 — 정책이 학습한 명령 크기는
그 값 하나뿐이라 이 보존이 계약이다.

아직 여기로 옮기지 않은 것
==========================
`obs_node.py`의 ResolvedMission 처리는 검증 로직이 370줄이 넘는다(경계 상자,
세그먼트 길이, 랜덤 자세 slew/dwell, 미션 시간/랩 한계). 이 노드는 **한계값
검사만 승계**하고 파라미터 waypoint 경로를 먼저 지원한다. 전체 이관은 obs_node를
퇴역시킬 때 한다 — 그 전까지 둘을 **동시에 띄우면 안 된다**(목표가 두 벌 나온다).
"""

from __future__ import annotations

from geometry_msgs.msg import Quaternion, Vector3
import rclpy
from rclpy.node import Node
import torch

from brov_interfaces.msg import BrovState, DesiredState

from brov_base.guidance import LOSGuidance


def _parse_waypoints(text: str) -> torch.Tensor:
    """``"0,0,0;3,0,0"`` -> (1, N, 3)."""
    pts = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        xyz = [float(v) for v in chunk.split(",")]
        if len(xyz) != 3:
            raise ValueError(f"waypoint {chunk!r}는 x,y,z 3개여야 한다")
        pts.append(xyz)
    if len(pts) < 2:
        raise ValueError("waypoint가 2개 미만이면 세그먼트를 만들 수 없다")
    return torch.tensor([pts], dtype=torch.float32)


class GuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("brov_guidance")

        self.declare_parameter("waypoints", "0,0,0;3,0,0")
        self.declare_parameter("cruise_speed", 0.2)
        self.declare_parameter("heading_mode", "straight")
        self.declare_parameter("reach_threshold", 0.15)
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("lookahead_vert", 0.0)     # 0이면 수평값과 동일
        self.declare_parameter("loop", False)
        self.declare_parameter("terminal_hold_kp", 0.5)
        self.declare_parameter("terminal_speed_limit", 0.1)
        # obs_node에서 승계한 한계값. 파라미터가 이 범위를 넘으면 뜨지 않는다 —
        # 조용히 clamp하면 설정 실수가 수조에서 드러난다.
        self.declare_parameter("max_waypoints", 50)
        self.declare_parameter("max_segment_length_m", 4.0)
        self.declare_parameter("max_cruise_speed", 0.30)
        self.declare_parameter("max_lookahead_dist", 1.0)
        self.declare_parameter("max_reach_threshold", 0.50)
        self.declare_parameter("state_max_age_s", 0.2)

        p = self.get_parameter
        wps = _parse_waypoints(str(p("waypoints").value))
        speed = float(p("cruise_speed").value)
        lookahead = float(p("lookahead_dist").value)
        reach = float(p("reach_threshold").value)
        self._state_max_age = float(p("state_max_age_s").value)

        self._check_limits(wps, speed, lookahead, reach)

        vert = float(p("lookahead_vert").value)
        self._los = LOSGuidance(
            wps, "cpu",
            lookahead_dist=lookahead,
            cruise_speed=speed,
            reach_threshold=reach,
            heading_mode=str(p("heading_mode").value),
            loop=bool(p("loop").value),
            terminal_hold_kp=float(p("terminal_hold_kp").value),
            terminal_speed_limit=float(p("terminal_speed_limit").value),
            **({"lookahead_vert": vert} if vert > 0.0 else {}),
        )
        self._seq = 0
        self._pub = self.create_publisher(DesiredState, "/brov/desired", 10)
        self.create_subscription(BrovState, "/brov/state", self._on_state, 1)
        self.get_logger().info(
            f"guidance_node 시작 — waypoint {wps.shape[1]}개, "
            f"cruise {speed} m/s, heading {p('heading_mode').value}"
        )

    def _check_limits(self, wps, speed, lookahead, reach) -> None:
        p = self.get_parameter
        n = wps.shape[1]
        if n > int(p("max_waypoints").value):
            raise ValueError(f"waypoint {n}개 > 한계 {p('max_waypoints').value}")
        seg = (wps[0, 1:] - wps[0, :-1]).norm(dim=-1)
        longest = float(seg.max()) if len(seg) else 0.0
        if longest > float(p("max_segment_length_m").value):
            raise ValueError(
                f"세그먼트 {longest:.2f} m > 한계 {p('max_segment_length_m').value} m")
        for value, key in ((speed, "max_cruise_speed"),
                           (lookahead, "max_lookahead_dist"),
                           (reach, "max_reach_threshold")):
            if value > float(p(key).value):
                raise ValueError(f"{key.replace('max_','')} {value} > 한계 {p(key).value}")

    def _on_state(self, state: BrovState) -> None:
        out = DesiredState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.seq = self._seq
        self._seq += 1

        if not state.valid or max(state.attitude_age_s, state.position_age_s) > self._state_max_age:
            # 상태가 신뢰할 수 없으면 목표를 만들지 않는다. 관측 노드가 이
            # 침묵을 stale로 보고, 정책은 명령을 내지 않으며, base watchdog이
            # 중립 정지한다 — 실패가 안전한 방향으로 전파된다.
            return

        pos = torch.tensor([[state.position.x, state.position.y, state.position.z]],
                           dtype=torch.float32)
        quat = torch.tensor([[state.attitude.w, state.attitude.x,
                              state.attitude.y, state.attitude.z]], dtype=torch.float32)
        v_d_b, q_d = self._los.compute(pos, quat)

        out.velocity_body = Vector3(x=float(v_d_b[0, 0]), y=float(v_d_b[0, 1]),
                                    z=float(v_d_b[0, 2]))
        out.attitude = Quaternion(w=float(q_d[0, 0]), x=float(q_d[0, 1]),
                                  y=float(q_d[0, 2]), z=float(q_d[0, 3]))
        out.waypoint_index = int(self._los._wp_idx[0])
        out.mission_complete = bool(getattr(self._los, "mission_complete", torch.zeros(1))[0])
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
