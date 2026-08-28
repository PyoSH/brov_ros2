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

from brov_base import math_utils as mu
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
        # 운용자가 준 파라미터에 대한 **온전성 검사**다. 조용히 clamp하면 설정
        # 실수가 수조에서 드러나므로, 넘으면 아예 뜨지 않는다.
        #
        # obs_node의 max_resolved_* 와는 대상이 다르다 — 그쪽은 **외부에서 들어오는
        # ResolvedMission**(신뢰할 수 없는 입력)의 경계이고, 이쪽은 launch 인자다.
        # 기본값은 정본 미션(mission_sim2sim_mk2_case_a_0p5.yaml: cruise 0.50,
        # lookahead 0.40, 세그먼트 2.0 m)을 통과시키도록 잡는다. ResolvedMission
        # 경로를 이 노드로 옮길 때는 그 입력에 별도의(더 엄격한) 경계가 필요하다.
        self.declare_parameter("max_waypoints", 50)
        self.declare_parameter("max_segment_length_m", 4.0)
        self.declare_parameter("max_cruise_speed", 0.55)
        self.declare_parameter("max_lookahead_dist", 1.0)
        self.declare_parameter("max_reach_threshold", 0.50)
        self.declare_parameter("state_max_age_s", 0.2)
        # waypoint를 어느 프레임에서 읽을지. obs_node의 기본값과 같다.
        #   start_heading — 시작 위치를 원점, 시작 yaw를 +X로 삼는 프레임.
        #                   "0,0,0;3,0,0"이 "기체 정면으로 3 m"를 뜻한다.
        #   ned           — 원시 world 좌표 그대로.
        # 이 구분이 없으면 기체가 90° yaw로 떠 있을 때 정면 주행 미션이
        # 횡방향 주행이 된다(실제 SITL에서 v_d_b가 body -Y로 나왔다).
        self.declare_parameter("waypoint_frame", "start_heading")

        p = self.get_parameter
        wps = _parse_waypoints(str(p("waypoints").value))
        speed = float(p("cruise_speed").value)
        lookahead = float(p("lookahead_dist").value)
        reach = float(p("reach_threshold").value)
        self._state_max_age = float(p("state_max_age_s").value)
        self._waypoint_frame = str(p("waypoint_frame").value)
        if self._waypoint_frame not in ("start_heading", "ned"):
            raise ValueError(
                f"waypoint_frame={self._waypoint_frame!r} — "
                "'start_heading' 또는 'ned'여야 한다")

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
        # LOSGuidance는 heading_mode="straight"/"takeoff_then_align"에서 기준 자세를
        # reset(initial_quat=...) 시점에 잡는다. 호출하지 않으면 identity가 남아,
        # 기체가 90° yaw로 떠 있으면 목표 자세가 identity가 되어 **자세 오차 90°**로
        # 시작한다(실제 SITL에서 z_q 포화 + yaw 토크 -22 N·m 포화로 드러났다).
        # 첫 유효 state에서 한 번 잡는다.
        self._initialized = False
        # NED -> mission frame 회전과 원점. start_heading일 때만 identity가 아니다.
        self._q_ned_to_mission = mu.identity_quat(1, "cpu")
        self._origin_ned = torch.zeros(1, 3)
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

    def _to_mission_quat(self, quat_ned: torch.Tensor) -> torch.Tensor:
        return mu.quat_mul(self._q_ned_to_mission, quat_ned)

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

        if not self._initialized:
            self._origin_ned = pos.clone()
            if self._waypoint_frame == "start_heading":
                yaw0 = mu.yaw_from_quat(quat)
                zero = torch.zeros_like(yaw0)
                self._q_ned_to_mission = mu.quat_from_euler_xyz(zero, zero, -yaw0)
                self.get_logger().info(
                    f"mission frame 확정 — 원점 {pos[0].tolist()}, "
                    f"yaw0 {float(yaw0[0]) * 57.2958:.1f}°")
            else:
                self.get_logger().info("waypoint_frame=ned — 원시 world 좌표를 쓴다")
            # LOSGuidance의 기준 자세도 **mission frame에서** 잡아야 한다.
            self._los.reset(torch.tensor([0]),
                            initial_quat=self._to_mission_quat(quat))
            self._initialized = True

        # 유도는 mission frame에서 계산한다.
        pos_m = mu.quat_apply(self._q_ned_to_mission, pos - self._origin_ned)
        quat_m = self._to_mission_quat(quat)
        v_d_b, q_d_m = self._los.compute(pos_m, quat_m)
        # q_d는 state와 **같은 프레임(NED)**으로 되돌려 보낸다. observation_node가
        # q_e = conj(q_d) x q 를 계산하는데, 두 자세의 프레임이 다르면 그 오차에
        # mission 회전이 통째로 섞인다. v_d_b는 body frame이라 변환이 없다.
        q_d = mu.quat_mul(mu.quat_conjugate(self._q_ned_to_mission), q_d_m)

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
