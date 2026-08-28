#!/usr/bin/env python3
"""brov_observation_node — 상태와 목표를 16-D 관측으로, 적분을 단독 소유.

역할 분리에서 이 노드가 소유하는 것
====================================
  sub  /brov/state    BrovState
  sub  /brov/desired  DesiredState
  pub  /brov/observation  Observation
  srv  /brov/observation/reset_integrator

**적분 상태 z_v, z_q를 단독 소유한다.** 여러 노드가 각자 적분하면 주기가
갈라져 드리프트한다. 정책 주기와 적분 주기가 다르면 학습 계약 위반이므로
`integration_dt_s`를 메시지에 실어 소비자가 검사하게 한다.

프레임을 몰라도 되는 이유
=========================
관측 16-D `[q_e, v_e_b, ω_b, z_v, z_q]`에 **위치가 들어가지 않는다.** 그리고
`q_e = conj(q_d) ⊗ q`는 두 자세에 걸린 공통 프레임 회전이 소거되고, `v_e_b`는
양쪽 다 body frame이다. 따라서 이 관측은 **world frame 선택에 불변**이고,
origin/waypoint frame은 guidance 노드만 소유하면 된다.

동기화
======
state와 desired는 서로 다른 노드에서 오므로 짝이 맞지 않을 수 있다. 여기서는
**state를 트리거로 삼고 가장 최근 desired를 쓴다** — 그 desired가
`desired_max_age_s`보다 오래됐으면 관측을 `valid=False`로 낸다. 정책은 그때
명령을 내지 않고, base watchdog이 중립 정지한다. 실패가 안전한 방향으로 전파된다.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import torch

from brov_interfaces.msg import BrovState, DesiredState, Observation

from brov_base.observation import ObservationBuilder


_CONTRACT = "brov_velocity_observation_v2"


class ObservationNode(Node):
    def __init__(self) -> None:
        super().__init__("brov_observation")

        self.declare_parameter("integral_vel_limit", 5.0)
        self.declare_parameter("integral_att_limit", 5.0)
        self.declare_parameter("max_integration_dt_s", 0.15)
        self.declare_parameter("desired_max_age_s", 0.2)
        p = self.get_parameter

        self._builder = ObservationBuilder(
            device="cpu",
            integral_vel_limit=float(p("integral_vel_limit").value),
            integral_att_limit=float(p("integral_att_limit").value),
        )
        self._max_dt = float(p("max_integration_dt_s").value)
        self._desired_max_age = float(p("desired_max_age_s").value)

        # 제어 루프가 닫혀 있을 때만 적분한다. 열린 채로 적분하면 기체가 움직이지
        # 않는 동안 v_e = -v_d 가 계속 쌓여, arm 하는 순간 z_v가 이미 clamp에 붙어
        # 있다(실제 SITL에서 확인). 관측은 계속 내되 적분만 멈춘다 -- 그래야
        # 운용자가 arm 전에 q_e/v_e를 볼 수 있다.
        self._control_active = False
        self._desired: DesiredState | None = None
        self._desired_rx_ns: int | None = None
        self._last_state_ns: int | None = None
        self._seq = 0

        self._pub = self.create_publisher(Observation, "/brov/observation", 1)
        self.create_subscription(DesiredState, "/brov/desired", self._on_desired, 1)
        self.create_subscription(Bool, "/brov/control_active", self._on_active, 1)
        self.create_subscription(BrovState, "/brov/state", self._on_state, 1)
        # legacy 스택은 `/brov/reset_integrator` 를 쓴다. 같은 콜백을 두 이름에
        # 걸어 demo_orchestrator 와 기존 운용 절차가 그대로 통하게 한다.
        for _name in ("/brov/observation/reset_integrator", "/brov/reset_integrator"):
            self.create_service(Trigger, _name, self._srv_reset)
        self.get_logger().info(
            f"observation_node 시작 — contract {_CONTRACT}, "
            f"clamp z_v±{p('integral_vel_limit').value} z_q±{p('integral_att_limit').value}"
        )

    def _on_active(self, msg: Bool) -> None:
        if msg.data and not self._control_active:
            # 루프가 닫히는 순간 적분을 0에서 시작한다. 열려 있는 동안 쌓인 값은
            # 물리적 의미가 없다(기체가 명령을 받지 않았으므로).
            self._builder.reset_integrators()
            self.get_logger().info("제어 활성 — 적분 초기화 후 시작")
        elif not msg.data and self._control_active:
            self.get_logger().info("제어 비활성 — 적분 정지")
        self._control_active = bool(msg.data)

    def _on_desired(self, msg: DesiredState) -> None:
        self._desired = msg
        self._desired_rx_ns = self.get_clock().now().nanoseconds

    def _srv_reset(self, _req, res):
        self._builder.reset_integrators()
        res.success, res.message = True, "z_v, z_q = 0"
        return res

    def _emit_invalid(self, reason: str, state_seq: int) -> None:
        out = Observation()
        out.header.stamp = self.get_clock().now().to_msg()
        out.seq = self._seq
        self._seq += 1
        out.contract = _CONTRACT
        out.state_seq = state_seq
        out.valid = False
        out.reason = reason
        self._pub.publish(out)

    def _on_state(self, state: BrovState) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if not state.valid:
            self._emit_invalid(f"state invalid: {state.reason}", state.seq)
            self._last_state_ns = now_ns
            return
        if self._desired is None:
            self._emit_invalid("desired 미수신", state.seq)
            self._last_state_ns = now_ns
            return
        age = (now_ns - self._desired_rx_ns) * 1e-9
        if age > self._desired_max_age:
            self._emit_invalid(f"desired stale {age:.3f}s", state.seq)
            self._last_state_ns = now_ns
            return

        dt = 0.0 if self._last_state_ns is None else (now_ns - self._last_state_ns) * 1e-9
        self._last_state_ns = now_ns
        # dt가 한계를 넘으면 **적분하지 않는다.** 한 번의 긴 공백이 적분에
        # 큰 계단을 넣으면 그 뒤 수십 초를 오염시킨다.
        integrate = self._control_active and 0.0 < dt <= self._max_dt
        if self._control_active and dt > self._max_dt:
            self.get_logger().warn(f"dt {dt:.3f}s > {self._max_dt}s — 이번 스텝은 적분 생략")

        q = torch.tensor([state.attitude.w, state.attitude.x,
                          state.attitude.y, state.attitude.z], dtype=torch.float32)
        omega = torch.tensor([state.angular_velocity.x, state.angular_velocity.y,
                              state.angular_velocity.z], dtype=torch.float32)
        v_body = torch.tensor([state.linear_velocity.x, state.linear_velocity.y,
                               state.linear_velocity.z], dtype=torch.float32)
        d = self._desired
        v_d_b = torch.tensor([d.velocity_body.x, d.velocity_body.y, d.velocity_body.z],
                             dtype=torch.float32)
        q_d = torch.tensor([d.attitude.w, d.attitude.x, d.attitude.y, d.attitude.z],
                           dtype=torch.float32)

        # build_from_desired는 world frame 속도를 받아 body로 회전한다.
        # BrovState는 이미 body 속도를 싣고 오므로 되돌려 넣는다 — 한 벌뿐인
        # 수식을 재사용하기 위한 왕복이며, 회전은 정확히 역이라 손실이 없다.
        from brov_base import math_utils as mu
        vel_world = mu.quat_apply(q.unsqueeze(0), v_body.unsqueeze(0)).reshape(3)

        obs, debug = self._builder.build_from_desired(
            q_frame=q, body_rates_ned=omega, vel_frame=vel_world,
            v_d_b_ned=v_d_b, q_d_ned=q_d, dt=dt, integrate=integrate,
        )

        out = Observation()
        out.header.stamp = self.get_clock().now().to_msg()
        out.seq = self._seq
        self._seq += 1
        out.data = [float(v) for v in obs]
        out.contract = _CONTRACT
        out.state_seq = state.seq
        out.desired_seq = d.seq
        out.integration_dt_s = float(dt)
        out.valid = True
        out.reason = "integrator clamped" if debug["integrator_clamped"] else ""
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
