#!/usr/bin/env python3
"""brov_policy_wrench_node — 관측을 wrench로. 아티팩트 계약을 단독 소유.

역할 분리에서 이 노드가 소유하는 것
====================================
  sub  /brov/observation  Observation
  pub  /brov/cmd/wrench   Wrench6 (SNAME/FRD)

**아티팩트 고유 지식 전부**다 — TorchScript 정책, 계약 검증, `wrench_scale`,
FLU/Z-up → SNAME/FRD 부호 변환(T6), action clamp.

**액추에이터를 모른다.** 할당행렬도, 추력 테이블도, PWM도, 전압도 여기 없다 —
전부 `brov_base_node`의 것이다. 그래서 이 노드는
`model_based_controller_node`로 **교체 하나로 대체된다**(같은 토픽에 같은
Wrench6를 낸다). 정책 대 모델기반 A/B가 launch 인자 하나가 된다.

기존 `policy_node.py`와의 차이
==============================
`policy_node.py`(356줄)는 관측을 `Float32MultiArray`로 받아 정책·할당·PWM을
모두 한다. 이 노드는 **타입 있는 `Observation`을 받고 wrench에서 멈춘다.**
`policy_node.py`는 배포 검증을 마친 경로라 그대로 남긴다 — 이관이 끝날 때까지
**둘을 동시에 띄우면 안 된다**(base에 wrench와 PWM이 동시에 들어간다).

wire에서 계약을 강제하는 이유
=============================
한 프로세스였을 때 관측 계약은 암묵적이었다. 분리하면 잘못된 관측 노드가
붙어도 아무도 못 막는다. `Observation.contract`가 아티팩트 metadata의
`observation_contract`와 **문자열 동일**해야 하고, 다르면 명령을 내지 않는다.
"""

from __future__ import annotations

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
import torch

from brov_interfaces.msg import Observation, Wrench6

from brov_control.policy_contract import (
    MK2_ACTION_CONTRACT,
    WRENCH_SCALE,
    action_to_allocation_multiplier,
    resolve_policy_artifact_contract,
)
from brov_control.policy_runner import PolicyRunner


class PolicyWrenchNode(Node):
    def __init__(self) -> None:
        super().__init__("brov_policy_wrench")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("metadata_path", "")
        self.declare_parameter("vehicle_model_path", "")
        self.declare_parameter("expected_policy_dt_s", 0.04)
        self.declare_parameter("max_dt_deviation", 0.5)
        self.declare_parameter("action_abs_limit", [1.0] * 6)
        p = self.get_parameter

        policy_path = str(p("policy_path").value)
        if not policy_path:
            raise ValueError("policy_path 파라미터가 필요하다")

        self._contract = resolve_policy_artifact_contract(
            policy_path,
            requested_action_contract=MK2_ACTION_CONTRACT,
            metadata_path=str(p("metadata_path").value) or None,
            vehicle_model_path=str(p("vehicle_model_path").value) or None,
        )
        self._expected_obs_contract = self._contract.observation_contract
        self._policy = PolicyRunner(policy_path, device="cpu")
        self._to_sname = action_to_allocation_multiplier(MK2_ACTION_CONTRACT)
        self._scale = torch.tensor(WRENCH_SCALE, dtype=torch.float32)
        self._limit = torch.tensor(
            [float(v) for v in p("action_abs_limit").value], dtype=torch.float32)
        if self._limit.numel() != 6 or not torch.isfinite(self._limit).all():
            raise ValueError("action_abs_limit은 유한한 6개 값이어야 한다")

        self._expected_dt = float(p("expected_policy_dt_s").value)
        self._dt_tol = float(p("max_dt_deviation").value)
        self._seq = 0
        self._last_obs_seq: int | None = None
        self._contract_logged = False

        self._pub = self.create_publisher(Wrench6, "/brov/cmd/wrench", 1)
        self.create_subscription(Observation, "/brov/observation", self._on_obs, 1)
        self.get_logger().info(
            f"policy_wrench_node 시작 — profile {self._contract.profile}, "
            f"obs contract {self._expected_obs_contract}, "
            f"policy sha {self._contract.policy_sha256[:12]}"
        )

    def _on_obs(self, msg: Observation) -> None:
        # ── 계약: 다르면 명령을 내지 않는다 (조용히 넘어가면 안 된다) ──
        if msg.contract != self._expected_obs_contract:
            if not self._contract_logged:
                self.get_logger().error(
                    f"관측 계약 불일치 — wire {msg.contract!r} != "
                    f"artifact {self._expected_obs_contract!r}. 명령을 내지 않는다")
                self._contract_logged = True
            return
        if not msg.valid:
            return
        if len(msg.data) != 16:
            self.get_logger().error(f"관측 차원 {len(msg.data)} != 16")
            return

        # ── 적분 주기 검사 ──
        # 관측 노드가 정책 주기와 다른 간격으로 적분했다면 학습 계약 위반이다.
        # 명령은 내되(정지가 더 위험할 수 있다) 소리를 낸다.
        dt = msg.integration_dt_s
        if dt > 0.0 and abs(dt - self._expected_dt) > self._dt_tol * self._expected_dt:
            self.get_logger().warn(
                f"적분 dt {dt:.4f}s가 기대값 {self._expected_dt:.4f}s에서 벗어남")

        if self._last_obs_seq is not None and msg.seq != self._last_obs_seq + 1:
            self.get_logger().warn(
                f"관측 seq 도약 {self._last_obs_seq} → {msg.seq}")
        self._last_obs_seq = msg.seq

        obs = torch.tensor(msg.data, dtype=torch.float32)
        if not torch.isfinite(obs).all():
            self.get_logger().error("관측에 NaN/Inf — 명령을 내지 않는다")
            return

        action = self._policy.act(obs).clamp(-self._limit, self._limit)
        # FLU/Z-up 정책 출력 → SNAME/FRD wrench. T6 = diag(1,-1,-1,1,-1,-1)이며
        # det=+1인 진짜 회전이라 모멘트도 벡터처럼 변환된다.
        wrench = action * self._scale * self._to_sname

        out = Wrench6()
        out.header.stamp = self.get_clock().now().to_msg()
        out.seq = self._seq
        self._seq += 1
        out.force = Vector3(x=float(wrench[0]), y=float(wrench[1]), z=float(wrench[2]))
        out.torque = Vector3(x=float(wrench[3]), y=float(wrench[4]), z=float(wrench[5]))
        out.source = "policy"
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PolicyWrenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
