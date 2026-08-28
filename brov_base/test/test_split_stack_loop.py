"""4노드 루프 통합 시험 — Gazebo 없이 최소 플랜트로 폐루프를 닫는다.

무엇을 시험하는가
=================
실제 rclpy 노드 4개를 띄우고 DDS로 실제 토픽을 주고받으며

    state ──▶ guidance ──▶ desired ──▶ observation ──▶ policy ──▶ wrench ──▶ base
      ▲                                                                       │
      └───────────────────────── 최소 surge 플랜트 ◀───────────────────────────┘

가 닫히는지 본다. Gazebo/ArduSub SITL은 다른 환경에 있으므로, 여기서는 MAVLink
자리에 **실측 항력으로 적분하는 1-DOF surge 플랜트**를 놓는다:

    m_eff * du/dt = tau_x - (Xu*u + Xuu*u|u|)
    Xu = 9.81, Xuu = 145.73     (2026-08-28 수조 실측, 14.5V 보정)
    m_eff = 14.635 + 6.36 kg    (질량 + surge added mass)

이것은 Gazebo 시험을 대체하지 않는다 — 자세/횡방향/부력/센서지연이 전부 없다.
**배선과 계약과 안전이 맞는지**를 보는 것이고, 그것이 노드 분리에서 새로 생긴
위험이 있는 곳이다.
"""

import math
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import torch

from brov_interfaces.msg import BrovState, DesiredState, Observation, Wrench6

from brov_base.base_node import BaseNode
from brov_base.guidance_node import GuidanceNode
from brov_base.observation_node import ObservationNode
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix
from brov_control.policy_wrench_node import PolicyWrenchNode


_ARTIFACT = (
    "artifacts/policies/sim2swim_paperfix_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt"
)
_VEHICLE = "brov_base/brov_base/vendor/brov2_heavy.yaml"
_XU, _XUU, _M_EFF = 9.81, 145.73, 14.635 + 6.36


class _SurgePlant:
    """MAVLink 자리에 놓는 최소 플랜트. PWM을 받아 surge를 적분한다."""

    def __init__(self):
        pos, dir_ = thruster_pos_dir_ned(load_brov2_yaml())
        self._thruster = BROV2ThrusterModel(
            num_envs=1, dt=0.04, device="cpu", pos=pos, dir=dir_, voltage=14.5)
        self._B = build_allocation_matrix(self._thruster._pos, self._thruster._dir)
        self.u = 0.0
        self.x = 0.0
        self._last = None
        self.pwm_count = 0
        self.neutral_calls = 0
        self.armed = False
        self.tau_history = []

    def _advance(self, tau_x: float) -> None:
        now = time.monotonic()
        dt = 0.0 if self._last is None else min(0.1, now - self._last)
        self._last = now
        drag = _XU * self.u + _XUU * self.u * abs(self.u)
        self.u += (tau_x - drag) / _M_EFF * dt
        self.x += self.u * dt

    def send_pwm(self, pwm):
        thrust = self._thruster._table.force(pwm, self._thruster._voltage).reshape(-1)
        tau_x = float((self._B @ thrust)[0])
        self.tau_history.append(tau_x)
        self.pwm_count += 1
        self._advance(tau_x)

    def snapshot(self):
        self._advance(0.0) if self._last is None else None
        return {
            "att_quat_ned": torch.tensor([1.0, 0.0, 0.0, 0.0]),
            "pos_ned": torch.tensor([self.x, 0.0, 0.0]),
            "vel_ned": torch.tensor([self.u, 0.0, 0.0]),
            "body_rates_ned": torch.zeros(3),
            "att_age_s": 0.005, "pos_age_s": 0.005,
        }

    def control_snapshot(self):
        return {"heartbeat_age_s": 0.05, "custom_mode": 19, "armed": self.armed}

    def neutral_stop(self):
        self.neutral_calls += 1
        self._advance(0.0)

    def enable_passthrough(self): pass
    def arm(self): self.armed = True
    def disarm(self): self.armed = False
    def close(self, send_stop=True): pass


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(nodes, seconds: float):
    ex = MultiThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    time.sleep(seconds)
    ex.shutdown()
    t.join(timeout=2.0)


def _build_stack(plant, **guidance_params):
    base = BaseNode(interface=plant)
    base._send_pwm = True
    for k, v in (("waypoints", "0,0,0;3,0,0"), ("cruise_speed", 0.3),
                 ("heading_mode", "straight"), ("lookahead_dist", 1.0)):
        pass  # 파라미터는 노드 생성 전에 override할 수 없으므로 기본값을 쓴다
    guide = GuidanceNode()
    obs = ObservationNode()
    pol = PolicyWrenchNode()
    return base, guide, obs, pol


@pytest.fixture
def _params(monkeypatch):
    """노드 파라미터를 CLI 없이 주입한다."""
    import rclpy.node as rnode
    original = rnode.Node.declare_parameter
    overrides = {
        "policy_path": _ARTIFACT,
        "vehicle_model_path": _VEHICLE,
        "cruise_speed": 0.3,
        "waypoints": "0,0,0;3,0,0",
        "state_rate_hz": 25.0,
    }

    def patched(self, name, value=None, *a, **kw):
        if name in overrides:
            value = overrides[name]
        return original(self, name, value, *a, **kw)

    monkeypatch.setattr(rnode.Node, "declare_parameter", patched)
    return overrides


def test_loop_closes_and_vehicle_accelerates(_params):
    """폐루프가 닫히고 기체가 명령 속도로 가속한다."""
    plant = _SurgePlant()
    base, guide, obs, pol = _build_stack(plant)
    try:
        _spin([base, guide, obs, pol], 6.0)
        assert plant.pwm_count > 50, f"PWM이 거의 안 나갔다 ({plant.pwm_count})"
        assert plant.u > 0.15, f"가속하지 않았다 (u={plant.u:.3f})"
        assert plant.u < 0.45, f"명령(0.3)을 크게 넘었다 (u={plant.u:.3f})"
        assert max(plant.tau_history) > 5.0, "추력이 거의 0이다"
    finally:
        for n in (pol, obs, guide, base):
            n.destroy_node()


def test_vehicle_holds_position_after_reaching_the_final_waypoint(_params):
    """완주 후 종단 유지 — loop=False면 마지막 waypoint에서 멈춘다.

    측정된 거동: 0.3 m/s 명령에 0.294 m/s(98%)로 3 m를 주파한 뒤, reach
    판정이 나면 LOSGuidance가 terminal hold로 넘어가 속도가 0으로 수렴한다.
    속도 목표를 0으로 고정하지 않고 최종 waypoint에 대한 position outer-loop를
    유지하므로, 음성부력이나 외력으로 이탈해도 복귀한다.
    """
    plant = _SurgePlant()
    base, guide, obs, pol = _build_stack(plant)
    try:
        _spin([base, guide, obs, pol], 9.0)
        cruising = plant.u
        assert cruising > 0.25, f"순항 속도가 낮다 (u={cruising:.3f})"
        _spin([base, guide, obs, pol], 5.0)          # waypoint 통과 후
        assert plant.x > 2.5, f"완주하지 못했다 (x={plant.x:.2f})"
        assert plant.u < 0.10, f"완주 후 멈추지 않았다 (u={plant.u:.3f})"
        assert plant.neutral_calls == 0, "정상 완주인데 watchdog이 발동했다"
    finally:
        for n in (pol, obs, guide, base):
            n.destroy_node()


def test_watchdog_stops_when_policy_node_is_killed(_params):
    """정책 노드를 죽이면 base가 중립 정지한다 — 분리가 만든 위험의 대응."""
    plant = _SurgePlant()
    base, guide, obs, pol = _build_stack(plant)
    try:
        _spin([base, guide, obs, pol], 2.0)
        assert plant.pwm_count > 10
        before = plant.neutral_calls
        pol.destroy_node()                    # 정책만 죽인다
        _spin([base, guide, obs], 1.5)        # watchdog 0.25s
        assert plant.neutral_calls > before, "정책이 죽었는데 중립 정지가 없다"
    finally:
        for n in (obs, guide, base):
            n.destroy_node()


def test_policy_refuses_mismatched_observation_contract(_params):
    """계약 불일치면 wrench를 내지 않는다."""
    plant = _SurgePlant()
    base, guide, obs, pol = _build_stack(plant)
    received = []
    probe = rclpy.create_node("probe")
    probe.create_subscription(Wrench6, "/brov/cmd/wrench", received.append, 10)
    try:
        bad = Observation()
        bad.data = [0.0] * 16
        bad.data[0] = 1.0
        bad.contract = "wrong_contract_v9"
        bad.valid = True
        pub = probe.create_publisher(Observation, "/brov/observation", 1)
        for _ in range(5):
            pub.publish(bad)
        _spin([pol, probe], 1.0)
        assert not received, f"계약이 틀렸는데 wrench가 나왔다 ({len(received)}개)"
    finally:
        probe.destroy_node()
        for n in (pol, obs, guide, base):
            n.destroy_node()


def test_topics_carry_matching_sequence_numbers(_params):
    """관측이 어느 state/desired에서 나왔는지 역추적할 수 있다."""
    plant = _SurgePlant()
    base, guide, obs, pol = _build_stack(plant)
    seen = []
    probe = rclpy.create_node("probe_seq")
    probe.create_subscription(Observation, "/brov/observation", seen.append, 50)
    try:
        _spin([base, guide, obs, pol, probe], 2.0)
        valid = [m for m in seen if m.valid]
        assert len(valid) > 20, f"유효 관측이 너무 적다 ({len(valid)})"
        assert all(m.contract == "brov_velocity_observation_v2" for m in valid)
        assert all(m.state_seq > 0 for m in valid[1:])
        dts = [m.integration_dt_s for m in valid[2:]]
        assert 0.02 < sum(dts) / len(dts) < 0.08, f"적분 dt 평균 {sum(dts)/len(dts):.4f}"
    finally:
        probe.destroy_node()
        for n in (pol, obs, guide, base):
            n.destroy_node()
