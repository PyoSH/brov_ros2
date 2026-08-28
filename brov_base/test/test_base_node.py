"""base_node가 새로 떠안은 안전 책임에 대한 회귀 시험.

분리 전에는 명령 경로와 안전 경로가 한 프로세스 안이라 원자적이었다.
분리하면 **정책 노드가 죽어도 base는 마지막 명령을 계속 밀 수 있다.**
watchdog·fault latch·estop이 그 대가를 갚는 장치이므로, 실기 없이 검증한다.
"""

import time

from geometry_msgs.msg import Vector3
import pytest
import rclpy
import torch

from brov_interfaces.msg import Wrench6

from brov_base.base_node import BaseNode


class _FakeInterface:
    """MAVLink 대역. 보낸 PWM과 호출 순서를 기록만 한다."""

    def __init__(self):
        self.sent = []
        self.neutral_calls = 0
        self.armed = False
        self.closed = False
        self._snap = {
            "att_quat_ned": torch.tensor([1.0, 0.0, 0.0, 0.0]),
            "pos_ned": torch.zeros(3),
            "vel_ned": torch.zeros(3),
            "body_rates_ned": torch.zeros(3),
            "att_age_s": 0.01, "pos_age_s": 0.01,
        }

    def snapshot(self): return dict(self._snap)
    def control_snapshot(self):
        return {"heartbeat_age_s": 0.1, "custom_mode": 19, "armed": self.armed}
    def send_pwm(self, pwm): self.sent.append(pwm.clone())
    def neutral_stop(self): self.neutral_calls += 1
    def enable_passthrough(self): pass
    def arm(self): self.armed = True
    def disarm(self): self.armed = False
    def close(self, send_stop=True): self.closed = True


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(**params):
    iface = _FakeInterface()
    node = BaseNode(interface=iface)
    node._send_pwm = True
    for k, v in params.items():
        setattr(node, k, v)
    return node, iface


def _wrench(fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
    m = Wrench6()
    m.force = Vector3(x=fx, y=fy, z=fz)
    m.torque = Vector3(x=tx, y=ty, z=tz)
    return m


def test_pure_surge_wrench_stays_pure_surge_through_allocation():
    """할당은 base가 소유한다 — 순수 surge 명령이 축을 새지 않아야 한다."""
    node, iface = _node()
    node._on_wrench(_wrench(fx=30.0))
    assert len(iface.sent) == 1
    pwm = iface.sent[0]
    assert torch.isfinite(pwm).all() and pwm.shape == (8,)
    delivered = node._thruster._table.force(pwm, node._thruster._voltage).reshape(-1)
    B = torch.linalg.pinv(node._B_pinv)
    w = B @ delivered
    assert abs(float(w[0]) - 30.0) < 0.5          # surge는 전달됨
    assert abs(float(w[1])) < 0.05                # sway 누출 없음
    assert abs(float(w[5])) < 0.05                # yaw 누출 없음
    node.destroy_node()


def test_watchdog_neutral_stops_when_commands_stop():
    """명령이 끊기면 중립 정지 — 분리가 만든 위험을 갚는 유일한 장치."""
    node, iface = _node(_cmd_timeout=0.05)
    node._on_wrench(_wrench(fx=10.0))
    assert iface.neutral_calls == 0
    node._tick()                                   # 아직 안 끊김
    assert iface.neutral_calls == 0
    node._last_cmd_monotonic = time.monotonic() - 0.2
    node._tick()
    assert iface.neutral_calls == 1
    node._tick()                                   # 중복 정지는 안 한다
    assert iface.neutral_calls == 1
    node.destroy_node()


def test_watchdog_does_not_fire_before_first_command():
    """arm 직후 첫 명령을 기다리는 동안 watchdog이 오발동하면 안 된다."""
    node, iface = _node(_cmd_timeout=0.01)
    for _ in range(5):
        node._tick()
    assert iface.neutral_calls == 0
    node.destroy_node()


def test_nonfinite_wrench_trips_latched_fault():
    node, iface = _node()
    node._on_wrench(_wrench(fx=float("nan")))
    assert node._faulted and iface.neutral_calls == 1
    before = len(iface.sent)
    node._on_wrench(_wrench(fx=10.0))              # latch — 정상 명령도 안 받는다
    assert len(iface.sent) == before
    node.destroy_node()


def test_slew_limit_trips_fault():
    node, iface = _node(_max_pwm_delta_per_s=0.1, _first_dt=0.0)
    node._on_wrench(_wrench(fx=1.0))
    node._last_pwm_monotonic = time.monotonic()
    node._on_wrench(_wrench(fx=80.0))              # 같은 순간에 큰 도약
    assert node._faulted
    node.destroy_node()


def test_estop_blocks_further_commands():
    from std_msgs.msg import Empty
    node, iface = _node()
    node._on_estop(Empty())
    before = len(iface.sent)
    node._on_wrench(_wrench(fx=10.0))
    assert len(iface.sent) == before and iface.neutral_calls == 1
    node.destroy_node()


def test_disarm_is_the_only_path_that_clears_the_fault_latch():
    node, iface = _node()
    node._on_wrench(_wrench(fx=float("inf")))
    assert node._faulted
    res = node._srv_disarm(None, type("R", (), {"success": False, "message": ""})())
    assert res.success and not node._faulted
    node._on_wrench(_wrench(fx=10.0))              # 이제 다시 받는다
    assert len(iface.sent) == 1
    node.destroy_node()


def test_arm_is_refused_while_faulted():
    node, _ = _node()
    node._arm_permitted = True
    node._faulted, node._fault_reason = True, "시험"
    res = node._srv_arm(None, type("R", (), {"success": True, "message": ""})())
    assert not res.success and "fault" in res.message
    node.destroy_node()


def test_state_message_reports_stale_telemetry():
    node, iface = _node()
    iface._snap["att_age_s"] = 99.0
    msg = node._read_state()
    assert not msg.valid and "att" in msg.reason
    node.destroy_node()


def test_state_seq_increases_monotonically():
    """소비자가 누락/중복을 감지하는 유일한 수단이다."""
    node, _ = _node()
    seqs = [node._read_state().seq for _ in range(4)]
    assert seqs == sorted(seqs) and len(set(seqs)) == 4
    node.destroy_node()


def test_gazebo_linear_thruster_model_makes_the_round_trip_exact():
    """SITL에서는 요청한 wrench가 그대로 나가야 한다.

    ardupilot_gazebo는 servo PWM을 선형으로 매핑한다:
        cmd_thrust = ((pwm-1100)/800 - 0.5) * multiplier,  multiplier=100 -> ±50 N
    우리가 T200 비선형 테이블을 역변환해 PWM을 만들면 그 합성이 항등이 아니다 --
    요청 10~30 N 구간에서 Gazebo가 1.4~2.1배를 낸다. 모델을 맞추면 정확해진다.
    """
    iface = _FakeInterface()
    node = BaseNode(interface=iface)
    node._send_pwm = True
    node._thruster_model = "gazebo_linear"
    node._gz_half_range = 50.0
    for fx in (5.0, 20.0, 40.0, 80.0):
        iface.sent.clear()
        node._on_wrench(_wrench(fx=fx))
        pwm = iface.sent[0]
        # Gazebo가 실제로 낼 추력 = pwm * half_range
        delivered = pwm * node._gz_half_range
        B = torch.linalg.pinv(node._B_pinv)
        w = B @ delivered
        assert abs(float(w[0]) - fx) < 1e-3, f"fx={fx}: {float(w[0])}"
        assert abs(float(w[1])) < 1e-3 and abs(float(w[5])) < 1e-3
    node.destroy_node()


def test_unknown_thruster_model_is_refused():
    """조용히 기본값으로 넘어가면 SITL/실기를 뒤바꿔도 모른다."""
    import rclpy.node as rnode
    original = rnode.Node.declare_parameter

    def patched(self, name, value=None, *a, **kw):
        if name == "thruster_model":
            value = "made_up"
        return original(self, name, value, *a, **kw)

    rnode.Node.declare_parameter = patched
    try:
        with pytest.raises(ValueError, match="thruster_model"):
            BaseNode(interface=_FakeInterface())
    finally:
        rnode.Node.declare_parameter = original
