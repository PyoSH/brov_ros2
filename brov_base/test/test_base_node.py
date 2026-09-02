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
            # depth sensor 경로 (논문 5.2). SCALED_PRESSURE/2/3 = instance 0/1/2.
            "press_abs_hpa": [1013.25, 1028.0, None],
            "press_age_s": [0.01, 0.01, float("inf")],
            "press_seq": [1, 1, 0],
            # 마커(pool) 정렬이 구독하는 odometry envelope 용. seq 는 링크가
            # 실제로 새 샘플을 줬는지를 뜻한다 -- 올리지 않으면 재발행되지 않는다.
            "odometry_session_id": "boot-a",
            "att_rx_time": 100.0,
            "pos_rx_time": 100.0,
            "att_seq": 1,
            "pos_seq": 1,
        }
        self.params = {"BARO_PRIMARY": 0.0, "BARO_SPEC_GRAV": 1.0}

    def snapshot(self): return dict(self._snap)
    def control_snapshot(self):
        return {"heartbeat_age_s": 0.1, "custom_mode": 19, "armed": self.armed}
    def send_pwm(self, pwm): self.sent.append(pwm.clone())
    def neutral_stop(self): self.neutral_calls += 1
    def enable_passthrough(self): pass
    def get_parameter(self, name, timeout=5.0): return self.params.get(name)
    def request_telemetry_streams(self): self.stream_requests = getattr(self, "stream_requests", 0) + 1
    def rx_counts(self): return {"HEARTBEAT": 5, "ATTITUDE_QUATERNION": 0}
    def last_command_ack(self): return (511, 0)
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
    # 아래 시험들은 할당/watchdog/slew를 보는 것이라 lifecycle 자체는
    # 관심사가 아니다. 게이트는 test_wrench_refused_until_started 와
    # test_control_active_matches_actuation 이 따로 본다.
    node._armed_by_us = True
    node._started = True
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
    # disarm은 latch를 풀 뿐 arm 상태까지 돌려주지 않는다 — 다시 arm 해야 한다.
    node._on_wrench(_wrench(fx=10.0))
    assert iface.sent == [], "disarm 직후에는 아직 구동되면 안 된다"
    node._armed_by_us = True
    node._started = True
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
    node._armed_by_us = True
    node._started = True
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


def test_wrench_refused_until_started():
    """arm 만으로는 부족하다 — start 전에는 wrench 가 추진기에 닿지 않는다.

    2026-08-28 Gazebo SITL 회귀: 이 게이트가 없어 arm 전 RC override 가 나갔고,
    ArduSub 가 `armed=False` 를 보고하는데도 servo 출력이 중립을 벗어나 기체가
    해저(-9.99 m)까지 내려갔다. 같은 동안 observation_node 는 control_active 가
    false 라 적분을 멈추고 있었으므로, 정책은 z_v/z_q 가 영구히 0 인 관측으로
    실제 추진기를 몰았다.
    """
    R = type("R", (), {"success": True, "message": ""})
    node, iface = _node()
    node._armed_by_us = False
    node._started = False
    node._arm_permitted = True
    node._prepared = True

    node._on_wrench(_wrench(fx=30.0))
    assert iface.sent == [], "arm 전에 PWM 이 나갔다"

    assert node._srv_arm(None, R()).success
    node._on_wrench(_wrench(fx=30.0))
    assert iface.sent == [], "start 전에 PWM 이 나갔다"

    assert node._srv_start(None, R()).success
    node._on_wrench(_wrench(fx=30.0))
    assert len(iface.sent) == 1, "start 후에는 나가야 한다"


def test_stop_freezes_control_but_keeps_arm():
    """stop 은 제어만 멈춘다 — armed 와 prepared 는 유지된다.

    합쳐 두면 제어를 멈추려 할 때 disarm 밖에 길이 없고, 그러면 passthrough 가
    원복돼 재개하려면 prepare 부터 다시 해야 한다.
    """
    R = type("R", (), {"success": True, "message": ""})
    node, iface = _node()
    node._prepared = True
    node._on_wrench(_wrench(fx=30.0))
    assert len(iface.sent) == 1

    assert node._srv_stop(None, R()).success
    assert node._armed_by_us and node._prepared, "stop 이 arm/prepare 까지 풀었다"
    before = len(iface.sent)
    node._on_wrench(_wrench(fx=30.0))
    assert len(iface.sent) == before, "stop 뒤에도 PWM 이 나갔다"

    assert node._srv_start(None, R()).success   # prepare 없이 재개된다
    node._on_wrench(_wrench(fx=30.0))
    assert len(iface.sent) == before + 1


def test_start_requires_prepare_and_arm():
    R = type("R", (), {"success": True, "message": ""})
    node, _ = _node()
    node._armed_by_us = False
    node._started = False
    node._prepared = False
    res = node._srv_start(None, R())
    assert not res.success and "prepare" in res.message

    node._prepared = True
    res = node._srv_start(None, R())
    assert not res.success and "arm" in res.message

    node._armed_by_us = True
    assert node._srv_start(None, R()).success and node._started


def test_control_active_matches_actuation():
    """발행하는 control_active 와 실제 구동 여부가 어긋나면 안 된다.

    observation_node 가 이 신호로 적분을 gate 하고 guidance_node 가 이 신호로
    mission frame 원점을 잡으므로, 신호가 거짓이면 둘 다 잘못된 기준으로 돈다.
    """
    node, iface = _node()
    for armed in (False, True):
        for started in (False, True):
            node._armed_by_us, node._started = armed, started
            iface.sent.clear()
            node._on_wrench(_wrench(fx=20.0))
            actuated = len(iface.sent) > 0
            active = bool(node._send_pwm and node._armed_by_us and node._started
                          and not node._faulted and not node._estopped)
            assert actuated == active, (
                f"armed={armed} started={started}: 구동={actuated}, 발행={active}")


def test_arm_refused_without_prepare():
    """prepare 없이 arm 하면 거절한다.

    prepare 가 SERVO1~8 을 RCPassThru 로 바꾼다. 그게 없으면 ArduSub 가 우리
    추진기 PWM 을 조종사 입력으로 해석해 자체 믹싱을 돌린다 — 할당이 통째로
    다른 것이 된다. 2026-08-28 SITL 에서 prepare 실패 + arm 성공 조합으로
    40 m 미션이 두 번 해저 침강했다.
    """
    R = type("R", (), {"success": True, "message": ""})
    node, _ = _node()
    node._armed_by_us = False
    node._arm_permitted = True
    node._prepared = False
    res = node._srv_arm(None, R())
    assert not res.success and "prepare" in res.message
    assert not node._armed_by_us

    node._prepared = True
    res = node._srv_arm(None, R())
    assert res.success and node._armed_by_us


def test_lifecycle_services_use_legacy_names():
    """legacy 스택(demo_orchestrator)이 그대로 구동할 수 있어야 한다."""
    node, _ = _node()
    names = {n for n, _t in node.get_service_names_and_types()}
    for expected in ("/brov/prepare_control", "/brov/arm_control",
                     "/brov/start_control", "/brov/stop_control",
                     "/brov/disarm_control"):
        assert expected in names, f"{expected} 가 없다 — orchestrator 가 못 붙는다"


def _prepared(node, iface):
    R = type("R", (), {"success": True, "message": ""})
    res = node._srv_prepare(None, R())
    assert res.success, res.message
    return R


def test_depth_comes_from_the_baro_primary_instance():
    """어느 SCALED_PRESSURE 인지 추측하지 않는다 -- BARO_PRIMARY 가 정한다.

    ArduSub 는 init 에서 BARO_TYPE_WATER 인 첫 instance 를 primary 로
    set_and_save 한다(ArduSub/system.cpp:108, AP_Baro.h:181). SITL 은 모든
    baro 가 WATER 라(AP_Baro_SITL.cpp:21) 응답만으로는 구분되지 않으므로
    이 파라미터가 유일하게 확실한 근거다.
    """
    node, iface = _node(_depth_source="pressure")
    node._armed_by_us = False
    node._started = False
    node._arm_permitted = True
    iface.params["BARO_PRIMARY"] = 1.0            # instance 1 = SCALED_PRESSURE2
    R = _prepared(node, iface)
    assert node._depth_baro_instance == 1

    assert node._srv_arm(None, R()).success
    assert node._srv_start(None, R()).success
    # 기준압은 instance 1 의 값(1028.0 hPa)이어야 한다
    assert abs(node._depth_ref_pa - 102800.0) < 1e-6

    # 1 m 하강 = +9800 Pa (SPEC_GRAV 1.0). NED 는 아래가 양이므로 z = +1.0
    iface._snap["press_abs_hpa"] = [1013.25, 1028.0 + 98.0, None]
    msg = node._read_state()
    assert msg.valid, msg.reason
    assert abs(msg.position.z - 1.0) < 1e-3, msg.position.z
    assert msg.depth_source == "pressure" and msg.depth_baro_instance == 1


def test_depth_reference_cancels_constant_sensor_bias():
    """상수 편의는 기준압 차감으로 정확히 상쇄된다.

    SITL instance 1 은 +15 hPa 편의가 있었다. 절대 변환을 쓰면 0.15 m 오차가
    그대로 남지만, start 시점 기준압을 빼면 사라진다.
    """
    node, iface = _node(_depth_source="pressure")
    node._armed_by_us = False
    node._started = False
    node._arm_permitted = True
    iface._snap["press_abs_hpa"] = [1013.25, 1013.25 + 15.0, None]   # +15 hPa 편의
    iface.params["BARO_PRIMARY"] = 1.0
    R = _prepared(node, iface)
    assert node._srv_arm(None, R()).success
    assert node._srv_start(None, R()).success

    msg = node._read_state()
    assert abs(msg.position.z) < 1e-6, f"기준 시점 깊이가 0 이 아니다: {msg.position.z}"


def test_pressure_depth_refuses_when_parameters_unreadable():
    """BARO_PRIMARY 를 못 읽으면 prepare 가 거절한다 -- 조용히 EKF 로 넘어가지 않는다."""
    node, iface = _node(_depth_source="pressure")
    iface.params.pop("BARO_PRIMARY")
    R = type("R", (), {"success": True, "message": ""})
    res = node._srv_prepare(None, R())
    assert not res.success and "BARO_PRIMARY" in res.message


def test_stale_pressure_invalidates_state():
    """압력이 stale 이면 state 를 무효로 만든다 -- 얼어붙은 깊이로 조향하지 않는다."""
    node, iface = _node(_depth_source="pressure")
    node._armed_by_us = False
    node._started = False
    node._arm_permitted = True
    R = _prepared(node, iface)
    assert node._srv_arm(None, R()).success
    assert node._srv_start(None, R()).success
    iface._snap["press_age_s"] = [99.0, 99.0, float("inf")]
    msg = node._read_state()
    assert not msg.valid and "stale" in msg.reason


def test_default_depth_source_is_still_ekf():
    """GT 교차검증 전에는 기본 동작을 바꾸지 않는다."""
    node, iface = _node()
    msg = node._read_state()
    assert msg.depth_source == "mavlink_ekf" and msg.depth_baro_instance == -1


# ---------------------------------------------------------------- odometry
# 마커(pool) 정렬은 `/brov/odometry/local_with_session` 하나만 구독한다. 분리
# 스택에서 그것을 내는 곳이 여기뿐이므로(MAVLink 를 두 프로세스가 열 수 없다),
# 끊기면 절대 좌표 주행이 통째로 불가능해진다.
def _captured(node):
    captured = {}
    for attr, key in (("_pub_odom_with_session", "envelope"),
                      ("_pub_odom", "odometry"),
                      ("_pub_odom_session", "session"),
                      ("_pub_ahrs", "ahrs"),
                      ("_pub_depth_ekf", "depth_ekf")):
        publisher = getattr(node, attr)
        captured[key] = []
        publisher.publish = captured[key].append
    captured["pressure"] = [[], [], []]
    for i, publisher in enumerate(node._pub_pressure):
        publisher.publish = captured["pressure"][i].append
    return captured


def test_odometry_envelope_carries_the_session_pool_alignment_needs():
    node, _ = _node()
    captured = _captured(node)
    node._tick()
    assert len(captured["envelope"]) == 1
    envelope = captured["envelope"][0]
    assert envelope.odometry_session_id == "boot-a:nav0"
    assert envelope.odometry.header.frame_id == "odom"
    assert envelope.odometry.child_frame_id == "base_link"
    # 새 telemetry 가 없으면 재발행하지 않는다 -- 같은 샘플을 두 번 세면
    # pool_alignment 의 "정지 상태 20 표본" 조건이 가짜로 채워진다.
    node._tick()
    assert len(captured["envelope"]) == 1
    # session id 는 latched 로 한 번만 낸다.
    assert len(captured["session"]) == 1


def test_odometry_converts_ned_to_the_zup_frame_alignment_expects():
    """NED -> odom(Z-up)이 실제로 일어난다. 부호가 살아 있으면 정렬이 뒤집힌다."""
    node, iface = _node()
    iface._snap["pos_ned"] = torch.tensor([1.0, 2.0, 3.0])
    captured = _captured(node)
    node._tick()
    assert captured["odometry"], "새 세션의 첫 샘플은 발행돼야 한다"
    position = captured["odometry"][0].pose.pose.position
    assert (position.x, position.y, position.z) == (1.0, -2.0, -3.0)


def test_position_jump_advances_the_session_so_alignment_invalidates():
    """EKF 원점 리셋은 boot time 을 바꾸지 않는다. 인접 샘플 검사가 유일한 창이다."""
    node, iface = _node()
    captured = _captured(node)
    node._tick()
    assert captured["envelope"][0].odometry_session_id == "boot-a:nav0"
    iface._snap["pos_ned"] = torch.tensor([0.0, 0.0, 5.0])
    iface._snap["att_rx_time"] = 100.1
    iface._snap["pos_rx_time"] = 100.1
    iface._snap["pos_seq"] = 2
    node._tick()
    assert captured["envelope"][1].odometry_session_id == "boot-a:nav1"
    # fault 를 latch 하지는 않는다 -- start_heading 주행은 절대 프레임에
    # 의존하지 않으므로 계속 돌아야 한다.
    assert not node._faulted


def test_odometry_is_not_published_without_a_session_id():
    node, iface = _node()
    iface._snap["odometry_session_id"] = ""
    captured = _captured(node)
    node._tick()
    assert captured["envelope"] == []


# ------------------------------------------------------------------ 센서
# `/brov/state` 는 depth_source 가 **고른** 경로 하나만 싣는다. 깊이 게이트와
# dead time 분석은 고르지 않은 쪽도 있어야 성립하므로 원시값을 따로 낸다.
def test_raw_sensor_topics_carry_both_depth_paths():
    node, _ = _node()
    captured = _captured(node)
    node._tick()
    assert len(captured["ahrs"]) == 1
    assert len(captured["depth_ekf"]) == 1
    # instance 2 는 미수신이므로 내지 않는다.
    assert [len(x) for x in captured["pressure"]] == [1, 1, 0]
    assert captured["pressure"][1][0].fluid_pressure == pytest.approx(102800.0)


def test_ekf_depth_is_still_published_when_the_pressure_path_is_selected():
    """어느 쪽을 골랐든 두 경로가 같은 bag 에 남아야 사후 판정이 된다."""
    node, iface = _node(_depth_source="pressure")
    node._armed_by_us = False
    node._started = False
    node._arm_permitted = True
    R = _prepared(node, iface)
    assert node._srv_arm(None, R()).success
    assert node._srv_start(None, R()).success
    iface._snap["pos_ned"] = torch.tensor([0.0, 0.0, -0.42])
    captured = _captured(node)
    node._tick()
    assert captured["depth_ekf"][0].data == pytest.approx(-0.42)


def test_raw_topics_are_republished_only_on_a_new_sample():
    """느린 링크를 25 Hz 로 복제하면 bag 이 거짓 갱신으로 차고 hz 가 링크 주기를
    감춘다 -- 원시 토픽의 목적이 정확히 그것을 보는 것이다."""
    node, iface = _node()
    captured = _captured(node)
    node._tick()
    node._tick()
    assert [len(x) for x in captured["pressure"]] == [1, 1, 0]
    assert len(captured["ahrs"]) == 1
    assert len(captured["depth_ekf"]) == 1

    iface._snap["press_seq"] = [2, 1, 0]
    iface._snap["att_seq"] = 2
    node._tick()
    assert [len(x) for x in captured["pressure"]] == [2, 1, 0]
    assert len(captured["ahrs"]) == 2
    # LOCAL_POSITION_NED 는 안 왔다.
    assert len(captured["depth_ekf"]) == 1


def test_watchdog_stays_quiet_after_an_explicit_stop():
    """stop 뒤 watchdog 이 한 번 더 '중립 정지' 를 찍지 않는다 -- 실기 로그 오독 방지."""
    node, iface = _node()
    node._on_wrench(_wrench(fx=5.0))             # 마지막 명령 시각이 생긴다
    R = type("R", (), {"success": True, "message": ""})
    assert node._srv_stop(None, R()).success
    calls = iface.neutral_calls
    assert node._last_cmd_monotonic is None
    time.sleep(0.3)
    node._tick()
    assert iface.neutral_calls == calls, "stop 뒤 watchdog 이 다시 중립 정지를 불렀다"


def test_request_streams_service_resends_and_reports_what_arrives():
    """heartbeat 만 오는 링크는 재실행이 아니라 재요청으로 푼다."""
    node, iface = _node()
    R = type("R", (), {"success": True, "message": ""})
    res = node._srv_request_streams(None, R())
    assert res.success and iface.stream_requests == 1
    assert "HEARTBEAT×5" in res.message
    assert "안 옴: ATTITUDE_QUATERNION" in res.message
    assert "ACK cmd=511 result=0(수락)" in res.message
