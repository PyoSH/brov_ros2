"""dvl_record_node 가 **기록 전용**으로 남는지, 원시값을 변환 없이 내는지.

이 노드의 위험은 조용한 것들이다. 축을 몰래 변환하면 사후 검증이 불가능해지고,
제어 토픽을 하나라도 발행하면 되먹임 경로에 끼어든다. 둘 다 실행 중에는 눈에
띄지 않으므로 시험으로 못박는다.
"""
import pytest
import rclpy

from brov_interfaces.msg import DvlSample

from brov_control.dvl_record_node import DvlRecordNode


class _FakeReader:
    """DvlReader 대역.

    **실물과 같은 시그니처여야 한다.** 처음에 없는 메서드(snapshot)를 흉내내는
    바람에 시험은 통과하고 실물은 첫 tick 에서 죽었다. 아래
    test_fake_matches_the_real_reader_api 가 그 재발을 막는다.
    """

    def __init__(self, *_a, **_kw):
        self.started = False
        self.stopped = False
        self.payload = {
            "dvl_vx": None, "dvl_vy": None, "dvl_vz": None,
            "dvl_valid": False, "dvl_fom": None, "dvl_altitude": None,
            "dvl_beams_valid": None, "dvl_age_s": None,
            "dvl_connected": False, "dvl_error": "",
        }

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def sample(self, *, max_age_s: float = 0.5):
        return dict(self.payload)


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(monkeypatch):
    fake = _FakeReader()
    monkeypatch.setattr("brov_control.dvl_record_node.DvlReader",
                        lambda *a, **k: fake)
    return DvlRecordNode(), fake


def test_publishes_raw_velocity_without_transforming(monkeypatch):
    """DVL 프레임 원시값이 그대로 나가야 한다.

    A50 의 Mounting rotation offset 은 장비 설정에 있고 코드는 모른다. 여기서
    가정을 세워 변환하면 그 가정이 데이터에 박혀 EKF 와의 축·부호 대조가
    불가능해진다. 2026-08-28 수조에서 heave 부호가 반대로 나온 것을 찾아낸 것도
    원시값을 남겼기 때문이다.
    """
    node, fake = _node(monkeypatch)
    got = []
    node.create_subscription(DvlSample, "/brov/dvl/sample",
                             got.append, 10)
    fake.payload.update({"dvl_vx": 0.31, "dvl_vy": -0.02, "dvl_vz": 0.44,
                         "dvl_fom": 0.003, "dvl_altitude": 1.2,
                         "dvl_valid": True, "dvl_beams_valid": 4,
                         "dvl_age_s": 0.02, "dvl_connected": True})
    node._tick()
    rclpy.spin_once(node, timeout_sec=0.5)
    assert got, "발행되지 않았다"
    m = got[-1]
    assert (m.velocity_raw.x, m.velocity_raw.y, m.velocity_raw.z) == \
        pytest.approx((0.31, -0.02, 0.44)), "원시값이 변형됐다"
    assert m.valid and m.connected and m.beams_valid == 4
    node.destroy_node()


def test_invalid_sample_is_flagged_not_hidden(monkeypatch):
    """velocity_valid=false 를 조용히 0 으로 바꾸지 않는다."""
    node, fake = _node(monkeypatch)
    fake.payload.update({"dvl_vx": 0.1, "dvl_vy": 0.0, "dvl_vz": 0.0,
                         "dvl_fom": 9.9, "dvl_altitude": -1.0,
                         "dvl_valid": False, "dvl_beams_valid": 1,
                         "dvl_age_s": 0.02, "dvl_connected": True})
    got = []
    node.create_subscription(DvlSample, "/brov/dvl/sample",
                             got.append, 10)
    node._tick()
    rclpy.spin_once(node, timeout_sec=0.5)
    assert got and not got[-1].valid
    assert got[-1].reason, "무효 사유가 비어 있다"
    node.destroy_node()


def test_disconnected_reports_reason(monkeypatch):
    node, fake = _node(monkeypatch)
    fake.payload.update({"dvl_connected": False, "dvl_error": "연결 거부"})
    got = []
    node.create_subscription(DvlSample, "/brov/dvl/sample",
                             got.append, 10)
    node._tick()
    rclpy.spin_once(node, timeout_sec=0.5)
    assert got
    assert not got[-1].connected and not got[-1].valid
    assert "연결 거부" in got[-1].reason
    node.destroy_node()


def test_publishes_nothing_on_the_control_path(monkeypatch):
    """제어 토픽을 하나도 발행하지 않는다 -- 기록 전용이라는 계약.

    /brov/cmd/wrench 나 /brov/thruster_pwm 에 끼어들면 base_node 의 단일
    발행자 가정이 깨지고, 어느 쪽 명령이 나갔는지 사후에 알 수 없게 된다.
    """
    node, _ = _node(monkeypatch)
    published = {name for name, _types in node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace())}
    forbidden = {"/brov/cmd/wrench", "/brov/thruster_pwm", "/brov/state",
                 "/brov/observation", "/brov/desired"}
    assert not (published & forbidden), f"제어 경로에 발행하고 있다: {published & forbidden}"
    assert "/brov/dvl/sample" in published
    node.destroy_node()


def test_reader_is_stopped_on_shutdown(monkeypatch):
    """노드를 내리면 TCP 연결도 놓아야 한다.

    놓지 않으면 A50 의 클라이언트 슬롯을 계속 물고 있어, 단일 클라이언트만
    받는 경우 BlueOS 의 DVL extension 이 복구되지 못한다.
    """
    node, fake = _node(monkeypatch)
    node.destroy_node()
    assert fake.stopped, "reader.stop() 이 호출되지 않았다"


def test_fake_matches_the_real_reader_api():
    """시험용 대역이 실물 DvlReader 와 같은 시그니처인지.

    이게 없어서 한 번 당했다 -- 대역에 존재하지 않는 메서드(snapshot)를 만들어
    두니 시험 5 건이 전부 통과했는데 실물은 첫 tick 에서 AttributeError 로
    죽었다. 시험이 대역을 시험하고 있었던 것이다.
    """
    import inspect

    from brov_control.dvl_reader import DvlReader

    for name in ("start", "stop", "sample"):
        assert hasattr(DvlReader, name), f"실물에 {name} 이 없다"
        assert hasattr(_FakeReader, name), f"대역에 {name} 이 없다"

    real = inspect.signature(DvlReader.sample)
    fake = inspect.signature(_FakeReader.sample)
    assert set(real.parameters) == set(fake.parameters), (
        f"sample 시그니처가 다르다: 실물 {list(real.parameters)} "
        f"vs 대역 {list(fake.parameters)}")

    # 실물이 내놓는 키를 대역도 전부 가져야 한다. 없는 키를 노드가 읽으면
    # 실기에서만 None 이 되어 조용히 잘못된 값이 나간다.
    reader = DvlReader("127.0.0.1", 1)
    expected = set(reader.sample().keys())
    got = set(_FakeReader().sample().keys())
    assert expected <= got, f"대역에 없는 키: {sorted(expected - got)}"
