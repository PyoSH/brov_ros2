"""MAVLink boot-clock, session, and EKF health tests."""

import pytest
import torch

from brov_base.mavlink_interface import RealRobotInterface
from brov_base.mavlink_time import (
    BootTimeDisposition,
    MavlinkBootTimeTracker,
    normalize_time_boot_ms,
)


def test_time_boot_validation_is_fail_closed():
    assert normalize_time_boot_ms(123) == 123
    assert normalize_time_boot_ms(None) is None
    assert normalize_time_boot_ms(True) is None
    assert normalize_time_boot_ms(-1) is None
    assert normalize_time_boot_ms(1 << 32) is None
    assert normalize_time_boot_ms(1.5) is None

    result = MavlinkBootTimeTracker().observe("attitude", None, 1.0)
    assert result.disposition is BootTimeDisposition.INVALID
    assert result.accept_payload is False


def test_small_packet_reorder_is_explicitly_dropped():
    tracker = MavlinkBootTimeTracker(reorder_tolerance_ms=250)

    first = tracker.observe("attitude", 10_000, 1.0)
    reordered = tracker.observe("attitude", 9_900, 1.1)
    next_sample = tracker.observe("attitude", 10_040, 1.2)

    assert first.disposition is BootTimeDisposition.ACCEPT
    assert reordered.disposition is BootTimeDisposition.DROP_REORDERED
    assert reordered.accept_payload is False
    assert next_sample.disposition is BootTimeDisposition.ACCEPT
    assert tracker.reset_detected is False
    assert tracker.reset_count == 0


def test_uint32_wrap_is_explicit_and_not_an_autopilot_reset():
    tracker = MavlinkBootTimeTracker()

    tracker.observe("attitude", (1 << 32) - 20, 1.0)
    result = tracker.observe("attitude", 25, 1.1)

    assert result.disposition is BootTimeDisposition.WRAP
    assert result.accept_payload is True
    assert tracker.wrap_count == 1
    assert tracker.reset_detected is False
    assert tracker.reset_count == 0


def test_reset_requires_two_streams_and_changes_epoch_once():
    tracker = MavlinkBootTimeTracker(reset_coalesce_window_s=2.0)
    tracker.observe("attitude", 50_000, 1.0)
    tracker.observe("local_position", 50_020, 1.0)

    candidate = tracker.observe("attitude", 100, 10.0)
    confirmed = tracker.observe("local_position", 120, 10.1)

    assert candidate.disposition is BootTimeDisposition.RESET_CANDIDATE
    assert candidate.accept_payload is False
    assert confirmed.disposition is BootTimeDisposition.RESET
    assert confirmed.accept_payload is True
    assert tracker.reset_detected is True
    assert tracker.reset_count == 1
    assert tracker.last_reset_rx_time == 10.1

    # Both streams now belong to the new epoch and normal traffic is accepted.
    assert (
        tracker.observe("attitude", 140, 10.2).disposition
        is BootTimeDisposition.ACCEPT
    )


def test_single_delayed_old_packet_cannot_change_session():
    tracker = MavlinkBootTimeTracker()
    tracker.observe("attitude", 3_600_000, 1.0)
    tracker.observe("local_position", 3_600_020, 1.0)

    delayed = tracker.observe("attitude", 100_000, 1.1)
    recovered = tracker.observe("attitude", 3_600_040, 1.2)

    assert delayed.disposition is BootTimeDisposition.RESET_CANDIDATE
    assert delayed.accept_payload is False
    assert recovered.disposition is BootTimeDisposition.ACCEPT
    assert tracker.reset_count == 0


def test_far_into_boot_regression_is_dropped_not_promoted_to_reset():
    tracker = MavlinkBootTimeTracker(reset_candidate_max_boot_ms=300_000)
    tracker.observe("attitude", 4_000_000, 1.0)
    tracker.observe("local_position", 4_000_020, 1.0)

    attitude = tracker.observe("attitude", 2_000_000, 1.1)
    position = tracker.observe("local_position", 2_000_020, 1.2)

    assert attitude.disposition is BootTimeDisposition.DROP_REORDERED
    assert position.disposition is BootTimeDisposition.DROP_REORDERED
    assert tracker.reset_count == 0


def test_old_epoch_high_packet_is_dropped_after_confirmed_reset():
    tracker = MavlinkBootTimeTracker(max_forward_jump_ms=1_000)
    tracker.observe("attitude", 50_000, 1.0)
    tracker.observe("local_position", 50_020, 1.0)
    tracker.observe("attitude", 100, 2.0)
    tracker.observe("local_position", 120, 2.1)

    stale = tracker.observe("attitude", 50_100, 2.2)
    current = tracker.observe("attitude", 200, 2.3)

    assert stale.disposition is BootTimeDisposition.DROP_REORDERED
    assert stale.accept_payload is False
    assert current.disposition is BootTimeDisposition.ACCEPT
    assert tracker.reset_count == 1


def _ready_interface() -> RealRobotInterface:
    interface = RealRobotInterface("unused:test")
    interface._att_quat_ned = torch.tensor([1.0, 0.0, 0.0, 0.0])
    interface._body_rates_ned = torch.zeros(3)
    interface._pos_ned = torch.zeros(3)
    interface._vel_ned = torch.zeros(3)
    interface._att_time_boot_ms = 100
    interface._pos_time_boot_ms = 120
    return interface


def test_snapshot_adds_metadata_without_removing_existing_fields():
    interface = _ready_interface()
    snapshot = interface.snapshot()

    assert snapshot is not None
    assert snapshot["att_time_boot_ms"] == 100
    assert snapshot["pos_time_boot_ms"] == 120
    assert snapshot["odometry_session_id"] == interface.odometry_session_id
    assert snapshot["mavlink_time_reset_detected"] is False
    assert snapshot["mavlink_time_reset_count"] == 0
    for existing in (
        "att_quat_ned",
        "body_rates_ned",
        "pos_ned",
        "vel_ned",
        "att_rx_time",
        "pos_rx_time",
        "att_seq",
        "pos_seq",
    ):
        assert existing in snapshot


def test_receiver_keeps_cached_payload_when_reordered_payload_is_dropped():
    interface = _ready_interface()
    interface._mavlink_boot_time.observe("attitude", 10_000, 1.0)
    original = interface._att_quat_ned.clone()

    decision = interface._observe_boot_time_locked("attitude", 9_900, 1.1)

    assert decision.disposition is BootTimeDisposition.DROP_REORDERED
    assert decision.accept_payload is False
    assert torch.equal(interface._att_quat_ned, original)
    assert interface._att_time_boot_ms == 100


def test_true_reset_clears_both_streams_until_both_new_samples_arrive():
    interface = _ready_interface()
    interface._mavlink_boot_time.observe("attitude", 50_000, 1.0)
    interface._mavlink_boot_time.observe("local_position", 50_020, 1.0)

    candidate = interface._observe_boot_time_locked("attitude", 100, 2.0)
    assert candidate.disposition is BootTimeDisposition.RESET_CANDIDATE
    assert interface.snapshot() is None

    confirmed = interface._observe_boot_time_locked("local_position", 120, 2.1)
    assert confirmed.disposition is BootTimeDisposition.RESET
    assert interface._att_quat_ned is None
    assert interface._body_rates_ned is None
    assert interface._pos_ned is None
    assert interface._vel_ned is None
    assert interface._last_att_time == 0.0
    assert interface._last_pos_time == 0.0
    assert interface._att_time_boot_ms is None
    assert interface._pos_time_boot_ms is None

    # Mirror storage of the accepted confirming position payload.  Snapshot is
    # still withheld because the earlier attitude candidate was dropped.
    interface._pos_ned = torch.ones(3)
    interface._vel_ned = torch.ones(3)
    interface._pos_time_boot_ms = confirmed.time_boot_ms
    interface._last_pos_time = 2.1
    assert interface.snapshot() is None

    attitude = interface._observe_boot_time_locked("attitude", 140, 2.2)
    assert attitude.disposition is BootTimeDisposition.ACCEPT
    interface._att_quat_ned = torch.tensor([1.0, 0.0, 0.0, 0.0])
    interface._body_rates_ned = torch.zeros(3)
    interface._att_time_boot_ms = attitude.time_boot_ms
    interface._last_att_time = 2.2
    assert interface.snapshot() is not None


def test_session_id_changes_only_after_confirmed_boot_reset():
    first = _ready_interface()
    second = _ready_interface()
    session_id = first.odometry_session_id

    assert second.odometry_session_id == session_id
    assert session_id.endswith(":0")

    first._mavlink_boot_time.observe("attitude", 50_000, 1.0)
    first._mavlink_boot_time.observe("local_position", 50_020, 1.0)
    first._mavlink_boot_time.observe("attitude", 100, 2.0)
    assert first.odometry_session_id == session_id
    first._mavlink_boot_time.observe("local_position", 120, 2.1)

    assert first.odometry_session_id != session_id
    assert first.odometry_session_id.endswith(":1")
    assert second.odometry_session_id == session_id
    assert first.snapshot()["mavlink_time_reset_count"] == 1
    assert first.snapshot()["odometry_session_id"] == first.odometry_session_id
    with pytest.raises(AttributeError):
        first.odometry_session_id = "replacement"


def test_uint32_wrap_does_not_change_odometry_session():
    interface = _ready_interface()
    session_id = interface.odometry_session_id

    interface._mavlink_boot_time.observe("attitude", (1 << 32) - 20, 1.0)
    interface._mavlink_boot_time.observe("attitude", 25, 1.1)

    assert interface.odometry_session_id == session_id


def test_ekf_health_default_remains_variance_only():
    interface = _ready_interface()

    assert interface.is_ekf_healthy(
        {"ekf_vel_variance": 0.1, "ekf_flags": None}
    )
    assert not interface.is_ekf_healthy(
        {"ekf_vel_variance": 0.9, "ekf_flags": 0xFFFF}
    )


def test_ekf_health_required_mask_fails_closed_on_missing_bits():
    interface = _ready_interface()
    required = 0b0111

    assert not interface.is_ekf_healthy(
        {"ekf_vel_variance": 0.1, "ekf_flags": None}, required
    )
    assert not interface.is_ekf_healthy(
        {"ekf_vel_variance": 0.1, "ekf_flags": 0b0011}, required
    )
    assert interface.is_ekf_healthy(
        {"ekf_vel_variance": 0.1, "ekf_flags": 0b1111}, required
    )
    with pytest.raises(ValueError):
        interface.is_ekf_healthy(
            {"ekf_vel_variance": 0.1, "ekf_flags": 0b1111}, -1
        )


@pytest.mark.parametrize(
    "variance",
    [float("nan"), float("inf"), float("-inf"), -0.1],
)
def test_ekf_health_rejects_nonfinite_or_negative_variance(variance):
    interface = _ready_interface()

    assert not interface.is_ekf_healthy(
        {"ekf_vel_variance": variance, "ekf_flags": 0xFFFF}
    )


class _FakeMav:
    def __init__(self):
        self.commands = []

    def command_long_send(self, *args):
        self.commands.append(args)


class _FakeMaster:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = _FakeMav()


def test_arm_cancellation_before_send_emits_no_arm_command():
    interface = _ready_interface()
    interface._armed = False
    interface._master = _FakeMaster()

    assert interface.arm(timeout=1.0, cancel_check=lambda: True) is False
    assert interface._master.mav.commands == []


def test_arm_cancellation_during_ack_wait_stops_after_first_command():
    interface = _ready_interface()
    interface._armed = False
    interface._master = _FakeMaster()
    checks = 0

    def cancel_after_send():
        nonlocal checks
        checks += 1
        return checks >= 3

    assert interface.arm(timeout=1.0, cancel_check=cancel_after_send) is False
    assert len(interface._master.mav.commands) == 1


def test_arm_default_api_still_accepts_already_armed_vehicle():
    interface = _ready_interface()
    interface._armed = True

    assert interface.arm() is True


# ---------------------------------------------------------------- 스트림 요청
# SCALED_PRESSURE 는 수신 처리만 있고 요청이 없었다. 기체가 이 endpoint 로
# 흘려주는지가 BlueOS/ArduSub 기본 스트림 설정에 달려 있었고, 안 오면 깊이
# 게이트(docs/REAL_ROBOT_SESSION.md 1단계)를 아예 못 한다 -- 물에 들어간 뒤에야
# 드러나는 종류의 결함이다.
class _StreamRequestRecorder:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.sent = []
        self.mav = self

    def command_long_send(self, sysid, compid, command, confirmation,
                          p1, p2, *rest):
        self.sent.append((int(p1), int(p2)))


def _requested_intervals(telemetry_rate_hz=25.0, baro_rate_hz=10.0):
    from brov_base.mavlink_interface import RealRobotInterface

    interface = RealRobotInterface(
        "udpin:0.0.0.0:14550",
        telemetry_rate_hz=telemetry_rate_hz,
        baro_rate_hz=baro_rate_hz,
    )
    interface._master = _StreamRequestRecorder()
    interface._request_telemetry_streams()
    return dict(interface._master.sent)


def test_all_three_scaled_pressure_instances_are_requested():
    from pymavlink import mavutil

    intervals = _requested_intervals()
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3,
    ):
        assert msg_id in intervals, (
            "어느 instance 가 물속 센서인지는 BARO_PRIMARY 를 읽기 전에는 "
            "모른다. 셋 다 요청해야 한다"
        )


def test_pose_streams_are_still_requested_at_the_control_rate():
    from pymavlink import mavutil

    intervals = _requested_intervals(telemetry_rate_hz=25.0)
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    ):
        assert intervals[msg_id] == 40_000        # 25 Hz


def test_baro_is_requested_slower_than_the_control_rate():
    """기압을 제어 주기로 요청하면 같은 값이 중복으로 와 bag 을 부풀린다."""
    from pymavlink import mavutil

    intervals = _requested_intervals(telemetry_rate_hz=25.0, baro_rate_hz=10.0)
    assert intervals[mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE] == 100_000
    assert (
        intervals[mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE]
        > intervals[mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION]
    )
