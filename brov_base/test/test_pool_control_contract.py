import hashlib
import inspect
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import yaml

from brov_interfaces.msg import LocalizationStatus

from brov_base.obs_node import ObsNode, _random_attitude_config


class _Clock:
    def __init__(self, nanoseconds: int):
        self._nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self._nanoseconds)


def _status(
    *,
    stamp_ns=10_000_000_000,
    epoch=4,
    session="boot-a:nav0",
    alignment="alignment-a",
):
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return SimpleNamespace(
        state=LocalizationStatus.INITIALIZED,
        output_valid=True,
        reason="initialized",
        epoch=epoch,
        odometry_session_id=session,
        alignment_id=alignment,
        pool_to_odom=transform,
        header=SimpleNamespace(
            frame_id="pool",
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
    )


def _gate_owner(status=None):
    return SimpleNamespace(
        _localization_status=status or _status(),
        _localization_status_max_age_s=1.0,
        _active_localization_epoch=None,
        _active_odometry_session_id=None,
        _active_alignment_id=None,
        _control_active=False,
        _pool_frame="pool",
        _current_odometry_session_id=lambda snap: (
            f"{snap['odometry_session_id']}:nav0"
        ),
        get_clock=lambda: _Clock(10_200_000_000),
    )


def test_localization_gate_accepts_only_current_session_and_epoch() -> None:
    owner = _gate_owner()
    snapshot = {"odometry_session_id": "boot-a"}
    assert ObsNode._pool_localization_gate(owner, snapshot) is None

    owner._active_localization_epoch = 4
    owner._active_odometry_session_id = "boot-a:nav0"
    owner._active_alignment_id = "alignment-a"
    owner._control_active = True
    assert ObsNode._pool_localization_gate(owner, snapshot) is None

    owner._localization_status = _status(epoch=5)
    reason = ObsNode._pool_localization_gate(owner, snapshot)
    assert reason == "pool localization epoch changed during active control"


def test_localization_gate_rejects_stale_and_wrong_boot() -> None:
    owner = _gate_owner(_status(stamp_ns=8_000_000_000))
    reason = ObsNode._pool_localization_gate(
        owner, {"odometry_session_id": "boot-a"}
    )
    assert "stale/future" in reason

    owner = _gate_owner()
    reason = ObsNode._pool_localization_gate(
        owner, {"odometry_session_id": "boot-b"}
    )
    assert reason == "pool localization odometry session mismatch"


def test_localization_gate_rejects_zero_epoch_and_invalid_output() -> None:
    snapshot = {"odometry_session_id": "boot-a"}
    owner = _gate_owner(_status(epoch=0))
    assert (
        ObsNode._pool_localization_gate(owner, snapshot)
        == "pool localization initialized epoch must be non-zero"
    )

    invalid = _status()
    invalid.output_valid = False
    owner = _gate_owner(invalid)
    assert (
        ObsNode._pool_localization_gate(owner, snapshot)
        == "pool localization output is not valid"
    )


def test_resolved_mission_gate_binds_hash_epoch_session_and_frame() -> None:
    canonical = json.dumps(
        {
            "contract": "brov_pool_position_mission_v1",
            "frame_id": "pool",
            "waypoints": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            "cruise_speed": 0.1,
            "lookahead_dist": 0.4,
            "reach_threshold": 0.15,
            "heading_mode": "straight",
            "loop": False,
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    mission = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"),
        waypoints=[
            SimpleNamespace(x=0.0, y=0.0, z=0.0),
            SimpleNamespace(x=1.0, y=0.0, z=0.0),
        ],
        mission_id="mission-a",
        plan_hash=hashlib.sha256(canonical.encode("ascii")).hexdigest(),
        contract_version="brov_pool_position_mission_v1",
        canonical_plan_json=canonical,
        localization_epoch=4,
        odometry_session_id="boot-a:nav0",
        alignment_id="alignment-a",
        cruise_speed=0.1,
        cruise_speed_per_leg=[],
        lookahead_dist=0.4,
        reach_threshold=0.15,
        heading_mode="straight",
        loop=False,
    )
    owner = SimpleNamespace(
        _resolved_mission=mission,
        _localization_status=_status(),
        _odom_frame="odom",
        _pool_frame="pool",
        _max_resolved_waypoints=50,
        _max_resolved_segment_length_m=4.0,
        _max_resolved_cruise_speed=0.3,
        _max_resolved_lookahead_dist=1.0,
        _max_resolved_reach_threshold=0.5,
        _current_odometry_session_id=lambda snap: (
            f"{snap['odometry_session_id']}:nav0"
        ),
    )
    snapshot = {"odometry_session_id": "boot-a"}
    assert ObsNode._resolved_mission_gate(owner, snapshot) is None

    mission.odometry_session_id = "boot-old"
    assert (
        ObsNode._resolved_mission_gate(owner, snapshot)
        == "resolved mission odometry session mismatch"
    )


def _v2_gate_owner(*, loop=True, canonical_extra=None, random_extra=None):
    random = {
        "seed": 20260814,
        "reference_frame": "pool_zup_flu",
        "generator_version": "sha256_counter_uniform_rpy_v1",
        "rpy_min_rad": [-math.pi / 2.0, -math.pi / 2.0, -math.pi],
        "rpy_max_rad": [math.pi / 2.0, math.pi / 2.0, math.pi],
        "max_slew_rate_rad_s": 0.35,
        "attitude_tolerance_rad": 0.1745,
        "angular_speed_tolerance_rad_s": 0.0873,
        "dwell_time_s": 1.0,
        "max_duration_s": 120.0,
        "max_laps": 1,
    }
    if random_extra:
        random.update(random_extra)
    canonical_dict = {
        "contract": "brov_pool_position_mission_v2",
        "frame_id": "pool",
        "waypoints": [
            [0.0, 0.0, 0.0],
            [0.4, 0.0, 0.0],
            [0.4, 0.4, 0.0],
            [0.0, 0.4, 0.0],
        ],
        "cruise_speed": 0.05,
        "lookahead_dist": 0.15,
        "reach_threshold": 0.08,
        "heading_mode": "random_at_waypoint",
        "loop": loop,
        "random_attitude": random,
    }
    if canonical_extra:
        canonical_dict.update(canonical_extra)
    canonical = json.dumps(
        canonical_dict, sort_keys=True, separators=(",", ":")
    )
    mission = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"),
        waypoints=[
            SimpleNamespace(x=x, y=y, z=z)
            for x, y, z in canonical_dict["waypoints"]
        ],
        mission_id="mission-c",
        plan_hash=hashlib.sha256(canonical.encode("ascii")).hexdigest(),
        contract_version="brov_pool_position_mission_v2",
        canonical_plan_json=canonical,
        localization_epoch=4,
        odometry_session_id="boot-a:nav0",
        alignment_id="alignment-a",
        cruise_speed=0.05,
        cruise_speed_per_leg=[],
        lookahead_dist=0.15,
        reach_threshold=0.08,
        heading_mode="random_at_waypoint",
        loop=loop,
    )
    owner = SimpleNamespace(
        _resolved_mission=mission,
        _localization_status=_status(),
        _odom_frame="odom",
        _pool_frame="pool",
        _max_resolved_waypoints=50,
        _max_resolved_segment_length_m=4.0,
        _max_resolved_cruise_speed=0.3,
        _max_resolved_lookahead_dist=1.0,
        _max_resolved_reach_threshold=0.5,
        _max_random_attitude_slew_rate_rad_s=0.5,
        _max_random_attitude_tolerance_rad=0.35,
        _max_random_angular_speed_tolerance_rad_s=0.2,
        _min_random_attitude_dwell_s=0.5,
        _max_random_mission_duration_s=180.0,
        _max_random_mission_laps=1,
        _send_pwm=False,
        _max_pwm_delta_per_s=0.0,
        _current_odometry_session_id=lambda snap: (
            f"{snap['odometry_session_id']}:nav0"
        ),
    )
    return owner, canonical_dict


def test_v2_gate_accepts_exact_hash_bound_random_contract() -> None:
    owner, canonical = _v2_gate_owner()
    assert _random_attitude_config(canonical).seed == 20260814
    assert ObsNode._resolved_mission_gate(
        owner, {"odometry_session_id": "boot-a"}
    ) is None


def test_v2_gate_requires_loop_and_exact_canonical_keys() -> None:
    not_looping, _ = _v2_gate_owner(loop=False)
    assert ObsNode._resolved_mission_gate(
        not_looping, {"odometry_session_id": "boot-a"}
    ) == "resolved mission v2 requires loop=true"

    extra, _ = _v2_gate_owner(canonical_extra={"unapproved": True})
    assert ObsNode._resolved_mission_gate(
        extra, {"odometry_session_id": "boot-a"}
    ) == "resolved mission canonical top-level keys mismatch"

    nested_extra, _ = _v2_gate_owner(random_extra={"unapproved": 1})
    assert "keys mismatch" in ObsNode._resolved_mission_gate(
        nested_extra, {"odometry_session_id": "boot-a"}
    )


def test_v1_rejects_rehashed_unknown_top_level_key() -> None:
    owner, _ = _v2_gate_owner()
    mission = owner._resolved_mission
    canonical = json.loads(mission.canonical_plan_json)
    canonical.pop("random_attitude")
    canonical["contract"] = "brov_pool_position_mission_v1"
    canonical["heading_mode"] = "straight"
    canonical["unknown"] = 1
    mission.contract_version = "brov_pool_position_mission_v1"
    mission.heading_mode = "straight"
    mission.canonical_plan_json = json.dumps(
        canonical, sort_keys=True, separators=(",", ":")
    )
    mission.plan_hash = hashlib.sha256(
        mission.canonical_plan_json.encode("ascii")
    ).hexdigest()
    assert ObsNode._resolved_mission_gate(
        owner, {"odometry_session_id": "boot-a"}
    ) == "resolved mission canonical top-level keys mismatch"


def test_start_orders_all_fail_closed_checks_before_activation() -> None:
    source = inspect.getsource(ObsNode._on_start_control)
    assert source.index("_pool_localization_gate") < source.index(
        "self._control_active = True"
    )
    assert source.index("_resolved_mission_gate") < source.index(
        "self._control_active = True"
    )
    assert source.index("_prepared_gate") < source.index(
        "self._control_active = True"
    )
    assert source.index("_hardware_arm_approved") < source.index(
        "self._control_active = True"
    )


def test_constructor_never_calls_arm_and_services_split_lifecycle() -> None:
    constructor = inspect.getsource(ObsNode.__init__)
    assert "self.interface.arm()" not in constructor
    assert '"/brov/prepare_control"' in constructor
    assert '"/brov/arm_control"' in constructor
    assert '"/brov/start_control"' in constructor


def test_navigation_jump_advances_derived_session() -> None:
    import torch

    owner = SimpleNamespace(
        _raw_odometry_session_id="boot-a",
        _navigation_jump_count=0,
        _last_odom_position=torch.zeros(3),
        _last_odom_orientation=torch.tensor([0.0, 0.0, 0.0, 1.0]),
        _last_odom_sample_time=1.0,
        _odom_jump_max_dt_s=0.5,
        _odom_jump_translation_m=0.5,
        _odom_jump_rotation_rad=0.5,
    )
    converted = SimpleNamespace(
        position_odom=torch.tensor([1.0, 0.0, 0.0]),
        orientation_xyzw=torch.tensor([0.0, 0.0, 0.0, 1.0]),
    )
    snapshot = {
        "odometry_session_id": "boot-a",
        "att_rx_time": 1.1,
        "pos_rx_time": 1.1,
    }
    reason = ObsNode._detect_odometry_discontinuity(
        owner, converted, snapshot
    )
    assert "position discontinuity" in reason
    assert owner._navigation_jump_count == 1


def test_odometry_covariance_has_ros_xyz_rpy_diagonal() -> None:
    covariance = ObsNode._diagonal_covariance(2.0, 3.0)
    assert len(covariance) == 36
    assert [covariance[index] for index in (0, 7, 14)] == [2.0] * 3
    assert [covariance[index] for index in (21, 28, 35)] == [3.0] * 3
    assert sum(value != 0.0 for value in covariance) == 6


def test_local_odometry_is_published_with_atomic_session_identity() -> None:
    constructor = inspect.getsource(ObsNode.__init__)
    publisher = inspect.getsource(ObsNode._publish_odometry)
    assert '"/brov/odometry/local_with_session"' in constructor
    assert "OdometrySession" in constructor
    assert "envelope.odometry = message" in publisher
    assert "envelope.odometry_session_id = session_id" in publisher
    assert "self.pub_odometry_with_session.publish(envelope)" in publisher


class _ActuationInterface:
    def __init__(self, state=None) -> None:
        self.state = state or {
            "armed": True,
            "heartbeat_age_s": 0.1,
            "custom_mode": 19,
        }
        self.neutral_count = 0
        self.disarm_count = 0

    def snapshot(self):
        return {"snapshot": True}

    def control_snapshot(self):
        return dict(self.state)

    def neutral_stop(self) -> None:
        self.neutral_count += 1

    def disarm(self) -> None:
        self.disarm_count += 1


def test_arm_post_ack_generation_change_cancels_and_disarms() -> None:
    interface = _ActuationInterface()
    owner = SimpleNamespace(
        _send_pwm=True,
        _arm_permitted=True,
        _estopped=False,
        _faulted=False,
        _control_active=False,
        _arm_transaction_generation=7,
        _arm_in_progress=False,
        _hardware_arm_approved=False,
        _hardware_arm_deadline=None,
        _arm_to_start_timeout_s=8.0,
        interface=interface,
        _arm_lifecycle_gate=lambda _snapshot: None,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )

    def clear_prepared() -> None:
        owner._arm_transaction_generation += 1
        owner._hardware_arm_approved = False
        owner._hardware_arm_deadline = None

    def arm(*, cancel_check) -> bool:
        assert not cancel_check()
        # Simulate STOP/reset/disarm invalidating the transaction immediately
        # after the autopilot ACK but before post-ACK approval.
        owner._arm_transaction_generation += 1
        return True

    owner._clear_prepared_contract = clear_prepared
    owner._neutral_and_disarm = lambda: ObsNode._neutral_and_disarm(owner)
    owner._revoke_hardware_arm = lambda reason: ObsNode._revoke_hardware_arm(
        owner, reason
    )
    interface.arm = arm
    response = SimpleNamespace(success=None, message="")

    ObsNode._on_arm_control(owner, None, response)

    assert response.success is False
    assert "invalidated after ACK" in response.message
    assert interface.neutral_count >= 2
    assert interface.disarm_count == 1
    assert owner._hardware_arm_approved is False
    assert owner._arm_in_progress is False


def _inactive_watchdog_owner(gate_reason, deadline):
    interface = _ActuationInterface()
    errors = []
    owner = SimpleNamespace(
        _arm_in_progress=False,
        _hardware_arm_approved=True,
        _control_active=False,
        _hardware_arm_deadline=deadline,
        _first_pwm_deadline=123.0,
        _last_pwm_rx_monotonic=123.0,
        _send_pwm=True,
        interface=interface,
        _arm_lifecycle_gate=lambda _snapshot: gate_reason,
        _clear_prepared_contract=lambda: setattr(
            owner, "_hardware_arm_approved", False
        ),
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    owner._revoke_hardware_arm = lambda reason: ObsNode._revoke_hardware_arm(
        owner, reason
    )
    owner._neutral_and_disarm = lambda: ObsNode._neutral_and_disarm(owner)
    return owner, interface, errors


def test_inactive_arm_watchdog_revokes_timeout_and_gate_invalidation() -> None:
    timed_out, timeout_interface, timeout_errors = _inactive_watchdog_owner(
        None, time.monotonic() - 1.0
    )
    assert ObsNode._inactive_arm_watchdog(timed_out, {}) is True
    assert timeout_interface.neutral_count == 1
    assert timeout_interface.disarm_count == 1
    assert "ARM-to-START approval timed out" in timeout_errors[-1]

    invalid, invalid_interface, invalid_errors = _inactive_watchdog_owner(
        "ArduSub left MANUAL mode", time.monotonic() + 100.0
    )
    assert ObsNode._inactive_arm_watchdog(invalid, {}) is True
    assert invalid_interface.neutral_count == 1
    assert invalid_interface.disarm_count == 1
    assert "ArduSub left MANUAL mode" in invalid_errors[-1]


def test_actuation_gate_requires_fresh_heartbeat_and_manual_mode() -> None:
    owner = SimpleNamespace(
        interface=_ActuationInterface(),
        _heartbeat_max_age_s=2.0,
        _required_custom_mode=19,
    )
    assert ObsNode._actuation_mode_gate(owner) is None

    owner.interface.state["heartbeat_age_s"] = 2.1
    assert "heartbeat stale" in ObsNode._actuation_mode_gate(owner)

    owner.interface.state["heartbeat_age_s"] = float("inf")
    assert "heartbeat stale" in ObsNode._actuation_mode_gate(owner)

    owner.interface.state["heartbeat_age_s"] = 0.1
    owner.interface.state["custom_mode"] = 2
    reason = ObsNode._actuation_mode_gate(owner)
    assert "required MANUAL mode=19" in reason


def test_authority_gate_requires_exactly_one_pwm_publisher() -> None:
    owner = SimpleNamespace(
        _gazebo_truth_logging_enabled=False,
        _gazebo_truth_topic="/brov/sim/gazebo_odometry_raw",
        _require_pool_localization=False,
        _require_resolved_mission=False,
        _send_pwm=True,
        count_publishers=lambda _topic: 1,
    )
    assert ObsNode._authority_gate(owner) is None

    for count in (0, 2):
        owner.count_publishers = lambda _topic, count=count: count
        assert ObsNode._authority_gate(owner) == (
            "expected exactly one thruster PWM publisher; "
            f"found {count}"
        )


def test_authority_gate_requires_one_truth_publisher_when_recording() -> None:
    counts = {
        "/brov/sim/gazebo_odometry_raw": 1,
        "/brov/thruster_pwm": 1,
    }
    owner = SimpleNamespace(
        _gazebo_truth_logging_enabled=True,
        _gazebo_truth_topic="/brov/sim/gazebo_odometry_raw",
        _require_pool_localization=False,
        _require_resolved_mission=False,
        _send_pwm=True,
        count_publishers=lambda topic: counts[topic],
    )

    assert ObsNode._authority_gate(owner) is None
    for count in (0, 2):
        counts["/brov/sim/gazebo_odometry_raw"] = count
        assert ObsNode._authority_gate(owner) == (
            "expected exactly one Gazebo truth publisher on "
            "/brov/sim/gazebo_odometry_raw; "
            f"found {count}"
        )


def test_arm_gate_rejects_invalid_selected_feedback_before_arming() -> None:
    owner = SimpleNamespace(
        _telemetry_valid=lambda _snapshot: (True, "ok"),
        _selected_feedback_snapshot=lambda _snapshot: None,
        _feedback_valid=lambda _snapshot: (False, "Gazebo truth unavailable"),
        _authority_gate=lambda: None,
        _actuation_mode_gate=lambda: None,
        _require_pool_localization=False,
        _require_resolved_mission=False,
    )

    assert ObsNode._arm_lifecycle_gate(owner, {"mav": "healthy"}) == (
        "Gazebo truth unavailable"
    )


def test_pwm_watchdogs_precede_duplicate_sample_return_and_timestamp_send() -> None:
    tick = inspect.getsource(ObsNode._tick)
    start = inspect.getsource(ObsNode._on_start_control)
    pwm = inspect.getsource(ObsNode._on_pwm)

    duplicate_check = tick.index("sample_key =")
    assert tick.index("first controller PWM command timeout") < duplicate_check
    assert tick.index("controller PWM watchdog timeout") < duplicate_check
    assert start.index("self._control_active = True") < start.index(
        "self._first_pwm_deadline ="
    )
    assert pwm.index("self._authority_gate()") < pwm.index(
        "self.interface.send_pwm(pwm)"
    )
    assert pwm.index("self._actuation_mode_gate()") < pwm.index(
        "self.interface.send_pwm(pwm)"
    )
    assert pwm.index("self.interface.send_pwm(pwm)") < pwm.index(
        "self._last_pwm_rx_monotonic = time.monotonic()"
    )

    safety = yaml.safe_load(
        (
            Path(__file__).resolve().parents[1] / "config" / "safety.yaml"
        ).read_text(encoding="utf-8")
    )["brov_obs_node"]["ros__parameters"]
    assert safety["arm_to_start_timeout_s"] == 8.0
    assert safety["first_pwm_timeout_s"] == 8.0
    assert safety["pwm_command_timeout_s"] == 0.25
    assert safety["max_random_attitude_slew_rate_rad_s"] == 0.50
    assert safety["max_random_attitude_tolerance_rad"] == 0.35
    assert safety["max_random_angular_speed_tolerance_rad_s"] == 0.20
    assert safety["min_random_attitude_dwell_s"] == 0.50
    assert safety["max_random_mission_duration_s"] == 180.0
    assert safety["max_random_mission_laps"] == 1
    assert safety["max_pwm_abs"] == 1.0
    assert safety["max_pwm_delta_per_s"] == 0.0
    assert safety["pwm_rate_first_command_dt_s"] == 0.04


def test_passthrough_is_final_constructor_stage_with_failure_cleanup() -> None:
    constructor = inspect.getsource(ObsNode.__init__)
    enable = constructor.index("self.interface.enable_passthrough()")

    for setup_call in (
        "self.create_subscription(",
        "self.create_publisher(",
        "self.create_service(",
        "self.create_timer(",
    ):
        assert constructor.rfind(setup_call) < enable

    failure_tail = constructor[enable:]
    close = failure_tail.index("self.interface.close(send_stop=False)")
    assert failure_tail.index("except BaseException:") < close
    assert close < failure_tail.index("raise", close)


def test_prepare_while_armed_revokes_before_returning() -> None:
    interface = _ActuationInterface()
    errors = []
    owner = SimpleNamespace(
        _estopped=False,
        _faulted=False,
        _control_active=False,
        _send_pwm=True,
        interface=interface,
        _clear_prepared_contract=lambda: None,
        _first_pwm_deadline=None,
        _last_pwm_rx_monotonic=None,
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    owner._neutral_and_disarm = lambda: ObsNode._neutral_and_disarm(owner)
    owner._revoke_hardware_arm = lambda reason: ObsNode._revoke_hardware_arm(
        owner, reason
    )
    response = SimpleNamespace(success=None, message="")

    ObsNode._on_prepare_control(owner, None, response)

    assert response.success is False
    assert "neutral/disarm requested" in response.message
    assert interface.neutral_count == 1
    assert interface.disarm_count == 1
    assert "PREPARE requested while vehicle armed" in errors[-1]


def test_neutral_failure_never_skips_disarm() -> None:
    interface = _ActuationInterface()

    def failed_neutral() -> None:
        interface.neutral_count += 1
        raise RuntimeError("link write failed")

    interface.neutral_stop = failed_neutral
    owner = SimpleNamespace(_send_pwm=True, interface=interface)

    errors = ObsNode._neutral_and_disarm(owner)

    assert interface.neutral_count == 1
    assert interface.disarm_count == 1
    assert errors == ["neutral failed: link write failed"]


def test_arm_transport_exception_revokes_and_disarms() -> None:
    interface = _ActuationInterface()
    owner = SimpleNamespace(
        _send_pwm=True,
        _arm_permitted=True,
        _estopped=False,
        _faulted=False,
        _control_active=False,
        _arm_transaction_generation=3,
        _arm_in_progress=False,
        _hardware_arm_approved=False,
        _hardware_arm_deadline=None,
        _arm_to_start_timeout_s=8.0,
        _first_pwm_deadline=None,
        _last_pwm_rx_monotonic=None,
        interface=interface,
        _arm_lifecycle_gate=lambda _snapshot: None,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )

    def clear_prepared() -> None:
        owner._arm_transaction_generation += 1
        owner._hardware_arm_approved = False
        owner._hardware_arm_deadline = None

    def failed_arm(*, cancel_check) -> bool:
        assert not cancel_check()
        raise RuntimeError("ACK transport failed")

    owner._clear_prepared_contract = clear_prepared
    owner._neutral_and_disarm = lambda: ObsNode._neutral_and_disarm(owner)
    owner._revoke_hardware_arm = lambda reason: ObsNode._revoke_hardware_arm(
        owner, reason
    )
    interface.arm = failed_arm
    response = SimpleNamespace(success=None, message="")

    ObsNode._on_arm_control(owner, None, response)

    assert response.success is False
    assert response.message == "ACK transport failed"
    assert interface.neutral_count == 2
    assert interface.disarm_count == 1
    assert owner._hardware_arm_approved is False
    assert owner._arm_in_progress is False


class _PwmInterface(_ActuationInterface):
    def __init__(self) -> None:
        super().__init__()
        self.sent = []

    def send_pwm(self, value) -> None:
        self.sent.append(value.clone())


def _pwm_owner(now: float):
    interface = _PwmInterface()
    faults = []
    owner = SimpleNamespace(
        _estopped=False,
        _faulted=False,
        _send_pwm=True,
        _control_active=True,
        _active_obs_published=True,
        _max_pwm_abs=0.35,
        _max_pwm_delta_per_s=0.5,
        _pwm_rate_first_command_dt_s=0.04,
        _last_accepted_pwm=__import__("torch").zeros(8),
        _last_accepted_pwm_monotonic=now,
        _pwm_rate_first_command=True,
        _last_pwm_rx_monotonic=None,
        _require_pool_localization=False,
        _hardware_arm_approved=True,
        interface=interface,
        _telemetry_valid=lambda _snapshot: (True, "ok"),
        _selected_feedback_snapshot=lambda snapshot: snapshot,
        _feedback_valid=lambda _snapshot: (True, "ok"),
        _authority_gate=lambda: None,
        _actuation_mode_gate=lambda: None,
        _trip_fault=faults.append,
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )
    return owner, interface, faults


def test_pwm_gateway_applies_abs_and_first_nominal_dt_rate_gate(
    monkeypatch,
) -> None:
    import torch

    now = 100.0
    monkeypatch.setattr("brov_base.obs_node.time.monotonic", lambda: now)
    owner, interface, faults = _pwm_owner(now)

    # 0.5/s * nominal first 0.04 s = 0.02 maximum first increment.
    ObsNode._on_pwm(
        owner, SimpleNamespace(data=[0.02] * 8)
    )
    assert not faults
    assert len(interface.sent) == 1
    assert torch.allclose(interface.sent[0], torch.full((8,), 0.02))
    assert owner._pwm_rate_first_command is False

    now = 100.1
    ObsNode._on_pwm(owner, SimpleNamespace(data=[0.07] * 8))
    assert not faults
    assert len(interface.sent) == 2

    now = 100.2
    ObsNode._on_pwm(owner, SimpleNamespace(data=[0.20] * 8))
    assert "PWM slew-rate limit exceeded" in faults[-1]
    assert len(interface.sent) == 2

    owner, interface, faults = _pwm_owner(now)
    ObsNode._on_pwm(owner, SimpleNamespace(data=[0.36] * 8))
    assert "exceeds max_pwm_abs" in faults[-1]
    assert not interface.sent


def test_random_mission_completion_is_normal_neutral_disarm_not_fault() -> None:
    interface = _ActuationInterface()
    published_control = []
    published_complete = []
    logs = []
    owner = SimpleNamespace(
        _control_active=True,
        _active_obs_published=True,
        _hardware_arm_approved=True,
        _hardware_arm_deadline=1.0,
        _first_pwm_deadline=1.0,
        _last_pwm_rx_monotonic=1.0,
        _send_pwm=True,
        interface=interface,
        pub_control_active=SimpleNamespace(
            publish=lambda message: published_control.append(message.data)
        ),
        pub_mission_complete=SimpleNamespace(
            publish=lambda message: published_complete.append(message.data)
        ),
        _clear_active_contract=lambda: None,
        _clear_prepared_contract=lambda: None,
        get_logger=lambda: SimpleNamespace(info=logs.append),
    )
    owner._neutral_and_disarm = lambda: ObsNode._neutral_and_disarm(owner)

    ObsNode._complete_random_mission(owner, "maximum mission laps reached")

    assert owner._control_active is False
    assert owner._active_obs_published is False
    assert published_control == [False]
    assert published_complete == [True]
    assert interface.neutral_count == 1
    assert interface.disarm_count == 1
    assert "RANDOM MISSION COMPLETE" in logs[-1]
