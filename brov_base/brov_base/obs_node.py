#!/usr/bin/env python3
"""
관측(observation) + 액추에이션 ROS2 노드 — 정책 추론은 policy_node.py가 분리 담당.
MAVLink 연결과 수신 스레드는 이 노드 하나만 소유한다. Controller가 발행한
`/brov/thruster_pwm`을 구독해 송신하므로 여러 MAVLink reader가 같은 endpoint의
패킷을 경쟁해서 가져가는 문제를 피한다.
obs 자체는 여전히 정책과 완전히 분리되어 있어 `ros2 topic echo /brov/observation`
으로 독립 검증 가능하다.

실행(컨테이너 안, ROS2 소싱 후):
    ros2 run brov_base obs_node --ros-args \
        -p connection:=udpin:0.0.0.0:14550 \
        -p waypoints:="0,0,0;3,0,0" \
        -p cruise_speed:=0.3 \
        -p heading_mode:=straight \
        -p waypoint_frame:=start_heading \
        -p loop:=false \
        -p send_pwm:=true -p arm:=true

`loop`(기본 false): 마지막 웨이포인트 도달 후 처음으로 되돌아가 반복할지 여부. false면
마지막 웨이포인트 도달 이력으로 `/brov/mission_complete`가 True가 되며, 이후에도
최종 waypoint position hold는 계속 동작함.

`waypoint_frame`: `start_heading`(기본)은 start_control 순간 위치와 yaw를 각각
원점과 +X 전방으로 정의하고, `ned`는 시작 위치만 0으로 만든다.
`heading_mode`: `straight`는 시작 yaw 유지, `align`은 LOS 방향 정렬,
`upright`는 선택 frame의 yaw=0을 목표로 한다.

발행 토픽:
    /brov/observation      (Float32MultiArray, 16) — 배포 policy metadata와 동일 규약
    /brov/debug/pos_ned    (Float32MultiArray, 3)  — 원시 LOCAL_POSITION_NED (검증용)
    /brov/debug/vel_ned    (Float32MultiArray, 3)  — 원시 LOCAL_POSITION_NED 속도 (검증용)
    /brov/debug/att_quat_ned (Float32MultiArray, 4) — 원시 ATTITUDE_QUATERNION [w,x,y,z] (검증용)
    /brov/debug/q_desired_zup (Float32MultiArray, 4) — 현재 목표 자세 [w,x,y,z] (정책 Z-up 규약)
    /brov/target_waypoint  (Float32MultiArray, 3)  — 목표 웨이포인트(선택 frame, m)
    /brov/waypoint_idx     (Int32)                  — 현재 세그먼트 시작 인덱스(0-base)
    /brov/mission_complete (Bool)                   — loop:=false일 때만 의미 있음, 마지막 웨이포인트 도달 시 True
    /brov/control_active   (Bool)                   — 적분 및 PWM 허용 상태
    /brov/camera_tilt/commanded (Float32)           — 마지막으로 보낸 정규화 tilt [-1,1]
    /brov/odometry/local   (nav_msgs/Odometry)      — odom(Z-up) → base_link(FLU)
    /brov/odometry/session_id (String, latched)     — process/autopilot-boot epoch
    /brov/odometry/local_with_session (OdometrySession) — 위 두 값을 한 DDS sample로 결합

구독 토픽:
    /brov/thruster_pwm (8,) [-1,1] — policy_node.py가 발행. `send_pwm:=true`일 때만
    실제 RC_CHANNELS_OVERRIDE로 송신(기본 false — obs만 보고 싶을 때 안전하게 끌 수 있음).
    /brov/camera_tilt/command (Float32) [-1,1] — RC8과 분리된 MAVLink mount pitch 명령.
    -1=최대 아래, 0=중앙, +1=최대 위. 각도/속도 제한은 ROS parameter로 설정한다.
    /brov/estop (std_msgs/Empty) — 사용자 입력에 의한 즉시 정지. 아무 메시지나 오면
    트립: 중립(1500us) 즉시 송신 + disarm + 이후 /brov/thruster_pwm 영구 무시(자동
    재개 없음 — 재개하려면 노드를 재시작해야 함, 발산 중 자동 재무장으로 반복 트립되는
    것을 막기 위한 의도적 설계). 트리거 예시:
        ros2 topic pub -1 /brov/estop std_msgs/msg/Empty {}
    자동 발산 감지(다중 기준 트리거)는 아직 미구현 — 지금은 사람이 직접 판단해서
    이 토픽을 쏘는 수동 정지만 있다.

제어 lifecycle 서비스:
    ros2 service call /brov/prepare_control std_srvs/srv/Trigger {}
        committed pool mission을 legacy guidance로 변환하되 출력은 frozen 상태로
        유지한다. target/action을 확인하는 PREVIEW 단계다.
    ros2 service call /brov/arm_control std_srvs/srv/Trigger {}
        arm:=true가 허용된 경우 telemetry/localization/prepared mission을 재검사하고
        neutral 송신 후 명시적으로 hardware arm한다.
    ros2 service call /brov/start_control std_srvs/srv/Trigger {}
        모든 gate와 hardware arm 상태를 다시 확인한 뒤 적분/PWM을 허용한다.
    ros2 service call /brov/stop_control std_srvs/srv/Trigger {}
        적분을 동결하고 PWM neutral을 송신한다.
    ros2 service call /brov/reset_integrator std_srvs/srv/Trigger {}
        fault latch와 적분항을 reset하되 control은 frozen 상태로 유지한다.
    ros2 service call /brov/disarm_control std_srvs/srv/Trigger {}
        control을 동결하고 neutral/disarm하며 prepared/active contract를 폐기한다.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import time

import torch
import rclpy
from brov_interfaces.msg import (
    LocalizationStatus,
    OdometrySession,
    ResolvedMission,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import (
    Bool,
    Empty,
    Float32,
    Float32MultiArray,
    Int32,
    Int32MultiArray,
    String,
)
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from brov_base.mavlink_interface import RealRobotInterface
from brov_base.mission import (
    odom_waypoints_to_mission,
    parse_waypoints,
    pool_to_mission_quaternion,
    validate_waypoint_bounds,
)
from brov_base.odometry import ned_frd_to_odom_flu
from brov_base.observation import ObservationBuilder
from brov_base.guidance import LOSGuidance, RandomAttitudeConfig
from brov_base import math_utils as mu


_POOL_MISSION_V1 = "brov_pool_position_mission_v1"
_POOL_MISSION_V2 = "brov_pool_position_mission_v2"
_CANONICAL_V1_KEYS = frozenset(
    {
        "contract",
        "frame_id",
        "waypoints",
        "cruise_speed",
        "lookahead_dist",
        "reach_threshold",
        "heading_mode",
        "loop",
    }
)
_CANONICAL_V2_KEYS = _CANONICAL_V1_KEYS | {"random_attitude"}
_RANDOM_METADATA_KEYS = frozenset(
    {
        "seed",
        "reference_frame",
        "generator_version",
        "rpy_min_rad",
        "rpy_max_rad",
        "max_slew_rate_rad_s",
        "attitude_tolerance_rad",
        "angular_speed_tolerance_rad_s",
        "dwell_time_s",
        "max_duration_s",
        "max_laps",
    }
)


def _random_attitude_config(canonical: dict) -> RandomAttitudeConfig:
    """Strictly parse the hash-bound pool mission v2 behavior metadata."""

    metadata = canonical.get("random_attitude")
    if not isinstance(metadata, dict):
        raise ValueError("v2 mission random_attitude metadata must be an object")
    if set(metadata) != _RANDOM_METADATA_KEYS:
        missing = sorted(_RANDOM_METADATA_KEYS - set(metadata))
        extra = sorted(set(metadata) - _RANDOM_METADATA_KEYS)
        raise ValueError(
            "v2 mission random_attitude keys mismatch; "
            f"missing={missing}, extra={extra}"
        )
    seed = metadata["seed"]
    max_laps = metadata["max_laps"]
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("random attitude seed must be a uint64 integer")
    if isinstance(max_laps, bool) or not isinstance(max_laps, int):
        raise ValueError("random attitude max_laps must be an integer")

    def triple(name: str) -> tuple[float, float, float]:
        raw = metadata[name]
        if not isinstance(raw, list) or len(raw) != 3:
            raise ValueError(f"random attitude {name} must be a three-value array")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in raw
        ):
            raise ValueError(
                f"random attitude {name} must be a finite numeric array"
            )
        values = tuple(float(value) for value in raw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"random attitude {name} must be a finite numeric array"
            )
        return values

    def scalar(name: str) -> float:
        raw = metadata[name]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"random attitude {name} must be numeric")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"random attitude {name} must be finite")
        return value

    config = RandomAttitudeConfig(
        seed=seed,
        reference_frame=metadata["reference_frame"],
        generator_version=metadata["generator_version"],
        rpy_min_rad=triple("rpy_min_rad"),
        rpy_max_rad=triple("rpy_max_rad"),
        max_slew_rate_rad_s=scalar("max_slew_rate_rad_s"),
        attitude_tolerance_rad=scalar("attitude_tolerance_rad"),
        angular_speed_tolerance_rad_s=scalar(
            "angular_speed_tolerance_rad_s"
        ),
        dwell_time_s=scalar("dwell_time_s"),
        max_duration_s=scalar("max_duration_s"),
        max_laps=max_laps,
    )
    config.validate()
    return config


class ObsNode(Node):
    def __init__(self):
        super().__init__("brov_obs_node")
        self.declare_parameter("connection", "udpin:0.0.0.0:14550")
        self.declare_parameter("waypoints", "0,0,0;3,0,0")
        self.declare_parameter("cruise_speed", 0.2)
        self.declare_parameter("heading_mode", "straight")
        self.declare_parameter("waypoint_frame", "start_heading")
        self.declare_parameter("waypoint_bounds_enabled", False)
        self.declare_parameter("waypoint_min_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("waypoint_max_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("reach_threshold", 0.15)
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("depth_hold_kp", 0.8)
        self.declare_parameter("depth_speed_limit", 0.1)
        self.declare_parameter("terminal_hold_kp", 0.5)
        self.declare_parameter("terminal_speed_limit", 0.1)
        self.declare_parameter("send_pwm", False)
        self.declare_parameter("arm", False)
        self.declare_parameter("loop", False)
        self.declare_parameter("att_max_age_s", 0.2)
        self.declare_parameter("pos_max_age_s", 0.5)
        self.declare_parameter("ekf_max_age_s", 0.5)
        self.declare_parameter("max_integration_dt_s", 0.15)
        self.declare_parameter("integral_vel_limit", 5.0)
        self.declare_parameter("integral_att_limit", 5.0)
        self.declare_parameter("quat_norm_tolerance", 0.1)
        self.declare_parameter("camera_tilt_min_deg", -45.0)
        self.declare_parameter("camera_tilt_max_deg", 45.0)
        self.declare_parameter("camera_tilt_max_rate_deg_s", 30.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pool_frame", "pool")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("require_pool_localization", False)
        self.declare_parameter("require_resolved_mission", False)
        self.declare_parameter("localization_status_max_age_s", 1.0)
        self.declare_parameter("max_mission_start_distance_m", 0.35)
        self.declare_parameter("max_prepare_position_drift_m", 0.10)
        self.declare_parameter("max_prepare_attitude_drift_deg", 10.0)
        self.declare_parameter("max_resolved_waypoints", 50)
        self.declare_parameter("max_resolved_segment_length_m", 4.0)
        self.declare_parameter("max_resolved_cruise_speed", 0.30)
        self.declare_parameter("max_resolved_lookahead_dist", 1.0)
        self.declare_parameter("max_resolved_reach_threshold", 0.50)
        # Defense-in-depth limits for the versioned pool random-attitude
        # contract.  Mission metadata may only tighten these limits.
        self.declare_parameter("max_random_attitude_slew_rate_rad_s", 0.50)
        self.declare_parameter("max_random_attitude_tolerance_rad", 0.35)
        self.declare_parameter(
            "max_random_angular_speed_tolerance_rad_s", 0.20
        )
        self.declare_parameter("min_random_attitude_dwell_s", 0.50)
        self.declare_parameter("max_random_mission_duration_s", 180.0)
        self.declare_parameter("max_random_mission_laps", 1)
        self.declare_parameter("max_pwm_abs", 1.0)
        # Zero explicitly disables the rate gate for legacy profiles.  The
        # random-attitude profile must configure a finite positive value.
        self.declare_parameter("max_pwm_delta_per_s", 0.0)
        self.declare_parameter("pwm_rate_first_command_dt_s", 0.04)
        self.declare_parameter("telemetry_source_skew_max_ms", 150)
        self.declare_parameter("heartbeat_max_age_s", 2.0)
        self.declare_parameter("required_custom_mode", 19)
        # MAV_ESTIMATOR_TYPE flags: attitude, horizontal/vertical velocity,
        # relative horizontal position, and absolute vertical position.
        self.declare_parameter("required_ekf_flags", 47)
        self.declare_parameter("odom_jump_translation_m", 0.50)
        self.declare_parameter("odom_jump_rotation_deg", 45.0)
        self.declare_parameter("odom_jump_max_dt_s", 0.50)
        self.declare_parameter("arm_to_start_timeout_s", 8.0)
        self.declare_parameter("first_pwm_timeout_s", 8.0)
        self.declare_parameter("pwm_command_timeout_s", 0.25)
        self.declare_parameter("odom_position_variance", 1.0)
        self.declare_parameter("odom_orientation_variance", 0.25)
        self.declare_parameter("odom_linear_velocity_variance", 0.25)
        self.declare_parameter("odom_angular_velocity_variance", 0.25)

        conn = self.get_parameter("connection").value
        waypoints = parse_waypoints(self.get_parameter("waypoints").value)
        waypoint_frame = str(self.get_parameter("waypoint_frame").value)
        heading_mode = str(self.get_parameter("heading_mode").value)
        if waypoint_frame not in {"ned", "start_heading"}:
            raise ValueError("waypoint_frame은 'ned' 또는 'start_heading'이어야 함")
        valid_heading_modes = {"align", "upright", "straight", "random_at_waypoint"}
        if heading_mode not in valid_heading_modes:
            raise ValueError(
                f"heading_mode={heading_mode!r} invalid; expected {sorted(valid_heading_modes)}"
            )
        waypoint_bounds_enabled = bool(
            self.get_parameter("waypoint_bounds_enabled").value
        )
        waypoint_min_xyz = self.get_parameter("waypoint_min_xyz").value
        waypoint_max_xyz = self.get_parameter("waypoint_max_xyz").value
        validate_waypoint_bounds(
            waypoints,
            enabled=waypoint_bounds_enabled,
            minimum_xyz=waypoint_min_xyz,
            maximum_xyz=waypoint_max_xyz,
        )
        self._camera_tilt_min_deg = float(self.get_parameter("camera_tilt_min_deg").value)
        self._camera_tilt_max_deg = float(self.get_parameter("camera_tilt_max_deg").value)
        self._camera_tilt_max_rate_deg_s = float(
            self.get_parameter("camera_tilt_max_rate_deg_s").value
        )
        if self._camera_tilt_min_deg >= 0.0 or self._camera_tilt_max_deg <= 0.0:
            raise ValueError("camera tilt 범위는 min < 0 < max여야 함")
        if self._camera_tilt_max_rate_deg_s <= 0.0:
            raise ValueError("camera_tilt_max_rate_deg_s는 양수여야 함")
        self._odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        self._pool_frame = str(self.get_parameter("pool_frame").value).strip()
        if not self._odom_frame or not self._base_frame or not self._pool_frame:
            raise ValueError("pool_frame, odom_frame and base_frame must be non-empty")
        if len({self._pool_frame, self._odom_frame, self._base_frame}) != 3:
            raise ValueError("pool_frame, odom_frame and base_frame must be distinct")
        self._publish_odom_tf = bool(
            self.get_parameter("publish_odom_tf").value
        )
        self._require_pool_localization = bool(
            self.get_parameter("require_pool_localization").value
        )
        self._require_resolved_mission = bool(
            self.get_parameter("require_resolved_mission").value
        )
        if self._require_resolved_mission and not self._require_pool_localization:
            raise ValueError(
                "require_resolved_mission requires require_pool_localization"
            )
        self._max_mission_start_distance_m = float(
            self.get_parameter("max_mission_start_distance_m").value
        )
        self._localization_status_max_age_s = float(
            self.get_parameter("localization_status_max_age_s").value
        )
        self._max_prepare_position_drift_m = float(
            self.get_parameter("max_prepare_position_drift_m").value
        )
        self._max_prepare_attitude_drift_rad = math.radians(
            float(self.get_parameter("max_prepare_attitude_drift_deg").value)
        )
        self._max_resolved_waypoints = int(
            self.get_parameter("max_resolved_waypoints").value
        )
        self._max_resolved_segment_length_m = float(
            self.get_parameter("max_resolved_segment_length_m").value
        )
        self._max_resolved_cruise_speed = float(
            self.get_parameter("max_resolved_cruise_speed").value
        )
        self._max_resolved_lookahead_dist = float(
            self.get_parameter("max_resolved_lookahead_dist").value
        )
        self._max_resolved_reach_threshold = float(
            self.get_parameter("max_resolved_reach_threshold").value
        )
        self._max_random_attitude_slew_rate_rad_s = float(
            self.get_parameter("max_random_attitude_slew_rate_rad_s").value
        )
        self._max_random_attitude_tolerance_rad = float(
            self.get_parameter("max_random_attitude_tolerance_rad").value
        )
        self._max_random_angular_speed_tolerance_rad_s = float(
            self.get_parameter(
                "max_random_angular_speed_tolerance_rad_s"
            ).value
        )
        self._min_random_attitude_dwell_s = float(
            self.get_parameter("min_random_attitude_dwell_s").value
        )
        self._max_random_mission_duration_s = float(
            self.get_parameter("max_random_mission_duration_s").value
        )
        self._max_random_mission_laps = int(
            self.get_parameter("max_random_mission_laps").value
        )
        self._max_pwm_abs = float(self.get_parameter("max_pwm_abs").value)
        self._max_pwm_delta_per_s = float(
            self.get_parameter("max_pwm_delta_per_s").value
        )
        self._pwm_rate_first_command_dt_s = float(
            self.get_parameter("pwm_rate_first_command_dt_s").value
        )
        self._telemetry_source_skew_max_ms = int(
            self.get_parameter("telemetry_source_skew_max_ms").value
        )
        self._heartbeat_max_age_s = float(
            self.get_parameter("heartbeat_max_age_s").value
        )
        self._required_custom_mode = int(
            self.get_parameter("required_custom_mode").value
        )
        self._required_ekf_flags = int(
            self.get_parameter("required_ekf_flags").value
        )
        self._odom_jump_translation_m = float(
            self.get_parameter("odom_jump_translation_m").value
        )
        self._odom_jump_rotation_rad = math.radians(
            float(self.get_parameter("odom_jump_rotation_deg").value)
        )
        self._odom_jump_max_dt_s = float(
            self.get_parameter("odom_jump_max_dt_s").value
        )
        self._arm_to_start_timeout_s = float(
            self.get_parameter("arm_to_start_timeout_s").value
        )
        self._first_pwm_timeout_s = float(
            self.get_parameter("first_pwm_timeout_s").value
        )
        self._pwm_command_timeout_s = float(
            self.get_parameter("pwm_command_timeout_s").value
        )
        positive_values = {
            "max_mission_start_distance_m": self._max_mission_start_distance_m,
            "localization_status_max_age_s": self._localization_status_max_age_s,
            "max_prepare_position_drift_m": self._max_prepare_position_drift_m,
            "max_prepare_attitude_drift_deg": self._max_prepare_attitude_drift_rad,
            "max_resolved_segment_length_m": self._max_resolved_segment_length_m,
            "max_resolved_cruise_speed": self._max_resolved_cruise_speed,
            "max_resolved_lookahead_dist": self._max_resolved_lookahead_dist,
            "max_resolved_reach_threshold": self._max_resolved_reach_threshold,
            "max_random_attitude_slew_rate_rad_s": (
                self._max_random_attitude_slew_rate_rad_s
            ),
            "max_random_attitude_tolerance_rad": (
                self._max_random_attitude_tolerance_rad
            ),
            "max_random_angular_speed_tolerance_rad_s": (
                self._max_random_angular_speed_tolerance_rad_s
            ),
            "min_random_attitude_dwell_s": self._min_random_attitude_dwell_s,
            "max_random_mission_duration_s": (
                self._max_random_mission_duration_s
            ),
            "odom_jump_translation_m": self._odom_jump_translation_m,
            "odom_jump_rotation_deg": self._odom_jump_rotation_rad,
            "odom_jump_max_dt_s": self._odom_jump_max_dt_s,
            "heartbeat_max_age_s": self._heartbeat_max_age_s,
            "arm_to_start_timeout_s": self._arm_to_start_timeout_s,
            "first_pwm_timeout_s": self._first_pwm_timeout_s,
            "pwm_command_timeout_s": self._pwm_command_timeout_s,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self._max_resolved_waypoints < 2:
            raise ValueError("max_resolved_waypoints must be at least two")
        if self._max_random_mission_laps <= 0:
            raise ValueError("max_random_mission_laps must be positive")
        if (
            not math.isfinite(self._max_pwm_abs)
            or self._max_pwm_abs <= 0.0
            or self._max_pwm_abs > 1.0
        ):
            raise ValueError("max_pwm_abs must be finite in (0,1]")
        if (
            not math.isfinite(self._max_pwm_delta_per_s)
            or self._max_pwm_delta_per_s < 0.0
        ):
            raise ValueError("max_pwm_delta_per_s must be finite and non-negative")
        if (
            not math.isfinite(self._pwm_rate_first_command_dt_s)
            or self._pwm_rate_first_command_dt_s <= 0.0
        ):
            raise ValueError(
                "pwm_rate_first_command_dt_s must be finite and positive"
            )
        if self._telemetry_source_skew_max_ms <= 0:
            raise ValueError("telemetry_source_skew_max_ms must be positive")
        if self._required_ekf_flags < 0:
            raise ValueError("required_ekf_flags must be non-negative")
        if self._required_custom_mode < 0:
            raise ValueError("required_custom_mode must be non-negative")
        self._odom_covariance = self._read_odom_covariance()

        self.interface = RealRobotInterface(conn)
        self.interface.connect()

        self._send_pwm = bool(self.get_parameter("send_pwm").value)
        self._arm_permitted = bool(self.get_parameter("arm").value)
        self._hardware_arm_approved = False
        self._arm_transaction_generation = 0
        self._arm_in_progress = False
        self._hardware_arm_deadline: float | None = None
        self._first_pwm_deadline: float | None = None
        self._last_pwm_rx_monotonic: float | None = None
        self._last_accepted_pwm = torch.zeros(8, dtype=torch.float32)
        self._last_accepted_pwm_monotonic: float | None = None
        self._pwm_rate_first_command = True

        self.sub_pwm = self.create_subscription(
            Float32MultiArray, "/brov/thruster_pwm", self._on_pwm, 1
        )
        self._estopped = False
        self.sub_estop = self.create_subscription(Empty, "/brov/estop", self._on_estop, 10)
        self.sub_camera_tilt = self.create_subscription(
            Float32, "/brov/camera_tilt/command", self._on_camera_tilt, 10
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_localization_status = self.create_subscription(
            LocalizationStatus,
            "/brov/localization/status",
            self._on_localization_status,
            latched_qos,
        )
        self.sub_resolved_mission = self.create_subscription(
            ResolvedMission,
            "/brov/mission/resolved",
            self._on_resolved_mission,
            latched_qos,
        )

        self.obs_builder = ObservationBuilder(
            integral_vel_limit=float(self.get_parameter("integral_vel_limit").value),
            integral_att_limit=float(self.get_parameter("integral_att_limit").value),
            waypoint_frame=waypoint_frame,
        )
        self.guidance = LOSGuidance(
            waypoints, "cpu",
            cruise_speed=self.get_parameter("cruise_speed").value,
            lookahead_dist=self.get_parameter("lookahead_dist").value,
            heading_mode=heading_mode,
            reach_threshold=self.get_parameter("reach_threshold").value,
            loop=bool(self.get_parameter("loop").value),
            depth_hold_kp=self.get_parameter("depth_hold_kp").value,
            depth_speed_limit=self.get_parameter("depth_speed_limit").value,
            terminal_hold_kp=self.get_parameter("terminal_hold_kp").value,
            terminal_speed_limit=self.get_parameter("terminal_speed_limit").value,
        )

        self.pub_obs = self.create_publisher(
            Float32MultiArray, "/brov/observation", 10
        )
        self.pub_pos = self.create_publisher(
            Float32MultiArray, "/brov/debug/pos_ned", 10
        )
        self.pub_vel = self.create_publisher(
            Float32MultiArray, "/brov/debug/vel_ned", 10
        )
        self.pub_quat = self.create_publisher(Float32MultiArray, "/brov/debug/att_quat_ned", 10)
        self.pub_pos_mission = self.create_publisher(
            Float32MultiArray, "/brov/debug/pos_mission", 10
        )
        self.pub_v_body_zup = self.create_publisher(
            Float32MultiArray, "/brov/debug/v_body_zup", 10
        )
        self.pub_v_desired_body_zup = self.create_publisher(
            Float32MultiArray, "/brov/debug/v_desired_body_zup", 10
        )
        self.pub_q_desired_zup = self.create_publisher(
            Float32MultiArray, "/brov/debug/q_desired_zup", 10
        )
        self.pub_q_random_goal_pool = self.create_publisher(
            Float32MultiArray,
            "/brov/debug/q_random_goal_pool_zup_flu",
            10,
        )
        self.pub_servo_output = self.create_publisher(
            Int32MultiArray, "/brov/debug/servo_output_us", 10
        )
        self.pub_target_wp = self.create_publisher(Float32MultiArray, "/brov/target_waypoint", 10)
        self.pub_wp_idx = self.create_publisher(
            Int32, "/brov/waypoint_idx", 10
        )
        self.pub_mission_complete = self.create_publisher(Bool, "/brov/mission_complete", 10)
        self.pub_control_active = self.create_publisher(Bool, "/brov/control_active", 10)
        self.pub_camera_tilt_commanded = self.create_publisher(
            Float32, "/brov/camera_tilt/commanded", 10
        )
        self.pub_odometry = self.create_publisher(
            Odometry, "/brov/odometry/local", 10
        )
        self.pub_odometry_session = self.create_publisher(
            String, "/brov/odometry/session_id", latched_qos
        )
        self.pub_odometry_with_session = self.create_publisher(
            OdometrySession, "/brov/odometry/local_with_session", 10
        )
        self._odom_tf = TransformBroadcaster(self)

        self.srv_start_control = self.create_service(
            Trigger, "/brov/start_control", self._on_start_control
        )
        self.srv_prepare_control = self.create_service(
            Trigger, "/brov/prepare_control", self._on_prepare_control
        )
        self.srv_arm_control = self.create_service(
            Trigger,
            "/brov/arm_control",
            self._on_arm_control,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.srv_disarm_control = self.create_service(
            Trigger, "/brov/disarm_control", self._on_disarm_control
        )
        self.srv_stop_control = self.create_service(
            Trigger, "/brov/stop_control", self._on_stop_control
        )
        self.srv_reset_integrator = self.create_service(
            Trigger, "/brov/reset_integrator", self._on_reset_integrator
        )

        self._ready = False
        self._control_active = False
        # start_control 직후 policy가 이전(frozen) observation으로 계산한 PWM을 보내는
        # 경합을 막는다. 첫 active observation을 발행한 뒤에만 PWM을 허용한다.
        self._active_obs_published = False
        self._faulted = False
        self._last_sample_key = None
        self._last_sample_time = None
        self._last_wp_idx = -1
        self._logged_complete = False
        self._logged_integrator_clamp = False
        self._last_wait_reason = None
        self._last_no_snapshot_log = 0.0
        self._camera_tilt_target_deg = 0.0
        self._camera_tilt_commanded_deg = 0.0
        self._camera_tilt_has_command = False
        self._camera_tilt_last_update = time.monotonic()
        self._localization_status: LocalizationStatus | None = None
        self._resolved_mission: ResolvedMission | None = None
        self._active_localization_epoch: int | None = None
        self._active_odometry_session_id: str | None = None
        self._active_alignment_id: str | None = None
        self._active_plan_hash: str | None = None
        self._prepared = False
        self._prepared_mission_id: str | None = None
        self._prepared_plan_hash: str | None = None
        self._prepared_localization_epoch: int | None = None
        self._prepared_odometry_session_id: str | None = None
        self._prepared_alignment_id: str | None = None
        self._prepared_position_ned: torch.Tensor | None = None
        self._prepared_attitude_ned: torch.Tensor | None = None
        self._last_published_session_id = ""
        self._last_mavlink_reset_count = 0
        self._raw_odometry_session_id = ""
        self._navigation_jump_count = 0
        self._last_odom_position: torch.Tensor | None = None
        self._last_odom_orientation: torch.Tensor | None = None
        self._last_odom_sample_time: float | None = None
        self.timer = self.create_timer(0.04, self._tick)   # 현재 배포 pipeline: 25 Hz
        self.camera_tilt_timer = self.create_timer(0.05, self._camera_tilt_tick)  # 20Hz
        # Persistent SERVO/RC parameter mutation is deliberately the final
        # constructor step.  Any failure closes the MAVLink worker and restores
        # a partially-applied passthrough transaction before construction exits.
        if self._send_pwm:
            try:
                if self.interface.control_snapshot().get("armed"):
                    raise RuntimeError(
                        "refusing RCPassThru reconfiguration while vehicle is armed"
                    )
                self.interface.enable_passthrough()
            except BaseException:
                try:
                    self.interface.close(send_stop=False)
                except Exception as cleanup_error:
                    self.get_logger().error(
                        f"constructor cleanup failed: {cleanup_error}"
                    )
                raise
            self.get_logger().info(
                "RC7/RC8 camera option 격리 + RCPassThru 전환 완료 — "
                "/brov/thruster_pwm 수신 시 실제 송신"
            )
            if self._arm_permitted:
                self.get_logger().info(
                    "arm request recorded — constructor does not arm; call "
                    "/brov/prepare_control then /brov/arm_control"
                )
        self.get_logger().info(
            f"연결: {conn}, waypoints({waypoint_frame}, m): {waypoints.tolist()}, "
            f"heading_mode={heading_mode} — "
            "적분/PWM은 /brov/start_control 호출 전까지 동결"
        )
        if waypoint_bounds_enabled:
            self.get_logger().info(
                "waypoint input bounds enabled — "
                f"min={list(waypoint_min_xyz)}, max={list(waypoint_max_xyz)} "
                f"({waypoint_frame} frame; not a runtime geofence)"
            )
        self.get_logger().info(
            "camera tilt: /brov/camera_tilt/command [-1,1] → "
            f"[{self._camera_tilt_min_deg:.1f}, {self._camera_tilt_max_deg:.1f}]deg, "
            f"rate≤{self._camera_tilt_max_rate_deg_s:.1f}deg/s"
        )

    def _read_odom_covariance(self) -> dict[str, float]:
        values = {
            "position": float(
                self.get_parameter("odom_position_variance").value
            ),
            "orientation": float(
                self.get_parameter("odom_orientation_variance").value
            ),
            "linear_velocity": float(
                self.get_parameter("odom_linear_velocity_variance").value
            ),
            "angular_velocity": float(
                self.get_parameter("odom_angular_velocity_variance").value
            ),
        }
        for name, value in values.items():
            if not torch.isfinite(torch.tensor(value)) or value <= 0.0:
                raise ValueError(f"odom_{name}_variance must be finite and positive")
        return values

    @staticmethod
    def _diagonal_covariance(
        translation_variance: float, rotation_variance: float
    ) -> list[float]:
        covariance = [0.0] * 36
        for index in (0, 7, 14):
            covariance[index] = translation_variance
        for index in (21, 28, 35):
            covariance[index] = rotation_variance
        return covariance

    def _current_odometry_session_id(self, snap: dict) -> str:
        raw = str(snap.get("odometry_session_id", "")).strip()
        if not raw:
            raise RuntimeError("MAVLink snapshot has no odometry_session_id")
        return f"{raw}:nav{self._navigation_jump_count}"

    def _detect_odometry_discontinuity(
        self, converted, snap: dict
    ) -> str | None:
        """Advance a derived session when local position/attitude jumps.

        MAVLink boot time cannot reveal every DVL reconnect or EKF origin/yaw
        reset. This conservative adjacent-sample check catches discontinuities
        that are physically implausible within a short receive-time interval.
        """

        raw_session = str(snap.get("odometry_session_id", "")).strip()
        sample_time = max(float(snap["att_rx_time"]), float(snap["pos_rx_time"]))
        position = converted.position_odom.detach().clone()
        orientation = converted.orientation_xyzw.detach().clone()

        if raw_session != self._raw_odometry_session_id:
            self._raw_odometry_session_id = raw_session
            self._navigation_jump_count = 0
            self._last_odom_position = position
            self._last_odom_orientation = orientation
            self._last_odom_sample_time = sample_time
            return None

        reason = None
        if (
            self._last_odom_position is not None
            and self._last_odom_orientation is not None
            and self._last_odom_sample_time is not None
        ):
            dt = sample_time - self._last_odom_sample_time
            if 0.0 <= dt <= self._odom_jump_max_dt_s:
                translation = float(
                    torch.linalg.vector_norm(
                        position - self._last_odom_position
                    )
                )
                dot = float(
                    torch.dot(orientation, self._last_odom_orientation)
                )
                angle = 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))
                if translation > self._odom_jump_translation_m:
                    reason = (
                        f"local position discontinuity {translation:.3f}m "
                        f"in {dt:.3f}s"
                    )
                elif angle > self._odom_jump_rotation_rad:
                    reason = (
                        f"local attitude discontinuity "
                        f"{math.degrees(angle):.1f}deg in {dt:.3f}s"
                    )
        self._last_odom_position = position
        self._last_odom_orientation = orientation
        self._last_odom_sample_time = sample_time
        if reason is not None:
            self._navigation_jump_count += 1
        return reason

    def _publish_odometry(self, snap: dict) -> None:
        converted = ned_frd_to_odom_flu(
            snap["pos_ned"],
            snap["att_quat_ned"],
            snap["vel_ned"],
            snap["body_rates_ned"],
        )
        discontinuity = self._detect_odometry_discontinuity(converted, snap)
        session_id = self._current_odometry_session_id(snap)
        # These separate topics remain for standard-tool diagnostics. Pool
        # localization consumes the atomic OdometrySession envelope below.
        if session_id != self._last_published_session_id:
            self._last_published_session_id = session_id
            self.pub_odometry_session.publish(String(data=session_id))
            self.get_logger().info(f"odometry session: {session_id}")
        if discontinuity is not None:
            raise RuntimeError(
                f"{discontinuity}; derived odometry session advanced"
            )

        stamp = self.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        position = converted.position_odom.tolist()
        orientation = converted.orientation_xyzw.tolist()
        linear = converted.linear_velocity_body_flu.tolist()
        angular = converted.angular_velocity_body_flu.tolist()
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = float(orientation[0])
        message.pose.pose.orientation.y = float(orientation[1])
        message.pose.pose.orientation.z = float(orientation[2])
        message.pose.pose.orientation.w = float(orientation[3])
        message.twist.twist.linear.x = float(linear[0])
        message.twist.twist.linear.y = float(linear[1])
        message.twist.twist.linear.z = float(linear[2])
        message.twist.twist.angular.x = float(angular[0])
        message.twist.twist.angular.y = float(angular[1])
        message.twist.twist.angular.z = float(angular[2])
        message.pose.covariance = self._diagonal_covariance(
            self._odom_covariance["position"],
            self._odom_covariance["orientation"],
        )
        message.twist.covariance = self._diagonal_covariance(
            self._odom_covariance["linear_velocity"],
            self._odom_covariance["angular_velocity"],
        )
        envelope = OdometrySession()
        envelope.odometry = message
        envelope.odometry_session_id = session_id
        self.pub_odometry_with_session.publish(envelope)
        self.pub_odometry.publish(message)

        reset_count = int(snap.get("mavlink_time_reset_count", 0))
        if reset_count != self._last_mavlink_reset_count:
            self._last_mavlink_reset_count = reset_count
            if self._control_active:
                self._trip_fault("MAVLink boot-time reset changed odometry session")

        if self._publish_odom_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = message.pose.pose.position.x
            transform.transform.translation.y = message.pose.pose.position.y
            transform.transform.translation.z = message.pose.pose.position.z
            transform.transform.rotation = message.pose.pose.orientation
            self._odom_tf.sendTransform(transform)

    def _on_localization_status(self, message: LocalizationStatus) -> None:
        self._localization_status = message
        if self._control_active and self._require_pool_localization:
            reason = self._pool_localization_gate(self.interface.snapshot())
            if reason is not None:
                self._trip_fault(reason)

    def _clear_active_contract(self) -> None:
        self._active_localization_epoch = None
        self._active_odometry_session_id = None
        self._active_alignment_id = None
        self._active_plan_hash = None

    def _clear_prepared_contract(self) -> None:
        self._arm_transaction_generation += 1
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._prepared = False
        self._prepared_mission_id = None
        self._prepared_plan_hash = None
        self._prepared_localization_epoch = None
        self._prepared_odometry_session_id = None
        self._prepared_alignment_id = None
        self._prepared_position_ned = None
        self._prepared_attitude_ned = None

    def _actuation_mode_gate(self) -> str | None:
        state = self.interface.control_snapshot()
        heartbeat_age = float(state.get("heartbeat_age_s", float("inf")))
        if not math.isfinite(heartbeat_age) or heartbeat_age > self._heartbeat_max_age_s:
            return (
                f"autopilot heartbeat stale ({heartbeat_age:.3f}s > "
                f"{self._heartbeat_max_age_s:.3f}s)"
            )
        mode = state.get("custom_mode")
        if mode != self._required_custom_mode:
            return (
                f"ArduSub custom_mode={mode}; required MANUAL mode="
                f"{self._required_custom_mode}"
            )
        return None

    def _arm_lifecycle_gate(self, snap: dict | None) -> str | None:
        if snap is None:
            return "telemetry not ready"
        valid, reason = self._telemetry_valid(snap)
        if not valid:
            return reason
        reason = self._authority_gate()
        if reason is not None:
            return reason
        reason = self._actuation_mode_gate()
        if reason is not None:
            return reason
        if self._require_pool_localization:
            reason = self._pool_localization_gate(snap)
            if reason is not None:
                return reason
        if self._require_resolved_mission:
            reason = self._resolved_mission_gate(snap)
            if reason is not None:
                return reason
            reason = self._prepared_gate(snap)
            if reason is not None:
                return reason
        return None

    def _revoke_hardware_arm(self, reason: str) -> None:
        self._clear_prepared_contract()
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        cleanup_errors = self._neutral_and_disarm()
        suffix = (
            ""
            if not cleanup_errors
            else "; cleanup errors: " + "; ".join(cleanup_errors)
        )
        self.get_logger().error(
            f"HARDWARE ARM REVOKED — {reason}; neutral/disarm requested"
            f"{suffix}"
        )

    def _neutral_and_disarm(self) -> list[str]:
        """Attempt both safety actions even if the first transport call fails."""

        ObsNode._reset_pwm_rate_state(self)
        if not self._send_pwm:
            return []
        errors = []
        try:
            self.interface.neutral_stop()
        except Exception as error:
            errors.append(f"neutral failed: {error}")
        try:
            self.interface.disarm()
        except Exception as error:
            errors.append(f"disarm failed: {error}")
        return errors

    def _reset_pwm_rate_state(self, now: float | None = None) -> None:
        self._last_accepted_pwm = torch.zeros(8, dtype=torch.float32)
        self._last_accepted_pwm_monotonic = (
            time.monotonic() if now is None else float(now)
        )
        self._pwm_rate_first_command = True

    def _complete_random_mission(self, reason: str) -> None:
        """Close a finite v2 mission normally, without latching a fault."""

        self._control_active = False
        self._active_obs_published = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        self.pub_control_active.publish(Bool(data=False))
        self.pub_mission_complete.publish(Bool(data=True))
        cleanup_errors = self._neutral_and_disarm()
        suffix = (
            ""
            if not cleanup_errors
            else "; cleanup errors: " + "; ".join(cleanup_errors)
        )
        self.get_logger().info(
            f"RANDOM MISSION COMPLETE — {reason}; control frozen, "
            f"neutral/disarm requested{suffix}"
        )

    def _inactive_arm_watchdog(self, snap: dict | None) -> bool:
        if not self._arm_in_progress and not (
            self._hardware_arm_approved and not self._control_active
        ):
            return False
        reason = self._arm_lifecycle_gate(snap)
        if reason is None and self._hardware_arm_approved:
            state = self.interface.control_snapshot()
            if state.get("armed") is not True:
                reason = "autopilot armed state was lost before START"
            elif (
                self._hardware_arm_deadline is None
                or time.monotonic() > self._hardware_arm_deadline
            ):
                reason = "ARM-to-START approval timed out"
        if reason is None:
            return False
        self._revoke_hardware_arm(reason)
        return True

    def _authority_gate(self) -> str | None:
        if self._require_pool_localization:
            count = self.count_publishers("/brov/localization/status")
            if count != 1:
                return (
                    "expected exactly one localization status publisher; "
                    f"found {count}"
                )
        if self._require_resolved_mission:
            count = self.count_publishers("/brov/mission/resolved")
            if count != 1:
                return (
                    "expected exactly one resolved mission publisher; "
                    f"found {count}"
                )
        if self._send_pwm or self._require_resolved_mission:
            count = self.count_publishers("/brov/thruster_pwm")
            if count != 1:
                return (
                    "expected exactly one thruster PWM publisher; "
                    f"found {count}"
                )
        return None

    def _on_resolved_mission(self, message: ResolvedMission) -> None:
        if self._control_active:
            self.get_logger().warning(
                "resolved mission received while active — current mission remains immutable"
            )
            return
        if self._arm_in_progress or self._hardware_arm_approved:
            self._revoke_hardware_arm("resolved mission changed")
        else:
            self._clear_prepared_contract()
        self._resolved_mission = message
        self.get_logger().info(
            f"resolved pool mission received: id={message.mission_id}, "
            f"hash={message.plan_hash[:12]}, epoch={message.localization_epoch}"
        )

    def _pool_localization_gate(self, snap: dict | None) -> str | None:
        status = self._localization_status
        if status is None:
            return "pool localization status unavailable"
        if status.state != LocalizationStatus.INITIALIZED:
            return f"pool localization not initialized: {status.reason}"
        if not status.output_valid:
            return "pool localization output is not valid"
        if int(status.epoch) <= 0:
            return "pool localization initialized epoch must be non-zero"
        if not status.odometry_session_id:
            return "pool localization has empty odometry session"
        if not status.alignment_id:
            return "pool localization has empty alignment id"
        if status.header.frame_id != self._pool_frame:
            return (
                f"pool localization status frame must be {self._pool_frame!r}"
            )
        if snap is None:
            return "telemetry unavailable for localization session check"
        try:
            current_session = self._current_odometry_session_id(snap)
        except RuntimeError as error:
            return str(error)
        if status.odometry_session_id != current_session:
            return "pool localization odometry session mismatch"
        stamp_ns = (
            int(status.header.stamp.sec) * 1_000_000_000
            + int(status.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return "pool localization status has zero timestamp"
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age_s < -0.05 or age_s > self._localization_status_max_age_s:
            return f"pool localization status stale/future ({age_s:.3f}s)"
        if self._control_active and self._active_localization_epoch is not None:
            if int(status.epoch) != self._active_localization_epoch:
                return "pool localization epoch changed during active control"
            if status.odometry_session_id != self._active_odometry_session_id:
                return "odometry session changed during active control"
            if status.alignment_id != self._active_alignment_id:
                return "pool localization alignment changed during active control"
        return None

    def _resolved_mission_gate(self, snap: dict) -> str | None:
        mission = self._resolved_mission
        status = self._localization_status
        if mission is None:
            return "resolved pool mission unavailable"
        if mission.header.frame_id != self._odom_frame:
            return f"resolved mission frame must be {self._odom_frame!r}"
        waypoint_count = len(mission.waypoints)
        if waypoint_count < 2:
            return "resolved mission must contain at least two waypoints"
        if waypoint_count > self._max_resolved_waypoints:
            return "resolved mission exceeds waypoint count limit"
        if (
            not mission.mission_id
            or re.fullmatch(r"[0-9a-fA-F]{64}", mission.plan_hash) is None
        ):
            return "resolved mission identity/hash invalid"
        if mission.contract_version not in {_POOL_MISSION_V1, _POOL_MISSION_V2}:
            return "resolved mission contract version unsupported"
        try:
            canonical_bytes = mission.canonical_plan_json.encode("ascii")
        except UnicodeEncodeError:
            return "resolved mission canonical content must be ASCII"
        if hashlib.sha256(canonical_bytes).hexdigest() != mission.plan_hash:
            return "resolved mission canonical content/hash mismatch"
        try:
            canonical = json.loads(mission.canonical_plan_json)
        except (TypeError, ValueError):
            return "resolved mission canonical content is invalid JSON"
        if not isinstance(canonical, dict):
            return "resolved mission canonical content must be an object"
        if (
            canonical.get("contract") != mission.contract_version
            or canonical.get("frame_id") != self._pool_frame
        ):
            return "resolved mission canonical frame/contract mismatch"
        expected_keys = (
            _CANONICAL_V1_KEYS
            if mission.contract_version == _POOL_MISSION_V1
            else _CANONICAL_V2_KEYS
        )
        if set(canonical) != expected_keys:
            return "resolved mission canonical top-level keys mismatch"
        random_config = None
        if mission.contract_version == _POOL_MISSION_V1:
            # V1 is frozen as the original position-only straight/align
            # contract.  Random behavior must never be smuggled into it.
            if mission.heading_mode not in {"straight", "align"}:
                return "resolved mission v1 heading_mode is not allowed"
        else:
            if mission.heading_mode != "random_at_waypoint":
                return "resolved mission v2 requires random_at_waypoint"
            if not bool(mission.loop):
                return "resolved mission v2 requires loop=true"
            try:
                random_config = _random_attitude_config(canonical)
            except ValueError as error:
                return str(error)
            if (
                random_config.max_slew_rate_rad_s
                > self._max_random_attitude_slew_rate_rad_s
            ):
                return "random attitude slew rate exceeds operational limit"
            if (
                random_config.attitude_tolerance_rad
                > self._max_random_attitude_tolerance_rad
            ):
                return "random attitude tolerance exceeds operational limit"
            if (
                random_config.angular_speed_tolerance_rad_s
                > self._max_random_angular_speed_tolerance_rad_s
            ):
                return "random angular-speed tolerance exceeds operational limit"
            if random_config.dwell_time_s < self._min_random_attitude_dwell_s:
                return "random attitude dwell time is below operational minimum"
            if (
                random_config.max_duration_s
                > self._max_random_mission_duration_s
            ):
                return "random mission duration exceeds operational limit"
            if random_config.max_laps > self._max_random_mission_laps:
                return "random mission lap count exceeds operational limit"
            if self._send_pwm and self._max_pwm_delta_per_s <= 0.0:
                return "random mission requires max_pwm_delta_per_s > 0"
        if not mission.alignment_id:
            return "resolved mission alignment id is empty"
        if status is None:
            return "localization unavailable for mission check"
        if int(mission.localization_epoch) != int(status.epoch):
            return "resolved mission localization epoch mismatch"
        if mission.odometry_session_id != status.odometry_session_id:
            return "resolved mission odometry session mismatch"
        if mission.alignment_id != status.alignment_id:
            return "resolved mission localization alignment mismatch"
        try:
            current_session = self._current_odometry_session_id(snap)
        except RuntimeError as error:
            return str(error)
        if mission.odometry_session_id != current_session:
            return "resolved mission does not belong to current odometry session"
        points = torch.tensor(
            [[point.x, point.y, point.z] for point in mission.waypoints],
            dtype=torch.float32,
        )
        if not torch.isfinite(points).all():
            return "resolved mission contains NaN/Inf"
        try:
            pool_points = torch.tensor(
                canonical["waypoints"], dtype=torch.float32
            )
        except (KeyError, TypeError, ValueError):
            return "resolved mission canonical waypoints invalid"
        if pool_points.shape != points.shape or not torch.isfinite(
            pool_points
        ).all():
            return "resolved mission canonical waypoint shape/value mismatch"
        for name, message_value in (
            ("cruise_speed", float(mission.cruise_speed)),
            ("lookahead_dist", float(mission.lookahead_dist)),
            ("reach_threshold", float(mission.reach_threshold)),
        ):
            try:
                canonical_value = float(canonical[name])
            except (KeyError, TypeError, ValueError):
                return f"resolved mission canonical {name} invalid"
            if (
                not math.isfinite(canonical_value)
                or abs(canonical_value - message_value) > 1e-6
            ):
                return f"resolved mission canonical {name} mismatch"
        if (
            canonical.get("heading_mode") != mission.heading_mode
            or not isinstance(canonical.get("loop"), bool)
            or canonical["loop"] != bool(mission.loop)
        ):
            return "resolved mission canonical guidance settings mismatch"

        transform = status.pool_to_odom
        transform_values = torch.tensor(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
            dtype=torch.float32,
        )
        if not torch.isfinite(transform_values).all():
            return "pool localization transform contains NaN/Inf"
        q_xyzw = transform_values[3:]
        q_norm = float(torch.linalg.vector_norm(q_xyzw))
        if abs(q_norm - 1.0) > 1e-3:
            return "pool localization transform quaternion invalid"
        q_op_wxyz = torch.stack(
            (q_xyzw[3], -q_xyzw[0], -q_xyzw[1], -q_xyzw[2])
        ) / q_norm
        t_po = transform_values[:3]
        t_op = mu.quat_apply(q_op_wxyz, -t_po)
        expected_odom = mu.quat_apply(
            q_op_wxyz.unsqueeze(0).expand(pool_points.shape[0], -1),
            pool_points,
        ) + t_op
        if float(torch.max(torch.abs(expected_odom - points))) > 1e-3:
            return "resolved odom waypoints do not match bound pool alignment"
        segment_vectors = points[1:] - points[:-1]
        if mission.contract_version == _POOL_MISSION_V2 and bool(mission.loop):
            segment_vectors = torch.cat(
                (segment_vectors, (points[0] - points[-1]).unsqueeze(0)),
                dim=0,
            )
        segment_lengths = torch.linalg.vector_norm(segment_vectors, dim=1)
        if bool((segment_lengths <= 0.0).any()):
            return "resolved mission contains a zero-length segment"
        if bool(
            (segment_lengths > self._max_resolved_segment_length_m).any()
        ):
            return "resolved mission segment exceeds operational limit"
        settings = (
            (
                "cruise_speed",
                float(mission.cruise_speed),
                self._max_resolved_cruise_speed,
            ),
            (
                "lookahead_dist",
                float(mission.lookahead_dist),
                self._max_resolved_lookahead_dist,
            ),
            (
                "reach_threshold",
                float(mission.reach_threshold),
                self._max_resolved_reach_threshold,
            ),
        )
        for name, value, maximum in settings:
            if not math.isfinite(value) or value <= 0.0 or value > maximum:
                return f"resolved mission {name} outside operational range"
        if mission.contract_version == _POOL_MISSION_V1:
            if mission.heading_mode not in {"straight", "align"}:
                return "resolved mission heading_mode is not allowed"
        elif random_config is None:
            return "resolved mission v2 random metadata unavailable"
        return None

    def _load_resolved_guidance(self, snap: dict) -> None:
        mission = self._resolved_mission
        assert mission is not None
        random_config = None
        pool_to_mission_q = None
        if mission.contract_version == _POOL_MISSION_V2:
            canonical = json.loads(mission.canonical_plan_json)
            random_config = _random_attitude_config(canonical)
            status = self._localization_status
            if status is None:
                raise ValueError(
                    "localization unavailable for pool attitude conversion"
                )
            rotation = status.pool_to_odom.rotation
            pool_to_mission_q = pool_to_mission_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w),
                snap["att_quat_ned"],
                self.obs_builder.waypoint_frame,
            )
        points_odom = torch.tensor(
            [[point.x, point.y, point.z] for point in mission.waypoints],
            dtype=torch.float32,
        )
        waypoints = odom_waypoints_to_mission(
            points_odom,
            snap["pos_ned"],
            snap["att_quat_ned"],
            self.obs_builder.waypoint_frame,
        )
        first_distance = float(torch.linalg.vector_norm(waypoints[0, 0]))
        if first_distance > self._max_mission_start_distance_m:
            raise ValueError(
                f"resolved first waypoint is {first_distance:.3f} m from "
                f"current pose (limit {self._max_mission_start_distance_m:.3f} m)"
            )
        self.guidance = LOSGuidance(
            waypoints,
            "cpu",
            cruise_speed=float(mission.cruise_speed),
            lookahead_dist=float(mission.lookahead_dist),
            heading_mode=str(mission.heading_mode),
            reach_threshold=float(mission.reach_threshold),
            loop=bool(mission.loop),
            depth_hold_kp=self.get_parameter("depth_hold_kp").value,
            depth_speed_limit=self.get_parameter("depth_speed_limit").value,
            terminal_hold_kp=self.get_parameter("terminal_hold_kp").value,
            terminal_speed_limit=self.get_parameter("terminal_speed_limit").value,
            random_attitude_config=random_config,
            pool_to_mission_quaternion=pool_to_mission_q,
        )
        self.get_logger().info(
            f"pool mission prepared atomically: id={mission.mission_id}, "
            f"hash={mission.plan_hash[:12]}, epoch={mission.localization_epoch}, "
            f"resolved waypoints={waypoints.tolist()}"
        )

    def _reset_guidance_at_snapshot(self, snap: dict) -> None:
        self.obs_builder.reset(snap["pos_ned"], snap["att_quat_ned"])
        initial_quat = self.obs_builder.attitude_in_waypoint_frame(
            snap["att_quat_ned"]
        ).unsqueeze(0)
        self.guidance.reset(
            torch.zeros(1, dtype=torch.long),
            initial_quat=initial_quat,
            resample_random_attitude=False,
        )
        self._last_wp_idx = -1
        self._logged_complete = False
        self._last_sample_time = max(snap["att_rx_time"], snap["pos_rx_time"])

    def _on_prepare_control(self, _request, response):
        snap = self.interface.snapshot()
        if self._estopped:
            response.success, response.message = False, "estop latched"
            return response
        if self._faulted:
            response.success, response.message = False, "fault latched"
            return response
        if self._control_active:
            response.success, response.message = False, "control is active"
            return response
        if self._send_pwm and self.interface.control_snapshot().get("armed"):
            self._revoke_hardware_arm("PREPARE requested while vehicle armed")
            response.success = False
            response.message = (
                "vehicle was armed; neutral/disarm requested; call PREPARE again"
            )
            return response
        # Re-prepare is a new transaction.  It cancels any concurrent ARM and
        # discards the previous preview before validating the replacement.
        self._clear_prepared_contract()
        if not self._ready or snap is None:
            response.success, response.message = False, "telemetry not ready"
            return response
        valid, reason = self._telemetry_valid(snap)
        if not valid:
            response.success, response.message = False, reason
            return response
        reason = self._authority_gate()
        if reason is not None:
            response.success, response.message = False, reason
            return response
        if self._send_pwm:
            reason = self._actuation_mode_gate()
            if reason is not None:
                response.success, response.message = False, reason
                return response
        if self._require_pool_localization:
            reason = self._pool_localization_gate(snap)
            if reason is not None:
                response.success, response.message = False, reason
                return response
        if not self._require_resolved_mission:
            response.success = False
            response.message = "prepare_control is for resolved pool missions"
            return response
        reason = self._resolved_mission_gate(snap)
        if reason is not None:
            response.success, response.message = False, reason
            return response
        try:
            self._load_resolved_guidance(snap)
            self._reset_guidance_at_snapshot(snap)
        except (AssertionError, RuntimeError, ValueError) as error:
            self._clear_prepared_contract()
            response.success = False
            response.message = f"resolved mission rejected: {error}"
            return response

        mission = self._resolved_mission
        assert mission is not None
        self._prepared = True
        self._prepared_mission_id = mission.mission_id
        self._prepared_plan_hash = mission.plan_hash
        self._prepared_localization_epoch = int(mission.localization_epoch)
        self._prepared_odometry_session_id = mission.odometry_session_id
        self._prepared_alignment_id = mission.alignment_id
        self._prepared_position_ned = snap["pos_ned"].clone()
        self._prepared_attitude_ned = snap["att_quat_ned"].clone()
        self._hardware_arm_approved = False
        response.success = True
        response.message = (
            "committed mission prepared in frozen preview; inspect target/action, "
            "then call arm_control and start_control"
        )
        return response

    def _prepared_gate(self, snap: dict) -> str | None:
        if not self._prepared:
            return "resolved mission not prepared; call /brov/prepare_control"
        mission = self._resolved_mission
        status = self._localization_status
        if mission is None or status is None:
            return "prepared mission/localization unavailable"
        if mission.mission_id != self._prepared_mission_id:
            return "resolved mission changed after prepare"
        if mission.plan_hash != self._prepared_plan_hash:
            return "resolved mission hash changed after prepare"
        if int(status.epoch) != self._prepared_localization_epoch:
            return "localization epoch changed after prepare"
        if status.odometry_session_id != self._prepared_odometry_session_id:
            return "odometry session changed after prepare"
        if status.alignment_id != self._prepared_alignment_id:
            return "localization alignment changed after prepare"
        assert self._prepared_position_ned is not None
        assert self._prepared_attitude_ned is not None
        position_drift = float(
            torch.linalg.vector_norm(
                snap["pos_ned"] - self._prepared_position_ned
            )
        )
        dot = float(
            torch.dot(snap["att_quat_ned"], self._prepared_attitude_ned)
        )
        attitude_drift = 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))
        if position_drift > self._max_prepare_position_drift_m:
            return (
                f"vehicle moved {position_drift:.3f}m after prepare; "
                "prepare again"
            )
        if attitude_drift > self._max_prepare_attitude_drift_rad:
            return (
                f"vehicle rotated {math.degrees(attitude_drift):.1f}deg "
                "after prepare; prepare again"
            )
        return None

    def _on_arm_control(self, _request, response):
        if not self._send_pwm:
            response.success = False
            response.message = "send_pwm=false; hardware arming is disabled"
            return response
        if not self._arm_permitted:
            response.success = False
            response.message = "arm parameter is false; ROS arming not permitted"
            return response
        if self._estopped or self._faulted or self._control_active:
            response.success = False
            response.message = "arming blocked by estop/fault/active state"
            return response
        snap = self.interface.snapshot()
        reason = self._arm_lifecycle_gate(snap)
        if reason is not None:
            response.success, response.message = False, reason
            return response

        generation = self._arm_transaction_generation
        self._arm_in_progress = True
        try:
            self.interface.neutral_stop()
            armed = self.interface.arm(
                cancel_check=lambda: (
                    self._estopped
                    or self._faulted
                    or generation != self._arm_transaction_generation
                )
            )
            if not armed:
                raise RuntimeError("hardware arm failed or was cancelled")

            post_snap = self.interface.snapshot()
            post_reason = self._arm_lifecycle_gate(post_snap)
            cancelled = (
                self._estopped
                or self._faulted
                or generation != self._arm_transaction_generation
            )
            if cancelled or post_reason is not None:
                raise RuntimeError(
                    "hardware arm invalidated after ACK: "
                    + (post_reason or "lifecycle cancelled")
                )

            self._hardware_arm_approved = True
            self._hardware_arm_deadline = (
                time.monotonic() + self._arm_to_start_timeout_s
            )
            response.success = True
            response.message = (
                "hardware armed after neutral and post-ACK lifecycle gates; "
                f"start within {self._arm_to_start_timeout_s:.1f}s"
            )
            return response
        except Exception as error:
            self._revoke_hardware_arm(str(error))
            response.success = False
            response.message = str(error)
            return response
        finally:
            self._arm_in_progress = False

    def _on_disarm_control(self, _request, response):
        self._control_active = False
        self._active_obs_published = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        cleanup_errors = self._neutral_and_disarm()
        response.success = not cleanup_errors
        response.message = (
            "control frozen, neutral/disarm requested"
            if not cleanup_errors
            else "control frozen; " + "; ".join(cleanup_errors)
        )
        return response

    def _telemetry_valid(self, snap: dict) -> tuple[bool, str]:
        if snap["att_age_s"] >= float(self.get_parameter("att_max_age_s").value):
            return False, f"attitude stale ({snap['att_age_s']:.3f}s)"
        if snap["pos_age_s"] >= float(self.get_parameter("pos_max_age_s").value):
            return False, f"position stale ({snap['pos_age_s']:.3f}s)"
        if snap["ekf_age_s"] >= float(self.get_parameter("ekf_max_age_s").value):
            return False, f"EKF status stale ({snap['ekf_age_s']:.3f}s)"
        if not self.interface.is_ekf_healthy(snap):
            return False, f"EKF unhealthy (velocity_variance={snap['ekf_vel_variance']})"
        flags = snap.get("ekf_flags")
        if flags is None:
            return False, "EKF status flags unavailable"
        missing_flags = self._required_ekf_flags & ~int(flags)
        if missing_flags:
            return False, (
                f"EKF required flags missing (flags={int(flags)}, "
                f"missing_mask={missing_flags})"
            )

        att_boot = snap.get("att_time_boot_ms")
        pos_boot = snap.get("pos_time_boot_ms")
        if att_boot is None or pos_boot is None:
            return False, "telemetry source timestamps unavailable"
        raw_delta = (int(att_boot) - int(pos_boot)) % (1 << 32)
        signed_delta = (
            raw_delta - (1 << 32)
            if raw_delta >= (1 << 31)
            else raw_delta
        )
        if abs(signed_delta) > self._telemetry_source_skew_max_ms:
            return False, (
                f"attitude/position source-time skew {signed_delta}ms exceeds "
                f"{self._telemetry_source_skew_max_ms}ms"
            )

        tensors = (
            snap["att_quat_ned"], snap["body_rates_ned"],
            snap["pos_ned"], snap["vel_ned"],
        )
        if not all(torch.isfinite(value).all() for value in tensors):
            return False, "telemetry NaN/Inf"
        q_norm = float(snap["att_quat_ned"].norm())
        q_tol = float(self.get_parameter("quat_norm_tolerance").value)
        if abs(q_norm - 1.0) > q_tol:
            return False, f"quaternion norm invalid ({q_norm:.4f})"
        return True, "ok"

    def _publish_raw(self, snap: dict) -> None:
        self.pub_pos.publish(Float32MultiArray(data=snap["pos_ned"].tolist()))
        self.pub_vel.publish(Float32MultiArray(data=snap["vel_ned"].tolist()))
        self.pub_quat.publish(Float32MultiArray(data=snap["att_quat_ned"].tolist()))
        control = self.interface.control_snapshot()
        if control["servo_output_us"] is not None:
            self.pub_servo_output.publish(
                Int32MultiArray(data=control["servo_output_us"].tolist())
            )

    def _trip_fault(self, reason: str) -> None:
        if self._faulted:
            return
        self._faulted = True
        self._control_active = False
        self._active_obs_published = False
        self._camera_tilt_has_command = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        self.get_logger().error(f"CONTROL FAULT — {reason}; 적분 중지/PWM neutral")
        cleanup_errors = self._neutral_and_disarm()
        if cleanup_errors:
            self.get_logger().error("; ".join(cleanup_errors))

    def _on_start_control(self, _request, response):
        snap = self.interface.snapshot()
        if self._estopped:
            response.success, response.message = False, "estop latched"
            return response
        if self._faulted:
            response.success, response.message = False, "fault latched; reset_integrator 필요"
            return response
        if self._control_active:
            response.success, response.message = True, "control already active"
            return response
        if not self._ready or snap is None:
            response.success, response.message = False, "telemetry not ready"
            return response
        valid, reason = self._telemetry_valid(snap)
        if not valid:
            response.success, response.message = False, reason
            return response
        reason = self._authority_gate()
        if reason is not None:
            response.success, response.message = False, reason
            return response

        if self._require_pool_localization:
            reason = self._pool_localization_gate(snap)
            if reason is not None:
                response.success, response.message = False, reason
                return response
        if self._require_resolved_mission:
            reason = self._resolved_mission_gate(snap)
            if reason is not None:
                response.success, response.message = False, reason
                return response
            reason = self._prepared_gate(snap)
            if reason is not None:
                response.success, response.message = False, reason
                return response

        if self._send_pwm:
            reason = self._actuation_mode_gate()
            if reason is not None:
                response.success, response.message = False, reason
                return response
            armed = self.interface.control_snapshot().get("armed") is True
            if not self._hardware_arm_approved or not armed:
                response.success = False
                response.message = (
                    "hardware output not approved/armed; call /brov/arm_control"
                )
                return response

        if self._require_resolved_mission:
            self._active_localization_epoch = self._prepared_localization_epoch
            self._active_odometry_session_id = (
                self._prepared_odometry_session_id
            )
            self._active_alignment_id = self._prepared_alignment_id
            self._active_plan_hash = self._prepared_plan_hash
        elif self._require_pool_localization:
            status = self._localization_status
            assert status is not None
            self._active_localization_epoch = int(status.epoch)
            self._active_odometry_session_id = status.odometry_session_id
            self._active_alignment_id = status.alignment_id
            self._active_plan_hash = None
        else:
            self._clear_active_contract()

        if self._require_resolved_mission:
            # prepare_control already fixed the mission frame and published the
            # exact frozen observation/target for inspection. Preserve that
            # frame and only clear integral history at output enable.
            self.obs_builder.reset_integrators()
        else:
            # Legacy relative missions keep their original start-time reset.
            self._reset_guidance_at_snapshot(snap)
        initial_yaw_ned_deg = float(
            torch.rad2deg(mu.yaw_from_quat(snap["att_quat_ned"]))
        )
        self._last_wp_idx = -1
        self._logged_complete = False
        self._last_sample_time = max(snap["att_rx_time"], snap["pos_rx_time"])
        self._control_active = True
        self._active_obs_published = False
        self._hardware_arm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._first_pwm_deadline = (
            time.monotonic() + self._first_pwm_timeout_s
            if self._send_pwm
            else None
        )
        self._reset_pwm_rate_state()
        self._logged_integrator_clamp = False
        self.get_logger().info(
            "CONTROL ACTIVE — z_v/z_q reset 후 적분/PWM 허용; "
            f"frame={self.obs_builder.waypoint_frame}, "
            f"initial_yaw_ned={initial_yaw_ned_deg:.1f}deg"
        )
        response.success, response.message = True, "control active; integrators reset"
        return response

    def _on_stop_control(self, _request, response):
        self._control_active = False
        self._active_obs_published = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        if self._send_pwm:
            self.interface.neutral_stop()
        self._reset_pwm_rate_state()
        self.get_logger().info("CONTROL STOPPED — 적분 동결/PWM neutral")
        response.success, response.message = True, "control stopped; integrators frozen"
        return response

    def _on_reset_integrator(self, _request, response):
        self._control_active = False
        self._active_obs_published = False
        self._faulted = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        self.obs_builder.reset_integrators()
        snap = self.interface.snapshot()
        self._last_sample_time = (
            None if snap is None else max(snap["att_rx_time"], snap["pos_rx_time"])
        )
        if self._send_pwm:
            self.interface.neutral_stop()
        self._reset_pwm_rate_state()
        self.get_logger().info("integrator/fault reset — control은 frozen 상태")
        response.success, response.message = True, "integrators reset; control frozen"
        return response

    def _tick(self):
        snap = self.interface.snapshot()
        if self._inactive_arm_watchdog(snap):
            return
        if snap is None:
            # An active node has previously held a complete telemetry
            # snapshot.  Losing both streams (for example during an autopilot
            # reset) is therefore a control fault, not an ordinary startup
            # wait.  Close the controller gate before returning.
            if self._control_active:
                self.pub_control_active.publish(Bool(data=False))
                self._trip_fault("telemetry snapshot became unavailable")
                return
            now = time.monotonic()
            if now - self._last_no_snapshot_log >= 2.0:
                self.get_logger().warn(
                    "telemetry 대기 중 — ATTITUDE_QUATERNION/LOCAL_POSITION_NED 미수신"
                )
                self._last_no_snapshot_log = now
            return

        # 새 패킷이 없어도 매 timer tick에서 age를 다시 계산한 snapshot으로 stale을
        # 검사해야, MAVLink가 완전히 끊겼을 때도 active control을 확실히 fault 처리한다.
        # Delay the public output-enable edge until this START generation has
        # published at least one observation.  A controller therefore cannot
        # release a queued preview command at the instant START flips the
        # internal state.
        output_enabled = self._control_active and self._active_obs_published
        self.pub_control_active.publish(Bool(data=output_enabled))
        valid, reason = self._telemetry_valid(snap)
        if not valid:
            reason_kind = reason.split(" (", 1)[0]
            if reason_kind != self._last_wait_reason:
                self.get_logger().warn(f"observation gated: {reason}")
                self._last_wait_reason = reason_kind
            if self._control_active:
                self._trip_fault(reason)
            return
        self._last_wait_reason = None

        # This gate is checked before the duplicate-snapshot early return so a
        # lost localization heartbeat cannot stay hidden while MAVLink is idle.
        if self._control_active and self._require_pool_localization:
            reason = self._pool_localization_gate(snap)
            if reason is not None:
                self._trip_fault(reason)
                return

        if self._control_active and self._send_pwm:
            now = time.monotonic()
            if self._last_pwm_rx_monotonic is None:
                if (
                    self._first_pwm_deadline is not None
                    and now > self._first_pwm_deadline
                ):
                    self._trip_fault("first controller PWM command timeout")
                    return
            elif now - self._last_pwm_rx_monotonic > self._pwm_command_timeout_s:
                self._trip_fault(
                    "controller PWM watchdog timeout "
                    f"({now - self._last_pwm_rx_monotonic:.3f}s)"
                )
                return

        sample_key = (snap["att_seq"], snap["pos_seq"])
        if sample_key == self._last_sample_key:
            return   # 같은 MAVLink snapshot을 timer가 다시 읽어도 재적분/재발행하지 않음
        self._last_sample_key = sample_key
        self._publish_raw(snap)
        try:
            self._publish_odometry(snap)
        except (RuntimeError, ValueError) as error:
            if self._control_active:
                self._trip_fault(f"odometry publication failed: {error}")
            else:
                self.get_logger().error(f"odometry publication failed: {error}")
            return

        if not self._ready:
            self.obs_builder.reset(snap["pos_ned"], snap["att_quat_ned"])
            initial_quat = self.obs_builder.attitude_in_waypoint_frame(
                snap["att_quat_ned"]
            ).unsqueeze(0)
            self.guidance.reset(torch.zeros(1, dtype=torch.long), initial_quat=initial_quat)
            self._ready = True
            self._last_sample_time = max(snap["att_rx_time"], snap["pos_rx_time"])
            self.get_logger().info("첫 healthy telemetry 확보 — frozen obs 발행 시작")

        sample_time = max(snap["att_rx_time"], snap["pos_rx_time"])
        dt = 0.0 if self._last_sample_time is None else sample_time - self._last_sample_time
        self._last_sample_time = sample_time
        if dt < 0.0:
            if self._control_active:
                self._trip_fault(f"negative telemetry dt ({dt:.6f}s)")
            return
        max_dt = float(self.get_parameter("max_integration_dt_s").value)
        if self._control_active and dt > max_dt:
            self._trip_fault(f"telemetry dt too large ({dt:.6f}s > {max_dt:.6f}s)")
            return

        obs, debug = self.obs_builder.build(
            snap["att_quat_ned"], snap["body_rates_ned"],
            snap["pos_ned"], snap["vel_ned"], self.guidance,
            dt if self._control_active else 0.0,
            integrate=self._control_active,
            advance_waypoint=self._control_active,
        )
        if obs.shape != (16,) or not torch.isfinite(obs).all():
            if self._control_active:
                self._trip_fault("invalid observation shape or NaN/Inf")
            return
        random_goal_pool = self.guidance.random_goal_pool
        if random_goal_pool is not None:
            self.pub_q_random_goal_pool.publish(
                Float32MultiArray(data=random_goal_pool.tolist())
            )
        if self._control_active and self.guidance.termination_reason is not None:
            self._complete_random_mission(self.guidance.termination_reason)
            return
        if debug["integrator_clamped"] and not self._logged_integrator_clamp:
            self._logged_integrator_clamp = True
            self.get_logger().warn("integrator clamp 도달 — 학습 범위 경계에서 유지")
        self.pub_obs.publish(Float32MultiArray(data=obs.tolist()))
        self.pub_pos_mission.publish(Float32MultiArray(data=debug["pos_env"].tolist()))
        self.pub_v_body_zup.publish(
            Float32MultiArray(data=debug["v_body_zup"].tolist())
        )
        self.pub_v_desired_body_zup.publish(
            Float32MultiArray(data=debug["v_d_b_zup"].tolist())
        )
        self.pub_q_desired_zup.publish(
            Float32MultiArray(data=debug["q_d_zup"].tolist())
        )
        if self._control_active:
            self._active_obs_published = True

        # 개선 1: 지금 추종 중인 웨이포인트 발행 — guidance.compute()가 위 build() 안에서
        # 이미 _wp_idx를 갱신했으므로 여기서 그 결과를 그대로 읽기만 하면 됨.
        idx = int(self.guidance._wp_idx[0].item())
        target_wp = self.guidance._wp[0, (idx + 1) % self.guidance.num_wp]
        self.pub_target_wp.publish(Float32MultiArray(data=target_wp.tolist()))
        self.pub_wp_idx.publish(Int32(data=idx))
        self.pub_mission_complete.publish(Bool(data=bool(self.guidance.mission_complete[0])))

        if idx != self._last_wp_idx:
            self._last_wp_idx = idx
            self.get_logger().info(f"웨이포인트 전환: idx={idx} → target={target_wp.tolist()}")
        if bool(self.guidance.mission_complete[0]) and not self._logged_complete:
            self._logged_complete = True
            self.get_logger().info(
                "미션 완료 — 마지막 웨이포인트 도달, terminal position hold 유지 중"
            )

    def _on_pwm(self, msg: Float32MultiArray) -> None:
        if (self._estopped or self._faulted or not self._send_pwm
                or not self._control_active or not self._active_obs_published):
            return
        if len(msg.data) != 8:
            self.get_logger().warn(f"pwm 차원 {len(msg.data)} != 8 — 무시")
            return
        pwm = torch.tensor(msg.data, dtype=torch.float32)
        if not torch.isfinite(pwm).all() or bool(
            (pwm.abs() > self._max_pwm_abs).any()
        ):
            self._trip_fault(
                "invalid PWM command (NaN/Inf or exceeds max_pwm_abs)"
            )
            return
        now = time.monotonic()
        if self._max_pwm_delta_per_s > 0.0:
            previous_time = self._last_accepted_pwm_monotonic
            if previous_time is None:
                previous_time = now
            dt = max(0.0, now - previous_time)
            if self._pwm_rate_first_command:
                dt = max(dt, self._pwm_rate_first_command_dt_s)
            maximum_delta = self._max_pwm_delta_per_s * dt + 1e-6
            actual_delta = float(
                torch.max(torch.abs(pwm - self._last_accepted_pwm))
            )
            if actual_delta > maximum_delta:
                self._trip_fault(
                    "PWM slew-rate limit exceeded "
                    f"({actual_delta:.4f} > {maximum_delta:.4f})"
                )
                return
        snap = self.interface.snapshot()
        if snap is None:
            self._trip_fault("PWM received without telemetry")
            return
        valid, reason = self._telemetry_valid(snap)
        if not valid:
            self._trip_fault(f"PWM blocked: {reason}")
            return
        reason = self._authority_gate()
        if reason is not None:
            self._trip_fault(f"PWM blocked: {reason}")
            return
        reason = self._actuation_mode_gate()
        if reason is not None:
            self._trip_fault(f"PWM blocked: {reason}")
            return
        if self._require_pool_localization:
            reason = self._pool_localization_gate(snap)
            if reason is not None:
                self._trip_fault(f"PWM blocked: {reason}")
                return
        if (
            not self._hardware_arm_approved
            or self.interface.control_snapshot().get("armed") is not True
        ):
            self._trip_fault("PWM blocked: hardware arm approval/state lost")
            return
        self.interface.send_pwm(pwm)
        self._last_accepted_pwm = pwm.clone()
        self._last_accepted_pwm_monotonic = now
        self._pwm_rate_first_command = False
        self._last_pwm_rx_monotonic = time.monotonic()

    def _on_camera_tilt(self, msg: Float32) -> None:
        """정규화된 사용자/RL tilt 목표를 각도 목표로 변환한다."""
        value = float(msg.data)
        if self._estopped or self._faulted:
            self.get_logger().warn("camera tilt 명령 무시 — estop/fault latched")
            return
        if self._require_pool_localization and (
            self._localization_status is None
            or self._localization_status.state
            != LocalizationStatus.INITIALIZED
        ):
            self.get_logger().warn(
                "camera tilt command ignored before pool initialization; "
                "the calibrated neutral extrinsic must remain fixed"
            )
            return
        if not torch.isfinite(torch.tensor(value)) or not -1.0 <= value <= 1.0:
            self.get_logger().warn(f"camera tilt {value} outside [-1,1] — 무시")
            return
        self._camera_tilt_target_deg = (
            value * self._camera_tilt_max_deg if value >= 0.0
            else -value * self._camera_tilt_min_deg
        )
        self._camera_tilt_has_command = True

    def _camera_tilt_tick(self) -> None:
        """카메라 목표각에 rate limit을 적용해 20Hz mount 명령으로 보낸다."""
        now = time.monotonic()
        dt = min(max(now - self._camera_tilt_last_update, 0.0), 0.2)
        self._camera_tilt_last_update = now
        if not self._camera_tilt_has_command or self._estopped or self._faulted:
            return

        error = self._camera_tilt_target_deg - self._camera_tilt_commanded_deg
        max_step = self._camera_tilt_max_rate_deg_s * dt
        step = max(-max_step, min(max_step, error))
        self._camera_tilt_commanded_deg += step
        normalized = (
            self._camera_tilt_commanded_deg / self._camera_tilt_max_deg
            if self._camera_tilt_commanded_deg >= 0.0
            else -self._camera_tilt_commanded_deg / self._camera_tilt_min_deg
        )
        self.interface.set_camera_tilt(
            normalized,
            min_pitch_deg=self._camera_tilt_min_deg,
            max_pitch_deg=self._camera_tilt_max_deg,
        )
        self.pub_camera_tilt_commanded.publish(Float32(data=float(normalized)))
        if abs(error) <= max_step:
            self._camera_tilt_has_command = False

    def _on_estop(self, _msg: Empty) -> None:
        if self._estopped:
            return   # 이미 트립됨 — 중복 처리 불필요
        self._estopped = True
        self._control_active = False
        self._active_obs_published = False
        self._camera_tilt_has_command = False
        self._hardware_arm_approved = False
        self._hardware_arm_deadline = None
        self._first_pwm_deadline = None
        self._last_pwm_rx_monotonic = None
        self._clear_active_contract()
        self._clear_prepared_contract()
        self.get_logger().error("ESTOP 수신 — 즉시 중립 정지 + disarm, 재시작 전까지 재개 안 함")
        cleanup_errors = self._neutral_and_disarm()
        if cleanup_errors:
            self.get_logger().error("; ".join(cleanup_errors))


def main():
    rclpy.init()
    node = ObsNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node._send_pwm and bool(node.get_parameter("arm").value):
            node.interface.disarm()
        node.interface.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
