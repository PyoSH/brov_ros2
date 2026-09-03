#!/usr/bin/env python3
"""brov_base_node — 로봇 I/O와 액추에이터, 그리고 안전.

역할 분리에서 이 노드가 소유하는 것
====================================
**액추에이터 고유 지식 전부**와 **MAVLink 링크 단독 소유권**이다.

  소유:  할당행렬, T200 추력 테이블, 전압, deadband, 추력 한계, PWM slew 한계,
         arm/disarm/passthrough, estop, fault latch, **watchdog**
  비소유: 목표(guidance), 오차/적분(observation), 정책 계약(policy)

절단면을 PWM이 아니라 **wrench**로 둔 이유는 `Wrench6.msg` 주석 참조.
요약하면 액추에이터 고유 지식과 아티팩트 고유 지식의 경계가 물리량이 되어,
`model_based_controller`가 정책을 노드 교체 하나로 대체할 수 있다.

분리가 새로 만든 위험과 그 대응
================================
한 프로세스였을 때 명령 경로와 안전 경로는 원자적이었다. 분리하면
**정책 노드가 죽어도 이 노드는 마지막 명령을 계속 밀 수 있다.** 그래서:

1. `/brov/cmd/wrench`가 ``wrench_command_timeout_s`` 동안 끊기면 **즉시
   neutral_stop**한다. 이것이 분리의 대가를 갚는 유일한 장치다.
2. **MAVLink를 여는 프로세스는 이 노드 하나뿐이어야 한다.** 둘이 열면
   서로의 PWM을 덮어써서 어느 쪽이 이겼는지 사후에 알 수 없다.
3. fault는 latch된다 — 한 번 걸리면 명시적 disarm/재시작 전까지 안 풀린다.

이 노드가 내는 토픽
====================
    /brov/state                        BrovState — 선택된 경로 하나의 상태
    /brov/control_active               Bool
    /brov/odometry/local_with_session  OdometrySession — 마커(pool) 정렬 입력
    /brov/odometry/local               Odometry (진단용)
    /brov/odometry/session_id          String (latched)
    /brov/sensor/ahrs                  Imu — 원시 자세·각속도 (stamp = FC boot 시계)
    /brov/sensor/servo_out             JointState — SERVO_OUTPUT_RAW 8ch PWM µs
                                       (stamp = FC boot 시계)
    /brov/sensor/depth_ekf             Float32 — EKF 수직 위치 [m, 아래가 +]
    /brov/sensor/pressure0..2          FluidPressure — baro instance 0/1/2 원시압

뒤의 다섯은 `/brov/state`가 **고르지 않은 쪽**을 남기기 위한 것이다. depth 게이트
(docs/REAL_ROBOT_SESSION.md 1단계)와 dead time 분석은 선택되지 않은 경로의 값이
같은 bag 에 있어야 성립한다. `publish_odometry`/`publish_sensor_topics`로 끈다.

기존 `obs_node.py`와의 관계
===========================
obs_node는 2531줄로 로봇 I/O·유도·관측·안전을 모두 소유하던 monolith다.
이 노드는 그중 **로봇 I/O와 안전만** 떼어낸 것이며, obs_node를 즉시 대체하지
않는다. 두 경로가 공존하는 동안 **동시에 띄우면 안 된다**(위 2번).
"""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import FluidPressure, Imu, JointState
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger
import torch

from brov_interfaces.msg import BrovState, OdometrySession, Wrench6

from brov_base.mavlink_interface import (
    RealRobotInterface,
    thruster_reversal_sign_for_profile,
)
from brov_base.odometry import ned_frd_to_odom_flu
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix


_MANUAL_CUSTOM_MODE = 19


class BaseNode(Node):
    """MAVLink 링크, 액추에이터, 안전을 단독 소유하는 노드."""

    def __init__(self, interface=None, **node_kwargs) -> None:
        """``interface``를 주입하면 MAVLink 없이 구동한다.

        watchdog·fault latch·할당은 이 노드가 새로 떠안은 안전 책임이고,
        실기 없이 검증할 수 있어야 한다. 기본값 None이면 실제 링크를 연다.
        """
        super().__init__("brov_base", **node_kwargs)

        # ── 링크/액추에이터 ──
        self.declare_parameter("connection", "udpin:0.0.0.0:14550")
        self.declare_parameter("thruster_reversal_profile", "real_brov2")
        self.declare_parameter("battery_voltage", 14.8)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("velocity_source", "mavlink_ekf")
        # 논문 5.2 는 깊이를 Bar30 압력센서로 직접 측정한다 -- EKF 를 거치지 않는다.
        # 기본값은 아직 "mavlink_ekf" 다: GT 교차검증 전에는 조용히 바꾸지 않는다
        # (velocity_source 가 DVL 축 변환 확정 전까지 거부하는 것과 같은 규율).
        self.declare_parameter("depth_source", "mavlink_ekf")
        # G1 (sim2real_findings §6-2): telemetry 주기가 코드 상수(25 Hz)로
        # 굳어 있었다 -- 정책 dt 에서 온 값을 telemetry 에도 그대로 쓴 것.
        # telemetry 는 정책보다 빨라도 되고(관측 노드가 최신을 쓴다), 되먹임
        # 양자화(평균 20 ms, τ 의 23%)를 줄이는 유일한 싼 수단이라 노출한다.
        # 기본값 25.0 = 기존 동작 그대로.
        self.declare_parameter("telemetry_rate_hz", 25.0)
        # G3 실험: 액추에이션 경로 A/B (rc_override | do_set_servo).
        # do_set_servo 는 진단 전용 -- 미션에 쓰지 말 것.
        self.declare_parameter("actuation_backend", "rc_override")
        # 액추에이터 모델. base가 소유하는 지식이고 SITL과 실기가 다르다.
        #
        #   t200_table    실기. T200 제조사 실측 테이블(비선형·비대칭, deadband).
        #   gazebo_linear Edo SITL. ardupilot_gazebo가 servo PWM을 선형으로 매핑한다:
        #                 cmd_thrust = ((pwm-1100)/800 - 0.5) * multiplier
        #                 model.sdf의 multiplier=100 -> **선형 ±50 N**.
        #
        # 틀리면 왕복이 항등이 아니다. T200 역변환으로 만든 PWM을 Gazebo의 선형
        # 법칙에 넣으면 요청의 1.05~2.75배가 나온다(중간 영역 1.4~2.1배). 실제로
        # 기체가 0.5 m/s 명령에 0.92 m/s로 달렸다. 모델을 맞추면 왕복이 정확해진다.
        self.declare_parameter("thruster_model", "t200_table")
        self.declare_parameter("gazebo_linear_half_range_n", 50.0)

        # ── 안전 (obs_node와 같은 이름/의미를 승계한다) ──
        self.declare_parameter("send_pwm", False)
        self.declare_parameter("arm", False)
        self.declare_parameter("max_pwm_abs", 1.0)
        self.declare_parameter("max_pwm_delta_per_s", 0.0)
        self.declare_parameter("pwm_rate_first_command_dt_s", 0.04)
        self.declare_parameter("wrench_command_timeout_s", 0.25)
        self.declare_parameter("att_max_age_s", 0.2)
        self.declare_parameter("pos_max_age_s", 0.5)
        self.declare_parameter("heartbeat_max_age_s", 2.0)
        self.declare_parameter("required_custom_mode", _MANUAL_CUSTOM_MODE)

        # ── 마커(pool) 정렬용 odometry 발행 ──
        # pool_alignment_node 는 `/brov/odometry/local_with_session` 하나만
        # 구독한다. legacy obs_node 만 그것을 냈으므로 분리 스택에서는 마커
        # 정렬을 쓸 수 없었다. 링크를 단독 소유하는 것이 이 노드이므로
        # (동시에 두 프로세스가 MAVLink 를 열 수 없다) 여기서 낸다.
        self.declare_parameter("publish_odometry", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_position_variance", 1.0)
        self.declare_parameter("odom_orientation_variance", 0.25)
        self.declare_parameter("odom_linear_velocity_variance", 0.25)
        self.declare_parameter("odom_angular_velocity_variance", 0.25)
        # EKF 원점/yaw 리셋이나 DVL 재접속은 MAVLink boot time 을 바꾸지 않으므로
        # session id 만으로는 드러나지 않는다. 마커 정렬은 **한 번만** 맞추고
        # 이후 EKF odometry 로 절대 위치를 추측하므로, 이런 도약이 조용히
        # 지나가면 벽까지 남은 거리가 통째로 틀린다. obs_node 와 같은 인접 샘플
        # 검사로 session 을 진행시켜 pool_alignment_node 가 무효화하게 한다.
        self.declare_parameter("odom_jump_translation_m", 0.50)
        self.declare_parameter("odom_jump_rotation_deg", 45.0)
        self.declare_parameter("odom_jump_max_dt_s", 0.50)

        # ── 원시 센서 토픽 ──
        # `/brov/state` 는 **선택된** 경로 하나만 싣는다(depth_source 가 고른 z).
        # 지연·깊이 게이트 분석은 고르지 않은 쪽도 있어야 성립하므로, 원시값을
        # 따로 낸다. dvl 은 별도 노드(brov_control/dvl_record_node)가 낸다.
        self.declare_parameter("publish_sensor_topics", True)

        p = self.get_parameter
        self._send_pwm = bool(p("send_pwm").value)
        self._arm_permitted = bool(p("arm").value)
        self._max_pwm_abs = float(p("max_pwm_abs").value)
        self._max_pwm_delta_per_s = float(p("max_pwm_delta_per_s").value)
        self._first_dt = float(p("pwm_rate_first_command_dt_s").value)
        self._cmd_timeout = float(p("wrench_command_timeout_s").value)
        self._att_max_age = float(p("att_max_age_s").value)
        self._pos_max_age = float(p("pos_max_age_s").value)
        self._hb_max_age = float(p("heartbeat_max_age_s").value)
        self._required_mode = int(p("required_custom_mode").value)
        self._velocity_source = str(p("velocity_source").value)
        self._depth_source = str(p("depth_source").value)
        if self._depth_source not in ("mavlink_ekf", "pressure"):
            raise ValueError(
                f"depth_source={self._depth_source!r} — "
                "'mavlink_ekf' 또는 'pressure' 여야 한다")
        # prepare 에서 FC 로부터 확정한다. 추측하지 않는다.
        self._depth_baro_instance = -1
        self._depth_spec_grav = None
        self._depth_ref_pa = None          # start 시점 기준압

        if self._velocity_source != "mavlink_ekf":
            # layer 1(A50 DVL 직결)을 붙일 지점은 _read_state() 안의 한 곳이다.
            # 2026-08-28 수조 실측에서 EKF가 DVL 대비 12.9% 과소 보고했고
            # heave 축은 부호가 반대였다 — 승격하려면 그 변환을 먼저 확정해야
            # 하므로, 지금은 조용히 EKF로 넘어가지 않고 거부한다.
            raise ValueError(
                f"velocity_source={self._velocity_source!r} 미구현. "
                "현재는 'mavlink_ekf'만 지원한다 — DVL 직결은 축 부호 변환 "
                "확정 후 _read_state()에 추가할 것"
            )

        self._thruster_model = str(p("thruster_model").value)
        if self._thruster_model not in ("t200_table", "gazebo_linear"):
            raise ValueError(
                f"thruster_model={self._thruster_model!r} — "
                "'t200_table'(실기) 또는 'gazebo_linear'(Edo SITL)여야 한다")
        self._gz_half_range = float(p("gazebo_linear_half_range_n").value)
        if self._gz_half_range <= 0.0:
            raise ValueError("gazebo_linear_half_range_n은 양수여야 한다")

        pos, dir_ = thruster_pos_dir_ned(load_brov2_yaml())
        self._thruster = BROV2ThrusterModel(
            num_envs=1, dt=1.0 / float(p("state_rate_hz").value), device="cpu",
            pos=pos, dir=dir_, voltage=float(p("battery_voltage").value),
        )
        self._B_pinv = torch.linalg.pinv(
            build_allocation_matrix(self._thruster._pos, self._thruster._dir)
        )

        conn = str(p("connection").value)
        profile = str(p("thruster_reversal_profile").value)
        backend = str(p("actuation_backend").value)
        if backend not in ("rc_override", "do_set_servo"):
            raise ValueError(f"actuation_backend={backend!r}")
        telem_hz = float(p("telemetry_rate_hz").value)
        if not 1.0 <= telem_hz <= 100.0:
            raise ValueError(f"telemetry_rate_hz={telem_hz} — 1~100 Hz 범위여야 한다")
        if interface is None:
            self._interface = RealRobotInterface(
                conn,
                thruster_reversal_sign=thruster_reversal_sign_for_profile(profile, conn),
                telemetry_rate_hz=telem_hz,
            )
            self._interface.actuation_backend = backend
            if backend != "rc_override":
                self.get_logger().warn(f"실험 액추에이션 백엔드: {backend} — 진단 전용")
            self._interface.connect()
            self.get_logger().info(
                f"MAVLink {conn}, reversal profile {profile}, telemetry {telem_hz:.0f} Hz")
        else:
            self._interface = interface
            self.get_logger().warn("주입된 interface 사용 — 시험 전용")

        # ── 상태 ──
        self._seq = 0
        self._faulted = False
        self._fault_reason = ""
        self._estopped = False
        self._armed_by_us = False
        self._prepared = False
        # arm(추진기 통전)과 start(제어 루프 개시)는 별개다. 하나로 합치면
        # waypoint_frame=start_heading 의 원점이 "추진기가 켜진 직후" 로 잡히고,
        # 제어만 멈추려 해도 disarm 밖에 길이 없어 passthrough 까지 원복된다.
        self._started = False
        self._last_cmd_monotonic: float | None = None
        self._last_pwm = torch.zeros(8, dtype=torch.float32)
        self._last_pwm_monotonic: float | None = None
        self._first_command = True
        self._stopped_by_watchdog = False

        # ── odometry 발행 상태 ──
        self._publish_odometry = bool(p("publish_odometry").value)
        self._odom_frame = str(p("odom_frame").value).strip()
        self._base_frame = str(p("base_frame").value).strip()
        if not self._odom_frame or not self._base_frame:
            raise ValueError("odom_frame 과 base_frame 은 비어 있을 수 없다")
        if self._odom_frame == self._base_frame:
            raise ValueError("odom_frame 과 base_frame 은 서로 달라야 한다")
        self._odom_covariance = {
            key: self._positive("odom_" + key + "_variance")
            for key in (
                "position", "orientation", "linear_velocity", "angular_velocity"
            )
        }
        self._odom_jump_translation_m = self._positive("odom_jump_translation_m")
        self._odom_jump_rotation_rad = math.radians(
            self._positive("odom_jump_rotation_deg")
        )
        self._odom_jump_max_dt_s = self._positive("odom_jump_max_dt_s")
        self._raw_odometry_session_id = ""
        self._navigation_jump_count = 0
        self._last_odom_position = None
        self._last_odom_orientation = None
        self._last_odom_sample_time = None
        self._last_published_session_id = ""
        self._publish_sensor_topics = bool(p("publish_sensor_topics").value)
        self._last_press_seq = [-1, -1, -1]
        self._last_att_seq = None
        self._last_pos_seq = None
        self._last_odometry_key = None
        self._last_snapshot = None

        # ── 인터페이스 ──
        self._pub_state = self.create_publisher(BrovState, "/brov/state", 10)
        # 제어 루프가 실제로 닫혀 있는지. observation_node가 이걸 보고 적분한다 --
        # 루프가 열린 채(미무장/send_pwm=false) 적분하면 기체가 안 움직이는 동안
        # v_e = -v_d 가 계속 쌓여 arm 하는 순간 적분이 이미 포화 상태다.
        # 실제 SITL에서 z_v가 clamp 5.0에 붙는 것으로 드러났다.
        self._pub_active = self.create_publisher(Bool, "/brov/control_active", 10)
        self.create_subscription(Wrench6, "/brov/cmd/wrench", self._on_wrench, 1)
        self.create_subscription(Empty, "/brov/estop", self._on_estop, 10)
        # 이름은 legacy 스택(obs_node + policy_node_mk2, demo_orchestrator)과
        # **같게** 둔다. 분리 스택이 legacy orchestrator 로 그대로 구동된다.
        for name, cb in (("prepare_control", self._srv_prepare),
                         ("arm_control", self._srv_arm),
                         ("start_control", self._srv_start),
                         ("stop_control", self._srv_stop),
                         ("disarm_control", self._srv_disarm),
                         ("estop_control", self._srv_estop)):
            self.create_service(Trigger, f"/brov/{name}", cb)
        # 스트림 재요청. "telemetry 없음" 이 heartbeat 만 오는 상태라면 재실행 대신
        # 이것으로 푼다. 응답에 지금 수신 중인 메시지 타입을 실어 준다.
        self.create_service(Trigger, "/brov/request_streams", self._srv_request_streams)
        self._last_rx_counts: dict[str, int] = {}

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_odom = None
        self._pub_odom_session = None
        self._pub_odom_with_session = None
        if self._publish_odometry:
            self._pub_odom = self.create_publisher(
                Odometry, "/brov/odometry/local", 10)
            self._pub_odom_session = self.create_publisher(
                String, "/brov/odometry/session_id", latched)
            self._pub_odom_with_session = self.create_publisher(
                OdometrySession, "/brov/odometry/local_with_session", 10)
        self._pub_ahrs = None
        self._pub_servo_out = None
        self._pub_depth_ekf = None
        self._pub_pressure = []
        if self._publish_sensor_topics:
            self._pub_ahrs = self.create_publisher(Imu, "/brov/sensor/ahrs", 10)
            # EKF 수직 위치. depth_source 가 pressure 여도 계속 낸다 --
            # 2026-08-29 SITL 에서 이 값이 초기값에 얼어붙었고, 얼어붙었다는
            # 사실 자체가 bag 에 남아 있어야 사후에 판정할 수 있다.
            self._pub_depth_ekf = self.create_publisher(
                Float32, "/brov/sensor/depth_ekf", 10)
            # SCALED_PRESSURE/2/3 = baro instance 0/1/2 를 변환 없이 낸다.
            # 어느 instance 가 물속 센서인지는 probe 순서에 달렸으므로
            # (docs/REAL_ROBOT_SESSION.md 1단계) 추론하지 않고 셋 다 남긴다.
            self._pub_pressure = [
                self.create_publisher(
                    FluidPressure, f"/brov/sensor/pressure{i}", 10)
                for i in range(3)
            ]
            # dead time 분해(M4, LATENCY_DECOMPOSITION_PLAN.md)용. ahrs 와 함께
            # **FC boot 시계 stamp** 로 나가므로, 두 토픽의 header 끼리 교차상관
            # 하면 링크 지연이 전혀 안 낀 servo→gyro 순수 액추에이터 지연이 나온다.
            # 도착 시각은 bag 기록 시각에 따로 남는다.
            self._pub_servo_out = self.create_publisher(
                JointState, "/brov/sensor/servo_out", 10)
            self._last_servo_seq = None

        period = 1.0 / float(p("state_rate_hz").value)
        self.create_timer(period, self._tick)
        if self._publish_sensor_topics:
            # 원시 센서 토픽은 제어 틱과 분리해 100 Hz 로 살핀다 (G2 최소 수정,
            # 2026-09-03). 제어 틱(state_rate 25 Hz)에서 발행하면 telemetry 가
            # 그보다 빨리 와도 틱 사이 표본이 버려져 -- 토픽·bag 이 도착률을
            # 은폐하고(50 Hz 요청이 25 로 보임) M3/M4 분해능까지 깎는다.
            # seq 가드가 있어 새 표본이 없으면 아무것도 안 낸다.
            self.create_timer(0.01, self._sensor_tick)
        limits = (f"선형 ±{self._gz_half_range:.1f} N"
                  if self._thruster_model == "gazebo_linear"
                  else (
                      # force_limits_n 은 **전 전압 테이블 포락선**(보상 정규화
                      # 전용, -49.4/+65.9 N)이라 실제 클램프가 아니다. 2026-09-02
                      # 수조 세션에서 이 로그가 실제 클램프(-36.7/+47.2 N @14.8 V)
                      # 와 다른 값을 찍어 "추진기 한계 불일치" 로 기록됐다 --
                      # 불일치가 아니라 서로 다른 두 값이었다. 현장에서 보는
                      # 로그에는 실제 클램프를 찍는다.
                      f"T200 테이블, 클램프 {self._thruster.clamp_thrust(torch.full((1, 8), -1e9))[0, 0]:.1f}"
                      f"/{self._thruster.clamp_thrust(torch.full((1, 8), 1e9))[0, 0]:+.1f} N"
                      f" @{float(self._thruster.voltage[0]):.1f} V"
                  ))
        self.get_logger().info(
            f"base_node 시작 — send_pwm={self._send_pwm} arm={self._arm_permitted} "
            f"watchdog={self._cmd_timeout:.3f}s, 추진기 {self._thruster_model} ({limits})"
        )

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 은 유한한 양수여야 한다 (받은 값 {value})")
        return value

    # ------------------------------------------------------------------ 상태
    def _read_state(self) -> BrovState | None:
        snap = self._interface.snapshot()
        self._last_snapshot = snap
        msg = BrovState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = self._seq
        self._seq += 1
        msg.velocity_source = self._velocity_source
        if snap is None:
            msg.valid = False
            msg.reason = "telemetry 없음"
            return msg

        q = snap["att_quat_ned"].reshape(4)
        pos = snap["pos_ned"].reshape(3)
        vel_ned = snap["vel_ned"].reshape(3)
        omega = snap["body_rates_ned"].reshape(3)

        # 속도를 body frame으로. observation 단계가 Z-up/FLU 변환을 따로 한다.
        from brov_base import math_utils as mu
        v_body = mu.quat_apply(
            mu.quat_conjugate(q.unsqueeze(0)), vel_ned.unsqueeze(0)
        ).reshape(3)

        # ── 깊이 출처 ──
        # 논문 5.2 는 Bar30 압력센서로 z 를 직접 측정한다("We measure the depth
        # (z, positive down)"). EKF 를 거치지 않는다. ArduSub 자신의 변환식을
        # 그대로 쓴다 (AP_Baro.cpp:888):
        #     altitude = (ground_pressure - pressure) / 9800 / SPEC_GRAV   [Pa]
        # depth = -altitude 이고 NED z 는 아래가 양이므로 부호가 그대로 맞는다.
        # ground_pressure 자리에 start 시점 기준압을 쓰므로 결과는 **상대 깊이**다.
        z = float(pos[2])
        depth_src, depth_idx = "mavlink_ekf", -1
        if self._depth_source == "pressure" and self._depth_ref_pa is not None:
            hpa = snap["press_abs_hpa"][self._depth_baro_instance]
            age = snap["press_age_s"][self._depth_baro_instance]
            if hpa is None or age > self._pos_max_age:
                msg.valid = False
                msg.reason = (f"depth 압력 stale: instance "
                              f"{self._depth_baro_instance} age {age:.2f}s")
                return msg
            z = (hpa * 100.0 - self._depth_ref_pa) / (9800.0 * self._depth_spec_grav)
            depth_src, depth_idx = "pressure", self._depth_baro_instance

        msg.attitude = Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))
        msg.position = Vector3(x=float(pos[0]), y=float(pos[1]), z=z)
        msg.depth_source = depth_src
        msg.depth_baro_instance = depth_idx
        msg.linear_velocity = Vector3(x=float(v_body[0]), y=float(v_body[1]), z=float(v_body[2]))
        msg.angular_velocity = Vector3(x=float(omega[0]), y=float(omega[1]), z=float(omega[2]))
        msg.attitude_age_s = float(snap["att_age_s"])
        msg.position_age_s = float(snap["pos_age_s"])
        # DVL 기록기를 붙였을 때 EKF 융합이 끊기지 않았는지 보는 유일한 창이다.
        # A50 의 TCP 서버가 단일 클라이언트만 받으면 기록기가 BlueOS 의 DVL
        # extension 을 밀어내고, EKF 는 IMU dead reckoning 으로 조용히 떨어진다.
        # 그 경우 이 값이 눈에 띄게 오른다. 미수신이면 -1.
        _var = snap.get("ekf_vel_variance")
        msg.ekf_velocity_variance = float(_var) if _var is not None else -1.0

        stale = []
        if snap["att_age_s"] > self._att_max_age:
            stale.append(f"att {snap['att_age_s']:.2f}s")
        if snap["pos_age_s"] > self._pos_max_age:
            stale.append(f"pos {snap['pos_age_s']:.2f}s")
        msg.valid = not stale
        msg.reason = "stale: " + ", ".join(stale) if stale else ""
        return msg

    # -------------------------------------------------------------- odometry
    @staticmethod
    def _diagonal_covariance(translation: float, rotation: float) -> list[float]:
        covariance = [0.0] * 36
        for index in (0, 7, 14):
            covariance[index] = translation
        for index in (21, 28, 35):
            covariance[index] = rotation
        return covariance

    def _odometry_session_id(self, snap: dict) -> str:
        raw = str(snap["odometry_session_id"]).strip()
        return f"{raw}:nav{self._navigation_jump_count}"

    def _detect_odometry_jump(self, converted, snap: dict) -> str | None:
        """인접 샘플이 물리적으로 불가능하게 뛰면 session 을 진행시킨다.

        MAVLink boot time 은 EKF 원점/yaw 리셋이나 DVL 재접속을 드러내지 못한다.
        한 번만 맞추는 마커 정렬은 그 이후 EKF odometry 로 절대 위치를 추측하므로,
        이런 도약이 조용히 지나가면 벽까지의 거리가 통째로 틀린다.
        """
        raw = str(snap.get("odometry_session_id", "")).strip()
        sample_time = max(float(snap["att_rx_time"]), float(snap["pos_rx_time"]))
        position = converted.position_odom.detach().clone()
        orientation = converted.orientation_xyzw.detach().clone()

        if raw != self._raw_odometry_session_id:
            self._raw_odometry_session_id = raw
            self._navigation_jump_count = 0
            self._last_odom_position = position
            self._last_odom_orientation = orientation
            self._last_odom_sample_time = sample_time
            return None

        reason = None
        if (self._last_odom_position is not None
                and self._last_odom_orientation is not None
                and self._last_odom_sample_time is not None):
            dt = sample_time - self._last_odom_sample_time
            if 0.0 <= dt <= self._odom_jump_max_dt_s:
                translation = float(
                    torch.linalg.vector_norm(position - self._last_odom_position))
                dot = float(torch.dot(orientation, self._last_odom_orientation))
                angle = 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))
                if translation > self._odom_jump_translation_m:
                    reason = f"위치 도약 {translation:.3f} m / {dt:.3f} s"
                elif angle > self._odom_jump_rotation_rad:
                    reason = f"자세 도약 {math.degrees(angle):.1f}° / {dt:.3f} s"
        self._last_odom_position = position
        self._last_odom_orientation = orientation
        self._last_odom_sample_time = sample_time
        if reason is not None:
            self._navigation_jump_count += 1
        return reason

    def _publish_odometry_sample(self, snap: dict) -> None:
        """pool_alignment_node 가 구독하는 원자 envelope 을 낸다.

        fault 를 latch 하지 않는다 -- 도약은 session id 를 바꾸고, 마커 정렬은
        그 변화만으로 스스로 무효가 된다(pool 모드 guidance 는 그때 침묵하고
        base watchdog 이 중립 정지시킨다). start_heading 주행은 절대 프레임에
        의존하지 않으므로 계속 돈다.
        """
        if not str(snap.get("odometry_session_id", "")).strip():
            # session id 없이 낸 odometry 는 pool_alignment_node 가 어차피
            # 거절한다. 조용히 거절당하느니 내지 않는다.
            return
        converted = ned_frd_to_odom_flu(
            snap["pos_ned"], snap["att_quat_ned"],
            snap["vel_ned"], snap["body_rates_ned"],
        )
        jump = self._detect_odometry_jump(converted, snap)
        session_id = self._odometry_session_id(snap)
        if jump is not None:
            self.get_logger().error(
                f"odometry {jump} — session 진행: {session_id}. "
                "마커 정렬이 있었다면 무효가 된다")
        if session_id != self._last_published_session_id:
            self._last_published_session_id = session_id
            self._pub_odom_session.publish(String(data=session_id))
            self.get_logger().info(f"odometry session: {session_id}")

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
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
            self._odom_covariance["orientation"])
        message.twist.covariance = self._diagonal_covariance(
            self._odom_covariance["linear_velocity"],
            self._odom_covariance["angular_velocity"])

        envelope = OdometrySession()
        envelope.odometry = message
        envelope.odometry_session_id = session_id
        self._pub_odom_with_session.publish(envelope)
        self._pub_odom.publish(message)

    # ---------------------------------------------------------------- 센서
    def _sensor_tick(self) -> None:
        snap = self._interface.snapshot()
        if snap is None:
            return
        try:
            self._publish_sensor_sample(snap)
        except (ValueError, RuntimeError, KeyError) as exc:
            self.get_logger().error(f"센서 토픽 발행 실패: {exc}")

    def _publish_sensor_sample(self, snap: dict) -> None:
        """실제로 새로 온 샘플만 낸다.

        telemetry 는 `state_rate_hz` 보다 느리다. 같은 값을 tick 마다 복제하면
        bag 이 거짓 갱신으로 차고 `ros2 topic hz` 가 링크의 실제 주기를 감춘다 --
        원시 토픽의 목적이 정확히 그것을 보는 것이다.
        """
        stamp = self.get_clock().now().to_msg()
        att_seq = snap.get("att_seq")
        if att_seq != self._last_att_seq:
            self._last_att_seq = att_seq
            q = snap["att_quat_ned"].reshape(4)
            omega = snap["body_rates_ned"].reshape(3)
            imu = Imu()
            # stamp = **FC boot 시계** (ATTITUDE.time_boot_ms). servo_out 과 같은
            # 시계라 둘의 header 교차상관 = 링크 무관 M4 측정이 된다. 도착
            # 시각은 bag 이 따로 기록한다. FC 재부팅 시 되감길 수 있다.
            _tb = snap.get("att_time_boot_ms")
            if _tb:
                imu.header.stamp.sec = int(_tb // 1000)
                imu.header.stamp.nanosec = int((_tb % 1000) * 1_000_000)
            else:
                imu.header.stamp = stamp
            # NED world -> FRD body. `/brov/state.attitude` 와 **같은 규약**이다 --
            # 여기서 몰래 변환하면 두 토픽을 나란히 놓고 볼 수 없다.
            imu.header.frame_id = self._base_frame
            imu.orientation = Quaternion(
                w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))
            imu.angular_velocity = Vector3(
                x=float(omega[0]), y=float(omega[1]), z=float(omega[2]))
            # 선가속도는 이 링크로 오지 않는다. -1 이 REP-145 의 "미제공" 표시다.
            imu.linear_acceleration_covariance[0] = -1.0
            self._pub_ahrs.publish(imu)

        pos_seq = snap.get("pos_seq")
        if pos_seq != self._last_pos_seq:
            self._last_pos_seq = pos_seq
            self._pub_depth_ekf.publish(
                Float32(data=float(snap["pos_ned"].reshape(3)[2])))

        seqs = snap.get("press_seq") or [0, 0, 0]
        for i, hpa in enumerate(snap.get("press_abs_hpa") or [None, None, None]):
            if hpa is None or int(seqs[i]) == self._last_press_seq[i]:
                continue
            self._last_press_seq[i] = int(seqs[i])
            pressure = FluidPressure()
            pressure.header.stamp = stamp
            pressure.header.frame_id = f"baro{i}"
            pressure.fluid_pressure = float(hpa) * 100.0     # hPa -> Pa
            pressure.variance = 0.0
            self._pub_pressure[i].publish(pressure)

        ctrl = self._interface.control_snapshot()
        servo_seq = None if ctrl is None else ctrl.get("servo_seq")
        if (self._pub_servo_out is not None and servo_seq
                and servo_seq != self._last_servo_seq
                and ctrl.get("servo_output_us") is not None):
            self._last_servo_seq = servo_seq
            js = JointState()
            tu = int(ctrl.get("servo_time_usec") or 0)
            js.header.stamp.sec = tu // 1_000_000
            js.header.stamp.nanosec = (tu % 1_000_000) * 1000
            js.header.frame_id = self._base_frame
            js.name = [f"servo{k}" for k in range(1, 9)]
            js.position = [float(v) for v in ctrl["servo_output_us"].reshape(-1)]
            self._pub_servo_out.publish(js)

    def _rx_summary(self) -> str:
        """지난 호출 이후 늘어난 MAVLink 메시지 타입별 개수."""
        get = getattr(self._interface, "rx_counts", None)
        if get is None:
            return "(interface 가 수신 통계를 내지 않는다)"
        now = get()
        delta = {k: now[k] - self._last_rx_counts.get(k, 0) for k in now}
        self._last_rx_counts = now
        seen = ", ".join(f"{k}×{v}" for k, v in sorted(delta.items()) if v > 0)
        missing = [k for k in ("ATTITUDE_QUATERNION", "LOCAL_POSITION_NED")
                   if delta.get(k, 0) == 0]
        # SET_MESSAGE_INTERVAL(511) 의 ACK 가 왔는지가 결정적이다: ACK 없음 = 요청이
        # FC 에 닿지 않음(라우터), ACK 거절 = FC 가 거부, ACK 수락인데 안 옴 = 라우터가
        # 우리 쪽으로 안 보냄.
        ack_fn = getattr(self._interface, "last_command_ack", None)
        ack = ack_fn() if ack_fn else None
        ack_text = ("ACK 없음" if ack is None
                    else f"ACK cmd={ack[0]} result={ack[1]}({'수락' if ack[1] == 0 else '거절'})")
        return (f"수신: {seen or '없음'}"
                + (f" | 안 옴: {', '.join(missing)}" if missing else "")
                + f" | {ack_text}")

    def _srv_request_streams(self, _req, res):
        fn = getattr(self._interface, "request_telemetry_streams", None)
        if fn is None:
            res.success, res.message = False, "interface 가 재요청을 지원하지 않는다"
            return res
        try:
            fn()
        except Exception as exc:
            res.success, res.message = False, f"재요청 실패: {exc}"
            return res
        res.success, res.message = True, f"스트림 재요청 보냄. {self._rx_summary()}"
        return res

    def _tick(self) -> None:
        msg = self._read_state()
        if msg is not None:
            self._pub_state.publish(msg)
            if not msg.valid and msg.reason == "telemetry 없음":
                # 무엇이 오고 무엇이 안 오는지를 5 s 마다 찍는다. heartbeat 만 오면
                # /brov/request_streams, 아무것도 안 오면 링크/라우터 문제다.
                self.get_logger().warn(
                    f"telemetry 없음 — {self._rx_summary()}. "
                    "heartbeat 만 온다면: ros2 service call /brov/request_streams "
                    "std_srvs/srv/Trigger",
                    throttle_duration_sec=5.0)
        snap = self._last_snapshot
        if snap is not None:
            # telemetry 는 state_rate_hz 보다 느리게 온다. 같은 샘플을 다시 내면
            # pool_alignment_node 가 그것을 **독립 표본으로 세어** 정지 상태
            # 20 표본 조건을 가짜로 채운다 -- 정렬이 실제보다 정확해 보인다.
            key = (snap.get("att_seq"), snap.get("pos_seq"))
            if key != self._last_odometry_key:
                self._last_odometry_key = key
                if self._publish_odometry:
                    # 진단·정렬 경로다. 여기서 던진 예외가 timer 콜백을 타고
                    # 나가면 **제어 경로까지 같이 죽는다** -- watchdog 도 이
                    # 타이머 안에 있다. 기록하고 넘어간다.
                    try:
                        self._publish_odometry_sample(snap)
                    except (ValueError, RuntimeError, KeyError) as exc:
                        self.get_logger().error(f"odometry 발행 실패: {exc}")
        active = bool(
            self._send_pwm and self._armed_by_us and self._started
            and not self._faulted and not self._estopped
        )
        self._pub_active.publish(Bool(data=active))

        # ── watchdog: 분리가 만든 위험을 갚는 장치 ──
        # 정책 노드가 죽거나 멈추면 여기서만 잡을 수 있다.
        if self._last_cmd_monotonic is not None and not self._stopped_by_watchdog:
            idle = time.monotonic() - self._last_cmd_monotonic
            if idle > self._cmd_timeout:
                self._neutral_stop(f"wrench watchdog {idle:.3f}s > {self._cmd_timeout:.3f}s")
                self._stopped_by_watchdog = True

    # ------------------------------------------------------------- 액추에이터
    def _on_wrench(self, msg: Wrench6) -> None:
        # arm 검사가 여기 있어야 하는 이유 — 2026-08-28 Gazebo SITL이 드러냈다.
        # 이 검사가 없던 동안 base는 arm 전에도 RC override를 내보냈고, ArduSub가
        # `armed=False`를 보고하는 상태에서도 servo 출력이 중립을 벗어나 기체가
        # 실제로 구동됐다(40 m 미션이 8 m 침강으로 끝났다). 더 나쁜 것은 조합이다:
        # observation_node는 적분을 `/brov/control_active`로 gate하는데 그 신호는
        # arm을 반영하므로, 구동은 살아 있는 채 정책은 z_v/z_q가 영구히 0인
        # 관측으로 돌았다. 게이트 조건을 _tick()의 control_active와 **같게** 둔다 —
        # 발행하는 신호와 실제 구동 여부가 어긋나면 안 된다.
        if (
            self._estopped
            or self._faulted
            or not self._send_pwm
            or not self._armed_by_us
            or not self._started
        ):
            return
        w = torch.tensor([msg.force.x, msg.force.y, msg.force.z,
                          msg.torque.x, msg.torque.y, msg.torque.z], dtype=torch.float32)
        if not torch.isfinite(w).all():
            self._trip("wrench에 NaN/Inf")
            return

        desired = self._B_pinv @ w
        if self._thruster_model == "gazebo_linear":
            # ardupilot_gazebo의 선형 법칙을 그대로 역으로 쓴다 — 왕복이 항등이다.
            desired = desired.clamp(-self._gz_half_range, self._gz_half_range)
            pwm = (desired / self._gz_half_range).reshape(-1).clamp(-1.0, 1.0)
        else:
            desired = self._thruster.clamp_thrust(desired)
            pwm = self._thruster.inverse_thrust(desired).reshape(-1).clamp(-1.0, 1.0)

        if bool((pwm.abs() > self._max_pwm_abs).any()):
            self._trip(f"PWM이 max_pwm_abs {self._max_pwm_abs} 초과")
            return

        now = time.monotonic()
        if self._max_pwm_delta_per_s > 0.0:
            prev = self._last_pwm_monotonic if self._last_pwm_monotonic is not None else now
            dt = max(0.0, now - prev)
            if self._first_command:
                dt = max(dt, self._first_dt)
            limit = self._max_pwm_delta_per_s * dt + 1e-6
            delta = float((pwm - self._last_pwm).abs().max())
            if delta > limit:
                self._trip(f"PWM slew {delta:.4f} > {limit:.4f} (dt={dt:.4f}s)")
                return

        self._interface.send_pwm(pwm)
        self._last_pwm = pwm.clone()
        self._last_pwm_monotonic = now
        self._last_cmd_monotonic = now
        self._first_command = False
        self._stopped_by_watchdog = False

    def _neutral_stop(self, reason: str) -> None:
        try:
            self._interface.neutral_stop()
        except Exception as exc:                     # 정지는 절대 예외로 끝나면 안 된다
            self.get_logger().error(f"neutral_stop 실패: {exc}")
        self._last_pwm.zero_()
        self.get_logger().warn(f"중립 정지 — {reason}")

    def _trip(self, reason: str) -> None:
        """fault latch. 한 번 걸리면 명시적 disarm/재시작 전까지 안 풀린다."""
        if self._faulted:
            return
        self._faulted = True
        self._fault_reason = reason
        self._neutral_stop(f"fault: {reason}")
        self.get_logger().error(f"FAULT — {reason}")

    def _on_estop(self, _msg: Empty) -> None:
        self._estopped = True
        self._started = False
        self._prepared = False
        self._neutral_stop("estop")

    # --------------------------------------------------------------- 서비스
    def _srv_prepare(self, _req, res):
        ctrl = self._interface.control_snapshot()
        if ctrl["heartbeat_age_s"] > self._hb_max_age:
            res.success, res.message = False, f"heartbeat {ctrl['heartbeat_age_s']:.1f}s"
            return res
        if ctrl["custom_mode"] != self._required_mode:
            res.success, res.message = False, (
                f"custom_mode {ctrl['custom_mode']} != {self._required_mode}(MANUAL)")
            return res
        state = self._read_state()
        if state is None or not state.valid:
            res.success, res.message = False, (
                f"telemetry가 유효하지 않다: {state.reason if state else 'None'}")
            return res
        try:
            self._interface.enable_passthrough()
        except Exception as exc:
            res.success, res.message = False, f"passthrough 실패: {exc}"
            return res
        if self._depth_source == "pressure":
            # 어느 SCALED_PRESSURE 인지 **추측하지 않는다.** ArduSub 는 init 에서
            # BARO_TYPE_WATER 인 첫 instance 를 primary 로 set_and_save 하므로
            # (ArduSub/system.cpp:108, AP_Baro.h:181) BARO_PRIMARY 가 곧 depth
            # sensor 인덱스다. SITL 은 모든 baro 가 WATER 라 응답만으로는 구분되지
            # 않으므로(AP_Baro_SITL.cpp:21) 이 파라미터가 유일하게 확실한 근거다.
            idx = self._interface.get_parameter("BARO_PRIMARY")
            grav = self._interface.get_parameter("BARO_SPEC_GRAV")
            if idx is None or grav is None:
                res.success, res.message = False, (
                    "BARO_PRIMARY/BARO_SPEC_GRAV 조회 실패 — depth_source=pressure "
                    "로는 진행할 수 없다")
                return res
            idx = int(round(idx))
            if not 0 <= idx <= 2:
                res.success, res.message = False, (
                    f"BARO_PRIMARY={idx} — SCALED_PRESSURE/2/3 (0..2) 범위 밖")
                return res
            snap = self._interface.snapshot()
            if snap is None or snap["press_abs_hpa"][idx] is None:
                res.success, res.message = False, (
                    f"SCALED_PRESSURE instance {idx} 미수신")
                return res
            self._depth_baro_instance = idx
            self._depth_spec_grav = float(grav)
            self.get_logger().info(
                f"depth sensor 확정 — baro instance {idx} "
                f"(SCALED_PRESSURE{'' if idx == 0 else idx + 1}), "
                f"SPEC_GRAV {self._depth_spec_grav:.3f}")
        self._prepared = True
        res.success, res.message = True, "passthrough 활성"
        return res

    def _srv_arm(self, _req, res):
        if not self._arm_permitted:
            res.success, res.message = False, "arm 파라미터가 false다"
            return res
        if self._faulted:
            res.success, res.message = False, f"fault 상태: {self._fault_reason}"
            return res
        # prepare 없이 arm 하면 안 되는 이유 — 2026-08-28 Gazebo SITL 이 드러냈다.
        # prepare 가 하는 일은 SERVO1~8 을 RCPassThru 로 바꾸는 것이다. 그게 없으면
        # ArduSub 는 우리가 보낸 추진기 PWM 을 **조종사 입력**(roll/pitch/throttle/
        # yaw/forward/lateral)으로 해석해 자체 믹싱을 돌린다. 즉 할당행렬이 통째로
        # 다른 것으로 바뀐다. 실제로 prepare 가 telemetry 미도달로 실패했는데 arm 이
        # 성공해버려, 40 m 미션이 두 번 연속 해저 침강으로 끝났다.
        if not self._prepared:
            res.success, res.message = False, (
                "prepare 가 선행되어야 한다 (SERVO1~8 RCPassThru 미설정)")
            return res
        try:
            self._interface.arm()
        except Exception as exc:
            res.success, res.message = False, f"arm 실패: {exc}"
            return res
        self._armed_by_us = True
        self._last_cmd_monotonic = None      # watchdog은 첫 명령부터 센다
        self._stopped_by_watchdog = False
        res.success, res.message = True, "armed"
        return res

    def _srv_start(self, _req, res):
        """제어 루프 개시 — 적분과 PWM 을 허용한다.

        legacy `/brov/start_control` 과 같은 의미다. arm 이 하드웨어를 통전한
        상태에서, 여기서부터 control_active 가 true 가 되고 관측 노드가 적분을
        시작하며 guidance 가 mission frame 원점을 잡는다.
        """
        if self._estopped:
            res.success, res.message = False, "estop — 재시작 전까지 시작할 수 없다"
            return res
        if self._faulted:
            res.success, res.message = False, f"fault 상태: {self._fault_reason}"
            return res
        if not self._prepared:
            res.success, res.message = False, "prepare 가 선행되어야 한다"
            return res
        if not self._armed_by_us:
            res.success, res.message = False, "arm 이 선행되어야 한다"
            return res
        state = self._read_state()
        if state is None or not state.valid:
            res.success, res.message = False, (
                f"telemetry 가 유효하지 않다: {state.reason if state else 'None'}")
            return res
        if self._depth_source == "pressure":
            # 기준압은 **start 순간**에 잡는다 -- guidance 가 mission frame 원점을
            # 잡는 바로 그 순간이다. 상대 깊이가 되므로 센서 상수 편의가 정확히
            # 상쇄되고, guidance 는 어차피 원점을 빼므로 필요한 값과 일치한다.
            snap = self._interface.snapshot()
            hpa = None if snap is None else snap["press_abs_hpa"][self._depth_baro_instance]
            if hpa is None:
                res.success, res.message = False, "기준압 캡처 실패 — 압력 미수신"
                return res
            self._depth_ref_pa = float(hpa) * 100.0
            self.get_logger().info(
                f"깊이 기준압 캡처 — {hpa:.2f} hPa (instance {self._depth_baro_instance})")
        self._started = True
        self._last_cmd_monotonic = None      # watchdog 은 첫 명령부터 센다
        self._stopped_by_watchdog = False
        res.success, res.message = True, "started"
        return res

    def _srv_stop(self, _req, res):
        """제어 동결 — armed 는 유지한 채 적분과 PWM 만 멈춘다."""
        self._started = False
        self._depth_ref_pa = None
        # stop 뒤에는 wrench 를 무시하므로 마지막 명령 시각이 굳는다. 지우지 않으면
        # 0.25 s 뒤 watchdog 이 "중립 정지" 를 한 번 더 찍는다 -- 이미 멈춘 뒤라
        # 무해하지만, 로그를 읽는 사람은 고장으로 오독한다 (2026-09-02 실기).
        self._last_cmd_monotonic = None
        self._stopped_by_watchdog = False
        self._neutral_stop("stop 요청")
        res.success, res.message = True, "stopped (armed 유지)"
        return res

    def _srv_disarm(self, _req, res):
        self._neutral_stop("disarm 요청")
        try:
            self._interface.disarm()
        except Exception as exc:
            res.success, res.message = False, f"disarm 실패: {exc}"
            return res
        self._armed_by_us = False
        self._started = False
        self._depth_ref_pa = None
        self._prepared = False              # passthrough 는 disarm 에서 원복된다
        self._faulted = False               # disarm이 fault latch를 푸는 유일한 경로
        self._fault_reason = ""
        res.success, res.message = True, "disarmed (fault latch 해제)"
        return res

    def _srv_estop(self, _req, res):
        self._on_estop(Empty())
        res.success, res.message = True, "estop — 재시작 전까지 명령을 받지 않는다"
        return res

    def destroy_node(self) -> None:
        try:
            self._neutral_stop("노드 종료")
            if self._armed_by_us:
                self._interface.disarm()
            self._interface.close()
        finally:
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
