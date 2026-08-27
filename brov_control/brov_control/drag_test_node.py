#!/usr/bin/env python3
"""실기 surge 항력 측정 노드 — pool 정렬로 직진을 유지하며 종단속도를 잰다.

무엇을 왜 재는가
================
Sim2Swim 보상 Eq.(8)은 정상상태에서 0이 되지 않는 행동 비용을 만들고, 속도항
Eq.(6)은 오차 0 근방에서 기울기가 0이다. 균형점을 정하는 값은 하나다:

    A = drag(0.5 m/s) / 최대 surge 추력

    가설 A (sim 계수 Xu=13.7, Xuu=141):  v_max 0.88 m/s,  A 0.340,  추종률 28%
    가설 B (제조사 사양 1.5 m/s):        v_max 1.48 m/s,  A 0.149,  추종률 53%

surge는 반드시 open-loop
========================
**속도로 피드백하면 측정이 무의미해진다.** 이 시험은 "정해진 추력에서 나오는
종단속도"를 재는 것이다. ``model_based_controller``가 surge를 속도 PI로 닫기
때문에 그것을 그대로 쓸 수 없고, 여기서 surge만 open-loop로 두고 나머지 5축을
닫는다. 할당 경로는 그 컨트롤러의 ``allocate()``를 그대로 공유한다.

pool 정렬이 주는 것
===================
ArUco 정렬은 one-shot이라 **주행 중 방위 드리프트를 보정하지 못한다** — 마커는
초기화 시점의 상수 yaw 기준만 주고, 시변 방위는 ArduSub EKF3/AHRS에서 온다.
1.8 m/약 3초 주행에서 AHRS 드리프트는 무시할 수준이므로 문제가 되지 않는다.

정렬이 실제로 주는 것은 **절대 pool 위치**다. 벽까지 남은 거리, 차선 유지,
주행거리를 절대좌표로 알 수 있다. 3~4 m 수조에서 이는 안전과 직결되고, 시작점
기준 상대거리보다 훨씬 낫다. 덤으로 pool 위치 미분이 EKF 속도의 독립 교차검증이
된다.

액추에이션
==========
``obs_node``가 유일한 MAVLink 소유자다. 이 노드는 MAVLink를 열지 않고
``/brov/thruster_pwm``만 25 Hz로 낸다. ``_authority_gate``가 그 토픽의 발행자가
정확히 하나일 것을 요구하므로 ``model_based_controller_node``/``policy_node``와
**동시 실행할 수 없다**.

운용
====
    ros2 launch brov_bringup drag_test.launch.py send_pwm:=true arm:=true
    ros2 service call /brov/drag_test/prepare std_srvs/srv/Trigger    # 정지 상태로
    ros2 service call /brov/drag_test/start   std_srvs/srv/Trigger
    # ARMED 유지 구간에 테더를 놓는다. 깊이와 방위가 잡히는지 볼 것.
    ros2 service call /brov/drag_test/stop    std_srvs/srv/Trigger    # 언제든
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque

import rclpy
import torch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from brov_base.diag_terminal_velocity import (
    _Allocator,
    fit_drag,
    reward_optimal_tracking,
)
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_interfaces.msg import (
    AlignedOdometry,
    LocalizationStatus,
    OdometrySession,
)
from brov_interfaces.srv import InitializePool

from .drag_test import (
    Limits,
    Phase,
    SteadyDetector,
    build_level_plans,
    coast_fit,
    lsq_slope,
    recirculation_check,
    transient_fit,
    wrap_pi,
)
from .dvl_reader import DvlReader
from .model_based_controller import ModelBasedController

_PAPER_VD = 0.5


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """[x,y,z,w] -> (roll, pitch, yaw). R = Rz(yaw)Ry(pitch)Rx(roll)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class _PID:
    """자세/깊이 유지용 최소 PID. 적분은 출력 한계에 맞춰 clamp한다."""

    def __init__(self, kp: float, ki: float, kd: float, out_limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = abs(out_limit)
        self._i = 0.0
        self._prev: float | None = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev = None

    def __call__(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        if self.ki > 0.0:
            limit = self.out_limit / self.ki
            self._i = max(-limit, min(limit, self._i + error * dt))
        out = self.kp * error + self.ki * self._i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, out))


class DragTestNode(Node):
    """수준별 open-loop surge를 유지하며 종단속도를 재고 항력을 적합한다."""

    def __init__(self):
        super().__init__("brov_drag_test")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # ── 시험 설계 ──
        self.declare_parameter("levels", [0.10, 0.20, 0.32, 0.45, 0.60])
        self.declare_parameter("repeat_first_level", True)
        self.declare_parameter("voltage", 15.4)
        self.declare_parameter("buoyancy_n", 1.96)
        self.declare_parameter("rate_hz", 25.0)

        # ── 정상상태 판정 (몬테카를로로 확정한 값) ──
        self.declare_parameter("settle_window_s", 1.2)
        self.declare_parameter("settle_slope", 0.015)
        self.declare_parameter("settle_sd", 0.05)
        # 정상상태를 감지하면 RUN을 즉시 끝낸다. 정지 상태(u≈0)의 창도
        # 기울기·표준편차가 0이라 "정상상태"를 통과하므로, 가속 구간을
        # 지나기 전에는 판정하지 않는다. run_timeout_s는 이제 상한일 뿐이다.
        # RUN 시작 이전 이 시간만큼도 기록에 남긴다. 추력 인가 직전의
        # 정지 상태가 과도구간 적합의 초기조건을 고정해준다.
        # A50 DVL 직결. EKF를 대체하지 않고 같은 시각의 두 속도를 나란히
        # 기록해 사후 교차검증만 한다. 논문이 쓰는 경로가 이쪽이다.
        self.declare_parameter("dvl_enabled", True)
        self.declare_parameter("dvl_host", "192.168.2.95")
        self.declare_parameter("dvl_port", 16171)
        self.declare_parameter("dvl_max_age_s", 0.5)
        self.declare_parameter("pre_run_s", 1.0)
        # 짧은 창은 가속 중의 일시적 평탄부를 정상상태로 오인한다. 실측에서
        # level 0.20이 t=2.09에 du/dt 0.0142(문턱 0.015)로 통과했지만 그 뒤로도
        # 계속 올랐다. 이 배수만큼 긴 창에서도 정상이어야 끝낸다.
        self.declare_parameter("settle_confirm_factor", 2.0)
        self.declare_parameter("min_run_s", 2.0)
        self.declare_parameter("run_timeout_s", 8.0)

        # ── pool 프레임 기하 ──
        # 비전(ArUco pool 정렬)을 쓰지 않는 모드. prepare 시점의 포즈를
        # 원점으로 잡고 x/y/yaw를 그 기준 상대값으로 낸다. 잃는 것은 절대
        # 벽 위치뿐이고, run_x_*/lane_y/target_z가 모두 상대량이 된다.
        self.declare_parameter("use_pool_alignment", True)
        self.declare_parameter("ref_max_linear_speed_mps", 0.03)
        self.declare_parameter("ref_max_angular_speed_rad_s", 0.05)
        self.declare_parameter("axis_heading_rad", 0.0)
        self.declare_parameter("run_x_min", 0.50)
        self.declare_parameter("run_x_max", 2.60)
        self.declare_parameter("run_start_margin_m", 0.15)
        self.declare_parameter("lane_y", 0.85)
        self.declare_parameter("max_cross_track_m", 0.35)
        self.declare_parameter("target_z", 0.575)
        self.declare_parameter("max_z_error_m", 0.30)
        self.declare_parameter("z_min", 0.20)
        self.declare_parameter("z_max", 0.90)
        self.declare_parameter("max_tilt_deg", 30.0)

        # ── 단계 시간 ──
        # 바닥에서 시작할 때만 쓴다. 0이면 종전대로 부유 상태 시작을 전제한다.
        self.declare_parameter("initial_ascent_m", 0.0)
        self.declare_parameter("ascend_tol_m", 0.08)
        self.declare_parameter("ascend_hold_s", 3.0)
        self.declare_parameter("ascend_timeout_s", 40.0)
        self.declare_parameter("start_hold_s", 30.0)
        self.declare_parameter("inter_level_wait_s", 60.0)
        self.declare_parameter("coast_s", 3.0)
        self.declare_parameter("approach_timeout_s", 60.0)
        self.declare_parameter("settle_yaw_timeout_s", 20.0)
        self.declare_parameter("settle_yaw_tol_deg", 5.0)
        self.declare_parameter("settle_yaw_hold_s", 1.5)
        self.declare_parameter("turnaround_timeout_s", 30.0)

        # ── 게인 (Z-up/FLU wrench 기준, model_based_controller와 같은 규약) ──
        self.declare_parameter("depth_kp", 60.0)
        self.declare_parameter("depth_ki", 8.0)
        self.declare_parameter("depth_kd", 20.0)
        self.declare_parameter("heave_limit_n", 60.0)
        self.declare_parameter("sway_kp", 25.0)
        self.declare_parameter("sway_kd", 12.0)
        self.declare_parameter("sway_limit_n", 12.0)
        self.declare_parameter("approach_surge_kp", 20.0)
        self.declare_parameter("approach_surge_limit_n", 15.0)
        self.declare_parameter("att_kp", 8.0)
        self.declare_parameter("att_ki", 1.0)
        self.declare_parameter("att_kd", 3.0)
        self.declare_parameter("moment_limit_nm", 12.0)
        self.declare_parameter("minimum_active_pwm", 0.10)
        self.declare_parameter("thruster_force_activation", 0.25)

        # ── 기타 ──
        self.declare_parameter("localization_max_age_s", 1.0)
        self.declare_parameter("odometry_max_age_s", 0.5)
        self.declare_parameter("localization_min_samples", 20)
        self.declare_parameter("alignment_sample_timeout_s", 30.0)
        self.declare_parameter("service_timeout_s", 15.0)
        # 단발 운용: prepare→start 한 번에 수준 하나만 재고, 끝나면 스스로
        # 정지·disarm 하고 IDLE로 돌아간다. 노드는 살아 있다. 운용자가
        # 기체를 여유 있는 자리로 옮겨 다시 prepare→start 하면 다음
        # 수준이 돈다. 모든 수준을 마치면 그때 기록을 저장하고 끝낸다.
        self.declare_parameter("single_level_per_start", False)
        self.declare_parameter("control_active_timeout_s", 5.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("send_pwm", True)

        p = self.get_parameter
        self._levels = [float(v) for v in p("levels").value]
        if not self._levels or any(not 0.0 < v <= 1.0 for v in self._levels):
            raise ValueError("levels는 (0, 1] 범위의 비어있지 않은 목록이어야 한다")
        if bool(p("repeat_first_level").value):
            # 최저 수준을 처음과 마지막에 두 번 재서 재순환 편향을 검사한다.
            self._levels = self._levels + [self._levels[0]]
        self._rate_hz = float(p("rate_hz").value)
        self._period = 1.0 / self._rate_hz
        self._buoyancy_n = float(p("buoyancy_n").value)
        self._send_pwm = bool(p("send_pwm").value)
        self._loc_max_age = float(p("localization_max_age_s").value)
        self._odom_max_age = float(p("odometry_max_age_s").value)
        self._service_timeout = float(p("service_timeout_s").value)

        self._limits = Limits(
            run_x_min=float(p("run_x_min").value),
            run_x_max=float(p("run_x_max").value),
            lane_y=float(p("lane_y").value),
            max_cross_track_m=float(p("max_cross_track_m").value),
            z_min=float(p("z_min").value),
            z_max=float(p("z_max").value),
            target_z=float(p("target_z").value),
            max_z_error_m=float(p("max_z_error_m").value),
            max_tilt_rad=math.radians(float(p("max_tilt_deg").value)),
        )
        self._single = bool(p("single_level_per_start").value)
        self._plans = build_level_plans(
            self._levels, self._limits,
            float(p("axis_heading_rad").value),
            float(p("run_start_margin_m").value),
            alternate_direction=not self._single,
        )

        # 추력 할당 — 명령↔전달 역산과 τ_max는 검증된 _Allocator를 그대로 쓴다.
        voltage = float(p("voltage").value)
        self._alloc = _Allocator(voltage)
        self._tau_max = self._alloc.surge_max_n
        self._linear_max = self._alloc.linear_max_n
        linear_level = (self._linear_max / self._tau_max
                        if self._tau_max > 0 else 0.0)
        hot = [lv for lv in self._levels if lv > linear_level]

        # PWM 발행은 model_based_controller의 할당 경로를 공유한다 —
        # ZUP->SNAME 부호와 deadband 처리를 다시 유도하지 않기 위해서다.
        params = load_brov2_yaml()
        pos, dir_ = thruster_pos_dir_ned(params)
        self._pwm_alloc = ModelBasedController(
            pos, dir_,
            minimum_active_pwm=float(p("minimum_active_pwm").value),
            thruster_force_activation=float(
                p("thruster_force_activation").value),
        )

        self.get_logger().info(
            f"전압 {voltage:.1f} V | 순수 surge 최대 전달 {self._tau_max:.1f} N "
            f"(= level 1.00) | PWM 비포화 한계 {self._linear_max:.1f} N "
            f"(= level {linear_level:.2f})"
        )
        self.get_logger().info(
            f"수준 {self._levels} — "
            + ("단발 운용: 수준마다 prepare→start, 끝나면 IDLE로 복귀"
               if self._single else "순서대로 왕복"))
        if hot:
            self.get_logger().error(
                f"level {hot}은 PWM 포화 구간이다 — 수평 4기가 한계에 붙어 yaw "
                f"권한을 잃는다. 직진이 안 되면 측정이 무의미하다. "
                f"level {linear_level:.2f} 이하로 낮출 것"
            )

        # ── 상태 ──
        self._state = "IDLE"
        self._phase = Phase.DONE
        self._plan_idx = 0
        self._phase_t0 = 0.0
        self._last_tick: float | None = None
        self._results: list[dict] = []
        self._timeseries: list[dict] = []
        self._detector: SteadyDetector | None = None
        self._coast: list[tuple[float, float]] = []
        self._yaw_ok_since: float | None = None
        self._yaw_peak = 0.0
        self._run_x0: float | None = None
        self._saved = False
        self._abort_reason: str | None = None
        # _emit()이 매 틱 갱신하지만 _run()이 같은 틱에서 먼저 읽는다.
        # 첫 RUN 틱에 대비해 여기서 정의해 둔다.
        self._delivered_surge = 0.0
        self._last_x = 0.0

        self._aligned: AlignedOdometry | None = None
        self._aligned_rx = 0.0
        self._use_pool = bool(p("use_pool_alignment").value)
        self._ref_max_lin = float(p("ref_max_linear_speed_mps").value)
        self._ref_max_ang = float(p("ref_max_angular_speed_rad_s").value)
        # 종료 처리 중임을 알린다. _tick은 상태기계를 멈추되 중립 PWM은
        # 계속 낸다 — obs_node의 PWM 워치독이 0.25s라, stop_control 왕복
        # 동안 발행이 끊기면 fault가 걸리고 그 래치는 reset_integrator로만
        # 풀린다("arming blocked by estop/fault/active state").
        self._ending_run = False
        # RUN 직전 구간을 담아두는 회전 버퍼. RUN이 시작되면 음수 t로 풀어
        # _timeseries 앞에 붙인다 — 추력 인가 직전의 정지 상태가 남는다.
        self._dvl: DvlReader | None = None
        if bool(p("dvl_enabled").value):
            self._dvl = DvlReader(str(p("dvl_host").value),
                                  int(p("dvl_port").value))
            self._dvl.start()
        self._dvl_max_age = float(p("dvl_max_age_s").value)
        self._preroll: deque = deque()      # (monotonic, record)
        self._run_t0: float | None = None   # RUN 시작 monotonic
        self._run_flushed = False           # 이번 RUN의 전기록을 풀었는지
        self._session_stamp = ""            # 세션별 기록 파일 이름
        self._z_target_override: float | None = None
        self._ascend_ok_since: float | None = None
        self._odom: OdometrySession | None = None
        self._odom_rx = 0.0
        # odom-상대 모드의 기준 프레임: (x0, y0, z0, yaw0, session_id)
        self._ref: tuple[float, float, float, float, str] | None = None
        self._loc: LocalizationStatus | None = None
        self._loc_rx = 0.0
        self._control_active = False
        self._discard_next_active = False

        self._depth_pid = _PID(float(p("depth_kp").value),
                               float(p("depth_ki").value),
                               float(p("depth_kd").value),
                               float(p("heave_limit_n").value))
        self._roll_pid = _PID(float(p("att_kp").value), float(p("att_ki").value),
                              float(p("att_kd").value),
                              float(p("moment_limit_nm").value))
        self._pitch_pid = _PID(float(p("att_kp").value), float(p("att_ki").value),
                               float(p("att_kd").value),
                               float(p("moment_limit_nm").value))
        self._yaw_pid = _PID(float(p("att_kp").value), float(p("att_ki").value),
                             float(p("att_kd").value),
                             float(p("moment_limit_nm").value))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_pwm = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 10)
        self.pub_status = self.create_publisher(
            String, "/brov/drag_test/status", latched)
        self.pub_level = self.create_publisher(
            String, "/brov/drag_test/level_result", 10)
        self.pub_wrench = self.create_publisher(
            Float32MultiArray, "/brov/drag_test/wrench_zup", 10)
        self.pub_surge = self.create_publisher(
            Float32MultiArray, "/brov/drag_test/delivered_surge_n", 10)

        self.create_subscription(
            AlignedOdometry, "/brov/localization/odometry_pool_with_alignment",
            self._on_aligned, qos_profile_sensor_data, callback_group=self._group)
        self.create_subscription(
            LocalizationStatus, "/brov/localization/status",
            self._on_loc, latched, callback_group=self._group)
        self.create_subscription(
            OdometrySession, "/brov/odometry/local_with_session",
            self._on_odom, qos_profile_sensor_data, callback_group=self._group)
        self.create_subscription(
            Bool, "/brov/control_active", self._on_active, 1,
            callback_group=self._group)

        self._svc = {
            "tilt": self.create_client(
                Trigger, "/brov/localization/confirm_camera_tilt_neutral",
                callback_group=self._group),
            "init": self.create_client(
                InitializePool, "/brov/localization/initialize_pool",
                callback_group=self._group),
            "arm": self.create_client(Trigger, "/brov/arm_control",
                                      callback_group=self._group),
            "start": self.create_client(Trigger, "/brov/start_control",
                                        callback_group=self._group),
            "stop": self.create_client(Trigger, "/brov/stop_control",
                                       callback_group=self._group),
            "disarm": self.create_client(Trigger, "/brov/disarm_control",
                                         callback_group=self._group),
        }
        self.create_service(Trigger, "/brov/drag_test/prepare",
                            self._on_prepare, callback_group=self._group)
        self.create_service(Trigger, "/brov/drag_test/start",
                            self._on_start, callback_group=self._group)
        self.create_service(Trigger, "/brov/drag_test/stop",
                            self._on_stop, callback_group=self._group)

        self.create_timer(self._period, self._tick, callback_group=self._group)
        if self._dvl is not None:
            # 연결 확인은 비동기다 — 여기서는 상태만 알린다. 값이 없으면
            # 기록에 None으로 남을 뿐 측정은 그대로 진행된다.
            connected, error = self._dvl.status
            if connected:
                self.get_logger().info(
                    f"DVL 직결: {self.get_parameter('dvl_host').value}:"
                    f"{self.get_parameter('dvl_port').value} — body 속도를 "
                    "EKF와 나란히 기록한다")
            else:
                self.get_logger().warn(
                    f"DVL 미연결({error or '대기 중'}) — 기록은 EKF만 남는다")
        self._publish_status("IDLE", "call /brov/drag_test/prepare while stationary")

    # ------------------------------------------------------------ 구독 콜백
    def _on_aligned(self, msg: AlignedOdometry) -> None:
        with self._lock:
            self._aligned = msg
            self._aligned_rx = time.monotonic()

    def _on_odom(self, msg: OdometrySession) -> None:
        with self._lock:
            self._odom = msg
            self._odom_rx = time.monotonic()

    def _on_loc(self, msg: LocalizationStatus) -> None:
        with self._lock:
            self._loc = msg
            self._loc_rx = time.monotonic()

    def _on_active(self, msg: Bool) -> None:
        with self._lock:
            was = self._control_active
            self._control_active = bool(msg.data)
            if self._control_active and not was:
                # 상승 에지의 첫 표본은 버린다 — dt와 PID 미분이 오염된다.
                self._discard_next_active = True

    # ------------------------------------------------------------ 상태 헬퍼
    def _publish_status(self, state: str, detail: str = "") -> None:
        self._state = state
        text = f"{state}: {detail}" if detail else state
        self.pub_status.publish(String(data=text))
        self.get_logger().info(f"[drag_test] {text}")

    def _neutral(self) -> None:
        if self._send_pwm:
            self.pub_pwm.publish(Float32MultiArray(data=[0.0] * 8))

    def _call(self, key: str, request=None, timeout: float | None = None):
        """서비스를 동기 호출한다. 실패하면 (False, 사유)."""
        client = self._svc[key]
        timeout = self._service_timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"{key} 서비스를 찾을 수 없다"
        future = client.call_async(
            Trigger.Request() if request is None else request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False, f"{key} 응답 시간 초과"
            time.sleep(0.02)
        response = future.result()
        if response is None:
            return False, f"{key} 응답 없음"
        return bool(response.success), str(response.message)

    def _odom_snapshot(self):
        """odom-상대 프레임 포즈 + body 속도. prepare 시점 포즈가 원점이다."""
        with self._lock:
            odom = self._odom
            odom_rx = self._odom_rx
            ref = self._ref
        if odom is None or time.monotonic() - odom_rx > self._odom_max_age:
            return None, "local odometry 없음/지연"
        if ref is None:
            return None, "기준 포즈 없음 — prepare 먼저"
        if str(odom.odometry_session_id) != ref[4]:
            return None, ("odometry session이 바뀌었다 — EKF가 재시작했으므로 "
                          "prepare부터 다시 할 것")

        pose = odom.odometry.pose.pose
        twist = odom.odometry.twist.twist
        roll, pitch, yaw = _quat_to_rpy(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        x0, y0, z0, yaw0, session = ref
        dx = float(pose.position.x) - x0
        dy = float(pose.position.y) - y0
        cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
        return {
            # 기준 프레임: 원점은 prepare 시점 위치, +X는 그때의 뱃머리 방향.
            "x": cos0 * dx + sin0 * dy,
            "y": -sin0 * dx + cos0 * dy,
            "z": float(pose.position.z) - z0,
            # roll/pitch는 중력 기준이라 절대값 그대로 쓴다.
            "roll": roll, "pitch": pitch, "yaw": wrap_pi(yaw - yaw0),
            "u": float(twist.linear.x),
            "v": float(twist.linear.y),
            "w": float(twist.linear.z),
            # body FLU 각속도. 테더 장력·벽 근접이 만드는 교란은 surge보다
            # 횡/수직 성분과 각속도에 먼저 나타난다.
            "p": float(twist.angular.x),
            "q": float(twist.angular.y),
            "r": float(twist.angular.z),
            "epoch": 0,
            "alignment_id": f"odom_relative:{session}",
        }, ""

    def _capture_reference(self):
        """prepare 시점의 정지 포즈를 기준 프레임으로 고정한다."""
        with self._lock:
            odom = self._odom
            odom_rx = self._odom_rx
        if odom is None or time.monotonic() - odom_rx > self._odom_max_age:
            return False, "local odometry 없음/지연"
        twist = odom.odometry.twist.twist
        linear = math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2
                           + twist.linear.z ** 2)
        angular = math.sqrt(twist.angular.x ** 2 + twist.angular.y ** 2
                            + twist.angular.z ** 2)
        if linear > self._ref_max_lin or angular > self._ref_max_ang:
            return False, (f"기체가 정지 상태가 아니다 "
                           f"(선속도 {linear:.3f} m/s, 각속도 {angular:.3f} rad/s)")
        pose = odom.odometry.pose.pose
        _, _, yaw = _quat_to_rpy(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        with self._lock:
            self._ref = (float(pose.position.x), float(pose.position.y),
                         float(pose.position.z), yaw,
                         str(odom.odometry_session_id))
        return True, (f"기준 포즈 고정 — 방위 {math.degrees(yaw):+.1f}° "
                      f"(odom {pose.position.x:+.2f}, {pose.position.y:+.2f}, "
                      f"{pose.position.z:+.2f})")

    def _state_snapshot(self):
        """pool 프레임 포즈 + body 속도. 신선하지 않으면 None."""
        if not self._use_pool:
            return self._odom_snapshot()
        with self._lock:
            aligned = self._aligned
            aligned_rx = self._aligned_rx
            loc = self._loc
            loc_rx = self._loc_rx
        now = time.monotonic()
        if aligned is None or now - aligned_rx > self._odom_max_age:
            return None, "pool odometry 없음/지연"
        if loc is None or now - loc_rx > self._loc_max_age:
            return None, "localization status 없음/지연"
        if loc.state != LocalizationStatus.INITIALIZED or not loc.output_valid:
            return None, f"정렬 무효 (state={loc.state}, valid={loc.output_valid})"

        pose = aligned.odometry.pose.pose
        twist = aligned.odometry.twist.twist
        roll, pitch, yaw = _quat_to_rpy(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "roll": roll, "pitch": pitch, "yaw": yaw,
            # AlignedOdometry의 twist는 base_link FLU 그대로다 (회전되지 않음).
            "u": float(twist.linear.x),
            "v": float(twist.linear.y),
            "w": float(twist.linear.z),
            "p": float(twist.angular.x),
            "q": float(twist.angular.y),
            "r": float(twist.angular.z),
            "epoch": int(aligned.localization_epoch),
            "alignment_id": str(aligned.alignment_id),
        }, ""

    # ------------------------------------------------------------ 서비스
    def _wait_for_alignment_samples(self, required: int):
        """정렬 표본이 다시 찰 때까지 기다린다.

        `confirm_camera_tilt_neutral`은 확인 시점에 `_clear_measurements()`로
        기존 표본을 전부 버린다 — 알 수 없는 틸트에서 모은 표본을 남기지
        않으려는 의도다. 그래서 틸트 확인 직후에 `initialize_pool`을 부르면
        버퍼가 비어 있어 `waiting for samples: 0/N`으로 반드시 실패하고,
        `_last_odom`까지 None이라 그 앞의 신선도 검사에서
        `fresh local odometry is unavailable`이 나기도 한다. 기다린다.
        """
        timeout = float(self.get_parameter("alignment_sample_timeout_s").value)
        deadline = time.monotonic() + timeout
        count = 0
        reported = -1
        while time.monotonic() < deadline:
            with self._lock:
                loc = self._loc
            count = int(loc.sample_count) if loc is not None else 0
            if count >= required:
                return True, f"정렬 표본 {count}/{required}"
            if count != reported:
                reported = count
                self._publish_status("ALIGNING", f"정렬 표본 {count}/{required}")
            time.sleep(0.1)
        return False, (
            f"정렬 표본 부족 {count}/{required} ({timeout:.0f}s) — 기체를 "
            "정지시키고 마커가 보이는지 확인할 것")

    def _on_prepare(self, _request, response):
        if self._state not in ("IDLE", "ALIGNED", "STOPPED"):
            response.success = False
            response.message = f"prepare는 {self._state}에서 호출할 수 없다"
            return response
        if not self._use_pool:
            # 비전 없음 — 지금 포즈를 기준으로 잡고 끝이다.
            self._publish_status("ALIGNING", "기준 포즈 고정 (비전 미사용)")
            ok, msg = self._capture_reference()
            self._publish_status("ALIGNED" if ok else "IDLE", msg)
            response.success, response.message = ok, msg
            return response
        self._publish_status("ALIGNING", "confirm tilt + initialize pool")
        ok, msg = self._call("tilt")
        if not ok:
            self._publish_status("IDLE", f"카메라 틸트 확인 실패: {msg}")
            response.success, response.message = False, msg
            return response
        required = int(self.get_parameter("localization_min_samples").value)
        ok, msg = self._wait_for_alignment_samples(required)
        if not ok:
            self._publish_status("IDLE", msg)
            response.success, response.message = False, msg
            return response
        request = InitializePool.Request()
        request.min_samples = required
        ok, msg = self._call("init", request)
        if not ok:
            self._publish_status("IDLE", f"정렬 초기화 실패: {msg}")
            response.success, response.message = False, msg
            return response
        self._publish_status("ALIGNED", msg)
        response.success, response.message = True, msg
        return response

    def _wait_for_control_active(self):
        """obs_node가 /brov/control_active=true를 낼 때까지 기다린다.

        _tick은 state가 ARMED/RUNNING일 때만 동작하므로, ARMED로 넘어가기 전인
        이 구간에서 기다리는 것은 안전하다. 콜백그룹이 Reentrant라 대기 중에도
        구독 콜백이 처리된다.
        """
        timeout = float(self.get_parameter("control_active_timeout_s").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._control_active:
                    return True, "base control 활성 확인"
            time.sleep(0.02)
        return False, (f"base control 활성 확인 실패 ({timeout:.1f}s) — "
                       "obs_node가 /brov/control_active를 내지 않는다")

    def _on_start(self, _request, response):
        if self._state != "ALIGNED":
            response.success = False
            response.message = f"prepare 먼저 — 현재 {self._state}"
            return response
        snap, why = self._state_snapshot()
        if snap is None:
            response.success, response.message = False, why
            return response

        ok, msg = self._call("arm")
        if not ok:
            self._publish_status("ALIGNED", f"arm 실패: {msg}")
            response.success, response.message = False, msg
            return response
        ok, msg = self._call("start")
        if not ok:
            self._call("disarm")
            self._publish_status("ALIGNED", f"start 실패: {msg}")
            response.success, response.message = False, msg
            return response
        # start_control이 성공해도 /brov/control_active는 곧바로 오지 않는다.
        # obs_node는 주기 루프에서 `_control_active and _active_obs_published`를
        # 발행하므로 최소 한 틱(40ms) 이상 늦는다. 그 사이에 ARMED로 넘어가면
        # 첫 _tick이 아직 False인 값을 보고 "base control이 비활성화됨"으로
        # 즉시 중단한다 — 실제로 그렇게 죽었다. 활성 확인까지 기다린다.
        ok, msg = self._wait_for_control_active()
        if not ok:
            self._call("stop", timeout=8.0)
            self._call("disarm", timeout=8.0)
            self._publish_status("ALIGNED", msg)
            response.success, response.message = False, msg
            return response

        with self._lock:
            if not self._single:
                # 연속 운용은 start 한 번이 곧 한 세션이다.
                self._plan_idx = 0
            if not self._single or self._plan_idx == 0:
                self._session_stamp = time.strftime("%Y%m%d_%H%M%S",
                                                    time.localtime())
                # 단발 운용에서는 회차 사이에 누적 결과를 지우지 않는다.
                # 첫 회차에서만 새 세션으로 초기화한다.
                self._results.clear()
                self._timeseries.clear()
                self._saved = False
            self._abort_reason = None
            self._reset_pids()
            self._ascend_ok_since = None
            ascent = float(self.get_parameter("initial_ascent_m").value)
            if ascent > 0.0 and self._use_pool:
                # pool 모드의 target_z는 수조 바닥 기준 절대값이라 '상대 상승'이
                # 성립하지 않는다. 절대 좌표를 알면 target_z만 바꾸면 된다.
                msg = ("initial_ascent_m은 use_pool_alignment=false에서만 "
                       "쓴다 — pool 모드는 target_z를 직접 지정할 것")
                self._publish_status("ALIGNED", msg)
                response.success, response.message = False, msg
                return response
            if ascent > 0.0:
                # 기준 포즈가 바닥에서 잡혔다. 목표 깊이는 그보다 ascent 만큼
                # 위이고, 도달하면 그 자리에서 기준 프레임을 다시 잡는다.
                self._z_target_override = ascent
                self._enter_phase(Phase.ASCEND)
            else:
                self._z_target_override = None
                self._enter_phase(Phase.APPROACH)
        if self._phase == Phase.ASCEND:
            self._publish_status(
                "ARMED",
                f"상승 {float(self.get_parameter('initial_ascent_m').value):.2f} m "
                f"— yaw 고정, 도달 후 호버링하며 기준 재설정")
        else:
            self._publish_status(
                "ARMED",
                f"start_hold {self.get_parameter('start_hold_s').value:.0f}s "
                f"— 지금 테더를 놓고 깊이와 방위가 잡히는지 볼 것")
        response.success, response.message = True, "시험 시작"
        return response

    def _on_stop(self, _request, response):
        self._finish("운용자 중단")
        response.success, response.message = True, self._state
        return response

    # ------------------------------------------------------------ 상태기계
    def _reset_pids(self) -> None:
        for pid in (self._depth_pid, self._roll_pid,
                    self._pitch_pid, self._yaw_pid):
            pid.reset()

    def _enter_phase(self, phase: Phase) -> None:
        self._phase = phase
        self._phase_t0 = time.monotonic()
        self._yaw_ok_since = None
        if phase == Phase.RUN:
            self._detector = SteadyDetector(
                window_s=float(self.get_parameter("settle_window_s").value),
                max_slope=float(self.get_parameter("settle_slope").value),
                max_sd=float(self.get_parameter("settle_sd").value),
            )
            self._run_x0 = None
            self._run_t0 = self._phase_t0
            self._run_flushed = False
            self._yaw_peak = 0.0
        elif phase == Phase.COAST:
            self._coast = []

    def _current_plan(self):
        if 0 <= self._plan_idx < len(self._plans):
            return self._plans[self._plan_idx]
        return None

    def _end_single_run(self) -> None:
        """수준 하나를 끝내고 기체를 안전 상태로 되돌린다 — 세션은 계속된다.

        기록은 저장하지 않는다(세션 끝에 한 번만 저장한다). 기준 프레임을
        버리므로 다음 회차는 반드시 prepare부터 다시 해야 한다 — 운용자가
        기체를 옮긴 뒤의 자리가 새 원점이 되어야 하기 때문이다.
        """
        self._ending_run = True
        try:
            self._neutral()
            self._call("stop", timeout=8.0)
            self._call("disarm", timeout=8.0)
        finally:
            self._phase = Phase.DONE
            self._ending_run = False
        with self._lock:
            self._ref = None
            self._z_target_override = None
            self._ascend_ok_since = None
        # 여기서 저장하지 않으면 회차 사이의 Ctrl+C에 그때까지가 통째로
        # 날아간다 — 운용자가 기체를 옮기는 동안 스택을 재시작할 수 있다.
        # 세션 파일 하나를 회차마다 덮어써 항상 최신으로 유지한다.
        self._save_and_report("회차 완료 (세션 진행 중)", checkpoint=True)
        done, total = self._plan_idx, len(self._plans)
        nxt = self._plans[self._plan_idx].level
        self._publish_status(
            "IDLE",
            f"{done}/{total} 완료 — 기체를 여유 있는 자리로 옮기고 정지시킨 뒤 "
            f"prepare → start (다음 level {nxt:.2f})")

    def _finish(self, reason: str) -> None:
        """어떤 경로로 끝나도 중립·정지·기록 저장까지 반드시 수행한다."""
        self._ending_run = True
        try:
            self._neutral()
            if self._state in ("ARMED", "RUNNING"):
                self._call("stop", timeout=8.0)
                self._call("disarm", timeout=8.0)
        finally:
            self._phase = Phase.DONE
            self._ending_run = False
        if not self._saved:
            self._saved = True
            self._save_and_report(reason)
        with self._lock:
            # 다음 세션은 첫 수준부터. 단발 운용에서 중단으로 끝난 경우에도
            # 이어 달리지 않고 새로 시작하게 한다.
            self._plan_idx = 0
            self._ref = None
        self._publish_status("STOPPED", reason)

    # ------------------------------------------------------------ 주기 루프
    def _tick(self) -> None:
        if self._phase == Phase.DONE or self._state not in ("ARMED", "RUNNING"):
            return
        if self._ending_run:
            # stop/disarm 왕복 중. 상태기계는 세우고 워치독만 먹인다.
            self._neutral()
            return

        now = time.monotonic()
        dt = self._period if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        with self._lock:
            discard = self._discard_next_active
            self._discard_next_active = False
            active = self._control_active
        if discard:
            return
        if not active and self._send_pwm:
            self._finish("base control이 비활성화됨")
            return

        snap, why = self._state_snapshot()
        if snap is None:
            self._finish(f"상태 불가: {why}")
            return

        violation = self._limits.violation(
            snap["x"], snap["y"], snap["z"], snap["roll"], snap["pitch"],
            check_depth=self._phase != Phase.ASCEND)
        elapsed = now - self._phase_t0

        # RUN 중의 한계 위반은 그 수준을 끝낼 뿐 시험 전체를 끝내지 않는다 —
        # 마지막 창이 정상상태였다면 표본은 여전히 유효하다.
        if violation is not None and self._phase != Phase.RUN:
            self._finish(f"안전 한계: {violation}")
            return

        plan = self._current_plan()
        if plan is None:
            self._finish("모든 수준 완료")
            return

        # 단계 처리기가 _enter_phase로 단계를 바꾸므로, 기록에는 이 틱이
        # 실제로 속했던 단계를 남긴다. 안 그러면 RUN의 마지막 표본이 COAST로
        # 잘못 기록된다 — 사후 적합에서 그 표본이 엉뚱한 구간에 들어간다.
        tick_phase = self._phase
        surge_n = 0.0
        if self._phase == Phase.ASCEND:
            surge_n = self._ascend(snap, elapsed, now)
        elif self._phase == Phase.APPROACH:
            surge_n = self._approach(snap, plan, elapsed)
        elif self._phase == Phase.SETTLE_YAW:
            surge_n = self._settle_yaw(snap, plan, elapsed, now)
        elif self._phase == Phase.RUN:
            surge_n = self._run(snap, plan, elapsed, violation)
        elif self._phase == Phase.COAST:
            surge_n = self._coast_phase(snap, elapsed)
        elif self._phase == Phase.TURNAROUND:
            surge_n = self._turnaround(snap, elapsed)
        elif self._phase == Phase.WAIT:
            surge_n = self._wait(snap, elapsed)

        self._emit(snap, plan, surge_n, dt)
        self._record(snap, plan, surge_n, now, tick_phase)

    # -------- 단계별 --------
    def _ascend(self, snap, elapsed, now) -> float:
        """바닥에서 목표 깊이까지 올라가 호버링한다. surge는 내지 않는다.

        도달하면 기준 프레임을 그 자리에서 다시 잡는다. 바닥에서는 A50이
        bottom lock 최소고도 밑이라 EKF 수평속도가 부실할 수 있는데, 상승 전
        원점을 그대로 쓰면 그 구간의 드리프트가 주행거리에 그대로 실린다.
        """
        target = float(self._z_target_override or 0.0)
        if elapsed > float(self.get_parameter("ascend_timeout_s").value):
            self._finish(f"상승 시간 초과 (z={snap['z']:+.2f} 목표 {target:+.2f})")
            return 0.0
        # 상승 회랑 — z_min/z_max 대신 이 구간을 감시한다. 바닥 아래로
        # 가라앉거나 목표를 크게 넘어 수면으로 솟는 것을 막는다.
        band = float(self.get_parameter("max_z_error_m").value)
        if not (-band <= snap["z"] <= target + band):
            self._finish(
                f"상승 회랑 이탈 z={snap['z']:+.2f} "
                f"(허용 {-band:+.2f}~{target + band:+.2f})")
            return 0.0
        tol = float(self.get_parameter("ascend_tol_m").value)
        hold = float(self.get_parameter("ascend_hold_s").value)
        if abs(snap["z"] - target) > tol:
            self._ascend_ok_since = None
            return 0.0
        if self._ascend_ok_since is None:
            self._ascend_ok_since = now
            return 0.0
        if now - self._ascend_ok_since < hold:
            return 0.0

        ok, msg = self._capture_reference() if not self._use_pool else (True, "")
        if not ok:
            # 아직 흔들린다 — 중단하지 않고 다시 안정될 때까지 기다린다.
            # 계속 실패하면 ascend_timeout_s가 잡는다.
            self._ascend_ok_since = None
            return 0.0
        # 재설정 후에는 지금 깊이가 곧 목표다 — target_z(=0)로 되돌린다.
        self._z_target_override = None
        self._reset_pids()
        self.get_logger().info(
            f"상승 완료 — {target:+.2f} m 호버링, 기준 프레임 재설정. {msg}")
        self._publish_status(
            "ARMED",
            f"start_hold {self.get_parameter('start_hold_s').value:.0f}s "
            f"— 지금 테더를 놓고 깊이와 방위가 잡히는지 볼 것")
        self._enter_phase(Phase.APPROACH)
        return 0.0

    def _approach(self, snap, plan, elapsed) -> float:
        if elapsed > float(self.get_parameter("approach_timeout_s").value):
            self._finish("주행 시점 접근 시간 초과")
            return 0.0
        hold = float(self.get_parameter("start_hold_s").value)
        dx = plan.start_x - snap["x"]
        kp = float(self.get_parameter("approach_surge_kp").value)
        lim = float(self.get_parameter("approach_surge_limit_n").value)
        # 주행축 오차를 기체 전방 성분으로 투영한다.
        surge = max(-lim, min(lim, kp * dx * math.cos(snap["yaw"])))

        # 첫 수준은 운용자가 테더를 놓을 시간을 준다. 그 30초 동안에도 주행
        # 시점을 붙잡는다 — 예전에는 여기서 0을 냈는데, 깊이/방위/차선만 닫히고
        # x는 자유라 테더 장력에 0.006 m/s로 밀려 26초 만에 주행축 하한을
        # 밟았다. 이 구간은 측정이 아니므로 닫아도 open-loop 원칙에 걸리지
        # 않는다 (RUN만 open-loop여야 한다).
        if self._plan_idx == 0 and elapsed < hold:
            return surge

        if abs(dx) < 0.10:
            self._publish_status("RUNNING",
                                 f"level {plan.level:.2f} 방위 정렬 대기")
            self._enter_phase(Phase.SETTLE_YAW)
            return 0.0
        return surge

    def _settle_yaw(self, snap, plan, elapsed, now) -> float:
        tol = math.radians(float(self.get_parameter("settle_yaw_tol_deg").value))
        hold = float(self.get_parameter("settle_yaw_hold_s").value)
        err = abs(wrap_pi(plan.heading - snap["yaw"]))
        if err < tol:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = now
            elif now - self._yaw_ok_since >= hold:
                self.get_logger().info(
                    f"[{self._plan_idx + 1}/{len(self._plans)}] level "
                    f"{plan.level:.2f} → 목표 전달 "
                    f"{plan.level * self._tau_max:.1f} N, "
                    f"방위 {math.degrees(plan.heading):+.0f}°")
                self._enter_phase(Phase.RUN)
        else:
            self._yaw_ok_since = None
        if elapsed > float(self.get_parameter("settle_yaw_timeout_s").value):
            self._finish(f"방위 정렬 실패 (오차 {math.degrees(err):.0f}°) — "
                         f"AHRS 방위나 yaw 권한을 확인할 것")
        return 0.0

    def _run(self, snap, plan, elapsed, violation) -> float:
        if self._run_x0 is None:
            self._run_x0 = snap["x"]
        assert self._detector is not None
        target = plan.level * self._tau_max
        cmd = self._alloc.surge_command_for(target)
        # 전달 추력은 명령이 아니라 할당 왕복으로 계산한 값을 쓴다 —
        # sway/yaw가 수평 4기를 나눠 쓰면서 줄어드는 몫까지 반영된다.
        self._detector.add(elapsed, snap["u"], self._delivered_surge)
        self._yaw_peak = max(self._yaw_peak,
                             abs(wrap_pi(plan.heading - snap["yaw"])))

        timeout = elapsed > float(self.get_parameter("run_timeout_s").value)
        if violation is not None or timeout:
            self._end_run(plan, violation or "주행 시간 초과", elapsed, snap["x"])
            return 0.0

        # 정상상태에 들면 더 달릴 이유가 없다. 거리를 아끼는 것이자, 꼬리 창이
        # 뒤늦은 교란(테더 장력·벽 근접·재순환)에 오염되기 전에 끊는 것이다.
        # 실제로 8초를 다 쓰다가 감속 구간을 정상상태로 잡은 적이 있다.
        if elapsed >= float(self.get_parameter("min_run_s").value):
            window = float(self.get_parameter("settle_window_s").value)
            factor = float(self.get_parameter("settle_confirm_factor").value)
            out = self._detector.evaluate()
            if out.get("steady") and out.get("window_s", 0.0) >= window * 0.9:
                long_window = window * max(1.0, factor)
                confirm = self._detector.evaluate(long_window)
                if (confirm.get("steady")
                        and confirm.get("window_s", 0.0) >= long_window * 0.9):
                    self._end_run(plan, "정상상태 도달", elapsed, snap["x"])
                    return 0.0
        return cmd

    def _end_run(self, plan, abort_reason, elapsed, x_now: float) -> None:
        assert self._detector is not None
        out = self._detector.evaluate()
        out.update({
            "level": plan.level,
            "tau_x_max_n": self._tau_max,
            "abort": abort_reason,
            "elapsed_s": elapsed,
            "travelled_m": (abs(x_now - self._run_x0)
                            if self._run_x0 is not None else 0.0),
            "yaw_peak_deg": math.degrees(self._yaw_peak),
            "heading_deg": math.degrees(plan.heading),
        })
        self._results.append(out)
        if out["steady"]:
            self.get_logger().info(
                f"    정상상태 u = {out['u_mps']:+.3f} m/s "
                f"(sd {out['u_sd']:.3f}, du/dt {out['du_dt']:+.4f}), "
                f"전달 추력 {out['tau_x_n']:.1f} N, "
                f"주행 {out['travelled_m']:.2f} m")
        else:
            self.get_logger().warn(
                f"    사용 불가 — {out.get('reason') or abort_reason}")
        if out["yaw_peak_deg"] > 10.0:
            self.get_logger().warn(
                f"    [경고] 주행 중 방위 편차 {out['yaw_peak_deg']:.0f}° — "
                f"직진이 유지되지 않았다")
        self.pub_level.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self._enter_phase(Phase.COAST)

    def _coast_phase(self, snap, elapsed) -> float:
        self._coast.append((elapsed, snap["u"]))
        if elapsed >= float(self.get_parameter("coast_s").value):
            if self._results:
                self._results[-1]["coast"] = self._coast[:]
            self._plan_idx += 1
            if self._plan_idx >= len(self._plans):
                self._finish("모든 수준 완료")
            elif self._single:
                self._end_single_run()
            else:
                self._enter_phase(Phase.TURNAROUND)
        return 0.0

    def _turnaround(self, snap, elapsed) -> float:
        plan = self._current_plan()
        if plan is None:
            return 0.0
        tol = math.radians(float(self.get_parameter("settle_yaw_tol_deg").value))
        if abs(wrap_pi(plan.heading - snap["yaw"])) < tol * 2.0:
            self._enter_phase(Phase.WAIT)
        elif elapsed > float(self.get_parameter("turnaround_timeout_s").value):
            self._finish("선회 시간 초과")
        return 0.0

    def _wait(self, snap, elapsed) -> float:
        # 재순환 안정화. 물 7 m^3에 제트를 반복 주입하면 벽 반사류가 생기고,
        # 기체가 자기가 만든 흐름을 타면 종단속도가 편향된다.
        if elapsed >= float(self.get_parameter("inter_level_wait_s").value):
            self._enter_phase(Phase.APPROACH)
        return 0.0

    # -------- 기록 --------
    def _sample(self, snap, plan, surge_cmd: float, t: float,
                phase: Phase) -> dict:
        """한 틱의 상태 전부. 사후 해석이 다시 물을 일이 없게 다 담는다."""
        return {
            "level": None if plan is None else plan.level,
            "phase": phase.value,
            "t": t,
            # body FLU 선속도 / 각속도
            "u": snap["u"], "v": snap["v"], "w": snap["w"],
            "p": snap["p"], "q": snap["q"], "r": snap["r"],
            # 자세 (roll/pitch는 중력 기준 절대값, yaw는 기준 프레임 상대)
            "roll_deg": math.degrees(snap["roll"]),
            "pitch_deg": math.degrees(snap["pitch"]),
            "yaw_deg": math.degrees(snap["yaw"]),
            # 기준 프레임 위치
            "x": snap["x"], "y": snap["y"], "z": snap["z"],
            # 명령 surge와 할당 왕복으로 계산한 전달 surge
            "tau_x_cmd": float(surge_cmd),
            "tau_x_delivered": self._delivered_surge,
            **(self._dvl.sample(max_age_s=self._dvl_max_age)
               if self._dvl is not None else {}),
        }

    def _record(self, snap, plan, surge_cmd: float, now: float,
                phase: Phase) -> None:
        """RUN/COAST는 기록하고, 그 앞 구간은 회전 버퍼에 담아둔다."""
        pre = float(self.get_parameter("pre_run_s").value)
        if phase in (Phase.RUN, Phase.COAST) and self._run_t0 is not None:
            t = now - self._run_t0
            if phase == Phase.RUN and not self._run_flushed:
                self._run_flushed = True
                # RUN 첫 틱 — 직전 pre_run_s를 음수 t로 풀어 앞에 붙인다.
                for stamp, rec in self._preroll:
                    age = stamp - self._run_t0
                    if age < -pre:
                        continue
                    rec = dict(rec)
                    rec["t"] = age
                    rec["level"] = None if plan is None else plan.level
                    self._timeseries.append(rec)
            self._timeseries.append(
                self._sample(snap, plan, surge_cmd, t, phase))
            self._preroll.clear()
            return
        self._preroll.append(
            (now, self._sample(snap, plan, surge_cmd, 0.0, phase)))
        while self._preroll and now - self._preroll[0][0] > pre:
            self._preroll.popleft()

    # -------- wrench 조립과 발행 --------
    def _emit(self, snap, plan, surge_n: float, dt: float) -> None:
        """surge만 open-loop, 나머지 5축은 닫는다."""
        heading = plan.heading if plan is not None else snap["yaw"]

        # Z-up/FLU wrench. model_based_controller와 같은 규약이므로 부호를
        # 다시 유도하지 않는다.
        wrench = torch.zeros(6, dtype=torch.float32)
        wrench[0] = surge_n

        # sway: pool 차선 중심선 유지. pool 프레임 오차 (0, lane_err)를 기체
        # FLU로 회전한다 — body_y = -sin(yaw)*e_x + cos(yaw)*e_y, e_x=0.
        # surge 힘 균형에는 직교라 측정을 오염시키지 않는다. 수평 4기를 나눠
        # 쓰면서 줄어드는 τ_x는 전달값으로 재므로 자기일관적이다.
        lane_err = self._limits.lane_y - snap["y"]
        sway_body = math.cos(snap["yaw"]) * lane_err
        sway_kp = float(self.get_parameter("sway_kp").value)
        sway_kd = float(self.get_parameter("sway_kd").value)
        sway_lim = float(self.get_parameter("sway_limit_n").value)
        sway = sway_kp * sway_body - sway_kd * snap["v"]
        wrench[1] = max(-sway_lim, min(sway_lim, sway))

        # heave: pool +Z는 위쪽이므로 (목표 - 실측)이 양수면 위로 밀어야 한다.
        # 순중량은 상시 전방보상 — 없으면 수준마다 적분이 0에서 시작해 가라앉는다.
        z_target = (self._limits.target_z if self._z_target_override is None
                    else self._z_target_override)
        wrench[2] = (self._depth_pid(z_target - snap["z"], dt)
                     + self._buoyancy_n)
        wrench[3] = self._roll_pid(wrap_pi(0.0 - snap["roll"]), dt)
        wrench[4] = self._pitch_pid(wrap_pi(0.0 - snap["pitch"]), dt)
        wrench[5] = self._yaw_pid(wrap_pi(heading - snap["yaw"]), dt)

        out = self._pwm_alloc.allocate(wrench)
        self._delivered_surge = float(out.estimated_wrench_zup[0])
        self._last_x = snap["x"]

        if self._send_pwm:
            self.pub_pwm.publish(
                Float32MultiArray(data=[float(v) for v in out.pwm.tolist()]))
        self.pub_wrench.publish(
            Float32MultiArray(data=[float(v) for v in wrench.tolist()]))
        self.pub_surge.publish(
            Float32MultiArray(data=[self._delivered_surge, surge_n]))

    # ------------------------------------------------------------ 결과
    def _save_and_report(self, reason: str, *, checkpoint: bool = False) -> None:
        """기록을 남긴다.

        ``checkpoint=True``는 회차 사이 중간 저장이다 — 세션 파일만 갱신하고
        고정 경로도 건드리지 않고 보고도 하지 않는다.
        """
        path = str(self.get_parameter("output_path").value)
        if not path:
            data_dir = os.environ.get("BROV_DATA_DIR",
                                      os.path.expanduser("~/.ros/brov"))
            path = os.path.join(data_dir, "drag_test.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)

        fit = fit_drag(self._results)
        # 과도구간 적합 — 정상상태에 도달하지 못한 수준까지 살려 쓴다.
        # Xu는 정상상태 적합 값으로 고정한다(저속에서 u와 u|u|가 공선이라
        # 그러지 않으면 분리되지 않는다 — coast_fit과 같은 이유다).
        transient = transient_fit(
            self._timeseries,
            xu_known=fit["Xu"] if fit.get("ok") else None)
        # coast_fit이 쓰는 질량. 과도구간 적합이 m_eff를 재긴 하지만 잡음에
        # 낮게 치우치므로(감쇠편향), 기본값은 모델값을 유지하고 측정값은
        # 보고에만 남긴다.
        mass_eff = 14.635 + 6.36
        coasts = []
        for r in self._results:
            if r.get("coast") and fit.get("ok"):
                c = coast_fit(r["coast"], mass_eff, xu_known=fit["Xu"])
                if c.get("ok"):
                    c["level"] = r["level"]
                    coasts.append(c)
        recirc = recirculation_check(self._results)

        payload = {
            "stop_reason": reason,
            "tau_x_max_n": self._tau_max,
            "pwm_linear_max_n": self._linear_max,
            "voltage_v": float(self.get_parameter("voltage").value),
            "buoyancy_n": self._buoyancy_n,
            "levels": self._results,
            "fit": fit,
            "transient_fit": transient,
            "coast_fits": coasts,
            "recirculation": recirc,
            "timeseries": self._timeseries,
        }
        # 세션 파일을 먼저 남긴다. 고정 경로 하나만 쓰면 곧바로 끝난 다음
        # 세션이 직전의 좋은 결과를 덮어쓴다 — 실제로 수준 5개짜리 결과가
        # 그렇게 사라졌다. 세션 파일은 무슨 일이 있어도 남는다.
        stem, ext = os.path.splitext(path)
        stamp = self._session_stamp or time.strftime("%Y%m%d_%H%M%S",
                                                     time.localtime())
        archive = f"{stem}_{stamp}{ext}"
        targets = (archive,) if checkpoint else (archive, path)
        for target in targets:
            if target == path and not self._results:
                # 빈 세션은 고정 경로를 건드리지 않는다. 사본은 이미 남겼다.
                self.get_logger().warn(
                    f"수준 0개 — {path}를 덮어쓰지 않는다 (사본: {archive})")
                continue
            try:
                with open(target, "w", encoding="utf-8") as fh:
                    json.dump(payload, fh, indent=2, ensure_ascii=False)
                self.get_logger().info(
                    f"기록 저장: {target} (수준 {len(self._results)}개, "
                    f"표본 {len(self._timeseries)}개)")
            except Exception as error:
                self.get_logger().error(f"기록 저장 실패 {target}: {error}")

        if not checkpoint:
            self._report(fit, coasts, recirc, transient)

    def _report(self, fit: dict, coasts: list[dict], recirc: dict,
                transient: dict | None = None) -> None:
        log = self.get_logger()
        if not fit.get("ok"):
            log.error(f"[적합 실패] {fit.get('reason')}")
            return
        a = fit["A_ratio"]
        track = reward_optimal_tracking(fit["Xu"], fit["Xuu"],
                                        fit["tau_x_max_n"])
        log.info("=" * 60)
        log.info(f"실측 surge 항력   Xu = {fit['Xu']:.2f} N/(m/s)   "
                 f"Xuu = {fit['Xuu']:.2f} N/(m/s)^2   R^2 = {fit['r2']:.4f}   "
                 f"표본 {fit['n_samples']}개")
        log.info(f"  최대 surge 추력 {fit['tau_x_max_n']:.1f} N  →  "
                 f"v_max = {fit['v_max_mps']:.3f} m/s")
        log.info(f"  V_d = {_PAPER_VD} m/s 유지 정규화 추력 A = {a:.3f}   "
                 f"보상최적 추종률 {100 * track:.0f}%")
        # 판정 경계는 수조 오염(블로키지+재순환)이 값을 낮추기만 한다는 것에 근거한다.
        if fit["v_max_mps"] >= 1.15:
            log.info("→ 가설 B 확정: 시뮬레이션 Xuu(141)가 과대다. 계수를 먼저 고칠 것")
        elif fit["v_max_mps"] <= 0.95:
            log.info("→ 가설 A: plant가 맞다. 논문 보상 Eq.(8)을 바꿔야 한다")
        else:
            log.warn("→ 판정 애매 (0.95~1.15). 저수준만으로 재적합해 볼 것")
        for c in coasts:
            log.info(f"  [타행 교차검증] level {c['level']:.2f}: "
                     f"Xuu = {c['Xuu']:.1f} (추력테이블 무관), R^2 {c['r2']:.3f}")
        if transient and transient.get("ok"):
            log.info(
                f"  [과도구간 적합] Xuu = {transient['Xuu']:.1f} "
                f"(정상상태 불필요, 수준 {transient['n_levels']}개 "
                f"표본 {transient['n_samples']}개, 잔차 "
                f"{100 * transient['rms_relative']:.1f}%)")
            log.info(
                f"      m_eff = {transient['m_eff_kg']:.1f} kg "
                f"(모델 가정 20.99 kg) — 잡음에 낮게 치우치므로 자릿수 확인용")
            if not transient.get("xu_identifiable", False):
                log.info("      Xu는 이 적합에서 식별되지 않는다(고정값 사용)")
        elif transient:
            log.warn(f"  [과도구간 적합] 실패 — {transient.get('reason', '?')}")
        if recirc.get("checked") and recirc.get("biased"):
            log.warn(f"  [경고] 재순환 편향 — level {recirc['worst_level']}의 "
                     f"반복 측정이 {100 * recirc['worst_relative_spread']:.0f}% "
                     f"차이. inter_level_wait_s를 2배로 늘려 재측정할 것")
        log.info("=" * 60)


def main(argv=None):
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = DragTestNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("인터럽트 — 중립 정지 후 기록을 저장한다")
        node._finish("사용자 인터럽트")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
