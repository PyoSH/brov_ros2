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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
import torch

from brov_interfaces.msg import BrovState, Wrench6

from brov_base.mavlink_interface import (
    RealRobotInterface,
    thruster_reversal_sign_for_profile,
)
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix


_MANUAL_CUSTOM_MODE = 19


class BaseNode(Node):
    """MAVLink 링크, 액추에이터, 안전을 단독 소유하는 노드."""

    def __init__(self, interface=None) -> None:
        """``interface``를 주입하면 MAVLink 없이 구동한다.

        watchdog·fault latch·할당은 이 노드가 새로 떠안은 안전 책임이고,
        실기 없이 검증할 수 있어야 한다. 기본값 None이면 실제 링크를 연다.
        """
        super().__init__("brov_base")

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
        if interface is None:
            self._interface = RealRobotInterface(
                conn,
                thruster_reversal_sign=thruster_reversal_sign_for_profile(profile, conn),
            )
            self._interface.connect()
            self.get_logger().info(f"MAVLink {conn}, reversal profile {profile}")
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

        period = 1.0 / float(p("state_rate_hz").value)
        self.create_timer(period, self._tick)
        limits = (f"선형 ±{self._gz_half_range:.1f} N"
                  if self._thruster_model == "gazebo_linear"
                  else f"T200 테이블 {self._thruster.force_limits_n}")
        self.get_logger().info(
            f"base_node 시작 — send_pwm={self._send_pwm} arm={self._arm_permitted} "
            f"watchdog={self._cmd_timeout:.3f}s, 추진기 {self._thruster_model} ({limits})"
        )

    # ------------------------------------------------------------------ 상태
    def _read_state(self) -> BrovState | None:
        snap = self._interface.snapshot()
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

    def _tick(self) -> None:
        msg = self._read_state()
        if msg is not None:
            self._pub_state.publish(msg)
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
