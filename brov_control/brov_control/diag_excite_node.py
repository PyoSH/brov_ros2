#!/usr/bin/env python3
"""dead time 측정용 여기(excitation) 발생기 — 정책 자리에 알려진 신호를 넣는다.

왜 정책으로 재지 않는가
=======================
`diag_loop_delay` 는 명령과 가속도의 **교차상관**으로 지연을 낸다. 상관을 내려면
명령이 흔들려야 하는데, 정책이 잘 추종하면 순항 중 명령이 거의 일정하다.
그러면 여기는 선회와 (혹시 난다면) 진동에서만 나오고, **진동이 안 나면 측정이
성립하지 않는다.** 우연에 기대는 실험이 된다.

dead time 은 통신·계산·구동에 걸리는 **경로의 성질**이라 무엇이 명령을 만들든
같다. 분리 스택의 절단면이 wrench 이므로(`Wrench6.msg` 참조) 정책 노드를 이
노드로 갈아끼우면 그만이다 -- `base_node` 아래로는 아무것도 달라지지 않는다.
그러면 여기가 보장되고, 정책의 동특성이 추정에 섞이지도 않는다.

무엇을 내보내는가
=================
한 축에만 신호를 싣는다. 나머지 5 성분은 0 이다 -- 축이 섞이면 교차상관이
어느 경로의 지연인지 말해주지 못한다.

    square  주기 사각파. **기본값.** 모서리가 넓은 대역을 한 번에 때려서
            교차상관 봉우리가 가장 뾰족하게 선다. 고정 지연 추정에 최적이다.
    chirp   f0 -> f1 선형 스윕. 주파수별 위상을 보고 싶을 때.

`bias` 는 부력 상쇄용 상수항이다. 사각파는 평균이 0 이라 힘 자체로는 안 밀리지만,
BlueROV2 는 대개 약간 양성부력이라 그대로 두면 60 초 동안 떠오른다. heave 축
측정 전에 기체가 뜨지도 가라앉지도 않는 값을 찾아 넣는다.

안전
====
- launch 는 이 노드를 띄우기만 한다. `base_node` 가 `start_control` 전까지
  wrench 를 **무시**하므로 그때까지 아무 힘도 나가지 않는다.
- 신호 시각은 `/brov/control_active` 의 **상승 에지**에서 0 으로 잡는다.
  노드가 뜬 시점이 아니다 -- 그러면 start 순간의 위상이 매번 달라진다.
- `duration_s` 가 지나면 스스로 0 을 내보내고 멈춘다. 잊고 두어도 계속 흔들지
  않는다. 정지는 `stop_control` 로 한다.
- `max_amplitude_n` / `max_amplitude_nm` 을 넘는 설정은 **띄울 때 거절**한다.

깊이 유지
=========
기체가 음성부력이면 이 노드만 떠 있는 동안 가라앉는다. `depth_hold`(기본 on)가
heave 에 느린 PID 를 걸어 **start 순간의 깊이**를 지킨다. 이득은 heave 진동
문턱(정규화 3.52)의 1/30 이라 이 루프 자체는 떨지 않는다. yaw/surge/sway 여기와는
축이 달라 측정에 섞이지 않는다. 무게를 몰라도 된다 -- 적분항이 맞춘다.
axis 가 heave 면 되먹임이 측정 축에 들어가므로 깊이 유지를 하지 않는다 --
heave 여기는 짧게 돌리고, 지연 측정은 yaw 로 한다.

운용::

    ros2 launch brov_bringup deadtime_test.launch.py axis:=heave send_pwm:=true arm:=true
    ros2 service call /brov/prepare_control std_srvs/srv/Trigger
    ros2 service call /brov/arm_control     std_srvs/srv/Trigger
    ros2 service call /brov/start_control   std_srvs/srv/Trigger
    #   ... duration_s 뒤 자동으로 멈춘다 ...
    ros2 service call /brov/stop_control    std_srvs/srv/Trigger
    ros2 service call /brov/disarm_control  std_srvs/srv/Trigger
"""
from __future__ import annotations

import math

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from brov_interfaces.msg import BrovState, Wrench6


# `diag_loop_delay._AXIS_INDEX` 와 같은 순서다. 분석기와 여기 발생기가 축을
# 다르게 세면 측정이 통째로 무의미해진다.
_LINEAR_AXES = {"surge": 0, "sway": 1, "heave": 2}
_ANGULAR_AXES = {"roll": 0, "pitch": 1, "yaw": 2}


def excitation(
    t: float,
    *,
    kind: str,
    amplitude: float,
    bias: float,
    period_s: float,
    chirp_f0_hz: float,
    chirp_f1_hz: float,
    duration_s: float,
) -> float:
    """시각 `t` 에서의 축 성분 [N] 또는 [N*m]. 순수 함수 -- ROS 없이 시험한다."""
    if t < 0.0 or t >= duration_s:
        return 0.0
    if kind == "square":
        half = 0.5 * period_s
        return bias + (amplitude if (t % period_s) < half else -amplitude)
    if kind == "chirp":
        # 선형 스윕의 순시 위상은 주파수의 적분이다. f(t) = f0 + k t 이므로
        # phase = 2*pi*(f0 t + k t^2 / 2). k 를 duration 에 맞춰 정한다.
        k = (chirp_f1_hz - chirp_f0_hz) / duration_s
        phase = 2.0 * math.pi * (chirp_f0_hz * t + 0.5 * k * t * t)
        return bias + amplitude * math.sin(phase)
    raise ValueError(f"kind={kind!r} — 'square' 또는 'chirp' 여야 한다")


def depth_hold_force(
    depth_error_m: float,
    vertical_velocity_mps: float,
    integral_n: float,
    *,
    kp_n_per_m: float,
    kd_n_per_mps: float,
    bias_n: float,
    max_n: float,
) -> float:
    """느린 깊이 유지 힘 [N, SNAME/FRD: 아래가 +].

    `depth_error_m = z - z_ref` (NED, 아래가 +). 너무 깊으면(+) 위로(-) 민다.
    이 루프도 같은 80 ms 지연을 지나므로 **떨 수 있다** -- 그래서 이득을 문턱
    (heave 정규화 3.52 = 420 N/(m/s)) 의 1/30 아래에 둔다. kd 15 N/(m/s) 는
    정규화 0.125 다. 위치항·적분항은 그 주파수에서 그보다 훨씬 작다.
    """
    force = -(kp_n_per_m * depth_error_m
              + kd_n_per_mps * vertical_velocity_mps
              + integral_n) + bias_n
    return max(-max_n, min(max_n, force))


class ExciteNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "brov_diag_excite", parameter_overrides=parameter_overrides
        )

        self.declare_parameter("axis", "heave")
        self.declare_parameter("kind", "square")
        self.declare_parameter("amplitude", 20.0)
        self.declare_parameter("bias", 0.0)
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("chirp_f0_hz", 0.2)
        self.declare_parameter("chirp_f1_hz", 5.0)
        self.declare_parameter("duration_s", 60.0)
        # base_node 의 wrench watchdog 은 0.25 s 다. 제어 주기와 같은 25 Hz 로
        # 낸다 -- ZOH 위상이 분석기의 control_dt 가정과 일치해야 한다.
        self.declare_parameter("rate_hz", 25.0)
        # 실기에서 넘으면 안 되는 한계. 넘는 설정은 띄울 때 거절한다.
        self.declare_parameter("max_amplitude_n", 35.0)
        self.declare_parameter("max_amplitude_nm", 10.0)
        # ── 깊이 유지 ──
        # 기체가 음성부력이면 이 노드만 떠 있는 동안 바닥에 가라앉는다. 바닥에서
        # 돌면 마찰이 섞여 측정이 무의미하다. 그래서 heave 에 느린 PID 를 건다.
        # 기준 깊이는 control_active 상승 에지의 깊이다(그 자리를 지킨다).
        # axis 가 heave 면 되먹임이 측정 축에 섞이므로 **feedforward(bias)만** 쓴다.
        self.declare_parameter("depth_hold", True)
        self.declare_parameter("depth_kp_n_per_m", 20.0)
        self.declare_parameter("depth_kd_n_per_mps", 15.0)
        self.declare_parameter("depth_ki_n_per_m_s", 2.0)
        self.declare_parameter("depth_bias_n", 0.0)      # 순중량 상쇄, 위로 = 음수
        self.declare_parameter("depth_hold_max_n", 15.0)
        # start 깊이에서 몇 m 띄워 유지할지 (양수 = 위). 음성부력 기체는 start
        # 순간 바닥에 있으므로 0 이면 바닥을 "유지" 해 버린다.
        self.declare_parameter("rise_m", 0.0)

        p = self.get_parameter
        self._axis = str(p("axis").value).strip().lower()
        if self._axis in _LINEAR_AXES:
            self._index, self._angular = _LINEAR_AXES[self._axis], False
            limit = float(p("max_amplitude_n").value)
            unit = "N"
        elif self._axis in _ANGULAR_AXES:
            self._index, self._angular = _ANGULAR_AXES[self._axis], True
            limit = float(p("max_amplitude_nm").value)
            unit = "N*m"
        else:
            raise ValueError(
                f"axis={self._axis!r} — "
                f"{sorted(_LINEAR_AXES) + sorted(_ANGULAR_AXES)} 중 하나여야 한다")

        self._kind = str(p("kind").value).strip().lower()
        if self._kind not in ("square", "chirp"):
            raise ValueError(f"kind={self._kind!r} — 'square' 또는 'chirp'")
        self._amplitude = float(p("amplitude").value)
        self._bias = float(p("bias").value)
        self._period_s = float(p("period_s").value)
        self._f0 = float(p("chirp_f0_hz").value)
        self._f1 = float(p("chirp_f1_hz").value)
        self._duration_s = float(p("duration_s").value)
        rate = float(p("rate_hz").value)

        for name, value in (("amplitude", self._amplitude),
                            ("period_s", self._period_s),
                            ("duration_s", self._duration_s),
                            ("rate_hz", rate)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} 은 유한한 양수여야 한다 (받은 값 {value})")
        # 진폭과 bias 를 **합쳐서** 본다. 따로 보면 둘을 나눠 담아 한계를 우회할
        # 수 있고, 기체가 받는 것은 합이다.
        peak = abs(self._amplitude) + abs(self._bias)
        if peak > limit:
            raise ValueError(
                f"|amplitude| + |bias| = {peak:.1f} {unit} 가 한계 "
                f"{limit:.1f} {unit} 를 넘는다")
        if self._kind == "chirp":
            nyquist = 0.5 * rate
            if not 0.0 < self._f0 < self._f1:
                raise ValueError("chirp 는 0 < f0 < f1 이어야 한다")
            if self._f1 >= nyquist:
                raise ValueError(
                    f"chirp_f1_hz {self._f1} 가 Nyquist {nyquist} 이상이다 — "
                    "그 위는 재구성되지 않는다")

        self._depth_hold = bool(p("depth_hold").value)
        self._depth_kp = float(p("depth_kp_n_per_m").value)
        self._depth_kd = float(p("depth_kd_n_per_mps").value)
        self._depth_ki = float(p("depth_ki_n_per_m_s").value)
        self._depth_bias = float(p("depth_bias_n").value)
        self._depth_max = float(p("depth_hold_max_n").value)
        self._rise = float(p("rise_m").value)
        if not math.isfinite(self._rise) or not 0.0 <= self._rise <= 0.6:
            raise ValueError(f"rise_m={self._rise!r} — 0 이상 0.6 이하 (수조 깊이 여유)")
        for name, value in (("depth_kp_n_per_m", self._depth_kp),
                            ("depth_kd_n_per_mps", self._depth_kd),
                            ("depth_ki_n_per_m_s", self._depth_ki),
                            ("depth_hold_max_n", self._depth_max)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} 은 유한한 0 이상이어야 한다")
        # 되먹임 이득이 heave 문턱을 넘으면 이 루프가 먼저 떤다. 정규화 kd 가
        # 문턱 3.52 의 1/10 을 넘는 설정은 거절한다 -- "느린" 루프여야 한다.
        if self._depth_kd / 120.0 > 0.35:
            raise ValueError(
                f"depth_kd_n_per_mps={self._depth_kd} 는 정규화 {self._depth_kd/120:.2f} "
                "-- 문턱 3.52 의 1/10(0.35)을 넘는다. 이 루프는 느려야 한다")
        self._depth_feedback = self._depth_hold and not (
            self._axis == "heave" and not self._angular)
        if self._depth_hold and not self._depth_feedback:
            self.get_logger().warn(
                "axis=heave 라 깊이 유지를 끈다 (측정 축에 섞인다). 짧게 돌릴 것")
        self._depth_ref = None
        self._depth_int = 0.0
        self._z = None
        self._vz = None
        self._state_valid = False

        self._pub = self.create_publisher(Wrench6, "/brov/cmd/wrench", 1)
        self._active = False
        self._t0 = None
        self._finished = False
        self._seq = 0
        self._rate = rate
        self.create_subscription(
            Bool, "/brov/control_active", self._on_active, 1)
        self.create_subscription(BrovState, "/brov/state", self._on_state, 1)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"여기 발생기 대기 — axis={self._axis} kind={self._kind} "
            f"amplitude={self._amplitude} bias={self._bias} "
            f"{'period=' + str(self._period_s) + 's' if self._kind == 'square' else f'{self._f0}->{self._f1} Hz'} "
            f"duration={self._duration_s}s. "
            "start_control 이 있어야 신호가 시작된다")

    def _on_active(self, message: Bool) -> None:
        active = bool(message.data)
        if active and not self._active:
            # 신호 시각의 원점은 **여기**다. 노드가 뜬 시점으로 잡으면 start
            # 순간의 위상이 매번 달라져 주행 간 비교가 안 된다.
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            self._finished = False
            # NED 는 아래가 + 라 위로 띄우는 것은 빼는 것이다.
            self._depth_ref = None if self._z is None else self._z - self._rise
            self._depth_int = 0.0
            self.get_logger().info(
                "제어 활성 — 여기 시작"
                + (f", start 깊이 {self._z:+.2f} m 에서 {self._rise:.2f} m 띄워 유지"
                   if self._depth_hold and self._z is not None else ""))
        elif not active and self._active:
            self._t0 = None
            self._depth_ref = None
            self.get_logger().info("제어 비활성 — 여기 정지")
        self._active = active

    def _on_state(self, msg: BrovState) -> None:
        self._state_valid = bool(msg.valid)
        if msg.valid:
            self._z = float(msg.position.z)
            self._vz = float(msg.linear_velocity.z)

    def _depth_hold_force(self) -> float:
        """깊이 유지 힘. 상태가 없거나 무효면 feedforward 만 -- 모르는 채로 되먹이지 않는다."""
        if not self._depth_hold or not self._active:
            return 0.0
        if (not self._depth_feedback or self._depth_ref is None
                or self._z is None or not self._state_valid):
            return max(-self._depth_max, min(self._depth_max, self._depth_bias))
        error = self._z - self._depth_ref
        self._depth_int += self._depth_ki * error / self._rate
        self._depth_int = max(-self._depth_max, min(self._depth_max, self._depth_int))
        return depth_hold_force(
            error, self._vz or 0.0, self._depth_int,
            kp_n_per_m=self._depth_kp, kd_n_per_mps=self._depth_kd,
            bias_n=self._depth_bias, max_n=self._depth_max)

    def _tick(self) -> None:
        value = 0.0
        if self._active and self._t0 is not None:
            t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
            if t >= self._duration_s and not self._finished:
                self._finished = True
                self.get_logger().info(
                    f"여기 종료 ({self._duration_s:.0f}s). 중립을 유지한다 — "
                    "stop_control / disarm_control 로 내릴 것")
            value = excitation(
                t, kind=self._kind, amplitude=self._amplitude,
                bias=self._bias, period_s=self._period_s,
                chirp_f0_hz=self._f0, chirp_f1_hz=self._f1,
                duration_s=self._duration_s)

        message = Wrench6()
        message.header.stamp = self.get_clock().now().to_msg()
        message.seq = self._seq
        self._seq += 1
        force = [0.0, 0.0, 0.0]
        torque = [0.0, 0.0, 0.0]
        (torque if self._angular else force)[self._index] = value
        # 깊이 유지는 heave 에 **더한다.** axis 가 heave 면 feedforward 만 들어온다.
        force[2] += self._depth_hold_force()
        message.force = Vector3(x=force[0], y=force[1], z=force[2])
        message.torque = Vector3(x=torque[0], y=torque[1], z=torque[2])
        # 생성자 식별. bag 을 나중에 볼 때 정책 주행과 여기 주행을 구분하는
        # 유일한 단서다 -- 파일 이름은 믿을 것이 못 된다.
        message.source = "diag"
        self._pub.publish(message)


def main() -> None:
    rclpy.init()
    node = ExciteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
