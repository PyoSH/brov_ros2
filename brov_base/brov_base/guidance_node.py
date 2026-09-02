#!/usr/bin/env python3
"""brov_guidance_node — 경로를 목표로 바꾼다.

역할 분리에서 이 노드가 소유하는 것
====================================
**"어디로 가야 하는가"만** 소유한다. 로봇을 만지지 않고, 오차를 계산하지 않고,
적분을 갖지 않는다.

  sub  /brov/state                   BrovState
  sub  /brov/control_active          Bool
  sub  /brov/localization/status     LocalizationStatus  (waypoint_frame=pool 만)
  pub  /brov/desired    DesiredState  (v_d_b, q_d, waypoint_index, mission_complete)

**step_3에서 LOSGuidance가 NBV/커버리지 모듈로 교체되는 지점이 정확히 여기다.**
그때 바꾸는 것은 이 파일 하나이고, 관측·정책·base는 무변경이다. 그것이 유도를
별도 노드로 뽑는 가장 큰 이유다.

유도 법칙
=========
`brov_base/guidance.py`의 `LOSGuidance` — Sim2Swim이 인용한
Breivik & Fossen (2005) Sec. IV의 3D LOS다. 경로 고정 방위/앙각에 cross-track과
vertical-track 오차의 lookahead 보정을 **독립적으로** 더하므로
`||v_d_b|| = cruise_speed`가 정확히 보존된다 — 정책이 학습한 명령 크기는
그 값 하나뿐이라 이 보존이 계약이다.

아직 여기로 옮기지 않은 것
==========================
`obs_node.py`의 ResolvedMission 처리는 검증 로직이 370줄이 넘는다(경계 상자,
세그먼트 길이, 랜덤 자세 slew/dwell, 미션 시간/랩 한계). 이 노드는 **한계값
검사만 승계**하고 파라미터 waypoint 경로를 먼저 지원한다. 전체 이관은 obs_node를
퇴역시킬 때 한다 — 그 전까지 둘을 **동시에 띄우면 안 된다**(목표가 두 벌 나온다).
"""

from __future__ import annotations

from geometry_msgs.msg import Quaternion, Vector3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
import torch

from brov_interfaces.msg import BrovState, DesiredState, LocalizationStatus

from brov_base import math_utils as mu
from brov_base.guidance import LOSGuidance, RandomAttitudeConfig
from brov_base.mission import pool_to_mission_quaternion


# NED(Z-down) <-> pool/odom(Z-up) 기저 변환. 둘 다 X 축 180° 회전이라 자기
# 자신이 역이다. odometry.py 의 ``_S_DIAGONAL`` 과 같은 값이고, 같은 이유로
# 쿼터니언에서는 wxyz 성분에 (1, 1, -1, -1) 을 곱하는 것과 같다.
_S = (1.0, -1.0, -1.0)


def _parse_waypoints(text: str) -> torch.Tensor:
    """``"0,0,0;3,0,0"`` -> (1, N, 3)."""
    pts = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        xyz = [float(v) for v in chunk.split(",")]
        if len(xyz) != 3:
            raise ValueError(f"waypoint {chunk!r}는 x,y,z 3개여야 한다")
        pts.append(xyz)
    if len(pts) < 2:
        raise ValueError("waypoint가 2개 미만이면 세그먼트를 만들 수 없다")
    return torch.tensor([pts], dtype=torch.float32)


class GuidanceNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "brov_guidance", parameter_overrides=parameter_overrides
        )

        self.declare_parameter("waypoints", "0,0,0;3,0,0")
        self.declare_parameter("cruise_speed", 0.2)
        self.declare_parameter("heading_mode", "straight")
        # heading_mode="random_at_waypoint" 전용. 배선하지 않으면 LOSGuidance 의
        # _random_q_d 가 identity 로 고정돼 case (c) 가 조용히 "upright" 로
        # 퇴화한다 -- 실패가 아니라 **다른 실험**이 되므로 반드시 채워야 한다.
        # 이름은 mission_manager_sim2swim_c.yaml 과 같게 둔다.
        self.declare_parameter("random_attitude_seed", 20260814)
        self.declare_parameter("random_attitude_reference_frame", "pool_zup_flu")
        self.declare_parameter("random_attitude_generator_version",
                               "sha256_counter_uniform_rpy_v1")
        self.declare_parameter("random_attitude_rpy_min_rad",
                               [-0.2617993877991494, -0.2617993877991494,
                                -0.5235987755982988])
        self.declare_parameter("random_attitude_rpy_max_rad",
                               [0.2617993877991494, 0.2617993877991494,
                                0.5235987755982988])
        self.declare_parameter("random_attitude_max_slew_rate_rad_s", 0.17453292519943295)
        self.declare_parameter("random_attitude_tolerance_rad", 0.17453292519943295)
        self.declare_parameter("random_attitude_angular_speed_tolerance_rad_s",
                               0.08726646259971647)
        self.declare_parameter("random_attitude_dwell_time_s", 1.0)
        self.declare_parameter("random_attitude_max_duration_s", 60.0)
        self.declare_parameter("random_attitude_max_laps", 1)
        self.declare_parameter("reach_threshold", 0.15)
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("lookahead_vert", 0.0)     # 0이면 수평값과 동일
        self.declare_parameter("loop", False)
        self.declare_parameter("terminal_hold_kp", 0.5)
        self.declare_parameter("terminal_speed_limit", 0.1)
        # 운용자가 준 파라미터에 대한 **온전성 검사**다. 조용히 clamp하면 설정
        # 실수가 수조에서 드러나므로, 넘으면 아예 뜨지 않는다.
        #
        # obs_node의 max_resolved_* 와는 대상이 다르다 — 그쪽은 **외부에서 들어오는
        # ResolvedMission**(신뢰할 수 없는 입력)의 경계이고, 이쪽은 launch 인자다.
        # 기본값은 정본 미션(mission_sim2sim_mk2_case_a_0p5.yaml: cruise 0.50,
        # lookahead 0.40, 세그먼트 2.0 m)을 통과시키도록 잡는다. ResolvedMission
        # 경로를 이 노드로 옮길 때는 그 입력에 별도의(더 엄격한) 경계가 필요하다.
        self.declare_parameter("max_waypoints", 50)
        self.declare_parameter("max_segment_length_m", 4.0)
        self.declare_parameter("max_cruise_speed", 0.55)
        self.declare_parameter("max_lookahead_dist", 1.0)
        self.declare_parameter("max_reach_threshold", 0.50)
        self.declare_parameter("state_max_age_s", 0.2)
        # waypoint를 어느 프레임에서 읽을지. obs_node의 기본값과 같다.
        #   start_heading — 시작 위치를 원점, 시작 yaw를 +X로 삼는 프레임.
        #                   "0,0,0;3,0,0"이 "기체 정면으로 3 m"를 뜻한다.
        #   ned           — 원시 world 좌표 그대로.
        #   pool          — 마커(ArUco) 정렬이 세운 **수조 절대 프레임**.
        #                   Z-up 이고 단위는 m 다. "0.6,0.85,0.7;3.1,0.85,0.7"
        #                   이 수조 바닥 기준 실제 좌표를 뜻하므로, 기체를 어디에
        #                   놓았든 벽까지의 여유가 보장된다. start_heading 은
        #                   그것이 보장되지 않는다 -- 배치가 틀리면 경로가
        #                   통째로 벽 밖으로 나간다.
        # 이 구분이 없으면 기체가 90° yaw로 떠 있을 때 정면 주행 미션이
        # 횡방향 주행이 된다(실제 SITL에서 v_d_b가 body -Y로 나왔다).
        self.declare_parameter("waypoint_frame", "start_heading")
        # pool 프레임 전용. 정렬 상태가 이보다 오래됐으면 목표를 내지 않는다.
        # localization_node 의 status 주기는 0.5 s 다.
        self.declare_parameter("pool_status_max_age_s", 2.0)

        p = self.get_parameter
        wps = _parse_waypoints(str(p("waypoints").value))
        speed = float(p("cruise_speed").value)
        lookahead = float(p("lookahead_dist").value)
        reach = float(p("reach_threshold").value)
        self._state_max_age = float(p("state_max_age_s").value)
        self._pool_status_max_age = float(p("pool_status_max_age_s").value)
        self._waypoint_frame = str(p("waypoint_frame").value)
        if self._waypoint_frame not in ("start_heading", "ned", "pool"):
            raise ValueError(
                f"waypoint_frame={self._waypoint_frame!r} — "
                "'start_heading', 'ned' 또는 'pool'이어야 한다")
        if self._waypoint_frame == "pool":
            # 미션 프레임의 규약은 NED(Z-down) 다 -- LOSGuidance 도, 아래에서
            # state 를 회전시키는 식도 그것을 전제한다. pool 좌표(Z-up)를
            # 여기서 한 번 바꿔 두면 나머지 경로는 손댈 것이 없다. S 는 상수라
            # 정렬 결과를 기다릴 필요가 없다.
            wps = wps * torch.tensor(_S, dtype=wps.dtype)

        self._check_limits(wps, speed, lookahead, reach)

        vert = float(p("lookahead_vert").value)
        self._random_cfg = None
        if str(p("heading_mode").value) == "random_at_waypoint":
            self._random_cfg = RandomAttitudeConfig(
                seed=int(p("random_attitude_seed").value),
                reference_frame=str(p("random_attitude_reference_frame").value),
                generator_version=str(p("random_attitude_generator_version").value),
                rpy_min_rad=tuple(float(x) for x in p("random_attitude_rpy_min_rad").value),
                rpy_max_rad=tuple(float(x) for x in p("random_attitude_rpy_max_rad").value),
                max_slew_rate_rad_s=float(p("random_attitude_max_slew_rate_rad_s").value),
                attitude_tolerance_rad=float(p("random_attitude_tolerance_rad").value),
                angular_speed_tolerance_rad_s=float(
                    p("random_attitude_angular_speed_tolerance_rad_s").value),
                dwell_time_s=float(p("random_attitude_dwell_time_s").value),
                max_duration_s=float(p("random_attitude_max_duration_s").value),
                max_laps=int(p("random_attitude_max_laps").value),
            )
        # pool(Z-up/FLU) 난수 자세를 guidance 의 NED 규약으로 옮기는 회전.
        # 규약을 여기서 새로 쓰지 않고 mission.pool_to_mission_quaternion 을
        # 그대로 쓴다 -- 두 곳에 두면 언젠가 갈라진다. SITL 에는 pool 측량이
        # 없으므로 pool==odom (identity) 이고, 헬퍼가 odom->NED 의
        # diag(1,-1,-1) 과 start-heading yaw 제거를 담당한다.
        # waypoint_frame="start_heading" 이면 start 자세에 의존하므로
        # 아래 _on_state 의 mission frame 확정 시점에 다시 계산한다.
        self._pool_to_mission_q = None
        if self._random_cfg is not None:
            self._pool_to_mission_q = pool_to_mission_quaternion(
                (0.0, 0.0, 0.0, 1.0),
                torch.tensor([1.0, 0.0, 0.0, 0.0]),
                self._waypoint_frame,
            )
        self._los = LOSGuidance(
            wps, "cpu",
            lookahead_dist=lookahead,
            cruise_speed=speed,
            reach_threshold=reach,
            heading_mode=str(p("heading_mode").value),
            loop=bool(p("loop").value),
            terminal_hold_kp=float(p("terminal_hold_kp").value),
            terminal_speed_limit=float(p("terminal_speed_limit").value),
            random_attitude_config=self._random_cfg,
            pool_to_mission_quaternion=self._pool_to_mission_q,
            **({"lookahead_vert": vert} if vert > 0.0 else {}),
        )
        self._seq = 0
        # LOSGuidance는 heading_mode="straight"/"takeoff_then_align"에서 기준 자세를
        # reset(initial_quat=...) 시점에 잡는다. 호출하지 않으면 identity가 남아,
        # 기체가 90° yaw로 떠 있으면 목표 자세가 identity가 되어 **자세 오차 90°**로
        # 시작한다(실제 SITL에서 z_q 포화 + yaw 토크 -22 N·m 포화로 드러났다).
        # 첫 유효 state에서 한 번 잡는다.
        self._initialized = False
        # NED -> mission frame 회전과 원점. start_heading일 때만 identity가 아니다.
        self._q_ned_to_mission = mu.identity_quat(1, "cpu")
        self._origin_ned = torch.zeros(1, 3)
        self._pub = self.create_publisher(DesiredState, "/brov/desired", 10)
        self.create_subscription(BrovState, "/brov/state", self._on_state, 1)
        # waypoint_frame=start_heading 의 원점은 **start 순간**의 위치와 yaw 다
        # (legacy `/brov/start_control` 의미). 첫 유효 상태에서 잡으면 노드가
        # 뜨자마자 -- 아직 arm 도 start 도 하기 전에 -- 원점이 굳어버린다.
        # observation_node 가 적분을 같은 신호로 gate 하므로 기준이 일치한다.
        self._control_active = False
        self._last_state_wall = None      # random_at_waypoint 의 dwell 적산용 dt
        self.create_subscription(
            Bool, "/brov/control_active", self._on_active, 1)
        # pool 정렬 상태. one-shot 정렬이라 topic 은 latched(TRANSIENT_LOCAL)다 --
        # 이 노드가 늦게 떠도 마지막 상태를 받는다.
        self._pool_status: LocalizationStatus | None = None
        self._pool_status_wall = None
        self._locked_alignment: tuple[int, str] | None = None
        self._pool_gate_reason = ""
        if self._waypoint_frame == "pool":
            self.create_subscription(
                LocalizationStatus, "/brov/localization/status",
                self._on_pool_status,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        self.get_logger().info(
            f"guidance_node 시작 — waypoint {wps.shape[1]}개, "
            f"cruise {speed} m/s, heading {p('heading_mode').value}, "
            f"frame {self._waypoint_frame}"
        )

    def _check_limits(self, wps, speed, lookahead, reach) -> None:
        p = self.get_parameter
        n = wps.shape[1]
        if n > int(p("max_waypoints").value):
            raise ValueError(f"waypoint {n}개 > 한계 {p('max_waypoints').value}")
        seg = (wps[0, 1:] - wps[0, :-1]).norm(dim=-1)
        longest = float(seg.max()) if len(seg) else 0.0
        if longest > float(p("max_segment_length_m").value):
            raise ValueError(
                f"세그먼트 {longest:.2f} m > 한계 {p('max_segment_length_m').value} m")
        for value, key in ((speed, "max_cruise_speed"),
                           (lookahead, "max_lookahead_dist"),
                           (reach, "max_reach_threshold")):
            if value > float(p(key).value):
                raise ValueError(f"{key.replace('max_','')} {value} > 한계 {p(key).value}")

    def _to_mission_quat(self, quat_ned: torch.Tensor) -> torch.Tensor:
        return mu.quat_mul(self._q_ned_to_mission, quat_ned)

    def _on_active(self, msg: Bool) -> None:
        """false -> true 순간에 mission frame 을 다시 잡는다."""
        active = bool(msg.data)
        if active and not self._control_active:
            self._initialized = False
            self._locked_alignment = None
            self.get_logger().info("제어 시작 — mission frame 재확정 대기")
        self._control_active = active

    # ------------------------------------------------------------ pool 정렬
    def _on_pool_status(self, msg: LocalizationStatus) -> None:
        self._pool_status = msg
        self._pool_status_wall = self.get_clock().now().nanoseconds * 1e-9

    def _usable_pool_status(self) -> LocalizationStatus | None:
        """쓸 수 있는 정렬이면 status 를, 아니면 사유를 남기고 None 을 낸다."""
        status = self._pool_status
        if status is None:
            self._pool_gate_reason = "/brov/localization/status 미수신"
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - self._pool_status_wall
        if age > self._pool_status_max_age:
            self._pool_gate_reason = f"정렬 상태가 {age:.1f}s 째 갱신되지 않았다"
            return None
        if status.state != LocalizationStatus.INITIALIZED:
            self._pool_gate_reason = (
                f"정렬 미완료(state={status.state}): {status.reason}")
            return None
        if not status.output_valid:
            self._pool_gate_reason = f"정렬 출력 무효: {status.reason}"
            return None
        current = (int(status.epoch), str(status.alignment_id))
        if self._locked_alignment is not None and current != self._locked_alignment:
            # 주행 중 정렬이 갈렸다. **같은 절대 좌표계가 아니다** -- 여기서
            # 조용히 새 정렬로 갈아타면 벽까지 남은 거리가 통째로 바뀐다.
            self._pool_gate_reason = (
                f"정렬이 주행 중 바뀌었다 {self._locked_alignment} -> {current}")
            return None
        return status

    def _pool_mission_frame(
        self, status: LocalizationStatus
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """``status.pool_to_odom`` 에서 (q_ned_to_mission, origin_ned) 를 만든다.

        ``pool_to_odom`` 은 이름과 달리 **odom 좌표를 pool 로 옮기는** 변환
        ``T_pool_odom`` 이다(localization_node 가 ``self._alignment`` 를 그대로
        싣고, 같은 값을 pool->odom TF 로도 방송한다). 세 관계를 이으면 된다::

            p_odom    = S p_ned                     (S = diag(1, -1, -1))
            p_pool    = R_A p_odom + t_A            (T_pool_odom)
            p_mission = S p_pool                    (미션 프레임은 NED 규약)

        합치면 ``p_mission = (S R_A S) p_ned + S t_A`` 이므로

            R_nm        = S R_A S      -> 쿼터니언 wxyz 에 (1, 1, -1, -1)
            origin_ned  = -R_nm^T S t_A = -S (R_A^T t_A)

        (S S = I 를 썼다. ``quat_apply(q_nm, p - origin)`` 형태에 맞춘 것이다.)
        """
        rotation = status.pool_to_odom.rotation
        translation = status.pool_to_odom.translation
        q_a = torch.tensor(
            [[rotation.w, rotation.x, rotation.y, rotation.z]],
            dtype=torch.float32)
        t_a = torch.tensor(
            [[translation.x, translation.y, translation.z]], dtype=torch.float32)
        norm = torch.linalg.vector_norm(q_a, dim=-1, keepdim=True)
        if not torch.isfinite(norm).all() or float(norm.min()) <= 1e-6:
            raise ValueError(f"pool_to_odom 회전이 쿼터니언이 아니다: {q_a.tolist()}")
        # conj == inv 는 단위 쿼터니언에서만 참이다. 아니면 origin 이 배율만큼
        # 어긋나고, 그 오차는 수조 좌표 전체에 그대로 실린다.
        q_a = q_a / norm
        signs = torch.tensor(_S, dtype=torch.float32)

        q_ned_to_mission = q_a * torch.tensor(
            [[1.0, 1.0, -1.0, -1.0]], dtype=torch.float32)
        origin_ned = -mu.quat_apply(mu.quat_conjugate(q_a), t_a) * signs
        return q_ned_to_mission, origin_ned

    def _on_state(self, state: BrovState) -> None:
        out = DesiredState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.seq = self._seq
        self._seq += 1

        if not state.valid or max(state.attitude_age_s, state.position_age_s) > self._state_max_age:
            # 상태가 신뢰할 수 없으면 목표를 만들지 않는다. 관측 노드가 이
            # 침묵을 stale로 보고, 정책은 명령을 내지 않으며, base watchdog이
            # 중립 정지한다 — 실패가 안전한 방향으로 전파된다.
            return

        pos = torch.tensor([[state.position.x, state.position.y, state.position.z]],
                           dtype=torch.float32)
        quat = torch.tensor([[state.attitude.w, state.attitude.x,
                              state.attitude.y, state.attitude.z]], dtype=torch.float32)

        if self._waypoint_frame == "pool":
            # 절대 프레임 주행은 정렬이 살아 있는 동안에만 성립한다. 침묵하면
            # 정책이 명령을 못 내고 base watchdog 이 0.25 s 안에 중립 정지한다 --
            # 정렬이 무효인 채로 계속 미는 것보다 안전하다.
            status = self._usable_pool_status()
            if status is None:
                self.get_logger().warn(
                    f"pool 정렬을 쓸 수 없어 목표를 내지 않는다 — "
                    f"{self._pool_gate_reason}", throttle_duration_sec=2.0)
                return

        if not self._initialized:
            self._origin_ned = pos.clone()
            if self._waypoint_frame == "start_heading":
                yaw0 = mu.yaw_from_quat(quat)
                zero = torch.zeros_like(yaw0)
                self._q_ned_to_mission = mu.quat_from_euler_xyz(zero, zero, -yaw0)
                self.get_logger().info(
                    f"mission frame 확정 — 원점 {pos[0].tolist()}, "
                    f"yaw0 {float(yaw0[0]) * 57.2958:.1f}°")
            elif self._waypoint_frame == "pool":
                try:
                    self._q_ned_to_mission, self._origin_ned = (
                        self._pool_mission_frame(status))
                except ValueError as exc:
                    # 콜백에서 던지면 노드가 죽는다. 게이트처럼 침묵한다 --
                    # 결과는 같고(watchdog 중립 정지), 사유는 로그에 남는다.
                    self._pool_gate_reason = str(exc)
                    self.get_logger().error(
                        f"pool 정렬을 프레임으로 못 바꾼다 — {exc}",
                        throttle_duration_sec=2.0)
                    return
                self._locked_alignment = (
                    int(status.epoch), str(status.alignment_id))
                here = mu.quat_apply(
                    self._q_ned_to_mission, pos - self._origin_ned)
                self.get_logger().info(
                    f"mission frame = pool (epoch {status.epoch}, "
                    f"alignment {status.alignment_id}) — 현재 수조 좌표 "
                    f"{[round(v, 3) for v in (here * torch.tensor(_S))[0].tolist()]}")
            else:
                self.get_logger().info("waypoint_frame=ned — 원시 world 좌표를 쓴다")
            if self._random_cfg is not None and self._waypoint_frame == "start_heading":
                # start 자세가 정해져야 계산되는 값이다 (yaw 제거항이 들어간다).
                self._pool_to_mission_q = pool_to_mission_quaternion(
                    (0.0, 0.0, 0.0, 1.0), quat[0], self._waypoint_frame)
                self._los._pool_to_mission_q = self._pool_to_mission_q
            # LOSGuidance의 기준 자세도 **mission frame에서** 잡아야 한다.
            self._los.reset(torch.tensor([0]),
                            initial_quat=self._to_mission_quat(quat))
            self._initialized = True

        # 유도는 mission frame에서 계산한다.
        pos_m = mu.quat_apply(self._q_ned_to_mission, pos - self._origin_ned)
        quat_m = self._to_mission_quat(quat)
        # random_at_waypoint 는 "자세가 목표에 들어왔고 각속도가 잦아들었을 때"
        # dwell 을 세므로 각속도가 필요하다. state 의 body 각속도를 그대로 준다.
        extra = {}
        if self._random_cfg is not None:
            omega = torch.tensor([[state.angular_velocity.x,
                                   state.angular_velocity.y,
                                   state.angular_velocity.z]], dtype=torch.float32)
            extra["angular_speed_rad_s"] = omega.norm(dim=-1)
            now_wall = self.get_clock().now().nanoseconds * 1e-9
            extra["dt"] = (0.0 if self._last_state_wall is None
                           else max(0.0, now_wall - self._last_state_wall))
            self._last_state_wall = now_wall
        v_d_b, q_d_m = self._los.compute(pos_m, quat_m, **extra)
        # q_d는 state와 **같은 프레임(NED)**으로 되돌려 보낸다. observation_node가
        # q_e = conj(q_d) x q 를 계산하는데, 두 자세의 프레임이 다르면 그 오차에
        # mission 회전이 통째로 섞인다. v_d_b는 body frame이라 변환이 없다.
        q_d = mu.quat_mul(mu.quat_conjugate(self._q_ned_to_mission), q_d_m)

        out.velocity_body = Vector3(x=float(v_d_b[0, 0]), y=float(v_d_b[0, 1]),
                                    z=float(v_d_b[0, 2]))
        out.attitude = Quaternion(w=float(q_d[0, 0]), x=float(q_d[0, 1]),
                                  y=float(q_d[0, 2]), z=float(q_d[0, 3]))
        out.waypoint_index = int(self._los._wp_idx[0])
        out.mission_complete = bool(getattr(self._los, "mission_complete", torch.zeros(1))[0])
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
