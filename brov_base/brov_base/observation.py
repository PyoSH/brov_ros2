"""
실기체 텔레메트리 → 16-dim 정책 관측 조립
=============================================
배포 policy metadata가 정의하는 수식(q_e, v_e_b, ω_b, z_v, z_q)을 조립한다.
시뮬레이션의 고정-step 관측과 비교해 실기체에서 명시적으로 처리할 차이는 두 가지다:

1. **좌표계**: MAVLink는 NED(X=North,Y=East,Z=Down) world + SNAME 계열 body를 쓰고,
   학습된 정책은 IsaacLab의 Z-up world/body 관측으로 학습됨. `waypoint_frame=ned`는
   NED를 그대로 사용하고, `start_heading`은 시작 yaw만 제거해 초기 전방을 +X로 둔다.
   위치·속도·자세와 waypoint를 같은 frame으로 변환하되 실제 roll/pitch는 유지하고,
   body만 이 코드베이스 전체가 쓰는 `T3=diag(1,-1,-1)` 변환으로 Z-up으로 맞춘다.
   자세(쿼터니언)는 body만 바뀌므로 `q_zup = q_ned ⊗ Q_M`(Q_M=[0,1,0,0], X축 180도
   회전 — body 기저만 바꾸는 우측 곱) 하나로 충분하다. 좌표계 계약은 저장소의
   `docs/ARCHITECTURE.md`에도 요약되어 있다. 비행 전 IMU를 손으로 돌려 observation의
   각 성분과 부호가 예상과 일치하는지 다시 확인할 것.
2. **적분 dt**: sim은 고정 `policy_dt`, 실기체는 텔레메트리 지연/지터가 있어 매 호출
   실측 경과시간을 받는다. 호출부는 MAVLink 수신 timestamp 차이로 `dt`를 계산하고,
   새 telemetry sample에 대해서만 적분한다.

EKF 게이팅: `RealRobotInterface.is_ekf_healthy()`가 False인 프레임은 이 클래스가
아니라 호출부가 걸러야 한다(정책에 먹이지 않고 안전정지) — 여기는 순수 조립만.
"""

from __future__ import annotations

import torch

from brov_base import math_utils as mu

_T3 = torch.tensor([1.0, -1.0, -1.0])
_Q_M = torch.tensor([0.0, 1.0, 0.0, 0.0])   # NED body → Z-up body, X축 180도 회전


class ObservationBuilder:
    def __init__(
        self,
        device: str = "cpu",
        integral_vel_limit: float = 5.0,
        integral_att_limit: float = 5.0,
        waypoint_frame: str = "start_heading",
    ):
        self.device = device
        if integral_vel_limit <= 0 or integral_att_limit <= 0:
            raise ValueError("적분항 limit은 양수여야 함")
        self.integral_vel_limit = float(integral_vel_limit)
        self.integral_att_limit = float(integral_att_limit)
        if waypoint_frame not in {"ned", "start_heading"}:
            raise ValueError("waypoint_frame은 'ned' 또는 'start_heading'이어야 함")
        self.waypoint_frame = waypoint_frame
        self._t3 = _T3.to(device)
        self._q_m = _Q_M.to(device)
        self._z_v = torch.zeros(3, device=device)
        self._z_q = torch.zeros(3, device=device)
        self._origin_ned = None   # 미션 시작 시점 위치 — pos_env 기준점
        self._q_ned_to_mission = mu.identity_quat(1, device).squeeze(0)

    def reset(self, pos_ned: torch.Tensor, att_quat_ned: torch.Tensor | None = None) -> None:
        self.reset_integrators()
        self._origin_ned = pos_ned.clone()
        if self.waypoint_frame == "start_heading":
            if att_quat_ned is None:
                raise ValueError("start_heading reset에는 att_quat_ned가 필요함")
            yaw0 = mu.yaw_from_quat(att_quat_ned)
            zero = torch.zeros_like(yaw0)
            self._q_ned_to_mission = mu.quat_from_euler_xyz(zero, zero, -yaw0)
        else:
            self._q_ned_to_mission = mu.identity_quat(1, self.device).squeeze(0)

    def attitude_in_waypoint_frame(self, att_quat_ned: torch.Tensor) -> torch.Tensor:
        return mu.quat_mul(self._q_ned_to_mission, att_quat_ned)

    def reset_integrators(self) -> None:
        """학습 episode reset과 동일하게 z_v/z_q만 0으로 초기화한다."""
        self._z_v.zero_()
        self._z_q.zero_()

    def _ned_body_to_zup(self, q_ned: torch.Tensor) -> torch.Tensor:
        return mu.quat_mul(q_ned, self._q_m)

    def build(
        self,
        att_quat_ned: torch.Tensor,   # (4,) [w,x,y,z] body→NED-world
        body_rates_ned: torch.Tensor,   # (3,) [p,q,r] NED body frame
        pos_ned: torch.Tensor,   # (3,) NED world
        vel_ned: torch.Tensor,   # (3,) NED world
        guidance,                        # brov_base.guidance.LOSGuidance
        dt: float,
        integrate: bool = True,
        advance_waypoint: bool = True,
    ) -> tuple[torch.Tensor, dict]:
        assert self._origin_ned is not None, "reset()으로 origin을 먼저 잡아야 함"

        q_frame = self.attitude_in_waypoint_frame(att_quat_ned)
        pos_frame = mu.quat_apply(self._q_ned_to_mission, pos_ned - self._origin_ned)
        vel_frame = mu.quat_apply(self._q_ned_to_mission, vel_ned)
        q_b = q_frame.unsqueeze(0)          # (1,4)
        pos_env = pos_frame.unsqueeze(0)   # (1,3)

        # ── guidance는 선택된 waypoint frame에서 계산한다 ──
        v_d_b_ned, q_d_ned = guidance.compute(
            pos_env,
            q_b,
            advance_waypoint=advance_waypoint,
            dt=dt if integrate else 0.0,
            angular_speed_rad_s=body_rates_ned.norm().reshape(1),
        )
        v_d_b_ned = v_d_b_ned.squeeze(0)
        q_d_ned = q_d_ned.squeeze(0)

        return self.build_from_desired(
            q_frame=q_frame,
            body_rates_ned=body_rates_ned,
            vel_frame=vel_frame,
            v_d_b_ned=v_d_b_ned,
            q_d_ned=q_d_ned,
            dt=dt,
            integrate=integrate,
            pos_env=pos_env,
        )

    def build_from_desired(
        self,
        q_frame: torch.Tensor,        # (4,) 현재 자세 [w,x,y,z], NED body→world
        body_rates_ned: torch.Tensor,   # (3,) NED body frame
        vel_frame: torch.Tensor,      # (3,) world frame 선속도
        v_d_b_ned: torch.Tensor,      # (3,) 목표 body 속도 (NED body)
        q_d_ned: torch.Tensor,        # (4,) 목표 자세 (NED)
        dt: float,
        integrate: bool = True,
        pos_env: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict]:
        """유도 객체 없이, 이미 계산된 목표로 관측을 만든다.

        노드 분리 때문에 필요하다 — `build()`는 guidance를 직접 호출하지만,
        `observation_node`는 목표를 **토픽으로** 받는다. 수식은 한 벌만 두고
        진입점만 둘로 나눈다.

        **world frame 선택에 불변이다.** `q_e = conj(q_d) ⊗ q`에서 두 자세에
        같은 프레임 회전 M이 걸리면 `conj(M q_d) ⊗ (M q) = conj(q_d) ⊗ q`로
        소거되고, `v_e_b`는 양쪽 다 body frame이다. 그래서 origin/waypoint
        frame은 **guidance만 소유하면 되고** 관측 노드는 알 필요가 없다.
        (관측 16-D에 위치가 들어가지 않는다는 사실이 이것을 가능하게 한다.)
        """
        # ── 현재 상태: NED → Z-up body ──
        q_zup = self._ned_body_to_zup(q_frame)
        omega_zup = body_rates_ned * self._t3
        v_body_ned = mu.quat_apply(mu.quat_conjugate(q_frame), vel_frame)   # world→body 회전
        v_body_zup = v_body_ned * self._t3

        # ── 목표: NED → Z-up body ──
        q_d_zup = self._ned_body_to_zup(q_d_ned)
        v_d_b_zup = v_d_b_ned * self._t3

        # ── brov_velocity_observation_v2 수식 ──
        # q와 -q는 같은 자세지만 NN observation에서는 전혀 다른 숫자다. MAVLink
        # quaternion의 최초 hemisphere나 yaw ±pi 통과 여부에 따라 q_e가 갑자기
        # -identity 근처가 되지 않도록 scalar-positive 표현으로 고정한다.
        q_e = mu.quat_unique(mu.quat_mul(mu.quat_conjugate(q_d_zup), q_zup))
        v_e_b = v_body_zup - v_d_b_zup
        omega_b = omega_zup

        integrator_clamped = False
        if integrate:
            z_v_next = self._z_v + v_e_b * dt
            z_q_next = self._z_q + q_e[1:4] * dt
            self._z_v = z_v_next.clamp(-self.integral_vel_limit, self.integral_vel_limit)
            self._z_q = z_q_next.clamp(-self.integral_att_limit, self.integral_att_limit)
            integrator_clamped = bool(
                ((self._z_v != z_v_next).any() | (self._z_q != z_q_next).any()).item()
            )

        obs = torch.cat([q_e, v_e_b, omega_b, self._z_v, self._z_q], dim=-1)   # (16,)

        debug = {
            "q_zup": q_zup, "q_d_zup": q_d_zup,
            "v_body_zup": v_body_zup, "v_d_b_zup": v_d_b_zup,
            "q_e": q_e, "v_e_b": v_e_b, "omega_b": omega_b,
            "z_v": self._z_v.clone(), "z_q": self._z_q.clone(),
            # observation_node 경로에서는 위치가 없다 — 관측 16-D에 안 들어가고
            # 프레임 불변성 때문에 필요하지도 않다. 진단용으로만 넣는다.
            "pos_env": None if pos_env is None else pos_env.squeeze(0),
            "integrator_clamped": integrator_clamped,
        }
        return obs, debug
