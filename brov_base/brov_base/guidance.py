"""
LOSGuidance — IsaacLab 비의존 배포 구현
========================================
`brov_base.math_utils`와 torch만 사용하는 실기체 runtime의 LOS guidance 정본이다.
학습 저장소와 코드를 import하지 않으며, 정책과 guidance 계약 변경은 artifact
metadata/version과 함께 관리한다.

이 파일의 world frame은 호출자가 선택한다. 실기체에서는 NED 또는 제어 시작 yaw를
제거한 start-heading frame을 `ObservationBuilder`가 일관되게 전달한다. body frame
Z-up 변환 역시 `ObservationBuilder`가 이 함수의 출력에 대해 별도로 수행한다.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math

import torch

from brov_base import math_utils as mu


_RANDOM_GENERATOR_VERSION = "sha256_counter_uniform_rpy_v1"
_POOL_ATTITUDE_FRAME = "pool_zup_flu"


@dataclass(frozen=True)
class RandomAttitudeConfig:
    """Versioned random-attitude behavior consumed by pool mission v2.

    Random samples are absolute ``pool`` Z-up / FLU attitudes.  The caller
    supplies the fixed quaternion which maps those samples into the legacy
    guidance world/body convention used by :class:`ObservationBuilder`.
    """

    seed: int
    reference_frame: str
    generator_version: str
    rpy_min_rad: tuple[float, float, float]
    rpy_max_rad: tuple[float, float, float]
    max_slew_rate_rad_s: float
    attitude_tolerance_rad: float
    angular_speed_tolerance_rad_s: float
    dwell_time_s: float
    max_duration_s: float
    max_laps: int

    def validate(self) -> None:
        if (
            isinstance(self.seed, bool)
            or not isinstance(self.seed, int)
            or not 0 <= self.seed <= (1 << 63) - 1
        ):
            raise ValueError("random attitude seed must be in [0, 2^63-1]")
        if self.reference_frame != _POOL_ATTITUDE_FRAME:
            raise ValueError(
                f"random attitude reference_frame must be {_POOL_ATTITUDE_FRAME!r}"
            )
        if self.generator_version != _RANDOM_GENERATOR_VERSION:
            raise ValueError(
                "unsupported random attitude generator_version "
                f"{self.generator_version!r}"
            )
        if len(self.rpy_min_rad) != 3 or len(self.rpy_max_rad) != 3:
            raise ValueError("random attitude RPY bounds must contain three values")
        for axis, lower, upper in zip(
            "rpy", self.rpy_min_rad, self.rpy_max_rad
        ):
            if not math.isfinite(lower) or not math.isfinite(upper):
                raise ValueError(f"random attitude {axis} bounds must be finite")
            if lower >= upper:
                raise ValueError(
                    f"random attitude {axis} lower bound must be < upper bound"
                )
        absolute_limits = (math.pi / 2.0, math.pi / 2.0, math.pi)
        for axis, lower, upper, limit in zip(
            "rpy", self.rpy_min_rad, self.rpy_max_rad, absolute_limits
        ):
            if lower < -limit or upper > limit:
                raise ValueError(
                    f"random attitude {axis} bounds exceed [-{limit:g}, {limit:g}]"
                )
        for name, value in (
            ("max_slew_rate_rad_s", self.max_slew_rate_rad_s),
            ("attitude_tolerance_rad", self.attitude_tolerance_rad),
            (
                "angular_speed_tolerance_rad_s",
                self.angular_speed_tolerance_rad_s,
            ),
            ("dwell_time_s", self.dwell_time_s),
            ("max_duration_s", self.max_duration_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"random attitude {name} must be finite and positive")
        if self.max_slew_rate_rad_s > math.pi:
            raise ValueError("random attitude max_slew_rate_rad_s exceeds pi")
        if self.attitude_tolerance_rad > math.pi:
            raise ValueError("random attitude attitude_tolerance_rad exceeds pi")
        if self.angular_speed_tolerance_rad_s > math.pi:
            raise ValueError(
                "random attitude angular_speed_tolerance_rad_s exceeds pi"
            )
        if self.dwell_time_s >= self.max_duration_s:
            raise ValueError("random attitude dwell_time_s must be < max_duration_s")
        if (
            isinstance(self.max_laps, bool)
            or not isinstance(self.max_laps, int)
            or not 1 <= self.max_laps <= (1 << 31) - 1
        ):
            raise ValueError(
                "random attitude max_laps must be in [1, 2^31-1]"
            )


def _counter_uniform(seed: int, event_index: int, axis_index: int) -> float:
    """Portable stateless U[0,1) sample bound to mission/event/axis.

    Taking the first 64 SHA-256 bits and dividing by 2**64 makes the exact
    sequence independent of torch's global RNG, device, and unrelated draws.
    """

    payload = (
        f"{_RANDOM_GENERATOR_VERSION}:{int(seed)}:"
        f"{int(event_index)}:{int(axis_index)}"
    ).encode("ascii")
    integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
    return integer / float(1 << 64)


def deterministic_random_pool_attitude(
    config: RandomAttitudeConfig,
    event_index: int,
    *,
    device="cpu",
) -> torch.Tensor:
    """Return one deterministic pool-Z-up/FLU quaternion in wxyz order."""

    config.validate()
    if isinstance(event_index, bool) or int(event_index) < 0:
        raise ValueError("random attitude event_index must be non-negative")
    values = []
    for axis, (lower, upper) in enumerate(
        zip(config.rpy_min_rad, config.rpy_max_rad)
    ):
        unit = _counter_uniform(config.seed, int(event_index), axis)
        values.append(lower + unit * (upper - lower))
    roll, pitch, yaw = (
        torch.tensor(value, dtype=torch.float32, device=device)
        for value in values
    )
    quaternion = mu.quat_from_euler_xyz(roll, pitch, yaw)
    return mu.quat_unique(quaternion / quaternion.norm().clamp_min(1e-12))


def _quaternion_angle(first: torch.Tensor, second: torch.Tensor) -> torch.Tensor:
    first = first / first.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    second = second / second.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    dot = torch.sum(first * second, dim=-1).abs().clamp(0.0, 1.0)
    return 2.0 * torch.acos(dot)


def _slerp_towards(
    current: torch.Tensor, target: torch.Tensor, max_angle_rad: float
) -> torch.Tensor:
    """Move each wxyz quaternion toward target by at most ``max_angle``."""

    if not math.isfinite(max_angle_rad) or max_angle_rad < 0.0:
        raise ValueError("quaternion slew step must be finite and non-negative")
    current = current / current.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    target = target / target.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    raw_dot = torch.sum(current * target, dim=-1, keepdim=True)
    target = torch.where(raw_dot < 0.0, -target, target)
    dot = torch.sum(current * target, dim=-1, keepdim=True).clamp(0.0, 1.0)
    half_angle = torch.acos(dot)
    full_angle = 2.0 * half_angle
    max_angle = torch.full_like(full_angle, float(max_angle_rad))
    fraction = torch.where(
        full_angle > 1e-8,
        torch.minimum(torch.ones_like(full_angle), max_angle / full_angle),
        torch.ones_like(full_angle),
    )
    sin_half = torch.sin(half_angle)
    linear = current + fraction * (target - current)
    slerp = (
        torch.sin((1.0 - fraction) * half_angle)
        / sin_half.clamp_min(1e-8)
        * current
        + torch.sin(fraction * half_angle)
        / sin_half.clamp_min(1e-8)
        * target
    )
    result = torch.where(half_angle < 1e-6, linear, slerp)
    result = result / result.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    return mu.quat_unique(result)


class LOSGuidance:
    """Waypoint LOS guidance with depth and terminal-position hold."""

    def __init__(
        self,
        waypoints: torch.Tensor,
        device,
        lookahead_dist: float = 1.0,
        cruise_speed: float = 0.5,
        reach_threshold: float = 0.5,
        heading_mode: str = "align",
        loop: bool = True,
        depth_hold_kp: float = 0.8,
        depth_speed_limit: float | None = None,
        terminal_hold_kp: float = 0.5,
        terminal_speed_limit: float | None = None,
        random_attitude_config: RandomAttitudeConfig | None = None,
        pool_to_mission_quaternion: torch.Tensor | None = None,
    ):
        valid_heading_modes = {
            "align",
            "upright",
            "straight",
            "takeoff_then_align",
            "random_at_waypoint",
        }
        if heading_mode not in valid_heading_modes:
            raise ValueError(
                f"heading_mode={heading_mode!r} invalid; "
                f"expected one of {sorted(valid_heading_modes)}"
            )
        self._wp = waypoints
        self.device = device
        self._lookahead = lookahead_dist
        self._speed = cruise_speed
        self._reach = reach_threshold
        self._heading_mode = heading_mode
        self._loop = loop
        self._random_attitude_config = random_attitude_config
        if random_attitude_config is not None:
            random_attitude_config.validate()
            if heading_mode != "random_at_waypoint":
                raise ValueError(
                    "random_attitude_config requires heading_mode="
                    "'random_at_waypoint'"
                )
            if pool_to_mission_quaternion is None:
                raise ValueError(
                    "pool_to_mission_quaternion is required for pool random attitude"
                )
            pool_to_mission = torch.as_tensor(
                pool_to_mission_quaternion, dtype=torch.float32, device=device
            )
            if pool_to_mission.shape != (4,) or not torch.isfinite(
                pool_to_mission
            ).all():
                raise ValueError(
                    "pool_to_mission_quaternion must be one finite wxyz quaternion"
                )
            norm = pool_to_mission.norm()
            if abs(float(norm) - 1.0) > 1e-3:
                raise ValueError("pool_to_mission_quaternion norm is invalid")
            self._pool_to_mission_q = pool_to_mission / norm
        else:
            self._pool_to_mission_q = None
        self._depth_hold_kp = float(depth_hold_kp)
        self._depth_speed_limit = float(
            cruise_speed if depth_speed_limit is None else depth_speed_limit
        )
        self._terminal_hold_kp = float(terminal_hold_kp)
        self._terminal_speed_limit = float(
            cruise_speed if terminal_speed_limit is None else terminal_speed_limit
        )
        if self._depth_hold_kp <= 0.0 or self._depth_speed_limit <= 0.0:
            raise ValueError("depth hold gain/speed limit은 양수여야 함")
        if self._terminal_hold_kp <= 0.0 or self._terminal_speed_limit <= 0.0:
            raise ValueError("terminal hold gain/speed limit은 양수여야 함")

        self.num_envs, self.num_wp, _ = waypoints.shape
        if heading_mode == "takeoff_then_align" and (
            not loop or self.num_wp != 3
        ):
            raise ValueError(
                "takeoff_then_align requires loop=true and exactly three waypoints"
            )
        self._wp_idx = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        # loop=False일 때만 의미 있음 — 마지막 웨이포인트 도달 후 True로 고정,
        # 이후 final waypoint position hold로 전환한다. 실배포 미션은 보통 한 번
        # 돌고 종료 위치를 유지해야 하므로 loop와 completion을 명시적으로 관리한다.
        self.mission_complete = torch.zeros(self.num_envs, dtype=torch.bool, device=device)

        self._random_q_d = mu.identity_quat(self.num_envs, device)
        self._random_q_goal = mu.identity_quat(self.num_envs, device)
        self._random_q_goal_pool = mu.identity_quat(self.num_envs, device)
        self._random_event_index = torch.zeros(
            self.num_envs, dtype=torch.long, device=device
        )
        self._random_dwell_s = torch.zeros(self.num_envs, device=device)
        self._elapsed_s = torch.zeros(self.num_envs, device=device)
        self._lap_count = torch.zeros(
            self.num_envs, dtype=torch.long, device=device
        )
        self._termination_reason: str | None = None
        self._straight_q_d = mu.identity_quat(self.num_envs, device)
        if heading_mode == "random_at_waypoint":
            if random_attitude_config is None:
                self._random_q_d = self._sample_random_attitude(self.num_envs)
                self._random_q_goal = self._random_q_d.clone()
            else:
                self._set_deterministic_random_goal(
                    torch.arange(self.num_envs, device=device)
                )
                self._random_q_d = self._random_q_goal.clone()

    def _sample_random_attitude(self, n: int) -> torch.Tensor:
        roll = mu.sample_uniform(-torch.pi / 2, torch.pi / 2, (n,), self.device)
        pitch = mu.sample_uniform(-torch.pi / 2, torch.pi / 2, (n,), self.device)
        yaw = mu.sample_uniform(-torch.pi, torch.pi, (n,), self.device)
        return mu.quat_from_euler_xyz(roll, pitch, yaw)

    def _pool_flu_to_mission_frd(self, quaternion: torch.Tensor) -> torch.Tensor:
        assert self._pool_to_mission_q is not None
        q_left = self._pool_to_mission_q.expand(quaternion.shape[0], -1)
        q_body = quaternion.new_tensor([0.0, 1.0, 0.0, 0.0]).expand(
            quaternion.shape[0], -1
        )
        result = mu.quat_mul(mu.quat_mul(q_left, quaternion), q_body)
        result = result / result.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        return mu.quat_unique(result)

    def _set_deterministic_random_goal(self, env_ids: torch.Tensor) -> None:
        assert self._random_attitude_config is not None
        pool_values = []
        for env_id in env_ids.tolist():
            pool_values.append(
                deterministic_random_pool_attitude(
                    self._random_attitude_config,
                    int(self._random_event_index[env_id]),
                    device=self.device,
                )
            )
        pool = torch.stack(pool_values)
        self._random_q_goal_pool[env_ids] = pool
        self._random_q_goal[env_ids] = self._pool_flu_to_mission_frd(pool)

    def reset(
        self,
        env_ids: torch.Tensor,
        initial_quat: torch.Tensor | None = None,
        *,
        resample_random_attitude: bool = True,
    ) -> None:
        """Reset mission progress and heading state for selected environments.

        ``resample_random_attitude=False`` lets a real-vehicle activation keep
        the random target that was already visible during shadow mode. This
        prevents ``start_control`` from introducing an unobserved attitude step.
        """
        self._wp_idx[env_ids] = 0
        self.mission_complete[env_ids] = False
        self._random_dwell_s[env_ids] = 0.0
        self._elapsed_s[env_ids] = 0.0
        self._lap_count[env_ids] = 0
        self._termination_reason = None
        if self._heading_mode in {"straight", "takeoff_then_align"}:
            if initial_quat is None:
                raise ValueError("straight heading reset에는 initial_quat가 필요함")
            yaw = mu.yaw_from_quat(initial_quat)
            zero = torch.zeros_like(yaw)
            # 시작 roll/pitch는 목표에 포함하지 않고, 시작 yaw만 고정한다.
            self._straight_q_d[env_ids] = mu.quat_from_euler_xyz(zero, zero, yaw)
        if self._heading_mode == "random_at_waypoint":
            if self._random_attitude_config is None:
                if resample_random_attitude:
                    self._random_q_d[env_ids] = self._sample_random_attitude(
                        len(env_ids)
                    )
                self._random_q_goal[env_ids] = self._random_q_d[env_ids]
            else:
                self._random_event_index[env_ids] = 0
                self._set_deterministic_random_goal(env_ids)
                if initial_quat is None:
                    raise ValueError(
                        "pool random attitude reset requires initial_quat"
                    )
                initial = torch.as_tensor(
                    initial_quat, dtype=torch.float32, device=self.device
                )
                if initial.shape != (len(env_ids), 4) or not torch.isfinite(
                    initial
                ).all():
                    raise ValueError(
                        "random attitude initial_quat must have shape (N,4)"
                    )
                initial = initial / initial.norm(
                    dim=-1, keepdim=True
                ).clamp_min(1e-12)
                self._random_q_d[env_ids] = mu.quat_unique(initial)

    def _current_and_next(self, idx: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        env_i = torch.arange(self.num_envs, device=self.device)
        if self._heading_mode == "takeoff_then_align":
            next_idx = torch.where(
                idx == self.num_wp - 1,
                torch.ones_like(idx),
                idx + 1,
            )
        else:
            next_idx = (idx + 1) % self.num_wp
        return self._wp[env_i, idx], self._wp[env_i, next_idx]

    def compute(
        self,
        pos_env: torch.Tensor,
        root_quat_w: torch.Tensor,
        advance_waypoint: bool = True,
        dt: float = 0.0,
        angular_speed_rad_s: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """pos_env/root_quat_w는 world(NED) 기준. 반환 v_d_b/q_d도 NED-body 규약
        (obs_builder가 Z-up으로 변환).

        ``advance_waypoint=False``는 제어 시작 전 shadow observation을 만들 때 사용한다.
        이때 LOS 목표는 계산하되 waypoint index와 mission_complete는 변경하지 않는다.
        """
        if not math.isfinite(float(dt)) or float(dt) < 0.0:
            raise ValueError("guidance dt must be finite and non-negative")
        if self._random_attitude_config is not None and advance_waypoint:
            self._elapsed_s += float(dt)
            max_duration = self._random_attitude_config.max_duration_s
            if bool((self._elapsed_s >= max_duration).any()):
                self.mission_complete |= self._elapsed_s >= max_duration
                self._termination_reason = "maximum mission duration reached"

            self._random_q_d = _slerp_towards(
                self._random_q_d,
                self._random_q_goal,
                self._random_attitude_config.max_slew_rate_rad_s * float(dt),
            )

        _, next_wp = self._current_and_next(self._wp_idx)
        position_error = torch.norm(next_wp - pos_env, dim=-1)
        position_reached = position_error < self._reach
        if self._heading_mode == "takeoff_then_align":
            # Do not begin the horizontal loop while still far below its plane.
            takeoff_reached = position_error < min(self._reach, 0.05)
            position_reached = torch.where(
                self._wp_idx == 0, takeoff_reached, position_reached
            )
        if self._random_attitude_config is None:
            reached = position_reached
        else:
            if angular_speed_rad_s is None:
                raise ValueError(
                    "pool random attitude guidance requires angular speed"
                )
            angular_speed = torch.as_tensor(
                angular_speed_rad_s, dtype=torch.float32, device=self.device
            ).reshape(-1)
            if angular_speed.shape != (self.num_envs,) or not torch.isfinite(
                angular_speed
            ).all():
                raise ValueError(
                    "angular_speed_rad_s must contain one finite value per environment"
                )
            attitude_ready = _quaternion_angle(
                root_quat_w, self._random_q_goal
            ) <= self._random_attitude_config.attitude_tolerance_rad
            angular_ready = (
                angular_speed
                <= self._random_attitude_config.angular_speed_tolerance_rad_s
            )
            dwell_condition = position_reached & attitude_ready & angular_ready
            if advance_waypoint:
                self._random_dwell_s = torch.where(
                    dwell_condition,
                    self._random_dwell_s + float(dt),
                    torch.zeros_like(self._random_dwell_s),
                )
            reached = dwell_condition & (
                self._random_dwell_s
                >= self._random_attitude_config.dwell_time_s
            )
            reached &= ~self.mission_complete
        previous_wp_idx = self._wp_idx.clone()
        previous_mission_complete = self.mission_complete.clone()

        if advance_waypoint:
            if self._loop:
                if self._heading_mode == "takeoff_then_align":
                    next_idx = torch.where(
                        self._wp_idx == self.num_wp - 1,
                        torch.ones_like(self._wp_idx),
                        self._wp_idx + 1,
                    )
                else:
                    next_idx = (self._wp_idx + 1) % self.num_wp
                self._wp_idx = torch.where(reached, next_idx, self._wp_idx)
                if self._random_attitude_config is not None:
                    wrapped = reached & (previous_wp_idx == self.num_wp - 1)
                    self._lap_count += wrapped.to(self._lap_count.dtype)
                    lap_complete = (
                        self._lap_count >= self._random_attitude_config.max_laps
                    )
                    if bool(lap_complete.any()):
                        self.mission_complete |= lap_complete
                        self._termination_reason = "maximum mission laps reached"
            else:
                # 마지막 세그먼트(idx == num_wp-2)에서 도달하면 그 이상 전진하지 않고
                # mission_complete만 세운다 — idx가 num_wp-1까지 가면 (idx+1)%num_wp가
                # 0으로 wrap해서 처음으로 되돌아가버리므로(요청한 "반복" 버그) 아예 막음.
                at_last_segment = self._wp_idx == (self.num_wp - 2)
                self.mission_complete = self.mission_complete | (reached & at_last_segment)
                if (
                    self._random_attitude_config is not None
                    and bool((reached & at_last_segment).any())
                ):
                    self._termination_reason = "final waypoint reached"
                advance = reached & ~at_last_segment
                self._wp_idx = torch.where(advance, self._wp_idx + 1, self._wp_idx)

        cur_wp, next_wp = self._current_and_next(self._wp_idx)

        # A random attitude is sampled once per actual waypoint-arrival event.
        # With loop=False the final index intentionally does not advance, so
        # mission_complete's rising edge represents the final arrival. Sampling
        # from raw ``reached`` would otherwise command a new random attitude on
        # every control tick while the vehicle remained at the final waypoint.
        waypoint_arrival = (self._wp_idx != previous_wp_idx) | (
            self.mission_complete & ~previous_mission_complete
        )
        if (
            advance_waypoint
            and self._heading_mode == "random_at_waypoint"
            and waypoint_arrival.any()
        ):
            idx = waypoint_arrival.nonzero(as_tuple=True)[0]
            if self._random_attitude_config is None:
                self._random_q_d[idx] = self._sample_random_attitude(len(idx))
                self._random_q_goal[idx] = self._random_q_d[idx]
            else:
                # A terminal duration/lap event closes control; it must not
                # consume an unobservable extra random target.
                idx = idx[~self.mission_complete[idx]]
                if len(idx):
                    self._random_event_index[idx] += 1
                    self._set_deterministic_random_goal(idx)
                self._random_dwell_s[waypoint_arrival] = 0.0

        seg = next_wp - cur_wp
        seg_len = seg.norm(dim=-1, keepdim=True).clamp_min(1e-6)
        seg_dir = seg / seg_len

        s = ((pos_env - cur_wp) * seg_dir).sum(-1, keepdim=True).clamp(min=0.0)
        s = torch.minimum(s, seg_len)
        look_s = torch.minimum(s + self._lookahead, seg_len)
        los_point = cur_wp + look_s * seg_dir

        to_los = los_point - pos_env
        v_d_world = self._speed * to_los / to_los.norm(dim=-1, keepdim=True).clamp_min(1e-6)

        if not self._loop:
            # 마지막 waypoint에 한 번 도달했더라도 속도 목표를 0으로 고정하면
            # 음성부력/테더 외력으로 이탈한 뒤 복귀할 수 없다. 완주 상태에서는
            # 최종 waypoint에 대한 position outer-loop를 계속 유지한다.
            terminal_error = next_wp - pos_env
            terminal_velocity = self._terminal_hold_kp * terminal_error
            terminal_norm = terminal_velocity.norm(dim=-1, keepdim=True).clamp_min(1e-6)
            terminal_scale = torch.clamp(
                self._terminal_speed_limit / terminal_norm, max=1.0
            )
            terminal_velocity = terminal_velocity * terminal_scale
            v_d_world = torch.where(
                self.mission_complete.unsqueeze(-1), terminal_velocity, v_d_world
            )

        # 3D LOS 정규화에서는 긴 수평 lookahead가 작은 깊이 오차를 압도한다.
        # Z(NED)는 항상 현재 세그먼트의 next waypoint 깊이를 독립 추종한다.
        depth_error = next_wp[:, 2] - pos_env[:, 2]
        v_d_world[:, 2] = torch.clamp(
            self._depth_hold_kp * depth_error,
            -self._depth_speed_limit,
            self._depth_speed_limit,
        )

        v_d_b = mu.quat_apply(mu.quat_conjugate(root_quat_w), v_d_world)

        if self._heading_mode == "align":
            q_d = _heading_from_direction(v_d_world, self.device)
        elif self._heading_mode == "straight":
            q_d = self._straight_q_d
        elif self._heading_mode == "takeoff_then_align":
            aligned = _heading_from_direction(v_d_world, self.device)
            q_d = torch.where(
                (self._wp_idx == 0).unsqueeze(-1),
                self._straight_q_d,
                aligned,
            )
        elif self._heading_mode == "random_at_waypoint":
            q_d = self._random_q_d
        else:   # "upright": NED/mission frame yaw=0
            q_d = mu.identity_quat(self.num_envs, self.device)

        return v_d_b, q_d

    @property
    def termination_reason(self) -> str | None:
        return self._termination_reason

    @property
    def lap_count(self) -> int:
        return int(self._lap_count[0])

    @property
    def elapsed_s(self) -> float:
        return float(self._elapsed_s[0])

    @property
    def random_event_index(self) -> int:
        return int(self._random_event_index[0])

    @property
    def random_goal_pool(self) -> torch.Tensor | None:
        if self._random_attitude_config is None:
            return None
        return self._random_q_goal_pool[0].clone()


def _heading_from_direction(direction_w: torch.Tensor, device) -> torch.Tensor:
    d = direction_w / direction_w.norm(dim=-1, keepdim=True).clamp_min(1e-6)
    yaw = torch.atan2(d[:, 1], d[:, 0])
    pitch = torch.asin(d[:, 2].clamp(-1.0, 1.0))
    roll = torch.zeros_like(yaw)
    return mu.quat_from_euler_xyz(roll, pitch, yaw)
