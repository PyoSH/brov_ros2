import math

import torch

from brov_base.guidance import (
    deterministic_random_pool_attitude,
    LOSGuidance,
    RandomAttitudeConfig,
    _quaternion_angle,
)


def _config(**overrides) -> RandomAttitudeConfig:
    values = {
        "seed": 20260814,
        "reference_frame": "pool_zup_flu",
        "generator_version": "sha256_counter_uniform_rpy_v1",
        "rpy_min_rad": (-math.pi / 2.0, -math.pi / 2.0, -math.pi),
        "rpy_max_rad": (math.pi / 2.0, math.pi / 2.0, math.pi),
        "max_slew_rate_rad_s": 0.35,
        "attitude_tolerance_rad": 0.20,
        "angular_speed_tolerance_rad_s": 0.10,
        "dwell_time_s": 0.20,
        "max_duration_s": 120.0,
        "max_laps": 1,
    }
    values.update(overrides)
    return RandomAttitudeConfig(**values)


def _guidance(config=None, *, loop=True) -> LOSGuidance:
    guidance = LOSGuidance(
        torch.tensor(
            [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]]
        ),
        "cpu",
        cruise_speed=0.1,
        reach_threshold=0.1,
        heading_mode="random_at_waypoint",
        loop=loop,
        random_attitude_config=config or _config(),
        # Tests use a generic proper frame transform; exact pool/mission
        # composition is covered by mission transform tests.
        pool_to_mission_quaternion=torch.tensor([1.0, 0.0, 0.0, 0.0]),
    )
    guidance.reset(
        torch.tensor([0]),
        initial_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        resample_random_attitude=False,
    )
    return guidance


def test_counter_sampler_is_repeatable_and_does_not_consume_torch_rng() -> None:
    config = _config()
    first = deterministic_random_pool_attitude(config, 0)
    assert torch.equal(first, deterministic_random_pool_attitude(config, 0))
    assert not torch.equal(first, deterministic_random_pool_attitude(config, 1))
    assert torch.allclose(
        first,
        torch.tensor(
            [
                0.36995846033096313,
                0.15418720245361328,
                0.6187490820884705,
                -0.6756527423858643,
            ]
        ),
        atol=1e-7,
        rtol=0.0,
    )

    torch.manual_seed(17)
    expected = torch.rand(4)
    torch.manual_seed(17)
    deterministic_random_pool_attitude(config, 7)
    assert torch.equal(torch.rand(4), expected)


def test_preview_and_start_preserve_goal_while_command_slews() -> None:
    guidance = _guidance()
    goal = guidance.random_goal_pool
    initial_command = guidance._random_q_d.clone()

    guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        advance_waypoint=False,
        dt=10.0,
        angular_speed_rad_s=torch.tensor([0.0]),
    )
    assert torch.equal(guidance.random_goal_pool, goal)
    assert torch.equal(guidance._random_q_d, initial_command)
    assert guidance.elapsed_s == 0.0
    assert guidance.random_event_index == 0

    _, desired = guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        advance_waypoint=True,
        dt=0.1,
        angular_speed_rad_s=torch.tensor([0.0]),
    )
    assert torch.equal(guidance.random_goal_pool, goal)
    assert guidance.random_event_index == 0
    assert float(_quaternion_angle(initial_command, desired)) <= 0.035001


def _arrive(guidance: LOSGuidance, point: list[float]) -> None:
    # Two 0.1 s samples satisfy the configured 0.2 s dwell.  Actual attitude
    # and angular rate must both satisfy their gates.
    for _ in range(2):
        guidance.compute(
            torch.tensor([point]),
            guidance._random_q_goal.clone(),
            advance_waypoint=True,
            dt=0.1,
            angular_speed_rad_s=torch.tensor([0.0]),
        )


def test_transition_samples_once_after_pose_rate_and_dwell_gates() -> None:
    guidance = _guidance()
    target = [1.0, 0.0, 0.0]

    # Position alone is insufficient.
    guidance.compute(
        torch.tensor([target]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        dt=0.5,
        angular_speed_rad_s=torch.tensor([1.0]),
    )
    assert int(guidance._wp_idx[0]) == 0
    assert guidance.random_event_index == 0

    _arrive(guidance, target)
    assert int(guidance._wp_idx[0]) == 1
    assert guidance.random_event_index == 1

    # Dwelling at the old point cannot sample again because the target moved.
    for _ in range(5):
        guidance.compute(
            torch.tensor([target]),
            guidance._random_q_goal.clone(),
            dt=0.1,
            angular_speed_rad_s=torch.tensor([0.0]),
        )
    assert guidance.random_event_index == 1


def test_loop_wrap_counts_one_lap_and_does_not_consume_terminal_sample() -> None:
    guidance = _guidance()
    _arrive(guidance, [1.0, 0.0, 0.0])
    _arrive(guidance, [1.0, 1.0, 0.0])
    assert guidance.random_event_index == 2
    _arrive(guidance, [0.0, 0.0, 0.0])

    assert guidance.lap_count == 1
    assert bool(guidance.mission_complete[0])
    assert guidance.termination_reason == "maximum mission laps reached"
    assert guidance.random_event_index == 2


def test_duration_is_active_time_only_and_requests_normal_completion() -> None:
    guidance = _guidance(_config(max_duration_s=1.0, dwell_time_s=0.2))
    guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        advance_waypoint=False,
        dt=5.0,
        angular_speed_rad_s=torch.tensor([0.0]),
    )
    assert guidance.termination_reason is None

    guidance.compute(
        torch.tensor([[0.0, 0.0, 0.0]]),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        advance_waypoint=True,
        dt=1.0,
        angular_speed_rad_s=torch.tensor([0.0]),
    )
    assert bool(guidance.mission_complete[0])
    assert guidance.termination_reason == "maximum mission duration reached"


def test_random_config_rejects_unsafe_or_ambiguous_metadata() -> None:
    for config in (
        _config(seed=1 << 63),
        _config(reference_frame="start_heading"),
        _config(generator_version="torch_global_rng"),
        _config(rpy_max_rad=(math.pi, math.pi / 2.0, math.pi)),
        _config(dwell_time_s=120.0),
        _config(max_laps=0),
    ):
        try:
            config.validate()
        except ValueError:
            pass
        else:
            raise AssertionError(f"unsafe random config accepted: {config}")
