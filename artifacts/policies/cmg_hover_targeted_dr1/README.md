# CMG hover policy (Targeted DR1)

OBS17 -> direct ACTION8 hover-in-place TorchScript policy, SHA-256
`16f12c4a64f6876bb1be9a9cd52c604e0e777e25694c3089b5050909c9e84cef`. Ported
from the standalone `cmg_RL_deploy` package
(`/home/pyo/Downloads/cmg_RL_deploy`) into the `cmg_deploy` ROS2 package in
this repository.

## Contract

- Observation (17): `[target_q_wxyz(4), target_offset_body_xyz(3),
  current_q_wxyz(4), body_linear_velocity_xyz(3),
  body_angular_velocity_xyz(3)]`, no normalization/clipping/history.
- Action (8): direct per-thruster T1..T8 command, `clip(action, -1, 1)`.
  No 6-D wrench, allocation matrix, or pseudoinverse.
- Target: `HOVER_ORIGIN` by default -- the first pose observed after the
  control-active edge is latched as the hover setpoint. Fully
  self-contained; does not consume brov_mission's waypoint/guidance
  stack.

## Why this differs from the standalone `cmg_RL_deploy` package

The original package owns its own MAVLink connection
(`real/mavlink_interface.py`) so it can run with zero other dependencies.
That would conflict with brov_ros2's single-MAVLink-owner architecture if
run alongside it. `cmg_deploy` instead:

- subscribes to brov_base's already-validated
  `/brov/odometry/local_with_session` for state (its twist is already
  body-frame FLU, so no extra rotation is needed for velocity);
- publishes the clipped `[-1, 1]` ACTION8 directly to the shared
  `/brov/thruster_pwm` topic;
- does **not** apply PWM microsecond scaling or the real-vehicle T2/T3/T8
  reversal mask itself -- brov_base's PWM gateway
  (`thruster_reversal_profile: real_brov2` on hardware,
  `edo_sitl_identity` in Gazebo SITL) remains the single owner of both,
  exactly as it already is for `policy_node`/`policy_node_mk2`/
  `model_based_controller_node`.

`cmg_deploy`'s own `config/thruster_mapping.yaml` (policy_index ->
RC channel, all `UNVERIFIED`) is not used by this integration -- it is
superseded by brov_base's own verified channel order and reversal
profile.

## Status

Package scaffolded and unit-tested (2026-08-18). First Gazebo SITL run
(`state_source=mavlink_ekf`, the production default) failed to hold
hover: GT position drifted ~3m, mostly +Z, and the run's abort-drift
safety net stopped it. A second, diagnostic-only run
(`state_source=gazebo_truth_diagnostic`, added specifically to isolate
this -- see `cmg_deploy`'s own docstring/README) fed the same policy
clean Gazebo ground truth instead of MAVLink/EKF telemetry, and held
hover to `max_drift_m=0.75`, `mean_drift_m=0.27` over a 40s window. This
places the earlier failure's cause on the MAVLink/EKF feedback path
(`/brov/odometry/local_with_session` is always MAVLink-derived,
independent of `feedback_source`), not on the policy or thruster mapping
themselves.

0.75m max drift is not tight hover and this is still a single run --
**do not treat this as validated for real-vehicle use.** Before that:
rerun `gazebo_truth_diagnostic` a few more times for repeatability,
investigate why the MAVLink/EKF vertical estimate diverged so much in
the first run, and only then consider a real-vehicle shadow-mode check
with the conservative `cmg_deploy_real_v1.yaml` envelope, per the same
shadow-first procedure used for every other controller in this repo.
