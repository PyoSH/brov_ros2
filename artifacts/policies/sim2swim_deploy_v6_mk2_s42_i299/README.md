# Sim2Swim deploy-v6 MK2 policy (action-envelope + anti-windup retrain)

This bundle is separate from `demo_policy`, `sim2swim_deploy_v2_mk2_s42_i299`
through `sim2swim_deploy_v5_mk2_s42_i299`.

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD
  allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/
  observation contracts, wrench scale, and vehicle-model SHA all match.
  `policy_contract.py` must accept `profile="deploy_v6"` in
  `MK2_ACCEPTED_PROFILES` -- **not yet added as of this bundle's creation.**

## Why this checkpoint exists: the deploy_v5 real-water failure

2026-08-18, first real-water test of `sim2swim_deploy_v5_mk2_s42_i299`: 4
seconds after the P1->P2 waypoint transition (a LOS heading retarget), yaw
and pitch diverged together and attitude error reached 129 deg. Real
telemetry (228 samples, 9.12s active, 25 Hz) showed heavy saturation
**against the deployed clamp envelope**
(`rl_controller_mk2_real_v1.yaml`'s `action_abs_limit=[0.3,0.3,0.3,0.2,
0.15,0.2]`, not against `[-1,1]`): surge 69.7%, pitch 57.0%, sway 34.2%,
yaw 31.6%, heave 21.9%, roll 19.7%. Command signs were confirmed correct
(roll/pitch/yaw sign-agreement 70/58/78%), ruling out a T6/allocation bug.

Root cause: deploy_v5 (and every earlier checkpoint) trained with full
`[-1,1]` actuation authority (`_pre_physics_step` clamped only to
`[-1,1]`), but deployment clamps the same raw action to 15-30% of that
before computing wrench -- a structural authority mismatch, not a bug in
any single component. A follow-up real-bag analysis (3rd-round diagnosis,
same day) narrowed it further: of the 16-D observation's 6 integral terms,
only `qint_y` (pitch attitude-error integral) pinned at the -5.0 limit
(15.5% of samples); `vint_z` (heave velocity integral) had margin (-1.0).
The reported depth oscillation (6.14s period, +/-0.224m) was a downstream
symptom of pitch integral windup, not an independent cause. Pitch's
deployed torque budget (0.15 x 14.0 = 2.10 N*m) was even below the maximum
CoB-randomization trim moment (`dr_cob_radius=0.015` -> 2.16 N*m) --
insufficient authority to counter even the training-distribution
disturbance on that one axis. Ballast on vs off reproduced the identical
failure, ruling out a hardware/trim-specific cause.

## What changed vs deploy_v5

Two items from the resulting retrain spec, on top of deploy_v5's A+B+C+D
(unchanged):

1. **Physical action-envelope clamp during training.** `_pre_physics_step`
   now clamps the `[-1,1]`-clamped action to
   `deploy_v6_action_abs_limit = [0.3, 0.3, 0.3, 0.2, 0.25, 0.2]` before it
   is used anywhere else (wrench, reward, everything) -- reproducing
   `policy_node.py`'s own clamp chain exactly. `f_max`/`WRENCH_SCALE`
   ([85, 85, 120, 26, 14, 22]) are untouched; only the action that
   multiplies it is bounded first, so this bundle's `wrench_scale` field is
   identical to v2-v5's. Pitch is **0.25, not the currently-deployed
   0.15** -- see "Real-vehicle envelope requirement" below, this is a
   training target that requires a matching deployment-side config change.
   A matching `deploy_penalty_envelope_overflow_l2=0.15` reward term
   (mirroring deploy_v5's raw-overflow penalty, item D, but keyed to the
   envelope instead of `1.0`) discourages the actor from producing output
   beyond what has any physical effect once the clamp is active.
2. **Per-axis integrator anti-windup.** Halts a `z_v`/`z_q` axis's
   integration on ticks where that axis's own action is being clamped away
   by the envelope (reusing the same overflow signal as the reward term
   above). Chosen over the diagnosis's other two options: back-calculation
   anti-windup needs new per-axis gain state with no deployment-side
   precedent to mirror; narrowing `integral_velocity_limit`/
   `integral_attitude_limit` changes the observation contract itself and
   was explicitly ruled out. This keeps the +/-5.0 bound and the 16-D
   contract exactly as documented, only changing *when* each axis
   accumulates.
3. **LOS-coupled attitude retargets during training** (carried over from
   the item-1 planning pass, same day): `guidance/los_guidance.py`'s
   `heading_mode="align"` makes desired attitude a deterministic function
   of desired velocity direction every tick, so both jump together at a
   waypoint transition. The prior command curriculum (`DeployV3Scheduler`)
   sampled attitude and velocity targets independently. `DeployV6Scheduler`
   couples a configurable fraction (default 50%) of leg retargets to their
   own leg's velocity direction, using the vehicle's live current attitude
   at the coupling instant -- matching `LOSGuidance.compute()`'s own
   per-tick recomputation, not a static per-leg approximation.

Training: 2048 envs x 128-step rollout x 300 iterations, seed 42, same
scale as v2-v5. Mean episode reward converged to ~713 by iteration 299
(lower than v5's ~810-828, expected given the tighter/never-before-imposed
action authority -- not itself a failure signal). Action std shrank
smoothly 0.95 -> 0.073 (v5: ~0.061) with no collapse or divergence; episode
length stayed at the full 30s window throughout training after the initial
transient (no early-termination instability).

## Isaac-side check (not the deciding evidence -- see caveat)

`test_policy.py --profile deploy_v6` under the trained envelope
(`action_abs_limit=[0.3,0.3,0.3,0.2,0.25,0.2]`): `action_bound_per_axis_
fraction` is `[0,0,0,0,0,0]` (zero saturation, all axes) on both
`straight_line` and `square_random_attitude` (Trial c, full random
attitude retarget at every waypoint). `steady_thruster_force_clamp_any_
fraction=0.0` on both.

**Caveat, not a green light**: replaying the *old* `deploy_v5` checkpoint
under the same envelope in Isaac (`--eval_action_abs_limit`) *also* shows
zero saturation -- this reproduces the already-known Isaac-Gazebo gap from
the deploy_v5 F_max diagnostic (Isaac showed ~0% saturation for v5 while
Gazebo showed 42-88%). The Isaac check above confirms the new code paths
run correctly and the policy is not obviously broken; it does **not**
distinguish "deploy_v6 fixed the problem" from "Isaac doesn't reproduce
this failure mode for any checkpoint." Only a Gazebo comparison can answer
that.

## Deployment status: Gazebo validation not yet run

**No Gazebo Case-A/Case-C run has been performed for this checkpoint yet.**
The existing Case-A gate harness
(`sim2sim_mk2_case_a.launch.py`) hardcodes `rl_controller_mk2_deploy_v2.yaml`
(`action_abs_limit=[1,1,1,1,1,1]`, no clamp) -- every v2-v5 Gazebo number on
record, including v5's 49.8%/53.6%, was measured with **no** envelope
clamp active, so it cannot be compared against either the real-vehicle
failure numbers above or this checkpoint's design intent without a new
harness that actually applies `rl_controller_mk2_real_v1.yaml`. Do not
treat the Isaac-side zero-saturation result above as validation; per the
existing project history, no checkpoint through v5 has ever cleared the
*formal* Case-A gate either, and that remains true/unknown here until
measured.

## Real-vehicle envelope requirement (do not skip)

This checkpoint was trained assuming pitch `action_abs_limit=0.25`, not
the currently-deployed `0.15`. **Do not run this bundle against the
unmodified `rl_controller_mk2_real_v1.yaml`** -- doing so reintroduces a
new version of the exact authority mismatch this retrain exists to fix
(now on a checkpoint that has never been evaluated under the tighter,
still-deployed 0.15 limit). `rl_controller_mk2_real_v1.yaml`'s pitch
`action_abs_limit` must be raised to `0.25` before any real-vehicle use of
this bundle.

Separately, `brov_base/brov_base/observation.py`'s `_z_q`/`_z_v` update is
an independent reimplementation of this training environment's
integration rule (not an import of `envs/observation_contract.py`) -- it
needs the identical per-axis conditional-integration anti-windup logic
described above, or the exact train/deploy integral-behavior mismatch this
checkpoint fixes for pitch will reappear on the real vehicle through a
different path.

## Not yet decided

Real-vehicle use of this bundle has **not** been reviewed or approved --
unlike `sim2swim_deploy_v5_mk2_s42_i299`'s README, this section is
deliberately left without a decision. It requires, at minimum: the
`MK2_ACCEPTED_PROFILES` addition, the real-envelope Gazebo harness and a
comparison run against a re-baselined deploy_v5 (also under the real
envelope, not v5's existing unrestricted-envelope numbers), the
`rl_controller_mk2_real_v1.yaml` pitch bump to 0.25, and the
`observation.py` anti-windup parity change.
