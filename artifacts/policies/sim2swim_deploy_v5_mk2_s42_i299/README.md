# Sim2Swim deploy-v5 MK2 policy (A+B+C+D: raw-actor-overflow penalty)

This bundle is separate from `demo_policy`, `sim2swim_deploy_v2_mk2_s42_i299`,
`sim2swim_deploy_v3_mk2_s42_i299` (item A only), and
`sim2swim_deploy_v4_mk2_s42_i299` (A+B+C).

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD
  allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/
  observation contracts, wrench scale, and vehicle-model SHA all match.
  `policy_contract.py` accepts `profile` in
  `("deploy_v2", "deploy_v3", "deploy_v4", "deploy_v5")` for the MK2 action
  contract.

## Provenance

deploy_v4 (A+B+C) cut whole-cycle action-bound saturation from ~99%
(v2/v3) to 78.9% (GT) / 88.6% (DVL-EKF) and force-clamp to 17.7%/19.3%, but
still failed the formal Case-A gate. A follow-up diagnostic
(`diagnose_attitude_torque_budget.py` + `diagnose_desired_attitude_stability.py`
+ a baseline comparison against the model-based controller's own Gazebo GT
bag) found:

- the guidance target itself is stable during cruise (implied desired-
  attitude rate ~3 deg/s mean, ~8 deg/s p95, away from the two legitimate
  large discrete retargets at the outbound/return transitions);
- cruise-time tracking error is modest (~5.5 deg mean, ~14 deg p95);
- yet raw roll/pitch/yaw action and body rate still oscillate continuously
  at near-saturated levels even during that calm cruise;
- the same Gazebo environment under the model-based controller (known
  ~0% saturation) shows body rates 3-10x calmer (roll/pitch mean 3.8/9.0
  deg/s vs RL's 40.8/43.3 deg/s even when *not* pinned), ruling out
  "the environment/sensors are just noisy" as the explanation.

This pointed at item D (deferred from the original A/B/C/D/E spec): the
actor's raw (pre-clamp) output is discarded by `_pre_physics_step` before
the reward ever sees it, so the three existing action penalties
(`deploy_penalty_action_l2`, `_action_delta_l2`, `_thruster_clamp_l2`) are
all computed on the already-clamped `[-1,1]` action and cannot distinguish
a raw output that barely overshot the bound from one that overshot it by
several units -- consistent with `MK2_SIM2SIM_DEPLOY_RESULT.md`'s own
"minimum remediation" list (sec. 8, item 2) from an earlier phase of this
investigation.

**Item D implementation**: `envs/vel_env.py` now caches `self._raw_actions`
(pre-clamp) in `_pre_physics_step`, and `_get_rewards` adds
`- deploy_penalty_raw_overflow_l2 * sum(relu(|raw_action| - 1)^2)`. The
weight (0.15) was calibrated from a TorchScript replay of deploy_v4's own
Gazebo GT/DVL-EKF bags through the exported deploy_v4 policy: overflow
concentrated almost entirely in pitch/yaw (near-zero on surge/sway/heave/
roll), sum-of-squares overflow per step averaging ~0.3-0.36 (p95 ~1.8-2.1).

Trained 2048 envs x 128-step rollout x 300 iterations, seed 42, same scale
as v2/v3/v4. Mean episode reward converged to ~810-828 by iteration 299
(comparable to v4's ~818-828), action std reached 0.061 by iter 299
(v4: 0.071) -- training was not destabilized by the new penalty term.

## Deployment status: Gazebo Case-A/Case-C complete, formal gate still unmet

Measured whole-cycle action-bound saturation GT 78.9%->**49.8%**, DVL-EKF
88.6%->**53.6%**; force-clamp GT 17.7%->**0.8%**, DVL-EKF 19.3%->**2.2%**
versus deploy_v4. Roll-axis saturation reached 0%; pitch remains the
holdout (42-45%, plausibly a genuine torque-budget ceiling -- see
`diagnose_attitude_torque_budget.py` output, body rate ~2.5x higher while
pitch is pinned than while it isn't, on this bundle too). The same A-B+C-D
staircase reproduced on the independently-built Case-C mission (5 m square,
random attitude per corner), the first cross-geometry check in this
investigation. Both are in `project_step2_brov_retrain_spec` memory and the
"Found It" / "It Generalizes" artifacts.

The formal Case-A gate's strict absolute thresholds (<1% steady action-cap,
<5% whole-cycle, <=10 deg attitude excursion) are still not met -- no
checkpoint in this investigation (v2 through v5) has ever cleared them.

## Real-vehicle use: explicitly approved for the case a-2 first-water test

2026-08-18: the user reviewed the above (including that the formal gate is
unmet and pitch is still the least-improved axis) and explicitly chose this
checkpoint, on the real vehicle, for the case a-2 demo (case-a geometry +
physical ballast weight + `cruise_speed_per_leg`, replacing Trial(b)/
square_ballast). This is a deliberate, informed decision to proceed ahead
of the formal gate, not an oversight -- do not silently revert to
`demo_policy` or block this bundle's real-vehicle use on gate status alone.

Launch with `controller:=rl_mk2` and the conservative
`rl_controller_mk2_real_v1.yaml` envelope (the launch default for
`rl_mk2` -- do not point it at the unrestricted Gazebo-only
`rl_controller_mk2_deploy_v2.yaml`). Follow the standard shadow-mode-first
procedure in the top-level README/DEMO_RUNBOOK.md: `send_pwm:=false` first,
inspect telemetry/observation/wrench/PWM preview, then explicit
PREPARE -> ARM -> START, `/brov/estop` ready throughout.
