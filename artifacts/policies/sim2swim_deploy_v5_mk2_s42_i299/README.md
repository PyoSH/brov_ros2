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

## Deployment status: pending Gazebo Case-A A/B

Not yet evaluated in Gazebo at the time this bundle was created. Compare
against `sim2swim_deploy_v4_mk2_s42_i299` (whole-cycle action-bound GT
78.9%/DVL 88.6%, force-clamp GT 17.7%/DVL 19.3%) once the Case-A GT/DVL-EKF
runs complete, and re-run `diagnose_attitude_torque_budget.py` on the new
bags to check whether pitch/yaw saturation and body-rate oscillation
actually dropped -- that is the metric item D was designed to move, not
just the aggregate whole-cycle fraction.

Keep this bundle for diagnosis and retraining comparison only until it
passes the control-performance gate. It must not replace `demo_policy`, be
promoted as the Gazebo default, or be used on the real vehicle. For any
real-vehicle MK2 test regardless of which profile passes the Gazebo gate,
use the conservative `rl_controller_mk2_real_v1.yaml` envelope
(`action_abs_limit`/`pwm_abs_limit`/`pwm_slew_rate_per_s` all well below the
unrestricted Gazebo-only `rl_controller_mk2_deploy_v2.yaml`), not the SITL
config.
