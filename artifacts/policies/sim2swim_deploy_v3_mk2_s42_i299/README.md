# Sim2Swim deploy-v3 MK2 policy (mission-scale episode retrain)

This bundle is separate from `demo_policy`, `sim2swim_deploy_v2_mk2_s42_i49`
(50-iteration deploy_v2), and `sim2swim_deploy_v2_mk2_s42_i299`
(300-iteration deploy_v2).

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD
  allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/
  observation contracts, wrench scale, and vehicle-model SHA all match.
  `policy_contract.py` now accepts `profile` in `("deploy_v2", "deploy_v3")`
  for the MK2 action contract (previously hardcoded to `deploy_v2` only —
  widened because deploy_v3 shares the identical observation/action
  contract, only the desired-state curriculum and integral-reset behavior
  differ).

## Provenance

Root-cause diagnosis (see `step_2_BROV/project_step2_brov_jitter_investigation`
and `project_step2_brov_retrain_spec` memory) found that `z_v`/`z_q`
(observation indices 10:16, the velocity/attitude error integrals) diverge
catastrophically outside their training range during long real missions,
because deploy_v2 only ever trained on 5 s episodes. `deploy_v3` addresses
this directly:

- `episode_length_s`: 5s -> 30s
- Command curriculum: `DeployV2Scheduler`'s single mid-episode transition ->
  `DeployV3Scheduler`'s multi-leg (up to 16 legs) schedule spanning the full
  episode, each leg with an independently sampled body-velocity command and
  a 50% chance of a fresh attitude retarget.
- `z_v`/`z_q` now reset on every leg transition (bumpless transfer),
  mirroring the deployment-side waypoint-transition reset -- gated by
  `reset_integral_on_command_transition`, which is only enabled for
  deploy_v3 (deploy_v2's existing behavior and checkpoints are unaffected).

Trained 2048 envs x 128-step rollout x 300 iterations, seed 42, same
hardware/profile scale as the deploy_v2 i299 run it is being compared
against.

## Deployment status: pending Gazebo Case-A A/B

Not yet evaluated in Gazebo at the time this bundle was created. Compare
against `sim2swim_deploy_v2_mk2_s42_i299`'s rejected-candidate numbers (GT
outbound vector RMSE 0.355 m/s, whole-cycle action-bound 98.9%/99.4% across
i49/i299, force-clamp 30-40%) once the Case-A GT/DVL-EKF runs complete.

Keep this bundle for diagnosis and retraining comparison only until it
passes the control-performance gate. It must not replace `demo_policy`, be
promoted as the Gazebo default, or be used on the real vehicle.
