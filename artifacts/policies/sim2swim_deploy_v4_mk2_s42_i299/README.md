# Sim2Swim deploy-v4 MK2 policy (A+B+C combined retrain)

This bundle is separate from `demo_policy`, `sim2swim_deploy_v2_mk2_s42_i299`
(300-iteration deploy_v2, mission-scale-episode-blind baseline), and
`sim2swim_deploy_v3_mk2_s42_i299` (item A only: mission-scale episode +
bumpless-transfer integral reset).

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD
  allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/
  observation contracts, wrench scale, and vehicle-model SHA all match.
  `policy_contract.py` now accepts `profile` in
  `("deploy_v2", "deploy_v3", "deploy_v4")` for the MK2 action contract
  (widened again for deploy_v4 -- same observation/action contract as v2/v3,
  only desired-state curriculum, integral-reset behavior, and domain
  randomization differ).

## Provenance

deploy_v3 (item A: mission-scale episode + bumpless-transfer integral reset)
closed the `z_v`/`z_q` training/deployment distribution gap by ~10x (±0.18
vs -2.06) but did **not** meaningfully reduce whole-cycle raw-actor
saturation in Gazebo Case-A (v2 i299: 99.4% -> v3: 99.7%, essentially
unchanged; see `project_step2_brov_retrain_spec` memory, "deploy_v3
full-scale Gazebo results"). This proved integral windup, while real and
dominant for the most extreme long-mission failures, is not the dominant
driver of the ~99% baseline saturation observed even in short Gazebo
cycles. `deploy_v4` adds two further items on top of A, targeting the
sim2sim/sim2real plant-mismatch hypothesis directly:

- **Item A** (carried over from v3): `episode_length_s` 5s -> 30s,
  `DeployV3Scheduler` multi-leg command curriculum (up to 48 legs),
  `reset_integral_on_command_transition=True`.
- **Item B** (DVL sensor realism): `DVLRealismModel` in the observation
  path only (reward keeps ground-truth velocity) -- sample-and-hold at a
  randomized 5-15 Hz DVL rate, 0-0.15 s randomized measurement delay via a
  per-env ring buffer, 0-0.006 m/s Gaussian noise added only on fresh
  samples. Trains the policy against the same staleness/noise/rate
  characteristics the real DVL-driven EKF exhibits, instead of the
  simulator's per-step ground-truth velocity.
- **Item C** (thruster domain randomization): vehicle-wide correlated
  voltage-sag scale (`dr_thrust_voltage_scale_range=(0.85,1.0)`) combined
  with an independent per-thruster variance scale
  (`dr_thrust_individual_scale_range=(0.90,1.10)`), applied multiplicatively
  to the RPM->thrust polynomial output in `BROV2ThrusterModel.compute()`
  before force/torque assembly.

Trained 2048 envs x 128-step rollout x 300 iterations, seed 42, same
hardware/profile scale as the deploy_v2/deploy_v3 runs it is being compared
against. Mean episode reward converged to ~818-828 by iteration 299.

## Deployment status: pending Gazebo Case-A A/B

Not yet evaluated in Gazebo at the time this bundle was created. Compare
against `sim2swim_deploy_v2_mk2_s42_i299` (whole-cycle action-bound 99.4%,
force-clamp 40.4%) and `sim2swim_deploy_v3_mk2_s42_i299` (action-bound
99.7%, force-clamp 83.3%) once the Case-A GT/DVL-EKF runs complete.

Keep this bundle for diagnosis and retraining comparison only until it
passes the control-performance gate. It must not replace `demo_policy`, be
promoted as the Gazebo default, or be used on the real vehicle.
