# Sim2Swim deploy-v2 MK2 policy (300-iteration retrain)

This bundle is intentionally separate from `demo_policy` / `model_299`
(legacy) and from `sim2swim_deploy_v2_mk2_s42_i49` (the 50-iteration
deploy_v2 retrain evaluated 2026-08-17).

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies
  `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/observation
  contracts, wrench scale, and vehicle-model SHA all match.

## Provenance

Trained from the same `deploy_v2` profile/config as
`sim2swim_deploy_v2_mk2_s42_i49` (2048 envs, 128-step rollout, seed 42), but
carried to `max_iterations=300` instead of 50
(`step_2_BROV/logs/stage3_deploy_v2_2048x128_seed42_i300_20260817/model_299.pt`).
Purpose: test whether the i49 candidate's MK2/Gazebo control-performance
rejection (~99% whole-cycle action-bound occupancy, ~30% force clamp with GT
feedback) was caused by insufficient training rather than a structural
policy/reward defect.

Isaac-native Fig.4 evaluation of this checkpoint showed steady-state metrics
statistically indistinguishable from the i49 checkpoint (steady action-bound
occupancy 0% and zero opposite-bound flips in both), and the training reward
curve had already plateaued by iteration ~150-200 — i.e., the extra 250
iterations did not change the learned policy's Isaac-native behavior in any
meaningful way. This bundle exists to confirm (or refute) that same
insufficiency hypothesis in Gazebo.

## Deployment status: rejected candidate (confirmed 2026-08-17)

Fresh Gazebo Case-A GT and Water-Linked-aligned DVL-EKF runs
(`runtime/experiments/sim2sim_mk2_case_a_i300b_{gazebo_truth,mavlink_ekf}`)
both completed the takeoff/outbound/turn/return lifecycle with the artifact
contract verified (`mk2_artifact_exact_match: true`, T6/wrench identity
residual ~0). Control performance is statistically the same failure as
`sim2swim_deploy_v2_mk2_s42_i49` (50-iteration checkpoint), not better:

| metric (whole cycle) | i49 (50 iter) | i299 (300 iter) |
|---|---:|---:|
| GT any-axis action-bound fraction | 98.9% | 99.4% |
| DVL-EKF any-axis action-bound fraction | 98.4% | 99.1% |
| GT thruster force-clamp fraction | 30.2% | 40.4% |
| DVL-EKF thruster force-clamp fraction | 47.5% | 57.8% |
| GT outbound vector velocity RMSE (m/s) | 0.355 | 0.344 |
| GT return vector velocity RMSE (m/s) | 0.254 | 0.266 |
| DVL-EKF outbound vector velocity RMSE (m/s) | 0.471 | 0.485 |
| DVL-EKF return vector velocity RMSE (m/s) | 0.422 | 0.421 |

This confirms the "insufficient training" hypothesis this bundle was created
to test is **false**: 300 iterations reproduces (and on the saturation/clamp
metrics, slightly worsens) the same near-total actuator-saturation failure as
50 iterations, consistent with the Isaac-native finding that the reward curve
had already plateaued by iteration ~150-200. The root cause remains
structural (policy raw output saturates outside `[-1,1]` on Gazebo-realistic
observations; see `step_2_BROV/MK2_SIM2SIM_DEPLOY_RESULT.md` §6-8), not
training duration.

Keep this bundle for diagnosis and retraining comparison only. It must not
replace `demo_policy`, be promoted as the Gazebo default, or be used on the
real vehicle.
