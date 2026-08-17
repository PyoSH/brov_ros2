# Sim2Swim deploy-v2 MK2 policy

This bundle is intentionally separate from `demo_policy` / model_299.

- Run only with `brov_control/policy_node_mk2`.
- The raw actor action is body FLU/Z-up.
- `policy_node_mk2` applies
  `T6=diag(1,-1,-1,1,-1,-1)` before the SNAME/FRD allocation matrix.
- Startup fails unless the sibling JSON, policy SHA, profile, action/observation
  contracts, wrench scale, and vehicle-model SHA all match.

## Deployment status: rejected candidate

The 2026-08-17 fresh-Gazebo Case-A-shaped GT/DVL-EKF pair completed the
takeoff/outbound/180-degree-turn/return lifecycle, and the runtime policy SHA,
TorchScript replay, and T6 action-to-wrench contract all matched.  The control
performance gate did **not** pass: whole-cycle action-bound occupancy was about
99% and requested-force clamping was about 30% with GT feedback and 48% with
DVL-EKF feedback.

Keep this bundle for diagnosis and retraining comparison only.  It must not
replace `demo_policy`, be promoted as the Gazebo default, or be used on the
real vehicle.  The authoritative analysis is
`step_2_BROV/MK2_SIM2SIM_DEPLOY_RESULT.md` in the training workspace.
