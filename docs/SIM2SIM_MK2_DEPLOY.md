# MK2 sim2sim deployment

> **[2026-08-28] 이 문서가 언급하는 `sim2swim_deploy_v2`~`v6` 번들은 저장소에서 제거됐다.**
> 아래 내용은 그 시기의 실험 기록이므로 이름을 그대로 둔다 — 지우면 기록이 깨진다.
> 그 계보는 논문 Eq.(8)의 `w_a = 0.3` 아래에서 나온 **나쁜 draw 하나를 다섯 번
> 패치한 것**이고, 원인이 보상으로 확정되면서 후보 자격을 잃었다(시드 재현율 1/5,
> `w_a`만 0.017로 낮추면 5/5·Fig.4 3/3 통과). 현재 배포 후보는
> `artifacts/policies/sim2swim_paperfix_wa0017_mk2_s42_i299/` 하나다.
> 복원이 필요하면 `git checkout eae2ed7^ -- artifacts/policies/sim2swim_deploy_v5_mk2_s42_i299`.

The MK2 path is isolated from the legacy `demo_policy` path.

## Status (updated 2026-08-18 -- older than this is stale)

This page originally described `sim2swim_deploy_v2_mk2_s42_i49` (the first,
50-iteration MK2 checkpoint), which was ~99% action-saturated and rejected.
That checkpoint is long superseded. Four more candidates exist now, each in
its own `artifacts/policies/sim2swim_deploy_v{2,3,4,5}_mk2_s42_i299/`
bundle with its own README documenting exactly what was tried and measured
-- read the bundle's own README before trusting any summary here, including
this one.

Deployment mechanics (artifact-contract verification, T6 transform,
fail-closed loading) are verified and unchanged across all of them. Control
performance improved substantially in sequence:

- v2 (i299): ~99% whole-cycle action-bound saturation, matches i49.
- v3 (item A, episode redesign): no improvement (~99%, force-clamp worse).
- v4 (A + DVL/thruster domain randomization): action-bound down to
  ~79-89%, force-clamp ~17-19%.
- v5 (A+B+C + raw-actor-overflow penalty): action-bound down to ~50-55%,
  force-clamp ~1-2%. Pitch axis remains the least-improved (~42-45%,
  plausibly a torque-budget ceiling, not yet confirmed).

None of v2-v5 have passed the formal Gazebo Case-A gate (strict absolute
thresholds, <1%/<5% depending on window). **v5 was nonetheless explicitly
chosen by the user for a real-vehicle first-water test** (case a-2, see
`docs/SIM2SWIM_DEMO.md`) as an informed decision, not an oversight -- see
that bundle's README for the full record. Do not treat "gate not passed"
as equivalent to "rejected" for v5; it is a known, accepted, documented
risk for that specific real-vehicle test only. Do not promote any of
v2-v5 to replace `demo_policy` as a default without a separate decision.

Full metrics, the Case-A methodology, and the Case-C (5 m square,
random-attitude-per-corner) cross-validation that reproduced the same
improvement pattern are recorded in the training workspace's
`project_step2_brov_retrain_spec` notes and the "Found It" / "It
Generalizes" result pages, not in this repository.

## Runtime components

- artifact: one of `artifacts/policies/sim2swim_deploy_v{2,3,4,5}_mk2_s42_i299/
  policy_raw_flu_mk2.pt` -- pick per the "Status" section above and each
  bundle's own README, not by habit or by what an older example used.
- executable: `brov_control policy_node_mk2`
- launch: `brov_bringup sim2sim_mk2_case_a.launch.py`
- mission: `mission_sim2sim_mk2_case_a_0p5.yaml`
- action contract: `explicit_flu_zup_to_sname_frd_v1`
- transform before allocation: `T6=diag(1,-1,-1,1,-1,-1)`

The node verifies the sibling metadata, policy SHA, vehicle SHA, observation
schema and action contract before loading the policy.  Missing or unknown
contracts fail closed.

## Build and preflight

```bash
cd /home/bluerov2_sitl/brov_ros2
source /opt/ros/humble/setup.bash
source /home/bluerov2_sitl/colcon_ws/install/setup.bash
colcon build \
  --build-base build_mk2 \
  --install-base install_mk2 \
  --log-base log_mk2 \
  --symlink-install
source install_mk2/setup.bash

export BROV_MK2_POLICY_PATH=$PWD/artifacts/policies/\
sim2swim_deploy_v5_mk2_s42_i299/policy_raw_flu_mk2.pt
python3 docker/check_environment.py
```

## ROS launch

With Gazebo, ArduSub, the GT bridge and optional DVL injection already running:

```bash
ros2 launch brov_bringup sim2sim_mk2_case_a.launch.py \
  connection:=udpin:0.0.0.0:14552 \
  feedback_source:=gazebo_truth \
  start_gazebo_truth_bridge:=false \
  policy_path:=$BROV_MK2_POLICY_PATH \
  send_pwm:=false arm:=false
```

Use `feedback_source:=mavlink_ekf` for the DVL/EKF arm.  Keep output and arming
disabled until the artifact-contract topic, one-publisher authority and
telemetry gates have passed.  The full fresh-server/EEPROM/rosbag harness is
`step_2_BROV/run_mk2_case_a_deploy_host.sh` in the training workspace.

This launch is a direct-relative 2 m Case-A-shaped causal-isolation profile;
it is not the camera/pool-localized production Sim2Swim orchestrator.

