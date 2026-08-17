# MK2 sim2sim deployment

The MK2 path is isolated from the legacy `demo_policy` path.

## Status

The deployment mechanics are verified, but the current policy candidate is
**rejected for control performance**.  Fresh Gazebo GT and no-GPS
Water-Linked-aligned DVL/EKF runs both completed the
takeoff/outbound/turn/return lifecycle.  The GT run nevertheless spent about
99% of the cycle with at least one bounded action and clamped requested
thruster force about 30% of the time.  Do not use this policy on the real
vehicle and do not replace `demo_policy` with it.

The full metrics and causal checks are recorded in the training workspace at
`step_2_BROV/MK2_SIM2SIM_DEPLOY_RESULT.md`.

## Runtime components

- artifact:
  `artifacts/policies/sim2swim_deploy_v2_mk2_s42_i49/policy_raw_flu_mk2.pt`
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
sim2swim_deploy_v2_mk2_s42_i49/policy_raw_flu_mk2.pt
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

