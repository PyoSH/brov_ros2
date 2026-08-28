# Sim2Real demo runbook

> **[2026-08-28] 이 문서가 언급하는 `sim2swim_deploy_v2`~`v6` 번들은 저장소에서 제거됐다.**
> 아래 내용은 그 시기의 실험 기록이므로 이름을 그대로 둔다 — 지우면 기록이 깨진다.
> 그 계보는 논문 Eq.(8)의 `w_a = 0.3` 아래에서 나온 **나쁜 draw 하나를 다섯 번
> 패치한 것**이고, 원인이 보상으로 확정되면서 후보 자격을 잃었다(시드 재현율 1/5,
> `w_a`만 0.017로 낮추면 5/5·Fig.4 3/3 통과). 현재 배포 후보는
> `artifacts/policies/sim2swim_paperfix_wa0017_mk2_s42_i299/` 하나다.
> 복원이 필요하면 `git checkout eae2ed7^ -- artifacts/policies/sim2swim_deploy_v5_mk2_s42_i299`.

이 문서는 `sim2real_demo.launch.py`의 legacy `model`/`rl`(`policy_node`)
controller 경로를 다룬다. MK2 policy(`policy_node_mk2`, `controller:=rl_mk2`,
`sim2swim_deploy_v2..v5_mk2_s42_i299` artifact), pool-localized 배치, Case
A/A2/C Sim2Swim 데모는 이 문서가 아니라
[SIM2SWIM_DEMO.md](SIM2SWIM_DEMO.md)를 따른다. Preflight, estop, build/test,
camera/ArUco 절차(1, 2, 6, 7절)는 controller 종류와 무관하게 공통이다.

## 1. Preflight

- Vehicle submerged; propellers and tether clear.
- Physical power cut-off and `/brov/estop` operator ready.
- QGroundControl not simultaneously using this MAVLink control endpoint.
- ArduSub `MANUAL(19)` mode.
- DVL ExternalNav and `LOCAL_POSITION_NED` healthy.
- BlueOS MAVLink server reachable at `192.168.2.2:14550`.
- Camera endpoint targets the Mac tether IP on UDP 5600.

비상 정지 명령은 `obs_node`가 실행 중일 때만 수신된다. 아래 명령을 별도 터미널에
준비해 두고, 제어 중 이상 동작이 발생했을 때 실행한다(launch 전에 미리 발행해도
estop이 latch되지 않는다).

```bash
ros2 topic pub --once /brov/estop std_msgs/msg/Empty "{}"
```

## 2. Build and environment check

```bash
make build
make test
make check
make shell
```

## 3. Shadow-mode full bringup

```bash
ros2 launch brov_bringup sim2real_demo.launch.py \
  controller:=model camera:=true send_pwm:=false arm:=false
```

Confirm:

```bash
ros2 topic hz /brov/observation
ros2 topic echo --once /brov/debug/pos_mission
ros2 topic echo --once /brov/model_based/thruster_pwm_preview
ros2 topic hz /brov/camera/image_raw
```

RL shadow launch에서는 model preview 대신 아래를 확인한다. policy preview는
항상 계산되지만 base START 전에는 policy node가 실제 PWM topic을 발행하지 않는다.

```bash
ros2 topic echo --once /brov/action
ros2 topic echo --once /brov/policy/thruster_pwm_preview
```

수조 좌표의 절대 위치·자세 초기화와 pool waypoint를 사용하는 데모는 위의 legacy
launch 대신 다음 fail-closed profile을 사용한다.

```bash
ros2 launch brov_bringup pool_localized_demo.launch.py \
  controller:=model send_pwm:=false arm:=false
```

이 profile은 camera+AprilTag, one-shot localization, mission manager와 정확히 한
controller를 구성하지만 tilt confirm, initialize, validate, commit, prepare, arm 또는
start를 자동 호출하지 않는다. Optional visualization은 `rviz:=true`로만 포함되며
기본값은 `false`다.
새 session마다 수행할 전체 순서는
[POOL_LOCALIZATION_RUNBOOK.md](POOL_LOCALIZATION_RUNBOOK.md)를 따른다.

## 4. Model-based control

After shadow-mode inspection, restart the launch in a safe water-test setup:

```bash
ros2 launch brov_bringup model_demo.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true arm:=true
```

`arm=true` permits explicit ROS arming but does not arm during launch. This
legacy relative-mission profile has no committed pool mission to PREPARE, so
explicitly ARM then START and check `success=True` after every response:

```bash
ros2 service call /brov/arm_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/model_based/start std_srvs/srv/Trigger "{}"
```

ARM rechecks gates and sends neutral before arming. START opens the control gate
but never arms. The full pool-localized profile adds PREPARE before these steps.

## 5. RL control (legacy `policy_node` contract only)

This section is the legacy `rl_demo.launch.py` / `policy_node` path
(`demo_policy` artifact, no T6 action-frame transform). It is not the MK2
contract used for the current Sim2Swim work -- for `policy_node_mk2` /
`controller:=rl_mk2` / any `sim2swim_deploy_v*_mk2_*` artifact, use
[SIM2SWIM_DEMO.md](SIM2SWIM_DEMO.md) instead; the two controllers are not
interchangeable and must never run simultaneously.

Stop the model launch completely, then start the RL launch:

```bash
ros2 launch brov_bringup rl_demo.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true arm:=true
```

The policy path defaults to `BROV_POLICY_PATH` in Docker. RL has no separate
controller start service, but the legacy ARM → START lifecycle is still required.
The node starts with output disabled, keeps publishing
`/brov/policy/thruster_pwm_preview`, and forwards to `/brov/thruster_pwm` only
after `/brov/control_active=true`:

```bash
ros2 service call /brov/arm_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
```

The included policy must be revalidated on the real vehicle after the quaternion and
depth-guidance fixes documented in its metadata.

## 6. Camera calibration and ArUco

Collect intrinsic calibration samples:

```bash
ros2 launch brov_bringup camera.launch.py calibrate:=true
```

The result is written to `$BROV_DATA_DIR/calibration/camera_intrinsics.yaml`. Restart the
camera node, then enable ArUco:

```bash
ros2 launch brov_bringup camera.launch.py aruco:=true
```

Do not enable ArUco and calibration simultaneously.

The default reference configured in `brov_perception/config/aruco.yaml` is
AprilTag `16h5`, integer ID `2`. Its nominal 70 mm cell gives a 420 mm outer
black-edge length; the surrounding white quiet zone is physically required but
is not part of `marker_length_m`. Before metric testing, measure the finished
black edge and update the YAML if it differs from 0.420 m.

Keep the vehicle control stopped and lock camera tilt at its calibrated neutral
position. Verify the loaded contract and relative camera-to-marker output:

```bash
ros2 param get /brov_aruco_pose_node dictionary
ros2 param get /brov_aruco_pose_node marker_id
ros2 param get /brov_aruco_pose_node marker_length_m
ros2 topic echo /brov/aruco/visible
ros2 topic echo --once /brov/aruco/marker_pose
ros2 topic echo --once /brov/aruco/robot_pose
ros2 topic echo --once /brov/aruco/robot_pose_pool
rqt_image_view /brov/aruco/debug_image
```

`/brov/aruco/marker_pose` is expressed in `camera_optical_frame` (+x right,
+y down, +z forward). `/brov/aruco/robot_pose` is the `base_link` pose relative
to the observed marker. It uses the nominal camera position from
`/BROV2_Heavy/Camera_frame` in `brov2_custom_physics.usda`; USD camera rotation
is ignored and the orientation is derived from the CV optical convention.

The model-derived extrinsic assumes `base_link` equals the USD robot root and
is valid only with tilt locked neutral. The deployed marker survey fixes the
black-square centre at pool `[3.95, 0.85, 0.35]` m, with printed page top along
pool `+Z` and the marker face normal along pool `-X`. Runtime OpenCV axes show
decoded marker `+X=pool +Y`, `+Y=pool -Z`, and `+Z=pool -X`; this measured axis
contract, rather than the page label alone, defines the configured quaternion.
Therefore
`/brov/aruco/robot_pose_pool` is the raw, single-frame `base_link` pose in the
Z-up `pool` frame. It is not filtered or fused and must not directly drive the
controller. The perception node intentionally broadcasts neither the raw
`camera -> marker` TF nor canonical `marker -> base_link`/`pool -> base_link`
TFs (`publish_marker_tf: false`, `publish_robot_tf: false`).

## 7. RViz pool-frame verification

Keep the camera/AprilTag launch running and open a second container terminal:

```bash
ros2 launch brov_viz pool_vision.launch.py
```

The interactive window currently requires a compatible Linux display. The
macOS XQuartz path is suitable for rqt but is not qualified for RViz's
OGRE/OpenGL renderer; use the headless validation below until a dedicated
viewer bridge is provided.

The RViz fixed frame is `pool`. It displays the nominal 4.0 x 1.7 x 1.1 m
pool, the surveyed 420 mm tag, a translucent magenta raw-vision robot proxy and
a blue one-shot-aligned odometry proxy with FLU axes. An out-of-pool proxy is
red, and each robot proxy is deleted when its input becomes stale. The AprilTag
debug image is shown in the same RViz window.

This launch starts no camera, controller, MAVLink owner, TF broadcaster, arm,
or control service. The canonical `pool -> odom -> base_link` ownership chain
is implemented by `brov_localization` and `brov_base`; this visualization node
only converts their existing pool-frame measurements into RViz markers. The
current RViz config uses the Identity transformer for those already-pool-frame
markers, so do not add untransformed data from other frames.

Headless topic validation is also available:

```bash
ros2 launch brov_viz pool_vision.launch.py rviz:=false
ros2 topic echo --once /brov/viz/pool
ros2 topic echo --once /brov/viz/vision_robot
ros2 topic echo --once /brov/viz/localized_robot
```

## 8. Normal stop

Model controller:

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/model_based/stop std_srvs/srv/Trigger "{}"
ros2 service call /brov/disarm_control std_srvs/srv/Trigger "{}"
```

RL controller:

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/disarm_control std_srvs/srv/Trigger "{}"
```

Close the PWM gate first, stop the controller publisher, then explicitly disarm.
Terminate `obs_node` last so cleanup can release RC override and restore
servo/camera parameters.
