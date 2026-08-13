# Sim2Real demo runbook

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

## 4. Model-based control

After shadow-mode inspection, restart the launch in a safe water-test setup:

```bash
ros2 launch brov_bringup model_demo.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true arm:=true
```

The launch only prepares the nodes. Start control explicitly:

```bash
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/model_based/start std_srvs/srv/Trigger "{}"
```

Both responses must return `success=True`.

## 5. RL control

Stop the model launch completely, then start the RL launch:

```bash
ros2 launch brov_bringup rl_demo.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true arm:=true
```

The policy path defaults to `BROV_POLICY_PATH` in Docker. RL has no separate controller
start service; open only the observation/PWM gate:

```bash
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

## 7. Normal stop

Model controller:

```bash
ros2 service call /brov/model_based/stop std_srvs/srv/Trigger "{}"
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
```

RL controller:

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
```

Stop the controller first and `obs_node` last so normal cleanup can disarm, release RC
override, and restore servo/camera parameters.
