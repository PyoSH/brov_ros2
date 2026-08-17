# BROV ROS 2

BlueROV2 Heavy 실기체의 MAVLink interface, waypoint guidance, model/RL control,
BlueOS camera 및 ArUco perception을 위한 독립 ROS 2 Humble 저장소다.

학습·시뮬레이션 코드와 runtime을 분리한다. 학습 환경은 TorchScript policy artifact만
전달하며, 이 저장소는 IsaacLab이나 RSL-RL 없이 실기체에서 실행된다.

## Repository structure

```text
brov_ros2/
├── brov_base/        MAVLink, observation, guidance, PWM gateway, vehicle model
├── brov_control/     model-based controller and TorchScript policy runtime
├── brov_perception/  BlueOS camera, checkerboard calibration, ArUco pose
├── brov_interfaces/  typed localization and immutable mission contracts
├── brov_localization/ one-shot full-SE(3) pool-to-odom alignment
├── brov_mission/     pool-frame draft validation and odom mission resolution
├── brov_viz/         pool, raw vision, and aligned odometry visualization
├── brov_bringup/     launch composition and mission configuration
├── artifacts/        versioned deployment artifacts and metadata
├── runtime/          writable calibration, rosbag, and log output
├── docker/           arm64 ROS 2 Humble runtime tooling
├── Dockerfile
├── compose.yaml
└── Makefile
```

ROS package 경계와 Git repository 경계는 다르다. 기능별 package는 독립적인
`package.xml`과 dependency를 유지하지만, 동일 vehicle/observation/control contract로
release되어야 하므로 하나의 Git repository와 tag로 관리한다.

## Prerequisites

- Apple Silicon Mac
- Docker Desktop
- XQuartz (rqt 사용 시; 현재 macOS 경로의 RViz/OGRE는 qualified되지 않음)
- BlueOS MAVLink/video endpoint가 Mac tether IP를 향하도록 설정

현재 실기체에서 검증된 Docker Desktop network 설정:

```text
Enable host networking:        ON
Use kernel networking for UDP: OFF
```

## Build and test

```bash
git clone https://github.com/PyoSH/brov_ros2.git
cd brov_ros2
docker compose build
make build
make test
make check
make shell
```

`make shell`은 ROS Humble과 `/workspace/brov_ros2/install` overlay를 자동으로
source한다. 수동 `PYTHONPATH`나 다른 source repository mount는 필요하지 않다.

이 저장소에서는 Dockerfile이 runtime dependency의 기준이다. 특히 Apple Silicon용
PyTorch CPU wheel은 PyTorch index에서 설치하므로, `rosdep check`만 단독 실행하면
APT의 `python3-torch`가 없다고 표시할 수 있다. 실제 환경 검증은 `make check`로 한다.

## Safe first launch

모든 launch의 기본값은 `send_pwm=false`, `arm=false`다. `arm=true`는 명시적인
`/brov/arm_control` 호출을 허용할 뿐 자동 arm 요청이 아니다. Launch는 tilt 확인,
localization/mission transaction 또는 PREPARE/ARM/START service를 자동 호출하지 않는다.
`pool_localized_demo.launch.py`의 optional RViz도 기본 `false`다.

```bash
ros2 launch brov_bringup sim2real_demo.launch.py \
  controller:=model \
  camera:=true \
  send_pwm:=false \
  arm:=false
```

실제 제어 절차는 [docs/DEMO_RUNBOOK.md](docs/DEMO_RUNBOOK.md)를 따른다.
Sim2Swim은 기존 case `a`/`c`의 `start_heading` mission과 16-D policy contract를
유지하면서 AprilTag one-shot full-SE(3) pool pose 초기화를 control-start gate로
사용한다. 수조 배치, 초기화 및 별도 안전 gate는
[docs/SIM2SWIM_DEMO.md](docs/SIM2SWIM_DEMO.md)를 따른다.

현재 추진기는 검증된 RCPassThru/`RC_CHANNELS_OVERRIDE` backend로 구동한다.
후배가 작성한 custom ArduPilot mode로 이관하기 위한 입력 계약, 구현 단계 및
안전 승인 기준은
[docs/ACTUATION_BACKEND_ROADMAP.md](docs/ACTUATION_BACKEND_ROADMAP.md)에 기록한다.
해당 firmware와 interface가 전달되기 전에는 추측한 mode 코드를 추가하지 않는다.

현재 raw ArUco pool pose는 `brov_viz`에서 수조/marker와 함께 RViz로 확인할 수
있다. one-shot full-SE(3) 절대 pose 초기화와 pool mission 운용 방법은
[docs/POOL_LOCALIZATION_RUNBOOK.md](docs/POOL_LOCALIZATION_RUNBOOK.md)를 따른다.
InteractiveMarker 기반 3-D waypoint 편집 및 continuous fusion의 후속 방법은
[docs/RVIZ_POOL_LOCALIZATION_ROADMAP.md](docs/RVIZ_POOL_LOCALIZATION_ROADMAP.md)에
기록한다. 3-D reconstruction과 NBV 시각화·실행 단계는
[docs/NBV_RECONSTRUCTION_ROADMAP.md](docs/NBV_RECONSTRUCTION_ROADMAP.md)에
분리해 기록한다. continuous localization fusion과 waypoint editor는 여전히
roadmap 범위다.

IsaacLab 학습/평가와 Gazebo SITL·실기 배포 사이에서 확인된 차이, rosbag 관측,
Ubuntu 22.04/ROS 2 Humble 환경의 fresh clone/build 절차, sim2sim 분해 실험 및
재학습 요구사항은
[docs/SIM2SIM_RETRAINING_HANDOFF.md](docs/SIM2SIM_RETRAINING_HANDOFF.md)에 기록한다.

## Packages and executables

```text
brov_base
  obs_node
  diag_thruster_map

brov_control
  model_based_controller_node
  policy_node        # legacy contract, no T6 action-frame transform
  policy_node_mk2    # MK2 contract (all sim2swim_deploy_v2..v5_mk2_* artifacts)

brov_perception
  camera_stream_node
  checkerboard_calibration_node
  aruco_pose_node

brov_interfaces
  OdometrySession.msg
  AlignedOdometry.msg
  LocalizationStatus.msg
  ResolvedMission.msg
  InitializePool.srv

brov_localization
  pool_alignment_node

brov_mission
  mission_manager_node

brov_viz
  pool_scene_node
  pool_vision.launch.py

brov_bringup
  base.launch.py
  model_demo.launch.py
  rl_demo.launch.py
  camera.launch.py
  pool_localized_demo.launch.py
  sim2real_demo.launch.py
  sim2swim_demo.launch.py
```

## Runtime data and policy contract

- Vehicle/thruster parameters are read-only package resources in `brov_base`.
- The legacy demo TorchScript policy and its checksum/schema are in
  `artifacts/policies/demo_policy/` (runs with `policy_node`,
  `controller:=rl`).
- MK2-contract policy bundles (`sim2swim_deploy_v2_mk2_s42_i299` through
  `_v5_mk2_s42_i299` at the time of writing) live alongside it under
  `artifacts/policies/`, one directory per bundle, each with its own
  `README.md` documenting Gazebo Case-A/Case-C validation status. These run
  with `policy_node_mk2`, `controller:=rl_mk2` -- see
  `docs/SIM2SWIM_DEMO.md` for the controller-selection gotcha (the launch
  default is the legacy `rl` controller, not `rl_mk2`).
- Calibration, bags, and logs are written under `runtime/` and excluded from Git.
- Docker defines `BROV_DATA_DIR=/workspace/brov_ros2/runtime` and
  `BROV_POLICY_PATH` for the included demo policy.

The current demo policy expects a `(1,16)` observation and returns a `(1,6)` action.
Its metadata must be updated together with the policy, vehicle model, observation schema,
and wrench scaling.

## Safety notes

- Never run the model and RL PWM publishers simultaneously.
- Start in shadow mode and inspect telemetry, observation, wrench, PWM preview, and actual
  servo output before enabling control.
- Follow explicit PREPARE → ARM → START responses; `start_control` never performs arming.
- Keep `/brov/estop` ready and ensure the vehicle is submerged with clear propellers.
- A container or host crash can bypass Python cleanup; verify ArduSub arm state,
  `SERVO1..8_FUNCTION`, `RC7_OPTION`, and `RC8_OPTION` after abnormal termination.
- Docker Desktop is not hard real-time. This runtime targets the current 25 Hz pipeline,
  not a guaranteed 400 Hz deadline.

## License

Proprietary. No permission to redistribute is granted unless separately agreed.
