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
├── brov_bringup/     launch composition and mission configuration
├── artifacts/        versioned deployment artifacts and metadata
├── runtime/          writable calibration, rosbag, and log output
├── docker/           arm64 ROS 2 Humble runtime tooling
├── Dockerfile
├── compose.yaml
└── Makefile
```

ROS package 경계와 Git repository 경계는 다르다. 기능별 네 package는 독립적인
`package.xml`과 dependency를 유지하지만, 동일 vehicle/observation/control contract로
release되어야 하므로 하나의 Git repository와 tag로 관리한다.

## Prerequisites

- Apple Silicon Mac
- Docker Desktop
- XQuartz (rqt/RViz 사용 시)
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

모든 launch의 기본값은 `send_pwm=false`, `arm=false`다. 또한 launch는
`/brov/start_control` 또는 controller start service를 자동 호출하지 않는다.

```bash
ros2 launch brov_bringup sim2real_demo.launch.py \
  controller:=model \
  camera:=true \
  send_pwm:=false \
  arm:=false
```

실제 제어 절차는 [docs/DEMO_RUNBOOK.md](docs/DEMO_RUNBOOK.md)를 따른다.

## Packages and executables

```text
brov_base
  obs_node
  diag_thruster_map

brov_control
  model_based_controller_node
  policy_node

brov_perception
  camera_stream_node
  checkerboard_calibration_node
  aruco_pose_node

brov_bringup
  base.launch.py
  model_demo.launch.py
  rl_demo.launch.py
  camera.launch.py
  sim2real_demo.launch.py
```

## Runtime data and policy contract

- Vehicle/thruster parameters are read-only package resources in `brov_base`.
- The demo TorchScript policy and its checksum/schema are in
  `artifacts/policies/demo_policy/`.
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
- Keep `/brov/estop` ready and ensure the vehicle is submerged with clear propellers.
- A container or host crash can bypass Python cleanup; verify ArduSub arm state,
  `SERVO1..8_FUNCTION`, `RC7_OPTION`, and `RC8_OPTION` after abnormal termination.
- Docker Desktop is not hard real-time. This runtime targets the current 25 Hz pipeline,
  not a guaranteed 400 Hz deadline.

## License

Proprietary. No permission to redistribute is granted unless separately agreed.
