# Pool-localized Sim2Swim 실기체 데모

이 데모는 AprilTag로 매 실행의 로봇 시작 위치와 자세를 수조에 정렬하고, 사용자가
`pool` 절대좌표로 승인한 waypoint를 기존 TorchScript RL policy가 추종하게 한다.
`sim2swim_demo.launch.py`는 검증된 `pool_localized_demo.launch.py`의 RL branch를
재사용하며 다음 두 gate를 항상 켠다.

```text
require_pool_localization=true
require_resolved_mission=true
```

Launch만으로 camera-neutral 확인, localization 초기화, route commit, PREPARE, arm 또는
control start를 수행하지 않는다. `send_pwm=false`, `arm=false`가 기본값이다.

## 좌표계와 observation 계약

```text
pool (측량된 고정 Z-up frame: +X 먼 벽, +Y 좌측, +Z 위)
└── odom (현재 autopilot/DVL session의 연속 Z-up frame)
    └── base_link (FLU: +X 전방, +Y 좌측, +Z 위)
```

정지 상태에서 같은 시각의 vision과 local odometry를 사용해 다음 full-SE(3)를 한 번
계산하고 현재 odometry session 동안 고정한다.

```text
pool_T_odom = pool_T_base,vision * inverse(odom_T_base,mav)
```

사용자가 발행하는 `nav_msgs/Path`는 `pool` Z-up 절대좌표다. Mission manager는 승인
시점의 정확한 `pool_T_odom`, localization epoch, odometry session과 `alignment_id`에
경로를 결합하고 `odom`으로 한 번 resolve한다. PREPARE에서 `obs_node`가 그 불변
경로를 기존 내부 `start_heading` guidance 표현으로 바꾼다.

Vision pose를 policy 입력에 추가하지 않는다. 학습된 16-D observation은 그대로다.

```text
[q_error_wxyz(4), velocity_error_body(3), angular_velocity(3),
 velocity_error_integral(3), quaternion_vector_error_integral(3)]
```

따라서 새 observation 버전이나 policy 재학습 없이 수조 절대 경로를 사용할 수 있다.

## Case profile

### Case A: position-v1 / align

- contract: `brov_pool_position_mission_v1`
- pool waypoint: 정확히 2개
- `heading_mode=align`, `loop=true`
- speed 0.10 m/s, lookahead 0.40 m, reach 0.15 m
- 두 점 사이를 왕복하며 자동 종료가 없으므로 작업자가 정지한다.

기존 2.0 m 직선 실험을 수행하려면 첫 점 근처에 로봇을 놓고, 실제 pool bounds와
선체·tether 여유를 확인한 두 절대점을 입력한다.

### Case C: deterministic random-attitude-v2

- contract: `brov_pool_position_mission_v2`
- pool waypoint: 정확히 4개, 마지막 점에서 첫 점으로 닫히는 loop
- `heading_mode=random_at_waypoint`, `loop=true`
- speed 0.05 m/s, lookahead 0.15 m, reach 0.08 m
- deterministic seed와 exact generator contract
- shortest-path attitude slew 최대 0.174533 rad/s (10 deg/s)
- 자세/각속도 tolerance 안에서 1.0 s dwell 후 waypoint event 승인
- 1 lap 또는 active 60 s 중 먼저 도달한 조건에서 정상 control 종료
- policy action limit `[0.25, 0.25, 0.30, 0.15, 0.15, 0.15]`
- normalized PWM 절댓값 0.35, policy 변화율 0.40/s

V2 canonical plan에는 다음 metadata가 포함되며 전체가 mission hash에 들어간다.

```text
reference_frame = pool_zup_flu
generator_version = sha256_counter_uniform_rpy_v1
rpy bounds = roll/pitch ±0.261799 rad (15 deg), yaw ±0.523599 rad (30 deg)
```

같은 committed mission을 다시 PREPARE해도 random target sequence는 바뀌지 않는다.
새 sequence는 새 draft/commit transaction으로만 만든다.
이것은 legacy full-range(roll/pitch ±90 deg, yaw ±180 deg) 실험이 아닌 첫 수중
단계시험용 envelope다. 범위와 slew rate 확대는 구속 단일축 시험 결과를 검토한 뒤
별도 config 변경으로만 수행한다.

Generator의 바이트 수준 규약은 다음과 같다. 각 event와 축에 대해 trailing newline
없이 ASCII
`sha256_counter_uniform_rpy_v1:{seed}:{event_index}:{axis_index}`를 SHA-256
처리하고, digest의 첫 8 bytes를 unsigned big-endian 정수로 읽어 `2^64`로 나눈다.
각 placeholder는 `+` 기호와 zero padding이 없는 unsigned base-10 정수 문자열이다.
축 `0/1/2`는 각각 roll/pitch/yaw이며, `min + u * (max - min)`으로 bound에
사상한다. 이후 `Rz(yaw) Ry(pitch) Rx(roll)` ZYX Euler 조합을 정규화된
`[w,x,y,z]` quaternion으로 바꾸고, `w < 0`일 때만 전체 부호를 뒤집는다.

다음 값은 generator 정합성 검사용 legacy full-range golden vector이며, 위의 좁은
Case C 운용 bound에서 실제 생성되는 목표와는 다르다.

```text
seed=20260814, event_index=0
q_wxyz=[0.36995846, 0.15418720, 0.61874908, -0.67565274]
```

payload 문자, byte order, 축 순서, Euler 규약 또는 quaternion canonicalization을
바꾸려면 기존 version을 재해석하지 말고 새 `generator_version`을 정의해야 한다.

`case:=c`는 기본적으로 launch 단계에서 거부된다. `allow_case_c:=true`는 위험 profile을
구성해도 된다는 명시적 확인이지 물속 운항 승인 자체가 아니다. 현재 waypoint bounds는
로봇 중심만 검사하므로 다음 조건을 별도로 만족해야 한다.

- 모든 허용 자세에서 선체/추진기 swept volume과 수면·바닥·벽 여유 검증
- tether 걸림과 DVL bottom-lock 상실 가능성 검증
- 추진기 분리 및 구속 상태에서 낮은 자세 범위/authority부터 단계 시험
- 현장 estop 담당자와 QGroundControl disarm 담당자 지정

Case C launch는 위 operational envelope가 들어 있는 전용 RL config를 자동 선택한다.
또한 일반 bootstrap의 legacy random target 대신 1 cm straight/no-loop benign bootstrap을
사용하며, PREPARE에서만 committed v2 settings로 교체한다. 전용 gateway safety config도
PWM 절댓값 0.35와 gateway 변화율 0.50/s를 독립적으로 재검사한다. Policy의 실제
slew limiter는 더 엄격한 0.40/s라 정상 DDS/timer jitter가 같은 수치의 경계에서
false trip을 만들지 않게 margin을 둔다. 일반 `safety.yaml`이나
`rl_controller.yaml`로 바꾸거나 limit을 키워 이 gate를 우회하지 않는다.
Envelope 확대는 기록된 단계시험 결과와 시험 책임자 승인이 있는 별도 변경이다.

## 1. 빌드와 환경

```bash
cd /workspace/brov_ros2
colcon build --symlink-install --packages-select \
  brov_interfaces brov_base brov_control brov_perception \
  brov_localization brov_mission brov_viz brov_bringup
source install/setup.bash
```

Policy artifact를 지정한다.

```bash
export BROV_POLICY_PATH=/workspace/brov_ros2/artifacts/policies/demo_policy/policy.pt
```

다른 launch와 camera receiver가 남아 있지 않은지 확인한다. 실제 구동 전에는 estop
명령을 별도 터미널에 입력해 두고 Enter만 남겨 둔다.

```bash
ros2 topic pub --once /brov/estop std_msgs/msg/Empty "{}"
```

## 2. Shadow launch

먼저 실제 PWM과 ROS arm 권한을 모두 끈다.

```bash
# Case A
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=a \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=false \
  arm:=false \
  rviz:=false
```

Case C는 안전 acceptance 작업 중에도 explicit opt-in이 필요하다.

```bash
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=c \
  allow_case_c:=true \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=false \
  arm:=false \
  rviz:=false
```

두 case 모두 camera, AprilTag, localizer, mission manager, `obs_node`와 RL policy를
정확히 하나씩 실행한다. RViz가 필요한 Linux display 환경에서만 `rviz:=true`를
사용한다.

### Case A 간소화된 운영 경로

Case A의 정상 데모에서는 내부 localization/mission/control 서비스를 하나씩 직접
호출하지 않는다. `brov_demo_orchestrator`가 기존 fail-closed 서비스를 순서대로
호출하며, launch 자체는 어떤 서비스도 자동 호출하지 않는다.

로봇이 disarm·정지 상태이고, camera가 물리적으로 보정된 neutral이며 tag 2가 깨끗하게
보이는 것을 확인한 뒤 다음을 호출한다. 이 PREPARE 호출 자체가 camera neutral에 대한
작업자의 명시적 확인이다.

```bash
ros2 service call /brov/demo/prepare std_srvs/srv/Trigger "{}"
```

PREPARE는 최대 30초 동안 다음을 한 transaction으로 수행한다.

1. 이미 유효한 pool localization이 없으면 neutral 확인 후 20개 정지 vision sample 수집
2. full-SE(3) `pool→odom` one-shot 초기화
3. 현재 pool pose를 safe box로 짧게 진입시킨 뒤 pool 중앙 방향 0.20 m의 Case-A 2점 경로 생성
4. mission validate 및 immutable commit
5. `/brov/prepare_control`과 preview 생성

응답의 `success=True`와 확정된 두 pool point를 확인한다. 진행 상태는 다음 하나로 볼 수
있다.

```bash
ros2 topic echo --once --qos-durability transient_local /brov/demo/status
```

Shadow profile(`send_pwm:=false arm:=false`)에서는 여기까지만 수행한다. 실제 출력이
허용된 profile에서는 preview, 수조 여유, tether와 추진기 주변을 확인한 후 다음 한 번만
호출한다.

```bash
ros2 service call /brov/demo/start std_srvs/srv/Trigger "{}"
```

START는 ARM→base START를 수행하고 RL의 첫 post-START PWM을 확인한 뒤에만 성공한다.
중간 실패 시 base STOP과 DISARM을 시도하고 실패 원인을 응답에 남긴다. 정상 종료는
다음 하나다.

```bash
ros2 service call /brov/demo/stop std_srvs/srv/Trigger "{}"
```

STOP은 output gate를 먼저 닫고 DISARM한다. estop은 이 orchestration에 포함시키지 않으며
기존 `/brov/estop` latch를 독립적으로 유지한다. Case C는 아직 이 자동 경로를 사용하지
않고 아래의 명시적 staged 절차를 따른다.

## 3. Camera neutral과 one-shot 초기화

로봇을 완전히 정지시키고 camera tilt를 calibration 때의 neutral에 물리적으로
고정한다. Raw measurement와 atomic odometry session을 확인한다.

```bash
ros2 topic echo --once /brov/aruco/robot_pose_pool
ros2 topic echo --once /brov/odometry/local_with_session
```

물리 상태를 확인한 작업자가 neutral을 승인한다. 이 서비스는 camera를 움직이지 않으며
호출 전 sample을 모두 폐기한다.

```bash
ros2 service call /brov/localization/confirm_camera_tilt_neutral \
  std_srvs/srv/Trigger "{}"
```

```bash
ros2 topic echo --qos-durability transient_local \
  /brov/localization/status
```

`state=COLLECTING`이고 configured minimum 이상 sample이 쌓이면 초기화한다.

```bash
ros2 service call /brov/localization/initialize_pool \
  brov_interfaces/srv/InitializePool "{min_samples: 0}"
```

다음을 모두 확인한다.

- service `success=True`
- `state=INITIALIZED`, `output_valid=true`, `epoch>0`
- non-empty `odometry_session_id`와 `alignment_id`
- 실제 배치와 일치하는 pool 위치·자세

```bash
ros2 topic echo --once --qos-durability transient_local \
  /brov/localization/status
ros2 topic echo --once --qos-profile sensor_data \
  /brov/localization/odometry_pool
ros2 run tf2_ros tf2_echo pool base_link
```

값이 실제 측정과 맞지 않으면 진행하지 않고 marker survey, camera extrinsic과 축 정의를
수정한 뒤 새 session에서 다시 초기화한다.

## 4. Pool waypoint draft

`pool_safe_min_xyz`/`pool_safe_max_xyz`는 waypoint 중심과 waypoint 사이의 직선
segment에 적용되는 순항 영역이다. 현재 pose 자체에는 이 bounds를 적용하지 않는다.
따라서 로봇이 바닥에 있어 현재 `z=0.1756 m`처럼 최소 waypoint 높이 `0.20 m`보다
조금 낮아도 정상이다. 첫 waypoint를 현재 x/y와 같은 `z=0.20 m` 지점으로 두면 약
`0.0244 m`의 짧은 진입 경로가 된다.

현재 pose는 유한한 XYZ여야 하고, 첫 waypoint는 safe box 안이면서 현재
`/brov/localization/odometry_pool` 위치에서 0.30 m 이내여야 한다. 그러므로 바닥 시작을
허용해도 임의의 먼 경로로 건너뛰는 것은 계속 거부된다. 모든 waypoint quaternion은
identity여야 한다. 아래 수치는 형식 예시다. 실제 로봇 위치와 당일 측량한 안전 경로로
바꾸지 않고 그대로 사용하면 안 된다.

Case A의 2점 pool 직선 예시:

```bash
ros2 topic pub --once /brov/mission/draft_path nav_msgs/msg/Path "{
  header: {frame_id: pool},
  poses: [
    {header: {frame_id: pool}, pose: {
      position: {x: 0.80, y: 0.85, z: 0.40}, orientation: {w: 1.0}}},
    {header: {frame_id: pool}, pose: {
      position: {x: 2.80, y: 0.85, z: 0.40}, orientation: {w: 1.0}}}
  ]
}"
```

Case C의 4점 pool square 예시:

```bash
ros2 topic pub --once /brov/mission/draft_path nav_msgs/msg/Path "{
  header: {frame_id: pool},
  poses: [
    {header: {frame_id: pool}, pose: {
      position: {x: 1.80, y: 0.65, z: 0.40}, orientation: {w: 1.0}}},
    {header: {frame_id: pool}, pose: {
      position: {x: 2.20, y: 0.65, z: 0.40}, orientation: {w: 1.0}}},
    {header: {frame_id: pool}, pose: {
      position: {x: 2.20, y: 1.05, z: 0.40}, orientation: {w: 1.0}}},
    {header: {frame_id: pool}, pose: {
      position: {x: 1.80, y: 1.05, z: 0.40}, orientation: {w: 1.0}}}
  ]
}"
```

`loop=true`이면 manager가 네 번째 점에서 첫 번째 점으로 닫히는 segment도 검사한다.
따라서 첫 점은 5번째 pose로 반복하지 않는다. 반복하면 zero-length closing segment로
검증이 거부된다.

Manager는 정확히 4점인지, bounds와 모든 closing segment 길이는 검증하지만 직교성과
네 변의 동일 길이까지 판정하지 않는다. 원래 Case C square를 재현할 때는 작업자가
측량한 pool 좌표로 정사각형을 정의하고 commit 전에 형상을 별도로 확인한다.

## 5. Validate, commit, PREPARE

Draft와 현재 alignment를 검증하고 immutable mission으로 commit한다.

```bash
ros2 service call /brov/mission/validate std_srvs/srv/Trigger "{}"
ros2 service call /brov/mission/commit std_srvs/srv/Trigger "{}"
```

두 응답 모두 `success=True`여야 한다. 다음 출력에서 pool path, resolved odom path,
contract version, plan hash, epoch/session/alignment가 일치하는지 확인한다.

```bash
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/active_path_pool
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/resolved_path_odom
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/resolved
```

출력을 frozen 상태로 둔 채 resolved guidance를 로드한다.

```bash
ros2 service call /brov/prepare_control std_srvs/srv/Trigger "{}"
```

PREPARE 성공 후 실제 pool path 방향과 policy preview를 확인한다.

```bash
ros2 topic echo --once /brov/target_waypoint
ros2 topic echo --once /brov/debug/q_desired_zup
ros2 topic echo --once /brov/policy/thruster_pwm_preview
ros2 topic hz /brov/observation
```

Case C에서는 PREPARE를 다시 호출해도 committed deterministic target이 같아야 한다.
Target, path, vehicle clearance 또는 PWM sign이 예상과 다르면 ARM하지 않는다.

## 6. 실제 제어

Shadow launch를 완전히 종료한 뒤 실제 profile을 다시 실행한다. 새 `obs_node` 실행은
새 odometry session이므로 neutral 확인 → initialize → 새 draft → validate → commit →
PREPARE를 전부 다시 수행해야 한다.

```bash
# Case A actual profile
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=a \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true \
  arm:=true \
  rviz:=false
```

Case C는 위의 물리 acceptance가 완료되고 시험 책임자가 승인한 경우에만 다음 profile을
사용한다.

```bash
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=c \
  allow_case_c:=true \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true \
  arm:=true \
  rviz:=false
```

현재 ArduSub mode가 MANUAL이고 heartbeat/DVL/EKF가 정상인지 확인한다. 실제 PWM
publisher는 선택된 RL policy 하나여야 한다.

```bash
ros2 topic info /brov/thruster_pwm --verbose
```

다시 수행한 PREPARE까지 성공했다면 다음 순서로만 출력 gate를 연다.

```bash
ros2 service call /brov/arm_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
```

RL policy는 `/brov/control_active=true` 이후 다음 fresh observation부터 PWM을
발행하므로 별도 controller-start service가 없다. ARM 후 8 s 안에 START해야 하며,
START 후 첫 PWM과 이후 watchdog을 통과해야 한다.

```bash
ros2 topic echo --once /brov/control_active
ros2 topic hz /brov/thruster_pwm
```

Case A는 작업자가 종료한다. Case C v2는 1 lap 또는 60 s에서 먼저 충족된 조건으로
control gate를 닫고 neutral/disarm하는 정상 완료 lifecycle을 사용한다. 자동 완료를
기다리는 동안에도 estop 담당자는 계속 대기한다.

정상 수동 종료는 다음 순서다.

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/disarm_control std_srvs/srv/Trigger "{}"
```

STOP은 PWM gate를 닫고 neutral을 보내지만 hardware disarm을 대신하지 않는다.

## Invalidation과 한계

다음 변화는 기존 alignment와 committed mission을 무효화한다.

- `obs_node`, autopilot/DVL 또는 localizer 재시작
- MAVLink boot-time reset 또는 navigation discontinuity
- localization epoch/session/`alignment_id` 변경
- camera tilt 이동 또는 camera/marker survey 변경

새 session에서는 모든 one-shot 승인 절차를 반복한다. Case C fault 또는 수동 stop 후
자동으로 다시 arm/start하지 않는다.

Vision은 시작 정렬에만 사용되고 continuous fusion은 수행하지 않는다. 초기화 후
marker가 사라져도 frozen `pool -> odom`과 DVL/IMU로 경로를 계산하지만 장기 drift는
보정하지 않는다. Camera timestamp도 exposure time이 아닌 decode 측 ROS time이므로
초기화는 반드시 정지 상태에서 수행한다. Waypoint bounds는 center-point 입력 검사일
뿐 runtime hull/tether geofence가 아니다.
