# One-shot 수조 절대 위치 초기화 및 mission 운용

> **[2026-08-28] 이 문서가 언급하는 `sim2swim_deploy_v2`~`v6` 번들은 저장소에서 제거됐다.**
> 아래 내용은 그 시기의 실험 기록이므로 이름을 그대로 둔다 — 지우면 기록이 깨진다.
> 그 계보는 논문 Eq.(8)의 `w_a = 0.3` 아래에서 나온 **나쁜 draw 하나를 다섯 번
> 패치한 것**이고, 원인이 보상으로 확정되면서 후보 자격을 잃었다(시드 재현율 1/5,
> `w_a`만 0.017로 낮추면 5/5·Fig.4 3/3 통과). 현재 배포 후보는
> `artifacts/policies/sim2swim_paperfix_wa0017_mk2_s42_i299/` 하나다.
> 복원이 필요하면 `git checkout eae2ed7^ -- artifacts/policies/sim2swim_deploy_v5_mk2_s42_i299`.

## 정상 Case-A 데모: 3-operation API

`pool_localized_demo.launch.py`는 기본적으로 `brov_demo_orchestrator`를 함께 실행하지만
launch 시점에는 localization, mission, arm 또는 controller 서비스를 호출하지 않는다.
로봇이 disarm·정지 상태이고 camera가 물리 neutral이며 marker가 정상임을 확인한 뒤:

```bash
ros2 service call /brov/demo/prepare std_srvs/srv/Trigger "{}"
ros2 service call /brov/demo/start std_srvs/srv/Trigger "{}"
ros2 service call /brov/demo/stop std_srvs/srv/Trigger "{}"
```

PREPARE는 camera-neutral 확인, sample 수집, full-SE(3) one-shot 초기화, 현재 pose 기반
Case-A 3점 경로(`P0→P1(z=0.70)` takeoff 후 `P1↔P2` loop) 생성,
validate/commit, base PREPARE를 묶는다. START는 기존 ARM과
START 및 선택된 controller의 첫 실제 PWM 확인을 묶는다. STOP은 gate-close, controller
stop(model인 경우), neutral/DISARM 순서를 묶는다. 어떤 내부 gate가 거부하면 상위
서비스도 실패하고 그 이유를 그대로 반환한다. `/brov/estop`은 통합하지 않는다.

아래 절차는 진단, 사용자 draft, Case C 및 orchestrator 실패 복구를 위한 상세 수동
절차다.

## 목적과 현재 범위

ArduSub/DVL의 local odometry는 프로세스 또는 autopilot을 다시 시작할 때 원점과
방향 기준이 달라질 수 있다. 이 기능은 측량된 AprilTag의 raw vision pose를 한 번만
사용해 현재 odometry를 고정 `pool` 좌표에 정렬한다. 이후 제어 중에는 DVL/IMU
odometry의 연속성만 사용하며, marker 재검출이 경로 또는 자세 기준을 움직이지
않는다.

현재 구현은 다음 범위다.

1. MAVLink NED/FRD snapshot을 stamped `odom`/FLU odometry로 변환하고, odometry와
   그 session ID를 하나의 atomic DDS envelope로 발행한다.
2. 작업자가 실제 camera tilt neutral을 확인하고 명시적으로 승인해야 정지 상태의
   vision pose/local odometry pair 수집을 허용한다.
3. 작업자가 명시적으로 승인하면 full-SE(3) `pool -> odom`을 한 번 계산해 고정하고
   그 정확한 transform에 고유한 `alignment_id`를 부여한다.
4. `pool` waypoint draft를 검증하고 현재 alignment의 `odom` 경로로 한 번 변환해
   immutable mission으로 commit한다.
5. 현재 odometry session, localization epoch, `alignment_id`, committed plan이 모두
   일치할 때만 `obs_node`의 control lifecycle이 열린다. `arm=true`는 기동/prepare/
   start의 자동 arm이 아니라 명시적인 ARM 단계를 허용하는 권한 설정이다.

기존 policy/controller의 16-D observation은 변경하지 않는다.

```text
[q_error_wxyz(4), velocity_error_body(3), angular_velocity(3),
 velocity_error_integral(3), quaternion_vector_error_integral(3)]
```

Vision은 observation에 새 항목으로 입력되지 않으며, 절대 frame 초기화와
pool-waypoint 해석에만 사용된다.

## 좌표계와 계산

```text
pool (측량된 Z-up global frame)
└── odom (한 실행/session 동안 연속적인 Z-up frame)
    └── base_link (FLU)
```

MAVLink의 NED world/FRD body pose는 `S = diag(1,-1,-1)`을 사용해
Z-up/FLU odometry로 바꾼다. Vision raw pose와 같은 획득 시각의 odometry를 각각
`⁽pool⁾T_base,vision`, `⁽odom⁾T_base,mav`라 하면 정렬은 다음과 같다.

```text
⁽pool⁾T_odom = ⁽pool⁾T_base,vision
               · inverse(⁽odom⁾T_base,mav)

⁽pool⁾T_base(t) = ⁽pool⁾T_odom · ⁽odom⁾T_base(t)
```

translation뿐 아니라 rotation 전체를 초기화하므로 매번 달라지는 AHRS yaw 기준도
pool 기준으로 정렬된다. 여러 정지 sample의 translation/quaternion을 robust하게
평균하고 residual, roll/pitch 및 freshness gate를 통과해야 한다.

`alignment_id`는 승인된 **정확한** `⁽pool⁾T_odom`과 localizer lifetime을 식별하는
boot-unique UUID다. 외부에서 값의 구조를 해석하지 않는다. INITIALIZED status는
`alignment_id`와 `pool_to_odom` transform을 함께 발행하고, mission은 commit 때의
ID를 저장한다. session과 epoch 숫자가 우연히 같더라도 ID가 다르면 다른 transform으로
간주해 거부한다.

## ROS interface

| 목적 | interface | 계약 |
|---|---|---|
| canonical local odometry | `/brov/odometry/local_with_session` `brov_interfaces/OdometrySession` | `Odometry`와 해당 `odometry_session_id`를 한 DDS sample로 결합; localizer의 유일한 canonical 입력 |
| odometry diagnostics | `/brov/odometry/local`, `/brov/odometry/session_id` | 사람이 각각 확인하기 위한 파생 topic; 둘을 결합해 ownership 판단하지 않음 |
| raw vision pose | `/brov/aruco/robot_pose_pool` `PoseStamped` | parent `pool`, 초기화 측정 전용 |
| neutral 확인 | `/brov/localization/confirm_camera_tilt_neutral` `Trigger` | 물리적 확인 후 작업자 명시 호출; 카메라를 움직이지 않음 |
| initialize | `/brov/localization/initialize_pool` `InitializePool` | 작업자 명시 호출 |
| reset | `/brov/localization/reset` `Trigger` | alignment 폐기 및 epoch 변경 |
| canonical aligned odometry | `/brov/localization/odometry_pool_with_alignment` `brov_interfaces/AlignedOdometry` | `Odometry`와 해당 epoch/session/alignment ID를 한 DDS sample로 결합; mission first-point gate의 유일한 입력 |
| aligned odometry diagnostics | `/brov/localization/odometry_pool` `Odometry` | parent `pool`, child `base_link`; RViz/사람 확인용 |
| localization state | `/brov/localization/status` `LocalizationStatus` | state, atomic `output_valid`, epoch, session, alignment ID, exact `pool_to_odom`, sample count, reason |
| draft route | `/brov/mission/draft_path` `nav_msgs/Path` | 반드시 `pool` frame |
| validate/commit | `/brov/mission/validate`, `/brov/mission/commit` | 각각 `Trigger`; 자동 호출 없음 |
| committed routes | `/brov/mission/active_path_pool`, `/brov/mission/resolved_path_odom` | RViz/검사용 latched snapshot |
| control contract | `/brov/mission/resolved` `ResolvedMission` | contract version, canonical pool plan, SHA-256 hash, epoch/session/alignment ID가 포함된 immutable snapshot |

`pool -> odom`은 localization node, `odom -> base_link`는 `obs_node`만 발행한다.
perception과 mission node는 MAVLink, PWM, arm 또는 control service를 소유하지 않는다.

## 권장 구현·검증 순서

다음 순서는 현재 코드에 반영된 dependency 순서이며, 이후 변경 시에도 유지한다.

1. **Stamped local odometry와 session identity**: 같은 MAVLink snapshot에서 pose와
   child-frame twist를 만들고 time reset/process restart를 새 session으로 구분한다.
   `OdometrySession` envelope가 pose와 session identity를 원자적으로 묶으며, 별도
   diagnostic topic 두 개의 DDS 도착 순서에 의존하지 않는다.
2. **Raw vision 측정 검증**: intrinsic, marker survey, neutral camera extrinsic과
   `pool` frame 축을 독립적으로 확인한다.
3. **One-shot full-SE(3) alignment**: 정지·freshness·timestamp skew·residual gate 후
   명시적 서비스 호출로만 `pool -> odom`을 승인한다. configured sample minimum은
   request로 낮출 수 없으며 성공마다 exact transform의 새 UUID를 발급한다.
4. **Pool mission transaction**: Draft → Validate → Commit 순서로 수조 bounds,
   localization freshness, 첫 waypoint 근접성과 경로 hash를 검증한 뒤 한 번만
   `odom`에 resolve한다.
5. **Three-step control gate**: `obs_node`는 같은 session/epoch/alignment ID의
   localization과 resolved mission에 대해 PREPARE → ARM → START를 각각 명시적으로
   수행한다. PREPARE는 mission을 frozen preview로 load하고, ARM은 모든 gate를 다시
   확인한 뒤 neutral→arm하며, START는 prepared/armed 상태만 활성화하고 arm하지 않는다.
6. **실기체 shadow/actuation 검증**: 먼저 `send_pwm=false`, `arm=false`로 전체
   topic/TF/preview를 확인하고 실제 actuation 실행에서는 모든 one-shot 단계를 다시
   수행한다.
7. **향후 robustness**: multi-marker, 실제 tilt feedback, exposure timestamp,
   covariance/continuous fusion은 별도 검증 후 추가한다.

## 매 실행 시 operator 절차

### 1. Shadow launch

카메라, AprilTag, localizer, mission manager, observation/MAVLink owner와 정확히 한
controller를 실행한다. 기본값은 PWM 송신과 arm 모두 꺼져 있다.

```bash
ros2 launch brov_bringup pool_localized_demo.launch.py \
  controller:=model \
  send_pwm:=false \
  arm:=false
```

RL을 검증할 때는 legacy 계약이면 `controller:=rl`, MK2 계약
(`sim2swim_deploy_v2..v5_mk2_*`)이면 `controller:=rl_mk2`를 사용한다. 자세한
선택 기준과 policy artifact 경로는 [SIM2SWIM_DEMO.md](SIM2SWIM_DEMO.md)를
따른다. model과 RL을 동시에 실행하지 않는다.
이 launch는 tilt confirmation, initialize, validate, commit, PREPARE, ARM 또는 START를
호출하지 않는다.

`rviz`도 기본 `false`다. compatible Linux display가 준비된 경우 최초 launch 명령에
`rviz:=true`를 추가하면 visualization-only `pool_scene_node`와 RViz를 함께 구성한다.
이는 control/MAVLink/service owner를 추가하지 않는다. macOS에서 RViz/OGRE가
qualified되지 않은 현재 환경은 main launch를 기본값으로 유지하고 별도 터미널에서
headless marker publisher만 실행할 수 있다.

```bash
ros2 launch brov_viz pool_vision.launch.py rviz:=false
```

### 2. 입력 상태 확인

로봇을 움직이지 않고 tag가 보이는 상태에서 다음 입력이 연속적으로 갱신되는지
확인한다.

```bash
ros2 topic hz /brov/odometry/local_with_session
ros2 topic echo --once /brov/odometry/local_with_session
ros2 topic echo --once /brov/aruco/robot_pose_pool
ros2 topic echo /brov/localization/status
```

`/brov/odometry/local`과 transient-local `/brov/odometry/session_id`도 각각 진단할 수
있지만 localizer는 이를 조합하지 않는다. 위 atomic envelope 안의 odometry
`header.frame_id=odom`, `child_frame_id=base_link`, non-empty session ID가 함께
갱신되는지를 기준으로 확인한다.

카메라 tilt를 보정된 물리적 neutral에 고정하고 영상/장착 상태를 직접 확인한 뒤
다음 confirmation을 호출한다.

```bash
ros2 service call /brov/localization/confirm_camera_tilt_neutral \
  std_srvs/srv/Trigger "{}"
```

이 서비스는 카메라를 움직이거나 encoder로 neutral을 측정하지 않는다. 현재 고정
extrinsic을 사용해도 된다는 작업자 acknowledgement일 뿐이며 launch는 이를 자동
호출하지 않는다. non-empty odometry session이 있어야 성공하고, 성공 시 이전에
쌓인 odometry/vision/alignment sample을 모두 비운 뒤 새 sample만 수집한다.

confirmation 이전에는 `sample_count=0`이며 initialize도 fail-closed다. confirmation
이후 status가 `COLLECTING`이고 sample 수가 증가해야 한다. 증가하지 않으면 reason과
camera visibility, odometry twist, message age/skew를 먼저 해결한다.

### 3. One-shot pool 초기화

configured minimum 이상의 synchronized stationary sample을 승인한다. `0`은 config의
기본 minimum을 선택한다. 0이 아닌 값이 configured minimum보다 작으면 명시적으로
거부되며, 더 엄격하게 하려는 경우에만 더 큰 수를 요청한다.

```bash
ros2 service call /brov/localization/initialize_pool \
  brov_interfaces/srv/InitializePool "{min_samples: 0}"
```

반드시 `success=True`, `state=INITIALIZED`, `output_valid=true`, non-zero epoch와
non-empty `alignment_id`인지 확인한다. `INITIALIZED`이더라도 `output_valid=false`면
aligned odometry와 mission/control gate에 사용할 수 없다.

```bash
ros2 topic echo --once /brov/localization/status
ros2 topic echo --once /brov/localization/odometry_pool_with_alignment
ros2 topic echo --once /brov/localization/odometry_pool
ros2 run tf2_ros tf2_echo pool base_link
```

수조에서 측정한 실제 위치·자세와 차이가 크면 control로 진행하지 않고 reset 후
marker survey, 카메라 extrinsic, 축/부호를 다시 확인한다.

### 4. Pool waypoint draft 작성

먼저 `/brov/localization/odometry_pool`의 현재 위치를 사람이 확인한다. Mission
manager는 같은 pose가 epoch/session/alignment ID와 원자적으로 묶인
`/brov/localization/odometry_pool_with_alignment`만 first-point gate에 사용한다.
첫 waypoint는 이 위치에서 기본 0.30 m 이내여야 한다. 아래 좌표는 형식 예시이므로
현재 로봇 위치와 실제 안전 경로로 반드시 바꾼다.

```bash
ros2 topic pub --once /brov/mission/draft_path nav_msgs/msg/Path "{
  header: {frame_id: pool},
  poses: [
    {header: {frame_id: pool}, pose: {
      position: {x: 2.20, y: 0.85, z: 0.35}, orientation: {w: 1.0}}},
    {header: {frame_id: pool}, pose: {
      position: {x: 2.70, y: 0.85, z: 0.35}, orientation: {w: 1.0}}}
  ]
}"
```

기본 config는 position-only waypoint다. non-identity orientation은 fail-closed로
거부되며 heading mode는 `straight` 또는 `align`만 허용된다.

### 5. Validate와 immutable commit

```bash
ros2 service call /brov/mission/validate std_srvs/srv/Trigger "{}"
ros2 service call /brov/mission/commit std_srvs/srv/Trigger "{}"
```

두 응답 모두 `success=True`여야 한다. validate 이후 draft, session, epoch,
`alignment_id` 또는 exact transform이 변하면 commit은 거부되므로 다시 validate한다.
한 프로세스에서 다른 mission으로 교체할 수 없으며 mission manager를 재시작해 새
transaction을 시작해야 한다.

```bash
ros2 topic echo --once --qos-durability transient_local /brov/mission/status
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/active_path_pool
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/resolved_path_odom
ros2 topic echo --once --qos-durability transient_local \
  /brov/mission/resolved
```

`/brov/localization/status.alignment_id`와
`/brov/mission/resolved.alignment_id`가 정확히 같아야 한다. localizer를 재시작하면
INITIALIZED 전에는 ID가 비어 있고, 재초기화 성공 후에는 새 UUID가 발급된다. 따라서
이전 ID로 commit된 mission은 재사용할 수 없다.

또한 `contract_version`이 현재 지원하는
`brov_pool_position_mission_v1`인지 확인한다. `canonical_plan_json`은 승인한 pool
waypoint와 guidance 설정을 담고, 그 ASCII byte sequence의 SHA-256이 `plan_hash`와
일치해야 한다. `obs_node`는 version, hash, canonical 설정과 exact
`pool_to_odom`으로 다시 계산한 odom waypoint를 모두 검증하므로 어느 하나라도
다르면 PREPARE를 거부한다.

### 6. PREPARE와 shadow 결과 확인

Resolved guidance는 명시적 PREPARE에서 frozen/no-output 상태로 load된다. Shadow
launch의 `send_pwm=false`, `arm=false`를 유지한 채 다음 service를 호출한다.

```bash
ros2 service call /brov/prepare_control std_srvs/srv/Trigger "{}"
```

PREPARE는 localization/mission identity와 telemetry를 검사하지만 arm하거나 active
control로 전환하지 않는다. 따라서 PREPARE 성공 후 committed mission 기준의
observation/preview를 검사할 수 있다.

```bash
ros2 topic hz /brov/observation
ros2 topic echo --once /brov/target_waypoint
ros2 topic echo --once /brov/model_based/thruster_pwm_preview
```

RL controller에서는 `/brov/action`과
`/brov/policy/thruster_pwm_preview`를 대신 확인한다. policy preview는 frozen
observation에서도 계속 발행되지만 `/brov/control_active=true` 전에는 policy node가
`/brov/thruster_pwm`을 발행하지 않는다. pool 절대 pose와 resolved path, 첫 target의
방향 및 예상 thruster sign이 일치해야 한다. Shadow에서는 `/brov/arm_control`,
`/brov/start_control` 또는 controller start를 호출하지 않는다.

### 7. 실제 actuation

shadow launch를 종료하고 물리적 안전 조건을 다시 확인한 뒤 `send_pwm=true`,
`arm=true`로 launch한다. 여기서 `arm=true`는 explicit ARM service를 허용할 뿐
constructor, PREPARE 또는 START에서 자동 arm하지 않는다. 새 `obs_node` 실행은 새
odometry session이므로 **camera neutral confirmation부터 mission commit까지 반드시
다시 수행한다.**

```bash
ros2 launch brov_bringup pool_localized_demo.launch.py \
  controller:=model \
  send_pwm:=true \
  arm:=true
```

초기화와 commit을 완료한 후 정확히 PREPARE → ARM → START 순서로 진행한다.

직접 RCPassThru를 사용하는 현재 backend는 ArduSub `MANUAL` custom mode 19만
허용한다. 또한 autopilot heartbeat가 기본 2.0 s 이내로 계속 수신되어야 한다.
PREPARE/ARM/START 및 실제 PWM 직전마다 이 조건을 다시 검사하며, QGroundControl에서
mode와 link 상태를 먼저 확인하고 service 응답에 heartbeat/mode 오류가 있으면
진행하지 않는다. 실제 PWM controller publisher도 정확히 하나여야 한다.

```bash
ros2 topic info /brov/thruster_pwm --verbose
```

Publisher count가 정확히 `1`이고 선택한 model 또는 RL controller 하나만 표시되어야
한다. preview topic은 별도 topic이므로 이 count에 포함되지 않는다.

```bash
ros2 service call /brov/prepare_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/arm_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/model_based/start std_srvs/srv/Trigger "{}"
```

PREPARE는 committed mission을 frozen preview로 load한다. ARM은 모든 gate를 다시
검사하고 neutral을 보낸 뒤 hardware arm한다. START는 prepared 상태와, PWM을 보낼
때의 hardware armed 상태를 확인해 control active로 전환할 뿐 arm하지 않는다. 각
응답이 `success=True`인지 확인한 뒤에만 다음 단계로 넘어간다.

기본 safety 설정에서 ARM 승인은 8.0 s 동안만 유효하므로 ARM 성공 후 8.0 s 안에
START가 성공해야 한다. START 뒤 첫 controller PWM도 8.0 s 안에 도착해야 하며, 첫
명령 이후에는 연속 PWM 간격이 0.25 s를 넘으면 fault가 걸리고 neutral/disarm한다.
따라서 model controller는 START 직후 `/brov/model_based/start`까지 8.0 s 안에
완료한다. RL은 별도 controller start service 없이 다음 active observation부터
출력한다. timeout이나 heartbeat/mode/publisher/localization/mission invalidation이
발생하면 다시 PREPARE부터 현재 상태를 승인한다.

RL은 마지막 model controller service를 호출하지 않는다. Stop은 PWM gate를 먼저
닫아 neutral을 보낸 뒤 controller를 정지하고 명시적으로 disarm한다.

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
ros2 service call /brov/model_based/stop std_srvs/srv/Trigger "{}"
ros2 service call /brov/disarm_control std_srvs/srv/Trigger "{}"
```

## Invalidation과 재초기화

다음 경우 기존 alignment와 mission을 재사용하지 않는다.

- `obs_node` 또는 autopilot/DVL odometry session이 바뀜
- MAVLink boot/time reset 또는 navigation pose/attitude discontinuity가 감지됨
- localizer 프로세스가 재시작되어 alignment lifetime이 바뀜
- 작업자가 `/brov/localization/reset`을 호출함
- localization epoch/alignment ID가 바뀌거나 status가 stale/invalid가 됨
- committed mission의 session, epoch, alignment ID 또는 hash가 현재 값과 다름
- heartbeat가 stale해지거나 ArduSub가 MANUAL mode 19를 벗어남
- ARM 후 8.0 s 안에 START하지 않거나 START 후 첫 PWM이 8.0 s 안에 오지 않음
- active PWM stream이 0.25 s 이상 끊기거나 PWM publisher가 정확히 하나가 아님

수동 reset은 control을 정지한 상태에서만 수행한다.

```bash
ros2 service call /brov/localization/reset std_srvs/srv/Trigger "{}"
```

reset 또는 empty/changed odometry session은 tilt-neutral confirmation과
`alignment_id`를 모두 지운다. active control 중 navigation discontinuity 또는
identity 변경이 감지되면 `obs_node`는 즉시 neutral/disarm하고 fault 처리한다. 정상
연속 motion과 구분하기 위해 짧은 receive-time 간격과 configurable translation/
rotation jump threshold를 함께 사용한다.

원인을 해결한 뒤 새 session에서 camera neutral confirm → sample 수집 → initialize →
새 draft → validate → 새 commit 순서를 처음부터 수행한다. mission manager에 이전
immutable mission이 남아 있으면 이를 재시작한 뒤 새 mission을 commit한다. localizer만
재시작해 epoch 숫자가 다시 같아져도 boot-unique `alignment_id`가 달라 기존 mission은
gate를 통과하지 못한다.

## 현재 한계

- **ArUco 처리 적체:** 카메라 decoder FPS가 정상이더라도 synchronous ArUco 검출과
  debug image 생성이 입력률보다 느리면 과거 영상이 queue에 남아
  `/brov/aruco/robot_pose_pool` freshness gate를 실패할 수 있다. 후속 구현에서는
  image subscription을 `KEEP_LAST depth=1`로 두고, 처리 중 도착한 과거 frame을
  버려 항상 최신 frame을 다음 대상으로 삼는다. debug image 구독자가 없을 때는
  annotation/conversion/publication을 생략한다. `SUBPIX`가 기본 정확도 모드지만,
  측정된 처리 지연 때문에 정지 one-shot 초기화가 불가능한 경우에만 `NONE`을 임시
  선택하고 pose repeatability를 다시 확인한다. 이 개선이 적용·검증되기 전에는
  localizer의 message-age 제한을 늘려 queue 적체를 숨기지 않는다.
- **정지 초기화:** 움직이는 동안 수집한 pair는 사용하지 않는다. DVL twist bias가
  gate보다 크면 실제로 정지해도 sample이 쌓이지 않을 수 있다.
- **decode-time stamp:** 현재 camera pose stamp는 영상 exposure 시각이 아니라
  수신/decoder 측 ROS 시각이며 local odometry도 MAVLink 수신 시각에 의존한다.
  freshness/skew gate는 지연 자체를 추정·보상하지 못하므로 네트워크·decode 지연
  변동은 pairing 오차가 된다.
- **neutral tilt:** 실제 tilt encoder/TF가 없으므로 카메라는 calibrated neutral에
  고정해야 한다. normalized tilt command를 측정 각도로 간주하지 않는다.
- **nominal extrinsic:** USD에서 가져온 camera translation과 수동 optical-axis
  정의는 정밀 hand-eye calibration을 대체하지 않는다.
- **single marker:** 한 marker의 거리·관측각·탁도·pose ambiguity에 민감하다.
- **navigation reset heuristic:** 현재 discontinuity 검출은 짧은 receive-time 내의 큰
  position/attitude jump threshold 기반이다. 작은 EKF origin 변화나 장기 drift를
  완전히 검출·보정하는 DVL reset counter/fusion은 아니다.
- **continuous fusion 없음:** 승인 이후 vision은 `pool -> odom`을 갱신하지 않는다.
  exact transform과 alignment ID는 reset까지 frozen이다. 장시간 DVL drift 보정이나
  marker reacquisition은 아직 구현하지 않았다.
- **position-only mission:** waypoint orientation, runtime swept-volume geofence,
  tether model 및 NBV action interface는 후속 범위다.
