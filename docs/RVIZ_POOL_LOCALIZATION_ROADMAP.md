# RViz 수조 위치·자세 및 3-D waypoint roadmap

## 상태와 목적

**상태: P1 local odometry, P2 raw vision, P3 one-shot alignment 및 typed mission
backend 구현. InteractiveMarker editor와 continuous fusion은 보류.**

현재 구현된 neutral confirmation, full-SE(3) 초기화, exact-transform
`alignment_id`, session invalidation, Draft → Validate → Commit 및
PREPARE → ARM → START 명령은
[POOL_LOCALIZATION_RUNBOOK.md](POOL_LOCALIZATION_RUNBOOK.md)를 기준 문서로 삼는다.
아래 내용 중 "향후" interface 제안은 구현된 typed interface와 충돌할 경우 기준
문서 및 실제 message/service 정의가 우선한다.

이 문서는 RViz에서 다음 정보를 일관된 수조 좌표로 입력·표시하기 위한 향후
구현 계약을 기록한다.

- DVL/ArduSub 기반 상대 odometry와 로봇의 위치·자세
- 수조 벽·바닥·수면과 안전 여유
- 설정된 경로, 현재 목표, 주행 궤적
- RViz에서 작성한 3차원 waypoint와 목표 자세
- 수조 벽면의 surveyed ArUco marker로 초기화한 절대 수조 pose

현재 단계에서는 odometry bridge, one-shot localization backend, pool scene RViz config와
typed mission transaction까지 구현했다. URDF/RobotModel, InteractiveMarker waypoint
editor와 continuous fusion은 추가하지 않았다. RViz는 시각화와 사용자 입력 UI일
뿐이며, 위치를 추정하거나 PREPARE/ARM/START를 호출하거나 PWM을 발행하지 않는다.

## 현재 가능한 것과 부족한 것

현재 코드에는 다음 기반 요소가 있다.

- `/brov/camera/image_raw`와 `/brov/camera/camera_info`
- ArUco의 camera-relative marker pose와 raw pool-frame robot pose
- ArduSub `ATTITUDE_QUATERNION` 및 `LOCAL_POSITION_NED`에서 얻은 위치·자세
- stamped `/brov/odometry/local`, `odom → base_link` TF와 session identity
- explicit one-shot `pool → odom`, aligned pool odometry와 localization status
- typed pool draft Validate/Commit 및 immutable odom-resolved mission
- nominal pool/marker/raw vision visualization marker

그러나 아래 항목은 아직 없다.

- 실제 tilt feedback을 포함한 URDF/RobotModel
- DVL drift를 계속 보정하는 covariance 기반 continuous fusion/multi-marker
- InteractiveMarker 기반 runtime waypoint editor와 custom RViz panel
- runtime swept-volume/tether geofence, executed trail과 NBV action interface

현재 `/brov/debug/pos_ned`, `/brov/debug/att_quat_ned`,
`/brov/debug/pos_mission`은 header가 없는 `Float32MultiArray`다. frame과 획득
시각이 없으므로 RViz용 표준 pose로 단순 포장해서는 안 된다. 또한 현재
`start_heading` frame은 `/brov/start_control` 때마다 다시 정의되므로 여러
실험과 3-D map을 공유하는 절대 수조 frame이 될 수 없다.

## 제안 좌표계와 TF 소유권

이 프로젝트에서는 `pool`을 REP-105의 global `map` 역할을 하는 영구 frame으로
사용한다. 추후 외부 도구가 반드시 `map`이라는 이름을 요구하면 전체 시스템에서
하나의 canonical 이름을 선택하며, `pool`과 `map`을 서로 다른 두 global root로
운영하지 않는다.

```text
pool
├── odom
│   └── base_link
│       └── camera_mount_link
│           └── camera_tilt_link
│               └── camera_link
│                   └── camera_optical_frame
└── aruco_map_<id>
```

권장 수조 좌표는 다음과 같다.

- 원점: surveyed 수조 내부 바닥의 가까운 우측 모서리
- +X: 수조 4.0 m 길이 방향
- +Y: +X를 바라볼 때 왼쪽, 수조 1.7 m 너비 방향
- +Z: 위쪽, 바닥에서 수면 방향
- `base_link`: ROS FLU(X forward, Y left, Z up)

명목 수조 부피는 `[0,4.0] × [0,1.7] × [0,1.1] m`지만, 실제 내벽과
수면 높이는 측량값으로 저장한다. 현재 제어의 `start_heading`은 +Y right,
+Z down 계열의 mission-relative 계약을 포함하므로 `pool`과 동일시하지 않는다.
제어 시작 시 `T_pool_mission`을 명시적으로 계산·기록하고 경로를 변환해야 한다.

각 TF edge의 broadcaster는 하나뿐이어야 한다.

| TF edge | 향후 소유자 | 의미 |
|---|---|---|
| `pool → odom` | localization/alignment node | 절대 수조와 연속 local odometry의 정렬 |
| `odom → base_link` | MAVLink odometry bridge | DVL/IMU/EKF 기반의 연속적이고 jump 없는 local pose |
| `base_link → camera_mount_link` | robot description | 고정 장착 위치 |
| `camera_mount_link → camera_tilt_link` | tilt joint publisher | 획득 시각의 실제 tilt 각도 |
| `camera_tilt_link → camera_optical_frame` | robot description | 카메라/optical frame 고정 변환 |
| `pool → aruco_map_<id>` | surveyed marker map | 수조 벽면 marker의 고정 pose |

ArUco detector는 향후 raw measurement를 topic으로 발행하고 `pool → base_link`
TF를 직접 소유하지 않는다. 이렇게 해야 odometry 및 localization과 TF loop 또는
다중 parent가 생기지 않는다.

## ArUco를 이용한 절대 pose 초기화

현재 배포에서 DVL local position은 수조 측량 좌표와의 관계를 알 수 없다. 따라서
`pool` 기준 절대 pose를 얻으려면 surveyed wall marker와 같은 외부 anchor가
필요하다.

`⁽A⁾T_B`를 B 좌표를 A 좌표로 바꾸는 transform이라고 할 때 필요한 값은 다음이다.

- surveyed `⁽pool⁾T_marker`
- image detection의 `⁽camera⁾T_marker`
- 획득 시각의 calibrated `⁽base⁾T_camera`

로봇의 vision pose와 초기 alignment는 다음과 같다.

```text
⁽pool⁾T_base = ⁽pool⁾T_marker
                · inverse(⁽camera⁾T_marker)
                · inverse(⁽base⁾T_camera)

⁽pool⁾T_odom = ⁽pool⁾T_base · inverse(⁽odom⁾T_base)
```

초기 구현은 여러 연속 frame의 품질을 확인한 뒤 작업자가 명시적으로 승인하는
one-shot initialization으로 한다. 그 이후에는 DVL odometry가 연속성을 유지한다.
continuous correction/fusion은 innovation gate와 dropout 정책을 갖춘 후 별도
단계에서 구현한다. `pool → odom` 보정이 active control waypoint를 순간적으로
이동시키지 않도록 제어는 연속적인 odom/mission frame에서 유지한다.

초기화 전에 필요한 조건은 다음과 같다.

- 최종 housing과 수중 환경, 실제 stream 해상도에서 수행한 intrinsic calibration
- 실제 marker 크기와 `pool` 내 marker pose 측량
- camera-to-base translation/rotation extrinsic calibration
- reprojection error, view angle, 거리, 연속 detection 및 covariance gate
- image와 odometry의 공통 획득 timestamp
- marker loss, pose flip/jump 및 재검출에 대한 outlier/stale 처리

한 개의 marker로 6-DoF를 계산할 수 있어도 수중 탁도와 관측각에 취약하므로,
실험용 board 또는 여러 surveyed marker 사용을 우선 검토한다.

### 카메라 tilt 관련 blocker

현재 ArUco node는 `base_to_camera_xyz/rpy`라는 고정 extrinsic을 사용한다. 실제
카메라는 tilt할 수 있고 현재 ROS에는 normalized command만 있으며 실제 joint
각도 feedback은 없다. 따라서 카메라가 움직이면 계산된 절대 robot pose가 틀릴 수
있다.

첫 단계에서는 tilt를 calibrated neutral에 고정한 상태에서만 vision pose를
검증한다. 이후에는 tilt 축·영점·방향·기어/서보 오차를 보정하고, 획득 시각에 맞는
측정 또는 검증된 joint angle로 dynamic TF를 발행해야 한다. command 값만 사용하면
그 오차를 covariance에 포함해야 한다.

## 제안 ROS interface

현재 perception 단계는 surveyed marker와 nominal locked-neutral camera
extrinsic을 합성한 raw `PoseStamped` 측정
`/brov/aruco/robot_pose_pool`을 제공한다. 이는 covariance, reprojection gate,
outlier rejection 또는 DVL alignment가 없는 shadow/debug 출력이다. 아래
`/brov/localization/vision_pose`는 이 raw 측정을 검증·승격할 미래 localization
interface이며 아직 제공되지 않는다.

아래 중 local odometry와 mission transaction은 구현되었으며, covariance-bearing
vision 승격·diagnostics·editor 항목은 아직 proposal이다. 현재 실제 service/message
계약은 [POOL_LOCALIZATION_RUNBOOK.md](POOL_LOCALIZATION_RUNBOOK.md)를 따른다.

| 목적 | 제안 topic/type | frame/비고 |
|---|---|---|
| local pose/twist | `/brov/odometry/local` `nav_msgs/Odometry` | `odom`, child `base_link`, covariance 포함 |
| raw pool pose measurement | `/brov/aruco/robot_pose_pool` `PoseStamped` | `pool`, one-shot 입력; covariance 없음 |
| localization health | `/brov/localization/diagnostics` `DiagnosticArray` | stale, marker ID, reprojection/innovation 상태 |
| executed trail | `/brov/trajectory` `nav_msgs/Path` | `pool` 또는 initialization 전 `odom` |
| draft route | `/brov/mission/draft_path` `nav_msgs/Path` | 편집 중이며 제어에 사용하지 않음 |
| committed route | `/brov/mission/active_path_pool` `nav_msgs/Path` | 검증 후 immutable snapshot |
| current target | `/brov/mission/current_target` `PoseStamped` | pose와 frame을 명시 |
| pool geometry | `/brov/pool/markers` `MarkerArray` | 벽, 바닥, 수면, marker, safe volume |
| mission display | `/brov/mission/markers` `MarkerArray` | 번호, 선, 자세 축, valid/invalid 색상 |

`nav_msgs/Path`는 표준 시각화에는 적합하지만 speed, loop, heading mode, 허용오차와
검증 결과를 모두 담을 수 없다. 실제 mission commit/action 계약이 필요해지면
`brov_interfaces`의 typed message/service/action을 정의하며, 병렬 Float 배열이나
RViz `MarkerArray`를 planner API로 사용하지 않는다.

## RViz waypoint 입력 방식

| 방식 | 장점 | 한계 | 용도 |
|---|---|---|---|
| Publish Point | 가장 간단한 `PointStamped` 입력 | 화면의 surface를 클릭해야 하며 z/자세/순서 편집이 불편 | 고정 depth plane의 초기 prototype |
| 2D Goal Pose | x/y/yaw 입력이 쉬움 | 평면 navigation 의미이며 3-D 수중/NBV에 부적합 | 고정 depth의 임시 평면 실험만 |
| 6-DoF InteractiveMarker | xyz와 자세를 직접 drag하고 순서·색상 표시 가능 | 별도 marker server 필요 | **권장 핵심 editor** |
| Custom RViz panel | 정확한 수치, reorder, YAML, 검증 결과 표시 | C++/Qt plugin 유지보수 필요 | InteractiveMarker 이후 생산용 UX |

권장 최종 흐름은 다음과 같다.

```text
Confirm camera tilt neutral
  → Initialize exact pool → odom alignment
  → Edit Draft
  → Validate
  → operator visual check
  → Commit immutable active mission
  → PREPARE frozen shadow-mode verification
  → explicit ARM
  → explicit START
```

click/drag 한 번으로 active waypoint가 바뀌거나 제어가 시작되어서는 안 된다.
제어 중 편집은 draft에만 반영하거나 거부한다. commit은 frame, 유한값, pool bounds,
차체 swept envelope, tether 여유, localization validity를 확인해야 한다. 현재의
waypoint bounds는 설정 입력 검증일 뿐 runtime measured-position geofence가 아니다.

## RViz 화면 구성

ArUco initialization 전에는 Fixed Frame을 `odom`으로 두고 화면에
`RELATIVE / POOL UNINITIALIZED` 상태를 명확히 표시한다. 초기화 후 Fixed Frame을
`pool`로 전환한다.

- TF와 RobotModel(`brov_description` URDF/Xacro가 필요)
- local odometry pose, covariance, axes와 주행 trail
- 수조 벽·바닥·수면, surveyed marker, uncertainty-inflated safe volume
- draft/active path, waypoint 번호와 자세 축, current target
- DVL pose와 vision-only ghost pose의 비교
- `/brov/aruco/debug_image`와 localization diagnostics

권장 색상은 draft cyan, active green, current target yellow, invalid red,
DVL/local odometry blue, vision-only pose magenta다.

## 향후 package 경계

- `brov_base`: MAVLink 단일 소유권과 원본 telemetry 제공
- `brov_perception`: image-space marker detection과 품질 측정, global TF는 소유하지 않음
- `brov_description`: URDF, mesh, camera tilt kinematics
- `brov_localization`: one-shot ArUco alignment와 `pool → odom` TF; continuous
  fusion은 후속 범위
- `brov_viz`: pool marker와 RViz config; InteractiveMarker mission editor는 후속 범위
- `brov_interfaces`: localization status/initialize와 immutable mission typed 계약
- `brov_mission`: pool draft 검증과 current epoch의 odom mission commit
- `brov_bringup`: 노드 구성만 담당하며 control을 자동 시작하지 않음

## 단계별 구현 순서

1. **P0 — 완료:** frame, ownership, UI 계약을 기록했다.
2. **P1 — backend 완료:** 같은 MAVLink snapshot에서 stamped Odometry/TF와 session
   identity를 만든다. URDF/trail은 후속이다.
3. **P2 — 완료:** neutral-locked tilt와 marker survey를 사용한 raw pool pose 및
   RViz vision ghost를 제공한다.
4. **P3 — backend 완료/editor 보류:** explicit camera-neutral confirmation,
   `pool → odom` initialize, boot-unique exact-transform identity, typed
   Draft/Validate/Commit과 PREPARE → ARM → START gate를 구현했다.
   InteractiveMarker editor는 후속이다.
5. **P4 — robustness:** multi-marker/fusion, camera tilt kinematics, runtime geofence,
   localization loss/reacquisition 정책을 검증한다.
6. **P5 — NBV:** 같은 `pool` frame에서 3-D reconstruction과 candidate viewpoint를
   표시하고, 처음에는 suggestion-only로 운용한다.

## 구현 승인 기준

- TF가 하나의 연결된 acyclic tree이며 edge마다 broadcaster가 하나뿐이다.
- 모든 spatial message에 acquisition-time stamp와 non-empty frame가 있다.
- 측량한 수조 모서리와 marker가 RViz에서 정해진 tolerance 안에 표시된다.
- 재시작해도 `pool` frame과 marker map이 임의로 재정의되지 않는다.
- ArUco의 bias/repeatability/reprojection/covariance/latency/dropout 결과가 기록된다.
- vision pose나 RViz node가 중단되어도 25 Hz control 및 MAVLink owner에 영향이 없다.
- RViz 입력에서 `/brov/thruster_pwm`으로 직접 연결되는 경로가 없다.
