# NBV 및 3-D reconstruction 시각화 roadmap

## 상태와 목적

**상태: 장기 설계 기록만 완료, 알고리즘·sensor·runtime 구현은 보류.**

목적은 수조 내 로봇 pose, 카메라 pose, 3-D reconstruction, candidate viewpoint,
선택된 next-best-view와 계획/실행 경로를 하나의 영구적인 `pool` frame에서 RViz로
검증하는 것이다. RViz는 결과를 표시하고 작업자의 승인을 받는 도구이며, mapping,
NBV 판단 또는 안전 제어의 권한을 갖지 않는다.

현재 시스템에는 RGB image와 camera intrinsics, marker-relative ArUco pose만 있다.
metric depth, point cloud, TSDF/occupancy/mesh, persistent map, NBV runtime 또는 motion
planner는 없다. 단안 RGB와 ArUco만으로 dense metric 3-D reconstruction이 자동으로
생기는 것은 아니므로 depth/sonar/stereo/SfM-MVS 중 어떤 입력을 사용할지 먼저
결정해야 한다.

## frame과 localization 전제

모든 reconstruction 결과는
[RVIZ_POOL_LOCALIZATION_ROADMAP.md](RVIZ_POOL_LOCALIZATION_ROADMAP.md)의 surveyed
`pool` frame에 저장한다. `/brov/start_control` 때 바뀌는 `start_heading` frame은
실험 간 persistent map에 사용할 수 없다.

```text
pool → odom → base_link → camera mount/tilt → camera_optical_frame
```

mapping 전에 다음이 확보되어야 한다.

- timestamped `odom → base_link`와 initialized `pool → odom`
- 획득 시각의 camera TF와 intrinsics/extrinsics
- marker map 또는 다른 절대 anchor의 version
- image/depth/pose 간 time synchronization
- localization validity와 covariance

## 제안 data flow

```text
RGB + CameraInfo + metric depth/sonar + timestamped TF
  → synchronized frame
  → camera-frame metric points
  → exposure-time transform into pool
  → reconstruction/fusion artifact
  → NBV candidate generation
  → pool/geofence/tether/dynamics feasibility validation
  → RViz suggestion + operator approval
  → accepted mission interface
  → existing guidance/control
```

RViz `MarkerArray`는 display 용도로만 사용한다. candidate score, coverage gain,
collision result, rejection reason과 sensor constraint는 향후 typed
`brov_interfaces` message로 정의하며 planner 사이의 API를 marker나 병렬 Float
배열로 만들지 않는다.

## 제안 시각화 interface

아래 이름과 type은 구현 전 proposal이다.

| 목적 | 제안 topic/type | 비고 |
|---|---|---|
| metric depth | `/brov/depth/image` `sensor_msgs/Image` | `32FC1` 또는 `16UC1`, scale 명시 |
| registered points | `/brov/mapping/frame_cloud` `sensor_msgs/PointCloud2` | 획득 camera frame 또는 `pool` |
| fused map | `/brov/mapping/cloud` `sensor_msgs/PointCloud2` | `pool`, downsample level 명시 |
| occupancy/mesh display | `/brov/mapping/markers` `MarkerArray` | RViz 전용 surface/voxel 표현 |
| NBV candidate poses | `/brov/nbv/candidate_poses` `PoseArray` | `pool`, pose 목록 |
| candidate display | `/brov/nbv/candidate_markers` `MarkerArray` | frustum, score 색상, reject 상태 |
| selected NBV | `/brov/nbv/selected_pose` `PoseStamped` | 아직 실행 명령이 아닌 suggestion |
| planned route | `/brov/nbv/planned_path` `nav_msgs/Path` | feasibility 검증 결과 |
| map/NBV health | `/brov/nbv/diagnostics` `DiagnosticArray` | stale TF/depth/localization 포함 |

RViz에는 fused cloud/mesh, 수조 safe volume, robot/camera trajectory, camera frustum,
observed/unobserved 영역, candidate pose, selected NBV 및 planned/executed path를
동시에 표시한다.

## 시간 동기화와 calibration blocker

현재 camera node는 RTP source capture time이 아니라 decode 시점의 ROS `now`를
image stamp로 사용하며 기본 jitterbuffer latency는 약 200 ms다. 로봇이 움직이는
상태에서 이 image를 현재 pose와 결합하면 cloud와 marker pose가 공간적으로
밀릴 수 있다. mapping 전에 RTP/source timestamp 복원 또는 측정한 latency 보정과
TF history 조회를 구현해야 한다.

추가 필수 검증은 다음과 같다.

- 최종 수중 housing/viewport에서 calibration하고 live 해상도와 일치하는지 확인
- 움직이는 camera tilt의 실제 각도와 timestamp 확보
- depth와 RGB의 extrinsic 및 registration 오차 측정
- ArUco/odometry covariance와 outlier/dropout 처리
- TF가 없는 frame을 map에 조용히 통합하지 않고 명시적으로 drop/진단

## map artifact와 재현성

생성 결과는 package share가 아니라 다음과 같은 runtime session에 저장한다.

```text
runtime/maps/<session>/
  source.bag/
  cloud.pcd
  mesh.*
  volume_or_occupancy.*
  metadata.yaml
```

metadata에는 최소한 다음을 기록한다.

- `pool` axis/origin과 pool survey version
- marker map version
- camera/depth intrinsic 및 extrinsic checksum
- source timestamp/clock 및 latency 보정 방식
- reconstruction 알고리즘, resolution, crop/ROI와 parameter
- localization, planner, policy artifact version
- 생성 시각과 source bag

동일 bag과 metadata로 offline map을 재생성할 수 있어야 한다.

## NBV 안전 경계

- mapping/NBV process는 검증된 25 Hz control path와 resource를 격리한다.
- MAVLink는 계속 `brov_base` 하나만 소유한다.
- localization, TF 또는 depth가 stale하면 executable goal을 만들지 않는다.
- candidate는 차체 swept envelope, tether, localization uncertainty로 축소한
  3-D safe volume 밖에 생성하지 않는다.
- suggestion topic에서 PWM으로 직접 연결되는 경로를 만들지 않는다.
- 처음에는 human-approved execution만 허용하고 선택·검증·승인·완료/abort를 기록한다.
- `pool → odom` 보정으로 active waypoint가 jump하지 않도록 transform 시점과
  immutable mission semantics를 정의한다.

기존 simulation의 metric depth, known object center, TSDF 크기/voxel, camera 반경과
discrete spherical move 가정은 4.0 × 1.7 × 1.1 m 수조, tether, 실제 depth source에
그대로 적용할 수 없다. feasibility study와 필요 시 재학습이 선행되어야 한다.

## 단계별 구현 순서

1. **N0 — 현재:** 목적, frame, topic, persistence와 safety 계약만 기록한다.
2. **N1 — bag/TF contract:** stamped odometry/camera/depth와 reproducible rosbag을
   확보한다.
3. **N2 — pool localization:** surveyed ArUco와 camera extrinsic으로 vision pose를
   shadow mode에서 검증한다.
4. **N3 — offline reconstruction:** depth source와 알고리즘을 선정하고 bag으로
   deterministic map을 생성한다.
5. **N4 — online RViz map:** control resource를 침해하지 않는 속도로 map을 표시한다.
6. **N5 — NBV suggestion:** candidate와 selected pose만 표시하고 자동 실행하지 않는다.
7. **N6 — 승인 기반 실행:** geofence/planner 검증 후 작업자가 명시적으로 승인한다.
8. **N7 — autonomous loop:** localization loss, collision, timeout, estop을 포함한 전체
   안전 승인을 통과한 후에만 검토한다.

## 미확정 결정 사항

- metric depth source와 수중 유효 거리/해상도
- offline/online reconstruction 비중
- TSDF, occupancy, SfM/MVS 또는 sonar fusion 선택
- object segmentation/ROI와 coverage 정의
- candidate scoring, motion planner와 vehicle/tether model
- exact custom message/action/service schema
- ArUco layout와 continuous fusion package
- NBV policy export, observation normalization 및 재학습 여부

