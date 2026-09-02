# 실기 수조 세션 — 절차와 고려사항

> 이 문서는 **실기를 돌리는 Ubuntu 랩톱의 Claude 가 읽는다.** 명령은 그대로
> 복사해 쓸 수 있게 적었고, 검증된 것과 안 된 것을 구분해 표시했다.
> 작성 2026-08-31. **2026-09-02 세션 결과와 학습 쪽 이관은
> [deadtime_result_to_training.md](deadtime_result_to_training.md) 에 정리했다.**

## 이 세션의 목적

정책 성능 확인이 아니다. **세 가지 측정**이 목적이다.

1. **dead time** — 명령이 효과를 내기까지의 왕복 시간. 학습에 주입할 값의 근거.
2. **DVL vs EKF 편차** — 2026-08-28 수조에서 EKF 가 DVL 대비 12.9% 과소, heave
   축 부호 반대였다. 지금도 그런지.
3. **깊이 센서 게이트** — EKF 수직 위치를 믿을 수 있는지.

진동은 **나올 것으로 예상한다.** 이전 실기에서 3축 진동을 보셨고, 아래 §배경의
기전상 dead time 이 있으면 나온다. 그것을 없애는 게 이번 목적이 아니라,
**정량적으로 재는 것**이 목적이다.

---

## 배경 — 왜 dead time 인가

2026-08-31 Gazebo SITL 조사 결과다. 요약만 적는다. 자세한 것은
`plots/fig4/README.md`(OceanRL_test)와 artifact 페이지에 있다.

정책은 IsaacLab 에서 학습됐고 **거기엔 dead time 이 구조적으로 없다.** 환경이
같은 프로세스에서 action 을 물리에 적용하고 그 직후 관측을 읽으므로,
`obs_{t+1}` 에 `a_t` 의 효과가 이미 들어 있다. 배포는 다르다 — MAVLink, 라우팅,
ArduSub, 추진기, telemetry 를 왕복하는 데 시간이 걸린다.

순수 시간지연은 **크기를 하나도 깎지 않고 위상만 더한다**(`e^(-s*tau)`).
폐루프가 진동하려면 어떤 주파수에서 루프 이득 >= 1 이고 총 위상 = -180도 여야
하는데, dead time 은 그 -180도 교차점을 **기체가 아직 잘 반응하는 낮은
주파수로 끌어내린다.** 그래서 이득 조건이 함께 성립해 진동이 굳는다.

측정된 것:

| | IsaacLab | Gazebo SITL |
|---|---|---|
| dead time | 0 | 60~80 ms |
| 2 Hz 총 위상 | -126도 | -166도 |
| phase margin (3.06 Hz) | **+64도** | **-24도** |
| 진동 | 없음 | 1.98 Hz, 3축 |

정책의 이득은 실측 `K_p ~ 4.5` 이고, tau=60ms 의 안정 문턱이 4.4 다.
**경계에 걸쳐 있다.** 같은 이득의 단순 비례 제어를 같은 경로에 넣으니 1.99 Hz
로 똑같이 떨었다 -- 신경망이라서가 아니다.

**실기는 SITL 보다 나쁠 것으로 본다.** Gazebo 는 추진기를 즉시 선형으로 두어
위상을 18도 과소평가한다. 실기에는 T200 의 실제 동특성이 있다.

> **2026-09-02 실측: τ = 80 ms** (A2-yaw, §3-C/3-D). SITL(60~80 ms)과 같은
> 등급이었다. 진동도 예측대로 재현됐다(2.0~2.3 Hz, A1). 80 ms 의 구간별 분해
> 계획은 `LATENCY_DECOMPOSITION_PLAN.md`.

---

## 준비물 확인

### 코드

**topside 랩톱에서 전체 재빌드가 필요하다.** `brov_interfaces` 에 메시지 1 개와
필드 3 개가 늘었다.

```bash
cd <brov_ros2 저장소>
rm -rf build/brov_interfaces install/brov_interfaces
colcon build --symlink-install
source install/setup.bash
```

빌드 후 확인:

```bash
ros2 interface show brov_interfaces/msg/BrovState | grep -E "depth_source|depth_baro|ekf_velocity"
ros2 interface show brov_interfaces/msg/DvlSample | head -3
```

세 필드와 새 메시지가 보이면 된다.

#### 네이티브(도커 없이) 실행 준비 — 2026-09-02 확인

`source env_native.sh` 로 돌린다. 이 랩톱에서 실제로 걸렸던 것 셋:

1. **인터프리터 capability.** `/usr/bin/python3.10` 에 `cap_sys_nice` 가 붙어 있으면
   glibc 가 secure-execution mode 로 띄우고 `LD_LIBRARY_PATH` 를 **환경에서
   지운다.** `env_native.sh` 가 제대로 세팅해도 소용이 없고, `AMENT_PREFIX_PATH`
   는 살아남아 "패키지는 찾는데 `.so` 만 못 여는" 형태가 된다:
   `ImportError: librcl_action.so: cannot open shared object file`.
   `env_native.sh` 가 이제 이것을 검사해 경고한다. 해제는 `sudo setcap -r`.
   libfranka 실시간 제어에는 필요 없다 — `SCHED_FIFO` 는 `limits.conf` 의
   `@realtime rtprio 99` 만으로 되고, 그쪽은 secure mode 를 만들지 않는다.
2. **런타임 의존성.** 네이티브에는 없다:
   `python3 -m pip install --index-url https://download.pytorch.org/whl/cpu "torch>=2.1,<3"`
   와 `python3 -m pip install "pymavlink>=2.4.40,<3"`.
   **GStreamer 도 모자란다.** `camera_stream_node` 의 파이프라인은
   `udpsrc ! rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! ...` 인데
   Ubuntu 22.04 기본 설치에는 뒤의 둘이 없다 (`h264parse` 는 plugins-bad,
   `avdec_h264` 는 libav). 없으면 노드가 `no element "h264parse"` 로 즉시 죽고,
   **나머지 스택은 멀쩡히 뜬다** -- `/brov/camera/image_raw` 발행자가 0 이 되어
   마커 정렬이 영원히 UNINITIALIZED 에 머문다. Dockerfile 은 둘 다 깐다.
   ```bash
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav
   ```
   확인: `gst-inspect-1.0 h264parse && gst-inspect-1.0 avdec_h264`
3. **setuptools/packaging 불일치.** torch 설치가 `setuptools` 를 올리면
   `colcon build` 가 `canonicalize_version() got an unexpected keyword argument
   'strip_trailing_zero'` 로 깨진다. `python3 -m pip install "packaging>=24"`.

`colcon test` 는 **실기 launch 를 끄고** 돌린다. `test_demo_orchestrator_runtime`
과 `brov_mission/test_runtime_contract` 는 실제 노드를 띄우므로 같은 DDS 도메인에
살아 있는 스택이 있으면 충돌한다. 끄지 않고 돌리려면
`export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1` 로 분리한다.

### 정책 번들

```
artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/
    policy_raw_flu_mk2.pt
    policy_raw_flu_mk2.pt.metadata.json
```

`w_a = 0.017` + plant 수정본. SITL velocity_hold gate 에서 추종률 100.0%.
**실기 시험 이력 없음.**

> `sim2swim_paperfix_wa0017_mk2_s42_i299` 는 `w_a` 수정만 있는 판이다. plant
> 수정이 학습에도 배포에도 측정 가능한 차이를 만들지 않았으므로(reward 곡선
> 차이 0.5 이내, sim2sim cross-track 동급) `fixplant` 를 쓴다.

### 알려진 미해결

- **split stack 은 실기 첫 구동이다.** 이전 실기는 legacy 스택(`obs_node` +
  `policy_node_mk2`)이었다. 2026-08-31 에 arm 게이트, prepare 선행, start/stop
  lifecycle 을 새로 넣었다. §0단계를 건너뛰지 말 것.
- **추진기 클램프 값 — 해소됨 (2026-09-02).** 세 숫자가 전부 실재했고 의미가
  달랐다: ±51.5/64.1 N = 제거된 다항식 잔재(구 시험이 단언 → 시험 갱신으로
  해소), −49.4/+65.9 N = `force_limits_n` 전 전압 포락선(보상 정규화 전용 —
  구 `base_node` 로그가 이걸 찍어 오도 → 로그를 실제 클램프 표기로 수정),
  **−36.7/+47.2 N @14.8 V = `clamp_thrust` 의 실제 배포 클램프**(전압 의존:
  12.6 V −30.3/+38.8, 16.8 V −41.9/+54.5).
- **`test_sim2swim_contract` 2건은 여전히 실패.** 2026-09-01 수조 실험 커밋이
  case_a 프로파일의 `segment_length` 를 2.0→2.2 m 로 바꿨는데 a2 프로파일과
  계약 시험은 안 바꿨다. 어느 값이 정본인지는 **사용자 결정 필요** — 코드 쪽을
  임의로 맞추지 않고 남겨 둔다.
- **dead time 반영 정책은 없다.** 이번 측정값이 나온 뒤에 만든다.

---

## 수조 제약

안전 영역 (`mission_manager_sim2swim_c.yaml` 기준):

```
x 0.35 ~ 3.65 m     (3.3 m)
y 0.30 ~ 1.40 m     (1.1 m)
z 0.20 ~ 0.90 m     (0.7 m)
```

**SITL 의 5 m 사각이나 40 m 직선은 들어가지 않는다.** 직선 왕복만 가능하다.
`pool_mission.launch.py` 가 2.5 m 왕복으로 맞춰져 있다.

**기체 배치가 중요하다.** `waypoint_frame=start_heading` 이라 waypoint 는
`start_control` **순간의 위치와 기수** 기준이다. 기수 방향 2.5 m 앞이 안전 영역
안이어야 한다. x 0.5 m 근처에서 +x 를 보게 두면 끝점이 3.0 m 로 여유가 남는다.

배치에 기대고 싶지 않으면 `pool_demo_a.launch.py` (아래 3-B)를 쓴다. 마커 정렬로
수조 절대 프레임을 세우고 waypoint 를 그 좌표로 읽으므로, 끝점이 안전 영역 안인지
**launch 가 뜨기 전에 검사한다.**

### 속도를 0.25 m/s 로 하는 이유

정책은 V_d = 0.5 로 학습됐지만 **관측에 절대 속도가 없다.** 16-D 는
`[q_e(4), v_e_b(3), omega_b(3), z_v(3), z_q(3)]` 로 오차만 들어간다. 추종이 잘
되면 `v_e ~ 0` 이라 0.25 든 0.5 든 정책이 보는 값이 사실상 같다.

2026-08-31 에 export 된 정책의 Jacobian 을 실제 순항 관측 위에서 재서 확인했다:

| `z_v` 배율 | 0.0 | 1.0 | 5.0 |
|---|---:|---:|---:|
| surge `K_p` | -4.33 | -4.45 | -4.93 |
| heave `K_p` | -3.83 | -3.90 | -3.85 |

**이득이 속도와 무관하므로 진동 조건도 같다.** 3.3 m 에서 0.5 m/s 면 편도 6.6 초라
선회가 데이터의 절반이고 벽까지 여유가 적다. 0.25 면 편도 10 초다.

> **예측을 미리 적어 둔다:** 0.25 m/s 에서도 진동 주파수가 거의 같아야 한다.
> 크게 다르면 기전 설명에 빠진 것이 있다는 뜻이므로 그것대로 유용하다.

### dead time 측정에 속도는 무관하다

dead time 은 **경로의 성질**이다. 통신·계산·구동에 걸리는 시간이라 기체가
얼마나 빨리 움직이든 같다. 교차상관에 필요한 것은 명령의 여기(excitation)와
충분한 시간이지 전진 속도가 아니다.

---

## 절차

### 0단계 — 무추력 확인 (추진기 안 돔)

```bash
ros2 launch brov_bringup pool_mission.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=<저장소>/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \
  vehicle_model_path:=<저장소>/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=<기록경로>/dry_run \
  send_pwm:=false arm:=false
```

별 터미널에서:

```bash
ros2 topic hz /brov/state                      # 25 Hz 인가
ros2 topic echo /brov/state --once             # valid=true, 자세/속도가 말이 되나
ros2 topic hz /brov/observation                # 25 Hz
ros2 topic echo /brov/cmd/wrench --once        # 정책이 명령을 만들고 있나
```

원시 센서 토픽도 여기서 확인한다. `/brov/state` 는 `depth_source` 가 **고른**
경로 하나만 싣지만, 아래 셋은 고르지 않은 쪽까지 남긴다 -- 깊이 게이트(1단계)를
사후에 bag 만으로 판정할 수 있게 하는 것이 목적이다.

```bash
ros2 topic hz   /brov/sensor/ahrs              # 25 Hz, 자세·각속도 원시값
ros2 topic echo /brov/sensor/depth_ekf --once  # EKF 수직 위치 [m, 아래가 +]
ros2 topic hz   /brov/sensor/pressure0         # baro instance 0/1/2
ros2 topic hz   /brov/sensor/pressure1
ros2 topic hz   /brov/sensor/pressure2
```

**확인할 것**

- `/brov/state` 의 `valid: true`, `reason` 이 비어 있음
- `attitude_age_s`, `position_age_s` 가 0.1 s 미만
- `velocity_source: mavlink_ekf`, `depth_source: mavlink_ekf`
- 자세가 실제 기체 자세와 맞는가 (수동으로 기울여 보고 부호 확인)

lifecycle 도 여기서 확인한다. **추력이 안 나가는 상태이므로 안전하다.**

```bash
ros2 service call /brov/prepare_control std_srvs/srv/Trigger
ros2 service call /brov/arm_control     std_srvs/srv/Trigger
ros2 service call /brov/start_control   std_srvs/srv/Trigger
ros2 topic echo /brov/control_active --once     # send_pwm=false 라 false 여야 함
ros2 service call /brov/stop_control    std_srvs/srv/Trigger
ros2 service call /brov/disarm_control  std_srvs/srv/Trigger
```

> `prepare_control` 은 **SERVO1~8 을 RCPassThru 로 바꾼다.** 실제 설정 변경이다.
> disarm 상태에서 해도 안전하지만 되돌리려면 `disarm_control` 을 불러야 한다.

**prepare 가 실패하면** telemetry 가 아직 안 왔을 가능성이 크다. 몇 초 두고 다시
부른다. SITL 에서 7~9 회 재시도가 필요했다.

**계속 `telemetry 없음` 이면** (2026-09-02 실기, A2 직후 재실행에서 20 회 넘게):
heartbeat 는 오는데 ATTITUDE/LOCAL_POSITION/SCALED_PRESSURE 가 **하나도** 안 오는
상태다. 스트림 요청은 connect 직후 한 번만 나가므로, 라우터가 클라이언트를 늦게
등록했거나 다른 GCS(QGC, BlueOS Cockpit)가 endpoint 를 잡았다 놓으면 이렇게 된다.
`base_node` 가 5 s 마다 무엇이 오고 안 오는지 로그로 찍는다.

```bash
ros2 service call /brov/request_streams std_srvs/srv/Trigger   # 재요청 + 수신 통계
```

`runtime/lifecycle.sh` 는 이 경우 자동으로 재요청한 뒤 재시도한다. QGC/Cockpit 이
열려 있으면 닫을 것. 그래도 안 오면 launch 재실행.

---

### 1단계 — 깊이 게이트 (**주행 전 필수**)

**왜 필수인가.** SITL 에서 EKF 수직 위치가 초기값에 얼어붙었다 -- 기체가 GT 기준
5.8 m 상승하는 동안 `LOCAL_POSITION_NED.z` 가 `-0.10 ~ +0.12 m` 를 보고했다.
같은 메시지의 `vz` 는 상승을 정확히 알고 있었으므로 **속도는 맞고 위치만 적분되지
않는** 상태였다. `EK3_SRC1_POSZ = 1`(BARO)로 소스는 지정돼 있었으니 소스 선택
오설정도 아니다. **원인 미규명.**

guidance 의 수직 LOS 항이 이 값으로 오차를 계산하므로 보정이 전혀 안 나갔고,
폐루프가 부력 드리프트를 7.8 배 증폭해 기체가 1.77 m 떠올랐다.

**수조 깊이 여유는 0.7 m 다.** 같은 증상이면 수초 만에 수면 또는 바닥에 닿는다.

#### 검사 도구 — `diag_depth_gate` (2026-09-02 신설)

`base_node` 가 `/brov/sensor/pressure0..2` 와 `/brov/sensor/depth_ekf` 로 원시값을
내므로, **MAVLink 를 두 번 열지 않고** 토픽만 읽어 판정한다. 주행 스택이 떠 있는
상태에서 돌린다.

```bash
ros2 run brov_base diag_depth_gate
```

Enter 를 누르고 기체를 안전 영역 안에서 **위아래로 천천히 2~3 회** 움직인다
(기본 40 s). **거리를 잴 필요가 없다** — 수조의 z 여유가 0.20~0.90 m 라 1 m 를
내릴 수가 없고, 0.5 m 를 손으로 정확히 재는 것도 현실적이지 않다.

거리 대신 **압력을 기준자로 쓴다.** 반응한 baro 를 ArduSub 자신의 변환식
(9800 Pa/m × SPEC_GRAV)으로 미터로 바꾸고, `depth_ekf` 를 그것에 회귀한다:

    depth_ekf = a * depth_baro + b

`a ~ 1` 이고 R² 가 높으면 통과다. 판정은 네 갈래이고 **두 점 방식으로는 서로
구분되지 않는다**:

| 결과 | 뜻 |
|---|---|
| `PASS` | EKF 가 압력을 따라온다 → `depth_source:=mavlink_ekf` 유지 |
| `FAIL — 얼어붙음` (`a ~ 0`) | 2026-08-29 SITL 과 같은 증상 → `pressure` 로 넘길 것 |
| `FAIL — 부호가 반대` (`a < 0`) | 수직 보정이 반대로 나간다 |
| `FAIL — 배율이 틀리다` / `상관이 낮다` | 따라는 오지만 못 믿는다 |

수직 이동이 5 cm 미만이면 **판정하지 않는다** — 안 움직여 놓고 통과시키면
게이트가 아무것도 막지 않는다. 해수는 `--spec-grav 1.024`.

알려진 거리를 낼 수 있다면 `--drop 0.50` 으로 두 지점 방식이 되고, 그때는 압력의
**절대 배율**까지 함께 검증한다. sweep 방식은 그것을 못 한다(압력을 자로 쓰므로).
다만 논문 5.2 도 `depth_source:=pressure` 도 같은 변환식을 쓰므로, 실제로 필요한
판정은 sweep 쪽의 상대 비교다.

> `BARO_PRIMARY` 는 여전히 FC 에서 읽어 대조해야 한다. `depth_source:=pressure`
> 로 launch 하고 `/brov/prepare_control` 을 부르면 로그에 확정된 instance 와
> `SPEC_GRAV` 가 찍히고, `BrovState.depth_baro_instance` 에도 실린다. 그 값이
> 위 표의 WATER instance 와 **다르면 진행하지 말 것.**

#### (구판 pymavlink 스크립트는 삭제됨 — 2026-09-02)

`diag_depth_gate` 가 대체했다. 구판은 두 지점 깊이차를 손으로 재야 했고, DVL
확인용으로 함께 적혀 있던 `ekf_velocity_variance` 검사는 실기에서 **동작하지
않는 것으로 판명**됐다(항상 0.0 — §2단계 참조). 필요하면 git 이력
(106df62 이전)에 있다.

#### 판정

| 확인 | 통과 조건 | 실패 시 |
|---|---|---|
| `BARO_PRIMARY` | 위 표에서 WATER 로 나온 instance 를 가리킴 | **설정 오류.** 그대로 쓰면 안 된다 |
| `BARO_SPEC_GRAV` | 담수 1.0 / 해수 1.024, 시험 수조에 맞음 | 틀리면 깊이에 2.4% 스케일 오차 |
| 기울기 | WATER instance 가 ~98 hPa/m | 크게 다르면 센서/설정 확인 |
| 내부 baro | 깊이에 거의 반응 없음(~0) | 셋 다 반응하면 판별 불가 -- SITL 은 그랬다 |

**`BARO_PRIMARY` 값을 반드시 기록할 것.** 실기와 SITL 은 probe 순서가 달라
instance 번호가 다를 수 있다. `BrovState.depth_baro_instance` 에도 실린다.

#### 깊이 소스 결정

EKF 수직 위치가 실제 깊이를 따라오는지도 봐야 한다. 위 1 m 하강 중에:

```bash
ros2 topic echo /brov/state --field position   # z 가 1 m 만큼 변하는가
```

- **변한다** → EKF 가 정상. `depth_source:=mavlink_ekf` 유지.
- **안 변한다** → SITL 과 같은 증상. **`depth_source:=pressure` 로 넘길 것.**
  게이트가 통과했으므로 근거가 있다. 자세한 것은 `docs/DEPTH_SOURCE.md`.

---

### 2단계 — DVL 기록 붙이기 (**기본 끔** — 2026-09-02 사고로 강등)

**A50 에 붙으면 BlueOS DVL extension 이 밀려나고 EKF 위치가 죽는다 — 조건이
아니라 확인된 사실이다** (아래 사고 기록). EKF 위치가 필요한 주행에서는 켜지
말 것. `deadtime_test` 처럼 EKF 위치가 필요 없는 주행에서만 `dvl:=true`.

감시는 `./runtime/check_ekf.sh` 로 한다 (`LOCAL_POSITION_NED` 부재가 신호다).
구판이 지시하던 `ekf_velocity_variance` 전후 비교는 **무효** — 사고 당시에도
0.0 이었다. `BrovState.msg` 의 해당 필드 주석도 이 사실을 따라 읽을 것.

```bash
ros2 run brov_control dvl_record_node --ros-args -p dvl_host:=192.168.2.95
ros2 topic echo /brov/dvl/sample --once      # connected: true, valid: true
./runtime/check_ekf.sh                        # LOCAL_POSITION_NED 살아 있는지
```

> **2026-09-02 확인됨 — 조건이 아니라 사실이다.** `dvl_record_node` 가 15:21:35 에
> A50 에 붙었고, **15:22:53** 에 BlueOS DVL extension 의 `VISION_POSITION_DELTA` 와
> `DISTANCE_SENSOR` 가 멈췄으며, **15:22:58** 에 FC 가 `LOCAL_POSITION_NED` 를
> 끊었다(EKF ExtNav timeout → `CONST_POS_MODE`, `POS_HORIZ_REL` 소실). 증상은
> `base_node` 의 `telemetry 없음 — … 안 옴: LOCAL_POSITION_NED` 이고 prepare 가
> 거절된다. `ekf_velocity_variance` 는 그때도 0.0 이라 **분산으로는 잡히지 않았다**
> -- 잡히는 것은 `LOCAL_POSITION_NED` 의 부재다. 노드를 내려도 extension 은 **스스로
> 회복하지 않았다**(25 분 뒤에도 멈춤). 회복은 BlueOS 에서 extension 재시작.
>
> 그래서 `pool_demo_a.launch.py` 의 `dvl` 기본값을 **false** 로 바꿨다. DVL 기록은
> `deadtime_test` 처럼 EKF 위치가 필요 없는 주행에서만 `dvl:=true` 로 켠다.
> 상태 확인: `./runtime/check_ekf.sh` (mavlink2rest 만 읽는다).
> 회복: `./runtime/restart_dvl.sh` (kraken `POST /v2.0/extension/restart`) — 그래도
> 안 되면 로봇 재부팅. 2026-09-02 는 재부팅으로 ~2 분 만에 복귀했다
> (EKF flags 167 → 367, `POS_HORIZ_REL` 회복).

DVL 이 없거나 host 가 틀려도 노드는 죽지 않고 `connected: false` 와 사유를
보고한다.

> **되먹임으로 승격하지 말 것.** 축·부호 변환이 미확정이고(수조에서 heave 부호가
> 반대였다), DVL 은 5~15 Hz 라 2 Hz 에서 -36도 이상을 더한다. 지금 phase margin
> 이 -24도 라 진동이 나빠진다. `base_node` 의 `velocity_source` 가드가 그래서
> 거부한다.

---

### 3단계 — 주행

```bash
ros2 launch brov_bringup pool_mission.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=<저장소>/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \
  vehicle_model_path:=<저장소>/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=<기록경로>/pool_wa0017_run1 \
  depth_source:=<1단계 결정값> \
  cruise_speed:=0.25 leg_m:=2.5 \
  send_pwm:=true arm:=true
```

```bash
ros2 service call /brov/prepare_control std_srvs/srv/Trigger
ros2 service call /brov/arm_control     std_srvs/srv/Trigger
ros2 service call /brov/start_control   std_srvs/srv/Trigger
#   ... 60 초 ...
ros2 service call /brov/stop_control    std_srvs/srv/Trigger
ros2 service call /brov/disarm_control  std_srvs/srv/Trigger
```

**60 초면 왕복 3 회, 2 Hz 진동 기준 120 주기.** 교차상관에 충분하다.

#### 3-B. 마커 절대 프레임으로 도는 판 (`pool_demo_a.launch.py`)

위 3단계는 `waypoint_frame=start_heading` 이라 경로가 **기체를 놓은 자리**에
달려 있다. 배치가 틀리면 2.5 m 앞이 벽 밖이고, 게이트는 하나도 걸리지 않는다 --
guidance 의 한계 검사는 세그먼트 **길이**만 보지 경계 상자를 모른다.

`pool_demo_a.launch.py` 는 `sim2swim_demo.launch.py case:=a` 의 마커 기반 절대
프레임을 같은 분리 스택 위로 옮긴 것이다. waypoint 를 수조 좌표(Z-up [m])로 직접
쓰므로 기체를 어디에 놓든 끝점이 안전 영역 안이고, launch 가 그것을 뜨기 전에
검사한다. DVL·원시 센서·마커 토픽까지 **기본으로 기록한다.**

```bash
ros2 launch brov_bringup pool_demo_a.launch.py \
  connection:=udpout:192.168.2.2:14550 \
  policy_path:=<저장소>/artifacts/policies/sim2swim_fixplant_wa0017_mk2_s42_i299/policy_raw_flu_mk2.pt \
  vehicle_model_path:=<저장소>/brov_base/brov_base/vendor/brov2_heavy.yaml \
  bag_path:=<기록경로>/pool_demo_a_run1 \
  depth_source:=<1단계 결정값> \
  cruise_speed:=0.25 leg_m:=2.5 \
  send_pwm:=true arm:=true
```

주행 전에 **정렬을 한 번** 맞춘다. 기체를 마커가 보이는 곳에 **정지**시켜 둔다
(`stationary_linear_speed_mps: 0.03` 아래여야 샘플이 쌓인다).

```bash
ros2 service call /brov/localization/confirm_camera_tilt_neutral std_srvs/srv/Trigger
ros2 service call /brov/localization/initialize_pool \
    brov_interfaces/srv/InitializePool "{min_samples: 20}"
ros2 topic echo /brov/localization/status --once     # state: 2, output_valid: true
```

그 다음은 3단계와 같은 lifecycle 이다. **정렬 전에 start 하면 guidance 가 목표를
내지 않고** base watchdog 이 0.25 s 안에 중립 정지시킨다 -- 절대 좌표 경로를 절대
프레임 없이 따라가는 것보다 안전하다.

정렬은 **한 번만** 본다. `initialize_pool` 이후 `pool_alignment_node` 는 vision 을
무시하고 EKF odometry 로 절대 위치를 이어가므로, 주행 중 마커가 안 보여도 된다.
반대로 EKF 원점/yaw 리셋이 나면 그 절대 위치가 조용히 틀어지므로, `base_node` 가
인접 샘플 도약을 검사해 odometry session 을 진행시키고, 그러면 정렬이 무효가 되어
guidance 가 멈춘다.

기체 배치 방식으로 돌리려면 `frame:=start_heading` 을 준다. `drag_test` 와 같은
방식이고, 그때도 카메라/ArUco 는 그대로 떠서 bag 에 절대 위치를 남긴다
(`markers:=false` 로 끈다) -- EKF 적분 드리프트를 사후에 잴 유일한 기준이다.

| | `frame:=marker` (기본) | `frame:=start_heading` |
|---|---|---|
| waypoint 좌표 | 수조 절대 (Z-up [m]) | start 순간 위치·기수 기준 |
| 벽 여유 | launch 가 검사한다 | **기체 배치가 유일한 방어** |
| 주행 전 필요한 것 | 카메라 보정, 마커 가시, 정지 상태 정렬 | 없음 |
| 정렬이 깨지면 | guidance 가 멈춘다 | 영향 없음 |

#### 중단 기준 — 하나라도 걸리면 즉시 `stop_control`

- 벽까지 0.5 m 이내
- 깊이가 안전 영역(0.20~0.90 m)을 벗어남
- 진동이 눈에 띄게 커짐
- `/brov/state` 의 `valid: false` 가 지속

`stop_control` 은 **armed 를 유지한 채** 제어만 멈추고 중립을 보낸다.
`start_control` 로 바로 재개된다. 완전히 내리려면 `disarm_control`.

**watchdog 도 있다.** 명령이 0.25 s 끊기면 base 가 자동으로 중립 정지한다.

---

### 3-C. dead time 전용 측정 (`deadtime_test.launch.py`) — **권장**

3단계/3-B 의 주행 bag 으로도 `diag_loop_delay` 는 돌지만, 그 주행의 여기(excitation)
는 **선회와, 혹시 난다면 진동**뿐이다. 정책이 잘 추종하면 순항 중 명령이 거의
일정해서 교차상관이 서지 않는다 -- 진동이 나야 측정되는, 우연에 기댄 실험이다.

dead time 은 경로의 성질이라 정책이 필요 없다. 절단면이 wrench 이므로 정책 자리에
알려진 신호를 넣는다. `base_node` 아래로는 아무것도 달라지지 않는다.

```bash
ros2 launch brov_bringup deadtime_test.launch.py \
  axis:=heave amplitude:=20.0 bias:=0.0 duration_s:=60 \
  bag_path:=<기록경로>/deadtime_heave \
  send_pwm:=true arm:=true
# prepare -> arm -> start. duration_s 뒤 여기가 스스로 멈춘다. stop -> disarm.
ros2 run brov_base diag_loop_delay <기록경로>/deadtime_heave \
  --axis heave --open-loop --seconds 55
```

- `--open-loop`: 폐루프 진동 전제의 "지배 주파수 불일치" 비교를 끈다. 개루프에서
  지배 주파수는 **우리가 넣은 신호**라 그 줄은 무의미하다.
- `--seconds`: `duration_s − skip`. 여기가 끝난 뒤 `control_active` 는 계속 true 라
  명령 0 인 꼬리가 분석에 섞인다.
- `bias`: 부력 상쇄. 첫 시도는 0 으로 30 s 돌려 드리프트를 보고 정한다.

#### 2026-09-02 실기 결과 (heave, 1 Hz 사각파 20 N, 30 s)

```
피크 lag = 80.0 ms,  r = +0.661
lag 프로파일: 0 ms 0.603 / 60 ms 0.657 / 80 ms 0.661 / 100 ms 0.654 / 140 ms 0.516
명령 25.0 Hz p90 40.2 ms,  상태 25.0 Hz p90 42.0 ms  (링크 결손 없음)
```

| | IsaacLab | SITL 직결 | SITL mavproxy | **실기** |
|---|---|---|---|---|
| dead time | 0 | 60 ms | 80 ms | **80 ms** |
| K_p 문턱 (heave) | — | 4.40 | — | **3.52** |

**정책 이득 4.5 가 문턱 3.52 를 28% 넘는다.** SITL 은 4.5 vs 4.4 로 경계였다.

봉우리가 평평한 것(0~120 ms 에서 r 0.60~0.66)은 **1 Hz 협대역 여기의 분해능
한계**다 -- 교차상관 봉우리 반폭이 ~250 ms 라 원리적으로 더 좁게 못 잰다.
jitter 판정은 대역폭을 넓힌 뒤에야 할 수 있다. 다만 결론은 τ 에 견딘다:

| τ | 60 ms | 80 ms | 100 ms |
|---|---|---|---|
| K_p 문턱 | 4.40 | 3.52 | 2.93 |
| 정책 4.5 | 초과 2% | 초과 28% | 초과 53% |

r ≥ 0.65 인 60~100 ms 어디에 진짜 τ 가 있어도 진동 조건은 성립한다.

봉우리를 좁히려면 (선택):

```bash
# chirp 0.5 -> 8 Hz: 반폭 ~30 ms
ros2 launch brov_bringup deadtime_test.launch.py \
  axis:=heave kind:=chirp chirp_f0_hz:=0.5 chirp_f1_hz:=8.0 \
  amplitude:=20.0 duration_s:=40 bag_path:=<기록경로>/deadtime_heave_chirp \
  send_pwm:=true arm:=true
ros2 run brov_base diag_loop_delay <기록경로>/deadtime_heave_chirp \
  --axis heave --open-loop --seconds 35
```

### 3-D. 떨림 기전 판별 — 실험 A1 / A2 (2026-09-02 추가)

되먹임 루프가 떠는 조건은 "보정이 늦게 오고 **동시에** 세다" 다. 지연 80 ms 와
세기 4.5(문턱 3.52)는 각각 쟀지만, 실제 떨림이 그 조합 때문인지는 아직 모른다.

기체가 음성부력이라 start 전에는 바닥에 있다. 두 실험 모두 **바닥에 놓고 start**
한다: A1 은 `rise_m:=0.4` 로 정책이 0.4 m 띄운 뒤 왕복하고, A2 는 여기 노드의
느린 깊이 루프가 0.4 m 띄운 뒤 지킨다(무게를 몰라도 된다 -- 적분항이 맞춘다).

**A1 — 세기 ½.** `wrench_gain:=0.5` 로 60 s, 같은 배치에서 `1.0` 으로 60 s.

| 0.5 에서 | 뜻 |
|---|---|
| 떨림 사라짐 | 지연 + 세기 (위상 예산) |
| 남되 느리고 작아짐 | ESC deadband limit cycle |
| 주파수 그대로, 진폭 절반 | 정책 자체의 chatter |

```bash
./runtime/a1_gain.sh 1.0 ; ./runtime/lifecycle.sh ; (60 s) ; ./runtime/stop.sh
./runtime/a1_gain.sh 0.5 ; ./runtime/lifecycle.sh ; (60 s) ; ./runtime/stop.sh
ros2 run brov_base diag_loop_delay "$(./runtime/latest_bag.sh a1_gain10)" --axis heave   # 지배 주파수·r 비교
ros2 run brov_base diag_loop_delay "$(./runtime/latest_bag.sh a1_gain05)" --axis heave
```

**A2 — 역전 없는 지연 (yaw).** 오늘 80 ms 는 heave 사각파가 매 에지마다 추진기를
0 으로 관통시킨 값이라 ESC 역전 지연이 섞여 있다. yaw 에 bias 를 걸면 추진기가
한 방향으로 계속 돌고 기체는 제자리에서 천천히 돈다. 각속도는 자이로 직접이라
EKF 속도 융합 지연도 빠진다 -- **통신+추진기 지연만** 남는다.

```bash
./runtime/a2_yaw.sh ; ./runtime/lifecycle.sh ; (40 s 뒤 자동 정지) ; ./runtime/stop.sh
ros2 run brov_base diag_loop_delay "$(./runtime/latest_bag.sh a2_yaw)" --axis yaw --open-loop --seconds 35
```

| A2 결과 | 뜻 |
|---|---|
| 여전히 ~80 ms | 통신 경로 자체. 학습에 고정 지연 주입 |
| ~30~40 ms | 절반이 역전. 문제는 0 을 넘나드는 상황(hover)에 집중 → deadband 쪽 |

#### 2026-09-02 A2 결과 (yaw, 1 Hz 사각파 bias 1.0 ± 0.5 N·m, 40 s, 바닥에서 0.4 m 띄움)

```
피크 lag = 80.0 ms,  r = +0.809          (heave 역전 포함: 80.0 ms, r = 0.661)
lag 프로파일: 0 ms 0.742 / 60 ms 0.801 / 80 ms 0.809 / 100 ms 0.781 / 140 ms 0.571
명령 25.0 Hz p90 40.3 ms,  상태 25.0 Hz p90 40.5 ms
깊이: +0.85 → +0.47 m (8 s), 이후 30 s 동안 ±2 cm 유지
```

**80 ms 는 통신·처리 경로 자체다.** ESC 역전을 빼고 자이로로 직접 재도 같은 값이다.
heave 의 낮은 r(0.66)은 통신 jitter 가 아니라 **역전 시간의 흔들림과 EKF 속도 융합
지연·잡음**이었다 -- 그것을 빼자 0.81 로 올랐다. 학습에는 **고정 80 ms** 를 넣는
것이 근거를 갖는다. 봉우리가 여전히 넓은 것(0 ms 0.74 → 80 ms 0.81)은 1 Hz 협대역의
분해능 한계이고, jitter 상한을 더 좁히려면 chirp 로 한 번 더 잰다(선택).

bag 이름이 겹치면 launch 가 시각을 붙인다. 분석은 항상
`"$(./runtime/latest_bag.sh a2_yaw)"` 처럼 최신 것을 고를 것.

launch 를 Ctrl+C 로 내리면 기록기가 `metadata.yaml` 을 못 쓰고 죽는다
(`rosbag2_storage ... disk I/O error` 가 그 증상). `diag_loop_delay` 는 sqlite 를
직접 읽어 문제없지만 다른 도구는 못 연다. 세션 끝에 한 번:
`./runtime/reindex_bags.sh` — metadata 없는 bag 을 전부 복구한다.

#### 2026-09-02 A1 결과 (start_heading, leg 1.0 m, rise 0.5 m, 0.25 m/s, DVL off)

`grep` 한 줄 요약은 명령 스펙트럼이 0.04 Hz(미션 주기)에 눌려 "실측 지배 주파수"
줄이 빠졌으므로, 1.8~2.6 Hz 대역의 **절대** RMS 로 본다 (`runtime/analysis/a1_band.py`,
`a1_saturation.py`, `a1_legs.py` 의 출력):

| 축 | 명령 2 Hz 대역 RMS (1.0 → 0.5) | 응답 2 Hz 대역 RMS (1.0 → 0.5) | 주파수 |
|---|---|---|---|
| surge | 7.4 → 5.9 N (×0.80) | 0.31 → 0.30 m/s² (×0.96) | 2.2~2.3 Hz |
| sway | **21.1 → 6.3 N** (×0.30) | 0.90 → 0.49 m/s² (×0.54) | 1.95~2.1 Hz |
| heave | 6.9 → 3.7 N (×0.54) | 0.36 → 0.27 m/s² (×0.74) | 2.1~2.2 Hz |
| roll | **10.4 → 4.6 N·m** (×0.44) | 6.9 → 5.0 rad/s² (×0.72) | ~2 Hz |
| pitch | 5.7 → 2.2 N·m (×0.39) | 5.1 → 3.1 rad/s² (×0.61) | ~2 Hz |
| yaw | 5.7 → 2.3 N·m (×0.40) | 3.5 → 2.2 rad/s² (×0.63) | 1.95 Hz |

- **행동 포화 (|a| ≥ 0.99)**: gain 1.0 에서 surge **73 %**, pitch 43 %, yaw 26 %;
  gain 0.5 에서도 surge 62 %, sway 26 %. surge 평균 행동 +0.80 (68 N).
- **추진기 클램프**: gain 1.0 에서 T1 51 %, T2/T6 13 %. gain 0.5 에서 0 %.
- **적분기**: `|z_v|,|z_q| ≥ 4.9` 0 % — windup 아님.
- **실제 속도**: gain 1.0 surge 평균 **0.14 m/s**(목표 0.25), gain 0.5 **0.02 m/s**.
  다리 하나에 11 s / 26 s (직진이면 4 s). 각속도 RMS 0.5~0.9 rad/s -- 기체가 2 Hz 로
  구르며 거의 전진하지 못한다.
- **개루프 대조** (A2 yaw, deadtime heave): 응답의 2 Hz 대역이 명령이 실은 것 이상으로
  없다 (0.07 vs 1 Hz 대역 0.5) → **기체·추진기의 자체 2 Hz 공진은 없다.**

**판정.** 이득을 절반으로 해도 진동이 **사라지지 않고 진폭만 ×0.4~0.8 로 줄었으며
주파수는 그대로**다. 선형 "지연 + 이득" 루프라면 임계의 0.64 배에서 꺼져야 한다.
꺼지지 않는 것은 정책이 대부분의 시간 **포화(relay)** 상태이기 때문이다: 포화된
제어기 + 80 ms 지연은 −180° 교차 주파수(예측 2.39 Hz, 실측 2.0~2.3 Hz)에 **진폭이
포화 수준(=이득)에 비례하는 limit cycle** 을 만든다. 2×2 표의 왼쪽 아래 칸이되, 원인은
deadband 가 아니라 **포화**다 (수직 추진기 동작점 6 N/추진기로 deadband 밖).

가장 큰 진동 축은 **sway–roll** (sway 21 N, roll 10 N·m = 권한의 40 %) 이다. 측면
추력이 CoM 아래 레버암으로 roll 을 만들고, 그 둘이 80 ms 지연을 통해 서로를 되먹인다.
surge 는 클램프에 붙어 전진력을 내지만 진동이 권한을 잡아먹어 0.14 m/s 밖에 못 낸다.

**따라오는 것.**
- 이득만 낮추는 것은 해법이 아니다 (relay cycle 은 진폭만 줄인다). 필요한 것은
  학습에 **지연 주입**(정책이 고주파 이득을 스스로 낮추도록), 또는 배포에서 행동
  **변화율 제한/저역필터**로 relay 거동을 깨는 것 -- 후자는 위상을 더 깎으므로 SITL
  에서 먼저.
- `leg_m` 1.0 은 lookahead 1.0 과 같아 다리가 안착하지 못했다. 다음 주행은 2.0 m 이상.
- 1저자 질문에 추가: 수조 시험의 **행동 포화 비율**과 sway/roll 진동 유무.

### 4단계 — 분석

```bash
ros2 run brov_base diag_loop_delay <기록경로>/pool_wa0017_run1 --axis heave
ros2 run brov_base diag_loop_delay <기록경로>/pool_wa0017_run1 --axis surge
```


기록기가 정상 종료되지 못해 `metadata.yaml` 이 없어도 읽힌다(sqlite 직접 읽기로
폴백한다).

#### 읽는 법

```
피크 lag = XX ms,  r = +0.XXX
lag 프로파일: ...
지배 주파수   명령 X.XX Hz   응답 X.XX Hz
-180도 교차 예측 주파수 = X.XX Hz
이 주파수에서 |L|=1 이 되는 이득 문턱 K_p = X.XX
```

| `r` / 프로파일 | 의미 | 다음 |
|---|---|---|
| r > 0.8, 봉우리 뾰족 | 지연이 고정에 가까움 | 피크 lag 를 학습에 주입 |
| r < 0.5, 프로파일 평평 | jitter 가 큼 | **고정값이 아니라 분포**로 주입 |

SITL 은 후자였다(mavproxy 경유 r = 0.487, 직결 0.934). 실기도 BlueOS 라우팅을
거치므로 같은 서명이 나올 수 있다.

> `r` 은 Pearson 상관계수다. `r^2` 이 "가속도 변동 중 명령으로 설명되는 비율"
> 이다. 낮다고 곧 jitter 는 아니다 -- 다른 힘(항력·부력·커플링), 포화로 인한
> 비선형, 속도 미분 잡음도 낮춘다. SITL 에서 jitter 로 본 근거는 값이 아니라
> **변화** 다: 라우팅 홉 하나만 빼고 나머지를 동일하게 뒀는데 0.487 -> 0.934 가
> 됐다.

#### 예측과 대조

- 진동 주파수가 **1.5~3.5 Hz** 이면 SITL 과 같은 기전
- 예측 주파수와 실측이 어긋나면 jitter 로 실효 지연이 피크보다 크다는 뜻
- **0.25 m/s 에서 SITL(0.5 m/s)과 주파수가 크게 다르면** 기전 설명에 빠진 것이
  있다. 그 자체가 보고할 값어치가 있다

---

## 가져올 것

```
<기록경로>/pool_wa0017_run1/          # bag (3단계) 또는
<기록경로>/pool_demo_a_run1/          # bag (3-B단계, 원시 센서·마커 포함)
1단계 스크립트 출력                     # BARO_PRIMARY, SPEC_GRAV, 기울기
4단계 diag_loop_delay 출력 (heave, surge)
관측 메모                              # 진동이 눈에 보였는지, 어느 축인지
```

3-B 의 bag 에는 `/brov/sensor/pressure0..2` 와 `/brov/sensor/depth_ekf` 가 함께
들어 있으므로, 1단계 게이트를 **bag 만으로 재검증**할 수 있다. 세 압력 중 깊이에
~98 hPa/m 로 반응한 instance 가 물속 센서이고, 같은 구간에서 `depth_ekf` 가 그만큼
움직였는지가 EKF 수직 위치의 판정이다.

이 넷이 다음 학습(지연 주입)의 근거가 된다.

---

## 안 하는 것

- **DVL 되먹임 승격** — 축 변환 미확정, 위상 지연 증가
- **`depth_source:=pressure` 를 게이트 없이 사용** — 근거 없이 기본 동작 변경
- **5 m 사각 / Fig.4 (b)(c)** — 수조에 안 들어감
- **dead time 반영 정책 시험** — 아직 존재하지 않음. 이번 측정 후에 만든다
