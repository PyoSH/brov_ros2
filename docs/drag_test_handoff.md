# BlueROV2 실기 surge 항력 측정 — 수조 시험 이어받기

## 배경 — 무엇을 왜 재는가

Sim2Swim(arXiv:2512.08656) 재현 중이고, 시뮬레이션의 BlueROV2 surge 항력계수가
맞는지 실측으로 판정해야 한다. 판정 대상은 딱 하나다:

    A = drag(0.5 m/s) / 최대 surge 추력

    가설 A (sim 계수 Xu=13.7, Xuu=141):  v_max 0.88 m/s,  A 0.340,  보상최적 추종률 28%
    가설 B (제조사 사양 1.5 m/s):        v_max 1.48 m/s,  A 0.149,  추종률 53%

논문 보상 Eq.(8) `r_a = w_a·e^(-||a||)`는 정상상태에서 0이 되지 않는 행동 비용을
만들고, 속도항 Eq.(6) `e^(-||v_e||²)`는 오차 0 근방에서 기울기가 0이다. 두 항의
균형점이 정책이 수렴할 속도이고, 그걸 정하는 값이 A다. 실제로 학습된 정책 5개 중
4개가 0.5 m/s 명령에서 0.2 m/s 이하로 수렴하다 후진까지 갔다.

**측정 전 사전 증거**: `C_D = Xuu/(½ρA)`, 전면적 0.338×0.254=0.086 m² →
`Xuu=141`은 C_D **3.28**(개방프레임 실효면적 0.060이면 4.70)을 함의한다. 평판이
1.2, 낙하산이 1.4다. 물리적으로 불가능하다. `Xuu≈46`은 1.08로 정확히 예상 범위.
방향은 이미 보이지만 크기는 실측이 답한다.

## 이미 되어 있는 것 (다시 만들지 말 것)

`brov_control`에 ROS2 노드가 구현·검증되어 있다. origin/main의 `e0ead32`에 있다.

| 파일 | 역할 |
|---|---|
| `brov_control/brov_control/drag_test.py` | 순수 로직 (ROS/MAVLink 없음) |
| `brov_control/brov_control/drag_test_node.py` | 노드 본체 |
| `brov_control/config/drag_test.yaml` | 파라미터 49개 |
| `brov_bringup/launch/drag_test.launch.py` | 구성 |
| `brov_control/test/test_drag_test.py` | 시험 40개 |

`brov_control` 시험 72개(기존 32 + 신규 40) 전부 통과 상태다.

`brov_base/brov_base/diag_terminal_velocity.py`는 MAVLink 직결 대안이다(같은
적합 함수 `fit_drag`/`report`/`_Allocator`를 노드가 import해서 쓴다). 노드와
동시에 쓸 수 없다 — MAVLink 소유자가 둘이 된다.

### 설계에서 반드시 지켜야 할 것
- **surge는 open-loop다.** 속도로 피드백하면 측정이 무의미해진다. 이 시험은
  "정해진 추력에서 나오는 종단속도"를 재는 것이다. `model_based_controller`가
  surge를 속도 PI로 닫기 때문에 그것을 쓸 수 없고, 이 노드는 surge만 open-loop로
  두고 나머지 5축(sway/heave/roll/pitch/yaw)을 닫는다.
- **yaw는 유지 구간에도 놓지 않는다.** 이전에 유지 루프에 yaw가 빠져 있어서
  호버링 중 기체가 계속 회전했다. 지금은 전 구간에서 잡는다.
- **ArUco 정렬은 one-shot이다.** 마커는 초기화 시점의 상수 yaw 기준만 주고,
  시변 방위는 ArduSub EKF3/AHRS에서 온다. 주행 중 방위 드리프트를 보정하지
  못한다(1.8m/약 3초 주행에서는 무시할 수준이라 문제없다). 정렬이 실제로 주는
  것은 **절대 pool 위치** — 벽까지 남은 거리, 차선 유지, 주행거리다.
- **DVL은 직접 소비되지 않는다.** ArduSub EKF3가 `EK3_SRC1_VELXY`로 융합한
  결과가 `LOCAL_POSITION_NED`로 올 뿐이다. DVL 토픽은 없다.

## 확정된 시험 환경 (다시 묻지 말 것)

- 수조 **3~4 m**(주행축) × 폭 **2 m** × 수심 **1.15 m**
- ArUco 마커 설치되어 있음 (데모와 동일 배치, `DICT_APRILTAG_16h5` id=2,
  변 0.42 m, pool 좌표 x=3.95 y=0.85 z=0.35)
- 배터리 15.3~15.4 V → **τ_max 123.0~123.9 N**, PWM 비포화 한계 **level 0.87**
- 순중량 **200 g 음성** (= 1.96 N, `buoyancy_n`)
- 연결: **`udpout:192.168.2.2:14550`**
  BlueOS는 14550에서 **UDP 서버**로 동작한다. `udpin:`(기체가 먼저 보내주기를
  기다리는 방식)은 절대 안 된다 — 실측으로 확인했다.

## 이미 검증된 것 (재조사하지 말 것)

macOS/Docker 환경에서 전 계통이 동작하는 것까지 확인했다:

```
obs_node       : heartbeat 수신 system 1 comp 1, "첫 healthy mavlink_ekf feedback 확보"
                 배선 프로파일 real_brov2 sign=[1,-1,-1,1,1,1,1,-1]
camera_stream  : decode 16 fps, RTP pushed 8948, lost=0
aruco_pose     : "marker id=2 acquired"
drag_test_node : τ_max 123.9 N, 비포화 한계 level 0.87, IDLE 정상
MAVLink 탐침    : 8초에 2014 패킷, msgid 30/31/32 포함
```

fail-closed도 두 경로 확인했다 — 정렬 없이 start는 거절, 서비스 없으면 prepare 거절.
PWM은 승인 전 한 번도 나가지 않는다.

## 지금 막혀 있는 지점

```
$ ros2 service call /brov/drag_test/prepare std_srvs/srv/Trigger
response: success=False, message='fresh local odometry is unavailable'
```

이건 `localization_node`가 내는 메시지다(`_on_initialize`). `/brov/odometry/
local_with_session`(`brov_interfaces/OdometrySession`)을 **0.5초 이내 신선도로**
받지 못했다는 뜻이다. 그 토픽은 `obs_node`가 낸다.

관측된 사실: `obs_node`가 프로세스 시작부터 "첫 healthy feedback 확보"까지
**약 4.5초** 걸린다. 그 전에 `prepare`를 부르면 이 메시지가 난다.

**첫 조치**: launch 후 `/brov/odometry/local_with_session`이 실제로 나오는지
확인하고 나서 `prepare`를 부를 것.

```bash
ros2 topic hz /brov/odometry/local_with_session     # 25 Hz 나와야 함
ros2 topic echo /brov/localization/status --once    # state / output_valid / sample_count
```

그래도 안 되면 `localization_node` 쪽 조건을 하나씩 볼 것 — 정렬 샘플은
**정지 상태**에서만 수집된다(`linear_speed ≤ 0.03 m/s`, `angular_speed ≤ 0.05
rad/s`), 마커가 보여야 하고(`/brov/aruco/visible`), 카메라 틸트 중립 확인이
선행되어야 한다. `min_samples`는 20 미만으로 낮출 수 없다(안전 하한).

## 새 랩톱 준비

```bash
git clone https://github.com/PyoSH/brov_ros2.git   # 또는 git pull
cd brov_ros2
colcon build --packages-select brov_interfaces brov_base brov_control \
                               brov_localization brov_perception brov_bringup
source install/setup.bash
python3 -m pytest brov_control/test/ -q      # 72개 통과해야 함
```

colcon은 파이썬 패키지를 `build/`로 **복사**한다. 소스만 고치고 재빌드 안 하면
반영 안 된다. 이 함정을 실제로 한 번 밟았다.

리눅스에서는 Docker `network_mode: host`가 진짜 host 네트워킹이므로 macOS에서
겪던 NAT 문제가 없다. 네이티브로 돌려도 되고 컨테이너로 돌려도 된다.
연결 문자열은 어느 쪽이든 `udpout:192.168.2.2:14550`이다.

## 시험 절차 — 단계적으로

### 1단계 · 추력 없이 정렬만
```bash
ros2 launch brov_bringup drag_test.launch.py send_pwm:=false
# 토픽이 살아있는지 먼저 확인한 뒤에:
ros2 service call /brov/drag_test/prepare std_srvs/srv/Trigger    # 기체 정지 상태로
```
정렬이 `INITIALIZED`가 되는지, **pool 포즈가 수조 실측과 맞는지** 확인한다.

여기서 두 값을 실측에 맞춰 조정할 것:
- `axis_heading_rad` — pool +X가 실제 주행축과 맞는가
- `run_x_min` / `run_x_max` (기본 0.50 / 2.60) — 실제 벽 위치 기준으로

### 2단계 · 1수준만
```bash
ros2 launch brov_bringup drag_test.launch.py send_pwm:=true arm:=true \
    --ros-args -p levels:="[0.10]" -p repeat_first_level:=false
ros2 service call /brov/drag_test/prepare std_srvs/srv/Trigger
ros2 service call /brov/drag_test/start   std_srvs/srv/Trigger
```
`ARMED` 유지 30초 구간에 **테더를 놓는다**(손을 놓아 느슨하게 — 통신선이자
안전줄이니 분리하지 말 것). 그 구간에서 **깊이와 방위가 잡히는지** 반드시
확인한다. 이전 회전 문제의 직접 검증이다. 안 잡히면 즉시
`/brov/drag_test/stop`.

기체는 **부유 상태로 시작**한다(바닥 아님). 바닥에 두면 DVL bottom lock을 잃고,
상승 과도가 거리 예산을 깎고, 수직 추력기가 바닥을 때려 재순환을 만든다.

### 3단계 · 전 수준
```bash
ros2 launch brov_bringup drag_test.launch.py send_pwm:=true arm:=true
```
기본 수준은 `[0.10, 0.20, 0.32, 0.45, 0.60]`이고 `repeat_first_level: true`라
최저 수준이 끝에 한 번 더 들어간다(재순환 편향 검사용).

## 관찰 — 버릴 수준 판단

정상 출력:
```
정상상태 u = +0.412 m/s (sd 0.021, du/dt -0.0040), 전달 추력 69.1 N, 주행 1.62 m
```

| 출력 | 뜻 | 조치 |
|---|---|---|
| `사용 불가 — du/dt ...` | 종단속도 전에 끝났다 | 해당 level 낮춤 |
| `[경고] 주행 중 방위 편차 N°` | 직진이 안 됐다 | yaw 권한/AHRS 확인 |
| `sd` ≥ 0.05 | 재순환/테더 흔들림 | `inter_level_wait_s` 증가 후 재측정 |
| `[경고] 재순환 편향` | 반복 측정이 5% 이상 차이 | 대기시간 2배로 재측정 |

**거리 한계로 중단돼도 표본은 유효하다.** 중단 사유와 무관하게 마지막 창으로
정상상태를 판정하고, `fit_drag`는 `steady`만 본다. 버리지 말 것.

## 판정 규칙

수조 오염(블로키지 +10.1%, 재순환)은 항력을 **과대측정**하므로 v_max를
낮추기만 한다. 따라서:

- 측정 `v_max ≥ 1.15` → **가설 B 확정** (오염을 감안하면 a fortiori)
- `v_max ≤ 0.95` → 가설 A
- 사이 → 애매. 저수준(0.10/0.20)만으로 재적합해 볼 것

조파저항은 없다 — 잠수체는 `exp(-2gh/U²)`로 감쇠하고 h=0.575 m에서 u=1.12 m/s일
때 1.3e-4다.

## 보고 항목

1. `실측 surge 항력` 블록 전체 (`Xu`, `Xuu`, `R²`, `v_max`, `A`, 추종률)
2. 수준별 `정상상태 u` / `전달 추력` 원자료
3. 버린 수준과 사유
4. **타행 교차검증 Xuu** — 추력 테이블과 무관한 독립 추정
5. 재순환 점검 결과 (최저 수준 반복 측정 두 값)
6. **테더 전개 길이, 실제 수심, 배터리 전압, 수조 치수, alignment_id/epoch**
   — 이거 없으면 숫자를 해석할 수 없다
7. `$BROV_DATA_DIR/drag_test.json`

## 알아둘 한계

- **속도 출처**: `AlignedOdometry`의 twist(base_link FLU) = ArduSub EKF3 출력.
  A50 DVL body velocity 직결 경로는 미구현이고 그쪽이 논문이 쓰는 경로다.
- **전진비**: 주행 구간에서 재는 것은 전진비 손실을 포함한 **유효 항력**이다
  (정지추력 테이블로 나누므로). 시뮬레이션도 정지 테이블을 쓰므로 자기일관적이고
  A 결정에는 오히려 맞다. `v_max`와 `A`는 테이블의 균일 배율 오차에 불변이지만
  `Xu`/`Xuu` 절대값은 테이블 정확도에 의존한다. **타행 적합이 그 교차검증이다.**
- **타행에서 Xu는 식별되지 않는다.** 관측 속도 범위에서 u와 u²이 거의 공선이라
  σ=0.02에서 Xu 오차가 -44~-108%까지 간다. 그래서 정상상태 적합의 Xu를 고정하고
  **Xuu만** 푼다(그때 오차 -1.3~-3.7%). `xu_identifiable=False`로 표시된 Xu는
  보고에 쓰지 말 것.

## 하지 말 것

- **surge를 피드백으로 닫지 말 것** — 측정 자체가 무의미해진다
- `model_based_controller_node` / `policy_node`와 동시 실행 금지 —
  `obs_node._authority_gate`가 `/brov/thruster_pwm` 단일 발행자를 요구한다
- `diag_terminal_velocity.py`와 노드를 동시에 쓰지 말 것 — MAVLink 소유자가 둘이 된다
- PWM 발행을 25 Hz 밑으로 떨어뜨리지 말 것 (obs_node 워치독 0.25 s)
- PWM 포화 구간(level 0.87 초과)에서 측정하지 말 것 — yaw 권한을 잃는다
- 전/후진을 섞어 재지 말 것 (T200 역추력 −51.5 N < 정추력 +64.1 N)
- 수준 사이에 테더 전개 길이를 바꾸지 말 것
- 정렬 초기화를 기체가 움직이는 중에 하지 말 것 — 정지 샘플만 받는다
- **크레인/줄에 매단 채로 측정하지 말 것** — 진자가 되어 u→0이 되고,
  `fit_drag`의 `abs(u)>1e-3` 필터를 통과해 `Xu=8192` 같은 값이 조용히 보고된다
- 사용자 승인 없이 추력을 내지 말 것 (`send_pwm:=true arm:=true`가 그 승인이다)
