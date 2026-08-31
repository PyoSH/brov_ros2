# 실기 수조 세션 — 절차와 고려사항

> 이 문서는 **실기를 돌리는 Ubuntu 랩톱의 Claude 가 읽는다.** 명령은 그대로
> 복사해 쓸 수 있게 적었고, 검증된 것과 안 된 것을 구분해 표시했다.
> 작성 2026-08-31.

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
- **추진기 클램프 값 불일치.** `test_thruster_force_clamp_matches_inverse_envelope`
  가 계속 실패한다 -- 제거된 다항식 모델의 +-51.5/64.1 N 을 단언하는데 T200
  테이블은 -36.7/+47.2 N 을 준다. 실기는 `t200_table` 을 쓰므로 어느 쪽이 맞는지
  확인이 필요하다. 이번 세션의 목적은 아니지만 기록해 둘 것.
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

#### 검사 스크립트

전용 도구는 아직 없다. 아래를 그대로 쓴다 (`pymavlink` 필요).

```python
#!/usr/bin/env python3
"""깊이 게이트 — BARO_PRIMARY 확인 + 세 SCALED_PRESSURE 반응 측정."""
import time
from pymavlink import mavutil

CONN = "udpout:192.168.2.2:14550"     # split stack 과 동시에 쓰면 포트가 겹칠 수
                                       # 있다. 필요하면 BlueOS 의 다른 endpoint 사용.
m = mavutil.mavlink_connection(CONN)
m.wait_heartbeat(timeout=20)
print("heartbeat OK")

# --- 파라미터 ---
want = ["BARO_PRIMARY", "BARO_SPEC_GRAV", "BARO_PROBE_EXT",
        "EK3_SRC1_POSZ", "EK3_SRC1_VELZ"]
got = {}
for n in want:
    m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
t0 = time.time()
while time.time() - t0 < 15 and len(got) < len(want):
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg and msg.param_id.strip("\x00") in want:
        got[msg.param_id.strip("\x00")] = msg.param_value
for n in want:
    print(f"  {n:16s} = {got.get(n, '수신 실패')}")

# --- 세 SCALED_PRESSURE 를 한 지점에서 읽는다 ---
def read_pressures(seconds=6.0):
    out = {}
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = m.recv_match(
            type=["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"],
            blocking=True, timeout=2)
        if msg:
            out[msg.get_type()] = msg.press_abs
    return out

input("\n기체를 얕은 기준 깊이에 두고 Enter: ")
shallow = read_pressures()
print("  ", shallow)
input("정확히 1.00 m 더 깊게 내리고 Enter: ")
deep = read_pressures()
print("  ", deep)

print("\n instance  메시지               dP [hPa]   기울기 [hPa/m]   판정")
for i, name in enumerate(["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"]):
    if name not in shallow or name not in deep:
        print(f"  {i:8d}  {name:20s} 수신 없음")
        continue
    dp = deep[name] - shallow[name]
    verdict = "WATER (깊이센서)" if abs(dp) > 50 else "dry/internal"
    print(f"  {i:8d}  {name:20s} {dp:8.2f}   {dp:12.2f}   {verdict}")
print("\nArduSub 변환 상수 98.0 hPa/m (9800 Pa/m / SPEC_GRAV=1.0)")
```

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

### 2단계 — DVL 기록 붙이기 (선택, 권장)

**EKF 융합을 밀어낼 위험이 있다.** A50 의 TCP 서버가 단일 클라이언트만 받는다면
우리 기록기가 BlueOS 의 DVL extension 을 밀어낼 수 있고, 그러면 EKF 가 IMU
dead reckoning 으로 **조용히** 떨어진다. 화면상 아무 일도 안 일어난 것처럼
보이므로 반드시 확인한다.

```bash
# 붙이기 전
ros2 topic echo /brov/state --field ekf_velocity_variance

# 붙이기
ros2 run brov_control dvl_record_node --ros-args -p dvl_host:=192.168.2.95

# 붙인 후 (같은 값이어야 한다)
ros2 topic echo /brov/state --field ekf_velocity_variance
ros2 topic echo /brov/dvl/sample --once      # connected: true, valid: true
```

**분산이 눈에 띄게 오르면 융합이 끊긴 것이다. 즉시 DVL 노드를 끄고**, 이번
세션에서는 DVL 기록을 포기한다. 제어 경로에는 영향이 없다(이 노드는 어떤 제어
토픽도 발행하지 않는다).

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

#### 중단 기준 — 하나라도 걸리면 즉시 `stop_control`

- 벽까지 0.5 m 이내
- 깊이가 안전 영역(0.20~0.90 m)을 벗어남
- 진동이 눈에 띄게 커짐
- `/brov/state` 의 `valid: false` 가 지속

`stop_control` 은 **armed 를 유지한 채** 제어만 멈추고 중립을 보낸다.
`start_control` 로 바로 재개된다. 완전히 내리려면 `disarm_control`.

**watchdog 도 있다.** 명령이 0.25 s 끊기면 base 가 자동으로 중립 정지한다.

---

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
<기록경로>/pool_wa0017_run1/          # bag
1단계 스크립트 출력                     # BARO_PRIMARY, SPEC_GRAV, 기울기
4단계 diag_loop_delay 출력 (heave, surge)
관측 메모                              # 진동이 눈에 보였는지, 어느 축인지
```

이 넷이 다음 학습(지연 주입)의 근거가 된다.

---

## 안 하는 것

- **DVL 되먹임 승격** — 축 변환 미확정, 위상 지연 증가
- **`depth_source:=pressure` 를 게이트 없이 사용** — 근거 없이 기본 동작 변경
- **5 m 사각 / Fig.4 (b)(c)** — 수조에 안 들어감
- **dead time 반영 정책 시험** — 아직 존재하지 않음. 이번 측정 후에 만든다
