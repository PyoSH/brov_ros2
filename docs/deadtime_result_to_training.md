# 실기 dead time·진동 측정 결과 → 학습 PC 이관 (2026-09-02)

수조 세션이 끝났다. 이 문서는 **무엇을 하려 했고, 무엇이 나왔고, 그것이 무슨 뜻이며,
학습 쪽에서 무엇을 고쳐야 하는지**를 넘긴다. 절차·명령·운용 주의는
`REAL_ROBOT_SESSION.md` §3-C/3-D 에 있고, 사후 분석 스크립트는 `runtime/analysis/`,
bag 은 `runtime/bags/` 에 있다.

## 결론 한 줄

**지연 80 ms 는 통신·처리 경로 자체이고, 그 아래에서 정책은 포화(relay) 상태로 돌며
2 Hz limit cycle 을 만든다. 이득을 낮춰도 진폭만 줄고 진동은 남는다. plant 도 적분기도
deadband 도 아니다. 학습에 지연을 넣어야 한다.**

---

## 0. 세션 목적과 실험 구성

| 실험 | 의도 | 도구 |
|---|---|---|
| 깊이 게이트 | EKF 수직 위치를 믿을 수 있는가, 어느 baro 가 물속 센서인가 | `diag_depth_gate` (sweep) |
| A2-heave | dead time 측정 (첫 시도) | `deadtime_test.launch.py axis:=heave` |
| A2-yaw | ESC 역전·EKF 속도 지연을 뺀 **순수 전송 지연** | `deadtime_test.launch.py axis:=yaw bias:=1.0` |
| A1 | 진동이 "지연+세기" 기전인지 판별 — 정책 이득 1.0 vs 0.5 | `pool_demo_a.launch.py wrench_gain:=` |

모두 분리 스택(`base_node`/`guidance_node`/`observation_node`/`policy_wrench_node`,
절단면 = wrench) 위에서, `frame:=start_heading`(마커 없음), 정책 번들
`sim2swim_fixplant_wa0017_mk2_s42_i299`, `depth_source:=mavlink_ekf`, DVL 기록 off.

원래 계획했던 마커(ArUco) 절대 프레임은 검출률 7.9 Hz 까지 확보했으나 정렬 시도
중 시간 관계로 접었다. 코드는 남아 있다(`guidance_node waypoint_frame=pool`,
`pool_demo_a frame:=marker`) — 측량 대조는 아직 안 했다.

---

## 1. 깊이 게이트 — 통과

**의도.** 2026-08-29 SITL 에서 `LOCAL_POSITION_NED.z` 가 초기값에 얼어붙어 기체가
1.77 m 떠오른 전례가 있다. 수조 깊이 여유 0.7 m 라 같은 증상이면 수초 만에 닿는다.

**방법.** 기체를 위아래로 흔들며(56 cm 폭) 세 baro 압력과 `depth_ekf` 를 40 s 기록,
반응한 baro 를 ArduSub 변환식(9800 Pa/m × SPEC_GRAV)으로 미터로 바꿔 EKF 깊이를
회귀. 거리를 잴 필요가 없다 — 압력이 자다.

**결과.**

| instance | 압력 폭 | 판정 |
|---|---|---|
| 0 (`SCALED_PRESSURE`) | 92.9 Pa (0.9 cm) | 내부 |
| 1 (`SCALED_PRESSURE2`) | 5510 Pa (56.2 cm) | **물속 센서** |
| 2 | 미수신 | 없음 |

`depth_ekf = 1.005 · depth_baro + 0.000`, R² 0.9935, 잔차 RMS 1.4 cm.

**분석.** EKF 가 압력을 따라온다 — SITL 증상은 실기에 없다. `mavlink_ekf` 유지.
기울기 1.005 = 1/SPEC_GRAV 이므로 FC 의 SPEC_GRAV ≈ 1.0(담수)과 일치. EKF 가 같은
baro 를 융합하므로 완전 독립 검증은 아니다 — "EKF 가 자기 소스를 제대로 적분하는가"
를 본 것이고, SITL 실패가 정확히 그 지점이었다.

---

## 2. A2 — dead time

### 2-1. heave 사각파 (첫 측정)

1 Hz, ±20 N, 30 s. **τ = 80 ms, r = 0.661** (여기 구간 25 s 만). 명령·상태 25.0 Hz,
p90 40.2/42.0 ms — 링크 결손 없음.

문제 둘. (a) ±20 N 은 수직 추진기 4개를 매 에지마다 PWM +0.25 → −0.26 으로 **0 을
관통**시킨다 → BLHeli_S 는 "startup power … limit the power applied during direction
reversal" 이라 역전 지연이 섞인다. (b) 1 Hz 협대역이라 교차상관 봉우리 반폭이
~250 ms — 0~120 ms 에서 r 이 0.60~0.66 으로 평평한 것은 분해능 한계이지 jitter 증거가
아니다.

### 2-2. yaw 사각파 (역전 없음, 자이로 직접) — **확정값**

bias 1.0 ± 0.5 N·m → 수평 추진기 4개가 PWM 0.10~0.16 에 머문다(0 을 안 넘음, deadband
±0.075 밖). 기체는 제자리에서 ~47°/s 로 돈다. 각속도는 자이로에서 직접 오므로 EKF
속도 융합 지연도 빠진다. 40 s, 바닥에서 0.4 m 띄운 뒤 유지(여기 노드의 느린 깊이
루프, 무게 불필요).

```
피크 lag = 80.0 ms,  r = +0.809
lag 프로파일: 0 ms 0.742 / 60 ms 0.801 / 80 ms 0.809 / 100 ms 0.781 / 140 ms 0.571
깊이: +0.85 → +0.47 m (8 s), 이후 30 s ±2 cm
```

**분석.** 역전을 빼고 자이로로 재도 **80 ms 그대로** → 통신·처리 경로 자체다
(랩톱 → MAVLink → BlueOS 라우터 → ArduSub → ESC → 모터 → 기체 → telemetry → 랩톱).
heave 의 낮은 r(0.66) 은 jitter 가 아니라 역전 시간의 흔들림과 EKF 속도 융합
잡음이었다 — 빼자 0.81 로 올랐다. **학습에는 고정 80 ms** 를 넣을 근거가 선다.
jitter 상한은 chirp(0.5→8 Hz, 반폭 ~30 ms)로 한 번 더 재면 좁혀진다 — 안 했다.

### 2-3. 위상 예산 (heave, m_eff 28.1 kg, 제어주기 40 ms)

| τ | 60 ms | **80 ms** | 100 ms |
|---|---|---|---|
| −180° 교차 | 2.99 Hz | **2.39 Hz** | 1.99 Hz |
| K_p 문턱 (정규화) | 4.40 | **3.52** | 2.93 |

IsaacLab 은 τ=0. SITL 직결 60 ms 에서 정책 K_p 4.5 는 문턱 4.40 과 경계였고, 실기
80 ms 에서는 **28 % 초과**. r ≥ 0.65 인 60~100 ms 어디에 진짜 τ 가 있어도 초과다.

---

## 3. A1 — 이득 ½ 대조

**의도.** 되먹임이 떠는 조건은 "보정이 늦고 **동시에** 세다". 지연은 못 줄이니
세기를 절반으로 해서(`wrench_gain:=0.5`) 떨림이 (a) 사라지면 지연+세기, (b) 남되
느려지면 deadband, (c) 주파수 그대로 진폭만 절반이면 정책 chatter.

**조건.** start_heading, leg 1.0 m(사용자 조정), rise 0.5 m, 0.25 m/s, 각 60 s,
같은 자리. bag: `a1_gain10-20260902-154555`, `a1_gain05-20260902-155131`.

### 3-1. 1.8~2.6 Hz 대역 절대 RMS (`runtime/analysis/a1_band.py`, `a1_legs.py`)

| 축 | 명령 (1.0 → 0.5) | 응답 (1.0 → 0.5) | 주파수 |
|---|---|---|---|
| surge | 7.4 → 5.9 N (×0.80) | 0.31 → 0.30 m/s² (×0.96) | 2.2~2.3 Hz |
| **sway** | **21.1 → 6.3 N** (×0.30) | 0.90 → 0.49 m/s² (×0.54) | 1.95~2.1 |
| heave | 6.9 → 3.7 N (×0.54) | 0.36 → 0.27 (×0.74) | 2.1~2.2 |
| **roll** | **10.4 → 4.6 N·m** (×0.44, 권한 40 %) | 6.9 → 5.0 rad/s² (×0.72) | ~2 |
| pitch | 5.7 → 2.2 N·m (×0.39) | 5.1 → 3.1 (×0.61) | ~2 |
| yaw | 5.7 → 2.3 N·m (×0.40) | 3.5 → 2.2 (×0.63) | 1.95 |

`diag_loop_delay` 의 전력비는 상대값이라 이득을 바꾸면 분모도 바뀐다 — 판정은 위
절대값으로 했다. (그 문제로 `실측 지배 주파수` 줄이 빠지던 것을 고쳐, 이제 1~5 Hz
진동대 기준으로 낸다: sway 예측 1.99 Hz vs 실측 1.95 Hz, 일치.)

### 3-2. 포화·클램프·적분기·속도 (`a1_saturation.py`, `a1_legs.py`)

| | gain 1.0 | gain 0.5 |
|---|---|---|
| 행동 \|a\| ≥ 0.99 비율 — surge / pitch / yaw / sway | **73 %** / 43 % / 26 % / 8 % | 62 % / 12 % / 22 % / 26 % |
| surge 평균 행동 | +0.80 (68 N) | +0.62 |
| 추진기 클램프(−49/+66 N) — T1 / T2 / T6 | **51 %** / 13 % / 13 % | 0 % |
| \|z_v\|, \|z_q\| ≥ 4.9 | 0 % | 0 % |
| 실제 surge 속도 (목표 0.25) | **0.14 m/s** | **0.02 m/s** |
| 다리당 시간 (직진이면 4 s) | 11.3 s | 26.4 s |
| 각속도 RMS (r/p/y) | 0.70 / 0.65 / 0.78 rad/s | 0.55 / 0.51 / 0.87 |
| v_e 평균 (v − v_d) | −0.07 m/s | −0.165 |

### 3-3. 개루프 대조

A2-yaw 와 A2-heave bag 의 응답에는 명령이 실은 것 이상의 2 Hz 가 **없다**
(응답 2 Hz 대역 0.068 / 0.070 vs 1 Hz 여기 대역 0.53 / 0.45). 기체·추진기·테더의 자체
2 Hz 공진은 없다. 2 Hz 는 폐루프가 만든다.

### 3-4. 판정

이득 ½ 에서 진동이 **사라지지 않고 진폭만 ×0.4~0.8, 주파수 그대로.** 선형 지연+이득
루프라면 임계의 0.64 배에서 꺼져야 한다. 안 꺼진 이유는 정책이 대부분의 시간
**포화(relay)** 상태이기 때문이다: 포화 제어기 + 80 ms 지연은 −180° 교차 주파수에
**진폭이 포화 수준(=이득)에 비례하는 limit cycle** 을 만든다 — 관측과 정확히 일치.

- 예측 주파수 2.39 Hz(translational) vs 실측 2.0~2.3 Hz — 기전이 맞다.
- 배제: 기체 공진(3-3), 적분기 windup(0 %), deadband(수직 추진기 동작점 6.3 N/추진기,
  deadband 가장자리 ~0.45 N), 관측 계단(속도 정보 갱신 21~24 Hz).
- 최대 축은 **sway–roll**. 측면 추력이 CoM 아래 레버암으로 roll 을 만들고 둘이 80 ms
  를 사이에 두고 서로 되먹인다. surge 는 클램프에 붙어 있으나 진동이 권한을 잡아먹어
  0.14 m/s 밖에 못 낸다.
- leg 1.0 m 가 상황을 악화시켰다(lookahead 1.0 과 같아 다리가 안착 못 함). 그래도
  진동 기전 판정에는 영향이 없다 — 주파수·이득 의존성이 그 증거다.

---

## 4. 관측 경로에 대해 알게 된 것

| 항목 | 값 | 뜻 |
|---|---|---|
| `attitude_age_s` 분포 (25 Hz 틱 기준) | 75.1 % <10 ms, 4.2 % 30~40, **15.1 % 40~50 ms** | 6 번에 1 번은 한 틱 묵은 관측. 학습에 관측 지연 jitter 로 넣을 값 |
| 속도 정보 갱신률 | 21.5 Hz(여기 중) / 24.5 Hz(정지) | DVL 5 Hz 계단 **아님** — EKF 가 IMU 로 전파 |
| EKF flags (정상 시) | 367 = ATT+VEL_H+VEL_V+POS_H_REL+POS_V_ABS+POS_V_AGL+PRED | DVL 속도·위치 융합됨 |
| `ekf_velocity_variance` | 항상 0.0 | **"융합 없음" 의 뜻이 아니다** (flags 가 VEL 정상). 이 필드로 DVL 상태를 읽지 말 것 |
| 압력 스트림 | 10 Hz (요청 후) | `SET_MESSAGE_INTERVAL` 을 넣기 전에는 요청 자체가 없었다 (고침) |

---

## 5. 오프라인 확인 (학습 PC 에서 이어갈 것)

- **정책 소신호 이득.** 정지 관측점(9060 표본, 단일 선형 영역)에서 wa0017 surge −1.94 /
  heave −1.22, v5 −1.64 / −1.67 (정규화). 문서의 순항 실측 4.5 와 다르다 → **이득이
  동작점에 강하게 의존**한다. 진짜 값은 A1 bag 의 `/brov/observation` 과
  `/brov/cmd/wrench` 로 회귀해야 한다(Δwrench vs Δv_e). 안 했다.
- **v5 는 순수 w_a=0.3 대조군이 아니다** (`reward_profile: deploy_v2`). "w_a 를 낮춰서
  이득이 올랐다" 는 가설은 미검증. 검증하려면 `paper_ref_v1` + w_a=0.3 체크포인트의
  Jacobian 을 같은 관측점에서.
- **deadband 추력**: T200 역변환 0.25 N → PWM 0.000, 0.5 N → 0.091 → 추진기당 ~0.45 N
  아래는 출력 0. 순항 0.25 m/s 는 추진기당 4.3 N(PWM 0.22)으로 밖.
- **추진기 한계 불일치**: `base_node` 실행 로그의 T200 테이블 한계는 −49.4 / +65.9 N,
  `REAL_ROBOT_SESSION.md` 는 −36.7 / +47.2 N 이라 적고 있다. 어느 쪽이 배포 코드의
  실제 값인지 확인할 것 (`test_thruster_force_clamp_matches_inverse_envelope` 가
  계속 실패하는 것과 같은 문제).

---

## 6. 학습 쪽에서 고칠 것

논문의 철학 — 식별하지 않을 것은 랜덤화한다 — 를 **시간 축**에 적용한다. Sim2Swim
본문은 지연·액추에이터·필터를 다루지 않고 DR 은 질량·부피·CoB 뿐이다. 전작(Learning
to Swim)은 "we do not model motor action delays which can lead to instabilities in
deployment" 라고 인정한다.

| 넣을 것 | 값 | 근거 |
|---|---|---|
| 행동 지연 | **2 스텝(80 ms @25 Hz)** 고정, 에피소드마다 1~3 스텝 랜덤 | §2-2 |
| 관측 신선도 | 15 % 확률로 1 스텝 묵은 관측 | §4 |
| 행동 변화율 벌점 (Δa) 또는 Lipschitz 정규화 | 추가 | §3 포화·relay |
| 액추에이터 | 3차(von Benzon) 유지 + 실제 T200 클램프 | T1 51 % 클램프 |
| 미션 | 직선 다리 ≥ 2.0 m 로 평가 | leg 1.0 의 왜곡 |

**수용 기준** (배포 도구로 잰다):
1. 순항 관측점에서 Jacobian **K_p < 3.5** (80 ms 문턱).
2. Gazebo SITL 폐루프에서 `diag_loop_delay` 의 1~5 Hz 진동대 응답 RMS 가 오늘
   실기(heave 0.36 m/s²)의 1/5 이하, 행동 포화 비율 < 10 %.
   > **정정 (2026-09-02, 학습 PC):** 원판의 "base_node 에 80 ms 인공 지연 주입"
   > 은 **이중 계상**이다 — SITL 경로 자체가 실측 60 ms(직결)/80 ms(mavproxy)
   > 를 이미 갖고, ESC 가 없는 대신 transport 가 길어 총합이 실기(80 ms)와
   > 같은 등급이다. baseline 이 SITL 에서 2~3 Hz 로 떤 것이 그 증거다.
   > **인공 주입 없이 SITL 을 있는 그대로** 쓰고, mavproxy 경유(80)/직결(60)
   > 을 두 시험점으로 삼는다. `thruster_model` 도 `t200_table` 이 아니라
   > **`gazebo_linear` 유지** — SITL 플랜트가 선형 플러그인이라 T200 역변환을
   > 쓰면 왕복이 항등이 아니다(실측 1.4~2.1배 초과). t200_table 은 실기 전용.
3. 실기 A1 재현: gain 1.0 에서 sway 2 Hz 대역 < 5 N, 전진 속도 ≥ 0.2 m/s.

배포 측 임시 완화(행동 slew 제한 `max_pwm_delta_per_s`, 저역필터)는 relay 거동을 깨긴
하지만 위상을 더 깎는다 — SITL 에서 먼저. 이득만 낮추는 것은 해법이 아니다(A1).

---

## 6-보강. 지연 주입의 문헌 근거 (2026-09-02, 학습 PC 조사)

§6 처방("행동 지연 2스텝 고정 + 1~3 스텝 랜덤")이 임의 결정이 아님을 확인했다.
근거를 이론 → 실증 → 우리 계약에의 함의 순으로 둔다.

### 이론 — 지연이 있으면 그 관측공간은 더 이상 Markov 가 아니다

[Katsikopoulos & Engelbrecht 2003](https://www.researchgate.net/publication/321001962)
(constant-delay MDP 의 표준 결과): 상수 지연 Δ 가 있는 MDP 는 **관측을 "마지막
상태 + 이후 보낸 행동 Δ개" 로 증강**하면 지연 없는 MDP 와 등가이고, 증강 공간의
최적 정책이 원문제의 최적이다. 관측 지연과 행동 지연은 에이전트 관점에서
등가다. [Walsh et al. 2008](https://arxiv.org/pdf/2010.02966) (재정식화) 도 같다.

**함의:** 우리 16-D 계약 `[q_e, v_e_b, ω_b, z_v, z_q]` 에는 **행동 이력이 없다.**
따라서 지연을 주입해도 이론적 최적까지는 못 간다 — 정책이 할 수 있는 최선은
"늦게 도착할 것을 전제로 이득을 낮추는" 강건화다. 그것이 정확히 우리가 원하는
것이므로(§3: 이득이 문턱 3.52 를 28% 초과) 계약을 유지한 채 주입하는 것이 맞다.
memoryless 정책 + 지연 학습의 전례:
[Schuitema et al. 2010](https://www.researchgate.net/publication/224199729)
"Control delay in RL for real-time dynamic systems: **a memoryless approach**".

### 실증 — sim2real 로봇에서 지연 모델링/랜덤화는 표준 관행

- [Tan et al., RSS 2018](https://arxiv.org/abs/1804.10332) (Minitaur 4족):
  sim2real 성공 요인 셋 중 하나로 **latency 를 시뮬레이션에 모델링**
  (나머지: 액추에이터 모델, 시스템 식별). 우리와 같은 "시뮬에는 없던 실기
  지연" 문제의 원형.
- [Imai et al. 2021, Multi-Modal Delay Randomization](https://arxiv.org/abs/2109.14549)
  (Unitree A1, memoryless MLP + 프레임 스태킹): 감각 지연을 **범위로 랜덤화**
  ([0, 40 ms] proprioception)해 학습 → 실기 4개 지형에서 이동거리 +35~94%.
  결정적 발견: **고정 지연으로 학습한 정책은 배포 지연 분포가 조금만 달라도
  급락한다** ("the performance of the Fixed-Delay agent drops drastically").
  → §6 의 "고정 2스텝 + 1~3 랜덤" 에서 **랜덤 폭이 장식이 아니라 필수**라는
  직접 근거. 관측 신선도 15% jitter 주입도 같은 계열이다.
- Sandha et al. 2021 (§7 기존 인용): 60~100 ms 에서 NN 정책 성능 급락, 지연
  랜덤화 + 행동 이력 관측(DMDP) 처방. 우리 τ=80 ms 가 정확히 그 구간이다.

### 우리 설계에의 적용

| §6 처방 | 문헌 대응 |
|---|---|
| 행동 지연 2스텝(80 ms) 기본 | Tan 2018 (실측 지연을 시뮬에 넣는다) |
| 에피소드마다 1~3 스텝 랜덤 | Imai 2021 (고정 지연 학습은 취약 — 범위 랜덤화) |
| 관측 신선도 15% 1스텝 지연 | Imai 2021 (다중 신호 지연 랜덤화) |
| 계약(16-D) 유지, 이력 미추가 | Schuitema 2010 (memoryless + 지연 학습 전례) |
| **fallback**: K_p < 3.5 미달 시 | Katsikopoulos 2003 — 행동 이력 증강이 이론 정합 해법. 단 **배포 계약 전면 변경**이므로 최후 수단 |

**논리 요약:** 지연 주입은 (1) 이론적으로는 차선이지만(이력 없는 계약), (2) 우리
목표가 최적성이 아니라 **이득 강하**이고, (3) 같은 구성(memoryless + 지연
랜덤화)의 실기 성공 전례가 4족·매니퓰레이션에 걸쳐 반복돼 있으므로, 계약을
지키는 선에서 근거 있는 첫 수다. 실패 판정 기준(§6 수용 기준)이 있으므로
안 되면 이력 증강으로 넘어갈 지점도 명확하다.

### 비판적 평가 (2026-09-02 추가) — 주입 전에 반영할 것 4건

**① 이중 계상 보정 — §6 의 "2스텝 고정"은 무보정 시 과잉이다.** 실측 80 ms 는
명령→자이로 전체라 실기 액추에이터 응답을 포함하는데, 학습 환경에는 이미
von Benzon 3차 모델이 있다. 2 Hz 에서 수치로 `H(j12.57)` = 이득 0.90,
위상 −15° ≈ **등가지연 21 ms**. 80 ms 를 통째로 얹으면 시뮬 총 지연 ~101 ms
로 실기보다 ~26% 과잉(보수적 방향이지만 대역 낭비). 올바른 주입값:
`τ_주입 = τ_total − τ_actuator,실기` — 후자가 `LATENCY_DECOMPOSITION_PLAN.md`
의 M4 다. **M4 전 임시값: 80 − 21 ≈ 60 ms (물리 스텝 6, 정책 1.5 스텝).**

**② 보상 압력은 있다 — 정량:** 지연 주입 환경에서 고이득 정책이 만드는
limit cycle 은 실측 진동 수치 기준 step 당 ~0.085 를 깎는다
(ω 항 0.049 — `|ω|`~2.1 rad/s 가 w_ω=0.05 항을 통째로 소멸시킴 — + v 항 0.030
+ a 항 0.006) ≈ **return 의 8~9%.** PPO 가 반응하기에 충분하나 진동 발현을
경유하는 간접 압력이므로 Δa 벌점은 여전히 싼 보험.

**③ MMDR 전례의 한계 — 위 표의 인용 범위를 좁힌다.** MMDR 정책의 관측에는
**prior actions 가 스택**돼 있다(부분적 이력 증강). "고정 지연 학습은 취약 →
랜덤화 필수" 부분만 우리 근거로 유효하고, "memoryless 로도 된다" 의 근거는
Schuitema 2010 하나뿐이며 저차원 벤치마크다. **memoryless 성공은 전례가 아니라
검증할 가설** — 수용 기준 미달 시 이력 증강 fallback 을 지킬 것.

**④ 수용 기준 1(K_p<3.5)의 지위:** describing function 논거로는 정당하다 —
포화 DF 는 원점 기울기 k 가 최대이므로 `k·|G(jω₁₈₀)|<1` 이면 limit cycle 의
해가 없다. 단 Jacobian 은 동작점 의존이 실측됐고(정지 −1.9 vs 순항 −4.5)
순항 Jacobian 이 원점 기울기의 좋은 대리라는 보장이 없다 → 기준 1 은
스크리닝, **진짜 게이트는 기준 2(SITL 폐루프).** 참고로 문턱은 τ 의 함수라
(τ=50 ms 에서 5.01, 40 ms 에서 5.86) **지연 축소가 먼저 성공하면 현재 정책
(4.5)이 재학습 없이 안정 영역에 들 수 있다** — 재학습과 지연 축소는 택일이
아니라 곱이다.

## 7. 문헌 (질문에 쓸 것)

- Sim2Swim (arXiv:2512.08656): 지연 없음, 배포 필터 없음, DR 질량·부피·CoB, w_a=0.3,
  수조에서 "속도 급변 시 오프셋", "heave 응답 느림".
- Learning to Swim (arXiv:2410.00120): 지연 미모델 인정, slerp 로 완화, 20 Hz.
- T200+Basic ESC (BlueRobotics 포럼): 반응 ~110 ms(cold), 100 Hz PWM 에서 ~130 ms,
  400 Hz 에서 ~25 ms. ArduSub 메인 출력 200 Hz (`RC_SPEED`).
- BLHeli_S Rev16.x: startup power 가 역전 중 전력을 제한.
- Sim2Real with stochastic delays (Sandha 21), 지연 랜덤화·행동 이력 관측(DMDP),
  "60~100 ms 에서 NN 정책 성능 급락".
- Tan et al. RSS 2018 (arXiv:1804.10332): Minitaur sim2real — latency 모델링이
  성공 요인. Imai et al. 2021 (arXiv:2109.14549): 지연 **랜덤화**, 고정 지연
  학습의 취약성 실증. Katsikopoulos & Engelbrecht 2003 / Walsh 2008: 지연 MDP
  이론(행동 이력 증강 등가). Schuitema 2010: memoryless + 지연 학습. (§6-보강)

1저자에게 추가할 질문: 실기 τ 와 진동 유무; 행동 포화 비율; sway–roll 커플링 처리;
w_a=0.3 이 이득을 낮춰 포화를 피한 것인지 (5시드 결과와 함께); Fig.4 통과 판정 기준.

---

## 8. 사고·환경 기록 (재발 방지)

| 사건 | 원인 | 조치 |
|---|---|---|
| 네이티브 rclpy import 실패 | `/usr/bin/python3.10` 의 `cap_sys_nice` → secure-exec 로 `LD_LIBRARY_PATH` 무시 | `setcap -r`; `env_native.sh` 가 검사 |
| 카메라 노드 즉사 | `h264parse`/`avdec_h264` 없음 | `gstreamer1.0-plugins-bad libav` |
| colcon build 실패 | torch 가 올린 setuptools ↔ 구 packaging | `packaging>=24` |
| bag 기록기 조용히 사망 | `ros2 bag record -o` 기존 디렉터리 거절 | launch 가 시각 접미사 |
| `metadata.yaml` 없음 | launch Ctrl+C 로 기록기 강제 종료 | `runtime/reindex_bags.sh` |
| **`LOCAL_POSITION_NED` 끊김** | `dvl_record_node` 가 15:21:35 A50 접속 → 15:22:53 BlueOS DVL extension 멈춤 → 15:22:58 EKF CONST_POS_MODE. 노드를 내려도 회복 안 됨 | 로봇 재부팅(2 분)으로 복구. `pool_demo_a` `dvl` 기본 **false**. `check_ekf.sh`/`restart_dvl.sh` |
| 스트림 요청 누락 | `SCALED_PRESSURE` 를 요청하지 않았음 | `mavlink_interface` 가 셋 다 요청; `/brov/request_streams` 서비스 |

---

## 9. 인벤토리

**bag** (`runtime/bags/`, 분석에 쓴 것만; 나머지는 중단된 시도)

| bag | 내용 |
|---|---|
| `dry_run` | 무추력, 정지 관측 6 분 (Jacobian replay, 관측 신선도) |
| `deadtime_heave` | A2-heave, 1 Hz ±20 N 30 s + 꼬리 |
| `a2_yaw-20260902-150426` | **A2-yaw 확정** |
| `a1_gain10-20260902-154555` | **A1 gain 1.0** |
| `a1_gain05-20260902-155131` | **A1 gain 0.5** |
| `marker_check-*` | ArUco 검출 시험 (정렬 미완) |

**코드** (신규/변경, 전부 시험 있음 — 527 tests, 사전 실패 3건만)

- `brov_bringup/launch/pool_demo_a.launch.py` — 실기 왕복, `frame:=marker|start_heading`,
  `wrench_gain`, `rise_m`, `leg_m`, bag 기본 on, `dvl` 기본 off
- `brov_bringup/launch/deadtime_test.launch.py` + `brov_control/diag_excite_node.py` —
  개루프 여기(square/chirp, 6축), 느린 깊이 유지(`rise_m`)
- `brov_base/diag_loop_delay.py` — 각축, `--seconds`, `--open-loop`, 진동대 지배 주파수
- `brov_base/diag_depth_gate.py` — sweep 방식 깊이 게이트
- `brov_base/base_node.py` — odometry session 발행(마커 정렬 입력), 원시 센서 토픽
  (`/brov/sensor/ahrs|depth_ekf|pressure0..2`), `/brov/request_streams`, 수신 통계 로그
- `brov_base/guidance_node.py` — `waypoint_frame=pool`
- `brov_control/policy_wrench_node.py` — `wrench_gain` (0, 1]
- `runtime/*.sh` — `a1_gain`, `a2_yaw`, `lifecycle`, `stop`, `latest_bag`, `reindex_bags`,
  `check_ekf`, `restart_dvl`; `runtime/analysis/` — A1 사후 분석 3종

**미해결**

- chirp 로 τ 정밀화·jitter 상한 — 안 함. surge A2 — 안 함.
- 마커 프레임 측량 대조 — 안 함 (`aruco.yaml` 의 수조 가정 미검증).
- 정책 순항점 Jacobian 을 A1 bag 으로 — 안 함 (§5).
- 사전 실패 시험 3건(`test_thruster_force_clamp_matches_inverse_envelope`,
  `test_sim2swim_contract` 2건) — 이번 변경과 무관, 그대로.
