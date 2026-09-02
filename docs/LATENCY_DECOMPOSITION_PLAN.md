# 실기 dead time 80 ms 분해 계획 (2026-09-02 작성)

2026-09-02 수조 세션이 **τ = 80 ms (A2-yaw, r=0.809)** 를 확정했다
(`deadtime_result_to_training.md` §2). 이 문서는 그 80 ms 를 **구간별로 쪼개는
실기 측정 계획**이다. 쪼개야 하는 이유는 둘이다.

1. **대응 수단이 구간마다 다르다.** 통신이 지배적이면 온보드 이전·라우터 교체가
   듣고, 액추에이터가 지배적이면 `RC_SPEED` 상향이 듣는다. 어느 쪽인지 모르고
   손대면 효과 없는 개조를 하게 된다.
2. **학습에 주입할 τ 의 근거가 정밀해진다.** 줄일 수 있는 몫을 줄인 뒤의 잔여
   τ 가 학습 주입값이다. 지금 80 ms 는 상한이지 최종값이 아닐 수 있다.

## 0. 왜 "80 ms = 통신·처리 경로 자체" 는 아직 결론이 아닌가

결과 문서 §2-2 는 A2-yaw 가 ESC **역전** 지연과 EKF 속도 융합을 뺐으므로 80 ms
를 "통신·처리 경로 자체"로 귀속했다. 그러나 A2-yaw 도 다음은 **빼지 못한다**:

- ESC 소신호 응답 (BLHeli_S 가 PWM 변화를 모터 전력으로 옮기는 시간)
- 로터·유체 부가질량의 스핀 변화 (추력 변화가 실제로 나타나기까지)
- `RC_SPEED` 200 Hz 출력 갱신 양자화 (0~5 ms)
- telemetry 생성 주기 (`SET_MESSAGE_INTERVAL` 로 요청한 간격의 양자화)

참고할 수치가 이미 있다 — BlueRobotics 포럼 실측(결과 문서 §7)은 T200+Basic ESC
가 **100 Hz PWM 에서 ~130 ms, 400 Hz 에서 ~25 ms** 응답이라 한다(정지→계단 기준.
A2-yaw 는 bias 로 계속 돌고 있으므로 소신호 응답은 이보다 빠를 것). 즉
**액추에이터 몫이 수십 ms 일 가능성이 실제로 있고**, 그 경우 온보드 이전은 거의
효과가 없다. SITL 직결 60 ms 는 ESC 도 테더도 없는 값이라 분해의 참고가 되지
않는다.

## 1. 경로 모형

```
[랩톱] policy_wrench → base_node                     (DDS 로컬, ~0)
   │ MAVLink UDP
   ▼
[테더] Fathom-X (HomePlugAV 100 Mbps)                 ← M1/M2 가 잰다
   ▼
[로봇] BlueOS mavlink-router                          ← M2 에 포함
   ▼
[FC]  ArduSub 수신 → override 적용(스케줄러)          ← 잔차로 추정
      → RC_SPEED 200 Hz 서보 출력                      (0~5 ms 양자화)
   ▼
[동력] ESC(BLHeli_S) → 모터·프로펠러 → 토크            ← M4 가 잰다 (FC 시계)
   ▼
[센서] 자이로 → ATTITUDE 생성(요청 간격) → router → 테더 → 랩톱
```

`τ_total(80 ms) ≈ τ_link(왕복) + τ_FC스케줄+양자화 + τ_actuator`

## 2. 측정 4종

### M1 — 네트워크 RTT 하한 (5분, 코드 불필요)

```bash
ping -c 50 192.168.2.2          # 랩톱 → 로봇 RPi
```

BlueOS 웹 UI 의 **Local Network Test** 도 같은 것을 잰다
([BlueOS Advanced Usage](https://blueos.cloud/docs/latest/usage/advanced/)).
Fathom-X 구간의 순수 전송 하한. 수 ms 이하가 예상이며, 크면 그 자체가 발견이다.

### M2 — MAVLink 왕복 (랩톱 ↔ FC, 라우터 포함)

pymavlink 로 `TIMESYNC`(tc1=0) 를 보내면 ArduPilot 이 응답한다 — 왕복시간이
곧 랩톱↔FC 링크+라우터 RTT 다. TIMESYNC 무응답이면 대체:
`PARAM_REQUEST_READ` → `PARAM_VALUE` 왕복을 50회 재서 중앙값.

**τ_link = M2 중앙값.** M1 과의 차 = 라우터+FC 수신처리 몫.

### M3 — 명령 → 서보 출력 도착 (교차상관)

`SERVO_OUTPUT_RAW` 는 이미 FC 에 요청·수신되고 있으나(`mavlink_interface.py`
`SET_MESSAGE_INTERVAL`) **토픽·bag 으로 안 나간다.** 배선 후(§3) A2-yaw 를
재실행하고 명령 PWM ↔ 서보 도착 시각을 교차상관한다.

**M3 = τ_up + τ_FC + τ_down.** M2 와 대조하면 FC 스케줄링 몫이 나온다.

### M4 — 서보 → 자이로 (FC 시계, **확정 측정**)

`SERVO_OUTPUT_RAW.time_usec` 과 `ATTITUDE.time_boot_ms` 는 **둘 다 FC 시계**다.
두 시계열을 FC 시간축 위에서 교차상관하면 링크가 전혀 안 끼는
**순수 액추에이터+센서 지연**이 나온다. 시계 매핑이 필요 없다 — 같은 시계니까.

**M4 = τ_actuator (+ 자이로 샘플링).** 이 계획의 핵심 측정이다.

### 정합성 검사

`τ_total(80) ≈ M2 + M4 + (FC 스케줄+양자화 잔차)` 가 맞는지 본다. 잔차가
20 ms 를 넘으면 모형에 빠진 구간이 있다는 뜻이므로 그것부터 찾는다.

## 3. 준비 배선 — **구현 완료 (2026-09-02)**

| 작업 | 상태 |
|---|---|
| `/brov/sensor/servo_out` (JointState, **stamp = FC boot 시계** `time_usec`) | `base_node` — seq 변화 시에만 발행 |
| `/brov/sensor/ahrs` stamp 를 FC boot 시계(`time_boot_ms`)로 | 도착 시각은 bag 기록 시각에 보존 |
| `deadtime_test` bag 목록에 servo 추가 | 완료 |
| `diag_loop_delay --mode m3/m4` | m3=명령→서보(도착 시계), m4=서보→자이로(**FC 시계**, 링크 무관). 핵심 수학 `xcorr_delay` 는 합성 시험으로 검증 |
| M2 유틸 | `ros2 run brov_base diag_link_rtt` (TIMESYNC, param 폴백, pymavlink 단독) |
| 시험 | `test_diag_loop_delay` 18개 (지연 복원·FC stamp 계약·중복 발행 금지 포함) |

**SITL null 검증 (2026-09-02, 직결):** M4 = **0 ms** (SITL 액추에이터는
gazebo_linear 즉시형이므로 0 이 정답 — 도구가 참을 잰다는 검증),
M3 = 50 ms (r=0.998). M3+M4 ≈ 직결 τ_total 실측 60 ms 로 산술이 닫힌다.

**함정 (실기 주의):** mavproxy 경유에서는 servo 스트림이 4 Hz 로 강제됐다
(mavproxy 기본 streamrate 가 우리의 25 Hz 요청을 덮어씀) — M4 불가.
`--mode m4` 가 저속이면 경고한다. 실기에서 BlueOS 에 QGC/Cockpit 이 붙어
있으면 같은 간섭이 가능하므로 **M4 주행 중에는 GCS 를 끊을 것.**

원판 계획표는 아래에 남긴다.

### (원판) 준비 배선 목록

| 작업 | 내용 |
|---|---|
| 서보 토픽 | `base_node` 가 `/brov/sensor/servo_out` 발행 — 8ch PWM + **FC `time_usec` 원본 보존** (`/brov/sensor/*` 원시 토픽 계열, 2026-09-02 신설분과 같은 자리) |
| AHRS FC 시각 | `/brov/sensor/ahrs` 에 FC `time_boot_ms` 원본 병행 보존 (현재 stamp 정책 확인 후) |
| bag 목록 | `deadtime_test.launch.py` 의 `_BAG_TOPICS` 에 서보 토픽 추가 |
| 분석기 | `diag_loop_delay` 에 M3/M4 모드 (`--from cmd|servo --to servo|gyro --clock wall|fc`) |
| M2 유틸 | `runtime/timesync_rtt.sh` (pymavlink 단독, 스택 불필요) |
| 시험 | M4 합성 신호 시험 (알려진 FC-시계 지연 복원) — `test_diag_loop_delay` 방식 재사용 |

기존 `mavlink_time.py` 의 boot time 추적(리셋 감지)이 있으므로 FC 시계 원본을
보존만 하면 된다. 벽시계 매핑은 M4 에는 불필요하다.

## 4. 실기 절차 (기존 A2-yaw 프로토콜 재사용)

1. **M1/M2** — 스택 없이. ping 50회 + TIMESYNC/param 왕복 50회. 기록.
2. **A2-yaw 재실행** — `deadtime_test.launch.py axis:=yaw bias:=1.0 amplitude:=0.5
   duration_s:=40` (2026-09-02 확정 프로토콜 그대로, 서보 토픽만 추가로 bag).
3. **chirp 1회** — `waveform:=chirp 0.5→8 Hz` (결과 문서 §2-2 의 미완 항목).
   교차상관 반폭 ~30 ms 로 τ 정밀화 + jitter 상한.
4. 분석: M3, M4, 정합성 검사.
5. **(조건부) RC_SPEED A/B** — §5 판정표가 가리킬 때만.

소요: 배선 반나절, 실기 측정 자체는 15분 내외 (2번이 40 s, 나머지는 정지 상태).

## 5. 판정표

| 결과 | 대응 | 근거 |
|---|---|---|
| **M4 ≥ 40 ms** (액추에이터 지배) | `RC_SPEED` 200→400 Hz A/B. [ArduSub 파라미터](https://www.ardusub.com/developers/full-parameter-list.html) 범위 50~490 Hz. 포럼 실측 400 Hz→~25 ms | 온보드 이전 **불필요** — 링크를 줄여도 소용없다 |
| **M2 ≥ 30 ms** (링크 지배) | 제어 스택 온보드 이전 검토. BlueOS 는 내부 프로그램용 loopback endpoint 를 지원: *"Bridges to internal programs can use the loopback IP `127.0.0.1`"* ([BlueOS docs](https://blueos.cloud/docs/latest/usage/advanced/)). 전례: [blueos-ros2 extension](https://github.com/itskalvik/blueos-ros2), [blue 프레임워크 RPi4 온보드 보고](https://github.com/Robotic-Decision-Making-Lab/blue/discussions/161) | RPi4 에서 정책 추론 25 Hz 가 도는지 먼저 확인 (TorchScript CPU 추론 벤치) |
| **잔차 지배** (FC 스케줄/telemetry) | `SET_MESSAGE_INTERVAL` 간격 상향, 라우터 교체 시험(MAVLinkServer/MAVP2P — BlueOS 에서 전환 가능) | 문서에 지연 수치는 없다 — A/B 로만 판정 |
| 어느 경우든 | **학습 지연 주입은 유지** — 잔여 τ 는 0 이 안 된다. 주입 범위만 분해 결과로 갱신 | `deadtime_result_to_training.md` §6 |

## 6. RC_SPEED 변경 시 주의 (조건부 항목)

- 지상 **벤치에서 먼저** (추진기 물 밖 무부하 짧게, 또는 프로펠러 제거).
  BLHeli_S 가 400 Hz PWM 입력을 받는 것은 포럼 실측으로 확인돼 있으나
  **이 개체의 ESC 펌웨어에서 재확인**할 것.
- 바꾸면 dead time 이 달라지므로 **A2-yaw 를 반드시 재측정** — 학습 주입값이
  바뀐다.
- 롤백: `RC_SPEED` 원복(기본 200) 후 재부팅.

## 6b. 보조 수단과 기각 수단 (조사 기록, 2026-09-02)

**telemetry 주기 상향 (보조, 이득 작음).** ATTITUDE 등을 25→50 Hz 로 올리면
되먹임 쪽 양자화가 평균 ~20→~10 ms 로 줄고, 실측된 "15% 가 1틱 묵음"
(attitude_age 40~50 ms) 도 완화된다. 대역폭 부담은 미미. 단
SET_MESSAGE_INTERVAL 이 무시되는 경로가 실재하므로(§3 함정) 적용 후 실제
수신 주기를 확인할 것.

**기각 — 정책 루프 25 Hz 상향.** ZOH 위상은 줄지만 정책이 dt=40 ms 로
학습됐고 z_v/z_q 적분 계약이 dt 에 묶여 있다. 재학습 없이 올리면 계약 위반.

**미검증 후보 — DShot 등 디지털 ESC 프로토콜.** 아날로그 PWM 의 프레임
지연을 원리상 제거하지만, 이 FC/Basic ESC(BLHeli_S) 조합의 지원 여부를
확인하지 않았다. RC_SPEED 400 Hz 로 부족할 때의 후속 조사 항목.

**상호작용 요약.** 지연 축소는 안정 문턱을 올린다(τ=50 ms 에서 K_p 문턱
5.01 — baseline 4.5 도 재학습 없이 안정권, 여유 10%). delayA 정책과는 자유
조합. **단 설계 B(행동 이력, 학습 40~80 ms)는 총 지연이 40 ms 미만이 되면
붕괴한다**(OceanRL `DELAY_TRAINING_PLAN.md` §5-8) — B 채택 시 하한 0 포함
재학습이 선행돼야 한다.

## 7. 이 계획이 답하지 않는 것

- **jitter 의 분포** — chirp 가 상한만 준다. 분포가 필요해지면(주입을 범위로
  할 때) M3 를 에지별 개별 지연으로 풀어서 히스토그램을 만든다.
- **DO_SET_SERVO 경로가 RC override 보다 빠른가** — ArduSub 문서에 대안으로
  존재하나 지연 이점은 **문서화돼 있지 않다**. 분해 결과 FC 스케줄 몫이 크게
  나오면 그때 A/B 후보.
