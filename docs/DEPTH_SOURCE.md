# 깊이 출처 — `depth_source`

논문(arXiv:2512.08656v2) §5.2 는 센서를 셋으로 나눈다:

> **Water Linked A50 DVL** — "measuring body velocity **v^b** and estimating position (x and y)"
> **BlueRobotics Bar30 Pressure Sensor** — "We measure the **depth** (z, positive down)"
> **INS** — "the orientation is provided by the inertial navigation system"

즉 **깊이는 압력센서 전담이고 EKF 를 거치지 않는다.** `depth_source=pressure` 가 그
구현이다. 우회가 아니라 논문 충실 구현이다.

## 왜 필요했나

2026-08-29 Gazebo SITL 에서 `LOCAL_POSITION_NED.z`(= EKF 수직 위치)가 **초기값에
얼어붙었다.** 기체가 GT 기준 5.8 m 상승하는 동안 `-0.10 ~ +0.12 m` 를 보고했다.
같은 메시지의 `vz` 는 `-0.318 m/s` 로 상승을 정확히 알고 있었으므로, **속도는 맞고
위치만 적분되지 않는** 상태였다. `EK3_SRC1_POSZ = 1`(BARO)로 소스는 이미 지정돼
있었으니 소스 선택 오설정은 아니다. 원인 미규명.

guidance 의 수직 LOS 항이 이 값으로 오차를 계산하므로 `w_desired` 가 계속 0 이었고,
폐루프가 자연 부력 드리프트(+4.2 mm/s)를 **7.8 배 증폭**했다(+32.8 mm/s). 세 미션
모두 기체가 수면까지 떠올랐다. 관측 16-D 에는 위치가 없으므로 정책 쪽에서 잡아줄
경로도 없다 — **이 문제는 guidance 전용이다.**

## 어떻게 구현했나

**instance 를 추측하지 않는다.** `SCALED_PRESSURE`/`2`/`3` 는 baro instance
`0`/`1`/`2` 를 그대로 보낸다(`GCS_Common.cpp:2309-2319`) — primary 와 무관하다.
ArduSub 는 init 에서 `BARO_TYPE_WATER` 인 첫 instance 를 찾아 `depth_sensor_idx` 로
잡고 `set_primary_baro()` 로 승격하는데(`ArduSub/system.cpp:108`), 그 함수가
`set_and_save` 이므로(`AP_Baro.h:181`) 결과가 **`BARO_PRIMARY` 파라미터에 남는다.**
그래서 prepare 에서 이 파라미터를 조회하면 확정된다. 실기/SITL 공통이다.

> SITL 에서는 응답 실험으로 식별할 수 없다 — `AP_Baro_SITL.cpp:21` 이 모든 시뮬레이션
> baro 를 `BARO_TYPE_WATER` 로 설정하므로 둘 다 깊이에 반응한다(실측 기울기
> instance 0 = 100.38, instance 1 = 98.29 hPa/m). 실기에서는 내부 baro 가 건조
> 하우징 안이라 ~0 이 나오지만, SITL 만으로 방법을 정하면 안 된다.

**변환은 ArduSub 식 그대로** (`AP_Baro.cpp:888`):

    altitude = (ground_pressure - pressure) / 9800 / SPEC_GRAV    [Pa]

`ground_pressure` 자리에 **start 시점 기준압**을 쓴다 — guidance 가 mission frame
원점을 잡는 바로 그 순간이다. 결과는 상대 깊이이고, (1) 센서 상수 편의가 정확히
상쇄되며 (2) guidance 는 어차피 원점을 빼므로 필요한 값과 일치한다.
`BARO_SPEC_GRAV` 도 prepare 에서 함께 읽는다 — 담수(1.0)/해수(1.024)를 암묵
가정하지 않는다.

## SITL 검증 결과 (통과)

**정적** — `set_pose` 로 알려진 깊이 5 점(4.5 m 범위):

| 항목 | 값 |
|---|---|
| 상관 | 0.999993 |
| 회귀 기울기 | 1.0262 |
| 절편 | -0.009 m |
| 오차 RMS / 최대 | 0.060 / 0.111 m |

기울기 2.6% 초과는 SITL 수압 모델(실측 100.38 hPa/m)과 ArduSub 상수(98.0)의
불일치다. 미션 전형 변위 ±0.5 m 에서 13 mm 라 무해하지만 SITL 산물로 기록해 둔다.

**미션** — Fig.4 (a)(b)(c), 총 수직 이동:

| case | EKF 깊이 | 압력 센서 |
|---|---:|---:|
| (a) | +1.767 m | **+0.186 m** |
| (b) | +1.727 m | **-0.712 m** |
| (c) | +1.800 m | **+0.158 m** |
| 제어 후 상승률 | +32.8 mm/s | **-0.6 ~ -1.1 mm/s** |

폐루프가 드리프트를 증폭하던 것에서 억제하는 쪽으로 바뀌었고 수면 접촉이 사라졌다.
(b) 의 -0.712 m 는 밸러스트 음성 부력에 대한 정상 반응이다(제어 전 -8.1 mm/s).

## 실기 게이트 결과 (2026-09-02) — 통과, 단 결론이 SITL 과 반대

수조에서 `diag_depth_gate`(sweep 방식, 56 cm 왕복 40 s)로 게이트를 돌렸다:

| 항목 | 결과 |
|---|---|
| 물속 센서 | **instance 1 (`SCALED_PRESSURE2`)** — 압력 폭 5510 Pa. instance 0 은 내부(92.9 Pa) |
| EKF 추종 | `depth_ekf = 1.005 · depth_baro + 0.000`, R² 0.9935, 잔차 RMS 1.4 cm |
| SPEC_GRAV | 기울기 1.005 ≈ 1/SPEC_GRAV → 담수 1.0 일치 |

**실기 EKF 는 압력을 정상 추종한다 — SITL 의 "수직 위치 얼어붙음" 증상이 실기에
없다.** 따라서 **실기는 `mavlink_ekf` 유지**가 결정이고, `pressure` 전환은
SITL 전용으로 남는다. instance 번호도 예상대로 SITL(0)과 실기(1)가 달랐다 —
`BARO_PRIMARY` 로 확정하는 설계가 맞았다.

단서: EKF 가 같은 baro 를 융합하므로 완전 독립 검증은 아니다 — "EKF 가 자기
소스를 제대로 적분하는가" 를 본 것이고, SITL 실패가 정확히 그 지점이었다.

## 실기 전환 게이트 (원판 — 위 결과로 소화됨)

기본값은 **`mavlink_ekf` 로 둔다.** SITL 게이트는 통과했지만 실기에서 한 번도
돌리지 않았다. 전환 전에 첫 수조 시험에서 아래를 확인할 것:

1. **`BARO_PRIMARY` 조회값이 Bar30 instance 인가.** prepare 로그의
   `depth sensor 확정 — baro instance N` 을 확인한다. 실기는 SITL 과 번호가 다를
   수 있다(probe 순서). 코드는 파라미터를 따르므로 대응되지만, **값이 무엇인지
   기록에 남겨야 한다** — `BrovState.depth_baro_instance` 에 실린다.
2. **`BARO_SPEC_GRAV` 가 시험 수조에 맞는가.** 담수 1.0 / 해수 1.024. 틀리면
   깊이에 2.4% 스케일 오차가 붙는다.
3. **기지 깊이 대조.** 표시한 줄로 1 m 하강시키고 `/brov/state.position.z` 와
   비교한다. SITL 정적 검증과 같은 절차다.
4. **내부 baro 가 건조한지 교차확인.** 세 `SCALED_PRESSURE*` 를 모두 기록해
   깊이에 반응하지 않는 instance 가 있는지 본다. 있다면 그것이 내부 baro 이고,
   `BARO_PRIMARY` 가 그것을 가리키면 **설정이 잘못된 것이다.**

네 가지가 통과하면 `base_node.py` 의 `declare_parameter("depth_source", ...)` 와
`split_stack.launch.py` 의 `DeclareLaunchArgument("depth_source", ...)` 기본값을
`"pressure"` 로 바꾼다. 그 전까지 SITL 실험은 launch 인자로 명시해서 쓴다.

## 영향 범위

- **`guidance_node`** — 영향 받는다. LOS 수직 오차와 waypoint 전환이 이 위치를 쓴다.
- **`observation_node` -> policy** — **영향 없다.** 관측 16-D
  `[q_e, v_e^b, omega^b, z_v, z_q]` 에 위치가 없다. 깊이는 정책에 도달하지 않으며
  정책 계약이나 artifact 는 건드리지 않는다.
