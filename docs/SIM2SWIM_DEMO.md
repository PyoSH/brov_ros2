# Sim2Swim 실기체 데모 (case a/c)

이 데모는 `step_2_BROV/test_policy.py`의 두 실험을 현재 수조에 맞게 축소한다.

- case `a`: `straight_line`, 2.0 m 직선 왕복, LOS 방향 자세 정렬
- case `c`: `square_random_attitude`, 한 변 0.4 m 사각 경로, waypoint마다 무작위 자세

두 미션 모두 `start_heading` frame과 시작 깊이 기준 `z=0`을 사용하며 계속
순환한다. 첫 수중 시험용 속도는 case `a` 0.10 m/s, case `c` 0.05 m/s로
제한했다. 두 case 모두 자동 시간·lap 종료가 없으므로 `/brov/stop_control` 또는
비상 정지를 실행할 담당자가 운항 내내 대기해야 한다.

## 수조 좌표와 배치

사용 가능한 수조 크기는 길이(+X) 4.0 m, 너비(+Y) 1.7 m, 수심(+Z, NED) 1.1 m로
가정한다. 아래 거리는 모두 **로봇 중심 기준**이다. 선체, 추진기, tether의 크기는
포함하지 않으므로 실제 물과 장애물까지의 clearance를 별도로 확인해야 한다.

`/brov/start_control`을 호출하는 순간의 로봇 위치가 `(0,0,0)`이 되고, 그 순간
선수 방향이 +X가 된다. +Y는 우현 방향, NED의 +Z는 아래 방향이다. 시작 전에
로봇 중심을 당일 측정한 실제 수심의 약 절반 깊이에 두면 waypoint의 `z=0`은 그
시작 깊이를 유지한다. 명목 1.1 m만 믿지 말고 실제 수면-바닥 거리와 상·하
clearance를 확인한다.

### Case a 배치

로봇 중심을 가까운 짧은 벽에서 1.00 m, 양쪽 긴 벽에서 각각 0.85 m 떨어진
수조 중앙선에 놓고, 선수를 4.0 m 길이 방향의 반대편 짧은 벽으로 향하게 한다.
waypoint 중심은 다음과 같다.

```text
(0.0, 0.0, 0.0) <---- 왕복 ----> (2.0, 0.0, 0.0)
```

따라서 명령된 끝점 기준 종방향 벽 여유는 양쪽 1.00 m, 횡방향 여유는 양쪽
0.85 m이다. 단, 각 끝점에서 `align` 목표가 즉시 180° 반전되며 자세 slew와
자동 감속은 없으므로, 이 여유가 곧 안전거리라는 뜻은 아니다. 낮은 authority의
구속 시험에서 회전 swept envelope와 최대 overshoot를 먼저 측정한다.

### Case c 배치

정사각형을 수조 중앙에 놓는다. 로봇 중심을 가까운 짧은 벽에서 1.80 m,
수조 횡방향 중심선보다 `-Y` 방향으로 0.20 m 떨어진 곳(즉 `-Y`측 긴 벽에서
0.65 m)에 놓고, 선수를 +X 길이 방향으로 향하게 한다. +Y(우현)가 수조
횡방향 중심을 향하는지 반드시 확인한다.

```text
(0.0, 0.4) <---------------- (0.4, 0.4)
     |                             ^
     v                             |
(0.0, 0.0) ----------------> (0.4, 0.0)
```

명령된 square 중심은 수조 중앙이며, waypoint 중심 기준 여유는 종방향 양쪽
1.80 m, 횡방향 양쪽 0.65 m이다.

### Case c 안전 승인 gate

**경고:** case `c`는 roll/pitch 최대 ±90°, yaw 최대 ±180°의 목표를 waypoint마다
새로 만든다. 현재 policy 출력에는 실기체용 torque/PWM authority cap과 slew-rate
제한이 없고, `loop=true`에는 자동 시간·lap 종료도 없다. 아래 항목이 모두 구현·시험·
승인되기 전에는 **case c를 수중에서 실행할 수 없다**.

- 실제 수심에서 all-attitude swept envelope와 tether clearance 측정
- 구속 상태에서 단일 축·작은 각도부터 증가시키는 단계별 자세 시험
- 실기체 torque/PWM authority cap과 slew-rate 제한 구현 및 검증
- `/brov/debug/q_desired_zup`으로 첫 목표 자세를 확인하고 승인하는 절차
- 최대 운항 시간 또는 lap 수에 따른 자동 종료와 비상 정지 담당자 지정

첫 무작위 목표 자세는 shadow mode에서 생성되어 `/brov/debug/q_desired_zup`으로
발행되며, `/brov/start_control`을 호출해도 다시 sample하지 않고 유지된다. 다만 이후
각 waypoint 도달 시의 무작위 자세 step은 여전히 slew 제한 없이 바뀐다.
`allow_case_c:=true`는 이 위험을 읽고 인지했다는 명시적 opt-in일 뿐, 그 자체가
수중 운항 승인은 아니다.

## 실행 전 확인

실제 제어 전에 별도 터미널에 아래 비상 정지 명령을 입력해 두되, 실행(Enter)은
비상시에만 한다. `obs_node` 실행 전에 발행한 메시지는 latch되지 않는다.

```bash
cd /workspace/brov_ros2
ros2 topic pub --once /brov/estop std_msgs/msg/Empty "{}"
```

추진기와 tether를 정리하고, DVL/ExternalNav 및 `LOCAL_POSITION_NED` 상태를
확인한다. 먼저 case별 shadow mode를 실행한다. 이 launch는 RL controller와
camera를 구성하지만 `/brov/start_control`을 자동 호출하지 않으며, 기본값으로
PWM 송신과 arm을 모두 끈다.

```bash
# case a
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=a send_pwm:=false arm:=false

# case c 구성 검토 전용(현재 수중 제어 금지)
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=c allow_case_c:=true send_pwm:=false arm:=false
```

새 터미널에서 구성과 신호를 확인한다.

```bash
cd /workspace/brov_ros2
ros2 param get /brov_obs_node waypoints
ros2 param get /brov_obs_node waypoint_min_xyz
ros2 param get /brov_obs_node waypoint_max_xyz
ros2 topic echo --once /brov/control_active
ros2 topic echo --once /brov/debug/pos_mission
ros2 topic echo --once /brov/debug/q_desired_zup
ros2 topic echo --once /brov/target_waypoint
ros2 topic hz /brov/observation
ros2 topic hz /brov/camera/image_raw
```

`allow_case_c` 기본값은 `false`다. 이 값을 명시하지 않고 case `c`를 선택하면
노드와 hardware 연결을 만들기 전에 launch가 실패한다.

`control_active`가 `false`이고 waypoint/bounds가 선택한 case와 정확히 같아야 한다.
로봇을 손으로 +X와 +Y 방향으로 조금씩 움직여 `/brov/debug/pos_mission`의 축과
부호도 확인한다.

## 실제 제어

shadow launch를 완전히 종료한 뒤 case `a`를 다시 실행한다.

```bash
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=a \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true \
  arm:=true
```

case `c`는 위의 안전 승인 gate가 모두 충족되고 시험 책임자가 승인한 뒤에만 다음
명령을 사용할 수 있다. `allow_case_c:=true`를 붙였다는 사실만으로 실행하면 안 된다.

```bash
ros2 launch brov_bringup sim2swim_demo.launch.py \
  case:=c \
  allow_case_c:=true \
  connection:=udpout:192.168.2.2:14550 \
  send_pwm:=true \
  arm:=true
```

telemetry, 시작 위치와 방향, 비상 정지 준비 상태를 마지막으로 확인한 뒤 제어를
명시적으로 시작한다. RL controller에는 별도 start service가 없다.

```bash
ros2 service call /brov/start_control std_srvs/srv/Trigger "{}"
```

정상 정지는 다음 순서로 수행한다.

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger "{}"
# success=True 확인 후 launch 터미널에서 Ctrl-C
```

## Bounds의 한계

`waypoint_bounds_enabled`, `waypoint_min_xyz`, `waypoint_max_xyz`는 launch 시
**입력된 waypoint가 허용 범위 안인지** 확인해 잘못된 미션을 거부한다. 로봇의
실측 위치를 감시하거나 벽에 접근했을 때 자동 정지/복귀시키는 runtime geofence가
아니다. 관성, 외란, corner cutting, 자세 변화, 위치추정 오차로 선체가 waypoint
box 밖으로 나갈 수 있으므로 작업자가 항상 비상 정지를 담당해야 한다.
