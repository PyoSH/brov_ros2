# A1/A2 사후 분석 스크립트 (2026-09-02)

`source env_native.sh` 뒤 `python3 runtime/analysis/<script> <bag...>`.

- `a1_band.py <gain1.0 bag> <gain0.5 bag>` — 축별 1.8~2.6 Hz 대역 **절대** RMS (명령·응답), 이득 비.
  `diag_loop_delay` 의 전력비는 상대값이라 이득을 바꾸면 비교가 안 된다.
- `a1_saturation.py <gain1.0> <gain0.5> <A2 yaw bag> <deadtime heave bag>` — 행동 포화 비율,
  적분기 클램프, 추진기 클램프, 개루프 대조(기체 자체 2 Hz 유무).
- `a1_drift.py <bagA> <bagB> [라벨A] [라벨B]` — 15 s 창별 yaw 율·|ω| RMS·위치 이동과
  **EKF 위치가 수조 안전영역을 벗어났는지**. 2026-09-03 에 이것으로 "회전이 DVL 을
  흔들어 EKF 를 발산시킨다" 를 확인했다.
- `a1_legs.py <gain1.0> <gain0.5>` — waypoint 전환 수, 실제 속도, roll/pitch/yaw 2 Hz 대역.

결과와 판정은 docs/REAL_ROBOT_SESSION.md §3-D "2026-09-02 A1 결과".

## 지연 분해 (4차)
- `ros2 run brov_base diag_loop_delay <bag> --mode transit [--offset <s>]` — 메시지별 편도
  (랩톱 도착 − FC 도장). 폭(p90−p10)이 큐잉, `--offset` 을 주면 절대값. offset 은
  `m1m2_link.sh`(diag_link_rtt) 가 찍는다. bag 에 `/brov/sensor/servo_out` 이 있어야 한다.
- `transit_compare.py <bag> <rpi_transit.csv> --offset-laptop <s>` — RPi 프로브
  (`runtime/rpi_transit_probe.py`, 로봇에서 실행) CSV 와 같은 메시지를 맞춰 편도를
  FC→RPi / RPi→랩톱 으로 가른다. 온보드 이전 가치 판정.
