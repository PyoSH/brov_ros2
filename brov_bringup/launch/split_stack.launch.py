#!/usr/bin/env python3
"""역할별로 분리한 4노드 스택.

    brov_base_node        로봇 I/O + 안전 + 액추에이터   /brov/state, /brov/cmd/wrench
    brov_guidance_node    경로 → 목표                    /brov/desired
    brov_observation_node 상태+목표 → 16-D, 적분 소유    /brov/observation
    brov_policy_wrench_node 관측 → wrench                /brov/cmd/wrench

    state ──▶ guidance ──▶ desired ──▶ observation ──▶ policy ──▶ wrench ──▶ base
      ▲                                    ▲                                  │
      └────────────────────────────────────┴──────────────────────────────────┘

절단면이 **wrench**인 이유
==========================
base   가 액추에이터 고유 지식을 독점한다 — 할당행렬, T200 테이블, 전압,
       deadband, 포화, PWM slew
policy 가 아티팩트 고유 지식을 독점한다 — 계약, wrench_scale, T6, clamp

경계가 물리량이라 테스트할 수 있고, `model_based_controller_node`가
policy를 **노드 교체 하나로** 대체한다(같은 토픽에 같은 Wrench6).

**기존 obs_node 스택과 동시에 띄우면 안 된다.**
=================================================
obs_node는 MAVLink를 직접 열고 PWM을 직접 보낸다. 두 프로세스가 링크를 열면
서로의 PWM을 덮어써서 어느 쪽이 이겼는지 사후에 알 수 없다. 이관이 끝날 때까지
둘 중 하나만 띄운다.

안전
====
`send_pwm`과 `arm`은 기본 false다. 실제로 추력을 내려면 둘 다 명시적으로
true여야 하고, 그 뒤에도 lifecycle 서비스를 차례로 불러야 한다. 이름과 의미는
legacy 스택과 같으므로 `demo_orchestrator_node`가 그대로 구동한다:

    /brov/prepare_control   telemetry/mode 확인 + SERVO1~8 → RCPassThru
    /brov/arm_control       하드웨어 arm (prepare 선행 필수)
    /brov/start_control     제어 개시 — 여기서부터 적분·PWM이 나가고,
                            waypoint_frame=start_heading의 원점이 잡힌다
    /brov/stop_control      제어 동결 (armed 유지, 중립 송신)
    /brov/disarm_control    제어 동결 + neutral/disarm + prepared 폐기
    /brov/reset_integrator  적분 reset (frozen 유지)
    /brov/estop             (토픽) 즉시 중립 + latch

arm 과 start 를 나눠 둔 이유: 합치면 원점이 "추진기가 켜진 직후"로 잡히고,
제어만 멈추려 해도 disarm 밖에 길이 없어 passthrough까지 원복된다.

명령이 `wrench_command_timeout_s` 동안 끊기면 base가 중립 정지한다 — 분리가
만든 위험을 갚는 유일한 장치다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("connection", default_value="udpin:0.0.0.0:14550"),
        # 실기(real_brov2)는 T2/T3/T8 반전 보정을 쓰고, Edo SITL은 RC1..8을
        # thruster1..8로 직결하므로 identity다. **기본값은 실기 쪽으로 둔다** --
        # 틀리면 세 추진기가 뒤집힌 채로 밀려 기체가 달아난다(실제로 겪었다:
        # 0.3 m/s 명령에 1.4 m/s로 발산). identity는 udpin: 연결에서만 허용된다.
        DeclareLaunchArgument("thruster_reversal_profile", default_value="real_brov2",
                              choices=["real_brov2", "edo_sitl_identity"]),
        # 액추에이터 모델. 실기는 T200 실측 테이블, Edo SITL은 ardupilot_gazebo의
        # 선형 ±50 N이다. 틀리면 왕복이 항등이 아니라 요청의 1.4~2.1배가 나간다.
        DeclareLaunchArgument("thruster_model", default_value="t200_table",
                              choices=["t200_table", "gazebo_linear"]),
        # 깊이 출처. 논문 5.2 는 Bar30 압력센서로 z 를 직접 측정하고 EKF 를 거치지
        # 않는다("We measure the depth (z, positive down)").
        #
        # "pressure": prepare 에서 BARO_PRIMARY 를 조회해 baro instance 를 확정하고
        #   (ArduSub 가 BARO_TYPE_WATER 인 첫 instance 를 primary 로 set_and_save
        #   하므로 이 값이 곧 depth sensor 다 -- ArduSub/system.cpp:108,
        #   AP_Baro.h:181), start 시점 기준압 대비 상대 깊이를 낸다. 조회에
        #   실패하면 prepare 를 거절한다 -- 조용히 EKF 로 넘어가지 않는다.
        #
        # 기본값이 아직 "mavlink_ekf" 인 이유: SITL 게이트는 통과했지만
        # (정적 상관 0.999993 / 오차 RMS 0.060 m, 미션 상승 1.767 -> 0.186 m)
        # **실기에서 한 번도 돌리지 않았다.** 실기 전환 게이트는
        # docs/DEPTH_SOURCE.md 를 볼 것. SITL 실험은 이 인자를 명시해서 쓴다.
        DeclareLaunchArgument("depth_source", default_value="mavlink_ekf",
                              choices=["mavlink_ekf", "pressure"]),
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("metadata_path", default_value=""),
        DeclareLaunchArgument("vehicle_model_path", default_value=""),
        DeclareLaunchArgument("waypoints", default_value="0,0,0;3,0,0"),
        DeclareLaunchArgument("cruise_speed", default_value="0.2"),
        DeclareLaunchArgument("heading_mode", default_value="straight"),
        DeclareLaunchArgument("lookahead_dist", default_value="1.0"),
        DeclareLaunchArgument("reach_threshold", default_value="0.15"),
        # obs_node와 같은 기본값. "0,0,0;3,0,0"이 "기체 정면으로 3 m"를 뜻한다.
        DeclareLaunchArgument("waypoint_frame", default_value="start_heading",
                              choices=["start_heading", "ned"]),
        # loop=false면 마지막 waypoint에서 terminal hold로 넘어간다. 순항 성능을
        # 재려면 정본 미션처럼 true여야 한다.
        DeclareLaunchArgument("loop", default_value="false",
                              choices=["true", "false"]),
        # 세그먼트 길이 상한. 짧게 두는 것이 안전하지만(폭주 시 멀리 못 간다),
        # 순항 추종을 재려면 선회가 지배하지 않는 긴 직선이 필요하다.
        DeclareLaunchArgument("max_segment_length_m", default_value="4.0"),
        # 추력을 내려면 둘 다 true여야 한다. 기본값은 안전한 쪽이다.
        DeclareLaunchArgument("send_pwm", default_value="false"),
        DeclareLaunchArgument("arm", default_value="false"),
        DeclareLaunchArgument("state_rate_hz", default_value="25.0"),
        DeclareLaunchArgument("wrench_command_timeout_s", default_value="0.25"),
    ]
    cfg = LaunchConfiguration
    nodes = [
        Node(
            package="brov_base", executable="base_node", name="brov_base",
            output="screen",
            parameters=[{
                "connection": cfg("connection"),
                "thruster_reversal_profile": cfg("thruster_reversal_profile"),
                "thruster_model": cfg("thruster_model"),
                "depth_source": cfg("depth_source"),
                "send_pwm": cfg("send_pwm"),
                "arm": cfg("arm"),
                "state_rate_hz": cfg("state_rate_hz"),
                "wrench_command_timeout_s": cfg("wrench_command_timeout_s"),
            }],
        ),
        Node(
            package="brov_base", executable="guidance_node", name="brov_guidance",
            output="screen",
            parameters=[{
                "waypoints": cfg("waypoints"),
                "cruise_speed": cfg("cruise_speed"),
                "heading_mode": cfg("heading_mode"),
                "lookahead_dist": cfg("lookahead_dist"),
                "reach_threshold": cfg("reach_threshold"),
                "waypoint_frame": cfg("waypoint_frame"),
                "loop": cfg("loop"),
                "max_segment_length_m": cfg("max_segment_length_m"),
            }],
        ),
        Node(
            package="brov_base", executable="observation_node",
            name="brov_observation", output="screen",
        ),
        Node(
            package="brov_control", executable="policy_wrench_node",
            name="brov_policy_wrench", output="screen",
            parameters=[{
                "policy_path": cfg("policy_path"),
                "metadata_path": cfg("metadata_path"),
                "vehicle_model_path": cfg("vehicle_model_path"),
                "expected_policy_dt_s": 0.04,
            }],
        ),
    ]
    return LaunchDescription(args + nodes)
