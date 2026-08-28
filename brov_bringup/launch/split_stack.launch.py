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
true여야 하고, 그 뒤에도 `/brov/base/prepare` → `/brov/base/arm` 서비스를
차례로 불러야 한다. 명령이 `wrench_command_timeout_s` 동안 끊기면 base가
중립 정지한다 — 분리가 만든 위험을 갚는 유일한 장치다.
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
