"""실기 수조용 왕복 미션 — dead time 측정과 정책 거동 확인을 한 번에.

수조 제약
=========
안전 영역이 `mission_manager_sim2swim_c.yaml` 기준 x 0.35~3.65, y 0.30~1.40,
z 0.20~0.90 [m] 다. 즉 **3.3 x 1.1 x 0.7 m** 이고, SITL 에서 쓴 5 m 사각이나
40 m 직선은 들어가지 않는다. 직선 왕복만 가능하다.

왜 0.25 m/s 인가
================
정책은 V_d = 0.5 로 학습됐지만 **관측 16-D 에 절대 속도가 없다**
(`[q_e, v_e_b, omega_b, z_v, z_q]`). 추종이 잘 되면 v_e ~ 0 이라 0.25 든 0.5 든
정책이 보는 값이 사실상 같다. 2026-08-31 에 export 된 정책의 Jacobian 을 실제
순항 관측 위에서 재서 확인했다 -- z_v 를 0 배에서 5 배까지 흔들어도 K_p 가
-3.8 ~ -4.9 에 머문다. **이득이 속도와 무관하므로 진동 조건도 같다.**

그래서 좁은 수조에서는 낮은 속도가 낫다. 3.3 m 에서 0.5 m/s 면 편도 6.6 초라
선회가 데이터의 절반을 차지하고, 진동 중 벽까지 여유가 적다. 0.25 m/s 면 편도
10 초, 왕복 20 초다.

예측을 하나 적어 둔다: 이득이 속도와 무관하므로 **0.25 m/s 에서도 진동 주파수가
거의 같아야 한다.** 크게 다르게 나오면 기전 설명에 빠진 것이 있다는 뜻이므로,
그것대로 유용한 정보다.

기하
====
`waypoint_frame=start_heading` 이므로 waypoint 는 **start_control 순간의 위치와
기수** 기준이다. `0,0,0;2.5,0,0` + `loop:=true` 로 그 방향 2.5 m 를 왕복한다.
z 는 전부 0 -- 수직 leg 이 없고 시작 깊이를 유지한다. SITL Fig.4 (a) 와 같은
구성이다.

**기체를 놓는 위치가 중요하다.** 기수 방향으로 2.5 m 앞이 안전 영역 안이어야
한다. x 0.5 m 근처에서 +x 를 보게 두면 끝점이 3.0 m 로 여유가 남는다.

깊이 되먹임 주의
================
`depth_source` 기본값은 `mavlink_ekf` 다. 그런데 2026-08-31 SITL 에서 EKF 수직
위치가 초기값에 얼어붙어(기체가 5.8 m 상승해도 +-0.1 m 로 보고) guidance 가
깊이 보정을 전혀 못 했고 기체가 1.77 m 떠올랐다. **수조 깊이 여유는 0.7 m 뿐이라
같은 증상이면 수초 만에 수면 또는 바닥에 닿는다.**

실기 EKF 가 SITL 과 같을지는 모른다. 그래서 `docs/REAL_ROBOT_SESSION.md` 의
깊이 게이트를 **주행 전에** 통과시키고, 필요하면 `depth_source:=pressure` 로
넘길 것.

사용법
======
    ros2 launch brov_bringup pool_mission.launch.py \\
        connection:=udpout:192.168.2.2:14550 \\
        policy_path:=<번들>/policy_raw_flu_mk2.pt \\
        bag_path:=<기록경로> \\
        send_pwm:=false arm:=false          # 0단계: 무추력 확인
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    def cfg(name):
        return LaunchConfiguration(name)

    split = os.path.join(
        get_package_share_directory("brov_bringup"), "launch", "split_stack.launch.py"
    )

    args = [
        # ── 실기 연결. BlueOS 가 Pixhawk 시리얼을 소유하므로 라우터를 우회할 수
        # 없다. SITL 에서 mavproxy 를 뺐던 것은 진단이었지 실기에 적용 가능한
        # 수정이 아니다.
        DeclareLaunchArgument("connection", default_value="udpout:192.168.2.2:14550"),
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("metadata_path", default_value=""),
        DeclareLaunchArgument("vehicle_model_path", default_value=""),
        # ── 안전 기본값. 둘 다 명시적으로 true 로 올려야 추력이 나간다.
        DeclareLaunchArgument("send_pwm", default_value="false"),
        DeclareLaunchArgument("arm", default_value="false"),
        # ── 수조 기하. 2.5 m 왕복이 3.3 m 안전 영역에 들어간다.
        DeclareLaunchArgument("leg_m", default_value="2.5"),
        DeclareLaunchArgument("cruise_speed", default_value="0.25"),
        # ── 깊이 출처. 게이트 통과 전에는 mavlink_ekf 를 유지한다.
        DeclareLaunchArgument("depth_source", default_value="mavlink_ekf",
                              choices=["mavlink_ekf", "pressure"]),
        # ── 기록은 기본으로 켠다. 지연은 사후에 다시 잴 방법이 없다.
        DeclareLaunchArgument("record_bag", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("bag_path", default_value="pool_run"),
    ]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(split),
        launch_arguments={
            "connection": cfg("connection"),
            "policy_path": cfg("policy_path"),
            "metadata_path": cfg("metadata_path"),
            "vehicle_model_path": cfg("vehicle_model_path"),
            "send_pwm": cfg("send_pwm"),
            "arm": cfg("arm"),
            "depth_source": cfg("depth_source"),
            "record_bag": cfg("record_bag"),
            "bag_path": cfg("bag_path"),
            # 실기 기본값을 그대로 쓴다: thruster_reversal_profile=real_brov2,
            # thruster_model=t200_table.
            "waypoints": ["0,0,0;", cfg("leg_m"), ",0,0"],
            "waypoint_frame": "start_heading",
            "heading_mode": "align",
            "loop": "true",
            "cruise_speed": cfg("cruise_speed"),
            # SITL Fig.4 (a) 와 같은 값. 바꾸면 그때 검증한 거동과 달라진다.
            "lookahead_dist": "1.0",
            "reach_threshold": "0.30",
            "max_segment_length_m": "4.0",
        }.items(),
    )
    return LaunchDescription(args + [include])
