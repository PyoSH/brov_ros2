"""dead time 측정 — 정책 대신 알려진 신호를 넣고 명령->가속도 지연을 잰다.

왜 별도 launch 인가
===================
`pool_demo_a.launch.py` 로도 bag 은 남고 `diag_loop_delay` 는 돌아간다. 그러나
그 주행의 여기(excitation)는 **선회와, 혹시 난다면 진동**뿐이다. 정책이 잘
추종하면 순항 중 명령이 거의 일정해서 교차상관이 서지 않는다 -- 진동이 나야
측정이 되는, 우연에 기댄 실험이 된다.

dead time 은 통신·계산·구동에 걸리는 **경로의 성질**이라 무엇이 명령을 만들든
같다. 분리 스택의 절단면이 wrench 이므로 정책 노드를 여기 발생기로 갈아끼우면
`base_node` 아래로는 아무것도 달라지지 않는다. 그러면 여기가 보장되고, 정책의
동특성이 추정에 섞이지도 않는다.

무엇을 띄우지 않는가
====================
`guidance_node`, `observation_node`, `policy_wrench_node` 를 띄우지 않는다.
`/brov/cmd/wrench` 의 발행자가 **정확히 하나**여야 하기 때문이다. 미션도
waypoint 도 없다 -- 기체는 제자리에서 흔들릴 뿐이다.

축 고르기
=========
`heave` 를 기본으로 둔다. 수평 이동이 없어 벽까지의 여유를 쓰지 않고, 수조
깊이 0.7 m 안에서 ±5~10 cm 로 끝난다. `surge` 는 같은 진폭에서 앞뒤로 밀리므로
길이 방향 여유를 확보하고 `duration_s` 를 짧게 잡을 것.

부력
====
사각파는 평균이 0 이지만 BlueROV2 는 대개 약간 양성부력이라 그대로 두면
서서히 떠오른다. 무추력(`send_pwm:=false`)으로 한 번 띄워 상승 속도를 보고,
`bias` 에 그것을 상쇄할 값을 넣은 뒤 본 측정을 한다.

역전 없이 재기 — yaw
====================
heave ±20 N 은 수직 추진기를 매 에지마다 0 을 관통시킨다(PWM +0.25 -> -0.26).
ESC(BLHeli_S)는 역전 중 전력을 제한하므로 그 측정값은 **통신 지연 + 역전 지연**
이다. 역전을 빼려면 추진기가 한 방향으로 계속 돌아야 하는데, heave 에 bias 를
걸면 기체가 그 힘만큼 움직인다 -- 15 N 이면 3 초 안에 바닥이다.

yaw 는 bias 를 걸어도 제자리에서 천천히 돌 뿐이다. `bias:=1.0 amplitude:=0.5`
면 수평 추진기 4 개가 PWM 0.10~0.16 에 머물고(deadband ±0.075 밖, 0 을 안 넘음),
회전율 ~47 deg/s 다. 덤으로 각속도는 자이로에서 직접 오므로 EKF 속도 융합의
필터 지연도 빠진다. 분석은 `diag_loop_delay --axis yaw --open-loop`.

운용::

    # 0) 무추력으로 부력 확인
    ros2 launch brov_bringup deadtime_test.launch.py send_pwm:=false arm:=false

    # 1) 본 측정
    ros2 launch brov_bringup deadtime_test.launch.py \\
        axis:=heave amplitude:=20.0 bias:=<상쇄값> duration_s:=60 \\
        bag_path:=<기록경로>/deadtime_heave send_pwm:=true arm:=true

    ros2 service call /brov/prepare_control std_srvs/srv/Trigger
    ros2 service call /brov/arm_control     std_srvs/srv/Trigger
    ros2 service call /brov/start_control   std_srvs/srv/Trigger
    #   ... duration_s 뒤 여기가 스스로 멈춘다 ...
    ros2 service call /brov/stop_control    std_srvs/srv/Trigger
    ros2 service call /brov/disarm_control  std_srvs/srv/Trigger

    # 2) 분석 — 여기를 넣은 축과 **같은 축**으로 본다
    ros2 run brov_base diag_loop_delay <기록경로>/deadtime_heave --axis heave
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 분석에 필요한 것만. 이 실험에는 미션도 관측도 없다.
_BAG_TOPICS = (
    # 교차상관은 이 둘이 같은 시계로 있어야 성립한다.
    "/brov/cmd/wrench",
    "/brov/state",
    "/brov/control_active",
    # 원시 센서. 지연이 어느 구간에서 생겼는지 사후에 좁힐 단서다.
    "/brov/sensor/ahrs",
    # M3/M4 지연 분해용 — FC boot 시계 stamp (2026-09-02 배선)
    "/brov/sensor/servo_out",
    "/brov/sensor/depth_ekf",
    "/brov/sensor/pressure0",
    "/brov/sensor/pressure1",
    "/brov/sensor/pressure2",
    "/brov/dvl/sample",
)


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _unique_bag_path(path: str) -> str:
    """이미 있는 경로면 시각을 덧붙인다.

    `ros2 bag record -o` 는 디렉터리가 있으면 거절하고 죽는데 나머지 스택은
    멀쩡히 돈다. 축을 바꿔 가며 여러 번 도는 실험이라 여기서 특히 잘 밟는다.
    """
    path = path.rstrip("/")
    if not path or not os.path.exists(path):
        return path
    return f"{path}-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}"


def _compose(context):
    def cfg(name):
        return LaunchConfiguration(name)

    def value(name):
        return LaunchConfiguration(name).perform(context)

    actions = [
        Node(
            package="brov_base",
            executable="base_node",
            name="brov_base",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "connection": cfg("connection"),
                "thruster_reversal_profile": cfg("thruster_reversal_profile"),
                "thruster_model": cfg("thruster_model"),
                "depth_source": cfg("depth_source"),
                "send_pwm": ParameterValue(cfg("send_pwm"), value_type=bool),
                "telemetry_rate_hz": ParameterValue(
                    cfg("telemetry_rate_hz"), value_type=float),
                "actuation_backend": cfg("actuation_backend"),
                "arm": ParameterValue(cfg("arm"), value_type=bool),
                "state_rate_hz": ParameterValue(
                    cfg("state_rate_hz"), value_type=float),
            }],
        ),
        Node(
            package="brov_control",
            executable="diag_excite_node",
            name="brov_diag_excite",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "axis": cfg("axis"),
                "kind": cfg("kind"),
                "amplitude": ParameterValue(cfg("amplitude"), value_type=float),
                "bias": ParameterValue(cfg("bias"), value_type=float),
                "period_s": ParameterValue(cfg("period_s"), value_type=float),
                "chirp_f0_hz": ParameterValue(
                    cfg("chirp_f0_hz"), value_type=float),
                "chirp_f1_hz": ParameterValue(
                    cfg("chirp_f1_hz"), value_type=float),
                "duration_s": ParameterValue(
                    cfg("duration_s"), value_type=float),
                # 여기 주기와 제어 주기를 **같게** 둔다. 다르면 ZOH 위상이
                # diag_loop_delay 의 control_dt 가정과 어긋난다.
                "rate_hz": ParameterValue(
                    cfg("state_rate_hz"), value_type=float),
                "depth_hold": ParameterValue(cfg("depth_hold"), value_type=bool),
                "rise_m": ParameterValue(cfg("rise_m"), value_type=float),
            }],
        ),
    ]

    if _is_true(value("dvl")):
        actions.append(
            Node(
                package="brov_control",
                executable="dvl_record_node",
                name="brov_dvl_record_node",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "dvl_host": ParameterValue(cfg("dvl_host"), value_type=str)
                }],
            )
        )

    if _is_true(value("record_bag")):
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o",
                     _unique_bag_path(value("bag_path")), *_BAG_TOPICS],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    # brov_bringup 의 share 가 존재하는지만 확인한다 -- 이 launch 는 config 를
    # 읽지 않지만, 설치되지 않은 워크스페이스에서 조용히 도는 것보다 낫다.
    get_package_share_directory("brov_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "connection", default_value="udpout:192.168.2.2:14550",
            description=(
                "BlueOS 는 14550 에서 UDP 서버로 동작한다 — udpin: 은 닿지 않는다."
            ),
        ),
        # ── 여기 신호 ──
        DeclareLaunchArgument(
            "axis", default_value="heave",
            choices=["surge", "sway", "heave", "roll", "pitch", "yaw"],
            description=(
                "여기를 실을 축. heave 가 기본 -- 수평 이동이 없어 벽 여유를 "
                "쓰지 않는다. 분석도 **같은 축**으로 해야 한다."
            ),
        ),
        DeclareLaunchArgument(
            "kind", default_value="square", choices=["square", "chirp"],
            description=(
                "square: 모서리가 넓은 대역을 때려 교차상관 봉우리가 가장 "
                "뾰족하다(고정 지연 추정에 최적). chirp: 주파수별 위상을 볼 때."
            ),
        ),
        DeclareLaunchArgument(
            "amplitude", default_value="20.0",
            description=(
                "축 성분 진폭 [N] 또는 [N*m]. heave 유효질량 28.1 kg 에서 "
                "20 N 이면 가속도 0.71 m/s^2, 1 s 주기에서 진폭 ±5~10 cm. "
                "각축(roll/pitch/yaw)은 한계가 10 N*m 라 기본 20 은 거절된다 -- "
                "yaw 는 amplitude:=0.5 bias:=1.0."
            ),
        ),
        DeclareLaunchArgument(
            "bias", default_value="0.0",
            description=(
                "부력 상쇄용 상수항 [N]. 사각파는 평균이 0 이지만 기체가 "
                "양성부력이면 서서히 떠오른다. |amplitude|+|bias| 가 한계를 "
                "넘으면 노드가 뜨지 않는다."
            ),
        ),
        DeclareLaunchArgument(
            "period_s", default_value="1.0",
            description=(
                "사각파 주기 [s]. 1 s 면 기본 주파수 1 Hz 이고 홀수 고조파가 "
                "3·5·7 Hz 를 덮는다 -- 예상 진동대(1.5~3.5 Hz)를 감싼다."
            ),
        ),
        # ── 깊이 유지 (음성부력 기체) ──
        DeclareLaunchArgument(
            "depth_hold", default_value="true", choices=["true", "false"],
            description=(
                "start 순간의 깊이를 느린 루프로 지킨다. 무게를 몰라도 된다 -- "
                "적분항이 맞춘다. 음성부력 기체가 바닥에 닿아 마찰이 섞이는 것을 "
                "막는다. axis:=heave 에서는 측정 축에 섞이므로 꺼진다."
            ),
        ),
        DeclareLaunchArgument(
            "rise_m", default_value="0.0",
            description=(
                "start 깊이에서 몇 m 띄워 유지할지 (0~0.6). 바닥에서 시작하면 0.4. "
                "0 이면 start 깊이 그대로."
            ),
        ),
        DeclareLaunchArgument("chirp_f0_hz", default_value="0.2"),
        DeclareLaunchArgument("chirp_f1_hz", default_value="5.0"),
        DeclareLaunchArgument(
            "duration_s", default_value="60.0",
            description=(
                "여기 지속 시간 [s]. 지나면 스스로 중립으로 돌아간다. "
                "1 Hz 사각파 60 s 면 60 주기 -- 교차상관에 충분하다."
            ),
        ),
        # ── 실기 기본값 ──
        DeclareLaunchArgument(
            "thruster_reversal_profile", default_value="real_brov2",
            choices=["real_brov2", "edo_sitl_identity"]),
        DeclareLaunchArgument(
            "thruster_model", default_value="t200_table",
            choices=["t200_table", "gazebo_linear"]),
        DeclareLaunchArgument(
            "depth_source", default_value="mavlink_ekf",
            choices=["mavlink_ekf", "pressure"]),
        DeclareLaunchArgument(
            "state_rate_hz", default_value="25.0",
            description=(
                "상태 발행과 여기 발행을 같은 주기로 묶는다. "
                "diag_loop_delay 의 --control-dt 와 일치해야 한다 (25 Hz = 0.04)."
            ),
        ),
        # ── 안전 기본값 ──
        # G1: telemetry 주기 A/B (25 vs 50). 직결 경로에서만 의미 있다 --
        # mavproxy 경유는 streamrate 가 덮어쓴다.
        DeclareLaunchArgument("telemetry_rate_hz", default_value="25.0"),
        # G3: 액추에이션 경로 A/B (진단 전용)
        DeclareLaunchArgument("actuation_backend", default_value="rc_override",
                              choices=["rc_override", "do_set_servo"]),
        DeclareLaunchArgument(
            "send_pwm", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "arm", default_value="false", choices=["true", "false"]),
        # ── 기록 ──
        DeclareLaunchArgument(
            "record_bag", default_value="true", choices=["true", "false"],
            description="지연은 사후에 다시 잴 방법이 없다. 기본으로 켠다.",
        ),
        DeclareLaunchArgument("bag_path", default_value="deadtime"),
        DeclareLaunchArgument(
            "dvl", default_value="false", choices=["true", "false"],
            description=(
                "이 실험에는 필요 없다. A50 의 TCP 슬롯을 차지해 BlueOS 의 DVL "
                "extension 을 밀어낼 수 있으므로 기본은 끈다."
            ),
        ),
        DeclareLaunchArgument("dvl_host", default_value="192.168.2.95"),
        OpaqueFunction(function=_compose),
    ])
