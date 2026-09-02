"""실기 수조 demo A — 마커 기반 절대 프레임 왕복 + 전 센서 기록.

무엇이 다른가
=============
`pool_mission.launch.py` 는 분리 스택(base/guidance/observation/policy_wrench)에
`waypoint_frame=start_heading` 으로 왕복을 시킨다. 경로가 **기체를 놓은 자리**에
달려 있으므로, 배치가 틀리면 2.5 m 앞이 벽 밖이다.

이 launch 는 `sim2swim_demo.launch.py case:=a` 의 **마커 기반 절대 프레임**을
같은 분리 스택 위로 옮긴 것이다. waypoint 를 수조 좌표(Z-up, m)로 직접 쓰므로
기체를 어디에 놓든 끝점이 안전 영역 안이다. 마커는 **한 번만** 본다 --
`initialize_pool` 이후 pool_alignment_node 는 vision 을 무시하고 EKF odometry 로
절대 위치를 이어간다. 주행 중 마커가 안 보여도 된다.

`frame:=start_heading` 으로 바꾸면 `drag_test.launch.py use_pool_alignment:=false`
와 같은 방식이 된다 -- start 순간의 AHRS 기수를 +X 로 삼는 상대 프레임이다.
그때도 카메라/ArUco 노드는 그대로 떠서 bag 에 절대 위치를 남긴다(`markers:=false`
로 끈다). 사후에 EKF 적분 드리프트를 재는 유일한 기준이 그것이다.

두 실험 목적
============
1. **dead time 측정.** `/brov/cmd/wrench` 와 `/brov/state` 가 같은 bag 에 같은
   시계로 들어간다. 분석은 `ros2 run brov_base diag_loop_delay <bag>`.
2. **`w_a = 0.017` 정책 배포.** `policy_path` 로 번들을 지정한다.

기본으로 기록한다
=================
지연도 센서 편차도 **사후에 다시 잴 방법이 없다.** `record_bag` 기본값이 true 인
이유다. `/brov/state` 는 depth_source 가 고른 z 하나만 싣지만, bag 에는
`/brov/sensor/depth_ekf` 와 `/brov/sensor/pressure0..2` 가 함께 들어가므로
고르지 않은 쪽도 남는다(docs/REAL_ROBOT_SESSION.md 1단계 깊이 게이트를 bag 만으로
판정할 수 있다). 카메라 영상은 넣지 않는다 -- 분 단위로 GB 를 먹는다.

운용 (marker 프레임)
====================
    ros2 launch brov_bringup pool_demo_a.launch.py \\
        connection:=udpout:192.168.2.2:14550 \\
        policy_path:=<번들>/policy_raw_flu_mk2.pt \\
        vehicle_model_path:=<저장소>/brov_base/brov_base/vendor/brov2_heavy.yaml \\
        bag_path:=<기록경로>/pool_demo_a_run1 \\
        send_pwm:=false arm:=false          # 0단계: 무추력 확인

    # 1) 마커 정렬 — 기체를 마커가 보이는 곳에 **정지시켜** 둔다
    ros2 service call /brov/localization/confirm_camera_tilt_neutral \\
        std_srvs/srv/Trigger
    ros2 service call /brov/localization/initialize_pool \\
        brov_interfaces/srv/InitializePool "{min_samples: 20}"
    ros2 topic echo /brov/localization/status --once      # state: 2, output_valid: true

    # 2) lifecycle
    ros2 service call /brov/prepare_control std_srvs/srv/Trigger
    ros2 service call /brov/arm_control     std_srvs/srv/Trigger
    ros2 service call /brov/start_control   std_srvs/srv/Trigger
    #   ... 60 초 ...
    ros2 service call /brov/stop_control    std_srvs/srv/Trigger
    ros2 service call /brov/disarm_control  std_srvs/srv/Trigger

launch 는 스스로 정렬하지도, arm 하지도, start 하지도 않는다. 정렬 전에
start 하면 guidance 가 **목표를 내지 않고** base watchdog 이 0.25 s 안에 중립
정지시킨다 -- 절대 프레임이 없는 채로 절대 좌표 경로를 따라가는 것보다 안전하다.
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# mission_manager_sim2swim_c.yaml 이 정의한 수조 안전 영역 [m] (pool frame, Z-up).
# 3.3 x 1.1 x 0.7 밖에 안 된다 -- SITL 의 5 m 사각은 들어가지 않는다.
_POOL_ENVELOPE = {
    "x": (0.35, 3.65),
    "y": (0.30, 1.40),
    "z": (0.20, 0.90),
}

# 분석에 필요한 것만 남긴다. 카메라 영상은 넣지 않는다.
_BAG_TOPICS = (
    # dead time 교차상관은 이 둘이 같은 시계로 있어야 성립한다.
    "/brov/cmd/wrench",
    "/brov/state",
    "/brov/control_active",
    "/brov/observation",
    "/brov/desired",
    # 원시 센서. `/brov/state` 가 고르지 않은 경로를 여기서 남긴다.
    "/brov/sensor/ahrs",
    "/brov/sensor/depth_ekf",
    "/brov/sensor/pressure0",
    "/brov/sensor/pressure1",
    "/brov/sensor/pressure2",
    "/brov/dvl/sample",
    # 절대 위치. start_heading 주행에서도 남겨 EKF 드리프트의 기준으로 쓴다.
    "/brov/odometry/local_with_session",
    "/brov/localization/status",
    "/brov/localization/odometry_pool_with_alignment",
    "/brov/aruco/robot_pose_pool",
    "/brov/aruco/visible",
)


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _default_camera_info_path() -> str:
    data_dir = os.environ.get("BROV_DATA_DIR", os.path.expanduser("~/.ros/brov"))
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def _float(context, name: str) -> float:
    text = LaunchConfiguration(name).perform(context).strip()
    try:
        return float(text)
    except ValueError as exc:
        raise RuntimeError(f"{name}={text!r} 는 실수여야 한다") from exc


def _inside_envelope(axis: str, value: float) -> bool:
    low, high = _POOL_ENVELOPE[axis]
    return low <= value <= high


def _unique_bag_path(path: str) -> str:
    """이미 있는 경로면 시각을 덧붙인다.

    `ros2 bag record -o` 는 디렉터리가 있으면 **거절하고 죽는다.** 나머지 스택은
    멀쩡히 돌기 때문에, 같은 `bag_path` 로 두 번째 주행을 하면 기록 없이 주행이
    끝나고 그 사실을 사후에야 안다 -- 지연도 센서 편차도 다시 잴 방법이 없는데.
    (2026-09-02 실기에서 실제로 그렇게 죽었다.)
    """
    path = path.rstrip("/")
    if not path or not os.path.exists(path):
        return path
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    return f"{path}-{stamp}"


def _compose(context):
    bringup_share = get_package_share_directory("brov_bringup")
    split_launch = os.path.join(bringup_share, "launch", "split_stack.launch.py")

    frame = LaunchConfiguration("frame").perform(context).strip().lower()
    if frame not in {"marker", "start_heading"}:
        raise RuntimeError("frame 은 'marker' 또는 'start_heading' 이어야 한다")
    markers = _is_true(LaunchConfiguration("markers").perform(context))
    if frame == "marker" and not markers:
        raise RuntimeError(
            "frame:=marker 는 markers:=true 를 요구한다 — 마커 정렬 없이는 "
            "수조 절대 좌표가 존재하지 않는다"
        )

    leg = _float(context, "leg_m")
    if leg <= 0.0:
        raise RuntimeError("leg_m 은 양수여야 한다")

    if frame == "marker":
        # 수조 좌표를 그대로 쓴다. 여기서 걸러내지 않으면 벽으로 향하는 경로가
        # 게이트를 하나도 건드리지 않고 통과한다 -- guidance 의 한계 검사는
        # 세그먼트 **길이**만 보지 경계 상자를 모른다.
        x0 = _float(context, "start_x_m")
        lane_y = _float(context, "lane_y_m")
        target_z = _float(context, "target_pool_z_m")
        x1 = x0 + leg
        for axis, value in (("x", x0), ("x", x1), ("y", lane_y), ("z", target_z)):
            if not _inside_envelope(axis, value):
                low, high = _POOL_ENVELOPE[axis]
                raise RuntimeError(
                    f"수조 좌표 {axis}={value:.2f} m 가 안전 영역 "
                    f"{low:.2f}~{high:.2f} m 밖이다 (leg_m={leg:.2f})"
                )
        waypoints = (
            f"{x0:.4f},{lane_y:.4f},{target_z:.4f};"
            f"{x1:.4f},{lane_y:.4f},{target_z:.4f}"
        )
        waypoint_frame = "pool"
    else:
        # start 순간의 위치를 원점, AHRS 기수를 +X 로 삼는다. 절대 좌표가
        # 없으므로 안전 영역 검사도 할 수 없다 -- 기체 배치가 유일한 방어다.
        #
        # rise_m: start 깊이에서 몇 m **띄워** 주행할지. 음성부력 기체는 start 전에
        # 바닥에 있으므로 0.4 를 주면 정책이 0.4 m 올린 뒤 그 깊이로 왕복한다.
        # 0 이면 start 깊이 유지. (guidance 는 NED 라 내부에서 부호를 뒤집는다.)
        rise = _float(context, "rise_m")
        if not 0.0 <= rise <= 0.6:
            raise RuntimeError(
                f"rise_m={rise:.2f} — 0 이상, 수조 깊이 여유 안에서 0.6 이하")
        z = -rise
        waypoints = f"0,0,{z:.4f};{leg:.4f},0,{z:.4f}"
        waypoint_frame = "start_heading"

    def cfg(name):
        return LaunchConfiguration(name)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(split_launch),
            launch_arguments={
                "connection": cfg("connection"),
                "policy_path": cfg("policy_path"),
                "metadata_path": cfg("metadata_path"),
                "vehicle_model_path": cfg("vehicle_model_path"),
                "wrench_gain": cfg("wrench_gain"),
                "send_pwm": cfg("send_pwm"),
                "arm": cfg("arm"),
                "depth_source": cfg("depth_source"),
                # 기록은 이 launch 가 직접 한다 -- 원시 센서와 마커 토픽까지
                # 넣어야 하는데 split_stack 의 목록은 제어 경로만 담는다.
                "record_bag": "false",
                "waypoints": waypoints,
                "waypoint_frame": waypoint_frame,
                "cruise_speed": cfg("cruise_speed"),
                # 실기 기본값: thruster_reversal_profile=real_brov2,
                # thruster_model=t200_table.
                "heading_mode": cfg("heading_mode"),
                "loop": "true",
                # SITL Fig.4 (a) 와 같은 값. 바꾸면 그때 검증한 거동과 달라진다.
                "lookahead_dist": "1.0",
                "reach_threshold": "0.30",
                "max_segment_length_m": "4.0",
            }.items(),
        ),
    ]

    if markers:
        actions += [
            Node(
                package="brov_perception",
                executable="camera_stream_node",
                name="brov_camera_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    cfg("camera_config"),
                    {
                        "udp_port": ParameterValue(
                            cfg("udp_port"), value_type=int
                        ),
                        "camera_info_path": ParameterValue(
                            cfg("camera_info_path"), value_type=str
                        ),
                    },
                ],
            ),
            Node(
                package="brov_perception",
                executable="aruco_pose_node",
                name="brov_aruco_pose_node",
                output="screen",
                emulate_tty=True,
                parameters=[cfg("aruco_config")],
            ),
            Node(
                package="brov_localization",
                executable="pool_alignment_node",
                name="brov_pool_alignment",
                output="screen",
                emulate_tty=True,
                parameters=[cfg("localization_config")],
            ),
        ]

    if _is_true(LaunchConfiguration("dvl").perform(context)):
        actions.append(
            Node(
                package="brov_control",
                executable="dvl_record_node",
                name="brov_dvl_record_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "dvl_host": ParameterValue(
                            cfg("dvl_host"), value_type=str
                        )
                    }
                ],
            )
        )

    if _is_true(LaunchConfiguration("record_bag").perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o",
                     _unique_bag_path(
                         LaunchConfiguration("bag_path").perform(context)),
                     *_BAG_TOPICS],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory("brov_perception")
    localization_share = get_package_share_directory("brov_localization")

    return LaunchDescription(
        [
            # ── 프레임 선택 ──
            DeclareLaunchArgument(
                "frame",
                default_value="marker",
                choices=["marker", "start_heading"],
                description=(
                    "marker: ArUco 정렬이 세운 수조 절대 프레임에서 waypoint 를 "
                    "읽는다(기체 배치와 무관하게 벽 여유가 보장된다). "
                    "start_heading: drag_test 와 같은 방식 — start 순간의 AHRS "
                    "기수를 진행 방향으로 삼는 상대 프레임."
                ),
            ),
            DeclareLaunchArgument(
                "markers",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "카메라/ArUco/pool_alignment 노드를 띄운다. frame:=marker "
                    "에서는 필수다. start_heading 에서도 기본으로 켜 둔다 — "
                    "EKF 적분 드리프트를 사후에 잴 유일한 절대 기준이다."
                ),
            ),
            # ── 실기 연결 ──
            DeclareLaunchArgument(
                "connection",
                default_value="udpout:192.168.2.2:14550",
                description=(
                    "BlueOS 는 14550 에서 UDP 서버로 동작한다 — udpin: 은 기체가 "
                    "먼저 보내주기를 기다리는 방식이라 닿지 않는다."
                ),
            ),
            DeclareLaunchArgument("policy_path", default_value=""),
            DeclareLaunchArgument("metadata_path", default_value=""),
            DeclareLaunchArgument("vehicle_model_path", default_value=""),
            DeclareLaunchArgument(
                "wrench_gain",
                default_value="1.0",
                description=(
                    "실험 A1: 정책 출력 배율 (0, 1]. 0.5 로 주행해 떨림이 "
                    "사라지면 지연+세기 기전(위상 예산), 남으면 deadband/"
                    "chatter. 해법이 아니라 진단이다 -- 추종률을 깎는다."
                ),
            ),
            # ── 안전 기본값. 둘 다 명시적으로 올려야 추력이 나간다 ──
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"]
            ),
            # ── 수조 기하 (frame:=marker 일 때 pool 좌표, Z-up [m]) ──
            DeclareLaunchArgument(
                "leg_m",
                default_value="2.5",
                description="왕복 직선 길이. 두 프레임 모두에서 쓰인다.",
            ),
            DeclareLaunchArgument(
                "start_x_m",
                default_value="0.60",
                description=(
                    "marker 프레임 출발점의 수조 x. 기본값 + leg 2.5 m = 3.10 m "
                    "로 벽(3.65 m)까지 0.55 m 남는다."
                ),
            ),
            DeclareLaunchArgument(
                "lane_y_m",
                default_value="0.85",
                description="marker 프레임 차선 y. 안전 영역 0.30~1.40 m 의 중앙.",
            ),
            DeclareLaunchArgument(
                "target_pool_z_m",
                default_value="0.70",
                description=(
                    "marker 프레임 주행 깊이(바닥 기준 높이). 안전 영역 "
                    "0.20~0.90 m. 시작 깊이와 다르면 그 자체가 takeoff 가 된다."
                ),
            ),
            DeclareLaunchArgument(
                "rise_m",
                default_value="0.0",
                description=(
                    "start_heading 전용. start 깊이에서 몇 m 띄워 주행할지 "
                    "(0~0.6). 바닥에서 시작하면 0.4 -- 정책이 0.4 m 올린 뒤 그 "
                    "깊이로 왕복한다. marker 프레임에서는 무시된다."
                ),
            ),
            DeclareLaunchArgument(
                "heading_mode",
                default_value="align",
                choices=["align", "straight", "upright"],
                description=(
                    "align: 경로 방향으로 기수를 맞춘다 — 왕복마다 180° 선회가 "
                    "강제된다. straight: start 순간의 yaw 를 고정한 채 앞뒤로 "
                    "병진한다(선회 없음). 2026-09-03 실기에서 align 의 반복 선회가 "
                    "|ω| RMS 를 0.9~1.7 rad/s 로 올렸고, A50 이 빔 3/4 로 도는 "
                    "상태라 DVL bottom lock 이 흔들려 EKF 위치가 수조 밖으로 "
                    "드리프트했다 — 지연/진동 실험은 straight 로 하는 편이 "
                    "surge 축만 남아 깨끗하고 DVL 에도 쉽다."
                ),
            ),
            DeclareLaunchArgument(
                "cruise_speed",
                default_value="0.25",
                description=(
                    "정책 관측 16-D 에 절대 속도가 없어 이득이 속도와 무관하다 — "
                    "좁은 수조에서는 낮은 쪽이 낫다. 근거는 pool_mission.launch.py."
                ),
            ),
            # ── 깊이 출처. 게이트 통과 전에는 mavlink_ekf 를 유지한다 ──
            DeclareLaunchArgument(
                "depth_source",
                default_value="mavlink_ekf",
                choices=["mavlink_ekf", "pressure"],
                description=(
                    "docs/REAL_ROBOT_SESSION.md 1단계 게이트를 통과한 뒤에만 "
                    "pressure 로 넘긴다. 어느 쪽이든 두 경로의 원시값이 bag 에 "
                    "함께 남는다."
                ),
            ),
            # ── 기록 ──
            DeclareLaunchArgument(
                "record_bag",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "지연도 센서 편차도 사후에 다시 잴 수 없다. 기본으로 켠다."
                ),
            ),
            DeclareLaunchArgument("bag_path", default_value="pool_demo_a"),
            # ── DVL (기록 전용, 되먹임 아님) ──
            DeclareLaunchArgument(
                "dvl",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "A50 을 붙여 `/brov/dvl/sample` 로 기록한다. **제어 경로가 "
                    "아니다.** 기본 off: 2026-09-02 실기에서 이 노드가 붙은 뒤 "
                    "BlueOS DVL extension 의 VISION_POSITION_DELTA 가 멈췄고 5 s "
                    "뒤 EKF 가 CONST_POS_MODE 로 떨어져 LOCAL_POSITION_NED 가 "
                    "끊겼다 -- 정책이 위치를 잃는다. 켜려면 extension 이 두 "
                    "클라이언트를 받는지 먼저 확인하고, 켠 뒤 "
                    "`/brov/state.valid` 와 EKF 플래그를 볼 것."
                ),
            ),
            DeclareLaunchArgument("dvl_host", default_value="192.168.2.95"),
            # ── 마커 파이프라인 설정 ──
            DeclareLaunchArgument(
                "camera_config",
                default_value=os.path.join(
                    perception_share, "config", "camera.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "aruco_config",
                default_value=os.path.join(
                    perception_share, "config", "aruco.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "localization_config",
                default_value=os.path.join(
                    localization_share, "config", "localization.yaml"
                ),
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            OpaqueFunction(function=_compose),
        ]
    )
