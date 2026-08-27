"""실기 surge 항력 측정 — pool 정렬 + open-loop surge.

``pool_localized_demo.launch.py``와 같은 데이터 생산자(카메라/ArUco/pool 정렬)를
쓰되 **미션 스택과 경로추종 제어기를 띄우지 않는다.**

두 가지가 다르다:

1. ``require_resolved_mission:=false``. 항력시험은 웨이포인트 미션이 아니다.
   ``obs_node``의 ``_resolved_mission_gate``/``_prepared_gate``는
   ``if self._require_resolved_mission:`` 안에만 있으므로, pool 정렬 게이트는
   그대로 걸리면서 ``prepare_control`` 없이 arm→start가 된다.
   (``prepare_control``은 resolved mission 전용이라 호출하면 실패한다.)

2. ``mission_manager_node``와 ``model_based_controller_node``를 띄우지 않는다.
   ``obs_node._authority_gate``가 ``/brov/thruster_pwm``의 발행자가 정확히
   하나일 것을 요구하는데, 그 하나가 ``drag_test_node``여야 한다.

``base.launch.py``가 부팅에 ``mission_file``을 요구하므로 인자는 넘기지만
내용은 쓰이지 않는다 — resolved mission 경로가 꺼져 있고 이 노드는
``/brov/observation``을 구독하지 않는다.

운용::

    ros2 launch brov_bringup drag_test.launch.py send_pwm:=true arm:=true
    ros2 service call /brov/drag_test/prepare std_srvs/srv/Trigger   # 정지 상태로
    ros2 service call /brov/drag_test/start   std_srvs/srv/Trigger
    ros2 service call /brov/drag_test/stop    std_srvs/srv/Trigger   # 언제든
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_camera_info_path() -> str:
    data_dir = os.environ.get("BROV_DATA_DIR", os.path.expanduser("~/.ros/brov"))
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    perception_share = get_package_share_directory("brov_perception")
    localization_share = get_package_share_directory("brov_localization")

    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")
    connection = LaunchConfiguration("connection")
    udp_port = LaunchConfiguration("udp_port")
    camera_info_path = LaunchConfiguration("camera_info_path")

    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=os.path.join(base_share, "config", "vehicle.yaml"),
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=os.path.join(
                    bringup_share, "config", "mission_demo.yaml"
                ),
                description=(
                    "obs_node 부팅에만 필요하다. resolved mission 경로가 꺼져 "
                    "있고 drag_test_node는 /brov/observation을 쓰지 않으므로 "
                    "내용은 측정에 영향을 주지 않는다."
                ),
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=os.path.join(base_share, "config", "safety.yaml"),
            ),
            DeclareLaunchArgument(
                "localization_config",
                default_value=os.path.join(
                    localization_share, "config", "localization.yaml"
                ),
            ),
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
                "drag_test_config",
                default_value=os.path.join(
                    control_share, "config", "drag_test.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "connection", default_value="udpout:192.168.2.2:14550",
                description=(
                    "BlueOS는 14550에서 UDP 서버로 동작한다 — udpin:은 기체가 "
                    "먼저 보내주기를 기다리는 방식이라 닿지 않는다."
                ),
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"],
                description="false면 진단 토픽만 내고 추력은 내지 않는다.",
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"],
                description="arm_control 서비스를 허용한다. launch는 절대 스스로 arm하지 않는다.",
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "vehicle_config": LaunchConfiguration("vehicle_config"),
                    "mission_file": LaunchConfiguration("mission_file"),
                    "safety_config": LaunchConfiguration("safety_config"),
                    "connection": connection,
                    "send_pwm": send_pwm,
                    "arm": arm,
                    # pool 절대위치는 쓰되(벽까지 남은 거리·차선 유지·주행거리),
                    # 미션 스택은 쓰지 않는다.
                    "require_pool_localization": "true",
                    "require_resolved_mission": "false",
                }.items(),
            ),
            Node(
                package="brov_localization",
                executable="pool_alignment_node",
                name="brov_pool_alignment",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("localization_config")],
            ),
            Node(
                package="brov_perception",
                executable="camera_stream_node",
                name="brov_camera_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("camera_config"),
                    {
                        "udp_port": ParameterValue(udp_port, value_type=int),
                        "camera_info_path": ParameterValue(
                            camera_info_path, value_type=str
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
                parameters=[LaunchConfiguration("aruco_config")],
            ),
            Node(
                package="brov_control",
                executable="drag_test_node",
                name="brov_drag_test",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("drag_test_config"),
                    {"send_pwm": ParameterValue(send_pwm, value_type=bool)},
                ],
            ),
        ]
    )
