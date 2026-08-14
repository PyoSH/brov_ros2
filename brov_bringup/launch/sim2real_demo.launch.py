"""
Full bringup with exactly one controller and optional camera/ArUco.

This launch never starts control services. The operator must inspect telemetry
and explicitly call the relevant start services.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate_options(context):
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    if controller not in {"model", "rl"}:
        raise RuntimeError("controller must be exactly one of: model, rl")
    if _is_true(LaunchConfiguration("aruco").perform(context)) and not _is_true(
        LaunchConfiguration("camera").perform(context)
    ):
        raise RuntimeError("aruco:=true requires camera:=true")
    return []


def _default_camera_info_path() -> str:
    data_dir = os.environ.get(
        "BROV_DATA_DIR", os.path.expanduser("~/.ros/brov")
    )
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    perception_share = get_package_share_directory("brov_perception")
    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")

    configs = {
        "vehicle_config": LaunchConfiguration("vehicle_config"),
        "mission_file": LaunchConfiguration("mission_file"),
        "safety_config": LaunchConfiguration("safety_config"),
        "connection": LaunchConfiguration("connection"),
        "send_pwm": LaunchConfiguration("send_pwm"),
        "arm": LaunchConfiguration("arm"),
        "require_pool_localization": LaunchConfiguration(
            "require_pool_localization"
        ),
        "require_resolved_mission": LaunchConfiguration(
            "require_resolved_mission"
        ),
    }
    controller = LaunchConfiguration("controller")
    camera = LaunchConfiguration("camera")
    aruco = LaunchConfiguration("aruco")
    policy_path = LaunchConfiguration("policy_path")
    camera_info_path = LaunchConfiguration("camera_info_path")
    udp_port = LaunchConfiguration("udp_port")
    model_selected = IfCondition(
        PythonExpression(["'", controller, "'.lower() == 'model'"])
    )
    rl_selected = IfCondition(
        PythonExpression(["'", controller, "'.lower() == 'rl'"])
    )

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
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=os.path.join(base_share, "config", "safety.yaml"),
            ),
            DeclareLaunchArgument(
                "model_config",
                default_value=os.path.join(
                    control_share, "config", "model_controller.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rl_config",
                default_value=os.path.join(
                    control_share, "config", "rl_controller.yaml"
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
                "connection", default_value="udpout:192.168.2.2:14550"
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "require_pool_localization",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Require a valid one-shot pool alignment before control."
                ),
            ),
            DeclareLaunchArgument(
                "require_resolved_mission",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Require an immutable pool mission before control."
                ),
            ),
            DeclareLaunchArgument(
                "controller", default_value="model", choices=["model", "rl"]
            ),
            DeclareLaunchArgument(
                "policy_path", default_value=os.environ.get("BROV_POLICY_PATH", "")
            ),
            DeclareLaunchArgument(
                "camera", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "aruco", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            OpaqueFunction(function=_validate_options),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments=configs.items(),
            ),
            Node(
                package="brov_control",
                executable="model_based_controller_node",
                name="brov_model_based_controller",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("model_config")],
                condition=model_selected,
            ),
            Node(
                package="brov_control",
                executable="policy_node",
                name="brov_policy_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("rl_config"),
                    {"policy_path": ParameterValue(policy_path, value_type=str)},
                ],
                condition=rl_selected,
            ),
            GroupAction(
                condition=IfCondition(camera),
                actions=[
                    Node(
                        package="brov_perception",
                        executable="camera_stream_node",
                        name="brov_camera_node",
                        output="screen",
                        emulate_tty=True,
                        parameters=[
                            LaunchConfiguration("camera_config"),
                            {
                                "udp_port": ParameterValue(
                                    udp_port, value_type=int
                                ),
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
                        condition=IfCondition(aruco),
                    ),
                ],
            ),
        ]
    )
