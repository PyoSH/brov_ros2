"""Bring up the BlueOS H264 camera and optional perception/calibration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate_modes(context):
    aruco = _is_true(LaunchConfiguration("aruco").perform(context))
    calibrate = _is_true(LaunchConfiguration("calibrate").perform(context))
    if aruco and calibrate:
        raise RuntimeError("aruco:=true and calibrate:=true are mutually exclusive")
    return []


def _default_camera_info_path() -> str:
    data_dir = os.environ.get(
        "BROV_DATA_DIR", os.path.expanduser("~/.ros/brov")
    )
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory("brov_perception")
    camera_config = LaunchConfiguration("camera_config")
    aruco_config = LaunchConfiguration("aruco_config")
    checkerboard_config = LaunchConfiguration("checkerboard_config")
    udp_port = LaunchConfiguration("udp_port")
    camera_info_path = LaunchConfiguration("camera_info_path")
    aruco = LaunchConfiguration("aruco")
    calibrate = LaunchConfiguration("calibrate")

    return LaunchDescription(
        [
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
                "checkerboard_config",
                default_value=os.path.join(
                    perception_share, "config", "checkerboard.yaml"
                ),
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            DeclareLaunchArgument(
                "aruco", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "calibrate", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=_validate_modes),
            Node(
                package="brov_perception",
                executable="camera_stream_node",
                name="brov_camera_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    camera_config,
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
                parameters=[aruco_config],
                condition=IfCondition(aruco),
            ),
            Node(
                package="brov_perception",
                executable="checkerboard_calibration_node",
                name="brov_checkerboard_calibration",
                output="screen",
                emulate_tty=True,
                parameters=[
                    checkerboard_config,
                    {
                        "output_path": ParameterValue(
                            camera_info_path, value_type=str
                        )
                    },
                ],
                condition=IfCondition(calibrate),
            ),
        ]
    )
