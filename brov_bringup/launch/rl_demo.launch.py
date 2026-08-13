"""Bring up observation and one TorchScript policy controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")
    policy_default = os.environ.get("BROV_POLICY_PATH", "")

    args = {
        name: LaunchConfiguration(name)
        for name in (
            "vehicle_config", "mission_file", "safety_config", "connection",
            "send_pwm", "arm",
        )
    }
    controller_config = LaunchConfiguration("controller_config")
    policy_path = LaunchConfiguration("policy_path")

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
                "controller_config",
                default_value=os.path.join(
                    control_share, "config", "rl_controller.yaml"
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
                "policy_path",
                default_value=policy_default,
                description="TorchScript policy; defaults to BROV_POLICY_PATH.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments=args.items(),
            ),
            Node(
                package="brov_control",
                executable="policy_node",
                name="brov_policy_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    controller_config,
                    {"policy_path": ParameterValue(policy_path, value_type=str)},
                ],
            ),
        ]
    )
