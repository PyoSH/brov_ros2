"""Bring up MAVLink and observation with safe output defaults."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate_output_options(context):
    send_pwm = _is_true(LaunchConfiguration("send_pwm").perform(context))
    arm = _is_true(LaunchConfiguration("arm").perform(context))
    if arm and not send_pwm:
        raise RuntimeError("arm:=true requires send_pwm:=true")
    return []


def generate_launch_description() -> LaunchDescription:
    base_share = get_package_share_directory("brov_base")
    bringup_share = get_package_share_directory("brov_bringup")

    vehicle_config = LaunchConfiguration("vehicle_config")
    mission_file = LaunchConfiguration("mission_file")
    safety_config = LaunchConfiguration("safety_config")
    connection = LaunchConfiguration("connection")
    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")
    require_pool_localization = LaunchConfiguration(
        "require_pool_localization"
    )
    require_resolved_mission = LaunchConfiguration(
        "require_resolved_mission"
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
                "connection", default_value="udpout:192.168.2.2:14550"
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Permit the explicit arm_control service; launch, prepare, "
                    "and start never arm automatically."
                ),
            ),
            DeclareLaunchArgument(
                "require_pool_localization",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Reject control start unless the current odometry session "
                    "has an initialized pool-frame alignment."
                ),
            ),
            DeclareLaunchArgument(
                "require_resolved_mission",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Reject control start unless an immutable mission resolved "
                    "for the current localization epoch is available."
                ),
            ),
            OpaqueFunction(function=_validate_output_options),
            Node(
                package="brov_base",
                executable="obs_node",
                name="brov_obs_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    vehicle_config,
                    mission_file,
                    safety_config,
                    {
                        "connection": ParameterValue(connection, value_type=str),
                        "send_pwm": ParameterValue(send_pwm, value_type=bool),
                        "arm": ParameterValue(arm, value_type=bool),
                        "require_pool_localization": ParameterValue(
                            require_pool_localization, value_type=bool
                        ),
                        "require_resolved_mission": ParameterValue(
                            require_resolved_mission, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
