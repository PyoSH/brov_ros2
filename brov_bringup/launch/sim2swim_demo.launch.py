"""Safe-default real-vehicle bringup for Sim2Swim demo cases (a) and (c).

The selected mission is composed with the RL branch of sim2real_demo.launch.py.
No control service is called by this launch; PWM and arming also default off.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _include_selected_case(context):
    bringup_share = get_package_share_directory("brov_bringup")
    selected_case = (
        LaunchConfiguration("case").perform(context).strip().lower()
    )
    mission_names = {
        "a": "mission_sim2swim_a.yaml",
        "c": "mission_sim2swim_c.yaml",
    }
    if selected_case not in mission_names:
        raise RuntimeError("case must be exactly one of: a, c")
    if selected_case == "c" and not _is_true(
        LaunchConfiguration("allow_case_c").perform(context)
    ):
        raise RuntimeError(
            "case c is fail-closed; pass allow_case_c:=true only after "
            "reading "
            "docs/SIM2SWIM_DEMO.md and satisfying its staged-acceptance gate"
        )

    forwarded_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "vehicle_config",
            "safety_config",
            "rl_config",
            "camera_config",
            "aruco_config",
            "connection",
            "send_pwm",
            "arm",
            "policy_path",
            "camera",
            "aruco",
            "udp_port",
            "camera_info_path",
        )
    }
    forwarded_arguments.update(
        {
            "controller": "rl",
            "mission_file": os.path.join(
                bringup_share, "config", mission_names[selected_case]
            ),
        }
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share, "launch", "sim2real_demo.launch.py"
                )
            ),
            launch_arguments=forwarded_arguments.items(),
        )
    ]


def _default_camera_info_path() -> str:
    data_dir = os.environ.get(
        "BROV_DATA_DIR", os.path.expanduser("~/.ros/brov")
    )
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    perception_share = get_package_share_directory("brov_perception")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "case",
                default_value="a",
                choices=["a", "c"],
                description="Sim2Swim real demo case: (a) line or (c) square.",
            ),
            DeclareLaunchArgument(
                "allow_case_c",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Explicit hazard acknowledgement required for "
                    "random-attitude case c."
                ),
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=os.path.join(
                    base_share, "config", "vehicle.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=os.path.join(
                    base_share, "config", "safety.yaml"
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
                "policy_path",
                default_value=os.environ.get("BROV_POLICY_PATH", ""),
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
            OpaqueFunction(function=_include_selected_case),
        ]
    )
