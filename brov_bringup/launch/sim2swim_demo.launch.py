"""Pool-localized Sim2Swim bringup for the RL case profiles.

The wrapper selects a case-specific, versioned mission-manager profile and
composes the RL branch of ``pool_localized_demo.launch.py``.  Camera, AprilTag,
one-shot full-SE(3) pool alignment, and an immutable resolved pool mission are
therefore mandatory.  Launch construction never confirms camera tilt,
initializes localization, commits a route, prepares control, arms, or starts
the policy output gate.
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


_CASE_PROFILES = {
    "a": {
        "bootstrap": "mission_sim2swim_a.yaml",
        "mission_manager": "mission_manager_sim2swim_a.yaml",
        "rl_package": "brov_control",
        "rl_config": "rl_controller.yaml",
    },
    "c": {
        "bootstrap": "mission_sim2swim_bootstrap.yaml",
        "mission_manager": "mission_manager_sim2swim_c.yaml",
        "safety": "safety_sim2swim_c.yaml",
        "rl_package": "brov_bringup",
        "rl_config": "rl_controller_sim2swim_c.yaml",
    },
}


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _include_selected_case(context):
    bringup_share = get_package_share_directory("brov_bringup")
    control_share = get_package_share_directory("brov_control")
    selected_case = (
        LaunchConfiguration("case").perform(context).strip().lower()
    )
    if selected_case not in _CASE_PROFILES:
        raise RuntimeError("case must be exactly one of: a, c")
    if selected_case == "c" and not _is_true(
        LaunchConfiguration("allow_case_c").perform(context)
    ):
        raise RuntimeError(
            "case c is fail-closed; pass allow_case_c:=true only after "
            "reading docs/SIM2SWIM_DEMO.md and satisfying its staged "
            "random-attitude acceptance gate"
        )

    profile = _CASE_PROFILES[selected_case]
    forwarded_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "vehicle_config",
            "localization_config",
            "camera_config",
            "aruco_config",
            "connection",
            "send_pwm",
            "arm",
            "policy_path",
            "udp_port",
            "camera_info_path",
            "rviz",
            "demo_orchestrator",
            "case_a_target_pool_z_m",
            "case_a_segment_length_m",
            "require_pool_localization",
            "require_resolved_mission",
        )
    }
    forwarded_arguments.update(
        {
            "controller": "rl",
            "demo_case": selected_case,
            "auto_generate_case_a_path": (
                "true" if selected_case == "a" else "false"
            ),
            # These files provide only bootstrap frame/depth settings.  The
            # committed pool Path selected below replaces their waypoints and
            # guidance settings before PREPARE can succeed.
            "mission_file": os.path.join(
                bringup_share, "config", profile["bootstrap"]
            ),
            "mission_manager_config": os.path.join(
                bringup_share, "config", profile["mission_manager"]
            ),
            # Case C cannot replace its gateway envelope with the permissive
            # general safety profile.  Case A preserves the caller-selected
            # legacy safety file.
            "safety_config": (
                os.path.join(bringup_share, "config", profile["safety"])
                if selected_case == "c"
                else LaunchConfiguration("safety_config")
            ),
            "rl_config": os.path.join(
                (
                    control_share
                    if profile["rl_package"] == "brov_control"
                    else bringup_share
                ),
                "config",
                profile["rl_config"],
            ),
        }
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share, "launch", "pool_localized_demo.launch.py"
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
    perception_share = get_package_share_directory("brov_perception")
    localization_share = get_package_share_directory("brov_localization")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "case",
                default_value="a",
                choices=["a", "c"],
                description=(
                    "Resolved Sim2Swim profile: (a) takeoff+align loop or "
                    "(c) random-attitude-v2."
                ),
            ),
            DeclareLaunchArgument(
                "allow_case_c",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Explicit acknowledgement required before composing the "
                    "bounded random-attitude-v2 profile."
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
                "connection", default_value="udpout:192.168.2.2:14550"
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "demo_orchestrator",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Expose the three-operation Case-A demo API without "
                    "performing any operation during launch."
                ),
            ),
            DeclareLaunchArgument(
                "case_a_target_pool_z_m",
                default_value="0.70",
                description="Case-A takeoff target in pool-frame +Z metres.",
            ),
            DeclareLaunchArgument(
                "case_a_segment_length_m",
                default_value="0.20",
                description="Horizontal Case-A loop length in metres.",
            ),
            DeclareLaunchArgument(
                "policy_path",
                default_value=os.environ.get("BROV_POLICY_PATH", ""),
            ),
            # Compatibility-facing fixed arguments make attempts to disable a
            # required producer fail during launch argument validation.
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized Sim2Swim profile.",
            ),
            DeclareLaunchArgument(
                "aruco",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized Sim2Swim profile.",
            ),
            DeclareLaunchArgument(
                "require_pool_localization",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized Sim2Swim profile.",
            ),
            DeclareLaunchArgument(
                "require_resolved_mission",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized Sim2Swim profile.",
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                choices=["true", "false"],
                description="Optional visualization-only pool scene and RViz.",
            ),
            OpaqueFunction(function=_include_selected_case),
        ]
    )
