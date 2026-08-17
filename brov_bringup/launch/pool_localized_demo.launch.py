"""Pool-localized bringup with fail-closed mission and control gates.

The launch composes data producers and exactly one controller. Operator
approval services remain explicit; the optional orchestrator calls them only
after an operator request. The launch construction never confirms camera tilt,
initializes alignment, commits a route, arms, or starts a controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_controller(context):
    controller = LaunchConfiguration("controller").perform(context).lower()
    if controller not in {"model", "rl", "rl_mk2"}:
        raise RuntimeError(
            "controller must be exactly one of: model, rl, rl_mk2"
        )
    return []


def _default_camera_info_path() -> str:
    data_dir = os.environ.get("BROV_DATA_DIR", os.path.expanduser("~/.ros/brov"))
    return os.path.join(data_dir, "calibration", "camera_intrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    perception_share = get_package_share_directory("brov_perception")
    localization_share = get_package_share_directory("brov_localization")
    mission_share = get_package_share_directory("brov_mission")
    viz_share = get_package_share_directory("brov_viz")

    controller = LaunchConfiguration("controller")
    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")
    connection = LaunchConfiguration("connection")
    policy_path = LaunchConfiguration("policy_path")
    udp_port = LaunchConfiguration("udp_port")
    camera_info_path = LaunchConfiguration("camera_info_path")
    rviz = LaunchConfiguration("rviz")
    demo_orchestrator = LaunchConfiguration("demo_orchestrator")
    model_selected = IfCondition(
        PythonExpression(["'", controller, "'.lower() == 'model'"])
    )
    rl_selected = IfCondition(
        PythonExpression(["'", controller, "'.lower() == 'rl'"])
    )
    rl_mk2_selected = IfCondition(
        PythonExpression(["'", controller, "'.lower() == 'rl_mk2'"])
    )

    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")
    viz_launch = os.path.join(viz_share, "launch", "pool_vision.launch.py")
    base_arguments = {
        "vehicle_config": LaunchConfiguration("vehicle_config"),
        "mission_file": LaunchConfiguration("mission_file"),
        "safety_config": LaunchConfiguration("safety_config"),
        "connection": connection,
        "send_pwm": send_pwm,
        "arm": arm,
        # These are deliberately not public launch switches in this profile.
        "require_pool_localization": "true",
        "require_resolved_mission": "true",
    }

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
                    "Legacy bootstrap mission; the committed pool mission "
                    "replaces it before the control gate can open."
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
                "mission_manager_config",
                default_value=os.path.join(
                    mission_share, "config", "mission_manager.yaml"
                ),
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
                "rl_mk2_config",
                default_value=os.path.join(
                    control_share, "config", "rl_controller_mk2_real_v1.yaml"
                ),
                description=(
                    "MK2 policy_node_mk2 parameters. Defaults to the "
                    "conservative first-water-test envelope -- do not point "
                    "this at rl_controller_mk2_deploy_v2.yaml (Gazebo-only, "
                    "no action/PWM limiting) for a real-vehicle run."
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
                "arm",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Permit the explicit arm_control service after prepare; "
                    "launch and start never arm automatically."
                ),
            ),
            DeclareLaunchArgument(
                "require_pool_localization",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized profile.",
            ),
            DeclareLaunchArgument(
                "require_resolved_mission",
                default_value="true",
                choices=["true"],
                description="Fixed on by the pool-localized profile.",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="model",
                choices=["model", "rl", "rl_mk2"],
            ),
            DeclareLaunchArgument(
                "demo_orchestrator",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Expose /brov/demo/{prepare,start,stop}; the node calls "
                    "existing fail-closed services and never acts at launch."
                ),
            ),
            DeclareLaunchArgument(
                "demo_case", default_value="a", choices=["a", "a2", "c"]
            ),
            DeclareLaunchArgument(
                "auto_generate_case_a_path",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Generate a short current-pose-anchored Case-A path in "
                    "/brov/demo/prepare; false uses an operator draft."
                ),
            ),
            DeclareLaunchArgument(
                "case_a_target_pool_z_m",
                default_value="0.70",
                description=(
                    "Pool-frame base_link height used by the automatic "
                    "Case-A takeoff prefix."
                ),
            ),
            DeclareLaunchArgument(
                "case_a_segment_length_m",
                default_value="2.0",
                description=(
                    "Horizontal P1<->P2 Case-A loop length in metres. Matches "
                    "the 2.0 m / 0.5 m/s Gazebo Case-A curriculum "
                    "deploy_v3/v4/v5 were trained on -- "
                    "mission_manager_sim2swim_a.yaml's own "
                    "max_segment_length_m=2.0 ceiling leaves no margin above "
                    "this default."
                ),
            ),
            DeclareLaunchArgument(
                "policy_path", default_value=os.environ.get("BROV_POLICY_PATH", "")
            ),
            DeclareLaunchArgument("udp_port", default_value="5600"),
            DeclareLaunchArgument(
                "camera_info_path", default_value=_default_camera_info_path()
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Optionally include visualization-only pool scene and RViz; "
                    "disabled by default."
                ),
            ),
            OpaqueFunction(function=_validate_controller),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments=base_arguments.items(),
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
                package="brov_mission",
                executable="mission_manager_node",
                name="brov_mission_manager",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("mission_manager_config")],
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
            Node(
                package="brov_control",
                executable="policy_node_mk2",
                name="brov_policy_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("rl_mk2_config"),
                    {"policy_path": ParameterValue(policy_path, value_type=str)},
                ],
                condition=rl_mk2_selected,
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
                package="brov_bringup",
                executable="demo_orchestrator_node",
                name="brov_demo_orchestrator",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "controller": ParameterValue(
                            controller, value_type=str
                        ),
                        "demo_case": ParameterValue(
                            LaunchConfiguration("demo_case"), value_type=str
                        ),
                        "auto_generate_case_a_path": ParameterValue(
                            LaunchConfiguration("auto_generate_case_a_path"),
                            value_type=bool,
                        ),
                        "case_a_target_pool_z_m": ParameterValue(
                            LaunchConfiguration("case_a_target_pool_z_m"),
                            value_type=float,
                        ),
                        "case_a_segment_length_m": ParameterValue(
                            LaunchConfiguration("case_a_segment_length_m"),
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(demo_orchestrator),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(viz_launch),
                launch_arguments={
                    "aruco_config": LaunchConfiguration("aruco_config"),
                    "rviz": "true",
                }.items(),
                condition=IfCondition(rviz),
            ),
        ]
    )
