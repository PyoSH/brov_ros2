"""Gazebo/SITL launch for the CMG hover policy.

Mirrors ``brov_bringup/launch/sim2sim_mk2_case_a.launch.py``'s structure,
but the CMG controller needs no mission/guidance stack -- its hover
target is self-latched from odometry at the control-active edge, and it
never subscribes to ``/brov/observation`` or ``/brov/target_waypoint``.
``obs_node`` still requires *some* in-bounds mission file to boot its
internal guidance object, so this reuses the existing Case-A sim2sim
mission (known valid for this Gazebo world's spawn bounds) purely for
that reason; its waypoints are otherwise irrelevant to this controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_GZ_ODOMETRY = "/model/bluerov2_heavy/odometry"
_ROS_GT_ODOMETRY = "/brov/sim/gazebo_odometry_raw"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    cmg_share = get_package_share_directory("cmg_deploy")
    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")

    feedback_source = LaunchConfiguration("feedback_source")
    connection = LaunchConfiguration("connection")
    policy_path = LaunchConfiguration("policy_path")
    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")
    start_bridge = LaunchConfiguration("start_gazebo_truth_bridge")
    cmg_state_source = LaunchConfiguration("cmg_state_source")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "feedback_source",
                default_value="mavlink_ekf",
                choices=["mavlink_ekf", "gazebo_truth"],
            ),
            DeclareLaunchArgument(
                "connection", default_value="udpin:0.0.0.0:14552"
            ),
            DeclareLaunchArgument(
                "policy_path",
                default_value=os.environ.get(
                    "CMG_POLICY_PATH",
                    os.path.join(
                        os.path.dirname(bringup_share.rstrip("/")),
                        "artifacts",
                        "policies",
                        "cmg_hover_targeted_dr1",
                        "policy.pt",
                    ),
                ),
                description=(
                    "OBS17/ACTION8 TorchScript policy; defaults to "
                    "CMG_POLICY_PATH or the checked-in "
                    "cmg_hover_targeted_dr1 bundle."
                ),
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "start_gazebo_truth_bridge",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "cmg_state_source",
                default_value="mavlink_ekf",
                choices=["mavlink_ekf", "gazebo_truth_diagnostic"],
                description=(
                    "cmg_policy_node's own state input, independent of "
                    "feedback_source above (which only affects obs_node's "
                    "16-D/debug pipeline, not what cmg_deploy reads). "
                    "gazebo_truth_diagnostic isolates the policy from "
                    "MAVLink/EKF quality for sim-only diagnosis; it is "
                    "auto-acknowledged as sim-only by this launch file "
                    "specifically, since this launch never runs on the "
                    "real vehicle."
                ),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="cmg_gazebo_truth_bridge",
                output="screen",
                arguments=[
                    f"{_GZ_ODOMETRY}@nav_msgs/msg/Odometry[gz.msgs.Odometry"
                ],
                remappings=[(_GZ_ODOMETRY, _ROS_GT_ODOMETRY)],
                condition=IfCondition(start_bridge),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "vehicle_config": os.path.join(
                        base_share, "config", "vehicle_sitl.yaml"
                    ),
                    "mission_file": os.path.join(
                        bringup_share,
                        "config",
                        "mission_sim2sim_mk2_case_a_0p5.yaml",
                    ),
                    "safety_config": os.path.join(
                        base_share, "config", "safety.yaml"
                    ),
                    "connection": connection,
                    "feedback_source": feedback_source,
                    "gazebo_truth_logging_enabled": "true",
                    "gazebo_truth_topic": _ROS_GT_ODOMETRY,
                    "gazebo_truth_max_age_s": "0.25",
                    "send_pwm": send_pwm,
                    "arm": arm,
                }.items(),
            ),
            Node(
                package="cmg_deploy",
                executable="cmg_policy_node",
                name="cmg_policy_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    os.path.join(cmg_share, "config", "cmg_deploy_sim2sim.yaml"),
                    {
                        "policy_path": ParameterValue(policy_path, value_type=str),
                        "state_source": ParameterValue(
                            cmg_state_source, value_type=str
                        ),
                        # Safe to always acknowledge here: this launch file
                        # only ever targets Gazebo SITL.
                        "i_understand_gazebo_truth_is_sim_only": True,
                    },
                ],
            ),
        ]
    )
