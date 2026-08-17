"""Fresh Gazebo/SITL Case-A launch for the classical model-based controller.

Mirrors ``sim2sim_mk2_case_a.launch.py`` (same mission/vehicle/safety
profile, same Gazebo-truth bridge) but swaps ``policy_node_mk2`` for
``model_based_controller_node`` so the identical Case-A cycle can be used as
a causal-isolation baseline: if the classical PI/PD controller does not
jitter on the same plant/allocation/actuator chain, the jitter is a
policy problem, not a plant problem.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_GZ_ODOMETRY = "/model/bluerov2_heavy/odometry"
_ROS_GT_ODOMETRY = "/brov/sim/gazebo_odometry_raw"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    base_launch = os.path.join(bringup_share, "launch", "base.launch.py")

    feedback_source = LaunchConfiguration("feedback_source")
    connection = LaunchConfiguration("connection")
    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")
    start_bridge = LaunchConfiguration("start_gazebo_truth_bridge")

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
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="brov_model_based_gazebo_truth_bridge",
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
                package="brov_control",
                executable="model_based_controller_node",
                name="brov_model_based_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    os.path.join(
                        control_share, "config", "model_controller.yaml"
                    ),
                ],
            ),
        ]
    )
