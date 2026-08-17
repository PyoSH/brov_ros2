"""Edo SITL 0.5 m/s A/B launch with synchronized Gazebo ground truth.

The bridge is always present so both runs record the same truth topic.  Only
``feedback_source`` changes which state reaches guidance and the policy;
MAVLink continues to own health gates, arming, actuation, and local odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_GZ_ODOMETRY = "/model/bluerov2_heavy/odometry"
_ROS_GT_ODOMETRY = "/brov/sim/gazebo_odometry_raw"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("brov_bringup")
    base_share = get_package_share_directory("brov_base")
    control_share = get_package_share_directory("brov_control")
    rl_demo = os.path.join(bringup_share, "launch", "rl_demo.launch.py")

    feedback_source = LaunchConfiguration("feedback_source")
    connection = LaunchConfiguration("connection")
    policy_path = LaunchConfiguration("policy_path")
    send_pwm = LaunchConfiguration("send_pwm")
    arm = LaunchConfiguration("arm")

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
                default_value=os.environ.get("BROV_POLICY_PATH", ""),
            ),
            DeclareLaunchArgument(
                "send_pwm", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm", default_value="false", choices=["true", "false"]
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="brov_gazebo_truth_bridge",
                output="screen",
                arguments=[
                    f"{_GZ_ODOMETRY}@nav_msgs/msg/Odometry[gz.msgs.Odometry"
                ],
                remappings=[(_GZ_ODOMETRY, _ROS_GT_ODOMETRY)],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rl_demo),
                launch_arguments={
                    "vehicle_config": os.path.join(
                        base_share, "config", "vehicle_sitl.yaml"
                    ),
                    "mission_file": os.path.join(
                        bringup_share,
                        "config",
                        "mission_sim2sim_straight_0p5.yaml",
                    ),
                    "safety_config": os.path.join(
                        base_share, "config", "safety.yaml"
                    ),
                    "controller_config": os.path.join(
                        control_share, "config", "rl_controller.yaml"
                    ),
                    "connection": connection,
                    "feedback_source": feedback_source,
                    "gazebo_truth_logging_enabled": "true",
                    "gazebo_truth_topic": _ROS_GT_ODOMETRY,
                    "gazebo_truth_max_age_s": "0.12",
                    "policy_path": policy_path,
                    "send_pwm": send_pwm,
                    "arm": arm,
                }.items(),
            ),
        ]
    )
