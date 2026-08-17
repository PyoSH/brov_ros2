"""Launch the RViz-only pool scene without camera, control, or TF owners."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from brov_viz.launch_contract import load_marker_survey


def _launch_setup(context):
    survey = load_marker_survey(
        LaunchConfiguration("aruco_config").perform(context)
    )
    return [
        Node(
            package="brov_viz",
            executable="pool_scene_node",
            name="brov_pool_scene",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("pool_viz_config"),
                survey,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="brov_pool_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    viz_share = get_package_share_directory("brov_viz")
    perception_share = get_package_share_directory("brov_perception")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "aruco_config",
                default_value=os.path.join(
                    perception_share, "config", "aruco.yaml"
                ),
                description="Single source for surveyed AprilTag geometry",
            ),
            DeclareLaunchArgument(
                "pool_viz_config",
                default_value=os.path.join(
                    viz_share, "config", "pool_viz.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    viz_share, "rviz", "pool_vision.rviz"
                ),
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", choices=["true", "false"]
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
