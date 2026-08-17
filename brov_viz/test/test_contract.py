from pathlib import Path

import pytest
import yaml

from brov_viz.launch_contract import load_marker_survey


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent


def test_launch_uses_the_perception_marker_survey_as_single_source() -> None:
    survey = load_marker_survey(
        REPOSITORY_ROOT / "brov_perception" / "config" / "aruco.yaml"
    )
    assert survey == {
        "pool_frame": "pool",
        "marker_xyz": pytest.approx([3.95, 0.85, 0.35]),
        "marker_quaternion_xyzw": pytest.approx([-0.5, -0.5, 0.5, 0.5]),
        "marker_size_m": pytest.approx(0.42),
        "marker_label": "APRILTAG_16h5 ID 2",
    }


def test_rviz_is_pool_only_and_uses_expiring_vision_markers() -> None:
    rviz = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "pool_vision.rviz").read_text(
            encoding="utf-8"
        )
    )
    manager = rviz["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "pool"
    transformer = manager["Transformation"]["Current"]["Class"]
    assert transformer == "rviz_common/Identity"
    topics = {
        display.get("Topic", {}).get("Value")
        for display in manager["Displays"]
    }
    assert "/brov/viz/pool" in topics
    assert "/brov/viz/vision_robot" in topics
    assert "/brov/viz/localized_robot" in topics
    assert "/brov/aruco/debug_image" in topics
    assert not any(
        display.get("Class")
        in {
            "rviz_default_plugins/TF",
            "rviz_default_plugins/RobotModel",
        }
        for display in manager["Displays"]
    )


def test_visualization_package_has_no_tf_or_control_authority() -> None:
    source = (PACKAGE_ROOT / "brov_viz" / "pool_scene_node.py").read_text(
        encoding="utf-8"
    )
    launch = (PACKAGE_ROOT / "launch" / "pool_vision.launch.py").read_text(
        encoding="utf-8"
    )
    assert "TransformBroadcaster" not in source
    assert "tf2_ros" not in source
    assert "/brov/thruster_pwm" not in source
    assert "/brov/start_control" not in source
    assert "camera_stream_node" not in launch
    assert "obs_node" not in launch
    assert "policy_node" not in launch
