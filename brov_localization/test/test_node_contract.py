from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent


def test_configuration_declares_required_topics_and_safety_gates() -> None:
    configuration = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "localization.yaml").read_text(encoding="utf-8")
    )["brov_pool_alignment"]["ros__parameters"]
    assert (
        configuration["odometry_session_topic"]
        == "/brov/odometry/local_with_session"
    )
    assert "odom_topic" not in configuration
    assert "session_topic" not in configuration
    assert configuration["vision_topic"] == "/brov/aruco/robot_pose_pool"
    assert configuration["visible_topic"] == "/brov/aruco/visible"
    assert (
        configuration["aligned_odometry_topic"]
        == "/brov/localization/odometry_pool_with_alignment"
    )
    assert configuration["pool_odometry_topic"] == "/brov/localization/odometry_pool"
    assert configuration["pool_frame"] == "pool"
    assert configuration["odom_frame"] == "odom"
    assert configuration["base_frame"] == "base_link"
    assert configuration["require_camera_tilt_neutral_confirmation"] is True
    for positive_name in (
        "default_min_samples",
        "max_buffer_samples",
        "sample_retention_s",
        "max_message_age_s",
        "visible_timeout_s",
        "max_timestamp_skew_s",
        "stationary_linear_speed_mps",
        "stationary_angular_speed_rad_s",
        "max_translation_residual_m",
        "max_rotation_residual_deg",
        "max_abs_alignment_roll_deg",
        "max_abs_alignment_pitch_deg",
    ):
        assert configuration[positive_name] > 0


def test_node_has_localization_authority_but_no_mavlink_or_control_authority() -> None:
    source = (
        PACKAGE_ROOT / "brov_localization" / "localization_node.py"
    ).read_text(encoding="utf-8")
    for required in (
        "TransformBroadcaster",
        '"/brov/localization/initialize_pool"',
        '"/brov/localization/reset"',
        '"/brov/localization/confirm_camera_tilt_neutral"',
        "LocalizationStatus",
        "AlignedOdometry",
        "OdometrySession",
        "odometry_session_topic",
        "InitializePool",
        "make_alignment_sample",
        "pool_odometry_topic",
        "aligned_odometry_topic",
    ):
        assert required in source
    for forbidden in (
        "pymavlink",
        "RealRobotInterface",
        "/brov/thruster_pwm",
        "/brov/start_control",
        "RC_CHANNELS_OVERRIDE",
        "self._on_session",
    ):
        assert forbidden not in source


def test_package_declares_every_runtime_dependency() -> None:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert {
        "brov_interfaces",
        "geometry_msgs",
        "nav_msgs",
        "python3-numpy",
        "rclpy",
        "std_msgs",
        "std_srvs",
        "tf2_ros",
    } <= dependencies


def test_custom_interface_contract_is_the_expected_version() -> None:
    status = (
        REPOSITORY_ROOT / "brov_interfaces" / "msg" / "LocalizationStatus.msg"
    ).read_text(encoding="utf-8")
    service = (
        REPOSITORY_ROOT / "brov_interfaces" / "srv" / "InitializePool.srv"
    ).read_text(encoding="utf-8")
    odometry_session = (
        REPOSITORY_ROOT / "brov_interfaces" / "msg" / "OdometrySession.msg"
    ).read_text(encoding="utf-8")
    aligned_odometry = (
        REPOSITORY_ROOT / "brov_interfaces" / "msg" / "AlignedOdometry.msg"
    ).read_text(encoding="utf-8")
    for field in (
        "uint8 UNINITIALIZED=0",
        "uint8 COLLECTING=1",
        "uint8 INITIALIZED=2",
        "uint8 INVALID=3",
        "uint64 epoch",
        "string odometry_session_id",
        "string alignment_id",
        "geometry_msgs/Transform pool_to_odom",
        "bool output_valid",
        "uint32 sample_count",
        "string reason",
    ):
        assert field in status
    assert "uint32 min_samples" in service
    assert "bool success" in service
    assert "string message" in service
    assert "uint64 epoch" in service
    assert "nav_msgs/Odometry odometry" in odometry_session
    assert "string odometry_session_id" in odometry_session
    for field in (
        "nav_msgs/Odometry odometry",
        "uint64 localization_epoch",
        "string odometry_session_id",
        "string alignment_id",
    ):
        assert field in aligned_odometry
    resolved = (
        REPOSITORY_ROOT / "brov_interfaces" / "msg" / "ResolvedMission.msg"
    ).read_text(encoding="utf-8")
    for field in (
        "string plan_hash",
        "string contract_version",
        "string canonical_plan_json",
        "uint64 localization_epoch",
        "string odometry_session_id",
        "string alignment_id",
    ):
        assert field in resolved


def test_readme_documents_exact_transform_and_one_shot_behavior() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    normalized = " ".join(readme.split())
    assert "pool_T_odom[i] = pool_T_base[vision, i] * inverse(odom_T_base[odom, i])" in readme
    assert "Once initialized, `pool_T_odom` is frozen" in readme
    assert "camera capture time" in normalized
    assert "pool --(this package)--> odom" in readme
    assert "/brov/localization/confirm_camera_tilt_neutral" in readme
    assert "positive request below `default_min_samples` is" in normalized
