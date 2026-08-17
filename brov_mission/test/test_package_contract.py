from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_package_does_not_import_control_or_mavlink() -> None:
    source = (ROOT / "brov_mission" / "mission_manager_node.py").read_text()
    forbidden = (
        "RealRobotInterface",
        "pymavlink",
        "/brov/start_control",
        "/brov/stop_control",
        "/brov/thruster_pwm",
    )
    for token in forbidden:
        assert token not in source


def test_default_configuration_is_position_only_and_pool_bounded() -> None:
    document = yaml.safe_load(
        (ROOT / "config" / "mission_manager.yaml").read_text()
    )
    parameters = document["brov_mission_manager"]["ros__parameters"]
    assert parameters["pool_frame"] == "pool"
    assert parameters["odom_frame"] == "odom"
    assert (
        parameters["aligned_odometry_topic"]
        == "/brov/localization/odometry_pool_with_alignment"
    )
    assert parameters["orientation_support_enabled"] is False
    assert (
        parameters["contract_version"]
        == "brov_pool_position_mission_v1"
    )
    assert parameters["allowed_heading_modes"] == ["straight", "align"]
    assert len(parameters["pool_safe_min_xyz"]) == 3
    assert len(parameters["pool_safe_max_xyz"]) == 3
    assert all(
        lower < upper
        for lower, upper in zip(
            parameters["pool_safe_min_xyz"],
            parameters["pool_safe_max_xyz"],
        )
    )


def test_resolved_mission_contract_fields_are_used() -> None:
    source = (ROOT / "brov_mission" / "mission_manager_node.py").read_text()
    for field in (
        "mission_id",
        "plan_hash",
        "contract_version",
        "canonical_plan_json",
        "localization_epoch",
        "odometry_session_id",
        "alignment_id",
        "waypoints",
        "cruise_speed",
        "lookahead_dist",
        "reach_threshold",
        "heading_mode",
        "loop",
    ):
        assert f"resolved.{field}" in source


def test_resolution_uses_alignment_bound_status_not_latest_tf_lookup() -> None:
    source = (ROOT / "brov_mission" / "mission_manager_node.py").read_text()
    assert "status.pool_to_odom" in source
    assert "candidate[\"alignment_id\"]" in source
    assert "lookup_transform" not in source


def test_first_point_gate_uses_only_identity_bound_odometry() -> None:
    source = (ROOT / "brov_mission" / "mission_manager_node.py").read_text()
    assert "AlignedOdometry" in source
    assert "envelope.localization_epoch" in source
    assert "envelope.odometry_session_id" in source
    assert "envelope.alignment_id" in source
    assert '"/brov/localization/odometry_pool"' not in source
    assert "_on_odometry_pool" not in source


def test_configurable_pool_frame_reaches_canonical_hash_and_payload() -> None:
    source = (ROOT / "brov_mission" / "mission_manager_node.py").read_text()
    assert source.count("frame_id=self._pool_frame") >= 2
    assert "CONTRACT_HEADING_MODES" in source
    assert "contract_version=self._contract_version" in source


def test_v2_random_metadata_parameters_are_complete_and_bounded() -> None:
    document = yaml.safe_load(
        (ROOT / "config" / "mission_manager.yaml").read_text()
    )
    parameters = document["brov_mission_manager"]["ros__parameters"]
    assert parameters["min_waypoints"] == 2
    assert parameters["min_waypoints"] <= parameters["max_waypoints"]
    required = {
        "random_attitude_seed",
        "random_attitude_reference_frame",
        "random_attitude_generator_version",
        "random_attitude_rpy_min_rad",
        "random_attitude_rpy_max_rad",
        "random_attitude_max_slew_rate_rad_s",
        "random_attitude_tolerance_rad",
        "random_attitude_angular_speed_tolerance_rad_s",
        "random_attitude_dwell_time_s",
        "random_attitude_max_duration_s",
        "random_attitude_max_laps",
    }
    assert required.issubset(parameters)
    assert parameters["random_attitude_reference_frame"] == "pool_zup_flu"
    assert (
        parameters["random_attitude_generator_version"]
        == "sha256_counter_uniform_rpy_v1"
    )
    assert len(parameters["random_attitude_rpy_min_rad"]) == 3
    assert len(parameters["random_attitude_rpy_max_rad"]) == 3
    assert parameters["random_attitude_max_duration_s"] > 0.0
    assert parameters["random_attitude_max_laps"] > 0
