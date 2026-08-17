"""Static contract tests for the isolated MK2 Case-A deployment launch."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROL_ROOT = PACKAGE_ROOT.parent / "brov_control"


def test_mk2_mission_is_takeoff_then_two_meter_loop_at_half_mps() -> None:
    mission = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mission_sim2sim_mk2_case_a_0p5.yaml")
        .read_text(encoding="utf-8")
    )["brov_obs_node"]["ros__parameters"]
    assert mission["waypoints"] == "0,0,0;0,0,0.20;2.0,0,0.20"
    assert mission["heading_mode"] == "takeoff_then_align"
    assert mission["loop"] is True
    assert mission["cruise_speed"] == 0.50


def test_mk2_controller_has_no_slew_or_artifact_fallback() -> None:
    controller = yaml.safe_load(
        (CONTROL_ROOT / "config" / "rl_controller_mk2_deploy_v2.yaml")
        .read_text(encoding="utf-8")
    )["brov_policy_node"]["ros__parameters"]
    assert controller["policy_path"] == ""
    assert controller["policy_metadata_path"] == ""
    assert controller["pwm_slew_rate_per_s"] == 0.0


def test_mk2_launch_uses_separate_executable_and_safe_defaults() -> None:
    source = (
        PACKAGE_ROOT / "launch" / "sim2sim_mk2_case_a.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="policy_node_mk2"' in source
    assert '"BROV_MK2_POLICY_PATH"' in source
    assert '"send_pwm", default_value="false"' in source
    assert '"arm", default_value="false"' in source
    assert '"start_gazebo_truth_bridge"' in source
