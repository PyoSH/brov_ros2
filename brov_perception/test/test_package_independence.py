from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_python_sources_have_no_legacy_repository_dependency() -> None:
    legacy_name = "de" + "ploy"
    forbidden = (
        f"from {legacy_name}",
        f"import {legacy_name}",
        "sys.path",
        "/workspace/" + legacy_name,
    )
    for path in (PACKAGE_ROOT / "brov_perception").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in source, f"{token!r} found in {path}"


def test_all_config_files_are_ros_or_calibration_yaml() -> None:
    for path in (PACKAGE_ROOT / "config").glob("*.yaml"):
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert isinstance(data, dict) and data
        if path.name != "camera_intrinsics.example.yaml":
            for node_config in data.values():
                assert "ros__parameters" in node_config


def test_package_metadata_and_entry_points_exist() -> None:
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "brov_perception"
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    for executable in (
        "camera_stream_node",
        "checkerboard_calibration_node",
        "aruco_pose_node",
    ):
        assert executable in setup_source
