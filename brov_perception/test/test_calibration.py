from pathlib import Path

from brov_perception.calibration import (
    brov_data_dir,
    default_calibration_path,
    load_calibration,
    resolve_calibration_path,
    save_calibration,
)


def test_data_dir_uses_environment_override() -> None:
    assert brov_data_dir(env={"BROV_DATA_DIR": "/data/brov"}) == Path("/data/brov")


def test_data_dir_defaults_below_ros_home() -> None:
    assert brov_data_dir(env={}, home="/home/operator") == Path(
        "/home/operator/.ros/brov"
    )


def test_default_calibration_location() -> None:
    path = default_calibration_path(env={"BROV_DATA_DIR": "/persistent"})
    assert path == Path("/persistent/calibration/camera_intrinsics.yaml")


def test_explicit_calibration_path_wins() -> None:
    path = resolve_calibration_path(
        "/experiment/intrinsics.yaml",
        env={"BROV_DATA_DIR": "/persistent"},
    )
    assert path == Path("/experiment/intrinsics.yaml")


def test_blank_calibration_path_uses_default() -> None:
    path = resolve_calibration_path(
        " ", env={"BROV_DATA_DIR": "/persistent"}
    )
    assert path == Path("/persistent/calibration/camera_intrinsics.yaml")


def test_calibration_round_trip_is_atomic(tmp_path: Path) -> None:
    destination = tmp_path / "nested" / "camera.yaml"
    expected = {
        "image_width": 640,
        "image_height": 480,
        "camera_matrix": {"rows": 3, "cols": 3, "data": list(range(9))},
    }
    save_calibration(destination, expected)
    assert load_calibration(destination) == expected
    assert not list(destination.parent.glob("*.tmp"))
