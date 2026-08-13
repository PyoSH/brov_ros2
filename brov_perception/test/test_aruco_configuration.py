from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from brov_perception.geometry import matrix_from_xyz_rpy
from brov_perception.aruco_pose_node import (
    _detector_parameters,
    _dictionary_capacity,
    _estimate_marker_transform,
    _marker_to_base_transform,
    _pool_to_base_transform,
    _resolve_dictionary,
    _validate_marker_contract,
    _validated_extrinsic,
    _validated_pool_marker_transform,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _deployed_parameters() -> dict:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "aruco.yaml").read_text(encoding="utf-8")
    )
    return config["brov_aruco_pose_node"]["ros__parameters"]


def test_deployed_marker_contract_is_apriltag_16h5_id2() -> None:
    parameters = _deployed_parameters()
    assert parameters["dictionary"] == "DICT_APRILTAG_16h5"
    assert parameters["marker_id"] == 2
    assert parameters["marker_length_m"] == pytest.approx(0.42)
    assert parameters["corner_refinement"] == "SUBPIX"
    assert parameters["publish_robot_pose"] is True
    assert parameters["publish_robot_tf"] is False
    assert parameters["publish_marker_tf"] is False
    assert parameters["publish_pool_pose"] is True
    assert parameters["pool_frame"] == "pool"
    assert parameters["pool_to_marker_xyz"] == pytest.approx(
        [3.8, 0.85, 0.24]
    )
    assert parameters["pool_to_marker_quaternion_xyzw"] == pytest.approx(
        [-0.5, -0.5, 0.5, 0.5]
    )
    assert parameters["base_to_camera_xyz"] == pytest.approx(
        [0.15751251578330994, 0.0052856863476336, 0.06784216314554214]
    )
    assert parameters["base_to_camera_rpy"] == pytest.approx(
        [-np.pi / 2.0, 0.0, -np.pi / 2.0]
    )


def test_dictionary_contains_id2() -> None:
    dictionary = _resolve_dictionary("DICT_APRILTAG_16h5")
    assert dictionary.markerSize == 4
    assert _dictionary_capacity(dictionary) == 30
    _validate_marker_contract(dictionary, marker_id=2, marker_length_m=0.42)


@pytest.mark.parametrize("marker_id", [-1, 30, 100])
def test_marker_id_outside_dictionary_is_rejected(marker_id: int) -> None:
    dictionary = _resolve_dictionary("DICT_APRILTAG_16h5")
    with pytest.raises(ValueError, match="marker_id"):
        _validate_marker_contract(dictionary, marker_id, 0.42)


@pytest.mark.parametrize(
    "marker_length_m", [0.0, -0.42, float("nan"), float("inf")]
)
def test_invalid_marker_length_is_rejected(marker_length_m: float) -> None:
    dictionary = _resolve_dictionary("DICT_APRILTAG_16h5")
    with pytest.raises(ValueError, match="finite and positive"):
        _validate_marker_contract(dictionary, 2, marker_length_m)


def test_empty_or_unknown_dictionary_is_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        _resolve_dictionary("")
    with pytest.raises(ValueError, match="unsupported"):
        _resolve_dictionary("DICT_DOES_NOT_EXIST")


def test_subpix_refinement_is_configured() -> None:
    parameters = _detector_parameters("SUBPIX")
    assert parameters.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX


def test_cv_optical_axes_are_expressed_in_base_flu() -> None:
    transform = _validated_extrinsic(
        [0.15751251578330994, 0.0052856863476336, 0.06784216314554214],
        [-np.pi / 2.0, 0.0, -np.pi / 2.0],
    )
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation[:, 0], [0.0, -1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(rotation[:, 1], [0.0, 0.0, -1.0], atol=1e-12)
    np.testing.assert_allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-12)


@pytest.mark.parametrize(
    ("xyz", "rpy"),
    [
        ([0.0, 0.0], [0.0, 0.0, 0.0]),
        ([0.0, 0.0, float("nan")], [0.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.0], [0.0, float("inf"), 0.0]),
    ],
)
def test_invalid_base_to_camera_extrinsic_is_rejected(xyz, rpy) -> None:
    with pytest.raises(ValueError, match="base_to_camera"):
        _validated_extrinsic(xyz, rpy)


def test_synthetic_apriltag_16h5_id2_is_detected() -> None:
    dictionary = _resolve_dictionary("DICT_APRILTAG_16h5")
    marker = cv2.aruco.drawMarker(dictionary, 2, 240)
    image = np.full((320, 320), 255, dtype=np.uint8)
    image[40:280, 40:280] = marker

    _, ids, _ = cv2.aruco.detectMarkers(
        image,
        dictionary,
        parameters=_detector_parameters("SUBPIX"),
    )

    assert ids is not None
    assert ids.flatten().tolist() == [2]


def test_420mm_marker_recovers_metric_relative_translation() -> None:
    marker_length_m = 0.42
    camera_matrix = np.array(
        [
            [465.5181, 0.0, 324.6713],
            [0.0, 465.3189, 243.1137],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)
    half = marker_length_m / 2.0
    object_corners = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    expected_translation = np.array([0.0, 0.0, 1.5])
    image_corners, _ = cv2.projectPoints(
        object_corners,
        np.zeros(3),
        expected_translation,
        camera_matrix,
        distortion,
    )

    transform, _, _ = _estimate_marker_transform(
        [image_corners.reshape(1, 4, 2).astype(np.float32)],
        marker_length_m,
        camera_matrix,
        distortion,
    )

    np.testing.assert_allclose(
        transform[:3, 3], expected_translation, atol=1e-6
    )


def test_marker_relative_robot_pose_closes_transform_chain() -> None:
    transform_base_camera = _validated_extrinsic(
        [0.15751251578330994, 0.0052856863476336, 0.06784216314554214],
        [-np.pi / 2.0, 0.0, -np.pi / 2.0],
    )
    transform_camera_marker = np.eye(4)
    transform_camera_marker[:3, 3] = [0.15, -0.08, 1.25]

    transform_marker_base = _marker_to_base_transform(
        transform_camera_marker, transform_base_camera
    )

    np.testing.assert_allclose(
        transform_base_camera
        @ transform_camera_marker
        @ transform_marker_base,
        np.eye(4),
        atol=1e-12,
    )


def test_pool_relative_robot_pose_recovers_nontrivial_base_pose() -> None:
    transform_pool_marker = _validated_pool_marker_transform(
        [3.8, 0.85, 0.24], [-0.5, -0.5, 0.5, 0.5]
    )
    transform_base_camera = _validated_extrinsic(
        [0.15751251578330994, 0.0052856863476336, 0.06784216314554214],
        [-np.pi / 2.0, 0.0, -np.pi / 2.0],
    )
    expected_pool_base = matrix_from_xyz_rpy(
        [1.2, 0.7, 0.4], [0.1, -0.2, 0.3]
    )
    transform_pool_camera = expected_pool_base @ transform_base_camera
    transform_camera_marker = (
        np.linalg.inv(transform_pool_camera) @ transform_pool_marker
    )

    actual_pool_base = _pool_to_base_transform(
        transform_pool_marker,
        transform_camera_marker,
        transform_base_camera,
    )

    np.testing.assert_allclose(actual_pool_base, expected_pool_base, atol=1e-12)


def test_head_on_marker_recovers_pool_aligned_base_pose() -> None:
    """Golden case: a level robot faces the marker along pool +X."""
    transform_pool_marker = _validated_pool_marker_transform(
        [3.8, 0.85, 0.24], [-0.5, -0.5, 0.5, 0.5]
    )
    transform_base_camera = _validated_extrinsic(
        [0.15751251578330994, 0.0052856863476336, 0.06784216314554214],
        [-np.pi / 2.0, 0.0, -np.pi / 2.0],
    )
    transform_camera_marker = np.eye(4)
    transform_camera_marker[:3, :3] = np.diag([-1.0, 1.0, -1.0])
    transform_camera_marker[:3, 3] = [
        0.0052856863476336,
        0.06784216314554214,
        1.6424874842166899,
    ]

    actual_pool_base = _pool_to_base_transform(
        transform_pool_marker,
        transform_camera_marker,
        transform_base_camera,
    )

    np.testing.assert_allclose(actual_pool_base[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(
        actual_pool_base[:3, 3], [2.0, 0.85, 0.24], atol=1e-12
    )
    np.testing.assert_allclose(
        actual_pool_base @ transform_base_camera @ transform_camera_marker,
        transform_pool_marker,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("xyz", "quaternion"),
    [
        ([3.8, 0.85], [-0.5, -0.5, 0.5, 0.5]),
        ([3.8, 0.85, float("nan")], [-0.5, -0.5, 0.5, 0.5]),
        ([3.8, 0.85, 0.24], [0.0, 0.0, 0.0, 0.0]),
        ([3.8, 0.85, 0.24], [1.0, 1.0, 1.0, 1.0]),
    ],
)
def test_invalid_pool_marker_survey_is_rejected(xyz, quaternion) -> None:
    with pytest.raises(ValueError, match="pool-to-marker"):
        _validated_pool_marker_transform(xyz, quaternion)
