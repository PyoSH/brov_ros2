"""Load the surveyed marker contract used by both perception and RViz."""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .geometry import finite_vector, normalized_quaternion


def load_marker_survey(path: str | Path) -> dict:
    """Extract the pool/marker parameters from brov_perception's YAML."""
    config_path = Path(path)
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    try:
        parameters = document["brov_aruco_pose_node"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"invalid ArUco parameter file: {config_path}"
        ) from exc

    if parameters.get("publish_pool_pose") is not True:
        raise ValueError("ArUco config must enable publish_pool_pose")
    pool_frame = str(parameters.get("pool_frame", "")).strip()
    if not pool_frame or pool_frame.startswith("/"):
        raise ValueError("pool_frame must be a non-empty relative frame name")

    marker_length = float(parameters.get("marker_length_m", 0.0))
    marker_id = int(parameters.get("marker_id", -1))
    dictionary = str(parameters.get("dictionary", "")).strip()
    if not math.isfinite(marker_length) or marker_length <= 0.0:
        raise ValueError("marker_length_m must be finite and positive")
    if marker_id < 0 or not dictionary:
        raise ValueError("dictionary and marker_id must identify the marker")

    return {
        "pool_frame": pool_frame,
        "marker_xyz": list(
            finite_vector(
                parameters["pool_to_marker_xyz"], 3, "pool_to_marker_xyz"
            )
        ),
        "marker_quaternion_xyzw": list(
            normalized_quaternion(
                parameters["pool_to_marker_quaternion_xyzw"]
            )
        ),
        "marker_size_m": marker_length,
        "marker_label": f"{dictionary.removeprefix('DICT_')} ID {marker_id}",
    }
