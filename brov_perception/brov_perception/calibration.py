"""Writable camera-calibration paths and ROS camera-calibration YAML I/O."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Mapping

import yaml


DATA_DIR_ENV = "BROV_DATA_DIR"
CALIBRATION_FILENAME = "camera_intrinsics.yaml"


def brov_data_dir(
    env: Mapping[str, str] | None = None,
    home: str | Path | None = None,
) -> Path:
    """Return the user-writable BROV state directory.

    ``BROV_DATA_DIR`` overrides the default ``~/.ros/brov`` location. The
    function does not create the directory, which keeps imports side-effect
    free and makes it safe to use in launch/config inspection.
    """

    values = os.environ if env is None else env
    configured = values.get(DATA_DIR_ENV, "").strip()
    if configured:
        return Path(configured).expanduser()
    base_home = Path.home() if home is None else Path(home).expanduser()
    return base_home / ".ros" / "brov"


def default_calibration_path(
    env: Mapping[str, str] | None = None,
    home: str | Path | None = None,
) -> Path:
    """Return the default persistent camera-intrinsic YAML path."""

    return brov_data_dir(env=env, home=home) / "calibration" / CALIBRATION_FILENAME


def resolve_calibration_path(
    configured: str | Path | None,
    env: Mapping[str, str] | None = None,
    home: str | Path | None = None,
) -> Path:
    """Resolve a parameter value, falling back to the writable default."""

    if configured is not None and str(configured).strip():
        return Path(str(configured)).expanduser()
    return default_calibration_path(env=env, home=home)


def load_calibration(path: str | Path) -> dict:
    """Load calibration YAML, returning an empty mapping for an empty file."""

    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"camera calibration root must be a mapping: {path}")
    return data


def save_calibration(path: str | Path, data: Mapping) -> None:
    """Atomically persist calibration YAML in a user-writable directory."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            yaml.safe_dump(dict(data), stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
        temporary_name = None
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
