#!/usr/bin/env python3
"""Check the standalone BROV runtime without transmitting MAVLink or PWM."""

from __future__ import annotations

import hashlib
import importlib
import os
import platform
import socket
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.machine()} / {platform.system()}")
    print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', '<unset>')}")
    print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '<unset>')}")
    print(f"BROV_DATA_DIR: {os.environ.get('BROV_DATA_DIR', '<unset>')}")

    failed = False
    for name in (
        "torch", "yaml", "pymavlink", "rclpy", "std_msgs", "sensor_msgs",
        "geometry_msgs", "nav_msgs", "std_srvs", "visualization_msgs",
        "cv_bridge", "cv2", "gi", "brov_interfaces", "brov_base",
        "brov_control", "brov_perception", "brov_localization",
        "brov_mission", "brov_viz", "brov_bringup",
    ):
        try:
            module = importlib.import_module(name)
            version = getattr(module, "__version__", "available")
            print(f"[OK] {name}: {version}")
        except Exception as exc:
            failed = True
            print(f"[FAIL] {name}: {exc}")

    try:
        import cv2
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is unavailable")
        dictionary_name = "DICT_APRILTAG_16h5"
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f"cv2.aruco.{dictionary_name} is unavailable")
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
        marker_count = int(dictionary.bytesList.shape[0])
        if int(dictionary.markerSize) != 4 or marker_count <= 2:
            raise RuntimeError(
                "unexpected AprilTag 16h5 dictionary contract: "
                f"markerSize={dictionary.markerSize}, count={marker_count}"
            )
        Gst.init(None)
        print(
            f"[OK] camera stack: OpenCV {cv2.__version__} / "
            f"{Gst.version_string()} / AprilTag 16h5 IDs=0..{marker_count - 1}"
        )
    except Exception as exc:
        failed = True
        print(f"[FAIL] camera stack: {exc}")

    try:
        from ament_index_python.packages import get_package_prefix

        for package in (
            "brov_interfaces", "brov_base", "brov_control",
            "brov_perception", "brov_localization", "brov_mission",
            "brov_viz", "brov_bringup",
        ):
            print(f"[OK] ROS package {package}: {get_package_prefix(package)}")
    except Exception as exc:
        failed = True
        print(f"[FAIL] BROV ROS package overlay: {exc}")

    vehicle_model_path: Path | None = None
    try:
        from brov_base.vendor import params as vehicle_params

        params = vehicle_params.load_brov2_yaml()
        vehicle_model_path = Path(vehicle_params.__file__).with_name(
            "brov2_heavy.yaml"
        )
        print(
            f"[OK] vehicle model: {params['name']} / "
            f"{params['thrusters']['num']} thrusters"
        )
    except Exception as exc:
        failed = True
        print(f"[FAIL] vehicle model: {exc}")

    policy_path = Path(
        os.environ.get(
            "BROV_POLICY_PATH",
            str(
                REPOSITORY_ROOT
                / "artifacts"
                / "policies"
                / "demo_policy"
                / "policy.pt"
            ),
        )
    )
    metadata_path = policy_path.with_name("metadata.yaml")
    checksum_path = policy_path.with_name("sha256.txt")
    if (
        policy_path.is_file()
        and metadata_path.is_file()
        and checksum_path.is_file()
    ):
        try:
            import torch
            import yaml

            metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            expected = str(metadata["sha256"])
            actual = _sha256(policy_path)
            if actual != expected:
                raise RuntimeError(f"checksum mismatch: {actual} != {expected}")

            checksum_fields = checksum_path.read_text(encoding="utf-8").split()
            if checksum_fields != [expected, policy_path.name]:
                raise RuntimeError(
                    "sha256.txt does not match metadata.yaml and policy filename"
                )

            if vehicle_model_path is None:
                raise RuntimeError("vehicle model path was not resolved")
            expected_vehicle = str(metadata["vehicle_model_sha256"])
            actual_vehicle = _sha256(vehicle_model_path)
            if actual_vehicle != expected_vehicle:
                raise RuntimeError(
                    "vehicle model checksum mismatch: "
                    f"{actual_vehicle} != {expected_vehicle}"
                )

            input_shape = tuple(int(value) for value in metadata["input"]["shape"])
            output_shape = tuple(int(value) for value in metadata["output"]["shape"])
            if input_shape != (1, 16) or output_shape != (1, 6):
                raise RuntimeError(
                    f"unsupported policy contract: {input_shape} -> {output_shape}"
                )
            model = torch.jit.load(str(policy_path), map_location="cpu")
            model.eval()
            output = model(torch.zeros(input_shape))
            if tuple(output.shape) != output_shape:
                raise RuntimeError(f"unexpected output shape: {tuple(output.shape)}")
            print(
                "[OK] policy: policy/vehicle checksums + input "
                f"{input_shape} -> {tuple(output.shape)}"
            )
        except Exception as exc:
            failed = True
            print(f"[FAIL] policy: {exc}")
    else:
        failed = True
        print(f"[FAIL] policy artifact/metadata/checksum missing: {policy_path}")

    for port in (14550, 5600):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("0.0.0.0", port))
            sock.close()
            print(f"[OK] UDP {port} can be bound (no packet transmitted)")
        except OSError as exc:
            failed = True
            print(f"[FAIL] UDP {port} bind: {exc}")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
