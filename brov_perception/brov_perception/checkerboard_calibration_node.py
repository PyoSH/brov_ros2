#!/usr/bin/env python3
"""Automatically collect checkerboard samples and calibrate a monocular camera."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .calibration import resolve_calibration_path, save_calibration


class CheckerboardCalibrationNode(Node):
    """Collect geometrically diverse checkerboard views without a GUI."""

    def __init__(self) -> None:
        super().__init__("brov_checkerboard_calibration")
        self.declare_parameter("columns", 8)
        self.declare_parameter("rows", 6)
        self.declare_parameter("square_size_m", 0.030)
        self.declare_parameter("target_samples", 30)
        self.declare_parameter("min_interval_s", 0.5)
        self.declare_parameter("min_descriptor_distance", 0.06)
        self.declare_parameter("output_path", "")

        self._columns = int(self.get_parameter("columns").value)
        self._rows = int(self.get_parameter("rows").value)
        self._square_size = float(self.get_parameter("square_size_m").value)
        self._target_samples = int(self.get_parameter("target_samples").value)
        self._minimum_interval = float(self.get_parameter("min_interval_s").value)
        self._minimum_distance = float(
            self.get_parameter("min_descriptor_distance").value
        )
        self._path = resolve_calibration_path(
            str(self.get_parameter("output_path").value)
        )
        if self._columns < 2 or self._rows < 2:
            raise ValueError("columns and rows must both be at least 2")
        if self._square_size <= 0.0:
            raise ValueError("square_size_m must be positive")
        if self._target_samples < 3:
            raise ValueError("target_samples must be at least 3")
        if self._minimum_interval < 0.0 or self._minimum_distance < 0.0:
            raise ValueError("sample interval and descriptor distance must be non-negative")

        self._bridge = CvBridge()
        self._object_points: list[np.ndarray] = []
        self._image_points: list[np.ndarray] = []
        self._descriptors: list[np.ndarray] = []
        self._last_sample_time = 0.0
        self._image_size: tuple[int, int] | None = None
        self._done = False

        grid = np.zeros((self._rows * self._columns, 3), np.float32)
        grid[:, :2] = np.mgrid[0 : self._columns, 0 : self._rows].T.reshape(-1, 2)
        self._grid = grid * self._square_size

        self.pub_debug = self.create_publisher(
            Image, "/brov/camera/calibration_debug", qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/brov/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"checkerboard={self._columns}x{self._rows}, "
            f"square={self._square_size:.4f} m, "
            f"target={self._target_samples}, output={self._path}"
        )

    def _descriptor(
        self, corners: np.ndarray, width: int, height: int
    ) -> np.ndarray:
        points = corners.reshape(-1, 2)
        image_extent = np.array([width, height], dtype=float)
        center = points.mean(axis=0) / image_extent
        span = (points.max(axis=0) - points.min(axis=0)) / image_extent
        top_edge = points[self._columns - 1] - points[0]
        angle = np.arctan2(top_edge[1], top_edge[0]) / np.pi
        return np.array([center[0], center[1], span[0], span[1], angle])

    def _save(
        self,
        width: int,
        height: int,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
    ) -> None:
        projection = np.zeros((3, 4), dtype=float)
        projection[:, :3] = camera_matrix
        data = {
            "image_width": int(width),
            "image_height": int(height),
            "camera_name": "brov_camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": camera_matrix.reshape(-1).tolist(),
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(distortion.size),
                "data": distortion.reshape(-1).tolist(),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": np.eye(3).reshape(-1).tolist(),
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": projection.reshape(-1).tolist(),
            },
        }
        save_calibration(self._path, data)

    def _calibrate(self) -> None:
        if self._image_size is None:
            raise RuntimeError("cannot calibrate before receiving an image")
        rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
            self._object_points,
            self._image_points,
            self._image_size,
            None,
            None,
        )
        width, height = self._image_size
        self._save(width, height, camera_matrix, distortion)
        self._done = True
        self.get_logger().info(
            f"calibration complete: RMS reprojection error={rms:.4f}, "
            f"saved={self._path}; restart camera_stream_node to reload it"
        )

    def _on_image(self, message: Image) -> None:
        if self._done:
            return
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        height, width = frame.shape[:2]
        if self._image_size is None:
            self._image_size = (width, height)
        elif self._image_size != (width, height):
            self.get_logger().error(
                "image resolution changed during collection; restart calibration"
            )
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            (self._columns, self._rows),
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        accepted = False
        if found:
            descriptor = self._descriptor(corners, width, height)
            diverse = not self._descriptors or min(
                np.linalg.norm(descriptor - previous)
                for previous in self._descriptors
            ) >= self._minimum_distance
            now = time.monotonic()
            if diverse and now - self._last_sample_time >= self._minimum_interval:
                self._object_points.append(self._grid.copy())
                self._image_points.append(corners.astype(np.float32))
                self._descriptors.append(descriptor)
                self._last_sample_time = now
                accepted = True
                self.get_logger().info(
                    f"sample {len(self._image_points)}/{self._target_samples} accepted"
                )
            cv2.drawChessboardCorners(
                frame, (self._columns, self._rows), corners, found
            )

        color = (0, 255, 0) if accepted else (0, 180, 255)
        cv2.putText(
            frame,
            f"samples {len(self._image_points)}/{self._target_samples}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )
        debug = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = message.header
        self.pub_debug.publish(debug)

        if len(self._image_points) >= self._target_samples:
            self._calibrate()


def main() -> None:
    rclpy.init()
    node = CheckerboardCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
