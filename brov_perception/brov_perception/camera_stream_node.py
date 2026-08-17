#!/usr/bin/env python3
"""Publish a BlueOS RTP/H264 UDP stream as ROS 2 Image and CameraInfo."""

from __future__ import annotations

from pathlib import Path

import gi
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo

from .calibration import load_calibration, resolve_calibration_path, save_calibration

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def _fixed_matrix(values: object, expected: int, default: list[float]) -> list[float]:
    if not isinstance(values, dict):
        return list(default)
    data = values.get("data")
    if not isinstance(data, list) or len(data) != expected:
        return list(default)
    return [float(value) for value in data]


def _distortion_vector(values: object) -> list[float]:
    if not isinstance(values, dict):
        return []
    data = values.get("data")
    if not isinstance(data, list):
        return []
    return [float(value) for value in data]


class CameraStreamNode(Node):
    """Decode the BlueOS H264 stream and publish calibrated camera messages."""

    def __init__(self) -> None:
        super().__init__("brov_camera_node")
        self.declare_parameter("udp_port", 5600)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("camera_info_path", "")
        self.declare_parameter("latency_ms", 200)

        port = int(self.get_parameter("udp_port").value)
        latency_ms = int(self.get_parameter("latency_ms").value)
        if not 1 <= port <= 65535:
            raise ValueError("udp_port must be in [1, 65535]")
        if latency_ms < 0:
            raise ValueError("latency_ms must be non-negative")

        self._frame_id = str(self.get_parameter("frame_id").value)
        configured_path = str(self.get_parameter("camera_info_path").value)
        self._info_path = resolve_calibration_path(configured_path)
        self._info = self._load_camera_info(self._info_path)
        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_diag_count = 0
        self._last_diag_ns = self.get_clock().now().nanoseconds

        self.pub_image = self.create_publisher(
            Image, "/brov/camera/image_raw", qos_profile_sensor_data
        )
        self.pub_info = self.create_publisher(
            CameraInfo, "/brov/camera/camera_info", qos_profile_sensor_data
        )
        self.srv_info = self.create_service(
            SetCameraInfo, "/brov/camera/set_camera_info", self._set_camera_info
        )

        Gst.init(None)
        pipeline = (
            f"udpsrc name=source port={port} buffer-size=2097152 "
            'caps="application/x-rtp,media=video,clock-rate=90000,'
            'encoding-name=H264" '
            f"! rtpjitterbuffer name=jitter latency={latency_ms} "
            "drop-on-latency=false do-lost=true "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "leaky=downstream "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
        self._pipeline = Gst.parse_launch(pipeline)
        self._bus = self._pipeline.get_bus()
        self._jitter = self._pipeline.get_by_name("jitter")
        sink = self._pipeline.get_by_name("sink")
        if sink is None or self._jitter is None:
            raise RuntimeError("GStreamer pipeline elements were not created")
        sink.connect("new-sample", self._on_sample)
        state_result = self._pipeline.set_state(Gst.State.PLAYING)
        if state_result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer pipeline failed to enter PLAYING state")
        self.create_timer(0.2, self._poll_gstreamer_bus)
        self.create_timer(5.0, self._report_stream_stats)
        self.get_logger().info(
            f"BlueOS H264 waiting on udp://0.0.0.0:{port} -> "
            f"/brov/camera/image_raw; calibration={self._info_path}"
        )

    def _report_stream_stats(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self._last_diag_ns) / 1e9
        frames = self._frame_count - self._last_diag_count
        fps = frames / elapsed if elapsed > 0.0 else 0.0
        self._last_diag_ns = now_ns
        self._last_diag_count = self._frame_count

        stats = self._jitter.get_property("stats")
        values: dict[str, object] = {}
        if stats is not None:
            for key in ("num-pushed", "num-lost", "num-late", "num-duplicates"):
                try:
                    values[key] = stats.get_value(key)
                except Exception:
                    values[key] = "?"
        self.get_logger().info(
            f"camera decode={fps:.1f} fps, RTP pushed={values.get('num-pushed', '?')}, "
            f"lost={values.get('num-lost', '?')}, "
            f"late={values.get('num-late', '?')}, "
            f"duplicates={values.get('num-duplicates', '?')}"
        )

    def _poll_gstreamer_bus(self) -> None:
        message = self._bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if message is None:
            return
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            self.get_logger().error(f"GStreamer error: {error}; {debug or 'no details'}")
        else:
            self.get_logger().warning("GStreamer stream reached end-of-stream")

    def _load_camera_info(self, path: Path) -> CameraInfo:
        message = CameraInfo()
        message.header.frame_id = self._frame_id
        if not path.is_file():
            self.get_logger().warning(
                f"camera calibration not found: {path}; publishing uncalibrated info"
            )
            return message
        data = load_calibration(path)
        message.width = int(data.get("image_width", 0))
        message.height = int(data.get("image_height", 0))
        message.distortion_model = str(data.get("distortion_model", "plumb_bob"))
        message.d = _distortion_vector(data.get("distortion_coefficients"))
        message.k = _fixed_matrix(data.get("camera_matrix"), 9, [0.0] * 9)
        message.r = _fixed_matrix(
            data.get("rectification_matrix"),
            9,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
        )
        message.p = _fixed_matrix(data.get("projection_matrix"), 12, [0.0] * 12)
        self.get_logger().info(f"loaded camera calibration: {path}")
        return message

    def _save_camera_info(self, message: CameraInfo) -> None:
        data = {
            "image_width": int(message.width),
            "image_height": int(message.height),
            "camera_name": "brov_camera",
            "camera_matrix": {"rows": 3, "cols": 3, "data": list(message.k)},
            "distortion_model": message.distortion_model,
            "distortion_coefficients": {
                "rows": 1,
                "cols": len(message.d),
                "data": list(message.d),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": list(message.r),
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": list(message.p),
            },
        }
        save_calibration(self._info_path, data)

    def _set_camera_info(self, request, response):
        try:
            self._info = request.camera_info
            self._info.header.frame_id = self._frame_id
            self._save_camera_info(self._info)
            response.success = True
            response.status_message = f"saved: {self._info_path}"
            self.get_logger().info(response.status_message)
        except Exception as exception:
            response.success = False
            response.status_message = str(exception)
            self.get_logger().error(f"failed to save camera info: {exception}")
        return response

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        caps = sample.get_caps().get_structure(0)
        width = int(caps.get_value("width"))
        height = int(caps.get_value("height"))
        buffer = sample.get_buffer()
        mapped_ok, mapped = buffer.map(Gst.MapFlags.READ)
        if not mapped_ok:
            return Gst.FlowReturn.ERROR
        try:
            frame = np.frombuffer(mapped.data, dtype=np.uint8).reshape(
                (height, width, 3)
            )
            image = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        finally:
            buffer.unmap(mapped)

        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self._frame_id
        info = CameraInfo()
        info.header = image.header
        info.width = width
        info.height = height
        info.distortion_model = self._info.distortion_model
        info.d = list(self._info.d)
        info.k = list(self._info.k)
        info.r = list(self._info.r)
        info.p = list(self._info.p)
        self.pub_image.publish(image)
        self.pub_info.publish(info)
        self._frame_count += 1
        return Gst.FlowReturn.OK

    def destroy_node(self):
        self._pipeline.set_state(Gst.State.NULL)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraStreamNode()
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
