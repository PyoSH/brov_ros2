#!/usr/bin/env python3
"""Estimate ArUco marker pose and optionally robot pose from calibrated images."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .geometry import matrix_from_xyz_rpy, quaternion_from_matrix


def _fill_pose(message: PoseStamped, transform: np.ndarray) -> None:
    quaternion = quaternion_from_matrix(transform[:3, :3])
    translation = transform[:3, 3]
    message.pose.position.x = float(translation[0])
    message.pose.position.y = float(translation[1])
    message.pose.position.z = float(translation[2])
    message.pose.orientation.x = float(quaternion[0])
    message.pose.orientation.y = float(quaternion[1])
    message.pose.orientation.z = float(quaternion[2])
    message.pose.orientation.w = float(quaternion[3])


def _detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


class ArucoPoseNode(Node):
    """Publish a selected marker's metric pose and corresponding TF."""

    def __init__(self) -> None:
        super().__init__("brov_aruco_pose_node")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_length_m", 0.15)
        self.declare_parameter("marker_frame", "aruco_reference")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_robot_pose", False)
        self.declare_parameter("base_to_camera_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_camera_rpy", [0.0, 0.0, 0.0])

        dictionary_name = str(self.get_parameter("dictionary").value)
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"unsupported ArUco dictionary: {dictionary_name}")
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
        self._detector_parameters = _detector_parameters()
        self._marker_id = int(self.get_parameter("marker_id").value)
        self._marker_length = float(self.get_parameter("marker_length_m").value)
        self._marker_frame = str(self.get_parameter("marker_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_robot = bool(self.get_parameter("publish_robot_pose").value)
        xyz = [float(value) for value in self.get_parameter("base_to_camera_xyz").value]
        rpy = [float(value) for value in self.get_parameter("base_to_camera_rpy").value]
        if self._marker_id < 0:
            raise ValueError("marker_id must be non-negative")
        if self._marker_length <= 0.0:
            raise ValueError("marker_length_m must be positive")
        self._transform_base_camera = matrix_from_xyz_rpy(xyz, rpy)

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._warned_uncalibrated = False
        self._tf = TransformBroadcaster(self)
        self.pub_marker = self.create_publisher(
            PoseStamped, "/brov/aruco/marker_pose", 10
        )
        self.pub_robot = self.create_publisher(
            PoseStamped, "/brov/aruco/robot_pose", 10
        )
        self.pub_visible = self.create_publisher(Bool, "/brov/aruco/visible", 10)
        self.pub_debug = self.create_publisher(
            Image, "/brov/aruco/debug_image", qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/brov/camera/camera_info",
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/brov/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"waiting for ArUco {dictionary_name} id={self._marker_id}, "
            f"length={self._marker_length:.3f} m"
        )

    def _on_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        if message.k[0] > 0.0:
            self._warned_uncalibrated = False

    def _publish_tf(self, parent: str, child: str, pose: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = pose.header
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf.sendTransform(transform)

    def _on_image(self, message: Image) -> None:
        if self._camera_info is None or self._camera_info.k[0] <= 0.0:
            self.pub_visible.publish(Bool(data=False))
            if not self._warned_uncalibrated:
                self._warned_uncalibrated = True
                self.get_logger().warning(
                    "valid camera intrinsics are required for metric ArUco pose"
                )
            return

        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._detector_parameters
        )
        detected_ids = [] if ids is None else ids.flatten().tolist()
        visible = self._marker_id in detected_ids
        self.pub_visible.publish(Bool(data=visible))
        if not visible:
            return

        index = detected_ids.index(self._marker_id)
        selected = [corners[index]]
        camera_matrix = np.asarray(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(self._camera_info.d, dtype=np.float64)
        rotation_vectors, translation_vectors, _ = cv2.aruco.estimatePoseSingleMarkers(
            selected, self._marker_length, camera_matrix, distortion
        )
        rotation_camera_marker, _ = cv2.Rodrigues(rotation_vectors[0, 0])
        transform_camera_marker = np.eye(4)
        transform_camera_marker[:3, :3] = rotation_camera_marker
        transform_camera_marker[:3, 3] = translation_vectors[0, 0]

        marker_pose = PoseStamped()
        marker_pose.header = message.header
        _fill_pose(marker_pose, transform_camera_marker)
        self.pub_marker.publish(marker_pose)
        self._publish_tf(message.header.frame_id, self._marker_frame, marker_pose)

        if self._publish_robot:
            transform_marker_base = np.linalg.inv(
                transform_camera_marker
            ) @ np.linalg.inv(self._transform_base_camera)
            robot_pose = PoseStamped()
            robot_pose.header.stamp = message.header.stamp
            robot_pose.header.frame_id = self._marker_frame
            _fill_pose(robot_pose, transform_marker_base)
            self.pub_robot.publish(robot_pose)
            self._publish_tf(self._marker_frame, self._base_frame, robot_pose)

        cv2.aruco.drawDetectedMarkers(
            frame, selected, np.array([[self._marker_id]], dtype=np.int32)
        )
        cv2.drawFrameAxes(
            frame,
            camera_matrix,
            distortion,
            rotation_vectors[0, 0],
            translation_vectors[0, 0],
            self._marker_length * 0.5,
        )
        debug = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = message.header
        self.pub_debug.publish(debug)


def main() -> None:
    rclpy.init()
    node = ArucoPoseNode()
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
