#!/usr/bin/env python3
"""Estimate ArUco marker pose and optionally robot pose from calibrated images."""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .geometry import (
    matrix_from_xyz_quaternion,
    matrix_from_xyz_rpy,
    quaternion_from_matrix,
)


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


def _resolve_dictionary(dictionary_name: str):
    """Return a predefined OpenCV dictionary or fail on an empty/unknown name."""
    dictionary_name = dictionary_name.strip()
    if not dictionary_name or not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unsupported ArUco dictionary: {dictionary_name!r}")
    return cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dictionary_name)
    )


def _dictionary_capacity(dictionary) -> int:
    bytes_list = np.asarray(dictionary.bytesList)
    if bytes_list.ndim < 1 or bytes_list.shape[0] <= 0:
        raise ValueError("selected dictionary contains no marker codes")
    return int(bytes_list.shape[0])


def _validate_marker_contract(
    dictionary, marker_id: int, marker_length_m: float
) -> None:
    capacity = _dictionary_capacity(dictionary)
    if not 0 <= marker_id < capacity:
        raise ValueError(
            f"marker_id must be in [0, {capacity - 1}], got {marker_id}"
        )
    if not math.isfinite(marker_length_m) or marker_length_m <= 0.0:
        raise ValueError("marker_length_m must be finite and positive")


def _validated_extrinsic(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build finite ``^base T_camera`` from metres and fixed-axis RPY."""
    values = np.asarray([*xyz, *rpy], dtype=np.float64)
    if len(xyz) != 3 or len(rpy) != 3 or values.size != 6:
        raise ValueError("base_to_camera xyz and rpy must each contain 3 values")
    if not np.all(np.isfinite(values)):
        raise ValueError("base_to_camera xyz and rpy must be finite")
    transform = matrix_from_xyz_rpy(xyz, rpy)
    if not np.all(np.isfinite(transform)):
        raise ValueError("base_to_camera transform is non-finite")
    return transform


def _marker_to_base_transform(
    transform_camera_marker: np.ndarray,
    transform_base_camera: np.ndarray,
) -> np.ndarray:
    """Compose ``^marker T_base`` from ``^camera T_marker`` and extrinsic."""
    camera_marker = np.asarray(transform_camera_marker, dtype=np.float64)
    base_camera = np.asarray(transform_base_camera, dtype=np.float64)
    if camera_marker.shape != (4, 4) or base_camera.shape != (4, 4):
        raise ValueError("rigid transforms must have shape (4, 4)")
    if not np.all(np.isfinite(camera_marker)) or not np.all(
        np.isfinite(base_camera)
    ):
        raise ValueError("rigid transforms must be finite")
    return np.linalg.inv(camera_marker) @ np.linalg.inv(base_camera)


def _validated_pool_marker_transform(
    xyz: list[float], quaternion_xyzw: list[float]
) -> np.ndarray:
    """Build the surveyed ``^pool T_marker`` rigid transform."""
    try:
        return matrix_from_xyz_quaternion(xyz, quaternion_xyzw)
    except ValueError as error:
        raise ValueError(f"invalid pool-to-marker transform: {error}") from error


def _pool_to_base_transform(
    transform_pool_marker: np.ndarray,
    transform_camera_marker: np.ndarray,
    transform_base_camera: np.ndarray,
) -> np.ndarray:
    """Compose ``^pool T_base`` from survey, detection, and extrinsic."""
    pool_marker = np.asarray(transform_pool_marker, dtype=np.float64)
    if pool_marker.shape != (4, 4) or not np.all(np.isfinite(pool_marker)):
        raise ValueError("pool-to-marker transform must be a finite 4x4 matrix")
    return pool_marker @ _marker_to_base_transform(
        transform_camera_marker, transform_base_camera
    )


def _detector_parameters(corner_refinement: str):
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    refinement_name = corner_refinement.strip().upper()
    refinement_constants = {
        "NONE": "CORNER_REFINE_NONE",
        "SUBPIX": "CORNER_REFINE_SUBPIX",
        "CONTOUR": "CORNER_REFINE_CONTOUR",
        "APRILTAG": "CORNER_REFINE_APRILTAG",
    }
    constant_name = refinement_constants.get(refinement_name)
    if constant_name is None or not hasattr(cv2.aruco, constant_name):
        choices = ", ".join(refinement_constants)
        raise ValueError(
            f"unsupported corner_refinement {corner_refinement!r}; "
            f"choose one of: {choices}"
        )
    parameters.cornerRefinementMethod = getattr(cv2.aruco, constant_name)
    return parameters


def _camera_model(message: CameraInfo | None):
    """Return a finite calibrated camera model, otherwise None."""
    if message is None:
        return None
    camera_matrix = np.asarray(message.k, dtype=np.float64)
    distortion = np.asarray(message.d, dtype=np.float64)
    if camera_matrix.size != 9:
        return None
    camera_matrix = camera_matrix.reshape(3, 3)
    if not np.all(np.isfinite(camera_matrix)):
        return None
    if not np.all(np.isfinite(distortion)):
        return None
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        return None
    return camera_matrix, distortion


def _estimate_marker_transform(
    corners, marker_length_m: float, camera_matrix, distortion
):
    """Estimate camera-to-marker transform from one marker's image corners."""
    rotation_vectors, translation_vectors, _ = (
        cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length_m, camera_matrix, distortion
        )
    )
    if not np.all(np.isfinite(rotation_vectors)) or not np.all(
        np.isfinite(translation_vectors)
    ):
        raise ValueError("non-finite marker pose")

    rotation_vector = rotation_vectors[0, 0]
    translation_vector = translation_vectors[0, 0]
    rotation_camera_marker, _ = cv2.Rodrigues(rotation_vector)
    transform_camera_marker = np.eye(4)
    transform_camera_marker[:3, :3] = rotation_camera_marker
    transform_camera_marker[:3, 3] = translation_vector
    return transform_camera_marker, rotation_vector, translation_vector


class ArucoPoseNode(Node):
    """Publish a selected marker's metric pose and corresponding TF."""

    def __init__(self) -> None:
        super().__init__("brov_aruco_pose_node")
        # Physical marker parameters intentionally fail closed without YAML.
        self.declare_parameter("dictionary", "")
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("marker_length_m", 0.0)
        self.declare_parameter("corner_refinement", "SUBPIX")
        self.declare_parameter("marker_frame", "aruco_reference")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_marker_tf", False)
        self.declare_parameter("publish_robot_pose", False)
        self.declare_parameter("publish_robot_tf", False)
        self.declare_parameter("publish_pool_pose", False)
        self.declare_parameter("pool_frame", "pool")
        self.declare_parameter("pool_to_marker_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "pool_to_marker_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter("base_to_camera_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_camera_rpy", [0.0, 0.0, 0.0])

        dictionary_name = str(self.get_parameter("dictionary").value)
        self._dictionary = _resolve_dictionary(dictionary_name)
        self._marker_id = int(self.get_parameter("marker_id").value)
        self._marker_length = float(self.get_parameter("marker_length_m").value)
        corner_refinement = str(
            self.get_parameter("corner_refinement").value
        )
        _validate_marker_contract(
            self._dictionary, self._marker_id, self._marker_length
        )
        self._detector_parameters = _detector_parameters(corner_refinement)
        self._marker_frame = str(self.get_parameter("marker_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_marker_tf = bool(
            self.get_parameter("publish_marker_tf").value
        )
        self._publish_robot = bool(self.get_parameter("publish_robot_pose").value)
        self._publish_robot_tf = bool(
            self.get_parameter("publish_robot_tf").value
        )
        self._publish_pool = bool(self.get_parameter("publish_pool_pose").value)
        self._pool_frame = str(self.get_parameter("pool_frame").value).strip()
        if not self._pool_frame:
            raise ValueError("pool_frame must be non-empty")
        pool_marker_xyz = [
            float(value)
            for value in self.get_parameter("pool_to_marker_xyz").value
        ]
        pool_marker_quaternion = [
            float(value)
            for value in self.get_parameter(
                "pool_to_marker_quaternion_xyzw"
            ).value
        ]
        self._transform_pool_marker = _validated_pool_marker_transform(
            pool_marker_xyz, pool_marker_quaternion
        )
        xyz = [float(value) for value in self.get_parameter("base_to_camera_xyz").value]
        rpy = [float(value) for value in self.get_parameter("base_to_camera_rpy").value]
        self._transform_base_camera = _validated_extrinsic(xyz, rpy)
        if self._publish_robot_tf and not self._publish_robot:
            raise ValueError("publish_robot_tf=true requires publish_robot_pose=true")

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._warned_uncalibrated = False
        self._last_visible: bool | None = None
        self._tf = TransformBroadcaster(self)
        self.pub_marker = self.create_publisher(
            PoseStamped, "/brov/aruco/marker_pose", 10
        )
        self.pub_robot = self.create_publisher(
            PoseStamped, "/brov/aruco/robot_pose", 10
        )
        self.pub_pool_robot = self.create_publisher(
            PoseStamped, "/brov/aruco/robot_pose_pool", 10
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
            f"waiting for {dictionary_name} id={self._marker_id}, "
            f"black-edge length={self._marker_length:.3f} m, "
            f"corner_refinement={corner_refinement.upper()}"
        )
        if self._publish_robot:
            self.get_logger().warning(
                "robot pose uses the nominal static base-to-camera extrinsic; "
                "camera tilt must remain locked at neutral"
            )
            self.get_logger().info(
                f"^base T_camera xyz={xyz}, rpy={rpy}; "
                f"marker-to-base TF broadcast={self._publish_robot_tf}"
            )
        if self._publish_pool:
            self.get_logger().info(
                f"surveyed ^{self._pool_frame} T_marker "
                f"xyz={pool_marker_xyz}, q_xyzw={pool_marker_quaternion}; "
                "publishing raw single-frame base pose on "
                "/brov/aruco/robot_pose_pool without TF"
            )

    def _on_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        if _camera_model(message) is not None:
            self._warned_uncalibrated = False

    def _publish_visibility(self, visible: bool) -> None:
        self.pub_visible.publish(Bool(data=visible))
        if visible and self._last_visible is not True:
            self.get_logger().info(f"marker id={self._marker_id} acquired")
        elif not visible and self._last_visible is True:
            self.get_logger().warning(f"marker id={self._marker_id} lost")
        self._last_visible = visible

    def _publish_debug(
        self,
        message: Image,
        frame: np.ndarray,
        corners,
        ids,
        status: str,
        visible: bool,
    ) -> None:
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        color = (0, 200, 0) if visible else (0, 0, 255)
        cv2.putText(
            frame,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        debug = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = message.header
        self.pub_debug.publish(debug)

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
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._detector_parameters
        )
        detected_ids = [] if ids is None else ids.flatten().tolist()
        visible = self._marker_id in detected_ids
        self._publish_visibility(visible)
        if not visible:
            self._publish_debug(
                message,
                frame,
                corners,
                ids,
                f"target id={self._marker_id}: not visible",
                False,
            )
            return

        camera_model = _camera_model(self._camera_info)
        if camera_model is None:
            if not self._warned_uncalibrated:
                self._warned_uncalibrated = True
                self.get_logger().warning(
                    "marker detected, but valid camera intrinsics are required "
                    "for metric relative pose"
                )
            self._publish_debug(
                message,
                frame,
                corners,
                ids,
                f"target id={self._marker_id}: visible; pose unavailable",
                True,
            )
            return

        index = detected_ids.index(self._marker_id)
        selected = [corners[index]]
        camera_matrix, distortion = camera_model
        try:
            (
                transform_camera_marker,
                rotation_vector,
                translation_vector,
            ) = _estimate_marker_transform(
                selected,
                self._marker_length,
                camera_matrix,
                distortion,
            )
        except ValueError:
            self.get_logger().error("non-finite marker pose rejected")
            self._publish_debug(
                message,
                frame,
                corners,
                ids,
                f"target id={self._marker_id}: invalid pose",
                True,
            )
            return

        marker_pose = PoseStamped()
        marker_pose.header = message.header
        _fill_pose(marker_pose, transform_camera_marker)
        self.pub_marker.publish(marker_pose)
        if self._publish_marker_tf:
            self._publish_tf(
                message.header.frame_id, self._marker_frame, marker_pose
            )

        if self._publish_robot or self._publish_pool:
            transform_marker_base = _marker_to_base_transform(
                transform_camera_marker, self._transform_base_camera
            )
        if self._publish_robot:
            robot_pose = PoseStamped()
            robot_pose.header.stamp = message.header.stamp
            robot_pose.header.frame_id = self._marker_frame
            _fill_pose(robot_pose, transform_marker_base)
            self.pub_robot.publish(robot_pose)
            if self._publish_robot_tf:
                self._publish_tf(
                    self._marker_frame, self._base_frame, robot_pose
                )
        if self._publish_pool:
            transform_pool_base = _pool_to_base_transform(
                self._transform_pool_marker,
                transform_camera_marker,
                self._transform_base_camera,
            )
            pool_robot_pose = PoseStamped()
            pool_robot_pose.header.stamp = message.header.stamp
            pool_robot_pose.header.frame_id = self._pool_frame
            _fill_pose(pool_robot_pose, transform_pool_base)
            self.pub_pool_robot.publish(pool_robot_pose)

        cv2.drawFrameAxes(
            frame,
            camera_matrix,
            distortion,
            rotation_vector,
            translation_vector,
            self._marker_length * 0.5,
        )
        range_m = float(np.linalg.norm(translation_vector))
        self._publish_debug(
            message,
            frame,
            corners,
            ids,
            f"target id={self._marker_id}: pose range={range_m:.2f} m",
            True,
        )


def main() -> None:
    rclpy.init()
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
