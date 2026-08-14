"""Publish an RViz-only pool scene and short-lived robot pose ghosts."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import (
    add_vectors,
    finite_vector,
    inside_pool,
    normalized_quaternion,
    pool_edge_segments,
    quaternion_rotation_matrix,
    rotate_vector,
)


STATIC_TOPIC = "/brov/viz/pool"
VISION_TOPIC = "/brov/viz/vision_robot"
LOCALIZED_TOPIC = "/brov/viz/localized_robot"
POSE_TOPIC = "/brov/aruco/robot_pose_pool"
ODOMETRY_TOPIC = "/brov/localization/odometry_pool"
VISIBLE_TOPIC = "/brov/aruco/visible"


def _point(values) -> Point:
    x, y, z = finite_vector(values, 3, "point")
    return Point(x=x, y=y, z=z)


def _color(
    red: float, green: float, blue: float, alpha: float = 1.0
) -> ColorRGBA:
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


def _identity(marker: Marker) -> None:
    marker.pose.orientation.w = 1.0


def _base_marker(
    frame: str, namespace: str, marker_id: int, marker_type: int
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    _identity(marker)
    return marker


def _duration_message(seconds: float):
    return Duration(seconds=seconds).to_msg()


class PoolSceneNode(Node):
    """Visualize measurements without claiming any canonical TF edge."""

    def __init__(self) -> None:
        super().__init__("brov_pool_scene")

        self.declare_parameter("pool_frame", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pool_size_xyz", [4.0, 1.7, 1.1])
        self.declare_parameter("marker_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "marker_quaternion_xyzw", [0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter("marker_size_m", 0.0)
        self.declare_parameter("marker_label", "")
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter(
            "robot_visual_size_xyz", [0.45976624, 0.57254642, 0.40049667]
        )
        self.declare_parameter(
            "robot_visual_center_xyz",
            [-0.00036876, 0.00521591, -0.059202215],
        )

        self._pool_frame = str(self.get_parameter("pool_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        if not self._pool_frame or self._pool_frame.startswith("/"):
            raise ValueError(
                "pool_frame must be a non-empty relative frame name"
            )
        if not self._base_frame or self._base_frame.startswith("/"):
            raise ValueError("base_frame must be a non-empty relative frame name")
        self._pool_size = finite_vector(
            self.get_parameter("pool_size_xyz").value, 3, "pool_size_xyz"
        )
        if min(self._pool_size) <= 0.0:
            raise ValueError("pool_size_xyz entries must be positive")
        self._marker_xyz = finite_vector(
            self.get_parameter("marker_xyz").value, 3, "marker_xyz"
        )
        self._marker_quaternion = normalized_quaternion(
            self.get_parameter("marker_quaternion_xyzw").value
        )
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        if not math.isfinite(self._marker_size) or self._marker_size <= 0.0:
            raise ValueError("marker_size_m must be finite and positive")
        self._marker_label = str(self.get_parameter("marker_label").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        if not math.isfinite(self._pose_timeout) or self._pose_timeout <= 0.0:
            raise ValueError("pose_timeout_s must be finite and positive")
        self._robot_size = finite_vector(
            self.get_parameter("robot_visual_size_xyz").value,
            3,
            "robot_visual_size_xyz",
        )
        if min(self._robot_size) <= 0.0:
            raise ValueError("robot_visual_size_xyz entries must be positive")
        self._robot_center = finite_vector(
            self.get_parameter("robot_visual_center_xyz").value,
            3,
            "robot_visual_center_xyz",
        )

        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._static_publisher = self.create_publisher(
            MarkerArray, STATIC_TOPIC, static_qos
        )
        self._vision_publisher = self.create_publisher(
            MarkerArray, VISION_TOPIC, dynamic_qos
        )
        self._localized_publisher = self.create_publisher(
            MarkerArray, LOCALIZED_TOPIC, dynamic_qos
        )
        self.create_subscription(
            PoseStamped, POSE_TOPIC, self._on_pose, dynamic_qos
        )
        self.create_subscription(
            Bool, VISIBLE_TOPIC, self._on_visible, dynamic_qos
        )
        self.create_subscription(
            Odometry,
            ODOMETRY_TOPIC,
            self._on_localized_odometry,
            qos_profile_sensor_data,
        )

        self._vision_ghost_present = False
        self._localized_ghost_present = False
        self._last_vision_received_ns: int | None = None
        self._last_localized_received_ns: int | None = None
        self._static_publisher.publish(self._static_scene())
        self.create_timer(0.1, self._expire_stale_pose)
        self.get_logger().info(
            f"RViz pool scene ready: frame={self._pool_frame}, "
            f"bounds={self._pool_size}, pose={POSE_TOPIC}; no TF published"
        )

    def _static_scene(self) -> MarkerArray:
        markers: list[Marker] = []
        sx, sy, sz = self._pool_size

        wire = _base_marker(self._pool_frame, "pool", 0, Marker.LINE_LIST)
        wire.scale.x = 0.018
        wire.color = _color(0.1, 0.75, 1.0, 0.9)
        for start, end in pool_edge_segments(self._pool_size):
            wire.points.extend((_point(start), _point(end)))
        markers.append(wire)

        floor = _base_marker(self._pool_frame, "pool", 1, Marker.CUBE)
        floor.pose.position = _point((sx / 2.0, sy / 2.0, -0.006))
        floor.scale.x, floor.scale.y, floor.scale.z = sx, sy, 0.012
        floor.color = _color(0.15, 0.45, 0.7, 0.18)
        markers.append(floor)

        surface = _base_marker(self._pool_frame, "pool", 2, Marker.CUBE)
        surface.pose.position = _point((sx / 2.0, sy / 2.0, sz))
        surface.scale.x, surface.scale.y, surface.scale.z = sx, sy, 0.008
        surface.color = _color(0.2, 0.75, 1.0, 0.08)
        markers.append(surface)

        axes = _base_marker(self._pool_frame, "pool_axes", 0, Marker.LINE_LIST)
        axes.scale.x = 0.028
        axis_length = 0.5
        origin = (0.0, 0.0, 0.0)
        axis_data = (
            ((axis_length, 0.0, 0.0), _color(1.0, 0.1, 0.1)),
            ((0.0, axis_length, 0.0), _color(0.1, 1.0, 0.1)),
            ((0.0, 0.0, axis_length), _color(0.1, 0.35, 1.0)),
        )
        for endpoint, color in axis_data:
            axes.points.extend((_point(origin), _point(endpoint)))
            axes.colors.extend((color, color))
        markers.append(axes)

        pool_label = _base_marker(
            self._pool_frame, "labels", 0, Marker.TEXT_VIEW_FACING
        )
        pool_label.pose.position = _point((0.2, 0.1, sz + 0.12))
        pool_label.scale.z = 0.11
        pool_label.color = _color(0.7, 0.9, 1.0)
        pool_label.text = "POOL: +X FAR / +Y LEFT / +Z UP"
        markers.append(pool_label)

        tag = _base_marker(self._pool_frame, "apriltag", 0, Marker.CUBE)
        tag.pose.position = _point(self._marker_xyz)
        (
            tag.pose.orientation.x,
            tag.pose.orientation.y,
            tag.pose.orientation.z,
            tag.pose.orientation.w,
        ) = self._marker_quaternion
        tag.scale.x = self._marker_size
        tag.scale.y = self._marker_size
        tag.scale.z = 0.012
        tag.color = _color(0.04, 0.04, 0.04, 0.9)
        markers.append(tag)

        tag_axes = _base_marker(
            self._pool_frame, "apriltag_axes", 0, Marker.LINE_LIST
        )
        tag_axes.scale.x = 0.018
        tag_rotation = quaternion_rotation_matrix(self._marker_quaternion)
        tag_axis_length = 0.28
        for column, color in zip(
            range(3),
            (
                _color(1.0, 0.1, 0.1),
                _color(0.1, 1.0, 0.1),
                _color(0.1, 0.35, 1.0),
            ),
        ):
            direction = tuple(
                row[column] * tag_axis_length for row in tag_rotation
            )
            endpoint = add_vectors(self._marker_xyz, direction)
            tag_axes.points.extend(
                (_point(self._marker_xyz), _point(endpoint))
            )
            tag_axes.colors.extend((color, color))
        markers.append(tag_axes)

        tag_label = _base_marker(
            self._pool_frame, "labels", 1, Marker.TEXT_VIEW_FACING
        )
        tag_label.pose.position = _point(
            (
                self._marker_xyz[0] - 0.03,
                self._marker_xyz[1],
                self._marker_xyz[2] + 0.31,
            )
        )
        tag_label.scale.z = 0.09
        tag_label.color = _color(1.0, 0.85, 0.2)
        tag_label.text = (
            f"{self._marker_label}  black edge={self._marker_size:.2f} m"
        )
        markers.append(tag_label)
        return MarkerArray(markers=markers)

    def _on_visible(self, message: Bool) -> None:
        if not message.data:
            self._clear_vision_ghost()

    def _on_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id != self._pool_frame:
            self.get_logger().warning(
                f"rejecting vision pose in frame '{message.header.frame_id}'; "
                f"expected '{self._pool_frame}'"
            )
            self._clear_vision_ghost()
            return

        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        quaternion = (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        try:
            position = finite_vector(position, 3, "vision position")
            quaternion = normalized_quaternion(quaternion)
        except ValueError as exc:
            self.get_logger().warning(f"rejecting invalid vision pose: {exc}")
            self._clear_vision_ghost()
            return

        self._last_vision_received_ns = self.get_clock().now().nanoseconds
        self._vision_publisher.publish(
            self._vision_scene(message, position, quaternion)
        )
        self._vision_ghost_present = True

    def _on_localized_odometry(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self._pool_frame
            or message.child_frame_id != self._base_frame
        ):
            self.get_logger().warning(
                "rejecting localized odometry with unexpected frames: "
                f"{message.header.frame_id!r} -> {message.child_frame_id!r}"
            )
            self._clear_localized_ghost()
            return
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        quaternion = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        try:
            position = finite_vector(position, 3, "localized position")
            quaternion = normalized_quaternion(quaternion)
        except ValueError as exc:
            self.get_logger().warning(
                f"rejecting invalid localized odometry: {exc}"
            )
            self._clear_localized_ghost()
            return
        source = PoseStamped()
        source.header = message.header
        source.pose = message.pose.pose
        self._last_localized_received_ns = self.get_clock().now().nanoseconds
        self._localized_publisher.publish(
            self._localized_scene(source, position, quaternion)
        )
        self._localized_ghost_present = True

    def _vision_scene(
        self, source: PoseStamped, position, quaternion
    ) -> MarkerArray:
        markers: list[Marker] = []
        valid_bounds = inside_pool(position, self._pool_size)
        lifetime = _duration_message(self._pose_timeout)

        body = _base_marker(self._pool_frame, "vision_robot", 0, Marker.CUBE)
        body.header.stamp = source.header.stamp
        centre_offset = rotate_vector(quaternion, self._robot_center)
        body.pose.position = _point(add_vectors(position, centre_offset))
        (
            body.pose.orientation.x,
            body.pose.orientation.y,
            body.pose.orientation.z,
            body.pose.orientation.w,
        ) = quaternion
        body.scale.x, body.scale.y, body.scale.z = self._robot_size
        body.color = (
            _color(0.85, 0.15, 1.0, 0.35)
            if valid_bounds
            else _color(1.0, 0.05, 0.05, 0.55)
        )
        body.lifetime = lifetime
        markers.append(body)

        origin = _base_marker(
            self._pool_frame, "vision_robot", 1, Marker.SPHERE
        )
        origin.header.stamp = source.header.stamp
        origin.pose.position = _point(position)
        origin.scale.x = origin.scale.y = origin.scale.z = 0.07
        origin.color = _color(1.0, 1.0, 1.0, 0.9)
        origin.lifetime = lifetime
        markers.append(origin)

        axis_length = 0.35
        rotation = quaternion_rotation_matrix(quaternion)
        for axis, color in zip(
            range(3),
            (
                _color(1.0, 0.1, 0.1),
                _color(0.1, 1.0, 0.1),
                _color(0.1, 0.35, 1.0),
            ),
        ):
            arrow = _base_marker(
                self._pool_frame, "vision_robot_axes", axis, Marker.ARROW
            )
            arrow.header.stamp = source.header.stamp
            direction = tuple(row[axis] * axis_length for row in rotation)
            arrow.points = [
                _point(position),
                _point(add_vectors(position, direction)),
            ]
            arrow.scale.x = 0.022
            arrow.scale.y = 0.045
            arrow.scale.z = 0.065
            arrow.color = color
            arrow.lifetime = lifetime
            markers.append(arrow)

        label = _base_marker(
            self._pool_frame, "vision_robot", 2, Marker.TEXT_VIEW_FACING
        )
        label.header.stamp = source.header.stamp
        label.pose.position = _point(
            (position[0], position[1], position[2] + 0.33)
        )
        label.scale.z = 0.09
        label.color = (
            _color(1.0, 0.55, 1.0)
            if valid_bounds
            else _color(1.0, 0.1, 0.1)
        )
        state = "RAW VISION" if valid_bounds else "OUTSIDE NOMINAL POOL"
        label.text = (
            f"{state}\nbase_link x={position[0]:.2f} "
            f"y={position[1]:.2f} z={position[2]:.2f} m"
        )
        label.lifetime = lifetime
        markers.append(label)
        return MarkerArray(markers=markers)

    def _localized_scene(
        self, source: PoseStamped, position, quaternion
    ) -> MarkerArray:
        """Render the held pool alignment propagated by local odometry."""

        markers = self._vision_scene(source, position, quaternion).markers
        valid_bounds = inside_pool(position, self._pool_size)
        for marker in markers:
            marker.ns = marker.ns.replace("vision_robot", "localized_robot")
            if marker.type == Marker.CUBE:
                marker.color = (
                    _color(0.1, 0.45, 1.0, 0.45)
                    if valid_bounds
                    else _color(1.0, 0.05, 0.05, 0.55)
                )
            elif marker.type == Marker.TEXT_VIEW_FACING:
                marker.color = (
                    _color(0.25, 0.7, 1.0, 1.0)
                    if valid_bounds
                    else _color(1.0, 0.1, 0.1, 1.0)
                )
                marker.text = (
                    (
                        "ONE-SHOT ALIGNED ODOM\n"
                        if valid_bounds
                        else "ALIGNED ODOM OUTSIDE POOL\n"
                    )
                    + f"base_link x={position[0]:.2f} "
                    f"y={position[1]:.2f} z={position[2]:.2f} m"
                )
        return MarkerArray(markers=markers)

    def _expire_stale_pose(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_vision_received_ns is not None:
            elapsed = (now_ns - self._last_vision_received_ns) * 1.0e-9
            if elapsed > self._pose_timeout:
                self._clear_vision_ghost()
        if self._last_localized_received_ns is not None:
            elapsed = (now_ns - self._last_localized_received_ns) * 1.0e-9
            if elapsed > self._pose_timeout:
                self._clear_localized_ghost()

    def _clear_vision_ghost(self) -> None:
        self._last_vision_received_ns = None
        if not self._vision_ghost_present:
            return
        delete = Marker()
        delete.header.frame_id = self._pool_frame
        delete.action = Marker.DELETEALL
        self._vision_publisher.publish(MarkerArray(markers=[delete]))
        self._vision_ghost_present = False

    def _clear_localized_ghost(self) -> None:
        self._last_localized_received_ns = None
        if not self._localized_ghost_present:
            return
        delete = Marker()
        delete.header.frame_id = self._pool_frame
        delete.action = Marker.DELETEALL
        self._localized_publisher.publish(MarkerArray(markers=[delete]))
        self._localized_ghost_present = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PoolSceneNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
