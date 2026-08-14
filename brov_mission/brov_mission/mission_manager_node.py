#!/usr/bin/env python3
"""Validate pool paths and resolve one immutable mission into odom.

This node is deliberately outside the actuation path. It owns no MAVLink
connection, publishes no thruster command, and calls no control service.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
import uuid

from brov_interfaces.msg import (
    AlignedOdometry,
    LocalizationStatus,
    ResolvedMission,
)
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import (
    canonical_plan_content,
    CONTRACT_HEADING_MODES,
    invert_transform,
    MissionSettings,
    normalized_quaternion,
    POOL_POSITION_MISSION_V1,
    POOL_POSITION_MISSION_V2,
    quaternion,
    RANDOM_ATTITUDE_GENERATOR_VERSION,
    RANDOM_ATTITUDE_REFERENCE_FRAME,
    RandomAttitudeSettings,
    ValidationSettings,
    plan_hash,
    transform_points,
    validate_draft_geometry,
    validate_mission_settings,
    xyz,
)


def _stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _identity_pose_stamped(frame_id: str, stamp, point) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(point[0])
    pose.pose.position.y = float(point[1])
    pose.pose.position.z = float(point[2])
    pose.pose.orientation.w = 1.0
    return pose


def _path_content_signature(message: Path) -> tuple:
    """Compare draft semantics while deliberately ignoring ROS timestamps."""

    return (
        message.header.frame_id.strip(),
        tuple(
            (
                stamped_pose.header.frame_id.strip(),
                float(stamped_pose.pose.position.x),
                float(stamped_pose.pose.position.y),
                float(stamped_pose.pose.position.z),
                float(stamped_pose.pose.orientation.x),
                float(stamped_pose.pose.orientation.y),
                float(stamped_pose.pose.orientation.z),
                float(stamped_pose.pose.orientation.w),
            )
            for stamped_pose in message.poses
        ),
    )


class MissionManagerNode(Node):
    """Fail-closed draft validator and one-shot pool-to-odom resolver."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "brov_mission_manager", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("pool_frame", "pool")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "aligned_odometry_topic",
            "/brov/localization/odometry_pool_with_alignment",
        )
        self.declare_parameter("orientation_support_enabled", False)
        self.declare_parameter("identity_orientation_tolerance", 1e-3)
        self.declare_parameter("pool_safe_min_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pool_safe_max_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("max_first_point_distance_m", 0.3)
        self.declare_parameter("min_segment_length_m", 0.05)
        self.declare_parameter("max_segment_length_m", 4.0)
        self.declare_parameter("min_waypoints", 2)
        self.declare_parameter("max_waypoints", 50)
        self.declare_parameter("localization_max_age_s", 1.0)
        self.declare_parameter("odometry_max_age_s", 0.5)
        self.declare_parameter("cruise_speed", 0.1)
        self.declare_parameter("lookahead_dist", 0.4)
        self.declare_parameter("reach_threshold", 0.15)
        self.declare_parameter("max_cruise_speed", 0.3)
        self.declare_parameter("max_lookahead_dist", 1.0)
        self.declare_parameter("max_reach_threshold", 0.5)
        self.declare_parameter("contract_version", POOL_POSITION_MISSION_V1)
        self.declare_parameter("heading_mode", "straight")
        self.declare_parameter("loop", False)
        self.declare_parameter(
            "allowed_heading_modes", ["straight", "align"]
        )
        self.declare_parameter("random_attitude_seed", 0)
        self.declare_parameter(
            "random_attitude_reference_frame",
            RANDOM_ATTITUDE_REFERENCE_FRAME,
        )
        self.declare_parameter(
            "random_attitude_generator_version",
            RANDOM_ATTITUDE_GENERATOR_VERSION,
        )
        self.declare_parameter(
            "random_attitude_rpy_min_rad",
            [-math.pi / 2.0, -math.pi / 2.0, -math.pi],
        )
        self.declare_parameter(
            "random_attitude_rpy_max_rad",
            [math.pi / 2.0, math.pi / 2.0, math.pi],
        )
        self.declare_parameter("random_attitude_max_slew_rate_rad_s", 0.35)
        self.declare_parameter("random_attitude_tolerance_rad", 0.1745329252)
        self.declare_parameter(
            "random_attitude_angular_speed_tolerance_rad_s", 0.0872664626
        )
        self.declare_parameter("random_attitude_dwell_time_s", 2.0)
        self.declare_parameter("random_attitude_max_duration_s", 120.0)
        self.declare_parameter("random_attitude_max_laps", 1)

        self._pool_frame = str(self.get_parameter("pool_frame").value).strip()
        self._odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self._base_frame = str(self.get_parameter("base_frame").value).strip()
        if not self._pool_frame or not self._odom_frame or not self._base_frame:
            raise ValueError("pool_frame, odom_frame, and base_frame must be non-empty")
        if bool(self.get_parameter("orientation_support_enabled").value):
            raise ValueError(
                "orientation_support_enabled=true is not implemented; "
                "position-only missions fail closed on non-identity orientation"
            )

        self._contract_version = str(
            self.get_parameter("contract_version").value
        ).strip()
        contract_heading_modes = CONTRACT_HEADING_MODES.get(
            self._contract_version
        )
        if contract_heading_modes is None:
            raise ValueError(
                "contract_version must be exactly one of: "
                f"{sorted(CONTRACT_HEADING_MODES)}"
            )
        allowed_heading_modes = tuple(
            str(value).strip()
            for value in self.get_parameter("allowed_heading_modes").value
        )
        configured_heading_modes = set(allowed_heading_modes)
        modes_supported = configured_heading_modes.issubset(
            contract_heading_modes
        )
        if not configured_heading_modes or not modes_supported:
            raise ValueError(
                "allowed_heading_modes must be a non-empty subset of the "
                "selected resolved mission contract: "
                f"{sorted(contract_heading_modes)}"
            )

        self._validation_settings = ValidationSettings(
            safe_min_xyz=xyz(
                self.get_parameter("pool_safe_min_xyz").value,
                "pool_safe_min_xyz",
            ),
            safe_max_xyz=xyz(
                self.get_parameter("pool_safe_max_xyz").value,
                "pool_safe_max_xyz",
            ),
            max_first_point_distance_m=float(
                self.get_parameter("max_first_point_distance_m").value
            ),
            min_segment_length_m=float(
                self.get_parameter("min_segment_length_m").value
            ),
            identity_orientation_tolerance=float(
                self.get_parameter("identity_orientation_tolerance").value
            ),
            allowed_heading_modes=allowed_heading_modes,
        )
        random_attitude = None
        if self._contract_version == POOL_POSITION_MISSION_V2:
            random_attitude = RandomAttitudeSettings(
                seed=int(self.get_parameter("random_attitude_seed").value),
                reference_frame=str(
                    self.get_parameter(
                        "random_attitude_reference_frame"
                    ).value
                ).strip(),
                generator_version=str(
                    self.get_parameter(
                        "random_attitude_generator_version"
                    ).value
                ).strip(),
                rpy_min_rad=xyz(
                    self.get_parameter("random_attitude_rpy_min_rad").value,
                    "random_attitude_rpy_min_rad",
                ),
                rpy_max_rad=xyz(
                    self.get_parameter("random_attitude_rpy_max_rad").value,
                    "random_attitude_rpy_max_rad",
                ),
                max_slew_rate_rad_s=float(
                    self.get_parameter(
                        "random_attitude_max_slew_rate_rad_s"
                    ).value
                ),
                attitude_tolerance_rad=float(
                    self.get_parameter("random_attitude_tolerance_rad").value
                ),
                angular_speed_tolerance_rad_s=float(
                    self.get_parameter(
                        "random_attitude_angular_speed_tolerance_rad_s"
                    ).value
                ),
                dwell_time_s=float(
                    self.get_parameter("random_attitude_dwell_time_s").value
                ),
                max_duration_s=float(
                    self.get_parameter("random_attitude_max_duration_s").value
                ),
                max_laps=int(
                    self.get_parameter("random_attitude_max_laps").value
                ),
            )
        self._mission_settings = MissionSettings(
            cruise_speed=float(self.get_parameter("cruise_speed").value),
            lookahead_dist=float(self.get_parameter("lookahead_dist").value),
            reach_threshold=float(self.get_parameter("reach_threshold").value),
            heading_mode=str(self.get_parameter("heading_mode").value).strip(),
            loop=bool(self.get_parameter("loop").value),
            random_attitude=random_attitude,
        )
        self._min_waypoints = int(self.get_parameter("min_waypoints").value)
        self._max_waypoints = int(self.get_parameter("max_waypoints").value)
        self._max_segment_length_m = float(
            self.get_parameter("max_segment_length_m").value
        )
        self._mission_setting_limits = {
            "max_cruise_speed": float(
                self.get_parameter("max_cruise_speed").value
            ),
            "max_lookahead_dist": float(
                self.get_parameter("max_lookahead_dist").value
            ),
            "max_reach_threshold": float(
                self.get_parameter("max_reach_threshold").value
            ),
        }
        validate_mission_settings(
            self._mission_settings,
            self._validation_settings.allowed_heading_modes,
            contract_version=self._contract_version,
            **self._mission_setting_limits,
        )
        if self._min_waypoints < 2:
            raise ValueError("min_waypoints must be at least two")
        if self._max_waypoints < 2:
            raise ValueError("max_waypoints must be at least two")
        if self._min_waypoints > self._max_waypoints:
            raise ValueError("min_waypoints must not exceed max_waypoints")
        if (
            not math.isfinite(self._max_segment_length_m)
            or self._max_segment_length_m <= 0.0
        ):
            raise ValueError("max_segment_length_m must be finite and positive")

        for name in (
            "localization_max_age_s",
            "odometry_max_age_s",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_active_pool = self.create_publisher(
            Path, "/brov/mission/active_path_pool", latched
        )
        self._pub_resolved_odom = self.create_publisher(
            Path, "/brov/mission/resolved_path_odom", latched
        )
        self._pub_resolved = self.create_publisher(
            ResolvedMission, "/brov/mission/resolved", latched
        )
        self._pub_status = self.create_publisher(
            String, "/brov/mission/status", latched
        )

        draft_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Path, "/brov/mission/draft_path", self._on_draft, draft_qos
        )
        self.create_subscription(
            LocalizationStatus,
            "/brov/localization/status",
            self._on_localization_status,
            latched,
        )
        self.create_subscription(
            AlignedOdometry,
            str(self.get_parameter("aligned_odometry_topic").value),
            self._on_aligned_odometry,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, "/brov/mission/validate", self._on_validate
        )
        self.create_service(Trigger, "/brov/mission/commit", self._on_commit)

        self._draft: Path | None = None
        self._draft_signature: tuple | None = None
        self._draft_revision = 0
        self._localization: LocalizationStatus | None = None
        self._aligned_odometry: AlignedOdometry | None = None
        self._validated: dict | None = None
        self._active: dict | None = None
        self._publish_status("EMPTY", "waiting for a pool-frame draft")
        self.get_logger().info(
            "mission manager ready — validates pool drafts and resolves one "
            "immutable odom mission; no control/MAVLink interfaces"
        )

    def _publish_status(self, state: str, reason: str) -> None:
        payload = {
            "state": state,
            "reason": reason,
            "contract_version": self._contract_version,
            "draft_revision": self._draft_revision,
            "active_mission_id": (
                "" if self._active is None else self._active["mission_id"]
            ),
            "active_plan_hash": (
                "" if self._active is None else self._active["plan_hash"]
            ),
        }
        self._pub_status.publish(
            String(data=json.dumps(payload, sort_keys=True, separators=(",", ":")))
        )

    def _on_draft(self, message: Path) -> None:
        signature = _path_content_signature(message)
        if signature == self._draft_signature:
            # Editors commonly heartbeat a Path with a new stamp. Requiring a
            # second validation for identical semantic content creates a race
            # between validate and commit without adding safety.
            return
        self._draft = deepcopy(message)
        self._draft_signature = signature
        self._draft_revision += 1
        self._validated = None
        if self._active is None:
            self._publish_status("DRAFT", "draft received; validation required")
        else:
            self._publish_status(
                "ACTIVE_IMMUTABLE",
                "new draft stored but cannot replace this process's active mission",
            )

    def _on_localization_status(self, message: LocalizationStatus) -> None:
        self._localization = deepcopy(message)

    def _on_aligned_odometry(self, message: AlignedOdometry) -> None:
        self._aligned_odometry = deepcopy(message)

    def _age_seconds(self, stamp, name: str, maximum: float) -> float:
        stamp_ns = _stamp_nanoseconds(stamp)
        if stamp_ns <= 0:
            raise ValueError(f"{name} has a zero timestamp")
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -0.05:
            raise ValueError(f"{name} timestamp is in the future ({age:.3f}s)")
        if age > maximum:
            raise ValueError(f"{name} is stale ({age:.3f}s > {maximum:.3f}s)")
        return age

    def _localization_snapshot(self) -> dict:
        status = self._localization
        if status is None:
            raise ValueError("localization status has not been received")
        if status.header.frame_id.strip() != self._pool_frame:
            raise ValueError(
                f"localization status frame must be {self._pool_frame!r}"
            )
        if status.state != LocalizationStatus.INITIALIZED:
            raise ValueError(
                f"localization is not INITIALIZED (state={status.state}, "
                f"reason={status.reason!r})"
            )
        if not status.output_valid:
            raise ValueError("localization status output_valid is false")
        if int(status.epoch) <= 0:
            raise ValueError("initialized localization epoch must be non-zero")
        session_id = status.odometry_session_id.strip()
        if not session_id:
            raise ValueError("odometry_session_id must be non-empty")
        alignment_id = status.alignment_id.strip()
        if not alignment_id:
            raise ValueError("initialized localization alignment_id is empty")
        transform = status.pool_to_odom
        translation = xyz(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            "pool_to_odom translation",
        )
        rotation = quaternion(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
            "pool_to_odom rotation",
        )
        norm = math.sqrt(sum(component * component for component in rotation))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError(
                f"pool_to_odom quaternion norm invalid ({norm:.6g})"
            )
        rotation = normalized_quaternion(rotation)
        self._age_seconds(
            status.header.stamp,
            "localization status",
            float(self.get_parameter("localization_max_age_s").value),
        )
        return {
            "epoch": int(status.epoch),
            "odometry_session_id": session_id,
            "alignment_id": alignment_id,
            # This is ^pool T_odom, i.e. the exact frozen localization result,
            # carried atomically with its alignment identity.
            "pool_to_odom_translation": translation,
            "pool_to_odom_rotation": rotation,
        }

    def _current_pool_position(
        self, localization: dict
    ) -> tuple[float, float, float]:
        envelope = self._aligned_odometry
        if envelope is None:
            raise ValueError("aligned pool odometry has not been received")
        if int(envelope.localization_epoch) != localization["epoch"]:
            raise ValueError(
                "aligned odometry localization epoch does not match status"
            )
        if (
            envelope.odometry_session_id.strip()
            != localization["odometry_session_id"]
        ):
            raise ValueError(
                "aligned odometry session does not match localization status"
            )
        if envelope.alignment_id.strip() != localization["alignment_id"]:
            raise ValueError(
                "aligned odometry alignment does not match localization status"
            )
        odometry = envelope.odometry
        if odometry.header.frame_id.strip() != self._pool_frame:
            raise ValueError(
                f"pool odometry frame must be {self._pool_frame!r}"
            )
        if odometry.child_frame_id.strip() != self._base_frame:
            raise ValueError(
                f"pool odometry child_frame_id must be {self._base_frame!r}"
            )
        self._age_seconds(
            odometry.header.stamp,
            "pool odometry",
            float(self.get_parameter("odometry_max_age_s").value),
        )
        position = odometry.pose.pose.position
        return xyz((position.x, position.y, position.z), "current pool position")

    def _validated_snapshot(self) -> dict:
        draft = self._draft
        if draft is None:
            raise ValueError("draft path has not been received")
        if draft.header.frame_id.strip() != self._pool_frame:
            raise ValueError(
                f"draft Path.header.frame_id must be {self._pool_frame!r}"
            )

        positions = []
        orientations = []
        for index, stamped_pose in enumerate(draft.poses):
            pose_frame = stamped_pose.header.frame_id.strip()
            if pose_frame and pose_frame != self._pool_frame:
                raise ValueError(
                    f"draft pose[{index}] frame must be empty or "
                    f"{self._pool_frame!r}"
                )
            position = stamped_pose.pose.position
            orientation = stamped_pose.pose.orientation
            positions.append((position.x, position.y, position.z))
            orientations.append(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )

        localization = self._localization_snapshot()
        points = validate_draft_geometry(
            positions,
            orientations,
            self._current_pool_position(localization),
            self._validation_settings,
            min_waypoints=self._min_waypoints,
            max_waypoints=self._max_waypoints,
            max_segment_length_m=self._max_segment_length_m,
            loop=bool(self._mission_settings.loop),
        )
        validate_mission_settings(
            self._mission_settings,
            self._validation_settings.allowed_heading_modes,
            contract_version=self._contract_version,
            **self._mission_setting_limits,
        )
        return {
            "revision": self._draft_revision,
            "draft": deepcopy(draft),
            "points": points,
            "localization_epoch": localization["epoch"],
            "odometry_session_id": localization["odometry_session_id"],
            "alignment_id": localization["alignment_id"],
            "pool_to_odom_translation": localization[
                "pool_to_odom_translation"
            ],
            "pool_to_odom_rotation": localization["pool_to_odom_rotation"],
            "plan_hash": plan_hash(
                points,
                self._mission_settings,
                frame_id=self._pool_frame,
                contract_version=self._contract_version,
            ),
        }

    def _on_validate(self, _request, response):
        try:
            candidate = self._validated_snapshot()
        except ValueError as error:
            self._validated = None
            self._publish_status("INVALID", str(error))
            response.success = False
            response.message = str(error)
            return response
        self._validated = candidate
        short_hash = candidate["plan_hash"][:12]
        response.success = True
        response.message = (
            f"draft revision {candidate['revision']} valid; "
            f"plan_hash={short_hash}; explicit commit required"
        )
        self._publish_status("VALIDATED", response.message)
        return response

    def _same_localization(self, candidate: dict) -> bool:
        try:
            current = self._localization_snapshot()
        except ValueError:
            return False
        return (
            current["epoch"] == candidate["localization_epoch"]
            and current["odometry_session_id"]
            == candidate["odometry_session_id"]
            and current["alignment_id"] == candidate["alignment_id"]
            and current["pool_to_odom_translation"]
            == candidate["pool_to_odom_translation"]
            and current["pool_to_odom_rotation"]
            == candidate["pool_to_odom_rotation"]
        )

    def _on_commit(self, _request, response):
        if self._validated is None:
            response.success = False
            response.message = "current draft has not passed /brov/mission/validate"
            self._publish_status("COMMIT_REJECTED", response.message)
            return response

        try:
            candidate = self._validated_snapshot()
        except ValueError as error:
            self._validated = None
            response.success = False
            response.message = f"commit revalidation failed: {error}"
            self._publish_status("COMMIT_REJECTED", response.message)
            return response

        validated = self._validated
        validation_key = (
            "revision",
            "plan_hash",
            "localization_epoch",
            "odometry_session_id",
            "alignment_id",
            "pool_to_odom_translation",
            "pool_to_odom_rotation",
        )
        if any(candidate[key] != validated[key] for key in validation_key):
            self._validated = None
            response.success = False
            response.message = (
                "draft or localization changed after validation; validate again"
            )
            self._publish_status("COMMIT_REJECTED", response.message)
            return response

        if self._active is not None:
            identity_key = (
                "plan_hash",
                "localization_epoch",
                "odometry_session_id",
                "alignment_id",
            )
            if all(
                candidate[key] == self._active[key] for key in identity_key
            ):
                response.success = True
                response.message = (
                    f"mission already committed: {self._active['mission_id']}"
                )
            else:
                response.success = False
                response.message = (
                    "active mission is immutable; stop the consumer and restart "
                    "the mission manager before committing another revision"
                )
            self._publish_status("ACTIVE_IMMUTABLE", response.message)
            return response

        try:
            # Status carries ^pool T_odom. Resolve pool points into odom using
            # its inverse, bound to the same alignment_id in one message.
            translation, rotation = invert_transform(
                candidate["pool_to_odom_translation"],
                candidate["pool_to_odom_rotation"],
            )
            resolved_points = transform_points(
                candidate["points"],
                translation,
                rotation,
            )
        except ValueError as error:
            response.success = False
            response.message = str(error)
            self._publish_status("COMMIT_REJECTED", response.message)
            return response
        if not self._same_localization(candidate):
            self._validated = None
            response.success = False
            response.message = (
                "localization changed while resolving the mission; validate again"
            )
            self._publish_status("COMMIT_REJECTED", response.message)
            return response

        commit_stamp = self.get_clock().now().to_msg()
        active_pool = Path()
        active_pool.header.frame_id = self._pool_frame
        active_pool.header.stamp = commit_stamp
        active_pool.poses = [
            _identity_pose_stamped(self._pool_frame, commit_stamp, point)
            for point in candidate["points"]
        ]
        resolved_odom = Path()
        resolved_odom.header.frame_id = self._odom_frame
        resolved_odom.header.stamp = commit_stamp
        resolved_odom.poses = [
            _identity_pose_stamped(self._odom_frame, commit_stamp, point)
            for point in resolved_points
        ]

        mission_id = str(uuid.uuid4())
        resolved = ResolvedMission()
        resolved.header.frame_id = self._odom_frame
        resolved.header.stamp = commit_stamp
        resolved.mission_id = mission_id
        resolved.plan_hash = candidate["plan_hash"]
        resolved.contract_version = self._contract_version
        resolved.canonical_plan_json = canonical_plan_content(
            candidate["points"],
            self._mission_settings,
            frame_id=self._pool_frame,
            contract_version=self._contract_version,
        ).decode("ascii")
        resolved.localization_epoch = candidate["localization_epoch"]
        resolved.odometry_session_id = candidate["odometry_session_id"]
        resolved.alignment_id = candidate["alignment_id"]
        resolved.waypoints = [
            Point(x=point[0], y=point[1], z=point[2])
            for point in resolved_points
        ]
        resolved.cruise_speed = self._mission_settings.cruise_speed
        resolved.lookahead_dist = self._mission_settings.lookahead_dist
        resolved.reach_threshold = self._mission_settings.reach_threshold
        resolved.heading_mode = self._mission_settings.heading_mode
        resolved.loop = self._mission_settings.loop

        self._pub_active_pool.publish(active_pool)
        self._pub_resolved_odom.publish(resolved_odom)
        self._pub_resolved.publish(resolved)
        self._active = {
            "mission_id": mission_id,
            "plan_hash": candidate["plan_hash"],
            "contract_version": self._contract_version,
            "revision": candidate["revision"],
            "localization_epoch": candidate["localization_epoch"],
            "odometry_session_id": candidate["odometry_session_id"],
            "alignment_id": candidate["alignment_id"],
        }
        response.success = True
        response.message = (
            f"committed immutable mission {mission_id}; "
            f"plan_hash={candidate['plan_hash'][:12]}; this node does not start control"
        )
        self._publish_status("COMMITTED", response.message)
        return response


def main() -> None:
    rclpy.init()
    node = MissionManagerNode()
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
