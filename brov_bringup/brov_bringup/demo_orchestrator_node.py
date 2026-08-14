#!/usr/bin/env python3
"""
Operator-facing orchestration for one pool-localized demo run.

The node does not replace any localization, mission, controller, or actuator
gate.  It calls their public services in the required order and collapses the
normal Case-A workflow into PREPARE, START, and STOP operations.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
import threading
import time

from brov_interfaces.msg import AlignedOdometry, LocalizationStatus
from brov_interfaces.srv import InitializePool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger


class DemoOperationError(RuntimeError):
    """Expected fail-closed rejection surfaced to the operator."""


def _finite_triplet(value, name: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} values must be finite")
    return result


def case_a_points(
    current_position,
    safe_min_xyz,
    safe_max_xyz,
    segment_length_m: float,
    max_entry_distance_m: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Generate a short Case-A line while preserving the mission gates."""
    current = _finite_triplet(current_position, "current_position")
    minimum = _finite_triplet(safe_min_xyz, "safe_min_xyz")
    maximum = _finite_triplet(safe_max_xyz, "safe_max_xyz")
    if any(lower >= upper for lower, upper in zip(minimum, maximum)):
        raise ValueError("each safe_min_xyz value must be below safe_max_xyz")
    length = float(segment_length_m)
    entry_limit = float(max_entry_distance_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("segment_length_m must be finite and positive")
    if not math.isfinite(entry_limit) or entry_limit <= 0.0:
        raise ValueError("max_entry_distance_m must be finite and positive")

    first = tuple(
        min(max(value, lower), upper)
        for value, lower, upper in zip(current, minimum, maximum)
    )
    entry_distance = math.dist(current, first)
    if entry_distance > entry_limit:
        raise ValueError(
            f"nearest safe first waypoint is {entry_distance:.3f} m from "
            f"the current pose (limit {entry_limit:.3f} m)"
        )

    centre_x = 0.5 * (minimum[0] + maximum[0])
    direction = 1.0 if first[0] <= centre_x else -1.0
    second_x = first[0] + direction * length
    if second_x < minimum[0] or second_x > maximum[0]:
        direction *= -1.0
        second_x = first[0] + direction * length
    if second_x < minimum[0] or second_x > maximum[0]:
        raise ValueError("Case-A segment does not fit inside the pool safe box")
    second = (second_x, first[1], first[2])
    return first, second


def _path_points(message: Path) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        )
        for pose in message.poses
    )


class DemoOrchestratorNode(Node):
    """Coordinate existing fail-closed services without owning actuation."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "brov_demo_orchestrator", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("controller", "rl")
        self.declare_parameter("demo_case", "a")
        self.declare_parameter("auto_generate_case_a_path", True)
        self.declare_parameter("pool_frame", "pool")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("safe_min_xyz", [0.35, 0.30, 0.20])
        self.declare_parameter("safe_max_xyz", [3.65, 1.40, 0.90])
        self.declare_parameter("case_a_segment_length_m", 0.20)
        self.declare_parameter("max_entry_distance_m", 0.30)
        self.declare_parameter("localization_min_samples", 20)
        self.declare_parameter("service_wait_timeout_s", 5.0)
        self.declare_parameter("service_call_timeout_s", 12.0)
        self.declare_parameter("localization_timeout_s", 30.0)
        self.declare_parameter("first_pwm_timeout_s", 6.0)

        self._controller = str(
            self.get_parameter("controller").value
        ).strip().lower()
        if self._controller not in {"model", "rl"}:
            raise ValueError("controller must be exactly 'model' or 'rl'")
        self._demo_case = str(
            self.get_parameter("demo_case").value
        ).strip().lower()
        if self._demo_case not in {"a", "c"}:
            raise ValueError("demo_case must be exactly 'a' or 'c'")
        self._auto_path = bool(
            self.get_parameter("auto_generate_case_a_path").value
        )
        self._pool_frame = str(
            self.get_parameter("pool_frame").value
        ).strip()
        self._base_frame = str(
            self.get_parameter("base_frame").value
        ).strip()
        if not self._pool_frame or not self._base_frame:
            raise ValueError("pool_frame and base_frame must be non-empty")
        self._safe_min = _finite_triplet(
            self.get_parameter("safe_min_xyz").value, "safe_min_xyz"
        )
        self._safe_max = _finite_triplet(
            self.get_parameter("safe_max_xyz").value, "safe_max_xyz"
        )
        self._segment_length = float(
            self.get_parameter("case_a_segment_length_m").value
        )
        self._max_entry_distance = float(
            self.get_parameter("max_entry_distance_m").value
        )
        self._minimum_samples = int(
            self.get_parameter("localization_min_samples").value
        )
        if self._minimum_samples < 1:
            raise ValueError("localization_min_samples must be positive")
        self._service_wait_timeout = self._positive_parameter(
            "service_wait_timeout_s"
        )
        self._service_call_timeout = self._positive_parameter(
            "service_call_timeout_s"
        )
        self._localization_timeout = self._positive_parameter(
            "localization_timeout_s"
        )
        self._first_pwm_timeout = self._positive_parameter(
            "first_pwm_timeout_s"
        )

        self._callback_group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._operation_lock = threading.Lock()
        self._localization: LocalizationStatus | None = None
        self._aligned_odometry: AlignedOdometry | None = None
        self._active_path: Path | None = None
        self._control_active = False
        self._last_pwm_monotonic: float | None = None
        self._prepared = False
        self._active = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        draft_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub_draft = self.create_publisher(
            Path, "/brov/mission/draft_path", draft_qos
        )
        self._pub_status = self.create_publisher(
            String, "/brov/demo/status", latched
        )
        self.create_subscription(
            LocalizationStatus,
            "/brov/localization/status",
            self._on_localization,
            latched,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            AlignedOdometry,
            "/brov/localization/odometry_pool_with_alignment",
            self._on_aligned_odometry,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Path,
            "/brov/mission/active_path_pool",
            self._on_active_path,
            latched,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            "/brov/control_active",
            self._on_control_active,
            1,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Float32MultiArray,
            "/brov/thruster_pwm",
            self._on_pwm,
            1,
            callback_group=self._callback_group,
        )

        self._service_clients = {
            "confirm_neutral": self.create_client(
                Trigger,
                "/brov/localization/confirm_camera_tilt_neutral",
                callback_group=self._callback_group,
            ),
            "initialize": self.create_client(
                InitializePool,
                "/brov/localization/initialize_pool",
                callback_group=self._callback_group,
            ),
            "validate": self.create_client(
                Trigger,
                "/brov/mission/validate",
                callback_group=self._callback_group,
            ),
            "commit": self.create_client(
                Trigger,
                "/brov/mission/commit",
                callback_group=self._callback_group,
            ),
            "prepare": self.create_client(
                Trigger,
                "/brov/prepare_control",
                callback_group=self._callback_group,
            ),
            "arm": self.create_client(
                Trigger,
                "/brov/arm_control",
                callback_group=self._callback_group,
            ),
            "start": self.create_client(
                Trigger,
                "/brov/start_control",
                callback_group=self._callback_group,
            ),
            "stop": self.create_client(
                Trigger,
                "/brov/stop_control",
                callback_group=self._callback_group,
            ),
            "disarm": self.create_client(
                Trigger,
                "/brov/disarm_control",
                callback_group=self._callback_group,
            ),
        }
        if self._controller == "model":
            self._service_clients["controller_start"] = self.create_client(
                Trigger,
                "/brov/model_based/start",
                callback_group=self._callback_group,
            )
            self._service_clients["controller_stop"] = self.create_client(
                Trigger,
                "/brov/model_based/stop",
                callback_group=self._callback_group,
            )

        self.create_service(
            Trigger,
            "/brov/demo/prepare",
            self._on_prepare,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/brov/demo/start",
            self._on_start,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/brov/demo/stop",
            self._on_stop,
            callback_group=self._callback_group,
        )
        self._publish_status("IDLE", "call /brov/demo/prepare while disarmed")
        self.get_logger().info(
            "demo orchestrator ready — operator API: PREPARE, START, STOP; "
            "all underlying safety gates remain authoritative"
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _publish_status(self, state: str, reason: str) -> None:
        message = {
            "state": state,
            "reason": reason,
            "case": self._demo_case,
            "controller": self._controller,
            "prepared": self._prepared,
            "active": self._active,
        }
        self._pub_status.publish(
            String(data=json.dumps(message, sort_keys=True, separators=(",", ":")))
        )

    def _notify(self) -> None:
        with self._condition:
            self._condition.notify_all()

    def _on_localization(self, message: LocalizationStatus) -> None:
        self._localization = deepcopy(message)
        self._notify()

    def _on_aligned_odometry(self, message: AlignedOdometry) -> None:
        self._aligned_odometry = deepcopy(message)
        self._notify()

    def _on_active_path(self, message: Path) -> None:
        self._active_path = deepcopy(message)
        self._notify()

    def _on_control_active(self, message: Bool) -> None:
        self._control_active = bool(message.data)
        if not self._control_active:
            self._active = False
        self._notify()

    def _on_pwm(self, _message: Float32MultiArray) -> None:
        self._last_pwm_monotonic = time.monotonic()
        self._notify()

    def _wait_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                result = predicate()
                if result:
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise DemoOperationError(f"timeout waiting for {description}")
                self._condition.wait(timeout=min(remaining, 0.2))

    def _call(
        self,
        key: str,
        request,
        *,
        require_success: bool = True,
        timeout: float | None = None,
    ):
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
            raise DemoOperationError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        call_timeout = self._service_call_timeout if timeout is None else timeout
        if not completed.wait(call_timeout):
            future.cancel()
            raise DemoOperationError(f"service timeout: {client.srv_name}")
        exception = future.exception()
        if exception is not None:
            raise DemoOperationError(f"{client.srv_name} failed: {exception}")
        response = future.result()
        if require_success and not bool(response.success):
            raise DemoOperationError(
                f"{client.srv_name} rejected: {response.message}"
            )
        return response

    def _current_initialized_status(self) -> LocalizationStatus | None:
        status = self._localization
        if status is None:
            return None
        valid = (
            status.state == LocalizationStatus.INITIALIZED
            and bool(status.output_valid)
            and int(status.epoch) > 0
            and bool(status.odometry_session_id.strip())
            and bool(status.alignment_id.strip())
        )
        return status if valid else None

    def _ensure_localized(self) -> LocalizationStatus:
        initialized = self._current_initialized_status()
        if initialized is None:
            deadline = time.monotonic() + self._localization_timeout
            self._publish_status(
                "LOCALIZING",
                "operator PREPARE confirms the camera is physically neutral",
            )
            self._call("confirm_neutral", Trigger.Request())

            def enough_samples():
                status = self._localization
                if status is None:
                    return None
                if status.state == LocalizationStatus.INVALID:
                    raise DemoOperationError(
                        f"localization invalid: {status.reason}"
                    )
                if (
                    status.state == LocalizationStatus.COLLECTING
                    and int(status.sample_count) >= self._minimum_samples
                ):
                    return status
                return None

            self._wait_until(
                enough_samples,
                max(0.1, deadline - time.monotonic()),
                f"{self._minimum_samples} stationary vision samples",
            )
            attempt = 0
            last_rejection = ""
            while self._current_initialized_status() is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise DemoOperationError(
                        "timeout waiting for a stable full-SE(3) pool "
                        f"alignment; last rejection: {last_rejection}"
                    )
                attempt += 1
                self._publish_status(
                    "INITIALIZING",
                    "approving full-SE(3) pool alignment with "
                    f"min_samples=0 (attempt {attempt})",
                )
                self.get_logger().info(
                    "stationary vision samples ready — calling "
                    "/brov/localization/initialize_pool with min_samples=0 "
                    f"(attempt {attempt})"
                )
                request = InitializePool.Request()
                request.min_samples = 0
                response = self._call(
                    "initialize",
                    request,
                    require_success=False,
                    timeout=min(self._service_call_timeout, remaining),
                )
                if response.success:
                    self.get_logger().info(
                        "full-SE(3) pool initialization accepted: "
                        f"{response.message}"
                    )
                    break
                last_rejection = response.message
                retryable = (
                    "residual gate left" in last_rejection
                    or "final residual gate left" in last_rejection
                    or "waiting for samples:" in last_rejection
                    or last_rejection
                    in {
                        "target marker is not currently visible",
                        "fresh local odometry is unavailable",
                        "vehicle is not currently stationary",
                    }
                )
                if not retryable:
                    raise DemoOperationError(
                        "/brov/localization/initialize_pool rejected: "
                        f"{last_rejection}"
                    )
                self.get_logger().warning(
                    "pool initialization not stable yet; retaining samples "
                    f"and retrying: {last_rejection}"
                )
                time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
            initialized = self._wait_until(
                self._current_initialized_status,
                max(0.1, deadline - time.monotonic()),
                "state=INITIALIZED(2), output_valid=true pool localization",
            )
        return initialized

    def _matching_odometry(
        self, status: LocalizationStatus
    ) -> AlignedOdometry | None:
        envelope = self._aligned_odometry
        if envelope is None:
            return None
        if (
            int(envelope.localization_epoch) != int(status.epoch)
            or envelope.odometry_session_id.strip()
            != status.odometry_session_id.strip()
            or envelope.alignment_id.strip() != status.alignment_id.strip()
        ):
            return None
        odometry = envelope.odometry
        if (
            odometry.header.frame_id.strip() != self._pool_frame
            or odometry.child_frame_id.strip() != self._base_frame
        ):
            return None
        return envelope

    def _case_a_path(self, envelope: AlignedOdometry) -> Path:
        position = envelope.odometry.pose.pose.position
        first, second = case_a_points(
            (position.x, position.y, position.z),
            self._safe_min,
            self._safe_max,
            self._segment_length,
            self._max_entry_distance,
        )
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._pool_frame
        for point in (first, second):
            stamped = PoseStamped()
            stamped.header = message.header
            stamped.pose.position.x = point[0]
            stamped.pose.position.y = point[1]
            stamped.pose.position.z = point[2]
            stamped.pose.orientation.w = 1.0
            message.poses.append(stamped)
        return message

    def _prepare_impl(self) -> str:
        if self._control_active or self._active:
            raise DemoOperationError("control is active; STOP before PREPARE")
        if self._demo_case != "a":
            raise DemoOperationError(
                "automatic orchestration currently supports Case A only; "
                "Case C remains on its explicit staged safety workflow"
            )
        status = self._ensure_localized()
        envelope = self._wait_until(
            lambda: self._matching_odometry(status),
            self._localization_timeout,
            "identity-matched pool odometry",
        )
        # A failed ARM/START can require a fresh control snapshot after the
        # vehicle moved beyond the base gate.  The mission manager intentionally
        # permits only one immutable commit per process, so reuse that committed
        # path and repeat only /brov/prepare_control.
        if self._active_path is not None:
            prepare = self._call("prepare", Trigger.Request())
            self._prepared = True
            return (
                f"{prepare.message}; reused committed pool path="
                f"{_path_points(self._active_path)}"
            )
        desired_path = None
        if self._auto_path:
            desired_path = self._case_a_path(envelope)
            if self._pub_draft.get_subscription_count() < 1:
                self._wait_until(
                    lambda: self._pub_draft.get_subscription_count() >= 1,
                    self._service_wait_timeout,
                    "mission draft subscriber",
                )
            self._pub_draft.publish(desired_path)
            # DDS does not impose cross-topic/service ordering. Republish until
            # validation observes this semantic draft or returns a real error.
            deadline = time.monotonic() + self._service_wait_timeout
            while True:
                response = self._call(
                    "validate", Trigger.Request(), require_success=False
                )
                if response.success:
                    break
                if "draft path has not been received" not in response.message:
                    raise DemoOperationError(
                        f"/brov/mission/validate rejected: {response.message}"
                    )
                if time.monotonic() >= deadline:
                    raise DemoOperationError(
                        "mission manager did not receive the generated draft"
                    )
                self._pub_draft.publish(desired_path)
                time.sleep(0.1)
        else:
            self._call("validate", Trigger.Request())

        self._call("commit", Trigger.Request())
        if desired_path is not None:
            expected = _path_points(desired_path)
            self._wait_until(
                lambda: (
                    self._active_path
                    if self._active_path is not None
                    and _path_points(self._active_path) == expected
                    else None
                ),
                self._service_wait_timeout,
                "committed Case-A pool path",
            )
        prepare = self._call("prepare", Trigger.Request())
        self._prepared = True
        points = (
            _path_points(desired_path)
            if desired_path is not None
            else _path_points(self._active_path)
        )
        return f"{prepare.message}; pool path={points}"

    def _cleanup_after_start_failure(self) -> list[str]:
        failures = []
        for key in (
            "stop",
            "controller_stop" if self._controller == "model" else None,
            "disarm",
        ):
            if key is None:
                continue
            try:
                response = self._call(
                    key, Trigger.Request(), require_success=False
                )
                if not response.success:
                    failures.append(f"{key}: {response.message}")
            except DemoOperationError as error:
                failures.append(str(error))
        self._prepared = False
        self._active = False
        return failures

    def _start_model_controller(self, deadline: float) -> None:
        last_message = ""
        while time.monotonic() < deadline:
            response = self._call(
                "controller_start",
                Trigger.Request(),
                require_success=False,
                timeout=min(2.0, max(0.1, deadline - time.monotonic())),
            )
            if response.success:
                return
            last_message = response.message
            if "fresh observation" not in last_message:
                break
            time.sleep(0.05)
        raise DemoOperationError(
            f"/brov/model_based/start rejected: {last_message}"
        )

    def _start_impl(self) -> str:
        if not self._prepared:
            raise DemoOperationError("demo is not prepared; call PREPARE first")
        if self._active or self._control_active:
            raise DemoOperationError("demo control is already active")
        self._last_pwm_monotonic = None
        self._call("arm", Trigger.Request())
        # The first-command deadline begins at base START, not before a
        # potentially multi-second hardware arming acknowledgement.
        start_mark = time.monotonic()
        self._call("start", Trigger.Request())
        self._wait_until(
            lambda: self._control_active,
            2.0,
            "base control_active=true",
        )
        deadline = start_mark + self._first_pwm_timeout
        if self._controller == "model":
            self._start_model_controller(deadline)
        self._wait_until(
            lambda: (
                self._last_pwm_monotonic
                if self._last_pwm_monotonic is not None
                and self._last_pwm_monotonic >= start_mark
                else None
            ),
            max(0.1, deadline - time.monotonic()),
            "first post-START controller PWM",
        )
        self._active = True
        return "control active and first post-START PWM observed"

    def _stop_impl(self) -> str:
        results = []
        failures = []
        sequence = ["stop"]
        if self._controller == "model":
            sequence.append("controller_stop")
        sequence.append("disarm")
        for key in sequence:
            try:
                response = self._call(
                    key, Trigger.Request(), require_success=False
                )
                results.append(f"{key}={response.message}")
                if not response.success:
                    failures.append(f"{key}: {response.message}")
            except DemoOperationError as error:
                failures.append(str(error))
        self._prepared = False
        self._active = False
        if failures:
            raise DemoOperationError(
                "STOP cleanup incomplete: " + "; ".join(failures)
            )
        return "; ".join(results)

    def _run(self, state: str, function, response):
        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another demo operation is already in progress"
            return response
        self._publish_status(state, f"{state.lower()} in progress")
        try:
            message = function()
        except (DemoOperationError, ValueError) as error:
            if state == "STARTING":
                cleanup = self._cleanup_after_start_failure()
                suffix = "" if not cleanup else "; cleanup: " + "; ".join(cleanup)
            else:
                suffix = ""
            response.success = False
            response.message = f"{error}{suffix}"
            self._publish_status("FAILED", response.message)
        except Exception as error:  # noqa: B902 - ROS boundary must fail closed
            if state == "STARTING":
                self._cleanup_after_start_failure()
            response.success = False
            response.message = f"unexpected orchestration error: {error}"
            self.get_logger().error(response.message)
            self._publish_status("FAILED", response.message)
        else:
            response.success = True
            response.message = message
            final_state = {
                "PREPARING": "PREPARED",
                "STARTING": "ACTIVE",
                "STOPPING": "IDLE",
            }[state]
            self._publish_status(final_state, message)
        finally:
            self._operation_lock.release()
        return response

    def _on_prepare(self, _request, response):
        return self._run("PREPARING", self._prepare_impl, response)

    def _on_start(self, _request, response):
        return self._run("STARTING", self._start_impl, response)

    def _on_stop(self, _request, response):
        return self._run("STOPPING", self._stop_impl, response)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = DemoOrchestratorNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
