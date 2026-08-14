"""In-process ROS contract test for the three-operation demo API."""

from __future__ import annotations

from copy import deepcopy
import threading

from brov_interfaces.msg import AlignedOdometry, LocalizationStatus
from brov_interfaces.srv import InitializePool
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

from brov_bringup.demo_orchestrator_node import DemoOrchestratorNode


class _FakeStack(Node):
    """Provide deterministic stand-ins for the authoritative stack nodes."""

    def __init__(self) -> None:
        super().__init__("test_demo_fake_stack")
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
        self.status_pub = self.create_publisher(
            LocalizationStatus, "/brov/localization/status", latched
        )
        self.odom_pub = self.create_publisher(
            AlignedOdometry,
            "/brov/localization/odometry_pool_with_alignment",
            qos_profile_sensor_data,
        )
        self.path_pub = self.create_publisher(
            Path, "/brov/mission/active_path_pool", latched
        )
        self.active_pub = self.create_publisher(
            Bool, "/brov/control_active", 1
        )
        self.pwm_pub = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 1
        )
        self.draft = None
        self.confirm_calls = 0
        self.initialize_calls = 0
        self.initialize_min_samples = None
        self.initialize_failures_remaining = 2
        self.validate_calls = 0
        self.commit_calls = 0
        self.prepare_control_calls = 0
        self.arm_calls = 0
        self.arm_failures_remaining = 1
        self.create_subscription(
            Path,
            "/brov/mission/draft_path",
            self._on_draft,
            draft_qos,
        )
        self.create_service(
            Trigger,
            "/brov/localization/confirm_camera_tilt_neutral",
            self._confirm,
        )
        self.create_service(
            InitializePool,
            "/brov/localization/initialize_pool",
            self._initialize,
        )
        self.create_service(
            Trigger, "/brov/mission/validate", self._validate
        )
        self.create_service(Trigger, "/brov/mission/commit", self._commit)
        self.create_service(
            Trigger, "/brov/prepare_control", self._prepare_control
        )
        self.create_service(Trigger, "/brov/arm_control", self._arm_control)
        self.create_service(
            Trigger, "/brov/disarm_control", self._success
        )
        self.create_service(
            Trigger, "/brov/start_control", self._start_control
        )
        self.create_service(
            Trigger, "/brov/stop_control", self._stop_control
        )

    def _on_draft(self, message: Path) -> None:
        self.draft = deepcopy(message)

    def _status(self, *, initialized: bool) -> LocalizationStatus:
        status = LocalizationStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "pool"
        if initialized:
            status.state = LocalizationStatus.INITIALIZED
            status.epoch = 1
            status.odometry_session_id = "runtime-session"
            status.alignment_id = "runtime-alignment"
            status.pool_to_odom.rotation.w = 1.0
            status.output_valid = True
            status.reason = "initialized by fake stack"
        else:
            status.state = LocalizationStatus.COLLECTING
            status.sample_count = 20
            status.reason = "20 stationary samples"
        return status

    def _confirm(self, _request, response):
        self.confirm_calls += 1
        self.status_pub.publish(self._status(initialized=False))
        response.success = True
        response.message = "neutral confirmed"
        return response

    def _initialize(self, request, response):
        self.initialize_calls += 1
        self.initialize_min_samples = int(request.min_samples)
        if self.initialize_failures_remaining > 0:
            self.initialize_failures_remaining -= 1
            self.status_pub.publish(self._status(initialized=False))
            response.success = False
            response.message = (
                "initialization rejected: residual gate left 14/20 "
                "required inliers"
            )
            return response
        status = self._status(initialized=True)
        self.status_pub.publish(status)
        envelope = AlignedOdometry()
        envelope.localization_epoch = status.epoch
        envelope.odometry_session_id = status.odometry_session_id
        envelope.alignment_id = status.alignment_id
        envelope.odometry.header.stamp = self.get_clock().now().to_msg()
        envelope.odometry.header.frame_id = "pool"
        envelope.odometry.child_frame_id = "base_link"
        envelope.odometry.pose.pose.position.x = 1.70
        envelope.odometry.pose.pose.position.y = 0.75
        envelope.odometry.pose.pose.position.z = 0.175618
        envelope.odometry.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(envelope)
        response.success = True
        response.message = "initialized"
        response.epoch = 1
        return response

    def _validate(self, _request, response):
        self.validate_calls += 1
        response.success = self.draft is not None
        response.message = (
            "valid" if response.success else "draft path has not been received"
        )
        return response

    def _commit(self, _request, response):
        self.commit_calls += 1
        response.success = self.draft is not None
        response.message = "committed" if response.success else "no draft"
        if self.draft is not None:
            self.path_pub.publish(self.draft)
        return response

    def _success(self, _request, response):
        response.success = True
        response.message = "ok"
        return response

    def _prepare_control(self, _request, response):
        self.prepare_control_calls += 1
        return self._success(_request, response)

    def _arm_control(self, _request, response):
        self.arm_calls += 1
        if self.arm_failures_remaining > 0:
            self.arm_failures_remaining -= 1
            response.success = False
            response.message = "vehicle moved 0.172m after prepare; prepare again"
            return response
        return self._success(_request, response)

    def _start_control(self, _request, response):
        self.active_pub.publish(Bool(data=True))
        self.pwm_pub.publish(Float32MultiArray(data=[0.0] * 8))
        response.success = True
        response.message = "started"
        return response

    def _stop_control(self, _request, response):
        self.active_pub.publish(Bool(data=False))
        response.success = True
        response.message = "stopped"
        return response


def _call(client, timeout: float = 5.0):
    assert client.wait_for_service(timeout_sec=timeout)
    future = client.call_async(Trigger.Request())
    event = threading.Event()
    future.add_done_callback(lambda _future: event.set())
    assert event.wait(timeout)
    assert future.exception() is None
    return future.result()


def test_prepare_start_stop_runtime_contract() -> None:
    rclpy.init()
    fake = _FakeStack()
    orchestrator = DemoOrchestratorNode(
        parameter_overrides=[
            Parameter("controller", value="rl"),
            Parameter("demo_case", value="a"),
            Parameter("service_wait_timeout_s", value=1.0),
            Parameter("service_call_timeout_s", value=2.0),
            Parameter("localization_timeout_s", value=3.0),
            Parameter("first_pwm_timeout_s", value=2.0),
        ]
    )
    caller = Node("test_demo_operator")
    prepare = caller.create_client(Trigger, "/brov/demo/prepare")
    start = caller.create_client(Trigger, "/brov/demo/start")
    stop = caller.create_client(Trigger, "/brov/demo/stop")
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (fake, orchestrator, caller):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        prepared = _call(prepare)
        assert prepared.success, prepared.message
        assert "pool path=" in prepared.message
        assert fake.confirm_calls == 1
        assert fake.initialize_calls == 3
        assert fake.initialize_min_samples == 0
        assert fake.draft is not None
        assert len(fake.draft.poses) == 3
        assert fake.draft.poses[0].pose.position.z == 0.20
        assert fake.draft.poses[1].pose.position.z == 0.70
        assert fake.draft.poses[2].pose.position.z == 0.70

        rejected_start = _call(start)
        assert not rejected_start.success
        assert "vehicle moved 0.172m after prepare" in rejected_start.message

        prepared_again = _call(prepare)
        assert prepared_again.success, prepared_again.message
        assert "reused committed pool path=" in prepared_again.message
        assert fake.confirm_calls == 1
        assert fake.initialize_calls == 3
        assert fake.validate_calls == 1
        assert fake.commit_calls == 1
        assert fake.prepare_control_calls == 2

        started = _call(start)
        assert started.success, started.message
        assert "first post-START PWM" in started.message

        stopped = _call(stop)
        assert stopped.success, stopped.message
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        for node in (caller, orchestrator, fake):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
