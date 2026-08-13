#!/usr/bin/env python3
"""ROS 2 wrapper for explicit waypoint-following model control.

The node always publishes preview diagnostics. Actual ``/brov/thruster_pwm``
publication begins only after base control is active and the operator calls
``/brov/model_based/start``. A stale observation, stopped base controller, or a
competing PWM publisher immediately disables output and publishes neutral.
"""

from __future__ import annotations

import time

import rclpy
import torch
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned

from .model_based_controller import ModelBasedController


def _array3(node: Node, name: str) -> list[float]:
    value = list(node.get_parameter(name).value)
    if len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    return [float(item) for item in value]


class ModelBasedControllerNode(Node):
    """Expose the model controller with explicit operator and watchdog gates."""

    def __init__(self):
        super().__init__("brov_model_based_controller")
        self.declare_parameter("vehicle_config", "")
        self.declare_parameter("linear_kp", [25.0, 25.0, 35.0])
        self.declare_parameter("linear_ki", [0.0, 0.0, 0.0])
        self.declare_parameter("attitude_kp", [3.0, 3.0, 3.0])
        self.declare_parameter("attitude_ki", [0.0, 0.0, 0.0])
        self.declare_parameter("angular_kd", [1.5, 1.5, 1.0])
        self.declare_parameter("force_limit", [15.0, 15.0, 20.0])
        self.declare_parameter("torque_limit", [3.0, 3.0, 3.0])
        self.declare_parameter("minimum_active_pwm", 0.10)
        self.declare_parameter("thruster_force_activation", 0.25)
        self.declare_parameter("observation_timeout_s", 0.25)

        vehicle_config = str(self.get_parameter("vehicle_config").value)
        params = (
            load_brov2_yaml(vehicle_config)
            if vehicle_config
            else load_brov2_yaml()
        )
        thruster_pos, thruster_dir = thruster_pos_dir_ned(params)
        self.controller = ModelBasedController(
            thruster_pos,
            thruster_dir,
            linear_kp=_array3(self, "linear_kp"),
            linear_ki=_array3(self, "linear_ki"),
            attitude_kp=_array3(self, "attitude_kp"),
            attitude_ki=_array3(self, "attitude_ki"),
            angular_kd=_array3(self, "angular_kd"),
            force_limit=_array3(self, "force_limit"),
            torque_limit=_array3(self, "torque_limit"),
            minimum_active_pwm=float(
                self.get_parameter("minimum_active_pwm").value
            ),
            thruster_force_activation=float(
                self.get_parameter("thruster_force_activation").value
            ),
        )
        self._timeout = float(
            self.get_parameter("observation_timeout_s").value
        )
        if self._timeout <= 0.0:
            raise ValueError("observation_timeout_s must be positive")

        self._enabled = False
        self._control_active = False
        self._last_obs_time: float | None = None

        self.pub_wrench_zup = self.create_publisher(
            Float32MultiArray, "/brov/model_based/wrench_zup", 10
        )
        self.pub_wrench_sname = self.create_publisher(
            Float32MultiArray, "/brov/model_based/wrench_sname", 10
        )
        self.pub_action = self.create_publisher(
            Float32MultiArray, "/brov/model_based/action", 10
        )
        self.pub_estimated_wrench = self.create_publisher(
            Float32MultiArray,
            "/brov/model_based/estimated_wrench_zup",
            10,
        )
        self.pub_force = self.create_publisher(
            Float32MultiArray, "/brov/model_based/thruster_force", 10
        )
        self.pub_preview = self.create_publisher(
            Float32MultiArray,
            "/brov/model_based/thruster_pwm_preview",
            10,
        )
        self.pub_enabled = self.create_publisher(
            Bool, "/brov/model_based/enabled", 10
        )
        self.pub_pwm = self.create_publisher(
            Float32MultiArray, "/brov/thruster_pwm", 10
        )

        self.sub_obs = self.create_subscription(
            Float32MultiArray,
            "/brov/observation",
            self._on_observation,
            10,
        )
        self.sub_active = self.create_subscription(
            Bool,
            "/brov/control_active",
            self._on_control_active,
            10,
        )
        self.srv_start = self.create_service(
            Trigger, "/brov/model_based/start", self._on_start
        )
        self.srv_stop = self.create_service(
            Trigger, "/brov/model_based/stop", self._on_stop
        )
        self.timer = self.create_timer(0.05, self._safety_tick)
        self.get_logger().info(
            "model controller ready in preview mode; call /brov/start_control "
            "and then /brov/model_based/start"
        )

    def _other_pwm_publishers(self) -> list[str]:
        publishers = []
        for info in self.get_publishers_info_by_topic("/brov/thruster_pwm"):
            own_publisher = (
                info.node_name == self.get_name()
                and info.node_namespace == self.get_namespace()
            )
            if not own_publisher:
                namespace = info.node_namespace.rstrip("/")
                publishers.append(f"{namespace}/{info.node_name}")
        return publishers

    def _publish_enabled(self) -> None:
        self.pub_enabled.publish(Bool(data=self._enabled))

    def _publish_neutral(self) -> None:
        self.pub_pwm.publish(Float32MultiArray(data=[0.0] * 8))

    def _disable(self, reason: str, *, warn: bool = True) -> None:
        was_enabled = self._enabled
        self._enabled = False
        if was_enabled:
            self._publish_neutral()
            message = f"MODEL CONTROL STOPPED - {reason}; neutral published"
            # Humble rejects changing severity at the same source location.
            if warn:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)
        self._publish_enabled()

    def _on_control_active(self, message: Bool) -> None:
        self._control_active = bool(message.data)
        if self._enabled and not self._control_active:
            self._disable("base control inactive")

    def _on_observation(self, message: Float32MultiArray) -> None:
        if len(message.data) != 16:
            if self._enabled:
                self._disable(
                    f"invalid observation dimension {len(message.data)}"
                )
            return
        try:
            output = self.controller.compute(
                torch.tensor(message.data, dtype=torch.float32)
            )
        except ValueError as error:
            if self._enabled:
                self._disable(str(error))
            return

        self._last_obs_time = time.monotonic()
        self.pub_wrench_zup.publish(
            Float32MultiArray(data=output.wrench_zup.tolist())
        )
        self.pub_wrench_sname.publish(
            Float32MultiArray(data=output.wrench_sname.tolist())
        )
        self.pub_action.publish(
            Float32MultiArray(data=output.normalized_action_zup.tolist())
        )
        self.pub_estimated_wrench.publish(
            Float32MultiArray(data=output.estimated_wrench_zup.tolist())
        )
        self.pub_force.publish(
            Float32MultiArray(data=output.thruster_force.tolist())
        )
        self.pub_preview.publish(Float32MultiArray(data=output.pwm.tolist()))
        if self._enabled:
            self.pub_pwm.publish(Float32MultiArray(data=output.pwm.tolist()))

    def _on_start(self, _request, response):
        if self._enabled:
            response.success = True
            response.message = "model control already active"
            return response
        if not self._control_active:
            response.success = False
            response.message = "base control is not active"
            return response
        if (
            self._last_obs_time is None
            or time.monotonic() - self._last_obs_time >= self._timeout
        ):
            response.success = False
            response.message = "fresh observation unavailable"
            return response
        others = self._other_pwm_publishers()
        if others:
            response.success = False
            response.message = (
                f"other PWM publisher exists: {', '.join(others)}"
            )
            return response

        self._enabled = True
        self._publish_enabled()
        self.get_logger().info(
            "MODEL CONTROL ACTIVE - explicit PI/PD wrench to PWM"
        )
        response.success = True
        response.message = "model control active"
        return response

    def _on_stop(self, _request, response):
        self._disable("stop service", warn=False)
        response.success = True
        response.message = "model control stopped; neutral published"
        return response

    def _safety_tick(self) -> None:
        self._publish_enabled()
        if not self._enabled:
            return
        if (
            self._last_obs_time is None
            or time.monotonic() - self._last_obs_time >= self._timeout
        ):
            self._disable("observation watchdog timeout")
            return
        others = self._other_pwm_publishers()
        if others:
            self._disable(
                f"competing PWM publisher: {', '.join(others)}"
            )

    def shutdown(self) -> None:
        """Publish neutral before the ROS context and publisher are destroyed."""
        was_enabled = self._enabled
        self._disable("node shutdown", warn=False)
        if was_enabled:
            for _ in range(2):
                time.sleep(0.05)
                self._publish_neutral()


def main(args=None) -> None:
    # Use KeyboardInterrupt so neutral can be sent before rclpy shuts down.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = ModelBasedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
