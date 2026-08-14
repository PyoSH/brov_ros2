"""Motor-free tests for model-controller activation freshness."""

from types import SimpleNamespace

from std_msgs.msg import Bool, Float32MultiArray

from brov_control.model_based_controller_node import ModelBasedControllerNode

from test_model_based_controller import _controller


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _owner():
    publishers = [_Publisher() for _ in range(7)]
    return SimpleNamespace(
        _enabled=False,
        _control_active=False,
        _discard_next_active_observation=False,
        _last_obs_time=None,
        _timeout=0.25,
        controller=_controller(),
        pub_wrench_zup=publishers[0],
        pub_wrench_sname=publishers[1],
        pub_action=publishers[2],
        pub_estimated_wrench=publishers[3],
        pub_force=publishers[4],
        pub_preview=publishers[5],
        pub_pwm=publishers[6],
        _other_pwm_publishers=lambda: [],
        _publish_enabled=lambda: None,
        _disable=lambda _reason: None,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )


def _observation() -> Float32MultiArray:
    values = [0.0] * 16
    values[0] = 1.0
    return Float32MultiArray(data=values)


def test_model_start_requires_sample_after_activation_barrier() -> None:
    owner = _owner()
    response = SimpleNamespace(success=None, message="")

    ModelBasedControllerNode._on_control_active(owner, Bool(data=True))
    ModelBasedControllerNode._on_observation(owner, _observation())
    assert owner._last_obs_time is None

    ModelBasedControllerNode._on_start(owner, None, response)
    assert response.success is False
    assert response.message == "fresh observation unavailable"

    ModelBasedControllerNode._on_observation(owner, _observation())
    assert owner._last_obs_time is not None

    ModelBasedControllerNode._on_start(owner, None, response)
    assert response.success is True
    assert owner._enabled is True


def test_model_activation_uses_depth_one_observation_queue() -> None:
    import inspect

    source = inspect.getsource(ModelBasedControllerNode.__init__)
    assert "self._discard_next_active_observation = False" in source
    assert "self._on_observation," in source
    assert "            1," in source
