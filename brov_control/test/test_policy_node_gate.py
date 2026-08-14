"""Motor-free tests for the RL preview/actual-output boundary."""

import inspect
from types import SimpleNamespace

import pytest
import torch
from std_msgs.msg import Bool, Float32MultiArray

from brov_control.policy_node import PolicyNode, _limit_pwm_step


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Policy:
    @staticmethod
    def act(_observation):
        return torch.tensor([0.2, -0.1, 0.0, 0.1, -0.2, 0.3])


class _Thruster:
    @staticmethod
    def inverse_thrust(_force):
        return torch.tensor(
            [[0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4]]
        )


def _owner():
    return SimpleNamespace(
        _control_active=False,
        _discard_next_active_observation=False,
        _obs_count=0,
        _vis_every_n=100,
        _action_abs_limit=torch.ones(6),
        _pwm_abs_limit=1.0,
        _pwm_max_delta=None,
        _last_sent_pwm=torch.zeros(8),
        policy=_Policy(),
        allocation_pinv=torch.zeros((8, 6)),
        thruster=_Thruster(),
        pub_action=_Publisher(),
        pub_preview=_Publisher(),
        pub_pwm=_Publisher(),
        get_logger=lambda: SimpleNamespace(
            warn=lambda _message: None,
            error=lambda _message: None,
            info=lambda _message: None,
        ),
    )


def _observation() -> Float32MultiArray:
    data = [0.0] * 16
    data[0] = 1.0
    return Float32MultiArray(data=data)


def test_policy_always_previews_but_never_outputs_before_base_start() -> None:
    owner = _owner()

    PolicyNode._on_observation(owner, _observation())

    assert len(owner.pub_action.messages) == 1
    assert len(owner.pub_preview.messages) == 1
    assert owner.pub_preview.messages[-1].data == pytest.approx(
        [0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4]
    )
    assert owner.pub_pwm.messages == []


def test_policy_outputs_only_after_current_control_active_signal() -> None:
    owner = _owner()

    PolicyNode._on_control_active(owner, Bool(data=True))
    PolicyNode._on_observation(owner, _observation())
    assert len(owner.pub_preview.messages) == 1
    assert owner.pub_pwm.messages == []

    PolicyNode._on_observation(owner, _observation())
    assert len(owner.pub_preview.messages) == 2
    assert len(owner.pub_pwm.messages) == 1

    PolicyNode._on_control_active(owner, Bool(data=False))
    PolicyNode._on_observation(owner, _observation())
    assert len(owner.pub_preview.messages) == 3
    assert len(owner.pub_pwm.messages) == 1


def test_constructor_wires_distinct_preview_and_active_topics() -> None:
    source = inspect.getsource(PolicyNode.__init__)

    assert '"/brov/policy/thruster_pwm_preview"' in source
    assert '"/brov/control_active"' in source
    assert "self._control_active = False" in source
    assert "self._discard_next_active_observation = False" in source
    assert "self._on_observation," in source
    assert "            1," in source


def test_pwm_envelope_limits_absolute_value_and_step() -> None:
    requested = torch.tensor([0.8, -0.8, 0.2])
    previous = torch.tensor([0.1, -0.1, 0.1])

    result = _limit_pwm_step(
        requested,
        previous,
        absolute_limit=0.5,
        max_delta=0.05,
    )

    assert torch.allclose(result, torch.tensor([0.15, -0.15, 0.15]))


def test_action_and_pwm_limits_apply_to_preview_and_live_output() -> None:
    owner = _owner()
    owner._action_abs_limit = torch.tensor([0.05] * 6)
    owner._pwm_abs_limit = 0.25

    PolicyNode._on_observation(owner, _observation())

    assert owner.pub_action.messages[-1].data == pytest.approx(
        [0.05, -0.05, 0.0, 0.05, -0.05, 0.05]
    )
    assert owner.pub_preview.messages[-1].data == pytest.approx(
        [0.1, -0.1, 0.2, -0.2, 0.25, -0.25, 0.25, -0.25]
    )


def test_slew_state_advances_only_when_live_pwm_is_published() -> None:
    owner = _owner()
    owner._pwm_max_delta = 0.05

    PolicyNode._on_observation(owner, _observation())
    assert torch.equal(owner._last_sent_pwm, torch.zeros(8))

    PolicyNode._on_control_active(owner, Bool(data=True))
    PolicyNode._on_observation(owner, _observation())
    assert torch.equal(owner._last_sent_pwm, torch.zeros(8))

    PolicyNode._on_observation(owner, _observation())
    assert owner.pub_pwm.messages[-1].data == pytest.approx(
        [0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05, -0.05]
    )
    assert torch.allclose(
        owner._last_sent_pwm,
        torch.tensor([0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05, -0.05]),
    )
