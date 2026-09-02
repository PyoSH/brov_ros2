"""실험 A1 의 손잡이 — `wrench_gain` 이 정책 출력에 정확히 곱해지는가.

되먹임 루프가 떠는 조건은 "보정이 늦게 오고 **동시에** 세다" 다. 지연(실기 80 ms)
은 오늘 못 줄이지만 세기는 이 배율로 줄어든다. 배율이 정확히 걸리지 않으면
0.5 로 돌린 주행의 결론이 통째로 틀린다. 그래서 값을 고정한다.
"""
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
import torch

from brov_interfaces.msg import Observation

from brov_control.policy_wrench_node import PolicyWrenchNode

_ROOT = Path(__file__).resolve().parents[2]
_POLICY = (
    _ROOT / "artifacts" / "policies" / "sim2swim_fixplant_wa0017_mk2_s42_i299"
    / "policy_raw_flu_mk2.pt"
)
# MK2 계약 검증은 vehicle model 해시까지 대조한다 -- 실기 launch 가 넘기는 그 파일.
_VEHICLE = _ROOT / "brov_base" / "brov_base" / "vendor" / "brov2_heavy.yaml"


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _FixedPolicy:
    ACTION = torch.tensor([0.4, -0.2, 0.6, 0.1, -0.3, 0.5])

    def act(self, _obs):
        return self.ACTION.clone()


def _node(gain: float) -> PolicyWrenchNode:
    node = PolicyWrenchNode(parameter_overrides=[
        Parameter("policy_path", value=str(_POLICY)),
        Parameter("vehicle_model_path", value=str(_VEHICLE)),
        Parameter("wrench_gain", value=gain),
    ])
    node._policy = _FixedPolicy()
    return node


def _observation(node) -> Observation:
    msg = Observation()
    msg.contract = node._expected_obs_contract
    msg.valid = True
    msg.data = [0.0] * 16
    msg.seq = 1
    msg.integration_dt_s = 0.04
    return msg


def _published(node):
    out = []
    node._pub.publish = out.append
    node._on_obs(_observation(node))
    assert out, "명령이 나가지 않았다"
    m = out[0]
    return torch.tensor([m.force.x, m.force.y, m.force.z,
                         m.torque.x, m.torque.y, m.torque.z])


@pytest.mark.skipif(not _POLICY.exists(), reason="정책 번들이 없다")
def test_gain_scales_the_whole_wrench_and_only_that():
    full = _node(1.0)
    half = _node(0.5)
    w_full = _published(full)
    w_half = _published(half)
    expected_full = _FixedPolicy.ACTION * full._scale * full._to_sname
    assert torch.allclose(w_full, expected_full, atol=1e-5)
    assert torch.allclose(w_half, 0.5 * expected_full, atol=1e-5)
    full.destroy_node(); half.destroy_node()


@pytest.mark.skipif(not _POLICY.exists(), reason="정책 번들이 없다")
def test_gain_default_is_unity():
    node = PolicyWrenchNode(parameter_overrides=[
        Parameter("policy_path", value=str(_POLICY)),
        Parameter("vehicle_model_path", value=str(_VEHICLE))])
    assert node._gain == 1.0
    node.destroy_node()


@pytest.mark.skipif(not _POLICY.exists(), reason="정책 번들이 없다")
@pytest.mark.parametrize("bad", [0.0, -0.5, 1.5, float("nan")])
def test_gain_outside_zero_one_is_refused(bad):
    """증폭은 받지 않는다 -- 정책이 학습한 이득 위로 올릴 근거가 없다."""
    # 메시지까지 본다 -- 다른 ValueError(계약 검증 등)로 통과하는 빈 시험이 되면 안 된다.
    with pytest.raises(ValueError, match="wrench_gain"):
        _node(bad)
