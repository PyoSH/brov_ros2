import numpy as np
import pytest

from cmg_deploy.core.action_mapper import clip_action


def test_clips_to_unit_envelope_without_rescaling():
    result = clip_action([-2, -1, 0, 1, 2, 0, 0, 0])
    assert np.allclose(result, [-1, -1, 0, 1, 1, 0, 0, 0])


def test_rejects_wrong_length():
    with pytest.raises(ValueError):
        clip_action([0] * 6)


def test_rejects_non_finite():
    with pytest.raises(ValueError):
        clip_action([float("nan")] * 8)
