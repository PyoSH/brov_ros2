import numpy as np

from cmg_deploy.core.math_utils import normalize_quaternion_wxyz, quat_apply_inverse_wxyz


def test_identity_rotation_is_a_noop():
    assert np.allclose(quat_apply_inverse_wxyz([1, 0, 0, 0], [1, 2, 3]), [1, 2, 3])


def test_normalize_rejects_zero_quaternion():
    import pytest

    with pytest.raises(ValueError):
        normalize_quaternion_wxyz([0, 0, 0, 0])


def test_normalize_scales_to_unit_norm():
    result = normalize_quaternion_wxyz([2, 0, 0, 0])
    assert np.allclose(result, [1, 0, 0, 0])
