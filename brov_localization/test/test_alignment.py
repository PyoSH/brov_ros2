import math

import numpy as np
import pytest

from brov_localization.alignment import (
    AlignmentSampleBuffer,
    TimedOdometry,
    TimedVisionPose,
    make_alignment_sample,
)
from brov_localization.math3d import make_transform, rotation_rpy_rad


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


def _sample(
    *,
    vision_stamp: float = 10.01,
    odom_stamp: float = 10.00,
    linear_speed: float = 0.0,
    angular_speed: float = 0.0,
):
    transform_pool_odom = make_transform(
        [2.0, 0.7, 0.3], _quaternion_from_rpy(0.08, -0.06, 1.2)
    )
    transform_odom_base = make_transform(
        [0.4, -0.2, 0.1], _quaternion_from_rpy(-0.02, 0.03, -0.4)
    )
    transform_pool_base = transform_pool_odom @ transform_odom_base
    result = make_alignment_sample(
        TimedVisionPose(vision_stamp, transform_pool_base),
        TimedOdometry(
            odom_stamp,
            transform_odom_base,
            linear_speed,
            angular_speed,
        ),
        collected_at_s=10.02,
        max_timestamp_skew_s=0.05,
        max_linear_speed_mps=0.03,
        max_angular_speed_rad_s=0.05,
    )
    return result, transform_pool_odom


def test_pair_implements_pool_base_times_inverse_odom_base() -> None:
    sample, expected = _sample()
    assert sample.transform_pool_odom == pytest.approx(expected)
    assert sample.timestamp_skew_s == pytest.approx(0.01)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"vision_stamp": 10.2}, "timestamp skew"),
        ({"linear_speed": 0.031}, "linear speed"),
        ({"angular_speed": 0.051}, "angular speed"),
    ],
)
def test_pair_rejects_skew_or_motion(kwargs, match) -> None:
    with pytest.raises(ValueError, match=match):
        _sample(**kwargs)


def test_buffer_rejects_duplicate_image_stamp_and_prunes_old_samples() -> None:
    buffer = AlignmentSampleBuffer(max_samples=4, retention_s=1.0)
    sample, _ = _sample()
    assert buffer.add(sample, now_s=10.02)
    assert not buffer.add(sample, now_s=10.03)
    assert len(buffer) == 1
    buffer.prune(11.03)
    assert len(buffer) == 0


def test_buffer_estimate_preserves_full_se3_roll_and_pitch() -> None:
    buffer = AlignmentSampleBuffer(max_samples=10, retention_s=3.0)
    expected = None
    for index in range(5):
        sample, expected = _sample(
            vision_stamp=10.0 + index * 0.02,
            odom_stamp=10.0 + index * 0.02,
        )
        assert buffer.add(sample, now_s=10.1)
    estimate = buffer.estimate(
        now_s=10.2,
        min_samples=5,
        max_translation_residual_m=0.01,
        max_rotation_residual_rad=math.radians(1.0),
    )
    assert estimate.transform == pytest.approx(expected)
    roll, pitch, _ = rotation_rpy_rad(estimate.transform[:3, :3])
    assert roll == pytest.approx(0.08)
    assert pitch == pytest.approx(-0.06)


def test_pair_rejects_nonfinite_pose() -> None:
    sample, _ = _sample()
    invalid = sample.transform_pool_odom.copy()
    invalid[0, 3] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        make_alignment_sample(
            TimedVisionPose(10.0, invalid),
            TimedOdometry(10.0, np.eye(4), 0.0, 0.0),
            collected_at_s=10.0,
            max_timestamp_skew_s=0.1,
            max_linear_speed_mps=0.1,
            max_angular_speed_rad_s=0.1,
        )
