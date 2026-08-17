"""ROS-independent one-shot pool-to-odom alignment primitives."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .math3d import (
    RobustTransformEstimate,
    invert_transform,
    robust_average_transforms,
    validate_transform,
)


@dataclass(frozen=True)
class TimedOdometry:
    stamp_s: float
    transform_odom_base: np.ndarray
    linear_speed_mps: float
    angular_speed_rad_s: float


@dataclass(frozen=True)
class TimedVisionPose:
    stamp_s: float
    transform_pool_base: np.ndarray


@dataclass(frozen=True)
class AlignmentSample:
    stamp_s: float
    collected_at_s: float
    timestamp_skew_s: float
    transform_pool_odom: np.ndarray


def make_alignment_sample(
    vision: TimedVisionPose,
    odometry: TimedOdometry,
    *,
    collected_at_s: float,
    max_timestamp_skew_s: float,
    max_linear_speed_mps: float,
    max_angular_speed_rad_s: float,
) -> AlignmentSample:
    if not all(
        math.isfinite(value)
        for value in (
            vision.stamp_s,
            odometry.stamp_s,
            collected_at_s,
            odometry.linear_speed_mps,
            odometry.angular_speed_rad_s,
        )
    ):
        raise ValueError("sample contains a non-finite scalar")
    if max_timestamp_skew_s < 0.0:
        raise ValueError("max_timestamp_skew_s must be non-negative")
    if max_linear_speed_mps < 0.0 or max_angular_speed_rad_s < 0.0:
        raise ValueError("stationary speed thresholds must be non-negative")
    skew = abs(vision.stamp_s - odometry.stamp_s)
    if skew > max_timestamp_skew_s:
        raise ValueError(
            f"timestamp skew {skew:.6f}s exceeds {max_timestamp_skew_s:.6f}s"
        )
    if odometry.linear_speed_mps > max_linear_speed_mps:
        raise ValueError(
            f"linear speed {odometry.linear_speed_mps:.6f}m/s is not stationary"
        )
    if odometry.angular_speed_rad_s > max_angular_speed_rad_s:
        raise ValueError(
            f"angular speed {odometry.angular_speed_rad_s:.6f}rad/s is not stationary"
        )

    transform_pool_base = validate_transform(vision.transform_pool_base)
    transform_odom_base = validate_transform(odometry.transform_odom_base)
    transform_pool_odom = transform_pool_base @ invert_transform(transform_odom_base)
    return AlignmentSample(
        stamp_s=vision.stamp_s,
        collected_at_s=collected_at_s,
        timestamp_skew_s=skew,
        transform_pool_odom=transform_pool_odom,
    )


class AlignmentSampleBuffer:
    """Bounded, time-limited collection of one-shot alignment candidates."""

    def __init__(self, *, max_samples: int, retention_s: float) -> None:
        if max_samples <= 0:
            raise ValueError("max_samples must be positive")
        if retention_s <= 0.0:
            raise ValueError("retention_s must be positive")
        self._samples: deque[AlignmentSample] = deque(maxlen=max_samples)
        self.retention_s = float(retention_s)

    def clear(self) -> None:
        self._samples.clear()

    def prune(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        while self._samples and now_s - self._samples[0].collected_at_s > self.retention_s:
            self._samples.popleft()

    def add(self, sample: AlignmentSample, *, now_s: float) -> bool:
        self.prune(now_s)
        # A repeated image timestamp must not count as a new independent sample.
        if any(existing.stamp_s == sample.stamp_s for existing in self._samples):
            return False
        self._samples.append(sample)
        return True

    def transforms(self, *, now_s: float) -> list[np.ndarray]:
        self.prune(now_s)
        return [sample.transform_pool_odom.copy() for sample in self._samples]

    def __len__(self) -> int:
        return len(self._samples)

    def estimate(
        self,
        *,
        now_s: float,
        min_samples: int,
        max_translation_residual_m: float,
        max_rotation_residual_rad: float,
    ) -> RobustTransformEstimate:
        return robust_average_transforms(
            self.transforms(now_s=now_s),
            max_translation_residual_m=max_translation_residual_m,
            max_rotation_residual_rad=max_rotation_residual_rad,
            min_inliers=min_samples,
        )
