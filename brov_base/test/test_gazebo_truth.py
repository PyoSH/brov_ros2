"""Frame, timestamp, and fail-closed tests for Gazebo truth feedback."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
import torch

from brov_base.gazebo_truth import (
    GazeboTruthBuffer,
    body_angular_velocity_from_quaternions,
    gazebo_enu_flu_to_ned_frd,
)
from brov_base.math_utils import quat_from_euler_xyz


def _odom(
    *,
    stamp_s: float,
    position=(0.0, 0.0, 0.0),
    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    linear=(0.0, 0.0, 0.0),
    angular=(0.0, 0.0, 0.0),
    frame="odom",
    child="base_link",
):
    sec = math.floor(stamp_s)
    nanosec = round((stamp_s - sec) * 1.0e9)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        child_frame_id=child,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=position[0], y=position[1], z=position[2]
                ),
                orientation=SimpleNamespace(
                    x=quaternion_xyzw[0],
                    y=quaternion_xyzw[1],
                    z=quaternion_xyzw[2],
                    w=quaternion_xyzw[3],
                ),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(
                    x=linear[0], y=linear[1], z=linear[2]
                ),
                angular=SimpleNamespace(
                    x=angular[0], y=angular[1], z=angular[2]
                ),
            )
        ),
    )


def _xyzw(wxyz: torch.Tensor) -> tuple[float, float, float, float]:
    return tuple(float(value) for value in wxyz[[1, 2, 3, 0]])


def test_identity_gazebo_pose_maps_enu_flu_to_ned_frd() -> None:
    converted = gazebo_enu_flu_to_ned_frd(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (1.0, 2.0, 3.0),
    )

    assert converted.position_ned.tolist() == pytest.approx([2.0, 1.0, -3.0])
    # A Gazebo body whose nose points East has +90 deg NED yaw.
    assert converted.attitude_quat_ned_frd_wxyz.tolist() == pytest.approx(
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    )
    assert converted.linear_velocity_ned.tolist() == pytest.approx(
        [0.0, 1.0, 0.0]
    )
    assert converted.angular_rate_frd_proxy.tolist() == pytest.approx(
        [1.0, -2.0, -3.0]
    )


def test_north_facing_gazebo_pose_maps_to_zero_ned_yaw() -> None:
    q_enu_flu = quat_from_euler_xyz(
        torch.tensor(0.0), torch.tensor(0.0), torch.tensor(math.pi / 2.0)
    )
    converted = gazebo_enu_flu_to_ned_frd(
        (0.0, 0.0, 0.0), _xyzw(q_enu_flu), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    )

    assert converted.attitude_quat_ned_frd_wxyz.tolist() == pytest.approx(
        [1.0, 0.0, 0.0, 0.0], abs=1.0e-6
    )
    assert converted.linear_velocity_ned.tolist() == pytest.approx(
        [1.0, 0.0, 0.0], abs=1.0e-6
    )


def test_body_rate_is_derived_from_pose_quaternions() -> None:
    previous = torch.tensor([1.0, 0.0, 0.0, 0.0])
    current = quat_from_euler_xyz(
        torch.tensor(0.02), torch.tensor(0.0), torch.tensor(0.0)
    )

    rate = body_angular_velocity_from_quaternions(previous, current, 0.02)

    assert rate.tolist() == pytest.approx([1.0, 0.0, 0.0], abs=1.0e-5)

    with pytest.raises(ValueError, match="positive norm"):
        body_angular_velocity_from_quaternions(torch.zeros(4), current, 0.02)


def test_buffer_uses_source_time_for_rate_and_receipt_time_for_age() -> None:
    buffer = GazeboTruthBuffer()
    q0 = quat_from_euler_xyz(
        torch.tensor(0.0), torch.tensor(0.0), torch.tensor(math.pi / 2.0)
    )
    # Increasing Gazebo ENU yaw maps to decreasing NED/FRD yaw.
    q1 = quat_from_euler_xyz(
        torch.tensor(0.0),
        torch.tensor(0.0),
        torch.tensor(math.pi / 2.0 + 0.02),
    )

    assert buffer.update(_odom(stamp_s=10.0, quaternion_xyzw=_xyzw(q0)), receive_time=100.0)
    assert buffer.snapshot(now=100.0) is None
    assert buffer.update(
        _odom(
            stamp_s=10.02,
            quaternion_xyzw=_xyzw(q1),
            # Deliberately bogus RPY derivative: policy rate must ignore it.
            angular=(20.0, 30.0, 40.0),
        ),
        receive_time=100.2,
    )
    snapshot = buffer.snapshot(now=100.25)

    assert snapshot is not None
    assert snapshot["feedback_source_time_s"] == pytest.approx(10.02)
    assert snapshot["att_age_s"] == pytest.approx(0.05)
    assert snapshot["body_rates_ned"].tolist() == pytest.approx(
        [0.0, 0.0, -1.0], abs=2.0e-5
    )


def test_buffer_ignores_duplicate_stamp_and_keeps_quaternion_continuous() -> None:
    buffer = GazeboTruthBuffer()
    message0 = _odom(stamp_s=1.0)
    message1 = _odom(stamp_s=1.02, quaternion_xyzw=(0.0, 0.0, 0.0, -1.0))

    assert buffer.update(message0, receive_time=5.0)
    assert buffer.update(message1, receive_time=5.02)
    assert not buffer.update(message1, receive_time=5.04)
    snapshot = buffer.snapshot(now=5.04)

    assert snapshot is not None
    assert snapshot["att_seq"] == 2
    assert snapshot["body_rates_ned"].norm().item() == pytest.approx(0.0)


@pytest.mark.parametrize(
    "message,match",
    [
        (_odom(stamp_s=1.0, frame="map"), "truth frame"),
        (_odom(stamp_s=1.0, child="vehicle"), "child frame"),
        (_odom(stamp_s=1.0, position=(float("nan"), 0.0, 0.0)), "finite"),
        (_odom(stamp_s=1.0, quaternion_xyzw=(0.0, 0.0, 0.0, 0.0)), "norm"),
        (_odom(stamp_s=1.0, quaternion_xyzw=(0.0, 0.0, 0.0, 0.01)), "tolerance"),
    ],
)
def test_buffer_latches_malformed_truth(message, match: str) -> None:
    buffer = GazeboTruthBuffer()

    with pytest.raises(ValueError, match=match):
        buffer.update(message, receive_time=2.0)

    assert buffer.invalid_reason is not None
    with pytest.raises(ValueError):
        buffer.update(_odom(stamp_s=2.0), receive_time=3.0)


def test_buffer_latches_backward_simulation_clock() -> None:
    buffer = GazeboTruthBuffer()
    buffer.update(_odom(stamp_s=2.0), receive_time=10.0)

    with pytest.raises(ValueError, match="moved backward"):
        buffer.update(_odom(stamp_s=1.9), receive_time=10.1)

    assert "moved backward" in (buffer.invalid_reason or "")
