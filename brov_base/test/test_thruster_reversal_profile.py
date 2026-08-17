"""Real/SITL actuator wiring profile regression tests."""

import pytest
import torch

from brov_base.mavlink_interface import thruster_reversal_sign_for_profile


def test_real_brov2_profile_preserves_t2_t3_t8_reversal():
    actual = thruster_reversal_sign_for_profile(
        "real_brov2", "udpout:192.168.2.2:14550"
    )
    expected = torch.tensor(
        [1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0]
    )
    assert torch.equal(actual, expected)


def test_edo_sitl_profile_is_identity_on_udpin():
    actual = thruster_reversal_sign_for_profile(
        "edo_sitl_identity", "udpin:0.0.0.0:14552"
    )
    assert torch.equal(actual, torch.ones(8))


def test_edo_sitl_profile_is_rejected_for_real_udpout():
    with pytest.raises(ValueError, match="requires a udpin"):
        thruster_reversal_sign_for_profile(
            "edo_sitl_identity", "udpout:192.168.2.2:14550"
        )


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="unknown thruster_reversal_profile"):
        thruster_reversal_sign_for_profile("typo", "udpin:0.0.0.0:14552")
