"""Frozen CMG hover policy contract.

Ported from ``cmg_RL_deploy/config/observation_contract.yaml`` and
``policy/POLICY_MANIFEST.json`` (Targeted DR1, policy SHA-256
``16f12c4a64f6876bb1be9a9cd52c604e0e777e25694c3089b5050909c9e84cef``).

Unlike the Sim2Swim MK2 contract, this policy outputs eight direct
per-thruster commands (no wrench allocation) and expects no PWM scaling
here -- ``/brov/thruster_pwm`` is normalized ``[-1, 1]``; the microsecond
conversion and the real-vehicle T2/T3/T8 reversal mask are applied once,
centrally, by brov_base's PWM gateway.
"""

OBS_DIM = 17
ACTION_DIM = 8
QUATERNION_ORDER = "WXYZ"
OBS_BLOCKS = (
    "target_quaternion_wxyz",
    "target_offset_body_xyz",
    "current_quaternion_wxyz",
    "body_linear_velocity_xyz",
    "body_angular_velocity_xyz",
)
POLICY_SHA256 = (
    "16f12c4a64f6876bb1be9a9cd52c604e0e777e25694c3089b5050909c9e84cef"
)
