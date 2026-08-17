"""ACTION8 -> normalized command.

Adapted from ``cmg_RL_deploy/core/action_mapper.py``: that standalone
package converts ACTION8 directly to PWM microseconds because it owns its
own MAVLink connection. Here the destination is brov_base's
``/brov/thruster_pwm`` gateway, which already owns the PWM-microsecond
scale, the real-vehicle T2/T3/T8 reversal mask, arming, and the RC
override transport -- so this only clips to the actuator envelope and
does not rescale or reverse anything.
"""
import numpy as np

from .contract import ACTION_DIM


def clip_action(action):
    a = np.asarray(action, dtype=np.float32).reshape(-1)
    if a.shape != (ACTION_DIM,) or not np.isfinite(a).all():
        raise ValueError(f"finite ACTION{ACTION_DIM} required")
    return np.clip(a, -1.0, 1.0)
