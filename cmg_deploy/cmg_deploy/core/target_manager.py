import numpy as np

from .math_utils import normalize_quaternion_wxyz


class TargetManager:
    """Latches a hover setpoint from the first update after (re)construction.

    ``HOVER_ORIGIN`` freezes the first observed position/attitude as the
    target. ``RELATIVE_TARGET`` adds a fixed local Z-up offset to that same
    origin. Re-create the instance (do not just call ``update`` again) to
    re-latch a new origin -- the caller is expected to do this at the
    control-active edge, not merely once per process lifetime.
    """

    def __init__(self, mode="HOVER_ORIGIN", relative_xyz=(0, 0, 0), target_q_mode="INITIAL"):
        self.mode = mode
        self.relative = np.asarray(relative_xyz, dtype=np.float32)
        self.qmode = target_q_mode
        self.origin = None
        self.target_q = None

    def update(self, position_world, current_q_wxyz):
        p = np.asarray(position_world, dtype=np.float32).reshape(3)
        q = normalize_quaternion_wxyz(current_q_wxyz)
        if self.origin is None:
            self.origin = p.copy()
            self.target_q = q.copy() if self.qmode == "INITIAL" else np.array([1, 0, 0, 0], np.float32)
        target = self.origin if self.mode == "HOVER_ORIGIN" else self.origin + self.relative
        return target.copy(), self.target_q.copy()
