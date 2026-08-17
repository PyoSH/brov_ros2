import numpy as np


def normalize_quaternion_wxyz(q):
    q = np.asarray(q, dtype=np.float32).reshape(4)
    n = float(np.linalg.norm(q))
    if not np.isfinite(q).all() or n < 1e-8:
        raise ValueError("invalid WXYZ quaternion")
    return q / n


def quat_apply_inverse_wxyz(q, v):
    """Rotate world vector v into body coordinates using body->world WXYZ q."""
    q = normalize_quaternion_wxyz(q)
    v = np.asarray(v, dtype=np.float32).reshape(3)
    xyz = q[1:]
    t = 2 * np.cross(xyz, v)
    return v - q[0] * t + np.cross(xyz, t)
