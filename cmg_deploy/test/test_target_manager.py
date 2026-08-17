import numpy as np

from cmg_deploy.core.target_manager import TargetManager


def test_hover_origin_latches_first_pose():
    manager = TargetManager(mode="HOVER_ORIGIN")
    target_pos, target_q = manager.update([1, 2, 3], [1, 0, 0, 0])
    assert np.allclose(target_pos, [1, 2, 3])
    assert np.allclose(target_q, [1, 0, 0, 0])
    # Later motion must not move an already-latched hover target.
    target_pos_2, _ = manager.update([9, 9, 9], [1, 0, 0, 0])
    assert np.allclose(target_pos_2, [1, 2, 3])


def test_relative_target_adds_offset_to_latched_origin():
    manager = TargetManager(mode="RELATIVE_TARGET", relative_xyz=(0, 0, 0.5))
    target_pos, _ = manager.update([1, 2, 3], [1, 0, 0, 0])
    assert np.allclose(target_pos, [1, 2, 3.5])


def test_target_quaternion_mode_level_ignores_current_attitude():
    manager = TargetManager(target_q_mode="LEVEL")
    _, target_q = manager.update([0, 0, 0], [0.7, 0.7, 0, 0])
    assert np.allclose(target_q, [1, 0, 0, 0])
