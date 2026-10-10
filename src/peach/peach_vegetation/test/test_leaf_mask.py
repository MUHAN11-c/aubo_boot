"""Zero-ROS tests for Excess Green / HSV leaf masks and overlay paint."""

import numpy as np

from peach_vegetation.split import (
    BRANCH_BGR,
    excess_green,
    LEAF_BGR,
    leaf_mask_bgr,
    paint_overlay,
)


def test_excess_green_positive_on_pure_green():
    bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    bgr[:, :] = (0, 200, 0)
    assert np.all(excess_green(bgr) == 400)


def test_leaf_mask_catches_green_patch():
    bgr = np.zeros((32, 32, 3), dtype=np.uint8)
    bgr[8:24, 8:24] = (20, 180, 20)
    mask = leaf_mask_bgr(
        bgr, exg_min=20.0, h_min=35, h_max=90, s_min=30, v_min=30)
    assert mask[16, 16]
    assert not mask[0, 0]


def test_overlay_paints_leaf_then_branch():
    bgr = np.full((4, 4, 3), 128, dtype=np.uint8)
    leaf = np.zeros((4, 4), dtype=bool)
    branch = np.zeros((4, 4), dtype=bool)
    leaf[1, 1] = True
    branch[1, 1] = True
    branch[2, 2] = True
    overlay = paint_overlay(bgr, leaf, branch)
    assert tuple(overlay[1, 1]) == BRANCH_BGR
    assert tuple(overlay[2, 2]) == BRANCH_BGR
    assert tuple(overlay[0, 0]) == (128, 128, 128)
    leaf_only = paint_overlay(bgr, leaf, np.zeros_like(leaf))
    assert tuple(leaf_only[1, 1]) == LEAF_BGR
