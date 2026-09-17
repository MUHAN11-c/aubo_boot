"""Zero-ROS tests for the MAD outlier masks."""

from aubo_hand_eye_calibration.stats import mad_inliers_2d, mad_mask_1d
import numpy as np


def test_mad_mask_1d_keeps_small_samples():
    values = [1.0, 100.0]
    assert list(mad_mask_1d(values)) == [True, True]


def test_mad_mask_1d_rejects_outlier():
    values = [0.0, 0.1, -0.1, 0.05, 25.0]
    mask = mad_mask_1d(values)
    assert list(mask) == [True, True, True, True, False]


def test_mad_mask_1d_all_identical_stays_inlier():
    values = [0.3] * 6
    assert list(mad_mask_1d(values)) == [True] * 6


def test_mad_inliers_2d_keeps_small_samples():
    errors = np.array([[0.0, 0.0], [10.0, 10.0]])
    assert list(mad_inliers_2d(errors)) == [True, True]


def test_mad_inliers_2d_rejects_translation_outlier():
    errors = np.array([
        [0.000, 0.00],
        [0.001, 0.05],
        [0.002, 0.10],
        [0.001, 0.02],
        [0.090, 0.03],  # 平移离群
        [0.000, 0.07],
    ])
    mask = mad_inliers_2d(errors)
    assert not mask[4]
    assert mask.sum() == 5


def test_mad_inliers_2d_rejects_rotation_outlier():
    errors = np.array([
        [0.000, 0.10],
        [0.001, 0.12],
        [0.002, 0.08],
        [0.001, 0.11],
        [0.000, 9.00],  # 旋转离群
        [0.001, 0.09],
    ])
    mask = mad_inliers_2d(errors)
    assert not mask[4]
    assert mask.sum() == 5
