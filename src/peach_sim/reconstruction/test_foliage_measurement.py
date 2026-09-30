"""Check image-derived foliage direction and missing-data handling."""

import cv2
from measure_foliage import estimate_support
import numpy as np


def test_rotated_leaf_axis_follows_image():
    rgb = np.zeros((160, 160, 3), dtype=np.uint8)
    cv2.ellipse(rgb, (80, 80), (42, 10), 35, 0, 360, (30, 110, 45), -1)
    depth = np.full((160, 160), 500, dtype=np.uint16)
    support = estimate_support(rgb, depth)
    assert support
    best = max(support, key=lambda q: q['support_pixels'])
    expected = np.array([np.cos(np.deg2rad(35)), np.sin(np.deg2rad(35))])
    assert abs(np.dot(best['axis_image'], expected)) > .98
    assert .045 < best['length_m'] < .075
    assert best['width_m'] < best['length_m'] / 2
    assert support == estimate_support(rgb, depth)


def test_missing_depth_does_not_create_geometry():
    rgb = np.full((160, 160, 3), [30, 110, 45], dtype=np.uint8)
    assert not estimate_support(rgb, np.zeros((160, 160), dtype=np.uint16))


def test_yellow_watermark_does_not_create_foliage():
    rgb = np.full((160, 160, 3), [170, 180, 60], dtype=np.uint8)
    assert not estimate_support(rgb, np.full((160, 160), 500, dtype=np.uint16))
