import numpy as np
from peach2_core.depth import backproject, depth_sigma_m, mask_points
import pytest

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])


def test_depth_sigma_quadratic():
    s1 = depth_sigma_m(np.array([0.5]), 600.0, 0.05, 0.25)
    s2 = depth_sigma_m(np.array([1.0]), 600.0, 0.05, 0.25)
    assert s1[0] == pytest.approx(0.25 * 0.25 / 30.0)
    assert s2[0] == pytest.approx(4.0 * s1[0])
    with pytest.raises(ValueError):
        depth_sigma_m(np.array([1.0]), 0.0, 0.05)


def test_backproject_centre_and_offset():
    depth = np.full((480, 640), 0.5)
    xyz, cov = backproject(320.0, 240.0, depth, K)
    assert np.allclose(xyz, [0.0, 0.0, 0.5])
    xyz, cov = backproject(380.0, 180.0, depth, K)
    assert np.allclose(xyz, [0.05, -0.05, 0.5])
    assert cov.shape == (3, 3)
    assert np.allclose(cov, cov.T)
    assert np.all(np.linalg.eigvalsh(cov) > 0.0)
    # depth floor dominates a perfectly flat window
    assert np.sqrt(cov[2, 2]) == pytest.approx(0.001)


def test_backproject_rejects_invalid_and_weights_confidence():
    depth = np.zeros((480, 640))
    assert backproject(100.0, 100.0, depth, K) == (None, None)
    assert backproject(-5.0, 100.0, np.ones((480, 640)), K) == (None, None)
    depth = np.full((480, 640), 0.6)
    depth[99:102, 99:102] = [[0.6, 0.9, 0.9], [0.6, 0.9, 0.9], [0.6, 0.6, 0.9]]
    conf = np.full((480, 640), 255, dtype=np.uint8)
    conf[99:102, 99:102] = [[255, 0, 0], [255, 0, 0], [255, 255, 0]]
    xyz, _ = backproject(100.0, 100.0, depth, K, win=1)
    assert xyz[2] == pytest.approx(0.9)
    xyz, _ = backproject(100.0, 100.0, depth, K, confidence=conf, win=1)
    assert xyz[2] == pytest.approx(0.6)


def test_mask_points_filters_depth_confidence_stride():
    depth = np.full((10, 10), 1.0)
    depth[0, 0] = 0.0
    mask = np.zeros((10, 10), dtype=bool)
    mask[0:4, 0:4] = True
    conf = np.ones((10, 10))
    conf[2, 2] = 0.1
    pts = mask_points(mask, depth, K, confidence=conf, min_conf=0.5)
    assert pts.shape == (14, 3)
    assert np.allclose(pts[:, 2], 1.0)
    pts2 = mask_points(mask, depth, K, stride=2)
    assert pts2.shape == (3, 3)
    with pytest.raises(ValueError):
        mask_points(np.zeros((5, 5), dtype=bool), depth, K)
