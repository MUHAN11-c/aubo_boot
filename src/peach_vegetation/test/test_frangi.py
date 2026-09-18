"""Zero-ROS tests for GPU/CPU Frangi ridges (torch optional)."""

import numpy as np
import pytest


def test_resolve_device_cpu():
    pytest.importorskip('torch')
    from peach_vegetation.frangi import resolve_device
    assert resolve_device('cpu') == 'cpu'


def test_frangi_dark_bar_is_high_on_line():
    pytest.importorskip('torch')
    from peach_vegetation.frangi import frangi_vesselness
    gray = np.full((64, 64), 200, dtype=np.uint8)
    gray[:, 30:34] = 15
    vessel = frangi_vesselness(gray, sigmas=(1, 3, 5), device='cpu')
    assert vessel.shape == gray.shape
    assert vessel[32, 31] > vessel[32, 8]


def test_threshold_keeps_dark_ridge():
    pytest.importorskip('torch')
    from peach_vegetation.frangi import frangi_vesselness, threshold_ridges
    gray = np.full((48, 48), 210, dtype=np.uint8)
    gray[20:28, 22:26] = 10
    vessel = frangi_vesselness(gray, sigmas=(1, 3, 5), device='cpu')
    mask = threshold_ridges(
        vessel, gray, percentile=80.0, dark_max=90.0, dilate_px=1)
    assert mask[24, 24]
    assert not mask[2, 2]


def test_splitter_leaf_and_branch_on_synthetic():
    pytest.importorskip('torch')
    from peach_vegetation.split import FrangiExgSplitter, SplitConfig
    bgr = np.full((64, 64, 3), 210, dtype=np.uint8)
    bgr[8:28, 8:40] = (30, 170, 25)
    bgr[:, 50:53] = (18, 18, 18)
    splitter = FrangiExgSplitter(SplitConfig(device='cpu'))
    result = splitter.split(bgr)
    assert result.leaf[16, 20]
    assert not result.leaf[0, 0]
    assert result.branch[:, 51].any()
    assert result.overlay.shape == bgr.shape
    assert result.infer_ms >= 0.0
    assert result.device == 'cpu'
