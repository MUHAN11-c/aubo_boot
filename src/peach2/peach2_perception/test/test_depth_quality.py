import numpy as np
from peach2_perception.depth_quality import (confident_depth, depth_edges, depth_to_metres,
                                             mask_depth_coverage, normalise_confidence,
                                             surrogate_confidence, valid_depth_mask)
import pytest


def test_uint16_conversion_zero_and_saturation_invalid():
    raw = np.array([[0, 4000, 65535]], dtype=np.uint16)
    d = depth_to_metres(raw, 0.00025)
    assert d.dtype == np.float32
    assert d[0, 0] == 0.0
    assert d[0, 1] == pytest.approx(1.0)
    assert d[0, 2] == 0.0


def test_percipio_unit():
    d = depth_to_metres(np.array([[600]], dtype=np.uint16), 0.001)
    assert d[0, 0] == pytest.approx(0.6)


def test_float_depth_is_metres_and_sanitised():
    d = depth_to_metres(np.array([[0.7, np.nan, -1.0, np.inf]], dtype=np.float32), 0.001)
    assert d.tolist()[0] == [pytest.approx(0.7), 0.0, 0.0, 0.0]


def test_rejects_bad_input():
    with pytest.raises(ValueError):
        depth_to_metres(np.zeros((2, 2), dtype=np.int32), 0.001)
    with pytest.raises(ValueError):
        depth_to_metres(np.zeros((2, 2, 3), dtype=np.uint16), 0.001)
    with pytest.raises(ValueError):
        depth_to_metres(np.zeros((2, 2), dtype=np.uint16), 0.0)


def test_valid_range():
    d = np.array([[0.0, 0.2, 0.5, 2.0]], dtype=np.float32)
    assert valid_depth_mask(d, 0.3, 1.5).tolist() == [[False, False, True, False]]


def test_normalise_confidence():
    assert normalise_confidence(np.array([[0, 255]], dtype=np.uint8)).tolist() == [[0.0, 1.0]]
    c = normalise_confidence(np.array([[np.nan, 2.0, 0.5]], dtype=np.float32))
    assert c.tolist() == [[0.0, 1.0, 0.5]]


def test_surrogate_flat_plane_confident_step_edge_not():
    d = np.full((20, 20), 0.6, dtype=np.float32)
    d[:, 10:] = 1.2
    valid = d > 0
    c = surrogate_confidence(d, valid, 0.01, 0.04)
    assert c[10, 3] == pytest.approx(1.0)
    assert c[10, 9] == 0.0 and c[10, 10] == 0.0
    assert c[10, 15] == pytest.approx(1.0)


def test_surrogate_isolated_speckle_zero():
    d = np.zeros((9, 9), dtype=np.float32)
    d[4, 4] = 0.6
    c = surrogate_confidence(d, d > 0, 0.01, 0.04)
    assert c[4, 4] == 0.0


def test_surrogate_linear_ramp_between_thresholds():
    d = np.full((5, 5), 1.0, dtype=np.float32)
    d[2, 3] = 1.025  # 2.5 % jump -> halfway between 1 % and 4 %
    c = surrogate_confidence(d, d > 0, 0.01, 0.04)
    assert c[2, 2] == pytest.approx(0.5, abs=0.02)


def test_depth_edges_and_confident_depth():
    d = np.full((6, 6), 0.5, dtype=np.float32)
    d[:, 3:] = 0.6
    e = depth_edges(d, d > 0, 0.03, 0.01)
    assert e[:, 2].all() and e[:, 3].all() and not e[:, 0].any()
    conf = np.ones_like(d)
    conf[0, 0] = 0.2
    dc = confident_depth(d, d > 0, conf, 0.5)
    assert dc[0, 0] == 0.0 and dc[1, 1] == pytest.approx(0.5)


def test_mask_coverage():
    mask = np.zeros((4, 4), bool)
    mask[:2, :2] = True
    ok = np.zeros((4, 4), bool)
    ok[0, :] = True
    assert mask_depth_coverage(mask, ok) == pytest.approx(0.5)
    assert mask_depth_coverage(np.zeros((4, 4), bool), ok) == 0.0
