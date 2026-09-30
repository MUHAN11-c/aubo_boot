import numpy as np
from peach2_core.landmarks2d import landmarks_from_mask
import pytest
from synthetic import bag_mask

DOWN = np.array([0.0, 1.0])


@pytest.mark.parametrize('axis_deg,gravity,tail', [
    (-90.0, (0.0, 1.0), 0.0),
    (-90.0, (0.0, 1.0), 10.0),
    (-70.0, (0.0, 1.0), 10.0),
    (-110.0, (0.2, 1.0), 0.0),
    (0.0, (-1.0, 0.05), 10.0),
])
def test_landmarks_on_synthetic_mask(axis_deg, gravity, tail):
    bottom = (160.0, 200.0) if axis_deg != 0.0 else (60.0, 120.0)
    mask, b, neck, tie = bag_mask(bottom_px=bottom, axis_deg=axis_deg, tail_len=tail)
    lm = landmarks_from_mask(mask, np.array(gravity))
    assert lm.ok, lm.flags
    assert np.linalg.norm(lm.bottom_px - b) < 3.0
    assert np.linalg.norm(lm.neck_px - neck) < 4.0
    assert np.linalg.norm(lm.tie_px - tie) < 4.0
    expected_axis = np.array([np.cos(np.radians(axis_deg)), np.sin(np.radians(axis_deg))])
    assert float(lm.axis_px @ expected_axis) > 0.99
    assert lm.widths_px[0] > lm.widths_px[-1]
    assert lm.confidence >= 0.6


def test_gravity_overrides_taper_and_flags_conflict():
    mask, b, neck, _ = bag_mask(axis_deg=-90.0)
    lm = landmarks_from_mask(mask, np.array([0.0, -1.0]))
    assert lm.ok
    assert 'taper_gravity_conflict' in lm.flags
    assert lm.confidence <= 0.3
    assert lm.bottom_px[1] < neck[1]


def test_taper_decides_when_gravity_is_uninformative():
    mask, b, neck, _ = bag_mask(axis_deg=-90.0)
    lm = landmarks_from_mask(mask, np.array([1.0, 0.0]))
    assert lm.ok
    assert np.linalg.norm(lm.bottom_px - b) < 3.0
    assert np.linalg.norm(lm.neck_px - neck) < 4.0


def test_small_mask_rejected():
    mask = np.zeros((50, 50), dtype=bool)
    mask[10:14, 10:14] = True
    lm = landmarks_from_mask(mask, DOWN)
    assert not lm.ok
    assert 'mask_too_small' in lm.flags


def test_round_mask_without_gravity_is_ambiguous():
    vv, uu = np.mgrid[0:100, 0:100]
    mask = (uu - 50) ** 2 + (vv - 50) ** 2 < 30 ** 2
    assert not landmarks_from_mask(mask, np.array([np.nan, np.nan])).ok
    lm = landmarks_from_mask(mask, DOWN)
    assert 'axis_from_gravity' in lm.flags
