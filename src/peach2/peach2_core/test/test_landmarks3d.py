import numpy as np
from peach2_core.landmarks3d import landmarks_from_points
import pytest
from synthetic import bag_cloud

GRAVITY = np.array([0.0, 0.0, -1.0])


def _axial(err, axis):
    return float(err @ axis)


def _lateral(err, axis):
    return float(np.linalg.norm(err - (err @ axis) * axis))


@pytest.mark.parametrize('seed', range(12))
def test_single_view_frustum_with_noise_and_flying_points(seed):
    rng = np.random.default_rng(seed)
    camera = (0.0, 0.0, 0.85) if seed % 2 == 0 else (0.9, 0.6, 1.0)
    bag = bag_cloud(rng, tilt_deg=5.0 + 2.0 * seed, camera=camera, flying_ratio=0.03)
    lm = landmarks_from_points(bag.points, GRAVITY)
    assert lm.ok, lm.flags
    eb = lm.bottom.position - bag.bottom
    en = lm.neck.position - bag.neck
    # M2 exit gate: bottom 3D <= 8 mm; neck axial (fused) <= 10 mm; single view must beat both.
    assert np.linalg.norm(eb) < 0.006
    assert abs(_axial(en, bag.axis)) < 0.006
    assert _lateral(en, bag.axis) < 0.004
    assert np.degrees(np.arccos(np.clip(lm.axis @ bag.axis, -1.0, 1.0))) < 2.0
    assert lm.d95_m == pytest.approx(bag.d_max, abs=0.006)
    assert lm.length_m == pytest.approx(0.13, abs=0.008)
    # axial neck sigma is at least half a bin (profile resolution)
    span = 0.16
    sig_ax_neck = float(np.sqrt(lm.axis @ lm.neck.cov @ lm.axis))
    assert sig_ax_neck >= 0.5 * span / 12 * 0.9
    # the reported 1-sigma covers the true error within ~3 sigma
    assert abs(_axial(en, bag.axis)) < 3.0 * sig_ax_neck
    assert lm.neck.confidence > 0.5
    assert lm.tie.valid
    assert float(lm.axis @ -GRAVITY) > 0.0


def test_axis_is_clamped_to_gravity_cone():
    rng = np.random.default_rng(3)
    bag = bag_cloud(rng, tilt_deg=60.0, flying_ratio=0.0)
    lm = landmarks_from_points(bag.points, GRAVITY)
    assert 'axis_gravity_clamped' in lm.flags
    assert np.degrees(np.arccos(lm.axis @ -GRAVITY)) == pytest.approx(45.0, abs=0.5)


def test_upside_down_bag_keeps_gravity_polarity():
    rng = np.random.default_rng(4)
    bag = bag_cloud(rng, tilt_deg=180.0, flying_ratio=0.0)
    lm = landmarks_from_points(bag.points, GRAVITY)
    assert lm.ok
    assert 'taper_inverted_gravity_kept' in lm.flags
    assert float(lm.axis @ -GRAVITY) > 0.0


def test_cylinder_without_taper_flags_neck_fallback():
    rng = np.random.default_rng(5)
    bag = bag_cloud(rng, r_bottom=0.03, r_top=0.03, r_neck=0.03, flying_ratio=0.0)
    lm = landmarks_from_points(bag.points, GRAVITY)
    assert lm.ok
    assert 'neck_at_top_no_taper' in lm.flags
    assert not lm.tie.valid
    assert lm.neck.confidence < 0.5


def test_degenerate_inputs():
    assert not landmarks_from_points(np.zeros((10, 3)), GRAVITY).ok
    rng = np.random.default_rng(6)
    bag = bag_cloud(rng)
    assert 'gravity_invalid' in landmarks_from_points(bag.points, np.zeros(3)).flags
    blob = rng.normal(0.0, 0.005, (500, 3))
    assert not landmarks_from_points(blob, GRAVITY).ok


def test_explicit_point_sigma_is_used():
    rng = np.random.default_rng(7)
    bag = bag_cloud(rng, flying_ratio=0.0)
    a = landmarks_from_points(bag.points, GRAVITY, sigma_point_m=np.full(len(bag.points), 0.01))
    b = landmarks_from_points(bag.points, GRAVITY)
    sa = float(np.sqrt(a.axis @ a.bottom.cov @ a.axis))
    sb = float(np.sqrt(b.axis @ b.bottom.cov @ b.axis))
    assert sa > sb
