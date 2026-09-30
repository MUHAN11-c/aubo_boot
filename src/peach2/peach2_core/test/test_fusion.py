import numpy as np
from peach2_core.fusion import fuse_views, ViewSample
from peach2_core.types import axial_lateral_cov, invalid_landmark, Landmark, Landmarks3D
import pytest

AXIS = np.array([0.0, 0.0, 1.0])
BOTTOM = np.array([0.5, 0.1, 0.8])
NECK = BOTTOM + 0.13 * AXIS


def _view(bottom_err=np.zeros(3), neck_err=np.zeros(3), axis=AXIS, sig_lat=0.002,
          sig_ax=0.006, ok=True, tie=False, d95=0.08):
    cov_b = axial_lateral_cov(axis, sig_lat, 0.003)
    cov_n = axial_lateral_cov(axis, sig_lat, sig_ax)
    t = Landmark(True, NECK + 0.02 * AXIS, cov_n, 0.3) if tie else invalid_landmark()
    lm = Landmarks3D(ok=ok, bottom=Landmark(True, BOTTOM + bottom_err, cov_b, 0.8),
                     neck=Landmark(True, NECK + neck_err, cov_n, 0.7), tie=t,
                     axis=np.asarray(axis, dtype=float), d95_m=d95, length_m=0.13)
    return ViewSample(landmarks=lm, stamp_s=0.0, camera_distance_m=0.4)


def test_single_view_uses_its_covariance():
    m = fuse_views([_view()])
    assert m.ok and m.n_views == 1
    assert m.sigma_lateral95_m == pytest.approx(1.96 * 0.002, rel=1e-6)
    assert m.sigma_axial95_m == pytest.approx(1.96 * 0.006, rel=1e-6)
    expected_theta = np.degrees(1.96 * np.hypot(0.002, 0.002) / 0.13)
    assert m.theta95_deg == pytest.approx(expected_theta, rel=1e-6)
    assert np.allclose(m.bottom, BOTTOM) and np.allclose(m.neck, NECK)
    assert m.length_m == pytest.approx(0.13)
    assert m.tie is None


def test_mad_scatter_dominates_when_views_disagree():
    views = [_view(neck_err=np.array([0.0, 0.0, dz])) for dz in (-0.012, 0.0, 0.012)]
    m = fuse_views(views)
    # MAD of axial residuals {-12, 0, 12} mm = 12 mm -> sigma 17.8 mm -> 95%: 34.9 mm
    assert m.sigma_axial95_m == pytest.approx(1.96 * 1.4826 * 0.012, rel=1e-3)
    assert m.n_views == 3


def test_consistent_views_shrink_propagated_sigma():
    views = [_view() for _ in range(4)]
    m = fuse_views(views)
    assert m.sigma_axial95_m == pytest.approx(1.96 * 0.006 / 2.0, rel=1e-3)


def test_huber_rejects_outlier_view():
    views = [_view() for _ in range(4)] + [_view(bottom_err=np.array([0.05, 0.0, 0.0]))]
    m = fuse_views(views)
    assert np.linalg.norm(m.bottom - BOTTOM) < 0.004
    assert m.inlier_ratio == pytest.approx(0.8)


def test_min_views_and_invalid_views():
    assert not fuse_views([_view()], min_views=2).ok
    m = fuse_views([_view(ok=False), _view()])
    assert m.ok and m.n_views == 1
    assert not fuse_views([]).ok


def test_axis_sense_conflict_view_dropped_and_tie_fused():
    views = [_view(tie=True), _view(tie=True), _view(axis=-AXIS)]
    m = fuse_views(views)
    assert 'axis_sense_views_dropped' in m.flags
    assert m.n_views == 2
    assert m.tie is not None


def test_axis_line_inconsistency_flag():
    tilted = np.array([np.sin(np.radians(20.0)), 0.0, np.cos(np.radians(20.0))])
    m = fuse_views([_view(axis=tilted)])
    assert 'axis_line_inconsistent' in m.flags
