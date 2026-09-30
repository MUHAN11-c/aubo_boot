import dataclasses
import math

import numpy as np
from peach2_core.types import axial_lateral_cov
from peach2_target_model.decision import Remeasure
from peach2_target_model.store import (aggregate_view, build_frame, fuse_job, FusionOutput,
                                       ModelStore, rotation_angle_deg, swing_from_views)
import pytest
from tm_fixtures import AXIS, BOTTOM, fused, NECK, params

COV = axial_lateral_cov(AXIS, 0.0015, 0.002).reshape(9)
NO_TIE = (False, (0.0, 0.0, 0.0), tuple(np.zeros(9)), 0.0)
CAM_POS = (0.2, 0.1, 0.9)
CAM_Q = (0.0, 0.0, 0.0, 1.0)
CAM = (CAM_POS, CAM_Q)
GOOD = (True, BOTTOM, COV, 0.8)
NECK_LM = (True, NECK, COV, 0.7)


def _quat_z(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _frame(t, dist=0.5, bottom_err=(0.0, 0.0, 0.0), neck_err=(0.0, 0.0, 0.0), axis=AXIS,
           swing=0.0, cov=COV, cam=CAM, swing_known=True):
    f = build_frame(t, (True, BOTTOM + bottom_err, cov, 0.8), (True, NECK + neck_err, cov, 0.7),
                    NO_TIE, axis, 0.05, dist, cam, swing_known, swing)
    assert not isinstance(f, str), f
    return f


def _build(neck=NECK_LM, bottom=GOOD, axis=AXIS, d95=0.05, cam=CAM, known=True, swing=0.0):
    return build_frame(0.0, bottom, neck, NO_TIE, axis, d95, 0.5, cam, known, swing)


def _fuse(store, p):
    return store.commit([fuse_job(job, p) for job in store.take_jobs()])


def test_build_frame_rejections_and_axis_fixups():
    assert _build(neck=(False, NECK, COV, 0.0)) == 'landmark_invalid'
    assert _build(bottom=(True, BOTTOM, tuple(np.zeros(9)), 0.8)) == 'bad_covariance'
    assert _build(neck=(True, (np.nan, 0.0, 0.0), COV, 0.7)) == 'non_finite'
    assert _build(neck=(True, BOTTOM, COV, 0.7), axis=(0.0, 0.0, 0.0)) == 'axis_invalid'
    assert _build(d95=np.nan) == 'diameter_invalid'
    derived = _build(axis=(0.0, 0.0, 0.0))
    assert np.allclose(derived.landmarks.axis, AXIS)
    flipped = _build(axis=-AXIS)
    assert np.allclose(flipped.landmarks.axis, AXIS)
    assert flipped.landmarks.length_m == pytest.approx(0.13)


def test_build_frame_camera_pose_and_swing_known():
    assert _build(cam=(CAM_POS, (0.0, 0.0, 0.0, 0.0))) == 'camera_pose_invalid'
    assert _build(cam=((np.nan, 0.0, 0.0), CAM_Q)) == 'camera_pose_invalid'
    f = _build(cam=(CAM_POS, (0.0, 0.0, 0.0, 1.02)))
    assert np.allclose(f.camera_quat_xyzw, [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(f.camera_position, CAM_POS)
    assert f.swing_known and f.swing_amplitude_m == 0.0
    unknown = _build(known=False, swing=0.0)
    assert not unknown.swing_known and math.isnan(unknown.swing_amplitude_m)
    assert not _build(known=True, swing=np.nan).swing_known
    assert not _build(known=True, swing=-0.001).swing_known


def test_rotation_angle():
    assert rotation_angle_deg(np.array(CAM_Q), np.array(_quat_z(30.0))) == pytest.approx(30.0)
    neg = -np.array(_quat_z(30.0))
    assert rotation_angle_deg(np.array(_quat_z(30.0)), neg) == pytest.approx(0.0, abs=1e-6)


def test_view_segmentation_by_camera_pose():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    for i in range(5):
        store.add_frame('t', _frame(0.1 * i))
    shifted = (np.array(CAM_POS) + [p.view_translation_change_m + 0.01, 0.0, 0.0], CAM_Q)
    store.add_frame('t', _frame(0.5, cam=shifted))
    store.add_frame('t', _frame(0.6, cam=(shifted[0], _quat_z(p.view_rotation_change_deg + 1))))
    store.add_frame('t', _frame(0.7, cam=(shifted[0], _quat_z(p.view_rotation_change_deg + 1))))
    store.add_frame('t', _frame(0.7 + p.view_gap_s + 0.01,
                                cam=(shifted[0], _quat_z(p.view_rotation_change_deg + 1))))
    jobs = store.take_jobs()
    assert [len(v) for v in jobs[0].views] == [5, 1, 2, 1]
    assert store.take_jobs() == []


def test_small_camera_motion_stays_in_view_regardless_of_distance():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    small = (np.array(CAM_POS) + [0.5 * p.view_translation_change_m, 0.0, 0.0],
             _quat_z(0.5 * p.view_rotation_change_deg))
    store.add_frame('t', _frame(0.0, dist=0.5))
    store.add_frame('t', _frame(0.1, dist=0.7, cam=small))
    assert [len(v) for v in store.take_jobs()[0].views] == [2]


def test_view_and_frame_caps():
    p = dataclasses.replace(params(), max_views_per_target=3, max_frames_per_view=4)
    store = ModelStore(p)
    store.set_epoch(1)
    for i in range(10):
        store.add_frame('t', _frame(0.05 * i))
    for k in range(4):
        store.add_frame('t', _frame(10.0 + 2.0 * k))
    views = store.take_jobs()[0].views
    assert len(views) == 3 and views[0][0].stamp_s == pytest.approx(12.0)
    store2 = ModelStore(p)
    store2.set_epoch(1)
    for i in range(10):
        store2.add_frame('t', _frame(0.05 * i))
    frames = store2.take_jobs()[0].views[0]
    assert len(frames) == 4 and frames[0].stamp_s == pytest.approx(0.30)


def test_aggregate_view_is_robust_and_keeps_single_frame_cov():
    frames = [_frame(0.1 * i, neck_err=(0.0, 0.0, 0.001 * (i % 2))) for i in range(6)]
    frames.append(_frame(0.7, neck_err=(0.0, 0.0, 0.08)))
    v = aggregate_view(frames)
    assert v.landmarks.ok
    assert abs(v.landmarks.neck.position[2] - NECK[2]) <= 0.001 + 1e-9
    assert np.allclose(v.landmarks.bottom.cov, COV.reshape(3, 3))
    assert not v.landmarks.tie.valid
    assert v.stamp_s == pytest.approx(0.7)


def test_fusion_revisions_and_epochs():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    rng = np.random.default_rng(0)
    for k in range(3):
        for i in range(5):
            err = rng.normal(0.0, 0.0005, 3)
            store.add_frame('t', _frame(2.0 * k + 0.1 * i, bottom_err=err, neck_err=err))
    assert _fuse(store, p) == ['t']
    rec = store.record('t')
    assert rec.revision == 1 and rec.fused.n_views == 3 and rec.converged
    assert rec.last_obs_s == pytest.approx(4.4)

    store.add_frame('t', _frame(4.5, bottom_err=np.zeros(3), neck_err=np.zeros(3)))
    assert _fuse(store, p) == []
    assert store.record('t').revision == 1
    assert store.record('t').last_obs_s == pytest.approx(4.5)

    store.add_frame('t', _frame(8.0, neck_err=(0.0, 0.0, 0.004)))
    assert _fuse(store, p) == ['t']
    assert store.record('t').revision == 2 and store.record('t').fused.n_views == 4

    assert store.set_epoch(2)
    assert store.records() == [] and store.take_jobs() == []
    store.add_frame('t', _frame(20.0))
    _fuse(store, p)
    assert store.record('t').revision == 3


def test_stale_epoch_output_ignored():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    store.add_frame('t', _frame(0.0))
    out = fuse_job(store.take_jobs()[0], p)
    store.set_epoch(2)
    store.add_frame('t', _frame(1.0))
    assert store.commit([out]) == []
    assert store.commit([FusionOutput('ghost', 2, fused(), 0.0, 1.0, [])]) == []


def test_remeasure_state_and_frames_since():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    for i in range(4):
        store.add_frame('t', _frame(float(i)))
    _fuse(store, p)
    assert [f.stamp_s for f in store.frames_since('t', 2.0)] == [2.0, 3.0]
    assert store.frames_since('nobody', 0.0) == []
    store.set_remeasure('t', Remeasure.PASSED)
    assert store.record('t').remeasure is Remeasure.PASSED
    store.set_remeasure('t', Remeasure.MISMATCH)
    store.set_remeasure('t', Remeasure.PASSED)
    assert store.record('t').remeasure is Remeasure.MISMATCH
    store.set_epoch(2)
    store.add_frame('t', _frame(10.0))
    _fuse(store, p)
    assert store.record('t').remeasure is Remeasure.NONE


def test_unconverged_single_noisy_view():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    wide = axial_lateral_cov(AXIS, 0.006, 0.008).reshape(9)
    store.add_frame('t', _frame(0.0, cov=wide))
    _fuse(store, p)
    rec = store.record('t')
    assert not rec.converged
    assert set(rec.unconverged) >= {'sigma_lateral', 'sigma_axial', 'theta'}


def test_swing_estimate_and_reported():
    t = np.arange(0.0, 4.0, 0.1)
    frames = [_frame(float(ti), bottom_err=(0.01 * np.sin(2 * np.pi * ti / 1.6), 0.0, 0.0))
              for ti in t]
    assert swing_from_views([frames], 5.0) == pytest.approx(0.01, abs=1e-3)
    static = [_frame(0.1 * i, swing=0.004) for i in range(12)]
    assert swing_from_views([static], 5.0) == pytest.approx(0.004, abs=1e-6)
    assert swing_from_views([[_frame(0.0, swing=0.002)]], 5.0) == 0.002
    assert math.isnan(swing_from_views([], 5.0))


def test_swing_ignores_unknown_frames():
    t = np.arange(0.0, 4.0, 0.1)
    moving = [_frame(float(ti), bottom_err=(0.01 * np.sin(2 * np.pi * ti / 1.6), 0.0, 0.0),
                     swing_known=False) for ti in t]
    assert math.isnan(swing_from_views([moving], 5.0))
    mixed = moving + [_frame(4.0, swing=0.003)]
    assert swing_from_views([mixed], 5.0) == pytest.approx(0.003)
    earlier = [_frame(0.1 * i, swing=0.006) for i in range(3)]
    later = [_frame(2.0 + 0.1 * i, swing_known=False) for i in range(3)]
    assert swing_from_views([earlier, later], 5.0) == pytest.approx(0.006)
    assert swing_from_views([earlier, [_frame(9.0, swing=0.001)]], 5.0) == pytest.approx(0.001)


def test_unknown_swing_propagates_to_record_and_revision():
    p = params()
    store = ModelStore(p)
    store.set_epoch(1)
    store.add_frame('t', _frame(0.0, swing_known=False))
    _fuse(store, p)
    rec = store.record('t')
    assert math.isnan(rec.swing_amplitude_m) and math.isnan(rec.fruit_top_offset_m)
    store.add_frame('t', _frame(0.1, swing=0.002))
    assert _fuse(store, p) == ['t']
    assert store.record('t').swing_amplitude_m == pytest.approx(0.002)
    assert store.record('t').revision == rec.revision + 1
