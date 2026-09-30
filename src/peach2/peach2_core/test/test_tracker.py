import numpy as np
from peach2_core.tracker import Detection3D, Tracker
import pytest

R = (0.005 ** 2) * np.eye(3)


def _det(p, category=0, payload=None):
    return Detection3D(position=np.asarray(p, dtype=float), cov=R.copy(), category=category,
                       payload=payload)


def test_confirmation_needs_consecutive_hits():
    tr = Tracker(confirm_hits=3)
    out = tr.update(0.0, [_det([0.5, 0.0, 0.8])])
    assert len(out) == 1 and not out[0].confirmed
    tid = out[0].track_id
    tr.update(0.2, [_det([0.5, 0.0, 0.8])])
    out = tr.update(0.4, [_det([0.5, 0.0, 0.8])])
    assert out[0].track_id == tid and out[0].confirmed and out[0].hits == 3


def test_tentative_track_dropped_on_miss():
    tr = Tracker(confirm_hits=3)
    tr.update(0.0, [_det([0.5, 0.0, 0.8])])
    tr.update(0.2, [])
    assert tr.tracks() == []


def test_two_targets_crossing_keep_identity():
    rng = np.random.default_rng(0)
    tr = Tracker(confirm_hits=3, process_accel_sigma=0.2)
    va = np.array([0.1, 0.0, 0.0])
    vb = np.array([-0.1, 0.0, 0.0])
    pa0 = np.array([0.3, 0.0, 0.80])
    pb0 = np.array([0.7, 0.0, 0.84])
    ids = None
    for k in range(40):
        t = 0.1 * k
        pa = pa0 + va * t + rng.normal(0.0, 0.003, 3)
        pb = pb0 + vb * t + rng.normal(0.0, 0.003, 3)
        out = tr.update(t, [_det(pb, payload='b'), _det(pa, payload='a')])
        by_payload = {o.payload: o.track_id for o in out}
        if ids is None:
            ids = by_payload
        assert by_payload == ids, f'identity swap at t={t:.1f}'
    tracks = {t.track_id: t for t in tr.tracks()}
    assert np.allclose(tracks[ids['a']].velocity, va, atol=0.03)
    assert np.allclose(tracks[ids['b']].velocity, vb, atol=0.03)


def test_lost_and_reacquired_within_ttl_keeps_id_and_after_ttl_gets_new_id():
    tr = Tracker(confirm_hits=2, ttl_s=3.0)
    p = [0.5, 0.1, 0.8]
    for k in range(3):
        out = tr.update(0.1 * k, [_det(p)])
    tid = out[0].track_id
    tr.update(1.0, [])
    tr.update(2.0, [])
    out = tr.update(2.5, [_det(p)])
    assert out[0].track_id == tid
    tr.update(3.0, [])
    out = tr.update(6.0, [_det(p)])
    assert out[0].track_id != tid
    assert len(tr.tracks()) == 1


def test_category_constraint_and_gate():
    tr = Tracker(confirm_hits=1)
    a = tr.update(0.0, [_det([0.5, 0.0, 0.8], category=0)])[0].track_id
    out = tr.update(0.1, [_det([0.5, 0.0, 0.8], category=1)])
    assert out[0].track_id != a
    out = tr.update(0.2, [_det([0.9, 0.0, 0.8], category=0)])
    assert out[0].track_id not in (a,)


def test_reset_keeps_id_counter():
    tr = Tracker(confirm_hits=1, id_prefix='t')
    first = tr.update(0.0, [_det([0.5, 0.0, 0.8])])[0].track_id
    tr.reset()
    assert tr.tracks() == []
    second = tr.update(0.1, [_det([0.5, 0.0, 0.8])])[0].track_id
    assert first == 't1' and second == 't2'


def test_returned_tracks_are_snapshots():
    tr = Tracker(confirm_hits=1)
    out = tr.update(0.0, [_det([0.5, 0.0, 0.8])])
    out[0].position[:] = 99.0
    assert tr.tracks()[0].position[0] == pytest.approx(0.5)


def test_invalid_parameters_and_covariance():
    with pytest.raises(ValueError):
        Tracker(confirm_hits=0)
    tr = Tracker()
    with pytest.raises(ValueError):
        tr.update(0.0, [Detection3D(np.zeros(3), np.full((3, 3), np.nan), 0)])
