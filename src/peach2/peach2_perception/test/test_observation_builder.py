import numpy as np
from peach2_core.types import invalid_landmark, Landmark
from peach2_perception import observation_builder
from peach2_perception.detector import Detection
from peach2_perception.observation_builder import (ANCHOR_2D, ANCHOR_3D, ANCHOR_BOX,
                                                   CATEGORY_BAG, CATEGORY_NOBAG,
                                                   cross_check_landmark, FrameData,
                                                   gravity_in_image, Measurement,
                                                   ObservationBuilder, pose_from_matrix,
                                                   transform_matrix)
from peach2_perception.stationary import Motion
from perception_fixtures import (bag_instance as _bag_instance, bag_points,
                                 BUILDER_PARAMS as PARAMS, camera_pose, K,
                                 make_frame as _frame, mask_bbox, R_BASE_CAM)
import pytest
from scipy.spatial.transform import Rotation

BOTTOM = np.array([0.6, 0.0, 0.35])
NECK = np.array([0.6, 0.0, 0.50])


def test_transform_matrix():
    q = Rotation.from_matrix(R_BASE_CAM).as_quat()
    T = transform_matrix((1.0, 2.0, 3.0), q)
    assert np.allclose(T[:3, :3], R_BASE_CAM) and np.allclose(T[:3, 3], [1, 2, 3])


def test_gravity_in_image():
    g = gravity_in_image(K, R_BASE_CAM, (320, 240), 0.6)
    assert np.allclose(g, [0.0, 1.0], atol=1e-9)
    looking_down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    assert gravity_in_image(K, looking_down, (320, 240), 0.6) is None
    rolled = R_BASE_CAM @ Rotation.from_euler('z', 90, degrees=True).as_matrix()
    g = gravity_in_image(K, rolled, (320, 240), 0.6)
    assert abs(abs(g[0]) - 1.0) < 1e-9
    assert gravity_in_image(K, R_BASE_CAM, (320, 240), float('nan')) is None


def test_bag_landmarks_in_base_link_with_rotated_covariance():
    T = camera_pose()
    frame, lab = _frame(T, [bag_points()])
    m = ObservationBuilder(PARAMS).measure(frame, [_bag_instance(lab)])[0]
    assert m.category == CATEGORY_BAG and m.anchor_source == ANCHOR_3D
    assert m.bottom.valid and m.neck.valid
    assert np.linalg.norm(m.bottom.position - BOTTOM) < 0.012
    assert np.linalg.norm(m.neck.position - NECK) < 0.012
    assert m.axis @ np.array([0.0, 0.0, 1.0]) > 0.99
    assert 0.07 < m.d95_m < 0.09
    assert m.camera_distance_m == pytest.approx(0.57, abs=0.01)
    assert not any('mismatch' in f for f in m.flags)
    assert max(m.cross_check_dev_m.values()) < 0.015
    # Axial (base Z) uncertainty dominates: the covariance is expressed in base_link.
    assert m.bottom.cov[2, 2] > m.bottom.cov[0, 0]
    assert m.depth_coverage > 0.9 and m.mask_quality > 0.8 and not m.edge_touch


def test_tie_has_its_own_covariance_and_cross_check():
    frame, lab = _frame(camera_pose(), [bag_points()])
    m = ObservationBuilder(PARAMS).measure(frame, [_bag_instance(lab)])[0]
    assert m.tie.valid and m.tie.position[2] > m.neck.position[2]
    assert m.tie.cov is not m.neck.cov and not np.allclose(m.tie.cov, m.neck.cov)
    assert np.all(np.linalg.eigvalsh(m.tie.cov) > 0.0)
    assert m.tie.cov[2, 2] > m.neck.cov[2, 2]  # tip position is less certain along the axis
    assert m.cross_check_dev_m['tie'] < 0.015


def test_tie_mismatch_invalidates_only_the_tie(monkeypatch):
    real = observation_builder.landmarks_from_points

    def shifted_tie(*args, **kwargs):
        lm3 = real(*args, **kwargs)
        lm3.tie.position = lm3.tie.position + np.array([0.0, 0.04, 0.0])
        return lm3

    monkeypatch.setattr(observation_builder, 'landmarks_from_points', shifted_tie)
    frame, lab = _frame(camera_pose(), [bag_points()])
    m = ObservationBuilder(PARAMS).measure(frame, [_bag_instance(lab)])[0]
    assert not m.tie.valid and 'tie_2d3d_mismatch' in m.flags
    assert m.bottom.valid and m.neck.valid
    assert np.allclose(m.anchor_cov, m.bottom.cov)


def test_same_bag_from_two_camera_poses_agrees_in_base():
    T1 = camera_pose((0.0, 0.0, 0.5))
    R2 = Rotation.from_euler('z', 20, degrees=True).as_matrix() @ \
        Rotation.from_euler('y', 15, degrees=True).as_matrix() @ R_BASE_CAM
    T2 = camera_pose((0.05, -0.2, 0.62), R2)
    b = ObservationBuilder(PARAMS)
    f1, l1 = _frame(T1, [bag_points()])
    f2, l2 = _frame(T2, [bag_points()])
    m1 = b.measure(f1, [_bag_instance(l1)])[0]
    m2 = b.measure(f2, [_bag_instance(l2)])[0]
    assert m1.bottom.valid and m2.bottom.valid
    assert np.linalg.norm(m1.bottom.position - m2.bottom.position) < 0.012
    assert np.linalg.norm(m1.neck.position - m2.neck.position) < 0.015


def test_cross_check_landmark():
    T = camera_pose()
    depth = np.full((480, 640), 0.56, np.float32)
    px = (320.0, 300.0)
    ray_cam = np.array([(px[0] - K[0, 2]) / K[0, 0], (px[1] - K[1, 2]) / K[1, 1], 1.0])
    on_axis_cam = ray_cam * 0.6  # 40 mm behind the visible surface along the ray
    on_axis = (T[:3, :3] @ on_axis_cam) + T[:3, 3]
    ok, dev = cross_check_landmark(on_axis, px, depth, K, T, None, 1, 0.08, 0.015)
    assert ok and dev < 1e-3
    lateral = on_axis + np.array([0.0, 0.03, 0.0])
    ok, dev = cross_check_landmark(lateral, px, depth, K, T, None, 1, 0.08, 0.015)
    assert ok is False and dev == pytest.approx(0.03, abs=0.003)
    in_front = (T[:3, :3] @ (ray_cam * 0.50)) + T[:3, 3]
    ok, dev = cross_check_landmark(in_front, px, depth, K, T, None, 1, 0.08, 0.015)
    assert ok is False
    ok, dev = cross_check_landmark(on_axis, px, np.zeros_like(depth), K, T, None, 1, 0.08, 0.015)
    assert ok is None and np.isnan(dev)


def test_nobag_uses_box_centre_anchor_without_landmarks():
    T = camera_pose()
    frame, lab = _frame(T, [bag_points()])
    det = Detection(mask_bbox(lab == 0), 1, 'peach_nobag', 0.8)
    m = ObservationBuilder(PARAMS).measure(frame, [(det, None)])[0]
    assert m.category == CATEGORY_NOBAG and m.anchor_source == ANCHOR_BOX
    assert not m.bottom.valid and not m.neck.valid
    assert 'anchor_box_centre' in m.flags
    assert m.anchor[0] == pytest.approx(0.56, abs=0.02)


def test_3d_failure_falls_back_to_2d_bottom_anchor():
    T = camera_pose()
    frame, lab = _frame(T, [bag_points()])
    det, seg = _bag_instance(lab)
    sparse = np.zeros_like(frame.depth_m)
    vs, us = np.nonzero(seg.mask)
    v_bot = vs.max()
    u_c = int(np.median(us[vs == v_bot]))
    patch = (slice(v_bot - 6, v_bot - 1), slice(u_c - 2, u_c + 3))
    sparse[patch] = frame.depth_m[patch]
    frame = FrameData(0.0, K, T, sparse, frame.confidence)
    m = ObservationBuilder(PARAMS).measure(frame, [(det, seg)])[0]
    assert m.anchor_source == ANCHOR_2D and 'anchor_from_2d' in m.flags
    assert not m.bottom.valid
    assert np.linalg.norm(m.anchor - BOTTOM) < 0.03


def _measurement(pos, cov_sigma=0.005, category=CATEGORY_BAG):
    lm = Landmark(True, np.asarray(pos, float), 1e-6 * np.eye(3), 0.8)
    return Measurement(
        detection=Detection((0, 0, 10, 10), 0, 'peach_bag', 0.9), category=category, mask=None,
        landmarks2d=None, landmarks3d=None, bottom=lm,
        neck=Landmark(True, np.asarray(pos, float) + [0, 0, 0.15], 1e-6 * np.eye(3), 0.7),
        tie=invalid_landmark(), axis=np.array([0.0, 0.0, 1.0]), d95_m=0.08,
        camera_distance_m=0.6, mask_quality=1.0, depth_coverage=1.0, edge_touch=False,
        anchor=np.asarray(pos, float), anchor_cov=cov_sigma ** 2 * np.eye(3),
        anchor_source=ANCHOR_3D)


def test_tracking_confirms_on_third_frame_by_stamp():
    b = ObservationBuilder(PARAMS)
    recs = [b.associate(0.1 * i, [_measurement(BOTTOM)], Motion.STATIONARY)[0]
            for i in range(3)]
    assert [r.confirmed for r in recs] == [False, False, True]
    assert {r.target_id for r in recs} == {'target_1'}
    confirmed, pending = b.track_summary()
    assert confirmed == frozenset({'target_1'}) and pending == 0
    b.reset()
    r = b.associate(1.0, [_measurement(BOTTOM)], Motion.STATIONARY)[0]
    assert r.target_id == 'target_2'


def test_moving_frames_invalidate_landmarks_unknown_only_flags():
    b = ObservationBuilder(PARAMS)
    r = b.associate(0.0, [_measurement(BOTTOM)], Motion.MOVING)[0]
    m = r.measurement
    assert not m.bottom.valid and not m.neck.valid and 'arm_moving' in m.flags
    r = b.associate(0.1, [_measurement(BOTTOM)], Motion.UNKNOWN)[0]
    assert r.measurement.bottom.valid and 'arm_motion_unknown' in r.measurement.flags


def test_swing_estimated_over_stationary_window_and_reset_on_motion():
    b = ObservationBuilder(PARAMS)
    amp, period = 0.02, 1.0
    rec = None
    for i in range(31):
        t = i / 15.0
        pos = BOTTOM + [0.0, amp * np.sin(2 * np.pi * t / period), 0.0]
        rec = b.associate(t, [_measurement(pos)], Motion.STATIONARY)[0]
    assert rec.target_id == 'target_1' and rec.swing_known
    assert rec.swing_amplitude_m == pytest.approx(amp, abs=0.004)
    assert rec.swing_period_s == pytest.approx(period, abs=0.1)
    b.associate(2.1, [_measurement(BOTTOM)], Motion.MOVING)
    rec = b.associate(2.2, [_measurement(BOTTOM)], Motion.STATIONARY)[0]
    assert not rec.swing_known
    assert rec.swing_amplitude_m == 0.0 and rec.swing_period_s == 0.0


def test_swing_known_for_a_still_bag_with_zero_amplitude():
    b = ObservationBuilder(PARAMS)
    rec = None
    for i in range(31):
        rec = b.associate(i / 15.0, [_measurement(BOTTOM)], Motion.STATIONARY)[0]
        if i == 0:
            assert not rec.swing_known
    assert rec.swing_known and rec.swing_amplitude_m < 1e-3


def test_pose_from_matrix_round_trip():
    q = Rotation.from_matrix(R_BASE_CAM).as_quat()
    T = transform_matrix((0.1, -0.2, 0.5), q)
    t, q_out = pose_from_matrix(T)
    assert np.allclose(t, [0.1, -0.2, 0.5]) and q_out[3] >= 0.0
    assert np.allclose(transform_matrix(t, q_out), T)
    assert np.allclose(pose_from_matrix(np.eye(4))[1], [0.0, 0.0, 0.0, 1.0])
