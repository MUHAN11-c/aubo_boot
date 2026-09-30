import numpy as np
from peach2_core.types import invalid_landmark, Landmark
from peach2_interfaces.msg import BagLandmark, TargetObservation
from peach2_perception.detector import Detection
from peach2_perception.lock_policy import LockState, LockStatus
from peach2_perception.msg_conversion import (bag_landmark_msg, NOT_IN_LOCKED_SET,
                                              observation_array_msg, pose_msg)
from peach2_perception.observation_builder import (ANCHOR_3D, CATEGORY_BAG, Measurement,
                                                   ObservationRecord)
from perception_fixtures import camera_pose, R_BASE_CAM
import pytest
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header


def _measurement():
    bottom = Landmark(True, np.array([0.6, 0.0, 0.35]), np.diag([1e-6, 2e-6, 4e-5]), 0.8)
    neck = Landmark(True, np.array([0.6, 0.0, 0.50]), np.diag([1e-6, 1e-6, 6e-5]), 0.7)
    tie = Landmark(True, np.array([0.6, 0.0, 0.54]), np.diag([2e-6, 2e-6, 2.4e-4]), 0.3)
    return Measurement(
        detection=Detection((10, 20, 110, 220), 0, 'peach_bag', 0.9), category=CATEGORY_BAG,
        mask=None, landmarks2d=None, landmarks3d=None, bottom=bottom, neck=neck, tie=tie,
        axis=np.array([0.0, 0.0, 1.0]), d95_m=0.08, camera_distance_m=0.57, mask_quality=0.9,
        depth_coverage=0.95, edge_touch=False, anchor=bottom.position.copy(),
        anchor_cov=bottom.cov.copy(), anchor_source=ANCHOR_3D, flags=['x'])


def _status(state, ids=frozenset()):
    return LockStatus(state, 3, frozenset(ids), '', 0, 0.0)


def _header():
    h = Header(frame_id='base_link')
    h.stamp.sec, h.stamp.nanosec = 12, 500
    return h


def test_bag_landmark_msg_keeps_each_covariance():
    lm = _measurement().tie
    msg = bag_landmark_msg(lm)
    assert msg.valid and msg.source == BagLandmark.SOURCE_GEOMETRY
    assert np.allclose(np.reshape(msg.covariance, (3, 3)), lm.cov)
    assert msg.confidence == pytest.approx(0.3)
    assert not bag_landmark_msg(invalid_landmark()).valid
    nan_cov = Landmark(True, np.zeros(3), np.full((3, 3), np.nan), 0.5)
    assert not bag_landmark_msg(nan_cov).valid


def test_pose_msg_is_camera_in_base():
    T = camera_pose((0.0, 0.1, 0.5))
    p = pose_msg(T)
    assert (p.position.x, p.position.y, p.position.z) == pytest.approx((0.0, 0.1, 0.5))
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    assert np.allclose(Rotation.from_quat(q).as_matrix(), R_BASE_CAM)


def test_array_fills_camera_pose_swing_known_and_locked_ids():
    m = _measurement()
    recs = [ObservationRecord(m, 'target_2', True, True, 0.01, 1.2),
            ObservationRecord(_measurement(), 'target_5', True, False, 0.0, 0.0)]
    T = camera_pose()
    out = observation_array_msg(_header(), T, _status(LockState.LOCKED, {'target_2', 'target_1'}),
                                recs)
    assert out.header.frame_id == 'base_link' and out.header.stamp.sec == 12
    assert out.scene_epoch == 3 and out.target_set_locked
    assert list(out.locked_target_ids) == ['target_1', 'target_2']
    a, b = out.observations
    assert a.header.stamp.nanosec == 500 and a.category == TargetObservation.CATEGORY_BAG
    assert a.camera_pose == pose_msg(T) and b.camera_pose == pose_msg(T)
    assert a.swing_known and a.swing_amplitude_m == pytest.approx(0.01)
    assert a.swing_period_s == pytest.approx(1.2)
    assert not b.swing_known and b.swing_amplitude_m == 0.0 and b.swing_period_s == 0.0
    assert not np.allclose(a.tie.covariance, a.neck.covariance)
    assert (a.roi.x_offset, a.roi.y_offset, a.roi.width, a.roi.height) == (10, 20, 100, 200)
    assert NOT_IN_LOCKED_SET not in a.flags and NOT_IN_LOCKED_SET in b.flags
    assert list(a.flags) == ['x']


def test_locked_ids_empty_until_locked():
    rec = ObservationRecord(_measurement(), 'target_1', True, False, 0.0, 0.0)
    out = observation_array_msg(_header(), np.eye(4), _status(LockState.COLLECTING, {'target_1'}),
                                [rec])
    assert not out.target_set_locked and list(out.locked_target_ids) == []
    assert NOT_IN_LOCKED_SET not in out.observations[0].flags
