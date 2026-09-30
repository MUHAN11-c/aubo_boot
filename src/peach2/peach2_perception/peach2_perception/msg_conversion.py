"""Pure conversions from builder records to peach2_interfaces messages (no rclpy)."""
from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
from peach2_core.types import Landmark
from peach2_interfaces.msg import BagLandmark, TargetObservation, TargetObservationArray
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header

from .lock_policy import LockStatus
from .observation_builder import ObservationRecord, pose_from_matrix

NOT_IN_LOCKED_SET = 'not_in_locked_set'


def bag_landmark_msg(lm: Landmark) -> BagLandmark:
    out = BagLandmark()
    if not lm.valid or not np.all(np.isfinite(lm.position)) or not np.all(np.isfinite(lm.cov)):
        return out
    out.valid = True
    out.source = BagLandmark.SOURCE_GEOMETRY
    out.position = Point(x=float(lm.position[0]), y=float(lm.position[1]),
                         z=float(lm.position[2]))
    out.covariance = [float(x) for x in np.asarray(lm.cov, dtype=np.float64).reshape(9)]
    out.confidence = float(np.clip(lm.confidence, 0.0, 1.0))
    return out


def pose_msg(T: np.ndarray) -> Pose:
    """Pose of a 4x4 transform (e.g. base_link <- camera optical at the image stamp)."""
    t, q = pose_from_matrix(T)
    return Pose(position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
                orientation=Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]),
                                       w=float(q[3])))


def observation_msg(rec: ObservationRecord, header: Header, camera_pose: Pose,
                    status: LockStatus) -> TargetObservation:
    m = rec.measurement
    obs = TargetObservation()
    obs.header = header
    obs.target_id = rec.target_id
    obs.category = (TargetObservation.CATEGORY_BAG if m.category == 0
                    else TargetObservation.CATEGORY_NOBAG)
    obs.confirmed = bool(rec.confirmed)
    obs.bottom = bag_landmark_msg(m.bottom)
    obs.neck = bag_landmark_msg(m.neck)
    obs.tie = bag_landmark_msg(m.tie)
    if np.all(np.isfinite(m.axis)):
        obs.axis = Vector3(x=float(m.axis[0]), y=float(m.axis[1]), z=float(m.axis[2]))
    obs.diameter95_m = float(m.d95_m) if math.isfinite(m.d95_m) else 0.0
    obs.camera_distance_m = float(m.camera_distance_m)
    obs.camera_pose = camera_pose
    obs.mask_quality = float(m.mask_quality)
    obs.depth_coverage = float(m.depth_coverage)
    obs.edge_touch = bool(m.edge_touch)
    obs.swing_known = bool(rec.swing_known)
    if rec.swing_known:
        obs.swing_amplitude_m = float(rec.swing_amplitude_m)
        obs.swing_period_s = float(rec.swing_period_s)
    x1, y1, x2, y2 = m.detection.bbox
    obs.roi = RegionOfInterest(x_offset=x1, y_offset=y1, width=x2 - x1, height=y2 - y1,
                               do_rectify=False)
    flags = list(m.flags)
    if status.locked and rec.confirmed and rec.target_id not in status.locked_ids:
        flags.append(NOT_IN_LOCKED_SET)
    obs.flags = flags
    return obs


def observation_array_msg(header: Header, T_base_cam: np.ndarray, status: LockStatus,
                          records: Sequence[ObservationRecord]) -> TargetObservationArray:
    """Array for one frame; header.stamp = image stamp, header.frame_id = base_link."""
    camera_pose = pose_msg(T_base_cam)
    out = TargetObservationArray()
    out.header = header
    out.scene_epoch = int(status.scene_epoch)
    out.target_set_locked = bool(status.locked)
    out.locked_target_ids = sorted(status.locked_ids) if status.locked else []
    out.observations = [observation_msg(r, header, camera_pose, status) for r in records]
    return out
