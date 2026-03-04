# ROS2: TransformStamped <-> 4x4; transform point/pose
from geometry_msgs.msg import TransformStamped, Point, Pose
from coordinate_transforms_py.core import point_transform as pt
from coordinate_transforms_py.ros2 import rotation_conversion_ros2 as rcr
import numpy as np

def transform_stamped_to_matrix4(t: TransformStamped) -> np.ndarray:
    from coordinate_transforms_py.core import rotation_conversion as rc
    tr = t.transform.translation
    r = t.transform.rotation
    q = [r.x, r.y, r.z, r.w]
    R = rc.quaternion_to_rotation_matrix(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3], T[1, 3], T[2, 3] = tr.x, tr.y, tr.z
    return T

def transform_point_ros2(p: Point, t: TransformStamped) -> Point:
    T = transform_stamped_to_matrix4(t)
    pc = np.array([p.x, p.y, p.z])
    out = pt.transform_point(pc, T)
    result = Point()
    result.x, result.y, result.z = float(out[0]), float(out[1]), float(out[2])
    return result

def transform_pose_ros2(pose: Pose, t: TransformStamped) -> Pose:
    origin, R = rcr.pose_to_origin_rotation(pose)
    T = transform_stamped_to_matrix4(t)
    new_origin, new_R = pt.transform_pose(origin, R, T)
    return rcr.origin_rotation_to_pose(new_origin, new_R)
