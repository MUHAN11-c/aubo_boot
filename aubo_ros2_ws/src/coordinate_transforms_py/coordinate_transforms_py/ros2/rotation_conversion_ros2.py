# ROS2: geometry_msgs Quaternion/Pose <-> core
from geometry_msgs.msg import Quaternion, Pose
from coordinate_transforms_py.core import rotation_conversion as rc
import numpy as np

def to_core_quaternion(q: Quaternion) -> np.ndarray:
    return np.array([q.x, q.y, q.z, q.w])

def to_ros_quaternion(q) -> Quaternion:
    q = np.asarray(q).flatten()
    out = Quaternion()
    out.x, out.y, out.z, out.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return out

def pose_to_origin_rotation(pose: Pose):
    origin = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = to_core_quaternion(pose.orientation)
    R = rc.quaternion_to_rotation_matrix(q)
    return origin, R

def origin_rotation_to_pose(origin, R) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(origin[0]), float(origin[1]), float(origin[2])
    q = rc.rotation_matrix_to_quaternion(R)
    pose.orientation = to_ros_quaternion(q)
    return pose

def rpy_to_ros_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    q = rc.rpy_to_quaternion(roll, pitch, yaw)
    return to_ros_quaternion(q)

def ros_quaternion_to_rpy(q: Quaternion) -> np.ndarray:
    cq = to_core_quaternion(q)
    return rc.quaternion_to_rpy(cq)
