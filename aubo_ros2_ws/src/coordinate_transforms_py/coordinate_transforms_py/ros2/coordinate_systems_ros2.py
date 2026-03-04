# ROS2: CameraInfo -> core intrinsics; project/unproject/reprojection_error
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from coordinate_transforms_py.core import CameraIntrinsics, project_3d_to_2d, unproject_2d_to_3d, reprojection_error

def camera_info_to_intrinsics(info: CameraInfo) -> CameraIntrinsics:
    K = CameraIntrinsics()
    if len(info.k) >= 9:
        K.fx = info.k[0]
        K.fy = info.k[4]
        K.cx = info.k[2]
        K.cy = info.k[5]
    return K

def project_3d_to_2d_point(point_3d: Point, info: CameraInfo) -> Point:
    import numpy as np
    K = camera_info_to_intrinsics(info)
    p = [point_3d.x, point_3d.y, point_3d.z]
    uv = project_3d_to_2d(p, K)
    out = Point()
    out.x = float(uv[0])
    out.y = float(uv[1])
    out.z = 0.0
    return out

def unproject_2d_to_3d_point(u: float, v: float, depth: float, info: CameraInfo) -> Point:
    K = camera_info_to_intrinsics(info)
    p = unproject_2d_to_3d(u, v, depth, K)
    out = Point()
    out.x = float(p[0])
    out.y = float(p[1])
    out.z = float(p[2])
    return out

def reprojection_error_ros2(point_2d_obs_uv: Point, point_3d: Point, info: CameraInfo, squared: bool = False) -> float:
    K = camera_info_to_intrinsics(info)
    uv = [point_2d_obs_uv.x, point_2d_obs_uv.y]
    p = [point_3d.x, point_3d.y, point_3d.z]
    return float(reprojection_error(uv, p, K, squared))
