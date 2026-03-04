#!/usr/bin/env python3
"""Non-ROS2 demo: use core only (project, unproject, reprojection error, rotation)."""
from coordinate_transforms_py.core import (
    CameraIntrinsics,
    project_3d_to_2d,
    unproject_2d_to_3d,
    reprojection_error,
    reprojection_error_rms,
    rpy_to_quaternion,
    quaternion_to_rpy,
    transform_point,
)
import numpy as np

def main():
    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    p3d = np.array([0.1, 0.0, 1.0])
    uv = project_3d_to_2d(p3d, K)
    print("project_3d_to_2d:", p3d, "->", uv)

    p3d_back = unproject_2d_to_3d(uv[0], uv[1], 1.0, K)
    print("unproject_2d_to_3d:", uv, ", Z=1 ->", p3d_back)

    obs_uv = uv + np.array([2.0, -1.0])
    err = reprojection_error(obs_uv, p3d, K)
    print("reprojection_error(obs_uv, p3d):", err)

    q = rpy_to_quaternion(0.1, 0.2, 0.3)
    rpy = quaternion_to_rpy(q)
    print("rpy_to_quaternion / quaternion_to_rpy:", rpy)

    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = 1.0, 2.0, 3.0
    p = np.array([0.0, 0.0, 0.0])
    p2 = transform_point(p, T)
    print("transform_point([0,0,0], T):", p2)
    print("run_core_demo done.")

if __name__ == '__main__':
    main()
