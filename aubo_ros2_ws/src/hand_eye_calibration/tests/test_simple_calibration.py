#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化测试：验证手眼标定公式的正确性
"""

import numpy as np
import cv2

# 地面真值
T_cam2gripper_true = np.eye(4)
T_cam2gripper_true[:3, :3] = cv2.Rodrigues(np.array([np.pi, 0, 0]))[0]
T_cam2gripper_true[:3, 3] = [50.0, 0.0, 150.0]

print("地面真值 T_cam2gripper:")
print(T_cam2gripper_true)
print(f"平移: {T_cam2gripper_true[:3, 3]}")

# 生成2个简单的机器人位姿
T_base2gripper1 = np.eye(4)
T_base2gripper1[:3, 3] = [400.0, 0.0, 300.0]

T_base2gripper2 = np.eye(4)
T_base2gripper2[:3, 3] = [400.0, 100.0, 300.0]  # Y方向移动100mm

# 标定板固定位置
T_base2board = np.eye(4)
T_base2board[:3, 3] = [400.0, 0.0, 0.0]

# 计算标定板在相机坐标系中的位姿
T_cam2gripper = T_cam2gripper_true
T_board2camera1 = T_cam2gripper @ np.linalg.inv(T_base2gripper1) @ T_base2board
T_board2camera2 = T_cam2gripper @ np.linalg.inv(T_base2gripper2) @ T_base2board

print("\n标定板位姿:")
print(f"位姿1: {T_board2camera1[:3, 3]}")
print(f"位姿2: {T_board2camera2[:3, 3]}")

# 计算A和B矩阵
A = np.linalg.inv(T_base2gripper1) @ T_base2gripper2
B = T_board2camera1 @ np.linalg.inv(T_board2camera2)

print("\nA矩阵（末端运动）:")
print(A)
print(f"A平移: {A[:3, 3]}")

print("\nB矩阵（标定板相对运动）:")
print(B)
print(f"B平移: {B[:3, 3]}")

# 验证AX = XB
X = T_cam2gripper_true
AX = A @ X
XB = X @ B

print("\n验证 AX = XB:")
print(f"AX平移: {AX[:3, 3]}")
print(f"XB平移: {XB[:3, 3]}")
print(f"差异: {np.linalg.norm(AX[:3, 3] - XB[:3, 3]):.6f} mm")

# 使用OpenCV标定
R_gripper2base1 = np.linalg.inv(T_base2gripper1)[:3, :3]
t_gripper2base1 = np.linalg.inv(T_base2gripper1)[:3, 3].reshape(3, 1) / 1000.0

R_gripper2base2 = np.linalg.inv(T_base2gripper2)[:3, :3]
t_gripper2base2 = np.linalg.inv(T_base2gripper2)[:3, 3].reshape(3, 1) / 1000.0

rvec1, _ = cv2.Rodrigues(T_board2camera1[:3, :3])
tvec1 = T_board2camera1[:3, 3].reshape(3, 1) / 1000.0

rvec2, _ = cv2.Rodrigues(T_board2camera2[:3, :3])
tvec2 = T_board2camera2[:3, 3].reshape(3, 1) / 1000.0

print("\nOpenCV输入:")
print(f"R_gripper2base1: \n{R_gripper2base1}")
print(f"t_gripper2base1 (米): {t_gripper2base1.flatten()}")
print(f"rvec1: {rvec1.flatten()}")
print(f"tvec1 (米): {tvec1.flatten()}")

try:
    R_cam2gripper_est, t_cam2gripper_est = cv2.calibrateHandEye(
        [R_gripper2base1, R_gripper2base2],
        [t_gripper2base1, t_gripper2base2],
        [rvec1, rvec2],
        [tvec1, tvec2],
        method=cv2.CALIB_HAND_EYE_DANIILIDIS
    )
    
    T_cam2gripper_est = np.eye(4)
    T_cam2gripper_est[:3, :3] = R_cam2gripper_est
    T_cam2gripper_est[:3, 3] = t_cam2gripper_est.flatten() * 1000.0
    
    print("\nOpenCV估计结果:")
    print(T_cam2gripper_est)
    print(f"平移: {T_cam2gripper_est[:3, 3]}")
    
    error = np.linalg.norm(T_cam2gripper_est[:3, 3] - T_cam2gripper_true[:3, 3])
    print(f"\n误差: {error:.6f} mm")
    
except Exception as e:
    print(f"\nOpenCV标定失败: {e}")

