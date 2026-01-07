#!/usr/bin/env python3
"""
最终验证：使用多组数据验证AX=XB中旋转和平移的求解公式
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def rot_to_matrix(r):
    return r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()

print("="*70)
print("  AX=XB 多组数据完整验证")
print("="*70)

# 1. 设定真实的X (相机到末端的变换)
Rx_true = rot_to_matrix(R.from_euler('xyz', [10, 20, 30], degrees=True))
tx_true = np.array([80.0, -20.0, 120.0])  # mm
X_true = np.eye(4)
X_true[:3, :3] = Rx_true
X_true[:3, 3] = tx_true

print(f"设定的真实X (相机→末端的变换):")
print(f"tx_true = {tx_true}")

# 2. 设定标定板在世界中的固定位姿
T_world_to_board = np.eye(4)
T_world_to_board[:3, :3] = rot_to_matrix(R.from_euler('xy', [0, 0], degrees=True))
T_world_to_board[:3, 3] = [200, 100, 0]  # mm

# 3. 创建多组机器人运动
n_motions = 3
A_list = []
B_list = []

T_grippers = [
    (np.array([0, 0, 500]), R.from_euler('z', 0, degrees=True)),
    (np.array([50, 80, 600]), R.from_euler('z', 25, degrees=True)),
    (np.array([-30, 100, 550]), R.from_euler('x', 20, degrees=True)),
    (np.array([70, -40, 580]), R.from_euler('y', 15, degrees=True)),
]

for i in range(n_motions):
    # 第i组的两个位姿
    pos1, rot1 = T_grippers[i]
    pos2, rot2 = T_grippers[i+1]
    
    T_gripper1 = np.eye(4)
    T_gripper1[:3, :3] = rot_to_matrix(rot1)
    T_gripper1[:3, 3] = pos1
    
    T_gripper2 = np.eye(4)
    T_gripper2[:3, :3] = rot_to_matrix(rot2)
    T_gripper2[:3, 3] = pos2
    
    # 计算A (修复后的公式)
    A = np.linalg.inv(T_gripper1) @ T_gripper2
    
    # 计算标定板在相机中的位姿
    T_camera1 = T_gripper1 @ X_true
    T_board1 = np.linalg.inv(T_camera1) @ T_world_to_board
    
    T_camera2 = T_gripper2 @ X_true
    T_board2 = np.linalg.inv(T_camera2) @ T_world_to_board
    
    # 计算B (修复后的公式)
    B = T_board1 @ np.linalg.inv(T_board2)
    
    A_list.append(A)
    B_list.append(B)
    
    # 验证约束
    error = np.linalg.norm(A @ X_true - X_true @ B)
    print(f"\n运动组 #{i+1}: || A@X - X@B || = {error:.10f}")

print(f"\n✅ 生成了{n_motions}组满足AX=XB的数据")

# 4. 提取旋转和平移
Ra_list = [A[:3, :3] for A in A_list]
ta_list = [A[:3, 3] for A in A_list]
Rb_list = [B[:3, :3] for B in B_list]
tb_list = [B[:3, 3] for B in B_list]

print("\n"+"="*70)
print("  步骤1：求解旋转（假设已知Rx_true用于测试）")
print("="*70)

# 验证旋转约束
for i in range(n_motions):
    left = Ra_list[i] @ Rx_true
    right = Rx_true @ Rb_list[i]
    error = np.linalg.norm(left - right)
    print(f"运动组 #{i+1}: || Ra@Rx - Rx@Rb || = {error:.10f}")

print("\n"+"="*70)
print("  步骤2：求解平移（使用堆叠的线性系统）")
print("="*70)

print("\n【方法1：代码中的实现】")
print("公式: (I - Ra) @ tx = Rx @ tb - ta")

A_mat_code = []
b_vec_code = []
for i in range(n_motions):
    A_block = np.eye(3) - Ra_list[i]
    b_block = Rx_true @ tb_list[i] - ta_list[i]
    A_mat_code.append(A_block)
    b_vec_code.append(b_block)

A_mat_code = np.vstack(A_mat_code)
b_vec_code = np.hstack(b_vec_code)

print(f"堆叠后的A矩阵形状: {A_mat_code.shape}")
print(f"矩阵秩: {np.linalg.matrix_rank(A_mat_code)}")

tx_solved_code, residuals, rank, s = np.linalg.lstsq(A_mat_code, b_vec_code, rcond=None)
error_code = np.linalg.norm(tx_solved_code - tx_true)

print(f"\n求解的 tx: {tx_solved_code}")
print(f"真实的 tx: {tx_true}")
print(f"误差: {error_code:.10f}")

if error_code < 1e-6:
    print("✅ 代码方法正确")
else:
    print("❌ 代码方法错误")

print("\n【方法2：备选公式】")
print("公式: (I - Ra) @ tx = ta - Rx @ tb")

A_mat_alt = []
b_vec_alt = []
for i in range(n_motions):
    A_block = np.eye(3) - Ra_list[i]
    b_block = ta_list[i] - Rx_true @ tb_list[i]
    A_mat_alt.append(A_block)
    b_vec_alt.append(b_block)

A_mat_alt = np.vstack(A_mat_alt)
b_vec_alt = np.hstack(b_vec_alt)

tx_solved_alt, residuals, rank, s = np.linalg.lstsq(A_mat_alt, b_vec_alt, rcond=None)
error_alt = np.linalg.norm(tx_solved_alt - tx_true)

print(f"\n求解的 tx: {tx_solved_alt}")
print(f"真实的 tx: {tx_true}")
print(f"误差: {error_alt:.10f}")

if error_alt < 1e-6:
    print("✅ 备选方法正确")
else:
    print("❌ 备选方法错误")

print("\n"+"="*70)
print("  【最终结论】")
print("="*70)

if error_code < 1e-6:
    print("\n✅ 代码中的平移求解公式是正确的！")
    print("   公式: (I - Ra) @ tx = Rx @ tb - ta")
    print("   位置: hand_eye_calibration_node.py 第1915-1916行")
elif error_alt < 1e-6:
    print("\n❌ 代码中的平移求解公式是错误的！")
    print("   当前代码: (I - Ra) @ tx = Rx @ tb - ta")
    print("   应该改为: (I - Ra) @ tx = ta - Rx @ tb")
    print("   需要修改第1916行的符号")
else:
    print("\n⚠️ 两种方法都不准确，需要进一步检查")

print("\n" + "="*70)
