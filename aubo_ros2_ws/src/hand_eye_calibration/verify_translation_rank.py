#!/usr/bin/env python3
"""
检查(I-Ra)矩阵的秩和条件数
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

print("="*70)
print("  检查(I-Ra)矩阵的秩")
print("="*70)

# 创建测试数据
r2 = R.from_euler('z', 25, degrees=True)
Ra = r2.as_matrix() if hasattr(r2, 'as_matrix') else r2.as_dcm()

I_minus_Ra = np.eye(3) - Ra

print(f"\nRa (绕Z轴旋转25度):")
print(Ra)

print(f"\n(I - Ra):")
print(I_minus_Ra)

print(f"\n矩阵秩: {np.linalg.matrix_rank(I_minus_Ra)}")
print(f"条件数: {np.linalg.cond(I_minus_Ra)}")

# 计算特征值
eigenvalues = np.linalg.eigvals(I_minus_Ra)
print(f"\n特征值: {eigenvalues}")

print("\n【关键发现】")
print("当只有单个运动时，(I-Ra)可能秩不足！")
print("这就是为什么AX=XB方法需要多组运动数据。")

print("\n【模拟多组数据】")
# 创建3组不同的运动
rotations = [
    R.from_euler('z', 25, degrees=True),
    R.from_euler('x', 20, degrees=True),
    R.from_euler('y', 15, degrees=True),
]

A_blocks = []
for r in rotations:
    Ra = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
    A_blocks.append(np.eye(3) - Ra)

A_stacked = np.vstack(A_blocks)
print(f"\n堆叠后的A矩阵形状: {A_stacked.shape}")
print(f"堆叠后的秩: {np.linalg.matrix_rank(A_stacked)}")
print(f"堆叠后的条件数: {np.linalg.cond(A_stacked)}")

print("\n✅ 通过多组运动的堆叠，矩阵变为满秩！")
print("这就是为什么代码中对多个运动组使用vstack:")
print("A_mat = np.vstack(A_mat)  # 第1920行")

print("\n" + "="*70)
