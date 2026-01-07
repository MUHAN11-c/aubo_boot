#!/usr/bin/env python3
"""
验证手眼标定的数学推导和代码实现
"""

import numpy as np

def print_section(title):
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)

print_section("手眼标定数学验证")

print("\n【坐标系定义】")
print("- Base: 机器人基座坐标系")
print("- Gripper: 机器人末端执行器坐标系")
print("- Camera: 相机坐标系")
print("- Board: 标定板坐标系")

print("\n【变换矩阵定义】")
print("- T_gripper_i: Base → Gripper_i (机器人末端在基座中的位姿)")
print("- X: Gripper → Camera (相机在末端中的固定位姿) [待求]")
print("- T_board_i: Board → Camera (solvePnP返回：标定板在相机中的位姿)")

print("\n【关键约束】")
print("标定板在世界中是固定的，所以标定板在基座坐标系中的位置在两个时刻应该相同：")
print("\n位置1: Base → Gripper1 → Camera → Board")
print("  T_base_to_board = T_gripper1 @ X @ T_board1")
print("\n位置2: Base → Gripper2 → Camera → Board")
print("  T_base_to_board = T_gripper2 @ X @ T_board2")
print("\n约束方程：")
print("  T_gripper1 @ X @ T_board1 = T_gripper2 @ X @ T_board2")

print("\n【推导AX=XB】")
print("\n从约束方程：")
print("  T_gripper1 @ X @ T_board1 = T_gripper2 @ X @ T_board2")
print("\n两边左乘 inv(T_gripper1)：")
print("  X @ T_board1 = inv(T_gripper1) @ T_gripper2 @ X @ T_board2")
print("\n两边右乘 inv(T_board2)：")
print("  X @ T_board1 @ inv(T_board2) = inv(T_gripper1) @ T_gripper2 @ X")
print("\n定义：")
print("  A = inv(T_gripper1) @ T_gripper2  (机器人运动)")
print("  B = T_board1 @ inv(T_board2)      (标定板运动)")
print("\n得到：X @ B = A @ X")
print("即：A @ X = X @ B  ✅")

print_section("代码实现检查")

print("\n【当前代码（第1142-1148行）】")
print("A = T_gripper2 @ inv(T_gripper1)    ❌ 错误！应该是反过来")
print("B = inv(T_board2) @ T_board1        ❌ 错误！应该是反过来")

print("\n【正确应该是】")
print("A = inv(T_gripper1) @ T_gripper2    ✅ 正确")
print("B = T_board1 @ inv(T_board2)        ✅ 正确")

print("\n【验证残差函数（第1882行）】")
print("代码：res = Ra_list[i] @ Rx - Rx @ Rb_list[i]")
print("这对应：A @ X - X @ B = 0")
print("即：A @ X = X @ B  ✅ 公式形式正确")

print_section("单位一致性检查")

print("\n【机器人位姿】")
print("来源：/api/robot_status")
print("单位：米 (m)")
print("例如：position.x = 0.318 (表示318mm)")

print("\n【标定板位姿】")
print("来源：solvePnP (obj_points使用 square_size=15.0)")
print("单位：毫米 (mm)")
print("例如：position.x = 15.2 (表示15.2mm)")

print("\n【问题】")
print("❌ T_gripper的平移部分：米 (m)")
print("❌ T_board的平移部分：毫米 (mm)")
print("❌ 两者参与AX=XB计算时，平移部分单位不一致！")

print("\n【解决方案】")
print("✅ 在构建T_gripper时，将米转换为毫米：")
print("   T_gripper = pose_to_transform([x*1000, y*1000, z*1000], quat)")

print_section("数值示例验证")

print("\n假设一个简单的情况：")
print("- 机器人从位置1移动到位置2，Z轴上升100mm")
print("- 标定板在相机中的观测Z值从300mm变为400mm")

# 创建简单的测试数据
T_g1 = np.eye(4)
T_g1[:3, 3] = [0, 0, 0.5]  # 机器人位置1: Z=500mm (单位：米)

T_g2 = np.eye(4)
T_g2[:3, 3] = [0, 0, 0.6]  # 机器人位置2: Z=600mm (移动100mm)

T_b1 = np.eye(4)
T_b1[:3, 3] = [0, 0, 300]  # 标定板位置1: Z=300mm (单位：毫米)

T_b2 = np.eye(4)
T_b2[:3, 3] = [0, 0, 400]  # 标定板位置2: Z=400mm (移动100mm)

print("\n【错误的实现（当前代码）】")
A_wrong = T_g2 @ np.linalg.inv(T_g1)
B_wrong = np.linalg.inv(T_b2) @ T_b1
print(f"A (错误):\n{A_wrong[:3, 3]}  (单位混乱：米)")
print(f"B (错误):\n{B_wrong[:3, 3]}  (单位：毫米)")

print("\n【正确的实现】")
# 先将机器人位姿转换为毫米
T_g1_mm = T_g1.copy()
T_g1_mm[:3, 3] *= 1000  # 转换为毫米
T_g2_mm = T_g2.copy()
T_g2_mm[:3, 3] *= 1000

A_correct = np.linalg.inv(T_g1_mm) @ T_g2_mm
B_correct = T_b1 @ np.linalg.inv(T_b2)
print(f"A (正确):\n{A_correct[:3, 3]}  (单位：毫米)")
print(f"B (正确):\n{B_correct[:3, 3]}  (单位：毫米)")

print("\n【结论】")
print("✅ A和B的平移部分单位都是毫米")
print("✅ 求解出的X的平移部分单位也是毫米")

print_section("总结")

print("\n发现的问题：")
print("1. ❌ A矩阵计算顺序错误")
print("2. ❌ B矩阵计算顺序错误")
print("3. ⚠️  单位不一致（机器人：米，标定板：毫米）")

print("\n影响：")
print("- 标定结果完全错误")
print("- 必须同时修复所有3个问题")

print("\n修复方案：")
print("修改 hand_eye_calibration_node.py 第1122-1148行")
print("")
