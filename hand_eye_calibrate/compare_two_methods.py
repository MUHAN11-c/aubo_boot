#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比两种手眼标定方法的结果
"""

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

np.set_printoptions(precision=6, suppress=True)

def rotation_matrix_difference(R1, R2):
    """计算两个旋转矩阵之间的角度差异"""
    R_diff = R1 @ R2.T
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
    return np.degrees(angle)

def print_matrix_info(name, T, unit="米"):
    """打印变换矩阵信息"""
    print(f"\n【{name}】")
    print("完整变换矩阵:")
    print(T)
    
    R = T[:3, :3]
    t = T[:3, 3]
    
    print("\n旋转矩阵:")
    print(R)
    print(f"\n平移向量 ({unit}): [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    print(f"平移范数 ({unit}): {np.linalg.norm(t):.6f}")
    print(f"旋转矩阵行列式: {np.linalg.det(R):.9f}")
    
    # 检查正交性
    orthogonal_error = np.linalg.norm(R @ R.T - np.eye(3))
    print(f"正交性检验 (R@R.T - I的范数): {orthogonal_error:.9f}")
    if orthogonal_error < 1e-6:
        print("  ✅ 旋转矩阵正交性良好")
    else:
        print(f"  ⚠️ 旋转矩阵正交性误差: {orthogonal_error:.6e}")
    
    # 欧拉角
    try:
        r = R_scipy.from_matrix(R)
    except AttributeError:
        r = R_scipy.from_dcm(R)
    
    euler_xyz = r.as_euler('xyz', degrees=True)
    euler_zyx = r.as_euler('zyx', degrees=True)
    
    print(f"\n欧拉角 (XYZ, 度): [{euler_xyz[0]:.2f}°, {euler_xyz[1]:.2f}°, {euler_xyz[2]:.2f}°]")
    print(f"欧拉角 (ZYX, 度): [{euler_zyx[0]:.2f}°, {euler_zyx[1]:.2f}°, {euler_zyx[2]:.2f}°]")

def main():
    print("="*80)
    print("手眼标定结果对比：run_calibration.py vs 自动标定系统")
    print("="*80)
    
    # 方法1: run_calibration.py 结果（OpenCV TSAI，所有19张图片）
    print("\n" + "="*80)
    print("方法1：run_calibration.py（OpenCV TSAI算法，19张图片）")
    print("="*80)
    
    T1 = np.array([
        [ 0.2581491,   0.06549221,  0.96388268, -0.25415469],
        [ 0.71116341,  0.6624187,  -0.23547415, -0.08287154],
        [-0.65391563,  0.74626553,  0.12442712,  0.02411134],
        [ 0.,          0.,          0.,          1.        ]
    ])
    
    camera_matrix_1 = np.array([
        [471.14132317,   0.,         321.22439592],
        [0.,         471.80617734, 242.9518823 ],
        [0.,           0.,           1.        ]
    ])
    
    print(f"\n相机内参:")
    print(f"  fx: {camera_matrix_1[0,0]:.2f} 像素")
    print(f"  fy: {camera_matrix_1[1,1]:.2f} 像素")
    print(f"  cx: {camera_matrix_1[0,2]:.2f} 像素")
    print(f"  cy: {camera_matrix_1[1,2]:.2f} 像素")
    
    print_matrix_info("方法1结果", T1, "米")
    
    # 方法2: 自动标定系统结果
    print("\n\n" + "="*80)
    print("方法2：自动标定系统（可能是OpenCV Daniilidis1999方法）")
    print("="*80)
    
    T2 = np.array([
        [ 0.260916, -0.962736,  0.071141,  69.010578],
        [ 0.965242,  0.261335, -0.003525, -45.927555],
        [-0.015198,  0.069588,  0.997460, 358.647756],
        [ 0.000000,  0.000000,  0.000000,   1.000000]
    ])
    
    print("\n注意：从数值大小判断，此结果单位为毫米")
    
    print_matrix_info("方法2结果", T2, "毫米")
    
    # 转换方法2为米单位以便对比
    T2_meters = T2.copy()
    T2_meters[:3, 3] /= 1000.0
    
    print("\n\n" + "="*80)
    print("差异分析（统一单位为米）")
    print("="*80)
    
    R1 = T1[:3, :3]
    t1 = T1[:3, 3]
    R2 = T2_meters[:3, :3]
    t2 = T2_meters[:3, 3]
    
    # 旋转差异
    rotation_diff = rotation_matrix_difference(R1, R2)
    print(f"\n1. 旋转差异: {rotation_diff:.2f}°")
    
    # 平移差异
    translation_diff = np.linalg.norm(t1 - t2)
    print(f"\n2. 平移差异: {translation_diff*1000:.2f} mm ({translation_diff:.6f} m)")
    
    print(f"\n3. 平移向量对比（米）:")
    print(f"   方法1: [{t1[0]:.6f}, {t1[1]:.6f}, {t1[2]:.6f}]")
    print(f"   方法2: [{t2[0]:.6f}, {t2[1]:.6f}, {t2[2]:.6f}]")
    print(f"   差值:  [{t1[0]-t2[0]:+.6f}, {t1[1]-t2[1]:+.6f}, {t1[2]-t2[2]:+.6f}]")
    
    print(f"\n4. 平移向量对比（毫米）:")
    print(f"   方法1: [{t1[0]*1000:.2f}, {t1[1]*1000:.2f}, {t1[2]*1000:.2f}]")
    print(f"   方法2: [{t2[0]*1000:.2f}, {t2[1]*1000:.2f}, {t2[2]*1000:.2f}]")
    print(f"   差值:  [{(t1[0]-t2[0])*1000:+.2f}, {(t1[1]-t2[1])*1000:+.2f}, {(t1[2]-t2[2])*1000:+.2f}]")
    
    # 平移范数对比
    t1_norm = np.linalg.norm(t1) * 1000
    t2_norm = np.linalg.norm(t2) * 1000
    print(f"\n5. 平移范数对比:")
    print(f"   方法1: {t1_norm:.2f} mm")
    print(f"   方法2: {t2_norm:.2f} mm")
    print(f"   差异:  {abs(t1_norm - t2_norm):.2f} mm")
    
    # 旋转矩阵对比
    print(f"\n6. 旋转矩阵逐元素差异:")
    R_diff_elements = R1 - R2
    print("   最大差异:", np.max(np.abs(R_diff_elements)))
    print("   平均差异:", np.mean(np.abs(R_diff_elements)))
    
    # 物理坐标系方向分析
    print(f"\n7. 坐标系方向分析:")
    print(f"   方法1 X轴方向: [{R1[0,0]:.3f}, {R1[1,0]:.3f}, {R1[2,0]:.3f}]")
    print(f"   方法2 X轴方向: [{R2[0,0]:.3f}, {R2[1,0]:.3f}, {R2[2,0]:.3f}]")
    print(f"   方法1 Y轴方向: [{R1[0,1]:.3f}, {R1[1,1]:.3f}, {R1[2,1]:.3f}]")
    print(f"   方法2 Y轴方向: [{R2[0,1]:.3f}, {R2[1,1]:.3f}, {R2[2,1]:.3f}]")
    print(f"   方法1 Z轴方向: [{R1[0,2]:.3f}, {R1[1,2]:.3f}, {R1[2,2]:.3f}]")
    print(f"   方法2 Z轴方向: [{R2[0,2]:.3f}, {R2[1,2]:.3f}, {R2[2,2]:.3f}]")
    
    # 结果一致性评估
    print("\n" + "="*80)
    print("结果一致性评估")
    print("="*80)
    
    if rotation_diff < 5.0 and translation_diff < 0.020:  # 20mm
        quality = "✅ 优秀 - 两种方法结果高度一致"
        detail = "旋转和平移差异都很小，两种方法都可靠"
    elif rotation_diff < 10.0 and translation_diff < 0.050:  # 50mm
        quality = "✓ 良好 - 结果基本一致"
        detail = "存在一定差异，但在可接受范围内"
    elif rotation_diff < 30.0 and translation_diff < 0.100:  # 100mm
        quality = "⚠️ 一般 - 存在明显差异"
        detail = "两种方法的结果有较大差异，建议检查数据质量"
    else:
        quality = "❌ 差 - 差异很大"
        detail = "两种方法的结果相差很大，可能存在严重问题"
    
    print(f"\n{quality}")
    print(f"{detail}")
    
    # 可能的差异原因
    print(f"\n可能的差异原因:")
    
    if rotation_diff > 20:
        print("  • 旋转差异很大（>20°）：")
        print("    - 可能使用了不同的坐标系定义")
        print("    - 可能是eye-in-hand vs eye-to-hand配置不同")
        print("    - 建议检查两种方法的坐标系约定")
    
    if translation_diff > 0.05:
        print("  • 平移差异较大（>50mm）：")
        print("    - 可能使用了不同的数据子集")
        print("    - 可能采用了不同的优化算法（TSAI vs Daniilidis1999）")
        print("    - 数据质量或采集姿态分布可能影响结果")
    
    if abs(t1[2] - t2[2]) > abs(t1[0] - t2[0]) and abs(t1[2] - t2[2]) > abs(t1[1] - t2[1]):
        print("  • Z方向差异最大：")
        z_diff_mm = abs(t1[2] - t2[2]) * 1000
        print(f"    - Z方向差异: {z_diff_mm:.2f}mm")
        print("    - 可能是深度测量或标定板Z坐标的系统性误差")
    
    # 推荐使用哪个结果
    print(f"\n" + "="*80)
    print("推荐建议")
    print("="*80)
    
    print(f"\n基于当前对比分析：")
    print(f"  • 两个结果的旋转差异: {rotation_diff:.1f}°")
    print(f"  • 两个结果的平移差异: {translation_diff*1000:.1f}mm")
    
    print(f"\n建议：")
    print(f"  1. 两个结果都应该在实际应用中测试")
    print(f"  2. 使用标定板或已知位置的物体进行精度验证")
    print(f"  3. 选择实际误差更小的方法")
    
    if rotation_diff > 45:
        print(f"  4. ⚠️ 旋转差异很大（{rotation_diff:.1f}°），建议检查：")
        print(f"     - 坐标系定义是否一致")
        print(f"     - 是否使用了相同的标定配置（eye-in-hand）")
        print(f"     - 数据采集是否正确")
    
    print("\n" + "="*80)

if __name__ == '__main__':
    main()
