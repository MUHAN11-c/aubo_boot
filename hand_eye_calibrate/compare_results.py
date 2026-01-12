#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比两次手眼标定结果
"""

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

np.set_printoptions(precision=6, suppress=True)

def rotation_matrix_difference(R1, R2):
    """计算两个旋转矩阵之间的角度差异"""
    R_diff = R1 @ R2.T
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
    return np.degrees(angle)

def print_matrix_info(name, T):
    """打印变换矩阵信息"""
    print(f"\n【{name}】")
    print("完整变换矩阵:")
    print(T)
    
    R = T[:3, :3]
    t = T[:3, 3]
    
    print("\n旋转矩阵:")
    print(R)
    print(f"\n平移向量: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    print(f"平移范数: {np.linalg.norm(t):.6f}")
    print(f"旋转矩阵行列式: {np.linalg.det(R):.6f}")
    
    # 欧拉角
    try:
        r = R_scipy.from_matrix(R)
    except AttributeError:
        r = R_scipy.from_dcm(R)
    euler_xyz = r.as_euler('xyz', degrees=True)
    euler_zyx = r.as_euler('zyx', degrees=True)
    
    print(f"欧拉角 (XYZ): [{euler_xyz[0]:.2f}°, {euler_xyz[1]:.2f}°, {euler_xyz[2]:.2f}°]")
    print(f"欧拉角 (ZYX): [{euler_zyx[0]:.2f}°, {euler_zyx[1]:.2f}°, {euler_zyx[2]:.2f}°]")

def main():
    print("="*80)
    print("手眼标定结果对比分析")
    print("="*80)
    
    # 1. run_calibration.py 结果（最新，19张图片全部有效）
    print("\n【方法1：run_calibration.py - 基于OpenCV TSAI算法】")
    print("数据源: /home/mu/IVG/hand_eye_calibrate/collect_data")
    print("有效图片: 19张")
    
    camera_matrix_1 = np.array([
        [467.30004274,   0.,         319.89063476],
        [0.,         467.60159736, 244.4975564 ],
        [0.,           0.,           1.        ]
    ])
    
    T1 = np.array([
        [ 0.2832403,   0.07613665,  0.95602204, -0.25340665],
        [ 0.71792068,  0.64412441, -0.26399553, -0.06809451],
        [-0.63589687,  0.76112216,  0.12778192,  0.03585163],
        [ 0.,          0.,          0.,          1.        ]
    ])
    
    print(f"\n相机内参:")
    print(f"  fx: {camera_matrix_1[0,0]:.2f}")
    print(f"  fy: {camera_matrix_1[1,1]:.2f}")
    print(f"  cx: {camera_matrix_1[0,2]:.2f}")
    print(f"  cy: {camera_matrix_1[1,2]:.2f}")
    
    print_matrix_info("方法1结果（单位：米）", T1)
    
    # 2. 自动手眼标定OpenCV模式结果
    print("\n\n" + "="*80)
    print("【方法2：自动手眼标定OpenCV模式】")
    
    camera_matrix_2 = np.array([
        [466.17,   0.,     326.07],
        [0.,       465.56, 244.79],
        [0.,       0.,     1.    ]
    ])
    
    T2 = np.array([
        [-0.142202,  0.989194,  0.035691, -22.981416],
        [-0.985119, -0.137915, -0.102562,  88.101624],
        [-0.096532, -0.049744,  0.994086, 246.939834],
        [ 0.000000,  0.000000,  0.000000,   1.000000]
    ])
    
    print(f"\n相机内参:")
    print(f"  fx: {camera_matrix_2[0,0]:.2f}")
    print(f"  fy: {camera_matrix_2[1,1]:.2f}")
    print(f"  cx: {camera_matrix_2[0,2]:.2f}")
    print(f"  cy: {camera_matrix_2[1,2]:.2f}")
    
    print_matrix_info("方法2结果（单位：毫米）", T2)
    
    # 转换T2为米单位以便对比
    T2_meters = T2.copy()
    T2_meters[:3, 3] /= 1000.0
    
    print("\n\n" + "="*80)
    print("【差异分析】")
    print("="*80)
    
    # 相机内参对比
    print("\n1. 相机内参对比:")
    fx_diff = abs(camera_matrix_1[0,0] - camera_matrix_2[0,0])
    fy_diff = abs(camera_matrix_1[1,1] - camera_matrix_2[1,1])
    cx_diff = abs(camera_matrix_1[0,2] - camera_matrix_2[0,2])
    cy_diff = abs(camera_matrix_1[1,2] - camera_matrix_2[1,2])
    
    print(f"   fx差异: {fx_diff:.2f} 像素")
    print(f"   fy差异: {fy_diff:.2f} 像素")
    print(f"   cx差异: {cx_diff:.2f} 像素")
    print(f"   cy差异: {cy_diff:.2f} 像素")
    
    if fx_diff < 5 and fy_diff < 5 and cx_diff < 10 and cy_diff < 10:
        print("   ✅ 相机内参一致性：优秀")
    else:
        print("   ⚠️ 相机内参存在差异")
    
    # 手眼标定结果对比
    print("\n2. 手眼标定变换矩阵对比（统一为米单位）:")
    
    R1 = T1[:3, :3]
    t1 = T1[:3, 3]
    R2 = T2_meters[:3, :3]
    t2 = T2_meters[:3, 3]
    
    # 旋转差异
    rotation_diff = rotation_matrix_difference(R1, R2)
    print(f"   旋转差异: {rotation_diff:.2f}°")
    
    # 平移差异
    translation_diff = np.linalg.norm(t1 - t2)
    print(f"   平移差异: {translation_diff*1000:.2f} mm")
    
    print(f"\n   方法1平移向量: [{t1[0]*1000:.2f}, {t1[1]*1000:.2f}, {t1[2]*1000:.2f}] mm")
    print(f"   方法2平移向量: [{t2[0]*1000:.2f}, {t2[1]*1000:.2f}, {t2[2]*1000:.2f}] mm")
    
    # 质量评估
    print("\n3. 结果一致性评估:")
    if rotation_diff < 5.0 and translation_diff < 0.020:  # 20mm
        quality = "✅ 优秀 - 两种方法结果高度一致"
    elif rotation_diff < 10.0 and translation_diff < 0.050:  # 50mm
        quality = "✓ 良好 - 结果基本一致，可接受"
    elif rotation_diff < 30.0 and translation_diff < 0.100:  # 100mm
        quality = "⚠️ 一般 - 存在明显差异"
    else:
        quality = "❌ 差 - 差异很大，建议检查数据和方法"
    
    print(f"   {quality}")
    
    # 可能的差异原因分析
    print("\n4. 差异原因分析:")
    
    reasons = []
    
    if abs(rotation_diff) > 20:
        reasons.append("   • 旋转差异较大：可能是坐标系定义不同或标定板检测精度问题")
    
    if abs(translation_diff) > 0.05:
        reasons.append("   • 平移差异较大：可能是单位转换、数据过滤策略或优化算法不同")
    
    # 检查有效图片数量（从日志看，方法1用19张，方法2可能用18张）
    reasons.append("   • 方法1使用19张有效图片，方法2可能使用18张（过滤了1张）")
    reasons.append("   • 两种方法可能使用了不同的优化算法或数据过滤策略")
    reasons.append("   • 标定板角点检测的亚像素精度可能不同")
    
    if reasons:
        for reason in reasons:
            print(reason)
    
    print("\n" + "="*80)
    print("建议:")
    print("="*80)
    print("1. 如果旋转差异 < 10°且平移差异 < 50mm，两个结果都可以使用")
    print("2. 建议在实际应用中测试两个标定结果，选择误差更小的")
    print("3. 可以尝试采集更多高质量数据重新标定以获得更准确的结果")
    print("4. 检查两种方法是否使用了相同的坐标系定义（手在眼上/手在眼外）")

if __name__ == '__main__':
    main()
