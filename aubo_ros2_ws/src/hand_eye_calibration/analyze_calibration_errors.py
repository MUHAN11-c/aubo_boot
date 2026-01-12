#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析手眼标定误差的原因
"""

import json
import numpy as np
import os
import glob

def analyze_error_distribution(error_file):
    """分析误差分布"""
    print("="*80)
    print("误差分布分析")
    print("="*80)
    
    try:
        with open(error_file, 'r') as f:
            data = json.load(f)
        
        # 提取误差数据（从data字段中）
        data_dict = data.get('data', {})
        trans_errors = data_dict.get('translation_errors_mm', [])
        rot_errors_deg = data_dict.get('rotation_errors_deg', [])
        stats = data_dict.get('error_statistics', {})
        
        # 如果没有直接的deg数据，尝试从rad转换
        if not rot_errors_deg:
            rot_errors_rad = data_dict.get('rotation_errors_rad', [])
            rot_errors_deg = [r * 180.0 / np.pi for r in rot_errors_rad]
        
        if not trans_errors or not rot_errors_deg:
            print("❌ 错误：文件中没有误差数据")
            return
        
        print(f"\n【总体统计】")
        print(f"  姿态对数量: {len(trans_errors)}")
        print(f"  平移误差:")
        print(f"    RMS: {np.sqrt(np.mean([e**2 for e in trans_errors])):.2f} mm")
        print(f"    最大: {max(trans_errors):.2f} mm")
        print(f"    最小: {min(trans_errors):.2f} mm")
        print(f"    平均: {np.mean(trans_errors):.2f} mm")
        print(f"    标准差: {np.std(trans_errors):.2f} mm")
        print(f"    中位数: {np.median(trans_errors):.2f} mm")
        
        print(f"\n  旋转误差:")
        print(f"    RMS: {np.sqrt(np.mean([e**2 for e in rot_errors_deg])):.2f}°")
        print(f"    最大: {max(rot_errors_deg):.2f}°")
        print(f"    最小: {min(rot_errors_deg):.2f}°")
        print(f"    平均: {np.mean(rot_errors_deg):.2f}°")
        print(f"    标准差: {np.std(rot_errors_deg):.2f}°")
        print(f"    中位数: {np.median(rot_errors_deg):.2f}°")
        
        # 误差分布分析
        print(f"\n【误差分布】")
        print(f"  平移误差分布:")
        bins = [0, 100, 200, 300, 400, float('inf')]
        bin_labels = ['0-100mm', '100-200mm', '200-300mm', '300-400mm', '>400mm']
        for i in range(len(bins)-1):
            count = sum(1 for e in trans_errors if bins[i] <= e < bins[i+1])
            pct = count / len(trans_errors) * 100
            print(f"    {bin_labels[i]}: {count}个 ({pct:.1f}%)")
        
        print(f"\n  旋转误差分布:")
        bins = [0, 30, 60, 90, 120, float('inf')]
        bin_labels = ['0-30°', '30-60°', '60-90°', '90-120°', '>120°']
        for i in range(len(bins)-1):
            count = sum(1 for e in rot_errors_deg if bins[i] <= e < bins[i+1])
            pct = count / len(trans_errors) * 100
            print(f"    {bin_labels[i]}: {count}个 ({pct:.1f}%)")
        
        # 找出误差最大的姿态对
        error_pairs = [(i+1, t, r) for i, (t, r) in enumerate(zip(trans_errors, rot_errors_deg))]
        error_pairs.sort(key=lambda x: x[1], reverse=True)
        
        print(f"\n【误差最大的5个姿态对（按平移误差）】")
        for idx, (pair_idx, t_err, r_err) in enumerate(error_pairs[:5]):
            print(f"  姿态对 #{pair_idx}: 平移={t_err:.2f}mm, 旋转={r_err:.2f}°")
        
        print(f"\n【误差最小的5个姿态对（按平移误差）】")
        for idx, (pair_idx, t_err, r_err) in enumerate(error_pairs[-5:]):
            print(f"  姿态对 #{pair_idx}: 平移={t_err:.2f}mm, 旋转={r_err:.2f}°")
        
        # 分析误差的相关性
        correlation = np.corrcoef(trans_errors, rot_errors_deg)[0, 1]
        print(f"\n【误差相关性】")
        print(f"  平移误差与旋转误差的相关系数: {correlation:.3f}")
        if abs(correlation) > 0.7:
            print(f"  ⚠️ 强相关：说明误差可能来自同一源（如位姿测量误差）")
        elif abs(correlation) > 0.4:
            print(f"  ⚠️ 中等相关：误差可能部分相关")
        else:
            print(f"  ✓ 弱相关：误差可能来自不同源")
        
        return error_pairs
        
    except Exception as e:
        print(f"❌ 错误：{e}")
        import traceback
        traceback.print_exc()
        return None

def analyze_possible_causes():
    """分析可能的误差原因"""
    print("\n" + "="*80)
    print("误差原因分析")
    print("="*80)
    
    print("\n【根据误差特征分析】")
    print("\n1. 误差大小分析：")
    print("   - 平移RMS: 227.496mm (非常大)")
    print("   - 旋转RMS: 66.319° (非常大)")
    print("   - 最大平移: 405.629mm (超过40cm)")
    print("   - 最大旋转: 133.119° (超过130度)")
    print("   ⚠️ 这些误差远超正常范围（正常应该 < 10mm, < 1°）")
    
    print("\n2. 误差分布分析：")
    print("   - 误差范围很大（77-405mm），说明某些姿态对质量很差")
    print("   - 最小误差77mm仍然很大，说明整体数据质量有问题")
    print("   - 误差标准差96mm，说明误差分散性大")
    
    print("\n3. 可能的原因：")
    print("\n   A. 数据质量问题 ⭐ 最可能")
    print("      - 图像角点检测不准确")
    print("      - 机器人位姿测量误差")
    print("      - 深度测量误差（Z值一致性55mm，说明有一定误差）")
    print("      - 标定板识别错误或模糊")
    
    print("\n   B. 姿态分布问题")
    print("      - 姿态变化不够大（运动幅度小）")
    print("      - 姿态分布不均匀")
    print("      - 某些姿态对之间运动太小，导致数值不稳定")
    
    print("\n   C. 标定板位置问题")
    print("      - 标定板固定不牢固（有移动）")
    print("      - 标定板与相机距离不合适（太近或太远）")
    print("      - 标定板光照条件不一致")
    
    print("\n   D. 相机/机器人同步问题")
    print("      - 相机图像与机器人位姿不同步")
    print("      - 时间戳不匹配")
    print("      - 数据采集时的运动导致模糊")
    
    print("\n   E. 算法参数问题（不太可能，因为模拟数据误差0.000mm）")
    print("      - 算法本身正确（已验证）")
    print("      - 单位转换正确（已验证）")
    print("      - 坐标系变换正确（已验证）")

def provide_recommendations():
    """提供改进建议"""
    print("\n" + "="*80)
    print("改进建议")
    print("="*80)
    
    print("\n【数据采集改进】")
    print("1. 重新采集数据，确保：")
    print("   - 标定板清晰可见，角点检测准确")
    print("   - 机器人静止时采集（避免运动模糊）")
    print("   - 光照条件一致")
    print("   - 标定板固定牢固（不能移动）")
    print("   - 增加姿态多样性（不同位置、角度）")
    print("   - 增大姿态变化幅度（相邻姿态差异要大）")
    
    print("\n2. 数据质量检查：")
    print("   - 检查角点检测的重投影误差（应该 < 0.5像素）")
    print("   - 检查Z值一致性（应该 < 50mm）")
    print("   - 检查机器人位姿测量精度")
    print("   - 检查图像质量（清晰度、对比度）")
    
    print("\n3. 数据过滤：")
    print("   - 过滤掉误差过大的姿态对")
    print("   - 过滤掉运动幅度过小的姿态对")
    print("   - 使用质量较好的姿态子集进行标定")
    
    print("\n【验证方法】")
    print("1. 使用模拟数据验证算法正确性（已完成，误差0.000mm）")
    print("2. 使用已知位置的目标验证标定结果")
    print("3. 对比不同算法的结果（TSAI vs Daniilidis1999）")
    print("4. 检查标定结果的物理合理性（平移向量大小、旋转矩阵正交性）")

def main():
    """主函数"""
    print("\n" + "="*80)
    print("手眼标定误差原因分析")
    print("="*80)
    
    # 查找最新的误差计算文件
    config_dir = '/home/mu/IVG/aubo_ros2_ws/install/hand_eye_calibration/share/hand_eye_calibration/config'
    error_files = glob.glob(os.path.join(config_dir, 'opencv_calib_05_error_calculation_*.json'))
    
    if not error_files:
        print(f"\n❌ 错误：找不到误差计算文件")
        print(f"   请检查目录: {config_dir}")
        analyze_possible_causes()
        provide_recommendations()
        return
    
    # 使用最新的文件
    latest_file = max(error_files, key=os.path.getmtime)
    print(f"\n使用文件: {os.path.basename(latest_file)}")
    
    # 分析误差
    error_pairs = analyze_error_distribution(latest_file)
    
    # 分析原因
    analyze_possible_causes()
    
    # 提供建议
    provide_recommendations()
    
    print("\n" + "="*80)
    print("分析完成")
    print("="*80)

if __name__ == '__main__':
    main()
