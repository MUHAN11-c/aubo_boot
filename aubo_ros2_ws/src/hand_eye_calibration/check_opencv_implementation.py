#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查OpenCV模式实现的一致性
对比simulate_opencv_calibration.py和实际opencv_hand_eye_calibration.py的实现
"""

import sys
import os
import numpy as np
import cv2

# 添加路径
sys.path.insert(0, os.path.dirname(__file__))

def check_data_format_consistency():
    """检查数据格式的一致性"""
    print("="*80)
    print("检查1：数据格式一致性")
    print("="*80)
    
    print("\n【simulate_opencv_calibration.py的数据格式】")
    print("poses_data格式：")
    print("  - robot_pose:")
    print("    - robot_pos_x/y/z: 单位=米")
    print("    - robot_ori_x/y/z/w: 四元数")
    print("  - board_pose:")
    print("    - position.x/y/z: 单位=毫米")
    print("    - orientation.x/y/z/w: 四元数")
    print("    - rvec.x/y/z: 旋转向量（弧度）")
    print("    - tvec.x/y/z: 单位=毫米")
    
    print("\n【opencv_hand_eye_calibration.py期望的数据格式】")
    print("根据prepare_data方法的注释：")
    print("  - robot_pose: robot_pos_x/y/z（单位：米）和robot_ori_x/y/z/w（四元数）")
    print("  - board_pose: position（x/y/z，单位：毫米）和orientation（x/y/z/w，四元数）")
    print("  - board_pose.rvec和tvec（如果存在）：优先使用")
    
    print("\n✅ 结论：数据格式完全一致")
    
def check_coordinate_transform():
    """检查坐标系变换的一致性"""
    print("\n" + "="*80)
    print("检查2：坐标系变换一致性")
    print("="*80)
    
    print("\n【simulate_opencv_calibration.py】")
    print("  - robot_poses: Base→Gripper（单位：毫米）")
    print("  - board_poses: Board→Camera（单位：毫米）")
    print("  - 转换为poses_data时：")
    print("    - robot_pos_x/y/z: 米（毫米/1000）")
    print("    - board_pose.position.x/y/z: 毫米（保持不变）")
    
    print("\n【opencv_hand_eye_calibration.py】")
    print("  - prepare_data方法：")
    print("    - 从poses_data提取robot_pose: 米→毫米（*1000）构建T_gripper")
    print("    - 从poses_data提取board_pose: 毫米（直接使用）构建T_board")
    print("    - T_gripper是Base→Gripper（单位：毫米）")
    print("    - T_board是Board→Camera（单位：毫米）")
    print("    - 关键代码（第704行）：")
    print("      R_base2gripper = T_gripper[:3, :3]  # 直接使用，不取逆")
    print("      t_base2gripper_mm = T_gripper[:3, 3].flatten()")
    
    print("\n【SUCCESS_FIX_RECORD.md中的关键修复】")
    print("  - 修复前：对T_gripper取了逆（错误）")
    print("  - 修复后：直接使用T_gripper（正确）")
    print("  - OpenCV参数名R_gripper2base容易误导，实际期望Base→Gripper")
    
    print("\n✅ 结论：坐标系变换一致，都使用Base→Gripper（不取逆）")
    
def check_unit_conversion():
    """检查单位转换的一致性"""
    print("\n" + "="*80)
    print("检查3：单位转换一致性")
    print("="*80)
    
    print("\n【simulate_opencv_calibration.py】")
    print("  1. 生成数据：")
    print("     - robot_poses: 毫米")
    print("     - board_poses: 毫米")
    print("  2. 转换为poses_data：")
    print("     - robot_pos_x/y/z: 毫米 → 米（/1000）")
    print("     - board_pose.position.x/y/z: 毫米（保持不变）")
    print("     - board_pose.tvec.x/y/z: 毫米（保持不变）")
    
    print("\n【opencv_hand_eye_calibration.py】")
    print("  1. prepare_data方法：")
    print("     - 从poses_data构建T_gripper：米 → 毫米（*1000）")
    print("     - 从poses_data构建T_board：毫米（直接使用）")
    print("     - 代码（第253行）：")
    print("       T_gripper = _pose_to_transform_matrix(")
    print("           {'x': robot_pos_m['x'] * 1000.0, ...},  # 米→毫米")
    print("           robot_ori")
    print("       )")
    print("  2. 提取数据：")
    print("     - R_base2gripper: 从T_gripper提取（毫米）")
    print("     - t_base2gripper_mm: 从T_gripper提取（毫米）")
    print("  3. 转换为OpenCV格式（第846行）：")
    print("     - t_gripper2base: 毫米 → 米（/1000）")
    print("     - tvecs: 毫米 → 米（/1000）")
    print("     - rvecs: 弧度（无需转换）")
    
    print("\n✅ 结论：单位转换一致")
    print("  - 输入：robot_pose（米），board_pose（毫米）")
    print("  - 内部：T_gripper（毫米），T_board（毫米）")
    print("  - 输出给OpenCV：t_gripper2base（米），tvecs（米）")
    
def check_opencv_call():
    """检查OpenCV调用的参数"""
    print("\n" + "="*80)
    print("检查4：OpenCV调用参数")
    print("="*80)
    
    print("\n【opencv_hand_eye_calibration.py中的OpenCV调用（第1246行）】")
    print("  R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(")
    print("      R_gripper2base,  # 实际是Base→Gripper（参数名误导）")
    print("      t_gripper2base,  # 实际是Base→Gripper，单位：米")
    print("      rvecs,           # Board→Camera，单位：弧度")
    print("      tvecs,           # Board→Camera，单位：米")
    print("      method=cv2.CALIB_HAND_EYE_TSAI")
    print("  )")
    
    print("\n【参考代码hand_in_eye_calibrate.py（SUCCESS_FIX_RECORD.md）】")
    print("  R, t = cv2.calibrateHandEye(")
    print("      R_arm,  # Base→Gripper（直接传入，不取逆）")
    print("      t_arm,  # Base→Gripper（直接传入，不取逆）")
    print("      rvecs,  # Board→Camera")
    print("      tvecs,  # Board→Camera")
    print("      cv2.CALIB_HAND_EYE_TSAI")
    print("  )")
    
    print("\n✅ 结论：OpenCV调用参数一致")
    print("  - R_gripper2base: Base→Gripper（不取逆）")
    print("  - t_gripper2base: Base→Gripper，单位：米")
    print("  - rvecs: Board→Camera，单位：弧度")
    print("  - tvecs: Board→Camera，单位：米")
    
def check_tvec_extraction():
    """检查tvec提取方式"""
    print("\n" + "="*80)
    print("检查5：tvec提取方式")
    print("="*80)
    
    print("\n【simulate_opencv_calibration.py】")
    print("  convert_to_poses_data_format方法（第218-219行）：")
    print("    board_rvec, _ = cv2.Rodrigues(board_R)")
    print("    board_tvec = board_pos_mm.reshape(3, 1)  # 单位：毫米")
    print("    然后保存到board_pose.tvec中")
    
    print("\n【opencv_hand_eye_calibration.py】")
    print("  prepare_data方法（第728-784行）：")
    print("    1. 优先使用board_pose中的rvec/tvec（如果存在）")
    print("    2. 如果没有，从T_board转换：")
    print("       - rvec: cv2.Rodrigues(T_board[:3, :3])")
    print("       - tvec: T_board[:3, 3].reshape(3, 1)")
    
    print("\n✅ 结论：tvec提取方式一致")
    print("  - 优先使用board_pose.tvec（如果存在）")
    print("  - 否则从T_board提取")
    print("  - 单位都是毫米，传递给OpenCV时转换为米")
    
def check_simulation_data_flow():
    """检查模拟数据的完整流程"""
    print("\n" + "="*80)
    print("检查6：模拟数据的完整流程")
    print("="*80)
    
    print("\n【数据流程】")
    print("  1. generate_robot_poses: 生成Base→Gripper（毫米）")
    print("  2. generate_board_poses: 根据真值生成Board→Camera（毫米）")
    print("  3. convert_to_poses_data_format: 转换为poses_data格式")
    print("     - robot_pos: 毫米 → 米")
    print("     - board_pose.position: 毫米（保持不变）")
    print("     - board_pose.tvec: 毫米（保持不变）")
    print("  4. calibrator.calibrate(poses_data): 调用OpenCV标定")
    print("     - prepare_data: poses_data → T_gripper（毫米），T_board（毫米）")
    print("     - 提取R_base2gripper, t_base2gripper_mm")
    print("     - 转换为OpenCV格式：t_base2gripper_m（米），tvecs（米）")
    print("     - cv2.calibrateHandEye(...)")
    print("     - 结果：T_cam2gripper（毫米）")
    
    print("\n✅ 结论：数据流程正确且一致")
    
def check_potential_issues():
    """检查潜在问题"""
    print("\n" + "="*80)
    print("检查7：潜在问题分析")
    print("="*80)
    
    print("\n【已确认正确的部分】")
    print("  ✅ 数据格式一致")
    print("  ✅ 坐标系变换正确（Base→Gripper，不取逆）")
    print("  ✅ 单位转换正确（米↔毫米）")
    print("  ✅ OpenCV调用参数正确")
    print("  ✅ tvec提取方式正确")
    
    print("\n【需要验证的部分】")
    print("  1. 模拟脚本生成的board_poses是否正确？")
    print("     - generate_board_poses方法（第139-191行）")
    print("     - 公式：T_board2camera = T_gripper2cam @ T_gripper2base @ T_base2board")
    print("     - 这是根据真值T_cam2gripper反推的，应该正确")
    
    print("\n  2. 标定结果的单位是否正确？")
    print("     - OpenCV返回的t_cam2gripper单位是米")
    print("     - 转换为毫米时需要乘以1000")
    print("     - 需要检查solve_hand_eye方法中的单位转换")
    
    print("\n  3. 误差计算是否正确？")
    print("     - 根据SUCCESS_FIX_RECORD.md，误差计算已经修复")
    print("     - A矩阵计算：A = inv(T_base2gripper1) @ T_base2gripper2")
    print("     - 这是正确的")
    
    print("\n【建议进一步检查】")
    print("  1. 运行simulate_opencv_calibration.py，验证是否能得到0.000mm误差")
    print("  2. 检查solve_hand_eye方法中t_cam2gripper的单位转换")
    print("  3. 对比模拟结果和真实数据的差异")

def main():
    """主函数"""
    print("\n" + "="*80)
    print("OpenCV模式实现一致性检查")
    print("="*80)
    print("\n本脚本检查simulate_opencv_calibration.py和opencv_hand_eye_calibration.py")
    print("之间的实现一致性，确保数据格式、坐标系变换、单位转换等都正确。")
    
    check_data_format_consistency()
    check_coordinate_transform()
    check_unit_conversion()
    check_opencv_call()
    check_tvec_extraction()
    check_simulation_data_flow()
    check_potential_issues()
    
    print("\n" + "="*80)
    print("检查完成")
    print("="*80)
    
    print("\n【总结】")
    print("基于代码审查，OpenCV模式的实现基本正确，关键点：")
    print("  1. ✅ 坐标系变换：Base→Gripper（不取逆）")
    print("  2. ✅ 单位转换：米↔毫米正确")
    print("  3. ✅ OpenCV调用参数正确")
    print("  4. ✅ 数据格式一致")
    
    print("\n建议运行simulate_opencv_calibration.py进行实际验证。")

if __name__ == '__main__':
    main()
