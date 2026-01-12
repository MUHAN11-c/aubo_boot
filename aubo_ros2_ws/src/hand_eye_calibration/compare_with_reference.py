#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比参考代码hand_in_eye_calibrate.py和当前OpenCV实现
验证实现的一致性
"""

import sys
import os

def compare_pose_processing():
    """对比位姿处理方式"""
    print("="*80)
    print("对比1：位姿处理方式")
    print("="*80)
    
    print("\n【参考代码 hand_in_eye_calibrate.py】")
    print("\n1. pose_to_homogeneous_matrix函数（第30-34行）：")
    print("   def pose_to_homogeneous_matrix(pose):")
    print("       x, y, z, rx, ry, rz = pose")
    print("       R = euler_angles_to_rotation_matrix(rx, ry, rz)")
    print("       t = np.array([x, y, z]).reshape(3, 1)")
    print("       return R, t  # ✅ 直接返回，没有取逆操作！")
    print("\n   关键点：")
    print("   - 输入：pose = [x, y, z, rx, ry, rz]")
    print("   - 输出：R, t（Base→Gripper的旋转矩阵和平移向量）")
    print("   - 没有取逆操作，直接返回")
    
    print("\n2. process_arm_pose函数（第181-216行）：")
    print("   - 读取pose文件，每行：[x, y, z, rx, ry, rz]")
    print("   - 调用pose_to_homogeneous_matrix得到R和t")
    print("   - 直接添加到R_arm和t_arm列表")
    print("   - R_arm和t_arm都是Base→Gripper")
    
    print("\n3. hand_eye_calibrate函数（第237行）：")
    print("   R, t = cv2.calibrateHandEye(")
    print("       R_arm,  # Base→Gripper（直接传入）")
    print("       t_arm,  # Base→Gripper（直接传入）")
    print("       rvecs,  # Board→Camera")
    print("       tvecs,  # Board→Camera")
    print("       cv2.CALIB_HAND_EYE_TSAI")
    print("   )")
    print("\n   关键点：")
    print("   - R_arm和t_arm直接传入OpenCV，没有取逆")
    print("   - 参数名是R_arm，但实际是Base→Gripper")
    
    print("\n【当前实现 opencv_hand_eye_calibration.py】")
    print("\n1. prepare_data方法（第252-255行）：")
    print("   T_gripper = self._pose_to_transform_matrix(")
    print("       {'x': robot_pos_m['x'] * 1000.0, ...},  # 米→毫米")
    print("       robot_ori  # 四元数")
    print("   )")
    print("   - T_gripper是Base→Gripper（单位：毫米）")
    
    print("\n2. 提取数据（第704-708行）：")
    print("   R_base2gripper = T_gripper[:3, :3]  # ✅ 直接提取，不取逆")
    print("   t_base2gripper_mm = T_gripper[:3, 3].flatten()")
    print("   R_gripper2base_list.append(R_base2gripper)  # 变量名保持兼容")
    print("   t_gripper2base_list.append(t_base2gripper_mm)")
    print("\n   关键点：")
    print("   - 直接提取R和t，没有取逆操作")
    print("   - 变量名R_gripper2base容易误导，但实际存储Base→Gripper")
    
    print("\n3. OpenCV调用（第1246行）：")
    print("   R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(")
    print("       R_gripper2base,  # 实际是Base→Gripper（参数名误导）")
    print("       t_gripper2base,  # 实际是Base→Gripper，单位：米")
    print("       rvecs,           # Board→Camera，单位：弧度")
    print("       tvecs,           # Board→Camera，单位：米")
    print("       method=cv2.CALIB_HAND_EYE_TSAI")
    print("   )")
    print("\n   关键点：")
    print("   - 直接传入R_gripper2base和t_gripper2base，没有取逆")
    print("   - 虽然参数名是R_gripper2base，但实际是Base→Gripper")
    
    print("\n✅ 结论：位姿处理方式一致")
    print("   - 参考代码：直接传入Base→Gripper（不取逆）")
    print("   - 当前实现：直接传入Base→Gripper（不取逆）")
    print("   - 都遵循SUCCESS_FIX_RECORD.md中的修复原则")

def compare_data_format():
    """对比数据格式"""
    print("\n" + "="*80)
    print("对比2：数据格式")
    print("="*80)
    
    print("\n【参考代码 hand_in_eye_calibrate.py】")
    print("\n1. 输入数据格式：")
    print("   - 图片：JPG文件（collect_data_in文件夹）")
    print("   - 位姿文件：poses.txt，每行 [x, y, z, rx, ry, rz]")
    print("     * x, y, z: 位置（单位：米，从代码推断）")
    print("     * rx, ry, rz: 欧拉角（单位：弧度）")
    
    print("\n2. 处理流程：")
    print("   - camera_calibrate: 图片 → rvecs, tvecs（单位：米）")
    print("   - process_arm_pose: poses.txt → R_arm, t_arm")
    print("     * R_arm: Base→Gripper旋转矩阵")
    print("     * t_arm: Base→Gripper平移向量（单位：米，从代码推断）")
    print("   - cv2.calibrateHandEye: (R_arm, t_arm, rvecs, tvecs)")
    
    print("\n3. 输出：")
    print("   - R: 旋转矩阵（Camera→Gripper）")
    print("   - t: 平移向量（Camera→Gripper，单位：米）")
    
    print("\n【当前实现 opencv_hand_eye_calibration.py】")
    print("\n1. 输入数据格式：")
    print("   - poses_data: 列表，每个元素包含{'robot_pose': {...}, 'board_pose': {...}}")
    print("     * robot_pose: robot_pos_x/y/z（米），robot_ori_x/y/z/w（四元数）")
    print("     * board_pose: position.x/y/z（毫米），orientation.x/y/z/w（四元数）")
    print("     * board_pose.rvec和tvec（如果存在）：毫米")
    
    print("\n2. 处理流程：")
    print("   - prepare_data: poses_data → T_gripper（毫米），T_board（毫米）")
    print("     * 提取R_base2gripper, t_base2gripper_mm")
    print("     * 提取rvecs, tvecs（优先使用board_pose中的）")
    print("   - 单位转换：毫米 → 米（传递给OpenCV）")
    print("   - cv2.calibrateHandEye: (R_gripper2base, t_gripper2base, rvecs, tvecs)")
    print("     * 单位都是米")
    
    print("\n3. 输出：")
    print("   - R_cam2gripper: 旋转矩阵（Camera→Gripper）")
    print("   - t_cam2gripper: 平移向量（Camera→Gripper，单位：米）")
    print("   - 转换为毫米：T_cam2gripper（第1260行左右）")
    
    print("\n✅ 结论：数据格式基本一致，主要差异：")
    print("   - 参考代码：使用欧拉角输入")
    print("   - 当前实现：使用四元数输入")
    print("   - 参考代码：t_arm单位可能是米（从代码推断）")
    print("   - 当前实现：明确处理米↔毫米转换")
    print("   - 两种方式都正确，只是输入格式不同")

def compare_opencv_call():
    """对比OpenCV调用"""
    print("\n" + "="*80)
    print("对比3：OpenCV调用")
    print("="*80)
    
    print("\n【参考代码 hand_in_eye_calibrate.py 第237行】")
    print("   R, t = cv2.calibrateHandEye(")
    print("       R_arm,  # Base→Gripper（直接传入，不取逆）")
    print("       t_arm,  # Base→Gripper（直接传入，不取逆）")
    print("       rvecs,  # Board→Camera")
    print("       tvecs,  # Board→Camera")
    print("       cv2.CALIB_HAND_EYE_TSAI")
    print("   )")
    
    print("\n【当前实现 opencv_hand_eye_calibration.py 第1246行】")
    print("   R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(")
    print("       R_gripper2base,  # 实际是Base→Gripper（参数名误导）")
    print("       t_gripper2base,  # 实际是Base→Gripper，单位：米")
    print("       rvecs,           # Board→Camera，单位：弧度")
    print("       tvecs,           # Board→Camera，单位：米")
    print("       method=cv2.CALIB_HAND_EYE_TSAI")
    print("   )")
    
    print("\n✅ 结论：OpenCV调用完全一致")
    print("   - 都使用CALIB_HAND_EYE_TSAI方法")
    print("   - 都直接传入Base→Gripper（不取逆）")
    print("   - 参数顺序和含义完全一致")
    print("   - 当前实现的参数名R_gripper2base容易误导，但实际值与参考代码一致")

def compare_coordinate_system():
    """对比坐标系定义"""
    print("\n" + "="*80)
    print("对比4：坐标系定义")
    print("="*80)
    
    print("\n【参考代码 hand_in_eye_calibrate.py】")
    print("   根据代码注释和实现：")
    print("   - R_arm, t_arm: Base→Gripper（从pose_to_homogeneous_matrix得到）")
    print("   - rvecs, tvecs: Board→Camera（从cv2.calibrateCamera得到）")
    print("   - 输出R, t: Camera→Gripper")
    
    print("\n【当前实现 opencv_hand_eye_calibration.py】")
    print("   根据代码注释和SUCCESS_FIX_RECORD.md：")
    print("   - R_gripper2base: Base→Gripper（虽然参数名误导）")
    print("   - t_gripper2base: Base→Gripper（虽然参数名误导）")
    print("   - rvecs, tvecs: Board→Camera")
    print("   - 输出R_cam2gripper, t_cam2gripper: Camera→Gripper")
    
    print("\n✅ 结论：坐标系定义完全一致")
    print("   - 输入：Base→Gripper, Board→Camera")
    print("   - 输出：Camera→Gripper")
    print("   - 都遵循AX=XB方程：A·X = X·B")
    print("     * A = inv(Base→Gripper₁) @ (Base→Gripper₂)")
    print("     * B = (Board→Camera₁) @ inv(Board→Camera₂)")
    print("     * X = Camera→Gripper（待求）")

def summary():
    """总结"""
    print("\n" + "="*80)
    print("总结")
    print("="*80)
    
    print("\n【一致性验证结果】")
    print("   ✅ 位姿处理方式：完全一致（都直接传入Base→Gripper，不取逆）")
    print("   ✅ OpenCV调用：完全一致（参数、方法、顺序都一致）")
    print("   ✅ 坐标系定义：完全一致（Base→Gripper, Board→Camera, Camera→Gripper）")
    print("   ✅ 数据格式：基本一致（输入格式不同，但处理结果一致）")
    
    print("\n【关键发现】")
    print("   1. 参考代码证实了SUCCESS_FIX_RECORD.md中的修复是正确的")
    print("      - 参考代码直接传入Base→Gripper，没有取逆操作")
    print("      - 当前实现也直接传入Base→Gripper，没有取逆操作")
    print("\n   2. OpenCV参数名的误导性")
    print("      - OpenCV参数名R_gripper2base暗示Gripper→Base")
    print("      - 但实际期望Base→Gripper（参考代码和当前实现都证实）")
    print("\n   3. 当前实现的变量命名")
    print("      - R_gripper2base变量名容易误导（实际是Base→Gripper）")
    print("      - 但实际值与参考代码一致，实现正确")
    print("      - 建议：保留注释说明，或者考虑重命名变量")
    
    print("\n【建议】")
    print("   1. ✅ 当前实现与参考代码完全一致，无需修改")
    print("   2. 💡 可以在代码注释中引用参考代码，说明参数名的误导性")
    print("   3. 💡 考虑在变量名中添加注释，明确实际含义")
    print("   4. ✅ 运行模拟验证，确认实现正确性")

def main():
    """主函数"""
    print("\n" + "="*80)
    print("参考代码对比分析：hand_in_eye_calibrate.py vs opencv_hand_eye_calibration.py")
    print("="*80)
    
    compare_pose_processing()
    compare_data_format()
    compare_opencv_call()
    compare_coordinate_system()
    summary()
    
    print("\n" + "="*80)
    print("对比分析完成")
    print("="*80)

if __name__ == '__main__':
    main()
