#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手上手眼标定 - 使用新的数据格式（每个姿态一个子文件夹）
使用 /home/mu/IVG/hand_eye_calibrate/collect_data 目录下的数据

完全使用OpenCV模式的算法和误差计算方法，确保与opencv_hand_eye_calibration.py一致：
- 算法：OpenCV calibrateHandEye (TSAI方法)
- 误差计算：AX=XB约束验证，使用毫米单位（与opencv模式完全一致）
- 数据格式：robot_pose（米），board_pose.tvec（毫米→米转换）
"""

import os
import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=6, suppress=True)

def load_pose_data(data_dir):
    """
    加载所有姿态数据
    
    参数:
        data_dir: 数据目录路径
    
    返回:
        robot_poses: 机器人姿态列表（Base->Gripper变换矩阵）
        board_poses: 棋盘格姿态列表（Board->Camera变换矩阵，rvec和tvec）
    """
    robot_poses = []
    board_poses = []
    pose_indices = []
    
    # 获取所有pose文件夹
    pose_dirs = []
    for item in os.listdir(data_dir):
        item_path = os.path.join(data_dir, item)
        if os.path.isdir(item_path) and item.startswith('pose_'):
            try:
                pose_idx = int(item.replace('pose_', ''))
                pose_dirs.append((pose_idx, item_path))
            except ValueError:
                continue
    
    # 按索引排序
    pose_dirs.sort(key=lambda x: x[0])
    
    print(f"找到 {len(pose_dirs)} 个姿态文件夹")
    
    for pose_idx, pose_dir in pose_dirs:
        robot_pose_file = os.path.join(pose_dir, 'robot_pose.json')
        board_pose_file = os.path.join(pose_dir, 'board_pose.json')
        
        if not os.path.exists(robot_pose_file):
            print(f"警告: {pose_dir} 缺少 robot_pose.json，跳过")
            continue
        
        if not os.path.exists(board_pose_file):
            print(f"警告: {pose_dir} 缺少 board_pose.json，跳过")
            continue
        
        try:
            # 加载机器人姿态
            with open(robot_pose_file, 'r', encoding='utf-8') as f:
                robot_data = json.load(f)
            
            cartesian = robot_data['cartesian_position']
            pos = cartesian['position']
            ori = cartesian['orientation']
            
            # 构建Base->Gripper变换矩阵
            quat = [ori['w'], ori['x'], ori['y'], ori['z']]  # w, x, y, z
            rotation = R.from_quat(quat)
            # 兼容旧版本scipy
            try:
                R_base2gripper = rotation.as_matrix()
            except AttributeError:
                R_base2gripper = rotation.as_dcm()  # 旧版本使用as_dcm
            t_base2gripper = np.array([pos['x'], pos['y'], pos['z']])  # 单位：米
            
            T_base2gripper = np.eye(4)
            T_base2gripper[:3, :3] = R_base2gripper
            T_base2gripper[:3, 3] = t_base2gripper
            
            robot_poses.append(T_base2gripper)
            
            # 加载棋盘格姿态
            with open(board_pose_file, 'r', encoding='utf-8') as f:
                board_data = json.load(f)
            
            rvec = np.array([
                board_data['rvec']['x'],
                board_data['rvec']['y'],
                board_data['rvec']['z']
            ])
            tvec = np.array([
                board_data['tvec']['x'],
                board_data['tvec']['y'],
                board_data['tvec']['z']
            ])  # 单位：毫米（solvePnP返回）
            
            # 验证数据一致性：从rvec和tvec构建变换矩阵，与保存的transformation_matrix对比
            R_board2cam_from_rvec, _ = cv2.Rodrigues(rvec)
            T_board2cam_from_rvec = np.eye(4)
            T_board2cam_from_rvec[:3, :3] = R_board2cam_from_rvec
            T_board2cam_from_rvec[:3, 3] = tvec  # 毫米
            
            T_board2cam_saved = np.array(board_data['transformation_matrix'])
            
            # 验证一致性（允许1mm误差）
            if not np.allclose(T_board2cam_from_rvec, T_board2cam_saved, atol=1.0):
                print(f"警告: pose_{pose_idx} 的transformation_matrix与rvec/tvec不一致，使用rvec/tvec构建的矩阵")
            
            board_poses.append({
                'rvec': rvec.reshape(3, 1),
                'tvec': tvec.reshape(3, 1) / 1000.0,  # 转换为米（OpenCV需要）
                'transformation_matrix': T_board2cam_from_rvec  # Board->Camera，单位：毫米（用于验证）
            })
            
            pose_indices.append(pose_idx)
            
        except Exception as e:
            print(f"警告: 加载 {pose_dir} 失败: {e}，跳过")
            continue
    
    print(f"成功加载 {len(robot_poses)} 个姿态数据")
    return robot_poses, board_poses, pose_indices


def calibrate_hand_eye(robot_poses, board_poses):
    """
    使用OpenCV的calibrateHandEye进行手眼标定（Eye-in-Hand）
    
    参数:
        robot_poses: Base->Gripper变换矩阵列表（单位：米）
        board_poses: Board->Camera的rvec和tvec列表（单位：米）
    
    返回:
        R_cam2gripper: 相机到末端执行器的旋转矩阵
        t_cam2gripper: 相机到末端执行器的平移向量（单位：米）
    """
    # 准备OpenCV格式的数据
    # 注意: OpenCV参数名R_gripper2base容易误导，实际期望Base→Gripper（不要取逆！）
    # 参考: SUCCESS_FIX_RECORD.md 和 hand_in_eye_calibrate.py
    R_gripper2base = []  # 实际存储Base->Gripper（参数名误导，但变量名保持不变）
    t_gripper2base = []  # 实际存储Base->Gripper（参数名误导，但变量名保持不变）
    rvecs = []
    tvecs = []
    
    for T_base2gripper, board_pose in zip(robot_poses, board_poses):
        # 直接使用Base->Gripper（不要取逆！参考SUCCESS_FIX_RECORD.md）
        R_base2gripper = T_base2gripper[:3, :3]  # 直接提取，不取逆
        t_base2gripper = T_base2gripper[:3, 3]  # 直接提取，不取逆
        R_gripper2base.append(R_base2gripper)  # 变量名保持不变以兼容后续代码
        t_gripper2base.append(t_base2gripper)  # 变量名保持不变以兼容后续代码
        
        # Board->Camera的rvec和tvec
        rvecs.append(board_pose['rvec'])
        tvecs.append(board_pose['tvec'])
    
    print(f"使用 {len(R_gripper2base)} 个姿态进行标定")
    
    # 验证数据格式和单位
    if len(R_gripper2base) < 4:
        raise ValueError(f"数据不足，至少需要4个姿态，当前只有{len(R_gripper2base)}个")
    
    # 验证单位：t_base2gripper应该是米（通常范围0.1-1.5米）
    sample_t_norm = np.linalg.norm(t_gripper2base[0])
    if sample_t_norm > 10.0:  # 如果>10米，可能是单位错误
        print(f"警告: 机器人位姿平移向量范数 {sample_t_norm:.3f} 米，可能单位有误")
    
    # 验证单位：tvec应该是米（通常范围0.1-1.5米）
    sample_tvec_norm = np.linalg.norm(tvecs[0])
    if sample_tvec_norm > 10.0:  # 如果>10米，可能是单位错误
        print(f"警告: 标定板tvec范数 {sample_tvec_norm:.3f} 米，可能单位有误")
    
    # 使用OpenCV的calibrateHandEye（TSAI方法，与opencv_hand_eye_calibration.py一致）
    print("使用OpenCV calibrateHandEye (TSAI方法)进行标定...")
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,  # 单位：米
        rvecs, tvecs,                     # 单位：米
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    print(f"标定完成: t_cam2gripper范数 = {np.linalg.norm(t_cam2gripper):.6f} 米 ({np.linalg.norm(t_cam2gripper)*1000:.3f} 毫米)")
    
    return R_cam2gripper, t_cam2gripper


def calculate_errors(robot_poses, board_poses, R_cam2gripper, t_cam2gripper):
    """
    计算AX=XB约束的误差（完全使用OpenCV模式的误差计算方法）
    
    参数:
        robot_poses: Base->Gripper变换矩阵列表（单位：米）
        board_poses: Board->Camera的rvec和tvec列表（单位：米）
        R_cam2gripper: 相机到末端执行器的旋转矩阵
        t_cam2gripper: 相机到末端执行器的平移向量（单位：米，OpenCV返回）
    
    返回:
        errors: 每个姿态对的误差列表
        error_stats: 误差统计信息
    """
    # 构建T_cam2gripper（单位：毫米，与opencv_hand_eye_calibration.py一致）
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten() * 1000.0  # 米 → 毫米
    
    # 转换为毫米单位（与opencv_hand_eye_calibration.py的calculate_errors一致）
    T_gripper_list = []  # Base->Gripper，单位：毫米
    T_board_list = []    # Board->Camera，单位：毫米
    
    for T_base2gripper, board_pose in zip(robot_poses, board_poses):
        # 机器人位姿：米 → 毫米
        T_gripper_mm = T_base2gripper.copy()
        T_gripper_mm[:3, 3] = T_base2gripper[:3, 3] * 1000.0
        T_gripper_list.append(T_gripper_mm)
        
        # 标定板位姿：使用rvec和tvec构建（tvec已经是米，转换为毫米）
        rvec = board_pose['rvec']
        tvec_m = board_pose['tvec']  # 米
        R_board2cam, _ = cv2.Rodrigues(rvec)
        T_board2cam = np.eye(4)
        T_board2cam[:3, :3] = R_board2cam
        T_board2cam[:3, 3] = tvec_m.flatten() * 1000.0  # 米 → 毫米
        T_board_list.append(T_board2cam)
    
    errors = []
    
    # 对相邻姿态对计算误差（与opencv_hand_eye_calibration.py:1086-1321完全一致）
    for i in range(len(T_gripper_list) - 1):
        try:
            # 计算A矩阵：Gripper1->Gripper2（通过Base）
            # A = inv(Base→Gripper₁) @ Base→Gripper₂ = Gripper₁→Gripper₂
            T_base2gripper1 = T_gripper_list[i]  # Base→Gripper，单位：毫米
            T_base2gripper2 = T_gripper_list[i + 1]  # Base→Gripper，单位：毫米
            A = np.linalg.inv(T_base2gripper1) @ T_base2gripper2
            
            # 计算B矩阵：Board1->Board2（通过Camera）
            # B = T_board1 @ inv(T_board2)，参考opencv_hand_eye_calibration.py:1122
            T_board1 = T_board_list[i]  # Board→Camera，单位：毫米
            T_board2 = T_board_list[i + 1]  # Board→Camera，单位：毫米
            T_board2_inv = np.linalg.inv(T_board2)
            B = T_board1 @ T_board2_inv
            
            # AX = XB 约束验证
            # 计算误差矩阵：E = A @ X - X @ B
            AX = A @ T_cam2gripper
            XB = T_cam2gripper @ B
            error_matrix = AX - XB
            
            # 平移误差：误差矩阵的平移部分的范数（单位：毫米）
            translation_error = float(np.linalg.norm(error_matrix[:3, 3]))
            
            # 旋转误差：计算A@X和X@B的旋转部分的角度差
            R_AX = (A @ T_cam2gripper)[:3, :3]
            R_XB = (T_cam2gripper @ B)[:3, :3]
            R_diff = R_AX @ R_XB.T
            trace = np.trace(R_diff)
            cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
            rotation_error_angle = float(np.arccos(cos_angle))
            
            errors.append({
                'pair_index': i,
                'translation_error_mm': translation_error,
                'rotation_error_rad': rotation_error_angle,
                'rotation_error_deg': np.degrees(rotation_error_angle)
            })
        except Exception as e:
            print(f"警告: 姿态对 #{i}-#{i+1} 误差计算失败: {e}，跳过")
            continue
    
    # 计算统计信息（与opencv_hand_eye_calibration.py:1326-1349完全一致）
    if errors:
        translation_errors = [e['translation_error_mm'] for e in errors]
        rotation_errors_rad = [e['rotation_error_rad'] for e in errors]
        
        # RMS误差（均方根误差）
        mean_translation_error = float(np.sqrt(np.mean([e**2 for e in translation_errors])))
        mean_rotation_error = float(np.sqrt(np.mean([e**2 for e in rotation_errors_rad])))
        max_translation_error = float(np.max(translation_errors))
        min_translation_error = float(np.min(translation_errors))
        std_translation_error = float(np.std(translation_errors))
        max_rotation_error = float(np.max(rotation_errors_rad))
        min_rotation_error = float(np.min(rotation_errors_rad))
        
        error_stats = {
            'mean_translation_error': mean_translation_error,
            'max_translation_error': max_translation_error,
            'min_translation_error': min_translation_error,
            'std_translation_error': std_translation_error,
            'mean_rotation_error_rad': mean_rotation_error,
            'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
            'max_rotation_error_rad': max_rotation_error,
            'max_rotation_error_deg': float(np.degrees(max_rotation_error)),
            'min_rotation_error_rad': min_rotation_error,
            'min_rotation_error_deg': float(np.degrees(min_rotation_error)),
            'translation_errors': translation_errors,
            'rotation_errors_rad': rotation_errors_rad,
            'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors_rad],
            # 兼容旧格式
            'translation': {
                'rms_mm': mean_translation_error,
                'mean_mm': np.mean(translation_errors),
                'max_mm': max_translation_error,
                'min_mm': min_translation_error,
                'std_mm': std_translation_error
            },
            'rotation': {
                'rms_deg': float(np.degrees(mean_rotation_error)),
                'mean_deg': float(np.degrees(np.mean(rotation_errors_rad))),
                'max_deg': float(np.degrees(max_rotation_error)),
                'min_deg': float(np.degrees(min_rotation_error)),
                'std_deg': float(np.degrees(np.std(rotation_errors_rad)))
            }
        }
    else:
        error_stats = None
    
    return errors, error_stats


def save_results(output_dir, R_cam2gripper, t_cam2gripper, errors, error_stats):
    """
    保存标定结果（OpenCV模式）
    
    参数:
        output_dir: 输出目录
        R_cam2gripper: 旋转矩阵
        t_cam2gripper: 平移向量（单位：米，OpenCV返回）
        errors: 误差列表
        error_stats: 误差统计
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # 构建变换矩阵（单位：毫米，与opencv_hand_eye_calibration.py一致）
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten() * 1000.0  # 米 → 毫米
    
    # 保存变换矩阵（文本格式）
    result_file = os.path.join(output_dir, 'hand_eye_result.txt')
    with open(result_file, 'w', encoding='utf-8') as f:
        f.write("=== 手眼标定结果 ===\n\n")
        f.write("旋转矩阵 R_cam2gripper:\n")
        f.write(str(R_cam2gripper))
        f.write("\n\n")
        f.write("平移向量 t_cam2gripper (米):\n")
        f.write(str(t_cam2gripper.flatten()))
        f.write("\n\n")
        f.write("平移向量 t_cam2gripper (毫米):\n")
        f.write(str(t_cam2gripper.flatten() * 1000))
        f.write("\n\n")
        f.write("变换矩阵 T_cam2gripper:\n")
        f.write(str(T_cam2gripper))
        f.write("\n\n")
        
        if error_stats:
            f.write("=== 误差统计 ===\n\n")
            f.write("平移误差:\n")
            f.write(f"  RMS: {error_stats['translation']['rms_mm']:.3f} mm\n")
            f.write(f"  均值: {error_stats['translation']['mean_mm']:.3f} mm\n")
            f.write(f"  最大值: {error_stats['translation']['max_mm']:.3f} mm\n")
            f.write(f"  最小值: {error_stats['translation']['min_mm']:.3f} mm\n")
            f.write(f"  标准差: {error_stats['translation']['std_mm']:.3f} mm\n\n")
            
            f.write("旋转误差:\n")
            f.write(f"  RMS: {error_stats['rotation']['rms_deg']:.3f} deg\n")
            f.write(f"  均值: {error_stats['rotation']['mean_deg']:.3f} deg\n")
            f.write(f"  最大值: {error_stats['rotation']['max_deg']:.3f} deg\n")
            f.write(f"  最小值: {error_stats['rotation']['min_deg']:.3f} deg\n")
            f.write(f"  标准差: {error_stats['rotation']['std_deg']:.3f} deg\n\n")
            
            f.write("各姿态对误差:\n")
            for e in errors:
                f.write(f"  姿态对 #{e['pair_index']}: 平移 {e['translation_error_mm']:.3f} mm, 旋转 {e['rotation_error_deg']:.3f} deg\n")
    
    print(f"\n结果已保存到: {result_file}")
    
    # 保存JSON格式
    result_json = {
        'rotation_matrix': R_cam2gripper.tolist(),
        'translation_vector_m': t_cam2gripper.flatten().tolist(),
        'translation_vector_mm': (t_cam2gripper.flatten() * 1000).tolist(),
        'transformation_matrix': T_cam2gripper.tolist(),
        'errors': errors,
        'error_statistics': error_stats
    }
    
    result_json_file = os.path.join(output_dir, 'hand_eye_result.json')
    with open(result_json_file, 'w', encoding='utf-8') as f:
        json.dump(result_json, f, indent=2, ensure_ascii=False)
    
    print(f"JSON结果已保存到: {result_json_file}")


def main():
    # 数据目录
    data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
    output_dir = '/home/mu/IVG/hand_eye_calibrate/calibration_results'
    
    print("="*60)
    print("眼在手上手眼标定 - OpenCV模式")
    print("="*60)
    print("算法: OpenCV calibrateHandEye (TSAI方法)")
    print("误差计算: AX=XB约束验证（与opencv_hand_eye_calibration.py完全一致）")
    print("="*60)
    
    # 加载数据
    print("\n1. 加载数据...")
    robot_poses, board_poses, pose_indices = load_pose_data(data_dir)
    
    if len(robot_poses) < 4:
        print(f"错误: 数据不足，至少需要4个姿态，当前只有{len(robot_poses)}个")
        return
    
    print(f"使用姿态索引: {pose_indices}")
    
    # 进行标定
    print("\n2. 进行手眼标定...")
    R_cam2gripper, t_cam2gripper = calibrate_hand_eye(robot_poses, board_poses)
    
    print("\n标定结果:")
    print("旋转矩阵 R_cam2gripper:")
    print(R_cam2gripper)
    print("\n平移向量 t_cam2gripper (米):")
    print(t_cam2gripper.flatten())
    print("\n平移向量 t_cam2gripper (毫米):")
    print(t_cam2gripper.flatten() * 1000)
    
    # 计算误差
    print("\n3. 计算误差...")
    errors, error_stats = calculate_errors(robot_poses, board_poses, R_cam2gripper, t_cam2gripper)
    
    if error_stats:
        print("\n误差统计（OpenCV模式）:")
        print(f"平移误差 RMS: {error_stats['mean_translation_error']:.3f} mm")
        print(f"平移误差 均值: {error_stats['translation']['mean_mm']:.3f} mm")
        print(f"平移误差 最大值: {error_stats['max_translation_error']:.3f} mm")
        print(f"平移误差 最小值: {error_stats['min_translation_error']:.3f} mm")
        print(f"平移误差 标准差: {error_stats['std_translation_error']:.3f} mm")
        print(f"\n旋转误差 RMS: {error_stats['mean_rotation_error_deg']:.3f} deg")
        print(f"旋转误差 均值: {error_stats['rotation']['mean_deg']:.3f} deg")
        print(f"旋转误差 最大值: {error_stats['max_rotation_error_deg']:.3f} deg")
        print(f"旋转误差 最小值: {error_stats['min_rotation_error_deg']:.3f} deg")
        print(f"旋转误差 标准差: {error_stats['rotation']['std_deg']:.3f} deg")
        
        print("\n各姿态对误差:")
        for e in errors:
            print(f"  姿态对 #{e['pair_index']}: 平移 {e['translation_error_mm']:.3f} mm, 旋转 {e['rotation_error_deg']:.3f} deg")
    
    # 保存结果
    print("\n4. 保存结果...")
    save_results(output_dir, R_cam2gripper, t_cam2gripper, errors, error_stats)
    
    print("\n" + "="*60)
    print("标定完成！")
    print("="*60)


if __name__ == '__main__':
    main()
