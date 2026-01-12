#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用HandEyeCalibrateForRobot实际数据验证OpenCV手眼标定模式
验证标定算法和误差计算的正确性
"""

import sys
import os
import cv2
import numpy as np
from datetime import datetime

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hand_eye_calibration.opencv_hand_eye_calibration import OpenCVHandEyeCalibration

np.set_printoptions(precision=6, suppress=True)


def euler_angles_to_rotation_matrix(rx, ry, rz):
    """欧拉角转旋转矩阵（ZYX顺序）"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    
    R = Rz @ Ry @ Rx
    return R


def pose_to_transform_matrix(x, y, z, rx, ry, rz):
    """位姿转换为4x4齐次变换矩阵"""
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """四元数转旋转矩阵"""
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
    ])


def rotation_matrix_to_quaternion(R):
    """旋转矩阵转四元数"""
    from scipy.spatial.transform import Rotation as R_scipy
    try:
        # 新版本scipy使用from_matrix
        r = R_scipy.from_matrix(R)
    except AttributeError:
        # 旧版本scipy使用from_dcm
        r = R_scipy.from_dcm(R)
    quat = r.as_quat()  # [x, y, z, w]
    return quat


def load_robot_poses(poses_file):
    """加载机器人位姿"""
    poses = []
    with open(poses_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            values = [float(v) for v in line.split(',')]
            if len(values) == 6:
                poses.append(values)
    return poses


def detect_chessboard(image_path, pattern_size=(11, 8), square_size=0.035):
    """检测棋盘格角点"""
    img = cv2.imread(image_path)
    if img is None:
        return None, None
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    
    if not ret:
        return None, None
    
    # 亚像素精度优化
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    
    # 构建3D点（标定板坐标系）
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
    
    return objp, corners2


def calibrate_camera(images_dir, pattern_size=(11, 8), square_size=0.035):
    """相机标定"""
    print("\n" + "="*80)
    print("步骤1：相机标定")
    print("="*80)
    
    obj_points = []  # 3D点
    img_points = []  # 2D点
    image_files = []
    
    # 加载所有图片
    for i in range(20):
        image_path = os.path.join(images_dir, f'images{i}.jpg')
        if not os.path.exists(image_path):
            continue
        
        objp, corners = detect_chessboard(image_path, pattern_size, square_size)
        if objp is not None:
            obj_points.append(objp)
            img_points.append(corners)
            image_files.append(image_path)
            print(f"  ✓ 图片 {i}: 检测到 {len(corners)} 个角点")
        else:
            print(f"  ✗ 图片 {i}: 未检测到角点")
    
    if len(obj_points) < 3:
        raise ValueError(f"有效图片不足，至少需要3张，当前只有{len(obj_points)}张")
    
    # 相机标定
    img = cv2.imread(image_files[0])
    img_size = (img.shape[1], img.shape[0])
    
    print(f"\n开始相机标定，使用 {len(obj_points)} 张图片...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )
    
    print(f"✓ 相机标定完成")
    print(f"\n内参矩阵:")
    print(camera_matrix)
    print(f"\n畸变系数:")
    print(dist_coeffs)
    
    return camera_matrix, dist_coeffs, rvecs, tvecs, obj_points, img_points


def prepare_hand_eye_data(robot_poses, rvecs, tvecs, camera_matrix, dist_coeffs):
    """准备手眼标定数据"""
    print("\n" + "="*80)
    print("步骤2：准备手眼标定数据")
    print("="*80)
    
    poses_data = []
    
    for i, (pose, rvec, tvec) in enumerate(zip(robot_poses, rvecs, tvecs)):
        x, y, z, rx, ry, rz = pose
        
        # 机器人位姿（单位：米）
        T_base2gripper = pose_to_transform_matrix(x, y, z, rx, ry, rz)
        R_base2gripper = T_base2gripper[:3, :3]
        t_base2gripper = T_base2gripper[:3, 3]
        quat = rotation_matrix_to_quaternion(R_base2gripper)
        
        # 标定板位姿（Board→Camera）
        R_board2cam, _ = cv2.Rodrigues(rvec)
        t_board2cam = tvec.flatten()
        quat_board = rotation_matrix_to_quaternion(R_board2cam)
        
        # 转换为项目数据格式
        rvec_flat = rvec.flatten()
        tvec_flat = tvec.flatten()
        
        pose_data = {
            'robot_pose': {
                'robot_pos_x': float(t_base2gripper[0]),  # 米
                'robot_pos_y': float(t_base2gripper[1]),
                'robot_pos_z': float(t_base2gripper[2]),
                'robot_ori_x': float(quat[0]),
                'robot_ori_y': float(quat[1]),
                'robot_ori_z': float(quat[2]),
                'robot_ori_w': float(quat[3])
            },
            'board_pose': {
                'rvec': {
                    'x': float(rvec_flat[0]),
                    'y': float(rvec_flat[1]),
                    'z': float(rvec_flat[2])
                },
                'tvec': {
                    'x': float(tvec_flat[0] * 1000.0),  # 转换为毫米
                    'y': float(tvec_flat[1] * 1000.0),
                    'z': float(tvec_flat[2] * 1000.0)
                },
                'position': {
                    'x': float(t_board2cam[0] * 1000.0),  # 毫米
                    'y': float(t_board2cam[1] * 1000.0),
                    'z': float(t_board2cam[2] * 1000.0)
                },
                'orientation': {
                    'x': float(quat_board[0]),
                    'y': float(quat_board[1]),
                    'z': float(quat_board[2]),
                    'w': float(quat_board[3])
                }
            }
        }
        
        poses_data.append(pose_data)
        
        print(f"  位姿 #{i+1}:")
        print(f"    机器人位置: [{t_base2gripper[0]:.6f}, {t_base2gripper[1]:.6f}, {t_base2gripper[2]:.6f}] m")
        print(f"    标定板位置: [{t_board2cam[0]*1000:.2f}, {t_board2cam[1]*1000:.2f}, {t_board2cam[2]*1000:.2f}] mm")
    
    print(f"\n✓ 准备完成：{len(poses_data)} 个位姿")
    return poses_data


def verify_opencv_calibration(poses_data):
    """验证OpenCV手眼标定"""
    print("\n" + "="*80)
    print("步骤3：OpenCV手眼标定")
    print("="*80)
    
    # 创建标定器
    calibrator = OpenCVHandEyeCalibration()
    
    # 执行标定
    result = calibrator.calibrate(poses_data)
    
    # 提取结果
    T_camera2gripper = result['T_camera2gripper']
    error_stats = result['error_statistics']
    
    print("\n✓ 标定完成！")
    print("\n【标定结果】")
    print("相机到末端执行器的变换矩阵 T_camera2gripper (单位: 毫米):")
    print(T_camera2gripper)
    
    R_cam2gripper = T_camera2gripper[:3, :3]
    t_cam2gripper = T_camera2gripper[:3, 3]
    
    print(f"\n旋转矩阵:")
    print(R_cam2gripper)
    print(f"\n平移向量: [{t_cam2gripper[0]:.6f}, {t_cam2gripper[1]:.6f}, {t_cam2gripper[2]:.6f}] mm")
    print(f"平移范数: {np.linalg.norm(t_cam2gripper):.2f} mm")
    print(f"旋转矩阵行列式: {np.linalg.det(R_cam2gripper):.6f}")
    
    # 转换为欧拉角显示
    from scipy.spatial.transform import Rotation as R_scipy
    try:
        r = R_scipy.from_matrix(R_cam2gripper)
    except AttributeError:
        r = R_scipy.from_dcm(R_cam2gripper)
    euler_angles = r.as_euler('xyz', degrees=True)
    print(f"旋转欧拉角 (XYZ, 度): [{euler_angles[0]:.2f}°, {euler_angles[1]:.2f}°, {euler_angles[2]:.2f}°]")
    
    print("\n【误差统计】")
    print("AX=XB约束验证:")
    print(f"  平移RMS误差: {error_stats['mean_translation_error']:.6f} mm")
    print(f"  旋转RMS误差: {error_stats['mean_rotation_error_deg']:.6f}°")
    print(f"  最大平移误差: {error_stats['max_translation_error']:.6f} mm")
    print(f"  最小平移误差: {error_stats['min_translation_error']:.6f} mm")
    print(f"  最大旋转误差: {error_stats['max_rotation_error_deg']:.6f}°")
    print(f"  最小旋转误差: {error_stats['min_rotation_error_deg']:.6f}°")
    
    # 评估标定质量
    translation_rms = error_stats['mean_translation_error']
    rotation_rms = error_stats['mean_rotation_error_deg']
    
    if translation_rms < 1.0 and rotation_rms < 0.5:
        quality = "✅ 优秀"
    elif translation_rms < 5.0 and rotation_rms < 2.0:
        quality = "✓ 良好"
    elif translation_rms < 10.0 and rotation_rms < 5.0:
        quality = "⚠ 可接受"
    else:
        quality = "❌ 需要改进"
    
    print(f"\n【标定质量评估】: {quality}")
    
    return result


def compare_with_reference(result):
    """与参考代码结果对比"""
    print("\n" + "="*80)
    print("步骤4：与参考代码结果对比")
    print("="*80)
    
    # 参考代码的结果（从/home/mu/IVG/hand_eye_calibrate/run_calibration.py运行结果）
    R_ref = np.array([
        [ 0.34218072,  0.0548817,   0.93803004],
        [ 0.73947496,  0.60019439, -0.30486632],
        [-0.57973195,  0.7979691,   0.16479131]
    ])
    t_ref = np.array([-0.22463304, -0.03962381, -0.04508892]).reshape(3, 1)
    
    T_ref = np.eye(4)
    T_ref[:3, :3] = R_ref
    T_ref[:3, 3] = t_ref.flatten()
    
    # 当前结果
    T_current = result['T_camera2gripper']
    
    # 注意：参考代码的单位可能是米，需要转换
    # 判断单位
    t_current_norm = np.linalg.norm(T_current[:3, 3])
    t_ref_norm = np.linalg.norm(t_ref)
    
    if t_current_norm > 10 * t_ref_norm:
        # 当前是毫米，参考是米
        T_ref[:3, 3] *= 1000.0
        t_ref *= 1000.0
        print("注意：参考结果单位已转换为毫米")
    
    print("\n【参考结果】（/home/mu/IVG/hand_eye_calibrate/run_calibration.py）")
    print("旋转矩阵:")
    print(R_ref)
    print(f"平移向量: {t_ref.flatten()}")
    
    print("\n【当前结果】（OpenCV模式）")
    print("旋转矩阵:")
    print(T_current[:3, :3])
    print(f"平移向量: {T_current[:3, 3]}")
    
    # 计算差异
    R_diff = T_current[:3, :3] @ R_ref.T
    rotation_diff_angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
    translation_diff = np.linalg.norm(T_current[:3, 3] - t_ref.flatten())
    
    print("\n【差异分析】")
    print(f"旋转差异: {rotation_diff_angle * 180 / np.pi:.6f}°")
    print(f"平移差异: {translation_diff:.6f} mm")
    
    if rotation_diff_angle * 180 / np.pi < 1.0 and translation_diff < 5.0:
        print("✅ 结果一致性：优秀（与参考代码结果高度一致）")
    elif rotation_diff_angle * 180 / np.pi < 5.0 and translation_diff < 20.0:
        print("✓ 结果一致性：良好")
    else:
        print("⚠ 结果一致性：存在差异，可能是算法或参数不同")


def main():
    """主函数"""
    print("="*80)
    print("使用实际数据验证OpenCV手眼标定模式")
    print("="*80)
    print(f"验证时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"数据来源: /home/mu/IVG/hand_eye_calibrate/collect_data")
    
    # 数据路径
    data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
    poses_file = os.path.join(data_dir, 'poses.txt')
    
    # 检查数据
    if not os.path.exists(data_dir):
        print(f"错误：数据目录不存在: {data_dir}")
        return
    
    if not os.path.exists(poses_file):
        print(f"错误：位姿文件不存在: {poses_file}")
        return
    
    try:
        # 1. 加载机器人位姿
        robot_poses = load_robot_poses(poses_file)
        print(f"\n✓ 加载了 {len(robot_poses)} 个机器人位姿")
        
        # 2. 相机标定
        camera_matrix, dist_coeffs, rvecs, tvecs, obj_points, img_points = calibrate_camera(
            data_dir, pattern_size=(11, 8), square_size=0.035
        )
        
        # 3. 准备手眼标定数据
        poses_data = prepare_hand_eye_data(robot_poses, rvecs, tvecs, camera_matrix, dist_coeffs)
        
        # 4. 执行OpenCV手眼标定
        result = verify_opencv_calibration(poses_data)
        
        # 5. 与参考代码对比
        compare_with_reference(result)
        
        print("\n" + "="*80)
        print("验证完成！")
        print("="*80)
        
    except Exception as e:
        print(f"\n❌ 验证失败: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

