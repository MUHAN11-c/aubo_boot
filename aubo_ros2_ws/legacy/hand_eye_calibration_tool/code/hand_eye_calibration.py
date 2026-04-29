#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手上（Eye-in-Hand）单目相机手眼标定程序 - 重新设计版本

数据含义：
1. robot_shot_pose.json: 拍照时末端执行器姿态（相机垂直于桌面）
2. CornerCoordinates.csv: 拍照时棋盘格角点在相机坐标系下的3D坐标
3. robot_status_*.json: 点选时末端执行器姿态（标志点接触角点）

核心约束：
- 棋盘格固定在桌面，角点在基座坐标系下的位置是固定的
- 通过拍照姿态和手眼标定可以计算角点在基座坐标系的位置
- 通过点选姿态可以知道角点在基座坐标系的XY位置（标志点位置）
- 两者应该一致！
"""

import os
import json
import csv
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from datetime import datetime
import xml.etree.ElementTree as ET
from xml.dom import minidom

# ========== 配置参数 ==========
# 多轮标定配置说明：
# 程序会进行多轮标定，每轮都会：
# 1. 使用当前数据点进行标定
# 2. 评估误差
# 3. 剔除误差大的点（根据阈值和比例）
# 4. 使用剩余点进行下一轮标定
# 最终选择平均误差最小的轮作为最佳结果
#
# 参数说明：
# - num_rounds: 标定轮数（至少1轮，建议3-10轮）
# - error_threshold_multiplier: 误差阈值倍数，用于确定异常点
#   计算方式：阈值 = 平均误差 + multiplier * 标准差
#   例如：2.0 表示剔除误差超过"平均值+2倍标准差"的点
# - remove_percentage: 每轮最多剔除的数据点比例（0.0-1.0）
#   例如：0.2 表示每轮最多剔除20%的点
#   实际剔除数量 = max(阈值以上点数, 比例对应点数)
# - min_points_per_round: 每轮至少需要的数据点数，少于则停止
# - stop_if_no_improvement: 如果误差改善不足是否提前停止
# - improvement_threshold: 误差改善阈值（0.0-1.0）
#   例如：0.01 表示如果误差改善不足1%，则提前停止
CALIBRATION_CONFIG = {
    'num_rounds': 5,  # 标定轮数（至少1轮）
    'error_threshold_multiplier': 2.0,  # 误差阈值倍数（平均值 + multiplier * 标准差）
    'remove_percentage': 0.2,  # 每轮剔除的最大比例（20%）
    'min_points_per_round': 3,  # 每轮至少需要的数据点数
    'min_points_final': 3,  # 最终结果至少需要的数据点数
    'stop_if_no_improvement': True,  # 如果误差不再改善是否提前停止
    'improvement_threshold': 0.01  # 误差改善阈值（1%），低于此值认为没有改善
}
# =============================


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """将四元数转换为旋转矩阵"""
    r = R.from_quat([qx, qy, qz, qw])
    return r.as_matrix()


def pose_to_transform_matrix(position, orientation_quat):
    """将位置和四元数组合成4x4齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_rotation_matrix(
        orientation_quat['x'],
        orientation_quat['y'],
        orientation_quat['z'],
        orientation_quat['w']
    )
    T[:3, 3] = [position['x'], position['y'], position['z']]
    return T


def load_robot_status_files(data_dir):
    """加载所有robot_status JSON文件"""
    robot_status_files = []
    for filename in sorted(os.listdir(data_dir)):
        if filename.startswith('robot_status_') and filename.endswith('.json'):
            filepath = os.path.join(data_dir, filename)
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if 'corner_index' in data:
                    data['_filename'] = filename
                    robot_status_files.append(data)
    return robot_status_files


def load_corner_coordinates(csv_path, expected_z=None):
    """加载角点坐标CSV文件
    
    参数:
        csv_path: CSV文件路径
        expected_z: 期望的Z坐标值（如果提供，会检查并修正Z值）
    """
    corner_dict = {}
    encodings = ['utf-8', 'gbk', 'gb2312', 'latin-1']
    content = None
    
    for encoding in encodings:
        try:
            with open(csv_path, 'r', encoding=encoding) as f:
                content = f.read()
                break
        except UnicodeDecodeError:
            continue
    
    if content is None:
        raise ValueError(f"无法读取文件 {csv_path}")
    
    lines = content.strip().split('\n')
    
    z_values_found = set()
    
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = line.split(',')
        if len(parts) >= 6:
            try:
                corner_idx = int(parts[0])
                u = float(parts[1])
                v = float(parts[2])
                x_cam = float(parts[3])
                y_cam = float(parts[4])
                z_cam = float(parts[5])
                
                # 记录找到的Z值
                z_values_found.add(z_cam)
                
                # 如果提供了期望的Z值，且当前Z值不匹配，则修正
                if expected_z is not None and abs(z_cam - expected_z) > 1.0:
                    print(f"警告: 角点 {corner_idx} 的Z坐标 {z_cam:.6f} 与期望值 {expected_z:.6f} 不匹配，已修正")
                    z_cam = expected_z
                
                corner_dict[corner_idx] = {
                    'pixel': [u, v],
                    'camera_coord': np.array([x_cam, y_cam, z_cam])
                }
            except (ValueError, IndexError) as e:
                print(f"警告: 解析行失败: {line}, 错误: {e}")
                continue
    
    # 检查Z值一致性
    if len(z_values_found) > 1:
        print(f"警告: 文件中发现多个不同的Z值: {sorted(z_values_found)}")
        if expected_z is not None:
            print(f"已将Z值统一修正为: {expected_z:.6f}")
    
    return corner_dict


def load_camera_params(xml_path):
    """加载相机内参"""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    camera_matrix_node = root.find('CameraMatrix')
    data = camera_matrix_node.find('data').text.strip().split()
    camera_matrix = np.array([float(x) for x in data]).reshape(3, 3)
    
    dist_coeffs_node = root.find('DistortionCoefficients')
    data = dist_coeffs_node.find('data').text.strip().split()
    dist_coeffs = np.array([float(x) for x in data])
    
    image_size_node = root.find('ImageSize')
    data = image_size_node.find('data').text.strip().split()
    image_size = (int(float(data[1])), int(float(data[0])))
    
    return camera_matrix, dist_coeffs, image_size


def solve_hand_eye_calibration(shot_pose, pick_poses, camera_points_shot, fixed_z=None):
    """
    手眼标定核心算法
    
    约束条件：
    对于每个角点i：
    P_base_i = T_gripper_shot2base * T_camera2gripper * P_camera_shot_i
    
    其中：
    - P_base_i: 角点i在基座坐标系下的位置（固定值）
    - T_gripper_shot2base: 拍照时末端执行器到基座的变换（已知）
    - T_camera2gripper: 相机到末端执行器的变换（待求）
    - P_camera_shot_i: 拍照时角点i在相机坐标系下的坐标（已知）
    
    同时，从点选姿态可以知道：
    - P_base_i_XY = T_gripper_pick2base_i[:2, 3]（标志点位置，即角点XY）
    - P_base_i_Z = T_gripper_pick2base_i[2, 3]（点选时末端执行器Z位置，即角点Z）
    
    因此：
    (T_gripper_shot2base * T_camera2gripper * P_camera_shot_i)[:2] = T_gripper_pick2base_i[:2, 3]
    (T_gripper_shot2base * T_camera2gripper * P_camera_shot_i)[2] = T_gripper_pick2base_i[2, 3]
    """
    n = len(pick_poses)
    
    # 为每个数据点使用各自点选姿态的Z值
    # 因为不同棋盘格姿态可能略有不同，使用各自的Z值更准确
    pick_z_values = [T_pick[2, 3] for T_pick in pick_poses]
    if fixed_z is None:
        fixed_z = np.mean(pick_z_values)
        print(f"平均Z值（棋盘格桌面高度）: {fixed_z:.6f} mm")
        print(f"Z值范围: {np.min(pick_z_values):.6f} ~ {np.max(pick_z_values):.6f} mm")
    
    # 构建优化目标函数
    def residuals(params):
        """
        计算残差
        参数：旋转轴角(3) + 平移向量(3) = 6个参数
        """
        # 提取参数
        axis_angle = params[:3]
        translation = params[3:6]
        
        # 构建旋转矩阵
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-6:
            R_mat = np.eye(3)
        else:
            axis = axis_angle / angle
            r = R.from_rotvec(axis * angle)
            R_mat = r.as_matrix()
        
        # 构建变换矩阵 T_camera2gripper
        T_camera2gripper = np.eye(4)
        T_camera2gripper[:3, :3] = R_mat
        T_camera2gripper[:3, 3] = translation
        
        residuals = []
        
        for i in range(n):
            T_gripper_pick2base = pick_poses[i]
            P_camera_shot = camera_points_shot[i]  # 拍照时角点在相机坐标系下的坐标
            
            # 从拍照姿态和手眼标定矩阵计算角点在基座坐标系下的位置
            # P_base = T_gripper_shot2base * T_camera2gripper * P_camera_shot
            # 步骤：
            # 1. 相机坐标系 -> 末端执行器坐标系：T_camera2gripper * P_camera_shot
            # 2. 末端执行器坐标系 -> 基座坐标系：shot_pose * (T_camera2gripper * P_camera_shot)
            P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
            P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
            P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
            P_base_computed = P_base_computed_homogeneous[:3]
            
            # 从点选姿态得到角点在基座坐标系下的实际位置
            # 点选时，标志点接触角点，所以角点的位置 = 标志点的位置（末端执行器位置）
            P_base_actual = T_gripper_pick2base[:3, 3]  # 标志点位置（XY和Z）
            
            # XY+Z约束：从拍照姿态计算出的位置应该等于点选时的位置
            residual = P_base_computed - P_base_actual
            residuals.extend(residual)
        
        return np.array(residuals)
    
    # 初始估计：使用SVD方法估计旋转和平移
    # 对于每个点i：
    # P_base_i = T_gripper_shot2base * T_camera2gripper * P_camera_shot_i
    # 
    # 转换到末端执行器坐标系：
    # T_gripper_shot2base^-1 * P_base_i = T_camera2gripper * P_camera_shot_i
    # P_gripper_i = T_camera2gripper * P_camera_shot_i
    # 
    # 使用SVD方法求解两组点云之间的刚体变换
    
    T_gripper_shot2base_inv = np.linalg.inv(shot_pose)
    
    # 构建点对
    points_gripper = []  # 末端执行器坐标系下的点
    points_camera = []   # 相机坐标系下的点
    
    for i in range(n):
        T_gripper_pick2base = pick_poses[i]
        P_camera_shot = camera_points_shot[i]
        
        # 从点选姿态得到角点在基座坐标系的位置
        P_base = T_gripper_pick2base[:3, 3]
        
        # 转换到末端执行器坐标系（拍照时的末端执行器坐标系）
        P_base_homogeneous = np.append(P_base, 1.0)
        P_gripper_homogeneous = T_gripper_shot2base_inv @ P_base_homogeneous
        P_gripper = P_gripper_homogeneous[:3]
        
        points_gripper.append(P_gripper)
        points_camera.append(P_camera_shot)
    
    points_gripper = np.array(points_gripper)
    points_camera = np.array(points_camera)
    
    # 使用SVD方法求解刚体变换（Umeyama算法）
    # 计算质心
    centroid_gripper = np.mean(points_gripper, axis=0)
    centroid_camera = np.mean(points_camera, axis=0)
    
    # 去中心化
    points_gripper_centered = points_gripper - centroid_gripper
    points_camera_centered = points_camera - centroid_camera
    
    # 计算协方差矩阵
    H = points_camera_centered.T @ points_gripper_centered
    
    # SVD分解
    U, S, Vt = np.linalg.svd(H)
    
    # 计算旋转矩阵
    R_init = Vt.T @ U.T
    
    # 确保是右手坐标系（行列式为1）
    if np.linalg.det(R_init) < 0:
        Vt[-1, :] *= -1
        R_init = Vt.T @ U.T
    
    # 计算平移向量
    t_init = centroid_gripper - R_init @ centroid_camera
    
    # 将旋转矩阵转换为轴角表示
    r_init = R.from_matrix(R_init)
    axis_angle_init = r_init.as_rotvec()
    
    initial_params = np.concatenate([axis_angle_init, t_init])
    
    print(f"\n初始估计 - 使用SVD方法估计旋转和平移")
    print(f"初始旋转轴角: {axis_angle_init}")
    print(f"初始旋转角度: {np.linalg.norm(axis_angle_init):.6f} rad ({np.degrees(np.linalg.norm(axis_angle_init)):.6f} deg)")
    print(f"初始平移向量: {t_init}")
    
    # 调试：检查初始估计的残差
    initial_residuals = residuals(initial_params)
    print(f"\n初始残差统计:")
    print(f"  平均残差: {np.mean(np.abs(initial_residuals)):.6f} mm")
    print(f"  最大残差: {np.max(np.abs(initial_residuals)):.6f} mm")
    
    # 优化求解
    print(f"\n开始优化求解手眼标定（使用XY+Z约束）...")
    result = least_squares(residuals, initial_params, method='lm',
                          max_nfev=10000, ftol=1e-8, xtol=1e-8, verbose=0)
    
    # 提取结果
    axis_angle = result.x[:3]
    translation = result.x[3:6]
    
    # 构建旋转矩阵
    angle = np.linalg.norm(axis_angle)
    if angle < 1e-6:
        R_camera2gripper = np.eye(3)
    else:
        axis = axis_angle / angle
        r = R.from_rotvec(axis * angle)
        R_camera2gripper = r.as_matrix()
    
    # 构建变换矩阵
    T_camera2gripper = np.eye(4)
    T_camera2gripper[:3, :3] = R_camera2gripper
    T_camera2gripper[:3, 3] = translation
    
    mean_residual = np.mean(np.abs(result.fun))
    print(f"优化完成，平均残差: {mean_residual:.6f} mm")
    
    return T_camera2gripper, fixed_z


def evaluate_calibration_error(shot_pose, pick_poses, camera_points_shot, 
                               T_camera2gripper, data_metadata, fixed_z):
    """评估标定误差"""
    errors = []
    error_details = []
    
    for i in range(len(pick_poses)):
        T_gripper_pick2base = pick_poses[i]
        P_camera_shot = camera_points_shot[i]
        metadata = data_metadata[i]
        
        # 实际角点位置（基座坐标系）
        # 点选时，标志点接触角点，所以角点位置 = 标志点位置（末端执行器位置）
        P_base_actual = T_gripper_pick2base[:3, 3]
        
        # 使用标定结果计算的位置（基座坐标系）
        # P_base = T_gripper_shot2base * T_camera2gripper * P_camera_shot
        P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
        P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
        P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
        P_base_computed = P_base_computed_homogeneous[:3]
        
        # 计算误差
        error = np.linalg.norm(P_base_computed - P_base_actual)
        errors.append(error)
        
        error_details.append({
            'chessboard_pose': metadata['chessboard_pose'],
            'json_filename': metadata['json_filename'],
            'corner_index': metadata['corner_index'],
            'camera_x': P_camera_shot[0],
            'camera_y': P_camera_shot[1],
            'camera_z': P_camera_shot[2],
            'actual_x': P_base_actual[0],
            'actual_y': P_base_actual[1],
            'actual_z': P_base_actual[2],
            'computed_x': P_base_computed[0],
            'computed_y': P_base_computed[1],
            'computed_z': P_base_computed[2],
            'error_x': P_base_computed[0] - P_base_actual[0],
            'error_y': P_base_computed[1] - P_base_actual[1],
            'error_z': P_base_computed[2] - P_base_actual[2],
            'error_magnitude': error
        })
    
    return errors, error_details


def save_calibration_result(output_path, T_camera2gripper, camera_matrix, dist_coeffs,
                           image_size, errors, error_details, calibration_type='Eye-in-Hand'):
    """保存标定结果到XML文件"""
    root = ET.Element('opencv_storage')
    
    comment = ET.Comment(' 相机标定参数 (Camera Calibration Parameters) ')
    root.append(comment)
    
    image_size_elem = ET.SubElement(root, 'ImageSize')
    image_size_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(image_size_elem, 'rows').text = '1'
    ET.SubElement(image_size_elem, 'cols').text = '2'
    ET.SubElement(image_size_elem, 'dt').text = 'd'
    ET.SubElement(image_size_elem, 'data').text = f'{image_size[0]} {image_size[1]}'
    
    camera_matrix_elem = ET.SubElement(root, 'CameraMatrix')
    camera_matrix_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(camera_matrix_elem, 'rows').text = '3'
    ET.SubElement(camera_matrix_elem, 'cols').text = '3'
    ET.SubElement(camera_matrix_elem, 'dt').text = 'd'
    data_str = ' '.join([f'{camera_matrix[i,j]:.16f}' for i in range(3) for j in range(3)])
    ET.SubElement(camera_matrix_elem, 'data').text = data_str
    
    focal_length_elem = ET.SubElement(root, 'FocalLength')
    focal_length_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(focal_length_elem, 'rows').text = '1'
    ET.SubElement(focal_length_elem, 'cols').text = '2'
    ET.SubElement(focal_length_elem, 'dt').text = 'd'
    ET.SubElement(focal_length_elem, 'data').text = f'{camera_matrix[0,0]:.16f} {camera_matrix[1,1]:.16f}'
    
    principal_point_elem = ET.SubElement(root, 'PrincipalPoint')
    principal_point_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(principal_point_elem, 'rows').text = '1'
    ET.SubElement(principal_point_elem, 'cols').text = '2'
    ET.SubElement(principal_point_elem, 'dt').text = 'd'
    ET.SubElement(principal_point_elem, 'data').text = f'{camera_matrix[0,2]:.16f} {camera_matrix[1,2]:.16f}'
    
    dist_coeffs_elem = ET.SubElement(root, 'DistortionCoefficients')
    dist_coeffs_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(dist_coeffs_elem, 'rows').text = '1'
    ET.SubElement(dist_coeffs_elem, 'cols').text = str(len(dist_coeffs))
    ET.SubElement(dist_coeffs_elem, 'dt').text = 'd'
    data_str = ' '.join([f'{x:.16f}' for x in dist_coeffs])
    ET.SubElement(dist_coeffs_elem, 'data').text = data_str
    
    comment2 = ET.Comment(' 手眼标定结果 (Hand-Eye Calibration Results) ')
    root.append(comment2)
    
    ET.SubElement(root, 'CalibrationType').text = calibration_type
    ET.SubElement(root, 'CalibrationMethod').text = 'Nonlinear Optimization with XY+Z Constraints'
    ET.SubElement(root, 'CalibrationDate').text = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    transform_elem = ET.SubElement(root, 'TransformationMatrix')
    transform_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(transform_elem, 'rows').text = '4'
    ET.SubElement(transform_elem, 'cols').text = '4'
    ET.SubElement(transform_elem, 'dt').text = 'd'
    data_str = ' '.join([f'{T_camera2gripper[i,j]:.16f}' for i in range(4) for j in range(4)])
    ET.SubElement(transform_elem, 'data').text = data_str
    
    rotation_elem = ET.SubElement(root, 'RotationMatrix')
    rotation_elem.set('type_id', 'opencv-matrix')
    ET.SubElement(rotation_elem, 'rows').text = '3'
    ET.SubElement(rotation_elem, 'cols').text = '3'
    ET.SubElement(rotation_elem, 'dt').text = 'd'
    data_str = ' '.join([f'{T_camera2gripper[i,j]:.16f}' for i in range(3) for j in range(3)])
    ET.SubElement(rotation_elem, 'data').text = data_str
    
    translation_elem = ET.SubElement(root, 'TranslationVector')
    translation_elem.set('type_id', 'opencv-matrix')
    translation_elem.set('unit', 'mm')
    ET.SubElement(translation_elem, 'rows').text = '3'
    ET.SubElement(translation_elem, 'cols').text = '1'
    ET.SubElement(translation_elem, 'dt').text = 'd'
    data_str = ' '.join([f'{T_camera2gripper[i,3]:.16f}' for i in range(3)])
    ET.SubElement(translation_elem, 'data').text = data_str
    
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    min_error = np.min(errors)
    std_error = np.std(errors)
    
    ET.SubElement(root, 'MeanCalibrationError', unit='mm').text = f'{mean_error:.6f}'
    ET.SubElement(root, 'MaxCalibrationError', unit='mm').text = f'{max_error:.6f}'
    ET.SubElement(root, 'MinCalibrationError', unit='mm').text = f'{min_error:.6f}'
    ET.SubElement(root, 'StdCalibrationError', unit='mm').text = f'{std_error:.6f}'
    ET.SubElement(root, 'DataPointCount').text = str(len(errors))
    
    usage_notes = ET.SubElement(root, 'UsageNotes')
    usage_text = f"""
      此文件包含完整的相机内参和手眼标定结果。
      变换矩阵将相机坐标系的点转换到末端执行器坐标系。
      标定场景: {calibration_type}
      标定方法: Nonlinear Optimization with XY+Z Constraints
      平均误差: {mean_error:.3f} mm
      单位说明: 平移向量单位为毫米(mm)，旋转矩阵无单位
    """
    usage_notes.text = usage_text
    
    xml_str = ET.tostring(root, encoding='utf-8')
    dom = minidom.parseString(xml_str)
    pretty_xml = dom.toprettyxml(indent='   ', encoding='utf-8')
    
    with open(output_path, 'wb') as f:
        f.write(pretty_xml)


def save_error_evaluation(output_path, error_details):
    """保存误差评估到CSV文件"""
    with open(output_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            '棋盘格姿态',
            '末端json文件',
            '角点索引',
            '相机坐标系X(mm)',
            '相机坐标系Y(mm)',
            '相机坐标系Z(mm)',
            '实际X(mm)',
            '实际Y(mm)',
            '实际Z(mm)',
            '计算X(mm)',
            '计算Y(mm)',
            '计算Z(mm)',
            '实际与计算误差(mm)'
        ])
        
        for detail in error_details:
            writer.writerow([
                detail['chessboard_pose'],
                detail['json_filename'],
                detail['corner_index'],
                f'{detail["camera_x"]:.6f}',
                f'{detail["camera_y"]:.6f}',
                f'{detail["camera_z"]:.6f}',
                f'{detail["actual_x"]:.6f}',
                f'{detail["actual_y"]:.6f}',
                f'{detail["actual_z"]:.6f}',
                f'{detail["computed_x"]:.6f}',
                f'{detail["computed_y"]:.6f}',
                f'{detail["computed_z"]:.6f}',
                f'{detail["error_magnitude"]:.6f}'
            ])


def main():
    """主函数"""
    base_dir = '/home/nvidia/RVG_ws/src/hand_eye_calibration/hand_eye_calibration_tool/data'
    camera_params_path = os.path.join(base_dir, 'cameraParams.xml')
    
    chessboard_pose_dirs = [
        os.path.join(base_dir, 'chessboard_pose_1'),
        os.path.join(base_dir, 'chessboard_pose_2'),
        os.path.join(base_dir, 'chessboard_pose_3')
    ]
    
    # 加载拍照姿态
    shot_pose_path = os.path.join(base_dir, 'robot_shot_pose.json')
    print("正在加载拍照姿态...")
    with open(shot_pose_path, 'r', encoding='utf-8') as f:
        shot_pose_data = json.load(f)
    shot_pose = pose_to_transform_matrix(
        shot_pose_data['cartesian_position']['position'],
        shot_pose_data['cartesian_position']['orientation']
    )
    print(f"拍照姿态加载完成")
    
    # 加载相机内参
    print("正在加载相机内参...")
    camera_matrix, dist_coeffs, image_size = load_camera_params(camera_params_path)
    print(f"相机内参加载完成，图像尺寸: {image_size}")
    
    # 收集所有数据
    print("\n正在加载标定数据...")
    pick_poses = []
    camera_points_shot = []
    data_metadata = []
    
    # 首先确定标准Z值（使用第一个pose的Z值）
    standard_z = None
    
    for pose_dir in chessboard_pose_dirs:
        print(f"  处理目录: {pose_dir}")
        pose_name = os.path.basename(pose_dir)
        
        robot_status_list = load_robot_status_files(pose_dir)
        csv_path = os.path.join(pose_dir, 'CornerCoordinates.csv')
        
        # 如果是第一个pose，确定标准Z值
        if standard_z is None:
            corner_dict_temp = load_corner_coordinates(csv_path)
            if corner_dict_temp:
                # 获取第一个角点的Z值作为标准
                first_corner = list(corner_dict_temp.values())[0]
                standard_z = first_corner['camera_coord'][2]
                print(f"  确定标准Z值（相机坐标系）: {standard_z:.6f} mm")
        
        # 加载角点坐标，统一Z值为标准值
        corner_dict = load_corner_coordinates(csv_path, expected_z=standard_z)
        
        for status in robot_status_list:
            corner_idx = status['corner_index']
            if corner_idx in corner_dict:
                cartesian_pos = status['cartesian_position']
                T_gripper_pick2base = pose_to_transform_matrix(
                    cartesian_pos['position'],
                    cartesian_pos['orientation']
                )
                pick_poses.append(T_gripper_pick2base)
                
                P_camera_shot = corner_dict[corner_idx]['camera_coord']
                camera_points_shot.append(P_camera_shot)
                
                data_metadata.append({
                    'chessboard_pose': pose_name,
                    'json_filename': status['_filename'],
                    'corner_index': corner_idx
                })
                
                print(f"    角点 {corner_idx}: 点选位置 [{cartesian_pos['position']['x']:.2f}, "
                      f"{cartesian_pos['position']['y']:.2f}, {cartesian_pos['position']['z']:.2f}], "
                      f"拍照时相机坐标 [{P_camera_shot[0]:.2f}, {P_camera_shot[1]:.2f}, {P_camera_shot[2]:.2f}]")
            else:
                print(f"    警告: 角点索引 {corner_idx} 在CSV文件中未找到")
    
    print(f"\n共收集到 {len(pick_poses)} 组数据点")
    
    if len(pick_poses) < CALIBRATION_CONFIG['min_points_per_round']:
        print(f"错误: 数据点不足，至少需要 {CALIBRATION_CONFIG['min_points_per_round']} 组数据")
        return
    
    # ========== 多轮标定循环 ==========
    print("\n" + "="*60)
    print(f"开始多轮手眼标定（配置：{CALIBRATION_CONFIG['num_rounds']} 轮）")
    print("="*60)
    
    # 初始化当前轮的数据
    current_pick_poses = pick_poses.copy()
    current_camera_points = camera_points_shot.copy()
    current_metadata = data_metadata.copy()
    
    # 存储所有轮的结果
    all_rounds_results = []
    previous_mean_error = None
    
    for round_num in range(1, CALIBRATION_CONFIG['num_rounds'] + 1):
        print("\n" + "="*60)
        print(f"第 {round_num} 轮手眼标定：使用 {len(current_pick_poses)} 个数据点")
        print("="*60)
        
        # 检查数据点数量
        if len(current_pick_poses) < CALIBRATION_CONFIG['min_points_per_round']:
            print(f"警告: 数据点不足 {CALIBRATION_CONFIG['min_points_per_round']} 个，停止标定")
            break
        
        # 执行标定
        print(f"\n开始第 {round_num} 轮手眼标定...")
        T_camera2gripper, fixed_z = solve_hand_eye_calibration(
            shot_pose, current_pick_poses, current_camera_points
        )
        
        print(f"第 {round_num} 轮手眼标定完成！")
        print(f"\n相机到末端执行器的变换矩阵（第 {round_num} 轮）:")
        print(T_camera2gripper)
        print(f"固定的Z值（棋盘格桌面高度）: {fixed_z:.6f} mm")
        
        # 评估标定误差
        print(f"\n评估第 {round_num} 轮标定误差...")
        errors, error_details = evaluate_calibration_error(
            shot_pose, current_pick_poses, current_camera_points,
            T_camera2gripper, current_metadata, fixed_z
        )
        
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        min_error = np.min(errors)
        std_error = np.std(errors)
        
        print(f"\n第 {round_num} 轮标定误差统计:")
        print(f"  平均误差: {mean_error:.6f} mm")
        print(f"  最大误差: {max_error:.6f} mm")
        print(f"  最小误差: {min_error:.6f} mm")
        print(f"  标准差: {std_error:.6f} mm")
        
        # 保存当前轮结果
        output_dir = base_dir
        output_xml = os.path.join(output_dir, f'hand_eye_calibration_round{round_num}.xml')
        output_csv = os.path.join(output_dir, f'calibration_error_evaluation_round{round_num}.csv')
        
        print(f"\n保存第 {round_num} 轮标定结果到: {output_xml}")
        save_calibration_result(
            output_xml, T_camera2gripper, camera_matrix, dist_coeffs,
            image_size, errors, error_details, 'Eye-in-Hand'
        )
        
        print(f"保存第 {round_num} 轮误差评估到: {output_csv}")
        save_error_evaluation(output_csv, error_details)
        
        # 保存结果信息
        all_rounds_results.append({
            'round': round_num,
            'num_points': len(current_pick_poses),
            'T_camera2gripper': T_camera2gripper,
            'fixed_z': fixed_z,
            'mean_error': mean_error,
            'max_error': max_error,
            'min_error': min_error,
            'std_error': std_error,
            'errors': errors,
            'error_details': error_details
        })
        
        # 检查是否提前停止（如果误差不再改善）
        if CALIBRATION_CONFIG['stop_if_no_improvement'] and previous_mean_error is not None:
            improvement = (previous_mean_error - mean_error) / previous_mean_error
            if improvement < CALIBRATION_CONFIG['improvement_threshold']:
                print(f"\n误差改善不足 {CALIBRATION_CONFIG['improvement_threshold']*100:.1f}% "
                      f"（改善 {improvement*100:.2f}%），提前停止标定")
                break
        
        # 如果不是最后一轮，准备下一轮（剔除误差大的点）
        if round_num < CALIBRATION_CONFIG['num_rounds']:
            # 计算误差阈值
            error_threshold = mean_error + CALIBRATION_CONFIG['error_threshold_multiplier'] * std_error
            # 计算要剔除的点数
            n_points_to_remove = max(1, int(len(errors) * CALIBRATION_CONFIG['remove_percentage']))
            
            # 找出误差大的点
            error_with_indices = [(i, err) for i, err in enumerate(errors)]
            error_with_indices.sort(key=lambda x: x[1], reverse=True)
            
            # 确定要剔除的点
            indices_to_remove = []
            for i, err in error_with_indices:
                if err > error_threshold or len(indices_to_remove) < n_points_to_remove:
                    indices_to_remove.append(i)
            
            indices_to_remove = sorted(indices_to_remove, reverse=True)
            
            print(f"\n准备第 {round_num + 1} 轮标定，剔除策略:")
            print(f"  误差阈值: {error_threshold:.6f} mm")
            print(f"  计划剔除点数: {len(indices_to_remove)} 个（共 {len(errors)} 个点）")
            
            if len(indices_to_remove) > 0:
                print(f"  剔除的点:")
                for idx in indices_to_remove:
                    metadata = current_metadata[idx]
                    print(f"    索引 {idx}: {metadata['chessboard_pose']}/{metadata['json_filename']}, "
                          f"角点 {metadata['corner_index']}, 误差 {errors[idx]:.6f} mm")
                
                # 剔除误差大的点
                current_pick_poses = [current_pick_poses[i] for i in range(len(current_pick_poses)) 
                                     if i not in indices_to_remove]
                current_camera_points = [current_camera_points[i] for i in range(len(current_camera_points)) 
                                         if i not in indices_to_remove]
                current_metadata = [current_metadata[i] for i in range(len(current_metadata)) 
                                   if i not in indices_to_remove]
                
                print(f"\n第 {round_num + 1} 轮将使用 {len(current_pick_poses)} 个数据点（剔除了 {len(indices_to_remove)} 个）")
            else:
                print(f"  没有需要剔除的点，下一轮将使用相同的 {len(current_pick_poses)} 个数据点")
        
        previous_mean_error = mean_error
    
    # ========== 选择最佳结果并保存为最终结果 ==========
    print("\n" + "="*60)
    print("所有轮标定完成，选择最佳结果")
    print("="*60)
    
    if len(all_rounds_results) == 0:
        print("错误: 没有完成任何一轮标定")
        return
    
    # 选择平均误差最小的轮作为最佳结果
    best_round = min(all_rounds_results, key=lambda x: x['mean_error'])
    
    print(f"\n最佳结果：第 {best_round['round']} 轮")
    print(f"  使用数据点: {best_round['num_points']} 个")
    print(f"  平均误差: {best_round['mean_error']:.6f} mm")
    print(f"  最大误差: {best_round['max_error']:.6f} mm")
    print(f"  最小误差: {best_round['min_error']:.6f} mm")
    print(f"  标准差: {best_round['std_error']:.6f} mm")
    
    # 保存最终结果
    output_xml_final = os.path.join(output_dir, 'hand_eye_calibration.xml')
    output_csv_final = os.path.join(output_dir, 'calibration_error_evaluation.csv')
    
    print(f"\n保存最终标定结果（第 {best_round['round']} 轮）到: {output_xml_final}")
    save_calibration_result(
        output_xml_final, best_round['T_camera2gripper'], camera_matrix, dist_coeffs,
        image_size, best_round['errors'], best_round['error_details'], 'Eye-in-Hand'
    )
    
    print(f"保存最终误差评估到: {output_csv_final}")
    save_error_evaluation(output_csv_final, best_round['error_details'])
    
    # 打印所有轮的对比结果
    print("\n" + "="*60)
    print("所有轮标定结果对比")
    print("="*60)
    print(f"{'轮次':<6} {'数据点数':<10} {'平均误差(mm)':<15} {'最大误差(mm)':<15} {'最小误差(mm)':<15}")
    print("-" * 60)
    for result in all_rounds_results:
        marker = " ← 最佳" if result == best_round else ""
        print(f"{result['round']:<6} {result['num_points']:<10} "
              f"{result['mean_error']:<15.6f} {result['max_error']:<15.6f} "
              f"{result['min_error']:<15.6f}{marker}")
    
    if len(all_rounds_results) > 1:
        first_round = all_rounds_results[0]
        improvement = ((first_round['mean_error'] - best_round['mean_error']) / 
                      first_round['mean_error'] * 100)
        print(f"\n总误差改善: {improvement:.2f}% "
              f"(从 {first_round['mean_error']:.6f} mm 降至 {best_round['mean_error']:.6f} mm)")
    
    print("\n手眼标定完成！")


if __name__ == '__main__':
    main()

