#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Custom模式手眼标定模块
完全独立于OpenCV模式，包含数据采集、处理、算法计算、误差计算等全过程
基于XY+Z约束方法，参考hand_eye_calibration.py实现
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import cv2
import json
import time


class CustomHandEyeCalibration:
    """Custom模式手眼标定类"""
    
    # 角点质量检查阈值
    MAX_REPROJECTION_ERROR_PX = 2.0  # 最大重投影误差（像素）
    MAX_DEPTH_ERROR_PCT = 10.0  # 最大深度误差百分比
    MIN_DEPTH_MM = 300.0  # 最小深度值（毫米）
    MAX_DEPTH_MM = 2000.0  # 最大深度值（毫米）
    
    def __init__(self, logger=None):
        """
        初始化
        
        参数:
            logger: ROS2 logger对象，用于日志输出（可选）
        """
        self.logger = logger
    
    def _log(self, level, message):
        """日志输出 - Custom模式专用，使用[Custom模式]前缀"""
        prefix = '[Custom模式]'
        if self.logger:
            if level == 'info':
                self.logger.info(f'{prefix} {message}')
            elif level == 'warning':
                self.logger.warning(f'{prefix} {message}')
            elif level == 'error':
                self.logger.error(f'{prefix} {message}')
            elif level == 'debug':
                self.logger.debug(f'{prefix} {message}')
        else:
            print(f'[{level.upper()}] {prefix} {message}')
    
    def _check_corner_quality(self, corner, board_pose_data):
        """
        检查角点质量
        
        参数:
            corner: 角点数据字典，包含x, y, z, depth_valid等
            board_pose_data: 标定板位姿数据，包含reprojection_error等
        
        返回:
            (is_valid, reason): (是否有效, 无效原因)
        """
        # 检查1: depth_valid
        if not corner.get('depth_valid', False):
            return False, "深度数据无效"
        
        # 检查2: 重投影误差
        reprojection_error = board_pose_data.get('reprojection_error')
        if reprojection_error is not None and reprojection_error > self.MAX_REPROJECTION_ERROR_PX:
            return False, f"重投影误差过大({reprojection_error:.3f}px > {self.MAX_REPROJECTION_ERROR_PX}px)"
        
        # 检查3: 深度值范围
        z = corner.get('z', 0)
        if z < self.MIN_DEPTH_MM or z > self.MAX_DEPTH_MM:
            return False, f"深度值超出范围({z:.1f}mm, 有效范围: {self.MIN_DEPTH_MM}-{self.MAX_DEPTH_MM}mm)"
        
        # 检查4: 深度误差（如果有estimated_depth）
        estimated_depth = corner.get('estimated_depth_mm')
        if estimated_depth is not None and z > 0:
            depth_error_pct = abs(z - estimated_depth) / z * 100.0
            if depth_error_pct > self.MAX_DEPTH_ERROR_PCT:
                return False, f"深度误差过大({depth_error_pct:.1f}% > {self.MAX_DEPTH_ERROR_PCT}%)"
        
        return True, ""
    
    def _quaternion_to_rotation_matrix(self, quat):
        """
        将四元数转换为旋转矩阵
        quat: [x, y, z, w]
        """
        x, y, z, w = quat
        
        # 归一化四元数
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-6:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 构建旋转矩阵
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def _rotation_matrix_to_quaternion(self, R):
        """
        将旋转矩阵转换为四元数（兼容不同版本的scipy）
        返回: [x, y, z, w]
        """
        from scipy.spatial.transform import Rotation as R_scipy
        try:
            # 尝试使用from_matrix（新版本scipy）
            if hasattr(R_scipy, 'from_matrix'):
                r = R_scipy.from_matrix(R)
                return r.as_quat()  # [x, y, z, w]
            else:
                # 使用from_dcm（旧版本scipy）
                r = R_scipy.from_dcm(R)
                return r.as_quat()
        except (AttributeError, TypeError):
            # 如果scipy方法都失败，使用手动转换（Shepperd's method）
            trace = np.trace(R)
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w = 0.25 * s
                x = (R[2, 1] - R[1, 2]) / s
                y = (R[0, 2] - R[2, 0]) / s
                z = (R[1, 0] - R[0, 1]) / s
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    w = (R[2, 1] - R[1, 2]) / s
                    x = 0.25 * s
                    y = (R[0, 1] + R[1, 0]) / s
                    z = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    w = (R[0, 2] - R[2, 0]) / s
                    x = (R[0, 1] + R[1, 0]) / s
                    y = 0.25 * s
                    z = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    w = (R[1, 0] - R[0, 1]) / s
                    x = (R[0, 2] + R[2, 0]) / s
                    y = (R[1, 2] + R[2, 1]) / s
                    z = 0.25 * s
            return np.array([x, y, z, w])
    
    def _pose_to_transform_matrix(self, position, orientation_quat):
        """将位置和四元数组合成4x4齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = self._quaternion_to_rotation_matrix([
            orientation_quat['x'],
            orientation_quat['y'],
            orientation_quat['z'],
            orientation_quat['w']
        ])
        T[:3, 3] = [position['x'], position['y'], position['z']]
        return T
    
    def prepare_data(self, poses_data):
        """
        Custom模式：准备数据用于XY+Z约束方法
        直接接受位姿列表或运动组列表，转换为XY+Z约束方法需要的格式
        
        Custom模式的XY+Z约束方法使用：
        - shot_pose: 第一个位姿（固定的拍照姿态）
        - pick_poses: 所有位姿（包括第一个，用于点选验证）
        - camera_points_shot: 所有位姿对应的角点或标定板中心点
        
        参数:
            poses_data: 位姿列表或运动组列表
                - 如果为位姿列表：每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                - 如果为运动组列表（向后兼容）：每个运动组包含pose1, pose2, board_pose1, board_pose2
                会自动检测格式并进行转换
        
        返回:
            shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
            pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
            camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
        """
        # 自动检测输入格式：位姿列表还是运动组
        if len(poses_data) > 0 and 'pose1' in poses_data[0]:
            # 旧格式：运动组列表，转换为位姿列表
            self._log('info', f'检测到运动组格式，正在转换为位姿列表：{len(poses_data)} 个运动组')
            poses_list = []
            for motion in poses_data:
                # 从运动组中提取pose1和pose2
                if 'pose1' in motion and 'board_pose1' in motion:
                    poses_list.append({
                        'robot_pose': motion['pose1'],
                        'board_pose': motion['board_pose1']
                    })
                if 'pose2' in motion and 'board_pose2' in motion:
                    poses_list.append({
                        'robot_pose': motion['pose2'],
                        'board_pose': motion['board_pose2']
                    })
            poses_data = poses_list
            self._log('info', f'已转换为位姿列表：{len(poses_data)} 个位姿')
        else:
            # 新格式：位姿列表
            self._log('info', f'检测到位姿列表格式：{len(poses_data)} 个位姿')
        
        if len(poses_data) == 0:
            raise ValueError('[Custom模式] 没有位姿数据')
        
        # 提取shot_pose（第一个位姿，作为固定的拍照姿态）
        first_pose = poses_data[0]
        robot_pose_data = first_pose.get('robot_pose')
        if not robot_pose_data:
            raise ValueError('[Custom模式] 第一个位姿缺少robot_pose数据')
        
        shot_pose = self._pose_to_transform_matrix(
            {'x': robot_pose_data.get('robot_pos_x', 0.0) * 1000.0, 'y': robot_pose_data.get('robot_pos_y', 0.0) * 1000.0, 'z': robot_pose_data.get('robot_pos_z', 0.0) * 1000.0},
            {'x': robot_pose_data.get('robot_ori_x', 0.0), 'y': robot_pose_data.get('robot_ori_y', 0.0), 'z': robot_pose_data.get('robot_ori_z', 0.0), 'w': robot_pose_data.get('robot_ori_w', 1.0)}
        )
        
        # 提取pick_poses和camera_points_shot
        # 使用所有采集的角点，每个角点对应一个数据点
        # 直接使用位姿列表，每个位姿对应一个数据点
        pick_poses = []
        camera_points_shot = []
        
        total_corners_count = 0
        skipped_poses = 0
        
        for idx, pose_data in enumerate(poses_data):
            robot_pose_data = pose_data.get('robot_pose')
            board_pose_data = pose_data.get('board_pose')
            
            if not robot_pose_data or not board_pose_data:
                self._log('warning', f'位姿 #{idx+1} 数据不完整，已跳过')
                skipped_poses += 1
                continue
            
            # 处理当前位姿
            pick_pose = self._pose_to_transform_matrix(
                {'x': robot_pose_data.get('robot_pos_x', 0.0) * 1000.0, 'y': robot_pose_data.get('robot_pos_y', 0.0) * 1000.0, 'z': robot_pose_data.get('robot_pos_z', 0.0) * 1000.0},
                {'x': robot_pose_data.get('robot_ori_x', 0.0), 'y': robot_pose_data.get('robot_ori_y', 0.0), 'z': robot_pose_data.get('robot_ori_z', 0.0), 'w': robot_pose_data.get('robot_ori_w', 1.0)}
            )
            
            corners_3d_filtered = board_pose_data.get('corners_3d_filtered', [])
            valid_corners_in_pose = 0
            depth_valid_count = 0
            depth_invalid_count = 0
            
            # #region agent log - 检查corners_3d_filtered状态
            log_entry_check = {
                'sessionId': 'debug-session',
                'runId': 'custom-prepare-data',
                'hypothesisId': 'A,B',
                'location': 'custom_hand_eye_calibration.py:prepare_data',
                'message': '检查corners_3d_filtered状态',
                'data': {
                    'pose_idx': idx + 1,
                    'has_corners_3d_filtered': corners_3d_filtered is not None and len(corners_3d_filtered) > 0,
                    'corners_3d_filtered_count': len(corners_3d_filtered) if corners_3d_filtered else 0
                },
                'timestamp': int(time.time() * 1000)
            }
            try:
                with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps(log_entry_check) + '\n')
            except: pass
            # #endregion
            
            if corners_3d_filtered and len(corners_3d_filtered) > 0:
                # 使用所有滤波后的角点，并进行质量检查
                for corner in corners_3d_filtered:
                    # 角点质量检查
                    is_valid, reason = self._check_corner_quality(corner, board_pose_data)
                    if not is_valid:
                        depth_invalid_count += 1
                        self._log('debug', f'位姿 #{idx+1} 角点被过滤: {reason}')
                        continue
                    
                    depth_valid_count += 1
                    camera_point = np.array([
                        corner['x'],
                        corner['y'],
                        corner['z']
                    ])
                    
                    pick_poses.append(pick_pose)
                    camera_points_shot.append(camera_point)
                    valid_corners_in_pose += 1
                    total_corners_count += 1
                    
                    # #region agent log - 数据准备：记录每个角点的详细信息
                    log_entry = {
                        'sessionId': 'debug-session',
                        'runId': 'custom-prepare-data',
                        'hypothesisId': 'A,B',
                        'location': 'custom_hand_eye_calibration.py:prepare_data',
                        'message': '角点数据记录',
                        'data': {
                            'pose_idx': idx + 1,
                            'corner_idx': valid_corners_in_pose,
                            'camera_point_mm': camera_point.tolist(),
                            'robot_pose_mm': pick_pose[:3, 3].tolist(),
                            'robot_rotation_det': float(np.linalg.det(pick_pose[:3, :3])),
                            'corner_quality_valid': is_valid,
                            'corner_depth_valid': corner.get('depth_valid', False),
                            'reprojection_error': board_pose_data.get('reprojection_error')
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    try:
                        with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                            f.write(json.dumps(log_entry) + '\n')
                    except: pass
                    # #endregion
                
                # 如果所有角点都未通过质量检查，回退到使用board_pose position
                if valid_corners_in_pose == 0:
                    self._log('warning', f'位姿 #{idx+1}: 所有{len(corners_3d_filtered)}个角点未通过质量检查，回退到使用board_pose position')
            else:
                # 如果没有滤波后的角点，使用board_pose的position（向后兼容，只使用一个点）
                camera_point = np.array([
                    board_pose_data.get('position', {}).get('x', 0.0),
                    board_pose_data.get('position', {}).get('y', 0.0),
                    board_pose_data.get('position', {}).get('z', 0.0)
                ])
                pick_poses.append(pick_pose)
                camera_points_shot.append(camera_point)
                total_corners_count += 1
                valid_corners_in_pose = 1
                
                # #region agent log - 数据准备：使用board_pose position（向后兼容）
                log_entry = {
                    'sessionId': 'debug-session',
                    'runId': 'custom-prepare-data',
                    'hypothesisId': 'A,B',
                    'location': 'custom_hand_eye_calibration.py:prepare_data',
                    'message': '使用board_pose position（向后兼容）',
                    'data': {
                        'pose_idx': idx + 1,
                        'camera_point_mm': camera_point.tolist(),
                        'robot_pose_mm': pick_pose[:3, 3].tolist(),
                        'reprojection_error': board_pose_data.get('reprojection_error')
                    },
                    'timestamp': int(time.time() * 1000)
                }
                try:
                    with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                        f.write(json.dumps(log_entry) + '\n')
                except: pass
                # #endregion
            
            if valid_corners_in_pose > 0:
                self._log('info', f'位姿 #{idx+1}: 使用{valid_corners_in_pose}个有效角点')
        
        self._log('info', f'数据准备完成：shot_pose(1个), pick_poses({len(pick_poses)}个), camera_points_shot({len(camera_points_shot)}个)')
        self._log('info', f'共使用{total_corners_count}个角点数据点，跳过{skipped_poses}个无效位姿')
        
        if len(pick_poses) < 3:
            raise ValueError(f'有效数据不足，至少需要3个数据点，当前只有{len(pick_poses)}个')
        
        return shot_pose, pick_poses, camera_points_shot
    
    def solve_hand_eye(self, shot_pose, pick_poses, camera_points_shot, fixed_z=None):
        """
        Eye-in-Hand手眼标定核心算法
        基于hand_eye_calibration.py中的实现（XY+Z约束方法）
        
        参数:
            shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
            pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
            camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
            fixed_z: 固定Z值（可选，如果为None则自动计算）
        
        返回:
            T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
            fixed_z: 固定Z值（棋盘格桌面高度，单位：毫米）
        """
        n = len(pick_poses)
        
        # 辅助函数：兼容不同版本的scipy Rotation.as_matrix()
        def rotation_to_matrix(r):
            """将Rotation对象转换为矩阵（兼容不同版本scipy）"""
            if hasattr(r, 'as_matrix'):
                return r.as_matrix()
            elif hasattr(r, 'as_dcm'):
                return r.as_dcm()
            else:
                # 手动转换（使用Rodrigues公式）
                rotvec = r.as_rotvec()
                angle = np.linalg.norm(rotvec)
                if angle < 1e-6:
                    return np.eye(3)
                axis = rotvec / angle
                K = np.array([[0, -axis[2], axis[1]],
                              [axis[2], 0, -axis[0]],
                              [-axis[1], axis[0], 0]])
                return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        
        # 计算Z值（棋盘格桌面高度）
        pick_z_values = [T_pick[2, 3] for T_pick in pick_poses]
        if fixed_z is None:
            fixed_z = np.mean(pick_z_values)
            self._log('info', f'平均Z值（棋盘格桌面高度）: {fixed_z:.6f} mm')
            self._log('info', f'Z值范围: {np.min(pick_z_values):.6f} ~ {np.max(pick_z_values):.6f} mm')
        
        # 优化目标函数
        def residuals(params):
            """计算残差"""
            axis_angle = params[:3]
            translation = params[3:6]
            
            # 构建旋转矩阵
            angle = np.linalg.norm(axis_angle)
            if angle < 1e-6:
                R_mat = np.eye(3)
            else:
                axis = axis_angle / angle
                r = R.from_rotvec(axis * angle)
                R_mat = rotation_to_matrix(r)
            
            # 构建变换矩阵 T_camera2gripper
            T_camera2gripper = np.eye(4)
            T_camera2gripper[:3, :3] = R_mat
            T_camera2gripper[:3, 3] = translation
            
            residuals_list = []
            
            for i in range(n):
                T_gripper_pick2base = pick_poses[i]
                P_camera_shot = camera_points_shot[i]
                
                # 从拍照姿态和手眼标定矩阵计算角点在基座坐标系下的位置
                P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
                P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
                P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
                P_base_computed = P_base_computed_homogeneous[:3]
                
                # 从点选姿态得到角点在基座坐标系下的实际位置
                P_base_actual = T_gripper_pick2base[:3, 3]
                
                # XY+Z约束
                residual = P_base_computed - P_base_actual
                residuals_list.extend(residual)
            
            return np.array(residuals_list)
        
        # 初始估计：使用SVD方法
        T_gripper_shot2base_inv = np.linalg.inv(shot_pose)
        
        points_gripper = []
        points_camera = []
        
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
        
        # SVD方法求解初始估计
        centroid_gripper = np.mean(points_gripper, axis=0)
        centroid_camera = np.mean(points_camera, axis=0)
        
        points_gripper_centered = points_gripper - centroid_gripper
        points_camera_centered = points_camera - centroid_camera
        
        H = points_camera_centered.T @ points_gripper_centered
        
        U, S, Vt = np.linalg.svd(H)
        
        R_init = Vt.T @ U.T
        
        if np.linalg.det(R_init) < 0:
            Vt[-1, :] *= -1
            R_init = Vt.T @ U.T
        
        t_init = centroid_gripper - R_init @ centroid_camera
        
        # 使用安全的转换函数
        quat_init = self._rotation_matrix_to_quaternion(R_init)
        # 四元数转轴角
        from scipy.spatial.transform import Rotation as R_scipy
        try:
            if hasattr(R_scipy, 'from_quat'):
                r_init = R_scipy.from_quat(quat_init)
                axis_angle_init = r_init.as_rotvec()
            else:
                # 手动转换四元数到轴角
                w = quat_init[3]
                if abs(w) >= 1.0:
                    axis_angle_init = np.array([0.0, 0.0, 0.0])
                else:
                    angle = 2 * np.arccos(abs(w))
                    if w < 0:
                        axis = -quat_init[:3] / np.sin(angle / 2)
                    else:
                        axis = quat_init[:3] / np.sin(angle / 2)
                    axis_angle_init = axis * angle
        except Exception as e:
            self._log('error', f'四元数转轴角失败: {e}')
            # 使用Rodrigues公式的逆变换
            axis_angle_init, _ = cv2.Rodrigues(R_init)
            axis_angle_init = axis_angle_init.flatten()
        
        initial_params = np.concatenate([axis_angle_init, t_init])
        
        # #region agent log - 优化前：记录初始估计
        initial_residuals = residuals(initial_params)
        log_entry = {
            'sessionId': 'debug-session',
            'runId': 'custom-solve',
            'hypothesisId': 'C,D',
            'location': 'custom_hand_eye_calibration.py:solve_hand_eye',
            'message': '优化前初始估计',
            'data': {
                'num_data_points': n,
                'R_init_det': float(np.linalg.det(R_init)),
                't_init_mm': t_init.tolist(),
                'initial_mean_residual_mm': float(np.mean(np.abs(initial_residuals))),
                'initial_max_residual_mm': float(np.max(np.abs(initial_residuals))),
                'T_camera2gripper_init': {
                    'R': R_init.tolist(),
                    't_mm': t_init.tolist()
                }
            },
            'timestamp': int(time.time() * 1000)
        }
        try:
            with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
        except: pass
        # #endregion
        
        # 调试：检查初始估计的残差
        initial_residuals = residuals(initial_params)
        self._log('info', f'初始估计完成，开始优化求解...')
        self._log('info', f'初始残差统计: 平均 {np.mean(np.abs(initial_residuals)):.6f} mm, 最大 {np.max(np.abs(initial_residuals)):.6f} mm')
        
        # 优化求解
        self._log('info', f'开始优化求解手眼标定（使用XY+Z约束）...')
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
            R_camera2gripper = rotation_to_matrix(r)
        
        # 构建变换矩阵
        T_camera2gripper = np.eye(4)
        T_camera2gripper[:3, :3] = R_camera2gripper
        T_camera2gripper[:3, 3] = translation
        
        mean_residual = np.mean(np.abs(result.fun))
        self._log('info', f'优化完成，平均残差: {mean_residual:.6f} mm')
        
        # #region agent log - 优化后：记录最终结果
        log_entry = {
            'sessionId': 'debug-session',
            'runId': 'custom-solve',
            'hypothesisId': 'C,D',
            'location': 'custom_hand_eye_calibration.py:solve_hand_eye',
            'message': '优化后最终结果',
            'data': {
                'num_data_points': n,
                'optimization_success': result.success,
                'optimization_status': result.status,
                'num_iterations': result.nfev,
                'final_mean_residual_mm': float(mean_residual),
                'final_max_residual_mm': float(np.max(np.abs(result.fun))),
                'R_final_det': float(np.linalg.det(R_camera2gripper)),
                'T_camera2gripper_final': {
                    'R': R_camera2gripper.tolist(),
                    't_mm': translation.tolist()
                },
                'residuals_by_point_mm': [float(np.linalg.norm(result.fun[i*3:(i+1)*3])) for i in range(n)]
            },
            'timestamp': int(time.time() * 1000)
        }
        try:
            with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
        except: pass
        # #endregion
        
        return T_camera2gripper, fixed_z
    
    def calculate_errors(self, shot_pose, pick_poses, camera_points_shot, T_camera2gripper):
        """
        Custom模式：计算标定误差（XY+Z约束的误差）
        
        参数:
            shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
            pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
            camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
            T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
        
        返回:
            errors: 列表，每个数据点的误差（单位：毫米）
            error_statistics: 字典，包含RMS、最大、最小、标准差等统计信息
        """
        self._log('info', f'开始误差计算：验证XY+Z约束')
        
        errors = []
        error_details = []
        
        for i in range(len(pick_poses)):
            P_camera_shot = camera_points_shot[i]
            T_gripper_pick2base = pick_poses[i]
            
            # 计算角点在基座坐标系下的位置
            P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
            P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
            P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
            P_base_computed = P_base_computed_homogeneous[:3]
            
            # 实际位置（点选姿态的末端执行器位置）
            P_base_actual = T_gripper_pick2base[:3, 3]
            
            # 计算误差
            error = np.linalg.norm(P_base_computed - P_base_actual)
            error_xyz = P_base_computed - P_base_actual
            errors.append(error)
            
            error_details.append({
                'data_point_idx': i + 1,
                'error_mm': float(error),
                'error_x_mm': float(error_xyz[0]),
                'error_y_mm': float(error_xyz[1]),
                'error_z_mm': float(error_xyz[2]),
                'computed_position': P_base_computed.tolist(),
                'actual_position': P_base_actual.tolist(),
                'camera_point': P_camera_shot.tolist()
            })
            
            # #region agent log - 误差计算：详细记录每个数据点的误差分解
            log_entry = {
                'sessionId': 'debug-session',
                'runId': 'custom-errors',
                'hypothesisId': 'E',
                'location': 'custom_hand_eye_calibration.py:calculate_errors',
                'message': '数据点误差详细分析',
                'data': {
                    'data_point_idx': i + 1,
                    'error_total_mm': float(error),
                    'error_x_mm': float(error_xyz[0]),
                    'error_y_mm': float(error_xyz[1]),
                    'error_z_mm': float(error_xyz[2]),
                    'computed_position_mm': P_base_computed.tolist(),
                    'actual_position_mm': P_base_actual.tolist(),
                    'camera_point_mm': P_camera_shot.tolist(),
                    'robot_pose_mm': T_gripper_pick2base[:3, 3].tolist()
                },
                'timestamp': int(time.time() * 1000)
            }
            try:
                with open('/home/mu/IVG/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps(log_entry) + '\n')
            except: pass
            # #endregion
        
        # 误差统计
        mean_error = float(np.mean(errors))
        max_error = float(np.max(errors))
        min_error = float(np.min(errors))
        std_error = float(np.std(errors))
        
        error_statistics = {
            'mean_error': mean_error,
            'max_error': max_error,
            'min_error': min_error,
            'std_error': std_error,
            'errors': [float(e) for e in errors],
            'error_details': error_details
        }
        
        self._log('info', f'误差计算完成：平均 {mean_error:.3f} mm, 最大 {max_error:.3f} mm, 最小 {min_error:.3f} mm, 标准差 {std_error:.3f} mm')
        
        return errors, error_statistics
    
    def calibrate(self, poses_data):
        """
        Custom模式完整标定流程
        
        参数:
            poses_data: 位姿列表或运动组列表
                - 如果为位姿列表：每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                - 如果为运动组列表（向后兼容）：每个运动组包含pose1, pose2, board_pose1, board_pose2
                会自动检测格式并进行转换
        
        返回:
            dict: 包含标定结果和误差统计的字典
        """
        # 步骤1：数据准备
        shot_pose, pick_poses, camera_points_shot = self.prepare_data(poses_data)
        
        if len(pick_poses) < 3:
            raise ValueError(f'有效数据不足，至少需要3组，当前只有{len(pick_poses)}组')
        
        # 步骤2：算法计算
        self._log('info', f'开始算法计算（XY+Z约束方法）')
        self._log('info', f'输入数据：{len(pick_poses)} 个数据点')
        T_camera2gripper, fixed_z = self.solve_hand_eye(shot_pose, pick_poses, camera_points_shot)
        self._log('info', f'算法计算完成，固定Z值: {fixed_z:.6f} mm')
        
        # 步骤3：误差计算
        errors, error_statistics = self.calculate_errors(shot_pose, pick_poses, camera_points_shot, T_camera2gripper)
        
        return {
            'T_camera2gripper': T_camera2gripper,
            'fixed_z': fixed_z,
            'errors': errors,
            'error_statistics': error_statistics,
            'shot_pose': shot_pose,
            'pick_poses': pick_poses,
            'camera_points_shot': camera_points_shot
        }
