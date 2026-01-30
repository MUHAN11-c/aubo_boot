#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenCV模式手眼标定模块
完全独立于Custom模式，包含数据处理、算法计算、误差计算等功能
基于OpenCV calibrateHandEye方法（TSAI算法）

注意：
- 数据采集流程在前端JavaScript代码中实现（script_v2_auto_calib_addon.js）
- OpenCV模式的自动采集流程中，位姿到位后延迟3秒，采集图像后再延迟2秒
- 本模块仅负责接收已采集的数据并执行标定计算
"""

import numpy as np
import cv2
import json
import time
import os
from datetime import datetime
import os
from datetime import datetime


def _convert_to_json_serializable(obj):
    """将NumPy类型转换为JSON可序列化的Python原生类型，处理NaN和Infinity"""
    if isinstance(obj, np.bool_):
        return bool(obj)
    elif isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        val = float(obj)
        # JSON不支持NaN和Infinity，转换为null
        if np.isnan(val) or np.isinf(val):
            return None
        return val
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: _convert_to_json_serializable(value) for key, value in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [_convert_to_json_serializable(item) for item in obj]
    elif isinstance(obj, float):
        # 处理Python原生float类型的NaN和Infinity
        if np.isnan(obj) or np.isinf(obj):
            return None
        return obj
    else:
        return obj


class OpenCVHandEyeCalibration:
    """OpenCV模式手眼标定类"""
    
    def __init__(self, logger=None):
        """
        初始化
        
        参数:
            logger: ROS2 logger对象，用于日志输出（可选）
        """
        self.logger = logger
        
        # 获取config文件夹路径
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('hand_eye_calibration')
            self.config_dir = os.path.join(package_share, 'config')
        except:
            # 如果无法获取package_share，使用相对路径
            self.config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
        
        # 确保config文件夹存在
        os.makedirs(self.config_dir, exist_ok=True)
        
        # 当前标定会话ID（时间戳）
        self.session_id = int(time.time() * 1000)  # 毫秒时间戳
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    def _log(self, level, message):
        """日志输出 - OpenCV模式专用，使用[OpenCV模式]前缀"""
        prefix = '[OpenCV模式]'
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
        OpenCV模式：准备数据用于calibrateHandEye
        直接接受位姿列表，转换为OpenCV需要的格式
        
        OpenCV的calibrateHandEye只需要位姿列表，内部会自动计算相邻位姿之间的运动（A和B）
        
        参数:
            poses_data: 位姿列表，每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                - robot_pose: 机器人位姿，包含robot_pos_x/y/z（单位：米）和robot_ori_x/y/z/w（四元数）
                - board_pose: 标定板位姿，包含position（x/y/z，单位：毫米）和orientation（x/y/z/w，四元数）
        
        返回:
            R_gripper2base: (N, 3, 3) numpy数组，机器人旋转矩阵列表
            t_gripper2base: 列表，每个元素(3,1)数组，机器人平移向量（单位：米）
            rvecs: 列表，每个元素(3,1)数组，标定板旋转向量（单位：弧度）
            tvecs: 列表，每个元素(3,1)数组，标定板平移向量（单位：米）
            T_gripper_list: 列表，机器人变换矩阵（单位：毫米，用于误差计算）
            T_board_list: 列表，标定板变换矩阵（单位：毫米，用于误差计算）
        """
        self._log('info', f'开始数据准备：输入 {len(poses_data)} 个位姿')
        
        # #region agent log - 记录输入数据概览
        input_summary = {
            'total_poses': len(poses_data),
            'poses_detail': []
        }
        for idx, pose_data in enumerate(poses_data[:5]):  # 只记录前5个位姿的详细信息
            robot_pose = pose_data.get('robot_pose', {})
            board_pose = pose_data.get('board_pose', {})
            input_summary['poses_detail'].append({
                'pose_idx': idx + 1,
                'has_robot_pose': bool(robot_pose),
                'has_board_pose': bool(board_pose),
                'robot_pos_m': {
                    'x': robot_pose.get('robot_pos_x', 0.0) if robot_pose else None,
                    'y': robot_pose.get('robot_pos_y', 0.0) if robot_pose else None,
                    'z': robot_pose.get('robot_pos_z', 0.0) if robot_pose else None
                } if robot_pose else None,
                'board_pos_mm': {
                    'x': board_pose.get('position', {}).get('x', 0.0) if board_pose else None,
                    'y': board_pose.get('position', {}).get('y', 0.0) if board_pose else None,
                    'z': board_pose.get('position', {}).get('z', 0.0) if board_pose else None
                } if board_pose else None
            })
        
        log_entry_input = {
            'sessionId': 'debug-session',
            'runId': 'check-transform-A',
            'hypothesisId': 'CHECK_TRANSFORM_A',
            'location': 'opencv_hand_eye_calibration.py:prepare_data',
            'message': '[检查变换矩阵A] 输入位姿数据概览',
            'data': input_summary,
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_input)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        # 提取所有唯一姿态数据用于OpenCV（去重处理）
        T_gripper_list = []  # 保持顺序的唯一姿态列表（单位：毫米）
        T_board_list = []    # 保持顺序的唯一标定板姿态列表（单位：毫米）
        board_rvec_tvec_map = {}  # 保存每个T_board对应的原始rvec/tvec数据
        seen_poses = set()   # 用于快速检查是否已存在
        
        # 姿态去重精度：0.1mm（可配置）
        pose_key_epsilon = 0.1  # mm
        
        def pose_key(T):
            """生成姿态的唯一标识（用于去重）"""
            return tuple(np.round(T.flatten() / pose_key_epsilon) * pose_key_epsilon)
        
        # 统计信息
        processed_poses = 0
        skipped_poses = 0
        duplicate_poses = 0
        
        for idx, pose_data in enumerate(poses_data):
            try:
                robot_pose_data = pose_data.get('robot_pose')
                board_pose_data = pose_data.get('board_pose')
                
                if not robot_pose_data or not board_pose_data:
                    skipped_poses += 1
                    self._log('warning', f'位姿 #{idx+1} 数据不完整，已跳过')
                    continue
                
                # #region agent log - 检查末端到基座的变换矩阵构建
                # 记录输入的机器人位姿数据（单位：米）
                robot_pos_m = {
                    'x': robot_pose_data.get('robot_pos_x', 0.0),
                    'y': robot_pose_data.get('robot_pos_y', 0.0),
                    'z': robot_pose_data.get('robot_pos_z', 0.0)
                }
                robot_ori = {
                    'x': robot_pose_data.get('robot_ori_x', 0.0),
                    'y': robot_pose_data.get('robot_ori_y', 0.0),
                    'z': robot_pose_data.get('robot_ori_z', 0.0),
                    'w': robot_pose_data.get('robot_ori_w', 1.0)
                }
                
                log_entry_robot_pose_input = {
                    'sessionId': 'debug-session',
                    'runId': 'check-gripper-transform',
                    'hypothesisId': 'CHECK_GRIPPER_TRANSFORM',
                    'location': 'opencv_hand_eye_calibration.py:prepare_data',
                    'message': '[检查变换矩阵] 输入的机器人位姿数据（米）',
                    'data': {
                        'pose_idx': idx + 1,
                        'position_m': robot_pos_m,
                        'position_mm': {k: v * 1000.0 for k, v in robot_pos_m.items()},
                        'orientation': robot_ori,
                        'quaternion_norm': float(np.sqrt(robot_ori['x']**2 + robot_ori['y']**2 + robot_ori['z']**2 + robot_ori['w']**2)),
                        'note': 'robot_pos_x/y/z单位是米，需要*1000转换为毫米'
                    },
                    'timestamp': int(time.time() * 1000)
                }
                log_entry_clean = _convert_to_json_serializable(log_entry_robot_pose_input)
                try:
                    with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                        f.write(json.dumps(log_entry_clean) + '\n')
                except: pass
                # #endregion
                
                # 构建变换矩阵（将机器人位姿从米转换为毫米）
                T_gripper = self._pose_to_transform_matrix(
                    {'x': robot_pos_m['x'] * 1000.0, 'y': robot_pos_m['y'] * 1000.0, 'z': robot_pos_m['z'] * 1000.0},
                    robot_ori
                )
                
                # #region agent log - 检查构建的变换矩阵
                # 验证构建的变换矩阵
                T_gripper_t_mm = T_gripper[:3, 3].flatten()
                T_gripper_R = T_gripper[:3, :3]
                T_gripper_R_det = float(np.linalg.det(T_gripper_R))
                T_gripper_R_is_orthogonal = np.allclose(T_gripper_R @ T_gripper_R.T, np.eye(3), atol=1e-3)
                
                # 验证单位转换是否正确
                pos_expected_mm = np.array([robot_pos_m['x'] * 1000.0, robot_pos_m['y'] * 1000.0, robot_pos_m['z'] * 1000.0])
                pos_actual_mm = T_gripper_t_mm
                pos_diff_mm = np.abs(pos_expected_mm - pos_actual_mm)
                pos_conversion_ok = np.all(pos_diff_mm < 0.01)  # 允许0.01mm误差
                
                log_entry_transform_matrix = {
                    'sessionId': 'debug-session',
                    'runId': 'check-gripper-transform',
                    'hypothesisId': 'CHECK_GRIPPER_TRANSFORM',
                    'location': 'opencv_hand_eye_calibration.py:prepare_data',
                    'message': '[检查变换矩阵] 构建的T_gripper变换矩阵（毫米）',
                    'data': {
                        'pose_idx': idx + 1,
                        'T_gripper': {
                            'matrix': T_gripper.tolist(),
                            'translation_mm': T_gripper_t_mm.tolist(),
                            'translation_m': (T_gripper_t_mm / 1000.0).tolist(),
                            'rotation_matrix': T_gripper_R.tolist(),
                            'rotation_det': T_gripper_R_det,
                            'rotation_is_orthogonal': bool(T_gripper_R_is_orthogonal),
                            'input_position_m': robot_pos_m,
                            'input_position_mm': {k: v * 1000.0 for k, v in robot_pos_m.items()},
                            'expected_position_mm': pos_expected_mm.tolist(),
                            'actual_position_mm': pos_actual_mm.tolist(),
                            'position_diff_mm': pos_diff_mm.tolist(),
                            'position_conversion_ok': bool(pos_conversion_ok),
                            'input_orientation': robot_ori
                        },
                        'validation': {
                            'T_gripper_valid': bool(pos_conversion_ok and abs(T_gripper_R_det - 1.0) < 0.01 and T_gripper_R_is_orthogonal),
                            'issues': []
                        }
                    },
                    'timestamp': int(time.time() * 1000)
                }
                
                # 检查是否有问题
                if not pos_conversion_ok:
                    log_entry_transform_matrix['data']['validation']['issues'].append('T_gripper位置转换错误')
                if abs(T_gripper_R_det - 1.0) > 0.01:
                    log_entry_transform_matrix['data']['validation']['issues'].append(f'T_gripper旋转矩阵行列式异常: {T_gripper_R_det:.6f}')
                if not T_gripper_R_is_orthogonal:
                    log_entry_transform_matrix['data']['validation']['issues'].append('T_gripper旋转矩阵不正交')
                
                log_entry_clean = _convert_to_json_serializable(log_entry_transform_matrix)
                try:
                    with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                        f.write(json.dumps(log_entry_clean) + '\n')
                except: pass
                # #endregion
                
                # 标定板位姿（已经是毫米单位）
                T_board = self._pose_to_transform_matrix(
                    {'x': board_pose_data.get('position', {}).get('x', 0.0), 
                     'y': board_pose_data.get('position', {}).get('y', 0.0), 
                     'z': board_pose_data.get('position', {}).get('z', 0.0)},
                    {'x': board_pose_data.get('orientation', {}).get('x', 0.0), 
                     'y': board_pose_data.get('orientation', {}).get('y', 0.0), 
                     'z': board_pose_data.get('orientation', {}).get('z', 0.0), 
                     'w': board_pose_data.get('orientation', {}).get('w', 1.0)}
                )
                
                # 保存姿态（基于机器人位姿+标定板位姿的组合去重，确保数据匹配）
                key_gripper = pose_key(T_gripper)
                key_board = pose_key(T_board)
                key_combined = (key_gripper, key_board)  # 组合键
                
                # 检查组合键（机器人位姿+标定板位姿）
                if key_combined not in seen_poses:
                    seen_poses.add(key_combined)
                    T_gripper_list.append(T_gripper)
                    T_board_list.append(T_board)
                    # 保存对应的原始board_pose数据，用于后续提取rvec/tvec
                    board_key = pose_key(T_board)
                    if 'rvec' in board_pose_data and 'tvec' in board_pose_data:
                        board_rvec_tvec_map[board_key] = {
                            'rvec': board_pose_data['rvec'],
                            'tvec': board_pose_data['tvec']
                        }
                    processed_poses += 1
                else:
                    duplicate_poses += 1
                    # #region agent log - 记录重复位姿
                    T_board_t_mm = T_board[:3, 3].flatten()
                    log_entry_duplicate = {
                        'sessionId': 'debug-session',
                        'runId': 'check-transform-A',
                        'hypothesisId': 'CHECK_TRANSFORM_A',
                        'location': 'opencv_hand_eye_calibration.py:prepare_data',
                        'message': '[检查变换矩阵A] 发现重复位姿',
                        'data': {
                            'pose_idx': idx + 1,
                            'robot_pos_mm': T_gripper_t_mm.tolist(),
                            'board_pos_mm': T_board_t_mm.tolist(),
                            'note': '该位姿已存在，已跳过'
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    log_entry_clean = _convert_to_json_serializable(log_entry_duplicate)
                    try:
                        with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                            f.write(json.dumps(log_entry_clean) + '\n')
                    except: pass
                    # #endregion
                
            except Exception as e:
                skipped_poses += 1
                self._log('warning', f'位姿 #{idx+1} 处理失败: {str(e)}')
                continue
        
        self._log('info', f'数据准备完成：处理{processed_poses}个位姿，跳过{skipped_poses}个，去重后{len(T_gripper_list)}个唯一姿态对（去除{duplicate_poses}个重复姿态对）')
        
        # #region agent log - 记录去重结果
        log_entry_dedup = {
            'sessionId': 'debug-session',
            'runId': 'check-transform-A',
            'hypothesisId': 'CHECK_TRANSFORM_A',
            'location': 'opencv_hand_eye_calibration.py:prepare_data',
            'message': '[检查变换矩阵A] 去重结果统计',
            'data': {
                'input_poses': len(poses_data),
                'processed_poses': processed_poses,
                'skipped_poses': skipped_poses,
                'unique_poses': len(T_gripper_list),
                'duplicate_poses': duplicate_poses,
                'dedup_rate': float(duplicate_poses) / len(poses_data) if len(poses_data) > 0 else 0.0
            },
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_dedup)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        if len(T_gripper_list) < 4:
            raise ValueError(f'有效姿态数据不足，至少需要4个姿态，当前只有{len(T_gripper_list)}个')
        
        
        # 坐标系方向（已通过参考代码hand_in_eye_calibrate.py验证）
        # MoveIt2 API: getGlobalLinkTransform() 返回 Base→Gripper
        # OpenCV实际需要: Base→Gripper（不要取逆！虽然参数名叫R_gripper2base但实际期望Base→Gripper）
        # 参考: hand_in_eye_calibrate.py 直接传入pose转换的R和t，没有取逆操作
        self._log('info', '坐标系: Base→Gripper (MoveIt2), 直接传给OpenCV（不取逆）')
        
        # 转换为OpenCV需要的格式
        # 注意: OpenCV参数名R_gripper2base容易误导，实际期望Base→Gripper
        R_gripper2base_list = []  # 实际存储Base→Gripper
        t_gripper2base_list = []  # 实际存储Base→Gripper
        rvecs_list = []  # 旋转向量列表（用于OpenCV calibrateHandEye）
        tvecs_list = []  # 平移向量列表（用于OpenCV calibrateHandEye）
        
        # 从motion_groups中提取rvec和tvec（如果存在）
        rvec_found_count = 0
        rvec_converted_count = 0
        tvec_found_count = 0
        tvec_converted_count = 0
        
        # #region agent log - 检查从T_gripper提取的数据
        # 记录从T_gripper提取的数据，用于验证单位转换
        extraction_detail = []
        # #endregion
        
        for idx, (T_gripper, T_board) in enumerate(zip(T_gripper_list, T_board_list)):
            # T_gripper是Base→Gripper，直接使用（参考hand_in_eye_calibrate.py，不要取逆！）
            R_base2gripper = T_gripper[:3, :3]
            t_base2gripper_mm = T_gripper[:3, 3].flatten()  # 单位：毫米
            
            R_gripper2base_list.append(R_base2gripper)  # 变量名保持不变以兼容后续代码
            t_gripper2base_list.append(t_base2gripper_mm)
            
            # #region agent log - 检查提取的数据
            # 记录每个姿态的提取数据
            # 为了日志记录，需要原始T_gripper的数据
            T_gripper_t_mm = T_gripper[:3, 3].flatten()
            T_gripper_R = T_gripper[:3, :3]
            
            extraction_detail.append({
                'pose_idx': idx + 1,
                'T_gripper_t_mm': T_gripper_t_mm.tolist(),
                'T_gripper_t_m': (T_gripper_t_mm / 1000.0).tolist(),
                'R_base2gripper_det': float(np.linalg.det(R_base2gripper)),
                'R_base2gripper_is_orthogonal': bool(np.allclose(R_base2gripper @ R_base2gripper.T, np.eye(3), atol=1e-3)),
                't_base2gripper_norm_mm': float(np.linalg.norm(t_base2gripper_mm)),
                't_base2gripper_norm_m': float(np.linalg.norm(t_base2gripper_mm)) / 1000.0,
                'coordinate_direction': 'Base→Gripper (直接使用，不取逆)'
            })
            # #endregion
            
            # 尝试从保存的原始数据中获取rvec和tvec（优先使用）
            rvec_found = False
            tvec_found = False
            
            # 首先检查board_rvec_tvec_map中是否保存了原始rvec/tvec
            board_key = pose_key(T_board)
            if board_key in board_rvec_tvec_map:
                rvec_tvec_data = board_rvec_tvec_map[board_key]
                rvec_data = rvec_tvec_data['rvec']
                tvec_data = rvec_tvec_data['tvec']
                rvec = np.array([[rvec_data['x']], [rvec_data['y']], [rvec_data['z']]], dtype=np.float64)
                tvec = np.array([[tvec_data['x']], [tvec_data['y']], [tvec_data['z']]], dtype=np.float64)
                rvecs_list.append(rvec)
                tvecs_list.append(tvec)
                rvec_found = True
                tvec_found = True
                rvec_found_count += 1
                tvec_found_count += 1
            else:
                # 如果没有保存的原始数据，从poses_data中查找匹配的board_pose
                # 遍历所有位姿数据，查找与当前T_board匹配的board_pose
                for pose_data_item in poses_data:
                    board_pose_item = pose_data_item.get('board_pose', {})
                    
                    # 检查board_pose是否匹配
                    if 'rvec' in board_pose_item and 'tvec' in board_pose_item:
                        T_check = self._pose_to_transform_matrix(
                            {'x': board_pose_item.get('position', {}).get('x', 0.0), 
                             'y': board_pose_item.get('position', {}).get('y', 0.0), 
                             'z': board_pose_item.get('position', {}).get('z', 0.0)},
                            {'x': board_pose_item.get('orientation', {}).get('x', 0.0), 
                             'y': board_pose_item.get('orientation', {}).get('y', 0.0), 
                             'z': board_pose_item.get('orientation', {}).get('z', 0.0), 
                             'w': board_pose_item.get('orientation', {}).get('w', 1.0)}
                        )
                        if np.allclose(T_check, T_board, atol=1e-3):
                            rvec = np.array([[board_pose_item['rvec']['x']], [board_pose_item['rvec']['y']], [board_pose_item['rvec']['z']]], dtype=np.float64)
                            tvec = np.array([[board_pose_item['tvec']['x']], [board_pose_item['tvec']['y']], [board_pose_item['tvec']['z']]], dtype=np.float64)
                            rvecs_list.append(rvec)
                            tvecs_list.append(tvec)
                            rvec_found = True
                            tvec_found = True
                            rvec_found_count += 1
                            tvec_found_count += 1
                            break
            
            # 如果没有找到rvec/tvec，从旋转矩阵转换回rvec（兼容旧数据）
            if not rvec_found:
                R_board = T_board[:3, :3]
                rvec, _ = cv2.Rodrigues(R_board)
                rvecs_list.append(rvec)
                rvec_converted_count += 1
            
            if not tvec_found:
                tvec = T_board[:3, 3].reshape(3, 1)
                tvecs_list.append(tvec)
                tvec_converted_count += 1
        
        self._log('info', f'rvec/tvec提取：从原始数据找到{rvec_found_count}个rvec，转换{rvec_converted_count}个；找到{tvec_found_count}个tvec，转换{tvec_converted_count}个')
        
        # 验证数据一致性
        if len(R_gripper2base_list) != len(rvecs_list):
            raise ValueError(f'数据不一致: R_gripper2base({len(R_gripper2base_list)}) != rvecs({len(rvecs_list)})')
        if len(t_gripper2base_list) != len(tvecs_list):
            raise ValueError(f'数据不一致: t_gripper2base({len(t_gripper2base_list)}) != tvecs({len(tvecs_list)})')
        
        # #region agent log - 检查单位转换过程
        # 记录单位转换前后的数据，验证转换正确性
        unit_conversion_detail = []
        for i, t_mm in enumerate(t_gripper2base_list):
            t_mm_array = np.array(t_mm).flatten() if isinstance(t_mm, (list, np.ndarray)) else np.array([t_mm])
            t_m_expected = t_mm_array / 1000.0
            t_norm_mm = float(np.linalg.norm(t_mm_array))
            t_norm_m = t_norm_mm / 1000.0
            
            unit_conversion_detail.append({
                'pose_idx': i + 1,
                't_gripper2base_mm': t_mm_array.tolist(),
                't_gripper2base_m_expected': t_m_expected.tolist(),
                't_norm_mm': t_norm_mm,
                't_norm_m': t_norm_m
            })
        
        log_entry_unit_conversion = {
            'sessionId': 'debug-session',
            'runId': 'check-gripper-transform',
            'hypothesisId': 'CHECK_GRIPPER_TRANSFORM',
            'location': 'opencv_hand_eye_calibration.py:prepare_data',
            'message': '[检查变换矩阵] 单位转换：毫米→米',
            'data': {
                'num_poses': len(t_gripper2base_list),
                'extraction_detail': extraction_detail,
                'unit_conversion_detail': unit_conversion_detail,
                'conversion_factor': 1000.0,
                't_gripper2base_stats_mm': {
                    'min_norm': float(np.min([np.linalg.norm(t) for t in t_gripper2base_list])),
                    'max_norm': float(np.max([np.linalg.norm(t) for t in t_gripper2base_list])),
                    'mean_norm': float(np.mean([np.linalg.norm(t) for t in t_gripper2base_list])),
                    'std_norm': float(np.std([np.linalg.norm(t) for t in t_gripper2base_list]))
                },
                't_gripper2base_stats_m': {
                    'min_norm': float(np.min([np.linalg.norm(t) for t in t_gripper2base_list])) / 1000.0,
                    'max_norm': float(np.max([np.linalg.norm(t) for t in t_gripper2base_list])) / 1000.0,
                    'mean_norm': float(np.mean([np.linalg.norm(t) for t in t_gripper2base_list])) / 1000.0,
                    'std_norm': float(np.std([np.linalg.norm(t) for t in t_gripper2base_list])) / 1000.0
                }
            },
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_unit_conversion)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        # 单位转换：毫米 → 米（OpenCV要求）
        R_gripper2base = np.array(R_gripper2base_list, dtype=np.float64)
        t_gripper2base = [t.reshape(3, 1) / 1000.0 if hasattr(t, 'reshape') else np.array(t).reshape(3, 1) / 1000.0 for t in t_gripper2base_list]  # 毫米 → 米
        rvecs = rvecs_list  # 弧度，无需转换
        tvecs = [t / 1000.0 if isinstance(t, np.ndarray) else np.array(t) / 1000.0 for t in tvecs_list]  # 毫米 → 米
        
        # #region agent log - 记录单位转换后的数据
        # 记录单位转换后的数据，验证转换是否正确
        unit_converted_detail = []
        for i in range(min(3, len(t_gripper2base))):  # 只记录前3个
            t_m = t_gripper2base[i]
            tvec_m = tvecs[i]
            t_mm_original = t_gripper2base_list[i]
            tvec_mm_original = tvecs_list[i]
            
            unit_converted_detail.append({
                'pose_idx': i + 1,
                't_gripper2base': {
                    'original_mm': t_mm_original.flatten().tolist() if hasattr(t_mm_original, 'flatten') else list(t_mm_original),
                    'converted_m': t_m.flatten().tolist() if hasattr(t_m, 'flatten') else list(t_m),
                    'back_to_mm': (np.array(t_m).flatten() * 1000.0).tolist(),
                    'conversion_ok': np.allclose(np.array(t_mm_original).flatten(), np.array(t_m).flatten() * 1000.0, atol=0.01)
                },
                'tvec': {
                    'original_mm': tvec_mm_original.flatten().tolist() if hasattr(tvec_mm_original, 'flatten') else list(tvec_mm_original),
                    'converted_m': tvec_m.flatten().tolist() if hasattr(tvec_m, 'flatten') else list(tvec_m),
                    'back_to_mm': (np.array(tvec_m).flatten() * 1000.0).tolist(),
                    'conversion_ok': np.allclose(np.array(tvec_mm_original).flatten(), np.array(tvec_m).flatten() * 1000.0, atol=0.01)
                }
            })
        
        log_entry_unit_converted = {
            'sessionId': 'debug-session',
            'runId': 'check-transform-A',
            'hypothesisId': 'CHECK_UNIT_CONVERSION',
            'location': 'opencv_hand_eye_calibration.py:prepare_data',
            'message': '[检查单位转换] 转换后的数据验证（毫米→米）',
            'data': {
                'num_poses': len(t_gripper2base),
                'unit_conversion_detail': unit_converted_detail,
                'conversion_factor': 1000.0,
                'summary': {
                    'all_t_gripper_conversions_ok': all(d['t_gripper2base']['conversion_ok'] for d in unit_converted_detail),
                    'all_tvec_conversions_ok': all(d['tvec']['conversion_ok'] for d in unit_converted_detail)
                }
            },
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_unit_converted)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        # #region agent log - 记录OpenCV内部计算的A和B矩阵（验证）
        # 在传递给OpenCV之前，手动计算相邻位姿对的A和B矩阵，用于验证
        if len(R_gripper2base) >= 2:
            manual_A_B_detail = []
            num_pairs_to_verify = min(5, len(R_gripper2base) - 1)
            
            for i in range(num_pairs_to_verify):
                # 手动计算A矩阵（使用OpenCV内部可能使用的方法）
                # A_R = inv(R_gripper2base[i]) @ R_gripper2base[i+1]
                # A_t = inv(R_gripper2base[i]) @ (t_gripper2base[i+1] - t_gripper2base[i])
                R1 = R_gripper2base[i]
                R2 = R_gripper2base[i+1]
                t1 = t_gripper2base[i]
                t2 = t_gripper2base[i+1]
                
                # OpenCV内部可能使用的方式
                A_R_opencv_style = R1.T @ R2  # 如果R1是正交的，inv(R1) = R1.T
                A_t_opencv_style = R1.T @ (t2 - t1)
                
                # 我们使用的完整变换矩阵方式
                T1_full = np.eye(4)
                T1_full[:3, :3] = R1
                T1_full[:3, 3] = t1.flatten()
                T2_full = np.eye(4)
                T2_full[:3, :3] = R2
                T2_full[:3, 3] = t2.flatten()
                A_full = np.linalg.inv(T1_full) @ T2_full
                
                # 比较两种方式
                A_R_full = A_full[:3, :3]
                A_t_full = A_full[:3, 3]
                A_R_diff = np.abs(A_R_opencv_style - A_R_full)
                A_t_diff = np.abs(A_t_opencv_style.flatten() - A_t_full)
                
                # 手动计算B矩阵（使用OpenCV内部可能使用的方法）
                # B_R = rvec_to_R(rvecs[i]) @ inv(rvec_to_R(rvecs[i+1]))
                # 但实际上OpenCV接受rvec和tvec，内部会转换
                rvec1 = rvecs[i]
                rvec2 = rvecs[i+1]
                tvec1 = tvecs[i]
                tvec2 = tvecs[i+1]
                
                # 将rvec转换为旋转矩阵
                R_board1, _ = cv2.Rodrigues(rvec1)
                R_board2, _ = cv2.Rodrigues(rvec2)
                
                # OpenCV内部可能使用的方式（基于变换矩阵的逆）
                # 对于变换矩阵 T = [R t; 0 1]，inv(T) = [R^T -R^T @ t; 0 1]
                # 所以 B = T1 @ inv(T2) = [R1 @ R2^T  -R1 @ R2^T @ t2 + t1; 0 1]
                B_R_opencv_style = R_board1 @ R_board2.T  # inv(R_board2) = R_board2.T（如果正交）
                B_t_opencv_style = tvec1 - B_R_opencv_style @ tvec2  # -R1 @ R2^T @ t2 + t1
                
                # 我们使用的完整变换矩阵方式
                T_board1_full = np.eye(4)
                T_board1_full[:3, :3] = R_board1
                T_board1_full[:3, 3] = tvec1.flatten()
                T_board2_full = np.eye(4)
                T_board2_full[:3, :3] = R_board2
                T_board2_full[:3, 3] = tvec2.flatten()
                B_full = T_board1_full @ np.linalg.inv(T_board2_full)
                
                # 比较两种方式
                B_R_full = B_full[:3, :3]
                B_t_full = B_full[:3, 3]
                B_R_diff = np.abs(B_R_opencv_style - B_R_full)
                B_t_diff = np.abs(B_t_opencv_style.flatten() - B_t_full)
                
                manual_A_B_detail.append({
                    'pose_pair_idx': i + 1,
                    'A_matrix': {
                        'opencv_style_R': A_R_opencv_style.tolist(),
                        'opencv_style_t_m': A_t_opencv_style.flatten().tolist(),
                        'full_matrix_R': A_R_full.tolist(),
                        'full_matrix_t_m': A_t_full.tolist(),
                        'R_diff_max': float(np.max(A_R_diff)),
                        't_diff_max_m': float(np.max(A_t_diff)),
                        't_diff_max_mm': float(np.max(A_t_diff)) * 1000.0,
                        'methods_match': bool(np.allclose(A_R_opencv_style, A_R_full, atol=1e-6) and np.allclose(A_t_opencv_style.flatten(), A_t_full, atol=1e-6))
                    },
                    'B_matrix': {
                        'opencv_style_R': B_R_opencv_style.tolist(),
                        'opencv_style_t_m': B_t_opencv_style.flatten().tolist(),
                        'full_matrix_R': B_R_full.tolist(),
                        'full_matrix_t_m': B_t_full.tolist(),
                        'R_diff_max': float(np.max(B_R_diff)),
                        't_diff_max_m': float(np.max(B_t_diff)),
                        't_diff_max_mm': float(np.max(B_t_diff)) * 1000.0,
                        'methods_match': bool(np.allclose(B_R_opencv_style, B_R_full, atol=1e-6) and np.allclose(B_t_opencv_style.flatten(), B_t_full, atol=1e-6)),
                        'formula': 'B = T_board1 @ inv(T_board2) = [R1 @ R2^T, -R1 @ R2^T @ t2 + t1; 0, 1]',
                        'meaning': '相机观测标定板的相对变换：从姿态1到姿态2，相机观测到的标定板相对变换（相机坐标系）',
                        'physical_meaning': 'B是相机观测标定板的相对变换，由于标定板固定，这个相对变换实际上是由于相机（固定在末端）的运动造成的'
                    }
                })
            
            log_entry_manual_AB = {
                'sessionId': 'debug-session',
                'runId': 'check-transform-A',
                'hypothesisId': 'CHECK_OPENCV_INTERNAL_AB',
                'location': 'opencv_hand_eye_calibration.py:prepare_data',
                'message': '[检查OpenCV内部A/B] 手动计算A和B矩阵，用于验证OpenCV内部计算',
                'data': {
                    'num_pairs_verified': num_pairs_to_verify,
                    'manual_A_B_detail': manual_A_B_detail,
                    'note': 'OpenCV内部会自动计算相邻位姿之间的A和B，这里手动计算用于验证'
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_manual_AB)
            try:
                with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps(log_entry_clean) + '\n')
            except: pass
            # #endregion
        
        self._log('info', f'   [OpenCV模式] 单位转换完成：机器人位姿({len(t_gripper2base)}个)和标定板位姿({len(tvecs)}个)已转换为米')
        
        # #region agent log - 验证最终传递给OpenCV的位姿对计算的A矩阵
        # 在传递给OpenCV之前，验证最终位姿对的A矩阵计算是否正确
        if len(T_gripper_list) >= 2:
            # 验证前几个相邻位姿对的A矩阵
            num_verify_pairs = min(5, len(T_gripper_list) - 1)
            final_A_verification = []
            
            for i in range(num_verify_pairs):
                T_g1_final = T_gripper_list[i]
                T_g2_final = T_gripper_list[i + 1]
                
                T_g1_final_t_mm = T_g1_final[:3, 3].flatten()
                T_g2_final_t_mm = T_g2_final[:3, 3].flatten()
                T_g1_final_R = T_g1_final[:3, :3]
                T_g2_final_R = T_g2_final[:3, :3]
                
                # 计算A矩阵：A = inv(T_gripper1) @ T_gripper2
                # A是末端运动前后的相对变换：从姿态1到姿态2的末端相对运动（在基座坐标系中）
                A_final = np.linalg.inv(T_g1_final) @ T_g2_final
                A_final_t_mm = A_final[:3, 3].flatten()
                A_final_R = A_final[:3, :3]
                
                # 验证：T_gripper1 @ A 应该等于 T_gripper2
                # 因为 A = inv(T_gripper1) @ T_gripper2 是末端运动前后的相对变换
                # 所以 T_gripper1 @ A = T_gripper1 @ inv(T_gripper1) @ T_gripper2 = T_gripper2
                # 物理意义：在基座坐标系中，应用相对运动A到姿态1，应该得到姿态2
                T1_A_check_final = T_g1_final @ A_final
                T1_A_diff_final = np.abs(T1_A_check_final - T_g2_final)
                T1_A_max_diff_final = float(np.max(T1_A_diff_final))
                T1_A_check_ok_final = bool(np.allclose(T1_A_check_final, T_g2_final, atol=1e-3))
                
                final_A_verification.append({
                    'pose_pair_idx': i + 1,
                    'T_gripper1_final': {
                        'translation_mm': T_g1_final_t_mm.tolist(),
                        'rotation_det': float(np.linalg.det(T_g1_final_R)),
                        'rotation_is_orthogonal': bool(np.allclose(T_g1_final_R @ T_g1_final_R.T, np.eye(3), atol=1e-3))
                    },
                    'T_gripper2_final': {
                        'translation_mm': T_g2_final_t_mm.tolist(),
                        'rotation_det': float(np.linalg.det(T_g2_final_R)),
                        'rotation_is_orthogonal': bool(np.allclose(T_g2_final_R @ T_g2_final_R.T, np.eye(3), atol=1e-3))
                    },
                    'A_matrix_final': {
                        'translation_mm': A_final_t_mm.tolist(),
                        'translation_norm_mm': float(np.linalg.norm(A_final_t_mm)),
                        'rotation_det': float(np.linalg.det(A_final_R)),
                        'rotation_is_orthogonal': bool(np.allclose(A_final_R @ A_final_R.T, np.eye(3), atol=1e-3))
                    },
                    'A_validation_final': {
                        'T1_A_check_ok': T1_A_check_ok_final,
                        'T1_A_max_diff': T1_A_max_diff_final,
                        'T1_A_translation_diff_mm': T1_A_diff_final[:3, 3].flatten().tolist(),
                        'T1_A_rotation_diff_max': float(np.max(np.abs(T1_A_diff_final[:3, :3]))),
                        'validation_formula': 'T_gripper1 @ A should equal T_gripper2',
                        'physical_meaning': 'A是末端运动前后的相对变换，在基座坐标系中应用A到姿态1应该得到姿态2'
                    },
                    'note': '这是在最终传递给OpenCV之前的验证，用于确认数据准备阶段A矩阵计算正确'
                })
            
            log_entry_final_A = {
                'sessionId': 'debug-session',
                'runId': 'check-transform-A',
                'hypothesisId': 'CHECK_TRANSFORM_A_FINAL',
                'location': 'opencv_hand_eye_calibration.py:prepare_data',
                'message': '[检查变换矩阵A] 最终传递给OpenCV的位姿对的A矩阵验证',
                'data': {
                    'total_poses': len(T_gripper_list),
                    'num_verified_pairs': num_verify_pairs,
                    'final_A_verification': final_A_verification,
                    'summary': {
                        'all_valid': all(v['A_validation_final']['T1_A_check_ok'] for v in final_A_verification),
                        'max_diff': max(v['A_validation_final']['T1_A_max_diff'] for v in final_A_verification),
                        'failed_pairs': [v['pose_pair_idx'] for v in final_A_verification if not v['A_validation_final']['T1_A_check_ok']]
                    }
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_final_A)
            try:
                with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps(log_entry_clean) + '\n')
            except: pass
            # #endregion
        
        return R_gripper2base, t_gripper2base, rvecs, tvecs, T_gripper_list, T_board_list
    
    def solve_hand_eye(self, R_gripper2base, t_gripper2base, rvecs, tvecs, opencv_algorithm='TSAI'):
        """
        OpenCV模式：调用calibrateHandEye算法计算手眼标定
        
        参数:
            R_gripper2base: (N, 3, 3) numpy数组，机器人旋转矩阵列表
            t_gripper2base: 列表，每个元素(3,1)数组，机器人平移向量（单位：米）
            rvecs: 列表，每个元素(3,1)数组，标定板旋转向量（单位：弧度）
            tvecs: 列表，每个元素(3,1)数组，标定板平移向量（单位：米）
            opencv_algorithm: OpenCV算法选择，可选值: 'TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS'（默认: 'TSAI'）
        
        返回:
            T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
        """
        # 算法名称映射
        algorithm_names = {
            'TSAI': 'TSAI (经典方法，稳定可靠)',
            'PARK': 'PARK (Park & Martin)',
            'HORAUD': 'HORAUD (Horaud & Dornaika)',
            'ANDREFF': 'ANDREFF (Andreff et al.)',
            'DANIILIDIS': 'DANIILIDIS (Daniilidis)'
        }
        algorithm_name = algorithm_names.get(opencv_algorithm, opencv_algorithm)
        self._log('info', f'开始算法计算（{algorithm_name}）')
        self._log('info', f'输入数据：{len(R_gripper2base)} 个姿态')
        
        # 验证输入数据格式
        if len(R_gripper2base) != len(t_gripper2base):
            raise ValueError(f'输入数据不一致: R_gripper2base({len(R_gripper2base)}) != t_gripper2base({len(t_gripper2base)})')
        if len(rvecs) != len(tvecs):
            raise ValueError(f'输入数据不一致: rvecs({len(rvecs)}) != tvecs({len(tvecs)})')
        if len(R_gripper2base) != len(rvecs):
            raise ValueError(f'输入数据不一致: R_gripper2base({len(R_gripper2base)}) != rvecs({len(rvecs)})')
        
        # 验证数据格式
        for i, (R, t, rvec, tvec) in enumerate(zip(R_gripper2base, t_gripper2base, rvecs, tvecs)):
            if R.shape != (3, 3):
                raise ValueError(f'姿态#{i+1}: R形状错误 {R.shape}，应为(3,3)')
            if t.shape != (3, 1):
                raise ValueError(f'姿态#{i+1}: t形状错误 {t.shape}，应为(3,1)')
            if rvec.shape != (3, 1):
                raise ValueError(f'姿态#{i+1}: rvec形状错误 {rvec.shape}，应为(3,1)')
            if tvec.shape != (3, 1):
                raise ValueError(f'姿态#{i+1}: tvec形状错误 {tvec.shape}，应为(3,1)')
        
        # 检查旋转矩阵的行列式（应该接近1.0）
        det_errors = []
        for i, R in enumerate(R_gripper2base):
            det = np.linalg.det(R)
            if abs(det - 1.0) > 0.01:
                det_errors.append((i+1, det))
        
        if det_errors:
            self._log('warning', f'⚠️ 发现{len(det_errors)}个旋转矩阵行列式异常')
            for idx, det in det_errors[:5]:  # 只显示前5个
                self._log('warning', f'姿态#{idx}: det={det:.6f}')
        
        # #region agent log - 记录OpenCV输入数据
        # 记录传递给OpenCV的数据，用于验证输入是否正确
        opencv_input_detail = []
        for i in range(min(5, len(R_gripper2base))):  # 只记录前5个
            R = R_gripper2base[i]
            t = t_gripper2base[i]
            rvec = rvecs[i]
            tvec = tvecs[i]
            
            opencv_input_detail.append({
                'pose_idx': i + 1,
                'R_gripper2base': {
                    'matrix': R.tolist() if isinstance(R, np.ndarray) else R,
                    'det': float(np.linalg.det(R)),
                    'is_orthogonal': bool(np.allclose(R @ R.T, np.eye(3), atol=1e-3))
                },
                't_gripper2base_m': t.flatten().tolist() if hasattr(t, 'flatten') else t,
                't_gripper2base_mm': (np.array(t).flatten() * 1000.0).tolist() if hasattr(t, 'flatten') else [x * 1000.0 for x in t],
                'rvec_rad': rvec.flatten().tolist() if hasattr(rvec, 'flatten') else rvec,
                'tvec_m': tvec.flatten().tolist() if hasattr(tvec, 'flatten') else tvec,
                'tvec_mm': (np.array(tvec).flatten() * 1000.0).tolist() if hasattr(tvec, 'flatten') else [x * 1000.0 for x in tvec]
            })
        
        log_entry_opencv_input = {
            'sessionId': 'debug-session',
            'runId': 'check-transform-A',
            'hypothesisId': 'CHECK_OPENCV_INPUT',
            'location': 'opencv_hand_eye_calibration.py:solve_hand_eye',
            'message': '[检查OpenCV输入] 传递给calibrateHandEye的数据',
            'data': {
                'num_poses': len(R_gripper2base),
                'solver_method': 'TSAI',
                'input_data_detail': opencv_input_detail,
                'input_data_summary': {
                    't_gripper2base_mean_mm': float(np.mean([np.linalg.norm(t) for t in t_gripper2base])) * 1000.0,
                    't_gripper2base_std_mm': float(np.std([np.linalg.norm(t) for t in t_gripper2base])) * 1000.0,
                    'tvec_mean_mm': float(np.mean([np.linalg.norm(t) for t in tvecs])) * 1000.0,
                    'tvec_std_mm': float(np.std([np.linalg.norm(t) for t in tvecs])) * 1000.0,
                    'tvec_z_mean_mm': float(np.mean([t[2, 0] for t in tvecs])) * 1000.0,
                    'tvec_z_std_mm': float(np.std([t[2, 0] for t in tvecs])) * 1000.0
                }
            },
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_opencv_input)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        # 调用OpenCV calibrateHandEye
        # 根据选择的算法设置solver_method
        algorithm_map = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS
        }
        solver_method = algorithm_map.get(opencv_algorithm.upper(), cv2.CALIB_HAND_EYE_TSAI)
        if opencv_algorithm.upper() not in algorithm_map:
            self._log('warning', f'未知的算法 "{opencv_algorithm}"，使用默认TSAI方法')
        
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,  # 单位：米
                rvecs, tvecs,                     # 单位：米
                method=solver_method
            )
        except Exception as e:
            self._log('error', f'calibrateHandEye调用失败: {str(e)}')
            # #region agent log - 记录OpenCV调用失败
            log_entry_opencv_error = {
                'sessionId': 'debug-session',
                'runId': 'check-transform-A',
                'hypothesisId': 'CHECK_OPENCV_ERROR',
                'location': 'opencv_hand_eye_calibration.py:solve_hand_eye',
                'message': '[检查OpenCV错误] calibrateHandEye调用失败',
                'data': {
                    'error': str(e),
                    'num_poses': len(R_gripper2base),
                    'solver_method': 'Daniilidis1999'
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_opencv_error)
            try:
                with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps(log_entry_clean) + '\n')
            except: pass
            # #endregion
            raise
        
        # #region agent log - 记录OpenCV输出结果
        # 记录OpenCV返回的标定结果
        t_cam2gripper_norm_mm = float(np.linalg.norm(t_cam2gripper)) * 1000.0
        t_cam2gripper_z_mm = float(t_cam2gripper[2, 0]) * 1000.0
        det_R_cam2gripper = float(np.linalg.det(R_cam2gripper))
        R_cam2gripper_is_orthogonal = bool(np.allclose(R_cam2gripper @ R_cam2gripper.T, np.eye(3), atol=1e-3))
        
        log_entry_opencv_output = {
            'sessionId': 'debug-session',
            'runId': 'check-transform-A',
            'hypothesisId': 'CHECK_OPENCV_OUTPUT',
            'location': 'opencv_hand_eye_calibration.py:solve_hand_eye',
            'message': '[检查OpenCV输出] calibrateHandEye返回的标定结果',
            'data': {
                'R_cam2gripper': {
                    'matrix': R_cam2gripper.tolist(),
                    'det': det_R_cam2gripper,
                    'is_orthogonal': R_cam2gripper_is_orthogonal
                },
                't_cam2gripper': {
                    'm': t_cam2gripper.flatten().tolist(),
                    'mm': (t_cam2gripper.flatten() * 1000.0).tolist(),
                    'norm_mm': t_cam2gripper_norm_mm,
                    'z_mm': t_cam2gripper_z_mm
                },
                'input_summary': {
                    'tvec_z_mean_mm': float(np.mean([tvec[2, 0] for tvec in tvecs])) * 1000.0,
                    'tvec_z_std_mm': float(np.std([tvec[2, 0] for tvec in tvecs])) * 1000.0
                },
                'z_ratio': t_cam2gripper_z_mm / (float(np.mean([tvec[2, 0] for tvec in tvecs])) * 1000.0) if len(tvecs) > 0 else 0.0
            },
            'timestamp': int(time.time() * 1000)
        }
        log_entry_clean = _convert_to_json_serializable(log_entry_opencv_output)
        try:
            with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                f.write(json.dumps(log_entry_clean) + '\n')
        except: pass
        # #endregion
        
        # 计算输入数据的Z值范围，用于验证输出Z值的合理性
        input_tvec_z_mean_mm = float(np.mean([tvec[2, 0] for tvec in tvecs])) * 1000.0
        input_tvec_z_std_mm = float(np.std([tvec[2, 0] for tvec in tvecs])) * 1000.0
        
        # Z值合理性检查：输出Z值应该在输入Z值的合理范围内（考虑相机到末端的距离）
        # 相机到末端的Z值通常应该在100-800mm范围内
        EXPECTED_Z_MIN_MM = 100.0
        EXPECTED_Z_MAX_MM = 800.0
        Z_ANOMALY_THRESHOLD = 1000.0  # 如果Z值>1000mm，认为是异常
        
        z_value_anomaly = False
        z_anomaly_reason = None
        
        if t_cam2gripper_z_mm > Z_ANOMALY_THRESHOLD:
            z_value_anomaly = True
            z_anomaly_reason = f'Z值过大({t_cam2gripper_z_mm:.1f}mm > {Z_ANOMALY_THRESHOLD}mm)'
            self._log('error', f'❌ Z值异常：{z_anomaly_reason}')
            self._log('error', f'输入标定板Z值平均值：{input_tvec_z_mean_mm:.1f}mm ± {input_tvec_z_std_mm:.1f}mm')
            self._log('error', f'输出Z值：{t_cam2gripper_z_mm:.1f}mm')
            self._log('error', f'Z值比率：{t_cam2gripper_z_mm / input_tvec_z_mean_mm:.2f}x')
        # 移除Z值过小的警告检查（某些安装情况下Z值可能确实很小）
        # elif t_cam2gripper_z_mm < EXPECTED_Z_MIN_MM:
        #     z_value_anomaly = True
        #     z_anomaly_reason = f'Z值过小({t_cam2gripper_z_mm:.1f}mm < {EXPECTED_Z_MIN_MM}mm)'
        #     self._log('warning', f'⚠️ Z值异常：{z_anomaly_reason}')
        elif t_cam2gripper_z_mm > EXPECTED_Z_MAX_MM:
            z_value_anomaly = True
            z_anomaly_reason = f'Z值较大({t_cam2gripper_z_mm:.1f}mm > {EXPECTED_Z_MAX_MM}mm)'
            self._log('warning', f'⚠️ Z值异常：{z_anomaly_reason}')
        
        if t_cam2gripper_norm_mm > 2000.0:  # 如果>2米（2000mm），异常
            self._log('error', f'❌ 严重问题：OpenCV返回的平移向量异常大 {t_cam2gripper_norm_mm:.1f}mm！')
        elif t_cam2gripper_norm_mm > 1000.0:  # 1-2米，警告
            self._log('warning', f'⚠️ OpenCV返回的平移向量较大 {t_cam2gripper_norm_mm:.1f}mm，建议检查输入数据')
        
        # 验证返回结果
        det_R = np.linalg.det(R_cam2gripper)
        if abs(det_R - 1.0) > 0.01:
            self._log('warning', f'⚠️ 返回的旋转矩阵行列式异常: {det_R:.6f}（应为1.0）')
        
        # 构建4x4变换矩阵（单位：毫米）
        T_camera2gripper = np.eye(4)
        T_camera2gripper[:3, :3] = R_cam2gripper
        T_camera2gripper[:3, 3] = t_cam2gripper.flatten() * 1000.0  # 米 → 毫米
        
        self._log('info', f'算法计算完成：t_cam2gripper={np.linalg.norm(T_camera2gripper[:3, 3]):.1f}mm, det(R)={det_R:.6f}')
        
        return T_camera2gripper
    
    def calculate_errors(self, T_gripper_list, T_board_list, T_camera2gripper):
        """
        OpenCV模式：计算标定误差（验证AX=XB约束）
        
        参数:
            T_gripper_list: 列表，机器人变换矩阵（单位：毫米）
            T_board_list: 列表，标定板变换矩阵（单位：毫米）
            T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
        
        返回:
            translation_errors: 列表，每个运动组的平移误差（单位：毫米）
            rotation_errors: 列表，每个运动组的旋转误差（单位：弧度）
            error_statistics: 字典，包含RMS、最大、最小、标准差等统计信息
        """
        self._log('info', f'开始误差计算：验证AX=XB约束')
        
        translation_errors = []
        rotation_errors = []
        
        # 验证输入数据一致性
        if len(T_gripper_list) != len(T_board_list):
            raise ValueError(f'输入数据不一致: T_gripper_list({len(T_gripper_list)}) != T_board_list({len(T_board_list)})')
        
        # 从去重后的姿态列表构建运动组（相邻姿态对）
        A_list_for_error = []  # 末端运动前后的相对变换矩阵列表（A矩阵）
        B_list_for_error = []  # 相机观测标定板的相对变换矩阵列表（B矩阵）
        
        num_motions = len(T_gripper_list) - 1  # 相邻姿态对的数量
        
        self._log('info', f'构建{num_motions}个相邻姿态对用于误差验证')
        
        for i in range(num_motions):
            pose1_idx = i
            pose2_idx = i + 1
            
            if pose2_idx >= len(T_gripper_list):
                continue
            
            try:
                # T_gripper_list存储的是Base→Gripper（与传给OpenCV的数据一致，不要取逆！）
                # OpenCV内部会自己计算 A = inv(Base→Gripper₁) @ Base→Gripper₂
                T_base2gripper1 = T_gripper_list[pose1_idx]  # Base→Gripper
                T_base2gripper2 = T_gripper_list[pose2_idx]  # Base→Gripper
                
                # 计算A：与OpenCV内部计算方式一致
                # A = inv(Base→Gripper₁) @ Base→Gripper₂ = Gripper₁→Gripper₂
                A = np.linalg.inv(T_base2gripper1) @ T_base2gripper2
                A_list_for_error.append(A)
                
                # 计算标定板运动：B = T_board1 @ inv(T_board2)
                # 注意：T_board1和T_board2应该是Board → Camera的变换矩阵（solvePnP返回）
                # 如果T_board是Camera → Board，则需要取逆
                # B是相机观测标定板的相对变换：从姿态1到姿态2，相机观测到的标定板相对变换（在相机坐标系中）
                # 物理意义：由于标定板固定，这个相对变换实际上是由于相机（固定在末端）的运动造成的
                
                T_board1 = T_board_list[pose1_idx]
                T_board2 = T_board_list[pose2_idx]
                
                # #region agent log - 验证T_board的坐标系方向
                # 验证T_board的坐标系方向是否正确
                # solvePnP返回的rvec/tvec应该是Board → Camera
                # 如果T_board是Board → Camera，那么：
                # - T_board的平移部分应该是标定板在相机坐标系中的位置
                # - B = T_board1 @ inv(T_board2) 应该表示从姿态1到姿态2的标定板相对运动
                
                # 计算B矩阵（假设T_board是Board → Camera）
                T_board2_inv = np.linalg.inv(T_board2)
                B = T_board1 @ T_board2_inv
                B_z = B[2, 3]
                
                # 如果T_board实际上是Camera → Board，那么应该使用：
                # B_wrong = inv(T_board1) @ T_board2
                B_wrong_direction = np.linalg.inv(T_board1) @ T_board2
                B_wrong_z = B_wrong_direction[2, 3]
                
                # 验证：B @ T_board2 应该等于 T_board1（如果B计算正确）
                B_T2_check = B @ T_board2
                B_T2_check_ok = np.allclose(B_T2_check, T_board1, atol=1e-3)
                
                # 如果T_board是Camera → Board，那么 T_board2 @ B_wrong 应该等于 T_board1
                T2_B_wrong_check = T_board2 @ B_wrong_direction
                T2_B_wrong_check_ok = np.allclose(T2_B_wrong_check, T_board1, atol=1e-3)
                
                log_entry_board_coordinate_check = {
                    'sessionId': 'debug-session',
                    'runId': 'check-transform-A',
                    'hypothesisId': 'CHECK_BOARD_COORDINATE_FRAME',
                    'location': 'opencv_hand_eye_calibration.py:calculate_errors',
                    'message': '[检查坐标系] 验证T_board的坐标系方向',
                    'data': {
                        'pose_pair_idx': i + 1,
                        'T_board1_z_mm': float(T_board1[2, 3]),
                        'T_board2_z_mm': float(T_board2[2, 3]),
                        'z_change_mm': float(T_board2[2, 3] - T_board1[2, 3]),
                        'B_calculation': {
                            'assumption': 'T_board is Board → Camera (solvePnP standard)',
                            'formula': 'B = T_board1 @ inv(T_board2)',
                            'B_z_mm': float(B_z),
                            'B_T2_check_ok': bool(B_T2_check_ok),
                            'B_T2_max_diff': float(np.max(np.abs(B_T2_check - T_board1))) if not B_T2_check_ok else 0.0
                        },
                        'B_wrong_calculation': {
                            'assumption': 'T_board is Camera → Board (WRONG)',
                            'formula': 'B_wrong = inv(T_board1) @ T_board2',
                            'B_wrong_z_mm': float(B_wrong_z),
                            'T2_B_wrong_check_ok': bool(T2_B_wrong_check_ok),
                            'T2_B_wrong_max_diff': float(np.max(np.abs(T2_B_wrong_check - T_board1))) if not T2_B_wrong_check_ok else 0.0
                        },
                        'coordinate_frame_verification': {
                            'T_board_is_board_to_camera': bool(B_T2_check_ok),
                            'T_board_is_camera_to_board': bool(T2_B_wrong_check_ok),
                            'recommended_direction': 'Board → Camera' if B_T2_check_ok else ('Camera → Board' if T2_B_wrong_check_ok else 'UNKNOWN'),
                            'note': 'solvePnP标准返回Board → Camera，如果B_T2_check_ok为True，则T_board方向正确'
                        }
                    },
                    'timestamp': int(time.time() * 1000)
                }
                log_entry_clean = _convert_to_json_serializable(log_entry_board_coordinate_check)
                try:
                    with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                        f.write(json.dumps(log_entry_clean) + '\n')
                except: pass
                # #endregion
                
                # 使用已验证的B矩阵（已在坐标系验证中计算）
                # B矩阵已在坐标系验证代码中计算，这里直接使用
                B_list_for_error.append(B)
                
                # #region agent log - 检查变换矩阵B的计算
                # 记录T_board1, T_board2和B矩阵的详细信息
                T_board1_t_mm = T_board1[:3, 3].flatten()
                T_board2_t_mm = T_board2[:3, 3].flatten()
                T_board1_R = T_board1[:3, :3]
                T_board2_R = T_board2[:3, :3]
                
                B_t_mm = B[:3, 3].flatten()
                B_R = B[:3, :3]
                B_t_norm_mm = float(np.linalg.norm(B_t_mm))
                B_R_det = float(np.linalg.det(B_R))
                B_R_is_orthogonal = bool(np.allclose(B_R @ B_R.T, np.eye(3), atol=1e-3))
                
                # 验证：B @ T_board2 应该等于 T_board1
                # 因为 B = T_board1 @ inv(T_board2) 是相机观测标定板的相对变换
                # 所以 B @ T_board2 = T_board1 @ inv(T_board2) @ T_board2 = T_board1
                # 物理意义：在相机坐标系中，应用相对变换B到姿态2的标定板位姿，应该得到姿态1的标定板位姿
                B_T2_check = B @ T_board2
                B_T2_diff = np.abs(B_T2_check - T_board1)
                B_T2_max_diff = float(np.max(B_T2_diff))
                B_T2_check_ok = bool(np.allclose(B_T2_check, T_board1, atol=1e-3))
                
                # 计算相机观测到的标定板相对变换幅度
                # 注意：由于标定板固定，这个相对变换实际上反映了相机的运动
                board_translation = B_t_norm_mm
                board_rotation = float(np.arccos(np.clip((np.trace(B_R) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                
                log_entry_B_calc = {
                    'sessionId': 'debug-session',
                    'runId': 'check-transform-A',
                    'hypothesisId': 'CHECK_TRANSFORM_B',
                    'location': 'opencv_hand_eye_calibration.py:calculate_errors',
                    'message': '[检查变换矩阵B] 计算相机观测标定板的相对变换B',
                    'data': {
                        'pose_pair_idx': i + 1,
                        'T_board1': {
                            'translation_mm': T_board1_t_mm.tolist(),
                            'translation_norm_mm': float(np.linalg.norm(T_board1_t_mm)),
                            'rotation_det': float(np.linalg.det(T_board1_R)),
                            'rotation_is_orthogonal': bool(np.allclose(T_board1_R @ T_board1_R.T, np.eye(3), atol=1e-3))
                        },
                        'T_board2': {
                            'translation_mm': T_board2_t_mm.tolist(),
                            'translation_norm_mm': float(np.linalg.norm(T_board2_t_mm)),
                            'rotation_det': float(np.linalg.det(T_board2_R)),
                            'rotation_is_orthogonal': bool(np.allclose(T_board2_R @ T_board2_R.T, np.eye(3), atol=1e-3))
                        },
                        'B_matrix': {
                            'formula': 'B = T_board1 @ inv(T_board2)',
                            'meaning': '相机观测标定板的相对变换：从姿态1到姿态2，相机观测到的标定板相对变换（相机坐标系）',
                            'physical_meaning': 'B是相机观测标定板的相对变换，由于标定板固定，这个相对变换实际上是由于相机（固定在末端）的运动造成的',
                            'translation_mm': B_t_mm.tolist(),
                            'translation_norm_mm': B_t_norm_mm,
                            'rotation_matrix': B_R.tolist(),
                            'rotation_det': B_R_det,
                            'rotation_is_orthogonal': B_R_is_orthogonal,
                            'rotation_angle_deg': board_rotation
                        },
                        'B_validation': {
                            'B_T2_check_ok': B_T2_check_ok,
                            'B_T2_max_diff': B_T2_max_diff,
                            'B_T2_expected_translation_mm': T_board1_t_mm.tolist(),
                            'B_T2_actual_translation_mm': B_T2_check[:3, 3].flatten().tolist(),
                            'B_T2_translation_diff_mm': B_T2_diff[:3, 3].flatten().tolist(),
                            'B_T2_rotation_diff_max': float(np.max(np.abs(B_T2_diff[:3, :3]))),
                            'validation_formula': 'B @ T_board2 should equal T_board1',
                            'validation_passed': B_T2_check_ok
                        },
                        'motion_amplitude': {
                            'translation_mm': board_translation,
                            'rotation_deg': board_rotation
                        }
                    },
                    'timestamp': int(time.time() * 1000)
                }
                log_entry_clean = _convert_to_json_serializable(log_entry_B_calc)
                try:
                    with open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a') as f:
                        f.write(json.dumps(log_entry_clean) + '\n')
                except: pass
                # #endregion
                
            except Exception as e:
                self._log('warning', f'姿态对 #{i+1}-#{i+2} 构建失败: {str(e)}')
                continue
        
        if len(A_list_for_error) == 0:
            raise ValueError('无法构建姿态对，至少需要2个姿态')
        
        # 直接验证AX=XB约束：计算 A @ X - X @ B 的误差
        error_calc_detail = []  # 记录每个运动对的详细误差计算过程
        for i in range(len(A_list_for_error)):
            try:
                A = A_list_for_error[i]
                B = B_list_for_error[i]
                
                # 计算误差矩阵：E = A @ X - X @ B
                # 理想情况下，E应该为零矩阵
                AX = A @ T_camera2gripper
                XB = T_camera2gripper @ B
                error_matrix = AX - XB
                
                # 平移误差：误差矩阵的平移部分的范数
                translation_error = float(np.linalg.norm(error_matrix[:3, 3]))
                translation_errors.append(translation_error)
                
                # 旋转误差：计算A@X和X@B的旋转部分的角度差
                R_AX = (A @ T_camera2gripper)[:3, :3]
                R_XB = (T_camera2gripper @ B)[:3, :3]
                R_diff = R_AX @ R_XB.T
                trace = np.trace(R_diff)
                cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
                rotation_error_angle = float(np.arccos(cos_angle))
                rotation_errors.append(rotation_error_angle)
                
                # 记录详细的误差计算过程
                error_calc_detail.append({
                    'motion_idx': i + 1,
                    'A_matrix': A.tolist(),
                    'B_matrix': B.tolist(),
                    'AX_matrix': AX.tolist(),
                    'XB_matrix': XB.tolist(),
                    'error_matrix': error_matrix.tolist(),
                    'error_matrix_t_mm': error_matrix[:3, 3].flatten().tolist(),
                    'error_matrix_t_norm_mm': translation_error,
                    'translation_error_mm': translation_error,
                    'rotation_error_rad': rotation_error_angle,
                    'rotation_error_deg': rotation_error_angle * 180.0 / np.pi,
                    'AX_t_mm': AX[:3, 3].flatten().tolist(),
                    'XB_t_mm': XB[:3, 3].flatten().tolist(),
                    'AX_t_z_mm': float(AX[2, 3]),
                    'XB_t_z_mm': float(XB[2, 3]),
                    'error_t_z_mm': float(error_matrix[2, 3]),
                    'AX_XB_z_diff_mm': float(AX[2, 3] - XB[2, 3])
                })
                
            except Exception as e:
                self._log('warning', f'姿态对 #{i+1} 误差计算失败: {str(e)}')
                continue
        
        if len(translation_errors) == 0:
            raise ValueError('无法计算误差，至少需要1个有效的姿态对')
        
        # 计算RMS误差
        mean_translation_error = float(np.sqrt(np.mean([e**2 for e in translation_errors])))
        mean_rotation_error = float(np.sqrt(np.mean([e**2 for e in rotation_errors])))
        max_translation_error = float(np.max(translation_errors))
        min_translation_error = float(np.min(translation_errors))
        std_translation_error = float(np.std(translation_errors))
        max_rotation_error = float(np.max(rotation_errors))
        min_rotation_error = float(np.min(rotation_errors))
        
        error_statistics = {
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
            'rotation_errors_rad': rotation_errors,
            'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
        }
        
        self._log('info', f'误差计算完成：平移RMS={mean_translation_error:.3f}mm，旋转RMS={np.degrees(mean_rotation_error):.3f}°')
        
        return translation_errors, rotation_errors, error_statistics
    
    def _rotation_matrix_to_euler_deg(self, R):
        """将旋转矩阵转换为欧拉角（度）"""
        try:
            # 使用OpenCV的旋转矩阵到欧拉角转换
            rvec, _ = cv2.Rodrigues(R)
            angle = np.linalg.norm(rvec)
            if angle < 1e-6:
                return [0.0, 0.0, 0.0]
            axis = rvec.flatten() / angle
            # 转换为欧拉角（ZYX顺序）
            from scipy.spatial.transform import Rotation as R_scipy
            r = R_scipy.from_rotvec(axis * angle)
            euler = r.as_euler('zyx', degrees=True)
            return euler.tolist()
        except:
            return [0.0, 0.0, 0.0]
    
    def calibrate(self, poses_data, opencv_algorithm='TSAI'):
        """
        OpenCV模式完整标定流程
        
        参数:
            poses_data: 位姿列表，每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                - robot_pose: 机器人位姿，包含robot_pos_x/y/z（单位：米）和robot_ori_x/y/z/w（四元数）
                - board_pose: 标定板位姿，包含position（x/y/z，单位：毫米）和orientation（x/y/z/w，四元数）
            opencv_algorithm: OpenCV算法选择，可选值: 'TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS'（默认: 'TSAI'）
        
        返回:
            dict: 包含标定结果和误差统计的字典
        """
        if not poses_data or len(poses_data) == 0:
            raise ValueError('位姿列表为空，至少需要1个位姿')
        
        self._log('info', f'开始OpenCV模式完整标定流程：输入 {len(poses_data)} 个位姿')
        
        # 验证标定板Z值与机械臂末端Z值的物理一致性
        z_consistency_check = []
        for i, pose in enumerate(poses_data):
            robot_pose = pose.get('robot_pose', {})
            board_pose = pose.get('board_pose', {})
            
            # 机械臂Z值（米转毫米）
            robot_z_m = robot_pose.get('robot_pos_z', 0.0)
            robot_z_mm = robot_z_m * 1000.0
            
            # 标定板Z值（已经是毫米，solvePnP返回的tvec[2]）
            board_z_mm = board_pose.get('position', {}).get('z', 0.0)
            
            # 计算差异
            diff_mm = abs(robot_z_mm - board_z_mm)
            diff_pct = (diff_mm / robot_z_mm * 100.0) if robot_z_mm > 0 else 0.0
            
            z_consistency_check.append({
                'pose_idx': i + 1,
                'robot_z_mm': robot_z_mm,
                'board_z_mm': board_z_mm,
                'diff_mm': diff_mm,
                'diff_pct': diff_pct,
                'is_consistent': diff_mm < 200.0  # 差异小于200mm认为一致
            })
        
        # 计算统计信息
        avg_diff = sum(d['diff_mm'] for d in z_consistency_check) / len(z_consistency_check) if z_consistency_check else 0.0
        max_diff = max(d['diff_mm'] for d in z_consistency_check) if z_consistency_check else 0.0
        min_diff = min(d['diff_mm'] for d in z_consistency_check) if z_consistency_check else 0.0
        consistent_count = sum(1 for d in z_consistency_check if d['is_consistent'])
        
        # 如果Z值差异过大，记录警告
        if avg_diff > 200.0:
            self._log('warning', f'⚠️ Z值一致性警告：平均差异 {avg_diff:.2f}mm > 200mm，可能存在坐标系或单位问题')
        elif avg_diff > 100.0:
            self._log('info', f'💡 Z值一致性提示：平均差异 {avg_diff:.2f}mm，在可接受范围内')
        else:
            self._log('info', f'✅ Z值一致性良好：平均差异 {avg_diff:.2f}mm < 100mm')
        
        # 步骤1：数据准备
        R_gripper2base, t_gripper2base, rvecs, tvecs, T_gripper_list, T_board_list = self.prepare_data(poses_data)
        
        # 步骤2：算法计算
        T_camera2gripper = self.solve_hand_eye(R_gripper2base, t_gripper2base, rvecs, tvecs, opencv_algorithm=opencv_algorithm)
        
        # 步骤3：误差计算
        translation_errors, rotation_errors, error_statistics = self.calculate_errors(T_gripper_list, T_board_list, T_camera2gripper)
        
        return {
            'T_camera2gripper': T_camera2gripper,
            'translation_errors': translation_errors,
            'rotation_errors': rotation_errors,
            'error_statistics': error_statistics,
            'T_gripper_list': T_gripper_list,
            'T_board_list': T_board_list,
            'session_id': self.session_id,
            'session_timestamp': self.session_timestamp,
            'config_dir': self.config_dir
        }
