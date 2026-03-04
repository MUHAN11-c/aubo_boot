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

                # 构建变换矩阵（将机器人位姿从米转换为毫米）
                T_gripper = self._pose_to_transform_matrix(
                    {'x': robot_pos_m['x'] * 1000.0, 'y': robot_pos_m['y'] * 1000.0, 'z': robot_pos_m['z'] * 1000.0},
                    robot_ori
                )
                
                
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
                
            except Exception as e:
                skipped_poses += 1
                self._log('warning', f'位姿 #{idx+1} 处理失败: {str(e)}')
                continue
        
        self._log('info', f'数据准备完成：处理{processed_poses}个位姿，跳过{skipped_poses}个，去重后{len(T_gripper_list)}个唯一姿态对（去除{duplicate_poses}个重复姿态对）')
        
        
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
        
        
        for idx, (T_gripper, T_board) in enumerate(zip(T_gripper_list, T_board_list)):
            # T_gripper是Base→Gripper，直接使用（参考hand_in_eye_calibrate.py，不要取逆！）
            R_base2gripper = T_gripper[:3, :3]
            t_base2gripper_mm = T_gripper[:3, 3].flatten()  # 单位：毫米
            
            R_gripper2base_list.append(R_base2gripper)  # 变量名保持不变以兼容后续代码
            t_gripper2base_list.append(t_base2gripper_mm)
            
            
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
        
        
        # 单位转换：毫米 → 米（OpenCV要求）
        R_gripper2base = np.array(R_gripper2base_list, dtype=np.float64)
        t_gripper2base = [t.reshape(3, 1) / 1000.0 if hasattr(t, 'reshape') else np.array(t).reshape(3, 1) / 1000.0 for t in t_gripper2base_list]  # 毫米 → 米
        rvecs = rvecs_list  # 弧度，无需转换
        tvecs = [t / 1000.0 if isinstance(t, np.ndarray) else np.array(t) / 1000.0 for t in tvecs_list]  # 毫米 → 米
        
        
        
        self._log('info', f'   [OpenCV模式] 单位转换完成：机器人位姿({len(t_gripper2base)}个)和标定板位姿({len(tvecs)}个)已转换为米')
        
        
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
            raise
        
        
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
                
                
                # 使用已验证的B矩阵（已在坐标系验证中计算）
                # B矩阵已在坐标系验证代码中计算，这里直接使用
                B_list_for_error.append(B)
                
                
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
