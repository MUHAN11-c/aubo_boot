#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
相机标定工具函数
包含读取XML参数、角点检测、坐标计算等功能
"""

import cv2
import numpy as np
import xml.etree.ElementTree as ET
from typing import Tuple, List, Dict, Optional


class CameraCalibrationUtils:
    """相机标定工具类"""
    
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        self.detected_pattern_size = None  # 检测到的实际棋盘格尺寸
        self.depth_scale_unit = None  # 深度图缩放因子（f_scale_unit），如果为None则自动推断
        # 注意：参考 depth_z_reader 的实现
        # - 对于 scale_unit=0.25 的相机（如 PS800-E1）：
        #   * 转换为米：depth_raw * 0.00025（depth_z_reader 使用此方式）
        #   * 转换为毫米：depth_raw * 0.25（hand_eye_calibration 使用此方式，因为需要毫米单位）
        # - 对于 scale_unit=1.0 的相机：
        #   * 转换为米：depth_raw * 0.001
        #   * 转换为毫米：depth_raw * 1.0
        # 查看相机启动日志中的 "Depth stream scale unit" 来确定正确的值
        
    def load_camera_params(self, xml_file: str) -> Dict:
        """
        加载相机标定参数从XML文件
        
        Args:
            xml_file: XML文件路径
            
        Returns:
            参数字典
        """
        try:
            tree = ET.parse(xml_file)
            root = tree.getroot()
            
            # 读取图像尺寸
            image_size_node = root.find('.//ImageSize/data')
            if image_size_node is not None:
                size_str = image_size_node.text.strip()
                self.image_size = tuple(map(int, size_str.split()))
            
            # 读取相机矩阵
            camera_matrix_node = root.find('.//CameraMatrix/data')
            if camera_matrix_node is not None:
                data_str = camera_matrix_node.text.strip()
                data = list(map(float, data_str.split()))
                self.camera_matrix = np.array(data).reshape(3, 3)
            
            # 读取畸变系数
            dist_coeffs_node = root.find('.//DistortionCoefficients/data')
            if dist_coeffs_node is not None:
                data_str = dist_coeffs_node.text.strip()
                data = list(map(float, data_str.split()))
                self.dist_coeffs = np.array(data)
            
            # 读取重投影误差
            mean_error_node = root.find('.//MeanReprojectionError')
            mean_error = float(mean_error_node.text) if mean_error_node is not None else 0.0
            
            # 提取fx, fy, cx, cy
            fx = self.camera_matrix[0, 0] if self.camera_matrix is not None else 0
            fy = self.camera_matrix[1, 1] if self.camera_matrix is not None else 0
            cx = self.camera_matrix[0, 2] if self.camera_matrix is not None else 0
            cy = self.camera_matrix[1, 2] if self.camera_matrix is not None else 0
            
            result = {
                'success': True,
                'image_size': self.image_size,
                'camera_matrix': {
                    'fx': fx,
                    'fy': fy,
                    'cx': cx,
                    'cy': cy
                },
                'dist_coeffs': self.dist_coeffs.tolist() if self.dist_coeffs is not None else [],
                'mean_reprojection_error': mean_error
            }
            
            return result
            
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
    
    def detect_checkerboard_corners(self, image: np.ndarray, pattern_size: Tuple[int, int] = (10, 7)) -> Optional[np.ndarray]:
        """
        检测棋盘格角点
        
        Args:
            image: 输入图像
            pattern_size: 棋盘格内角点数量 (列数, 行数)
            
        Returns:
            角点坐标数组 或 None
        """
        # 转为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 尝试多种模式检测角点
        flags_list = [
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
            cv2.CALIB_CB_ADAPTIVE_THRESH,
            None  # 不使用任何标志
        ]
        
        ret = False
        corners = None
        
        for flags in flags_list:
            if flags is None:
                ret, corners = cv2.findChessboardCorners(gray, pattern_size)
            else:
                ret, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
            
            if ret:
                # 亚像素精化
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                return corners
        
        # 如果都失败，尝试不同的棋盘格尺寸
        alternative_sizes = [
            (9, 6), (8, 6), (7, 5), (11, 8), (9, 7)
        ]
        
        for alt_size in alternative_sizes:
            for flags in flags_list[:2]:  # 只尝试前两种模式
                if flags is None:
                    ret, corners = cv2.findChessboardCorners(gray, alt_size)
                else:
                    ret, corners = cv2.findChessboardCorners(gray, alt_size, flags)
                
                if ret:
                    print(f"检测到棋盘格尺寸: {alt_size}")
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    # 更新pattern_size
                    self.detected_pattern_size = alt_size
                    return corners
        
        return None
    
    def pixel_to_camera_coords(self, pixel_points: np.ndarray, z_camera: float) -> np.ndarray:
        """
        将像素坐标转换为相机坐标系坐标
        
        Args:
            pixel_points: 像素坐标 (N, 2)
            z_camera: 相机坐标系下的Z坐标（深度），单位：mm
            
        Returns:
            相机坐标系坐标 (N, 3)
        """
        if self.camera_matrix is None:
            raise ValueError("Camera matrix not loaded")
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # 去畸变
        if self.dist_coeffs is not None:
            pixel_points = cv2.undistortPoints(
                pixel_points.reshape(-1, 1, 2), 
                self.camera_matrix, 
                self.dist_coeffs, 
                P=self.camera_matrix
            ).reshape(-1, 2)
        
        # 转换到相机坐标系
        camera_coords = np.zeros((len(pixel_points), 3))
        camera_coords[:, 0] = (pixel_points[:, 0] - cx) * z_camera / fx
        camera_coords[:, 1] = (pixel_points[:, 1] - cy) * z_camera / fy
        camera_coords[:, 2] = z_camera
        
        return camera_coords
    
    def calculate_adjacent_distances(self, points_3d: np.ndarray, pattern_size: Tuple[int, int]) -> List[float]:
        """
        计算相邻角点之间的距离
        
        Args:
            points_3d: 3D点坐标 (N, 3)
            pattern_size: 棋盘格大小 (列数, 行数)
            
        Returns:
            相邻距离列表
        """
        cols, rows = pattern_size
        distances = []
        
        for i in range(len(points_3d)):
            row = i // cols
            col = i % cols
            
            # 计算到右侧相邻点的距离
            if col < cols - 1:
                neighbor_idx = i + 1
                dist = np.linalg.norm(points_3d[i] - points_3d[neighbor_idx])
                distances.append(dist)
            else:
                # 最后一列没有右侧相邻点
                if distances:
                    distances.append(distances[-1])  # 使用上一个距离
                else:
                    distances.append(0.0)
        
        return distances
    
    def estimate_checkerboard_z(self, corners: np.ndarray, square_size: float, 
                                  pattern_size: Tuple[int, int]) -> float:
        """
        估计棋盘格到相机的距离（Z坐标）
        
        Args:
            corners: 角点像素坐标
            square_size: 棋盘格大小（mm）
            pattern_size: 棋盘格大小 (列数, 行数)
            
        Returns:
            估计的Z坐标（mm）
        """
        if self.camera_matrix is None:
            raise ValueError("Camera matrix not loaded")
        
        # 构建3D物体点（假设在Z=0平面上）
        cols, rows = pattern_size
        obj_points = np.zeros((rows * cols, 3), np.float32)
        obj_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj_points *= square_size
        
        # 使用solvePnP估计位姿
        corners_2d = corners.reshape(-1, 1, 2).astype(np.float32)
        obj_points_3d = obj_points.astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points_3d,
            corners_2d,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        if success:
            # tvec[2]是Z坐标
            return abs(float(tvec[2]))
        
        # 如果失败，使用简单估计
        # 基于两个角点之间的像素距离和实际距离
        pixel_dist = np.linalg.norm(corners[1] - corners[0])
        f = (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2
        z_est = f * square_size / pixel_dist
        
        return z_est
    
    def draw_corners(self, image: np.ndarray, corners: np.ndarray, 
                      pattern_size: Tuple[int, int]) -> np.ndarray:
        """
        在图像上绘制检测到的角点
        
        Args:
            image: 输入图像
            corners: 角点坐标
            pattern_size: 棋盘格大小
            
        Returns:
            绘制了角点的图像
        """
        img_with_corners = image.copy()
        cv2.drawChessboardCorners(img_with_corners, pattern_size, corners, True)
        
        # 添加角点序号
        for i, corner in enumerate(corners):
            pt = tuple(corner.ravel().astype(int))
            cv2.putText(img_with_corners, str(i), pt, 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        return img_with_corners
    
    def calculate_max_distance_error(self, points_3d: np.ndarray, 
                                       pattern_size: Tuple[int, int],
                                       expected_square_size: float) -> Dict:
        """
        计算最远相邻角点距离并与期望值对比
        
        Args:
            points_3d: 3D点坐标
            pattern_size: 棋盘格大小
            expected_square_size: 期望的棋盘格大小（mm）
            
        Returns:
            包含最大距离、误差等信息的字典
        """
        cols, rows = pattern_size
        
        # 计算所有水平相邻距离
        horizontal_distances = []
        for i in range(len(points_3d)):
            col = i % cols
            if col < cols - 1:
                neighbor_idx = i + 1
                dist = np.linalg.norm(points_3d[i] - points_3d[neighbor_idx])
                horizontal_distances.append(dist)
        
        # 计算所有垂直相邻距离
        vertical_distances = []
        for i in range(len(points_3d) - cols):
            dist = np.linalg.norm(points_3d[i] - points_3d[i + cols])
            vertical_distances.append(dist)
        
        all_distances = horizontal_distances + vertical_distances
        
        if not all_distances:
            return {
                'success': False,
                'error': '无法计算距离'
            }
        
        max_dist = max(all_distances)
        min_dist = min(all_distances)
        avg_dist = np.mean(all_distances)
        std_dist = np.std(all_distances)
        
        # 计算误差
        error_max = abs(max_dist - expected_square_size)
        error_avg = abs(avg_dist - expected_square_size)
        error_percent = (error_avg / expected_square_size) * 100
        
        # 计算第一个角点到最后一个角点的对角线距离（整体精度验证）
        first_point = points_3d[0]  # 左上角
        last_point = points_3d[-1]  # 右下角
        diagonal_distance = np.linalg.norm(last_point - first_point)
        
        # 理论对角线距离 = sqrt((cols-1)^2 + (rows-1)^2) * square_size
        diagonal_theoretical = np.sqrt((cols - 1)**2 + (rows - 1)**2) * expected_square_size
        diagonal_error = abs(diagonal_distance - diagonal_theoretical)
        diagonal_error_percent = (diagonal_error / diagonal_theoretical) * 100
        
        return {
            'success': True,
            'max_distance': max_dist,
            'min_distance': min_dist,
            'avg_distance': avg_dist,
            'std_distance': std_dist,
            'expected_distance': expected_square_size,
            'error_max': error_max,
            'error_avg': error_avg,
            'error_percent': error_percent,
            # 新增：对角线距离数据
            'diagonal_distance': diagonal_distance,
            'diagonal_theoretical': diagonal_theoretical,
            'diagonal_error': diagonal_error,
            'diagonal_error_percent': diagonal_error_percent,
            'pattern_size': pattern_size  # 返回棋盘格尺寸供前端显示
        }
    
    def get_depth_from_depth_image(self, depth_image: np.ndarray, pixel_u: int, pixel_v: int, 
                                    search_radius: int = 3) -> Optional[float]:
        """
        从深度图中获取指定像素位置的深度值（深度图已对齐到彩色图）
        
        参考 depth_z_reader 实现，改进了无效深度值检测和搜索算法。
        
        Args:
            depth_image: OpenCV深度图（numpy数组，16位无符号整数）
            pixel_u: 像素U坐标（列）
            pixel_v: 像素V坐标（行）
            search_radius: 搜索半径（像素），在指定位置周围圆形区域内搜索有效点
            
        Returns:
            深度值（mm），如果未找到则返回None
            
        注意：
            - 参考 depth_z_reader 的实现方式
            - 深度图像的原始值需要乘以 depth_scale_unit 才能得到毫米值
              * 对于 scale_unit=0.25 的相机：depth_scale_unit = 0.25（转换为毫米）
              * 对于 scale_unit=1.0 的相机：depth_scale_unit = 1.0（转换为毫米）
            - 无效深度值：0 或 65535（16位无符号整数的最大值）
            - 使用圆形搜索区域，从内到外搜索，找到第一个有效值即返回
        """
        try:
            if depth_image is None:
                return None
            
            height, width = depth_image.shape[:2]
            
            # 检查像素坐标是否在有效范围内
            if pixel_u < 0 or pixel_u >= width or pixel_v < 0 or pixel_v >= height:
                return None
            
            # 使用圆形搜索区域（参考 depth_z_reader 的实现）
            # 从内到外搜索，找到第一个有效深度值即返回
            for r in range(0, search_radius + 1):
                depths = []
                for du in range(-r, r + 1):
                    for dv in range(-r, r + 1):
                        # 检查是否在圆形范围内
                        if du * du + dv * dv > r * r:
                            continue
                        
                        u = pixel_u + du
                        v = pixel_v + dv
                        
                        if u < 0 or u >= width or v < 0 or v >= height:
                            continue
                        
                        # 读取深度值（原始像素值，16位无符号整数）
                        depth_value = depth_image[v, u]
                        
                        # 检查是否为有效深度值（0 和 65535 表示无效）
                        # 参考 depth_z_reader: depth_raw == 0 || depth_raw == 65535
                        if depth_value > 0 and depth_value < 65535:
                            depths.append(float(depth_value))
                
                # 如果找到有效深度值，返回中位数（更鲁棒）
                if len(depths) > 0:
                    median_depth_raw = float(np.median(depths))
                    
                    # #region agent log
                    # 记录深度图的原始像素值统计信息（仅前几次调用）
                    if not hasattr(CameraCalibrationUtils, '_depth_raw_log_count'):
                        CameraCalibrationUtils._depth_raw_log_count = 0
                    if CameraCalibrationUtils._depth_raw_log_count < 3:
                        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                        import json
                        import time
                        log_file.write(json.dumps({'id': 'log_depth_raw_stats', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:get_depth_from_depth_image', 'message': '深度图原始值统计', 'data': {'pixel_u': pixel_u, 'pixel_v': pixel_v, 'search_radius': r, 'num_samples': len(depths), 'raw_min': float(min(depths)), 'raw_max': float(max(depths)), 'raw_median': float(median_depth_raw), 'raw_mean': float(np.mean(depths))}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'U'}) + '\n')
                        log_file.close()
                        CameraCalibrationUtils._depth_raw_log_count += 1
                    # #endregion
                    
                    # 应用深度缩放因子（参考 depth_z_reader 的实现方式）
                    # depth_z_reader: depth_z = depth_raw * depth_scale (0.00025 for meters)
                    # hand_eye_calibration: depth_mm = depth_raw * depth_scale_unit (0.25 for mm)
                    # 对于 scale_unit=0.25 的相机：depth_scale_unit = 0.25（转换为毫米）
                    # 如果设置了深度缩放因子，应用它转换为毫米
                    if self.depth_scale_unit is not None:
                        return median_depth_raw * self.depth_scale_unit
                    
                    # 否则返回原始值（假设单位已经是毫米，或者后续会自动推断）
                    return median_depth_raw
            
            # 未找到有效深度值
            return None
            
        except Exception as e:
            # #region agent log
            log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
            import json
            import time
            log_file.write(json.dumps({'id': 'log_get_depth_image_exception', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:365', 'message': 'get_depth_from_depth_image异常', 'data': {'error': str(e), 'pixel_u': pixel_u, 'pixel_v': pixel_v}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'Q'}) + '\n')
            log_file.close()
            # #endregion
            return None
    
    def get_depth_from_point_cloud(self, point_cloud_msg, pixel_u: int, pixel_v: int, 
                                    search_radius: int = 3) -> Optional[float]:
        """
        从点云中获取指定像素位置的深度值
        
        Args:
            point_cloud_msg: sensor_msgs/PointCloud2 消息
            pixel_u: 像素U坐标（列）
            pixel_v: 像素V坐标（行）
            search_radius: 搜索半径（像素），在指定位置周围搜索有效点
            
        Returns:
            深度值（mm），如果未找到则返回None
        """
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            
            # 获取点云宽度和高度
            width = point_cloud_msg.width
            height = point_cloud_msg.height
            
            # 检查点云是否有序（height > 1表示有序点云，height == 1表示无序点云）
            is_organized = height > 1
            
            if is_organized:
                # 有序点云：使用uvs参数按像素位置提取
                # 检查像素坐标是否在有效范围内
                if pixel_u < 0 or pixel_u >= width or pixel_v < 0 or pixel_v >= height:
                    return None
                
                # 在指定位置周围搜索有效点
                depths = []
                for du in range(-search_radius, search_radius + 1):
                    for dv in range(-search_radius, search_radius + 1):
                        u = pixel_u + du
                        v = pixel_v + dv
                        
                        if u < 0 or u >= width or v < 0 or v >= height:
                            continue
                        
                        # 获取点
                        try:
                            point_gen = pc2.read_points(point_cloud_msg, 
                                                        field_names=("x", "y", "z"),
                                                        skip_nans=True,
                                                        uvs=[(u, v)])
                            for point in point_gen:
                                x, y, z = point
                                # z是深度值（单位：米），转换为毫米
                                if not np.isnan(z) and z > 0:
                                    depths.append(z * 1000.0)  # 转换为mm
                        except:
                            continue
            else:
                # 无序点云：需要将像素坐标转换为相机坐标系，然后找最近的点
                # 首先需要相机内参来转换像素坐标到相机坐标系
                if self.camera_matrix is None:
                    return None
                
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                
                # 从点云中读取所有点
                depths = []
                try:
                    point_gen = pc2.read_points(point_cloud_msg, 
                                                field_names=("x", "y", "z"),
                                                skip_nans=True)
                    
                    for point in point_gen:
                        x, y, z = point
                        if np.isnan(z) or z <= 0:
                            continue
                        
                        # 将3D点投影到图像平面（点云坐标单位是米）
                        if abs(z) > 1e-6:  # 避免除零
                            u_proj = x / z * fx + cx
                            v_proj = y / z * fy + cy
                            
                            # 计算像素距离
                            pixel_dist = np.sqrt((u_proj - pixel_u)**2 + (v_proj - pixel_v)**2)
                            
                            # 如果距离在搜索半径内，记录深度
                            if pixel_dist <= search_radius:
                                depths.append(z * 1000.0)  # 转换为mm
                    
                except Exception as e:
                    # #region agent log
                    log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                    import json
                    import time
                    log_file.write(json.dumps({'id': 'log_pc2_read_error', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:508', 'message': '读取点云失败', 'data': {'error': str(e), 'pixel_u': pixel_u, 'pixel_v': pixel_v}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'M'}) + '\n')
                    log_file.close()
                    # #endregion
                    return None
            
            if len(depths) == 0:
                return None
            
            # 返回中位数深度（更鲁棒）
            return float(np.median(depths))
            
        except Exception as e:
            # #region agent log
            log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
            import json
            import time
            log_file.write(json.dumps({'id': 'log_get_depth_exception', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:435', 'message': 'get_depth_from_point_cloud异常', 'data': {'error': str(e), 'pixel_u': pixel_u, 'pixel_v': pixel_v}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'N'}) + '\n')
            log_file.close()
            # #endregion
            return None
    
    def evaluate_depth_error(self, corners: np.ndarray, depth_data, 
                             pattern_size: Tuple[int, int],
                             square_size: float) -> Dict:
        """
        评估深度误差：对比估计深度和深度相机实际深度
        
        Args:
            corners: 角点像素坐标 (N, 2)
            depth_data: 深度图（numpy数组，16位无符号整数，单位：毫米）或PointCloud2消息（向后兼容）
            pattern_size: 棋盘格大小 (列数, 行数)
            square_size: 棋盘格大小（mm）
            
        Returns:
            包含深度误差统计的字典
        """
        if depth_data is None:
            return {
                'success': False,  # 使用 Python 原生 bool，避免 JSON 序列化问题
                'error': '深度数据不可用'
            }
        
        if self.camera_matrix is None:
            return {
                'success': False,  # 使用 Python 原生 bool，避免 JSON 序列化问题
                'error': '相机参数未加载'
            }
        
        # 使用solvePnP估计棋盘格姿态，然后为每个角点单独计算估计深度
        # 构建3D物体点（棋盘格坐标系，Z=0平面）
        cols, rows = pattern_size
        obj_points = np.zeros((rows * cols, 3), np.float32)
        obj_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj_points *= square_size
        
        # 使用solvePnP估计位姿
        corners_2d = corners.reshape(-1, 1, 2).astype(np.float32)
        obj_points_3d = obj_points.astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points_3d,
            corners_2d,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        if not success:
            return {
                'success': False,  # 使用 Python 原生 bool，避免 JSON 序列化问题
                'error': 'solvePnP求解失败，无法估计深度'
            }
        
        # 将旋转向量转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)
        
        # 将每个角点的3D坐标从棋盘格坐标系转换到相机坐标系
        # P_camera = R * P_board + tvec
        # 其中P_board是棋盘格坐标系下的3D点，P_camera是相机坐标系下的3D点
        estimated_depths_per_corner = []
        for obj_point in obj_points:
            # 转换为相机坐标系
            obj_point_cam = R @ obj_point.reshape(3, 1) + tvec
            # Z坐标就是深度
            estimated_depths_per_corner.append(float(obj_point_cam[2, 0]))
        
        # 平均估计深度（用于显示）
        mean_estimated_z = np.mean(estimated_depths_per_corner)
        
        # #region agent log
        # 记录solvePnP的tvec值，用于验证估计深度
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        import json
        import time
        log_file.write(json.dumps({'id': 'log_estimate_z_result', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:629', 'message': '深度估计完成', 'data': {'tvec_x': float(tvec[0, 0]), 'tvec_y': float(tvec[1, 0]), 'tvec_z': float(tvec[2, 0]), 'mean_estimated_z': mean_estimated_z, 'min_estimated_z': float(np.min(estimated_depths_per_corner)), 'max_estimated_z': float(np.max(estimated_depths_per_corner)), 'std_estimated_z': float(np.std(estimated_depths_per_corner))}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'K'}) + '\n')
        log_file.close()
        # #endregion
        
        # 从深度图或点云中获取每个角点的实际深度
        # 重新定义corners_2d用于遍历（形状为(N, 2)）
        corners_2d = corners.reshape(-1, 2)
        actual_depths = []
        estimated_depths = []
        depth_errors = []
        depth_details = []
        
        # 判断是深度图还是点云
        is_depth_image = isinstance(depth_data, np.ndarray)
        search_radius = 3  # 深度图使用较小的搜索半径即可
        
        # #region agent log
        # 记录深度图的整体统计信息
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        import json
        import time
        if is_depth_image:
            # 统计深度图的非零值
            valid_pixels = depth_data[depth_data > 0]
            if len(valid_pixels) > 0:
                log_file.write(json.dumps({'id': 'log_depth_image_stats', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:649', 'message': '深度图整体统计', 'data': {'shape': list(depth_data.shape), 'dtype': str(depth_data.dtype), 'valid_pixel_count': int(len(valid_pixels)), 'raw_min': float(np.min(valid_pixels)), 'raw_max': float(np.max(valid_pixels)), 'raw_median': float(np.median(valid_pixels)), 'raw_mean': float(np.mean(valid_pixels)), 'raw_std': float(np.std(valid_pixels))}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'V'}) + '\n')
            log_file.write(json.dumps({'id': 'log_extract_depth_start', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:658', 'message': '开始从深度图提取深度', 'data': {'width': depth_data.shape[1], 'height': depth_data.shape[0], 'dtype': str(depth_data.dtype), 'search_radius': search_radius, 'total_corners': len(corners_2d)}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'O'}) + '\n')
        else:
            # 向后兼容：点云数据
            is_organized = depth_data.height > 1
            search_radius = 3 if is_organized else 15
            log_file.write(json.dumps({'id': 'log_extract_depth_start', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:664', 'message': '开始从点云提取深度', 'data': {'is_organized': is_organized, 'width': depth_data.width, 'height': depth_data.height, 'search_radius': search_radius, 'total_corners': len(corners_2d)}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'O'}) + '\n')
        log_file.close()
        # #endregion
        
        # 统计信息
        total_corners = len(corners_2d)
        no_depth_count = 0
        filtered_count = 0
        valid_count = 0
        
        # 自动推断深度缩放因子（如果未设置）
        # 通过对比前几个角点的估计深度和实际深度来推断
        if is_depth_image and self.depth_scale_unit is None and len(corners_2d) > 0:
            # 尝试从前几个角点推断缩放因子
            # 直接读取原始深度值（不通过get_depth_from_depth_image，因为它会应用缩放因子）
            sample_corners = min(5, len(corners_2d))
            sample_ratios = []
            height, width = depth_data.shape[:2]
            
            for i in range(sample_corners):
                pixel_u = int(round(float(corners_2d[i][0])))
                pixel_v = int(round(float(corners_2d[i][1])))
                
                # 直接读取原始深度值（使用圆形搜索，参考 depth_z_reader）
                raw_depths = []
                for r in range(0, search_radius + 1):
                    for du in range(-r, r + 1):
                        for dv in range(-r, r + 1):
                            # 检查是否在圆形范围内
                            if du * du + dv * dv > r * r:
                                continue
                            
                            u = pixel_u + du
                            v = pixel_v + dv
                            if 0 <= u < width and 0 <= v < height:
                                depth_value = depth_data[v, u]
                                # 检查是否为有效深度值（0 和 65535 表示无效）
                                if depth_value > 0 and depth_value < 65535:
                                    raw_depths.append(float(depth_value))
                    
                    # 如果找到有效深度值，跳出循环
                    if len(raw_depths) > 0:
                        break
                
                if len(raw_depths) > 0 and estimated_depths_per_corner[i] > 0:
                    actual_depth_raw = float(np.median(raw_depths))
                    ratio = actual_depth_raw / estimated_depths_per_corner[i]
                    if 0.1 < ratio < 10.0:  # 合理的比例范围
                        sample_ratios.append(ratio)
            
            if len(sample_ratios) > 0:
                # 如果比例稳定（都在0.2-0.4范围内），说明需要缩放
                median_ratio = np.median(sample_ratios)
                if 0.2 < median_ratio < 0.5:  # 典型的原始像素值/毫米比例
                    inferred_scale = 1.0 / median_ratio
                    self.depth_scale_unit = inferred_scale
                    # #region agent log
                    log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                    import json
                    import time
                    log_file.write(json.dumps({'id': 'log_depth_scale_inferred', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:687', 'message': '自动推断深度缩放因子', 'data': {'sample_ratios': [float(r) for r in sample_ratios], 'median_ratio': float(median_ratio), 'inferred_scale_unit': float(inferred_scale)}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'W'}) + '\n')
                    log_file.close()
                    # #endregion
        
        for i, corner in enumerate(corners_2d):
            pixel_u = int(round(float(corner[0])))
            pixel_v = int(round(float(corner[1])))
            
            # 从深度图或点云获取实际深度
            if is_depth_image:
                actual_depth = self.get_depth_from_depth_image(
                    depth_data, pixel_u, pixel_v, search_radius
                )
            else:
                # 向后兼容：使用点云
                actual_depth = self.get_depth_from_point_cloud(
                    depth_data, pixel_u, pixel_v, search_radius
                )
            
            if actual_depth is None:
                no_depth_count += 1
                # #region agent log
                log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                log_file.write(json.dumps({'id': 'log_no_depth_value', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:655', 'message': '角点无深度值', 'data': {'corner_index': i, 'pixel_u': pixel_u, 'pixel_v': pixel_v}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'S'}) + '\n')
                log_file.close()
                # #endregion
                continue
            
            # 使用该角点对应的估计深度（考虑棋盘格倾斜）
            estimated_depth = estimated_depths_per_corner[i]
            
            # #region agent log
            if i < 5:  # 记录前几个角点的详细信息
                log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                import json
                import time
                log_file.write(json.dumps({'id': 'log_depth_comparison', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:690', 'message': '深度对比', 'data': {'corner_index': i, 'pixel_u': pixel_u, 'pixel_v': pixel_v, 'estimated_depth': estimated_depth, 'actual_depth': actual_depth, 'ratio': actual_depth / estimated_depth if estimated_depth > 0 else 0}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'N'}) + '\n')
                log_file.close()
            # #endregion
            
            # 过滤异常深度值（深度值应该在合理范围内）
            # 如果实际深度与估计深度差异过大，可能是深度图噪声，跳过
            # 放宽阈值到0.05-20.0，以允许更大的误差范围（因为可能存在深度图单位或标定问题）
            # 注意：由于深度图单位可能存在问题，暂时放宽过滤条件以显示深度误差
            DEPTH_RATIO_MIN = 0.05  # 最小深度比例阈值
            DEPTH_RATIO_MAX = 20.0  # 最大深度比例阈值
            if estimated_depth > 0:
                depth_ratio = actual_depth / estimated_depth
                should_filter = bool(depth_ratio < DEPTH_RATIO_MIN or depth_ratio > DEPTH_RATIO_MAX)  # 转换为 Python bool 类型，避免 JSON 序列化错误
                # #region agent log
                if i < 3:  # 记录前几个点的详细信息用于调试
                    log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                    import json
                    import time
                    log_file.write(json.dumps({'id': 'log_depth_filter_check', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:697', 'message': '深度过滤检查', 'data': {'corner_index': i, 'depth_ratio': float(depth_ratio), 'min_threshold': DEPTH_RATIO_MIN, 'max_threshold': DEPTH_RATIO_MAX, 'should_filter': should_filter}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'T'}) + '\n')
                    log_file.close()
                # #endregion
                if should_filter:  # 实际深度应该在估计深度的5%-2000%范围内（非常宽松的阈值）
                    filtered_count += 1
                    # #region agent log
                    log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                    import json
                    import time
                    log_file.write(json.dumps({'id': 'log_depth_outlier_filtered', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:705', 'message': '过滤异常深度值', 'data': {'corner_index': i, 'pixel_u': pixel_u, 'pixel_v': pixel_v, 'estimated_depth': estimated_depth, 'actual_depth': actual_depth, 'depth_ratio': depth_ratio, 'min_threshold': DEPTH_RATIO_MIN, 'max_threshold': DEPTH_RATIO_MAX}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'R'}) + '\n')
                    log_file.close()
                    # #endregion
                    continue  # 跳过异常值
            
            valid_count += 1
            
            # 计算深度误差
            depth_error = abs(estimated_depth - actual_depth)
            relative_error_percent = (depth_error / actual_depth * 100) if actual_depth > 0 else 0
            
            actual_depths.append(actual_depth)
            estimated_depths.append(estimated_depth)
            depth_errors.append(depth_error)
            
            depth_details.append({
                'corner_index': i,
                'pixel_u': float(pixel_u),
                'pixel_v': float(pixel_v),
                'estimated_depth': float(estimated_depth),
                'actual_depth': float(actual_depth),
                'depth_error': float(depth_error),
                'relative_error_percent': float(relative_error_percent)
            })
        
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        log_file.write(json.dumps({'id': 'log_depth_extraction_result', 'timestamp': int(time.time()*1000), 'location': 'camera_calibration_utils.py:730', 'message': '深度提取结果统计', 'data': {'total_corners': total_corners, 'valid_points': len(actual_depths), 'no_depth_count': no_depth_count, 'filtered_count': filtered_count, 'valid_count': valid_count}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'L'}) + '\n')
        log_file.close()
        # #endregion
        
        if len(actual_depths) == 0:
            return {
                'success': False,  # 使用 Python 原生 bool，避免 JSON 序列化问题
                'error': f'无法从深度数据中获取有效深度值（总角点数: {total_corners}, 无深度值: {no_depth_count}, 过滤异常值: {filtered_count}）'
            }
        
        # 计算统计信息
        mean_actual_depth = np.mean(actual_depths)
        std_actual_depth = np.std(actual_depths)
        mean_estimated_depth = np.mean(estimated_depths)
        mean_depth_error = np.mean(depth_errors)
        max_depth_error = np.max(depth_errors)
        min_depth_error = np.min(depth_errors)
        std_depth_error = np.std(depth_errors)
        
        # 计算相对误差
        mean_relative_error = (mean_depth_error / mean_actual_depth * 100) if mean_actual_depth > 0 else 0
        
        return {
            'success': True,  # 使用 Python 原生 bool，避免 JSON 序列化问题
            'num_valid_points': int(len(actual_depths)),
            'total_corners': int(total_corners),  # 总角点数
            'no_depth_count': int(no_depth_count),  # 无深度值的角点数
            'filtered_count': int(filtered_count),  # 被过滤的异常值数量
            'estimated_depth': float(mean_estimated_z),  # 平均估计深度（用于显示）
            'depth_statistics': {
                'mean_actual': float(mean_actual_depth),
                'std_actual': float(std_actual_depth),
                'mean_estimated': float(mean_estimated_depth),
                'mean_error': float(mean_depth_error),
                'max_error': float(max_depth_error),
                'min_error': float(min_depth_error),
                'std_error': float(std_depth_error),
                'mean_relative_error_percent': float(mean_relative_error)
            },
            'depth_details': depth_details,
            'evaluation': {
                'excellent': bool(mean_depth_error < 2.0),  # 平均误差 < 2mm，转换为 Python bool
                'good': bool(mean_depth_error < 5.0),       # 平均误差 < 5mm，转换为 Python bool
                'acceptable': bool(mean_depth_error < 10.0),  # 平均误差 < 10mm，转换为 Python bool
                'poor': bool(mean_depth_error >= 10.0)       # 平均误差 >= 10mm，转换为 Python bool
            }
        }

