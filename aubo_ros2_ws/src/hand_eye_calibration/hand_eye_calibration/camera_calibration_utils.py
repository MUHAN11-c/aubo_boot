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

