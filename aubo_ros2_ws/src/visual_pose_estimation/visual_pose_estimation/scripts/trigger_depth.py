#!/usr/bin/env python3
"""
深度图像显示脚本

功能：
1. 订阅深度图像话题并显示
2. 支持深度图像二值化和轮廓检测
3. 在彩色图像上显示识别框

使用方法：
    python3 trigger_depth.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from percipio_camera_interface.msg import CameraStatus
import cv2
from cv_bridge import CvBridge
import threading
import time
import numpy as np
import json
import os
from pathlib import Path


class ImageProcessor:
    """OpenCV图像处理器：处理所有图像显示和UI逻辑"""
    
    def __init__(self, camera_name='camera', depth_scale=0.25, logger=None,
                 enable_depth_processing=True, enable_zero_interp=True):
        """
        初始化图像处理器
        
        参数:
            camera_name: 相机名称
            depth_scale: 深度缩放因子
            logger: 日志记录器（可选）
            enable_depth_processing: 是否启用深度图处理（默认True）
            enable_zero_interp: 是否启用0值插值处理（默认True）
        """
        self.camera_name = camera_name
        self.depth_scale = depth_scale
        self.logger = logger
        
        # 深度图处理配置
        self.enable_depth_processing = enable_depth_processing
        self.enable_zero_interp = enable_zero_interp
        
        self.latest_depth_image = None
        self.latest_color_image = None
        self.detected_bbox = None
        self.extracted_features = None
        
        self.clicked_position = None
        self.clicked_depth_value = None
        self.clicked_depth_mm = None
        self.clicked_depth_m = None
        
        self.lock = threading.Lock()
        
        self._init_paths()
        self._init_thresholds()
        self._init_windows()
        self._update_trackbar_positions()
    
    def update_depth_image(self, depth_image):
        """更新深度图像（从ROS回调调用）"""
        with self.lock:
            self.latest_depth_image = depth_image
    
    def update_color_image(self, color_image):
        """更新彩色图像（从ROS回调调用）"""
        with self.lock:
            self.latest_color_image = color_image
    
    def _log(self, level, message):
        """记录日志"""
        if self.logger:
            if level == 'info':
                self.logger.info(message)
            elif level == 'warn':
                self.logger.warn(message)
            elif level == 'error':
                self.logger.error(message)
            elif level == 'debug':
                self.logger.debug(message)
        else:
            print(f'[{level.upper()}] {message}')
    
    def _init_paths(self):
        """初始化路径配置"""
        script_dir = Path(__file__).parent.absolute()
        package_dir = script_dir.parent
        configs_dir = package_dir / 'configs'
        configs_dir.mkdir(exist_ok=True)
        self.threshold_config_file = configs_dir / f'trigger_depth_thresholds_{self.camera_name}.json'
    
    def _init_thresholds(self):
        """初始化阈值配置"""
        self.binary_enabled = True
        self.show_contour = True
        self.contour_min_area = 10
        self.contour_max_area = 34921
        self.show_all_contours = False
        self.binary_threshold_min = 0
        self.binary_threshold_max = 65535
        
        self.component_min_aspect_ratio = 0.3
        self.component_max_aspect_ratio = 4.0
        self.component_min_width = 60
        self.component_min_height = 60
        self.component_max_count = 3
        
        self.big_circle_combine_contours = True
        self.big_circle_min_area = 100
        self.small_circle_erode_kernel = 11
        self.small_circle_erode_iterations = 1
        self.small_circle_largest_cc = True
        self.small_circle_dilate_kernel = 9
        self.small_circle_dilate_iterations = 1
        
        self.load_thresholds()
        
        if self.depth_scale > 0:
            self.binary_threshold_min_mm = int(self.binary_threshold_min * self.depth_scale)
            self.binary_threshold_max_mm = int(self.binary_threshold_max * self.depth_scale)
        else:
            self.binary_threshold_min_mm = 0
            self.binary_threshold_max_mm = 0
    
    def load_thresholds(self):
        """从文件加载保存的阈值"""
        try:
            if self.threshold_config_file.exists():
                with open(self.threshold_config_file, 'r') as f:
                    config = json.load(f)
                    min_raw = config.get('min_depth_raw')
                    max_raw = config.get('max_depth_raw')
                    if min_raw is None:
                        min_mm = config.get('min_depth_mm')
                        if min_mm is not None and self.depth_scale > 0:
                            min_raw = int(min_mm / self.depth_scale)
                    if max_raw is None:
                        max_mm = config.get('max_depth_mm')
                        if max_mm is not None and self.depth_scale > 0:
                            max_raw = int(max_mm / self.depth_scale)
                    
                    if min_raw is not None:
                        self.binary_threshold_min = int(min_raw)
                    if max_raw is not None:
                        self.binary_threshold_max = int(max_raw)
                    
                    contour_min_area = config.get('contour_min_area')
                    contour_max_area = config.get('contour_max_area')
                    if contour_min_area is not None:
                        self.contour_min_area = int(contour_min_area)
                    if contour_max_area is not None:
                        self.contour_max_area = int(contour_max_area)
                    
                    component_min_aspect = config.get('component_min_aspect_ratio')
                    component_max_aspect = config.get('component_max_aspect_ratio')
                    component_min_width = config.get('component_min_width')
                    component_min_height = config.get('component_min_height')
                    component_max_count = config.get('component_max_count')
                    
                    if component_min_aspect is not None:
                        self.component_min_aspect_ratio = float(component_min_aspect)
                    if component_max_aspect is not None:
                        self.component_max_aspect_ratio = float(component_max_aspect)
                    if component_min_width is not None:
                        self.component_min_width = int(component_min_width)
                    if component_min_height is not None:
                        self.component_min_height = int(component_min_height)
                    if component_max_count is not None:
                        self.component_max_count = int(component_max_count)
                    
                    if self.depth_scale > 0:
                        return self.binary_threshold_min * self.depth_scale, self.binary_threshold_max * self.depth_scale
                    return None, None
            return None, None
        except Exception as e:
            self._log('warn', f'加载阈值配置失败: {e}')
            return None, None
    
    def save_thresholds(self):
        """保存当前阈值到文件"""
        try:
            config = {
                'min_depth_raw': self.binary_threshold_min,
                'max_depth_raw': self.binary_threshold_max,
                'min_depth_mm': self.binary_threshold_min_mm,
                'max_depth_mm': self.binary_threshold_max_mm,
                'contour_min_area': self.contour_min_area,
                'contour_max_area': self.contour_max_area,
                'component_min_aspect_ratio': self.component_min_aspect_ratio,
                'component_max_aspect_ratio': self.component_max_aspect_ratio,
                'component_min_width': self.component_min_width,
                'component_min_height': self.component_min_height,
                'component_max_count': self.component_max_count,
                'camera_name': self.camera_name,
                'depth_scale': self.depth_scale
            }
            with open(self.threshold_config_file, 'w') as f:
                json.dump(config, f, indent=2)
            return True
        except Exception as e:
            self._log('error', f'保存阈值文件失败: {e}')
            return False
    
    def _init_windows(self):
        """初始化OpenCV窗口"""
        self.depth_window_name = 'Depth Image (Q:quit, Click:depth, +/-:min, [/]:max)'
        self.color_window_name = 'Color Image'
        self.binary_window_name = 'Binary Depth Image (S:save thresholds)'
        self.preprocessed_window_name = '预处理图像'
        
        for window_name in [self.depth_window_name, self.color_window_name, 
                           self.binary_window_name, self.preprocessed_window_name]:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        cv2.setMouseCallback(self.depth_window_name, self.on_mouse_click)
        
        # 调整窗口位置
        cv2.moveWindow(self.depth_window_name, 50, 50)
        cv2.moveWindow(self.color_window_name, 700, 50)
        cv2.moveWindow(self.binary_window_name, 1350, 50)
        cv2.moveWindow(self.preprocessed_window_name, 2000, 50)
        
        # 创建滑动条
        self._init_trackbars()
    
    def _init_trackbars(self):
        """初始化滑动条"""
        cv2.createTrackbar('Min Depth (raw)', self.binary_window_name, 
                          self.binary_threshold_min, 65535, 
                          self.on_binary_threshold_min_change)
        cv2.createTrackbar('Max Depth (raw)', self.binary_window_name, 
                          self.binary_threshold_max, 65535, 
                          self.on_binary_threshold_max_change)
        cv2.createTrackbar('Enable Binary', self.binary_window_name, 1, 1, 
                          self.on_binary_enable_change)
        cv2.createTrackbar('Show Contour', self.binary_window_name, 1, 1, 
                          self.on_show_contour_change)
        cv2.createTrackbar('Contour Min Area', self.binary_window_name, 
                          self.contour_min_area, 100000, 
                          self.on_contour_min_area_change)
        cv2.createTrackbar('Contour Max Area', self.binary_window_name, 
                          self.contour_max_area, 1000000, 
                          self.on_contour_max_area_change)
        
        cv2.createTrackbar('Min Aspect (x10)', self.binary_window_name, 
                          int(self.component_min_aspect_ratio * 10), 100, 
                          self.on_component_min_aspect_change)
        cv2.createTrackbar('Max Aspect (x10)', self.binary_window_name, 
                          int(self.component_max_aspect_ratio * 10), 100, 
                          self.on_component_max_aspect_change)
        cv2.createTrackbar('Min Width', self.binary_window_name, 
                          self.component_min_width, 500, 
                          self.on_component_min_width_change)
        cv2.createTrackbar('Min Height', self.binary_window_name, 
                          self.component_min_height, 500, 
                          self.on_component_min_height_change)
        cv2.createTrackbar('Max Count', self.binary_window_name, 
                          self.component_max_count, 10, 
                          self.on_component_max_count_change)
    
    def _update_trackbar_positions(self):
        """更新滑动条位置"""
        try:
            cv2.setTrackbarPos('Min Depth (raw)', self.binary_window_name, self.binary_threshold_min)
            cv2.setTrackbarPos('Max Depth (raw)', self.binary_window_name, self.binary_threshold_max)
            cv2.setTrackbarPos('Contour Min Area', self.binary_window_name, self.contour_min_area)
            cv2.setTrackbarPos('Contour Max Area', self.binary_window_name, self.contour_max_area)
            cv2.setTrackbarPos('Min Aspect (x10)', self.binary_window_name, int(self.component_min_aspect_ratio * 10))
            cv2.setTrackbarPos('Max Aspect (x10)', self.binary_window_name, int(self.component_max_aspect_ratio * 10))
            cv2.setTrackbarPos('Min Width', self.binary_window_name, self.component_min_width)
            cv2.setTrackbarPos('Min Height', self.binary_window_name, self.component_min_height)
            cv2.setTrackbarPos('Max Count', self.binary_window_name, self.component_max_count)
        except Exception:
            pass
    
    def on_binary_threshold_min_change(self, val):
        """二值化最小阈值滑动条回调"""
        old_min = self.binary_threshold_min
        self.binary_threshold_min = val
        if self.depth_scale > 0:
            self.binary_threshold_min_mm = int(self.binary_threshold_min * self.depth_scale)
        else:
            self.binary_threshold_min_mm = 0
        if self.binary_threshold_min > self.binary_threshold_max:
            self.binary_threshold_min = self.binary_threshold_max
            cv2.setTrackbarPos('Min Depth (raw)', self.binary_window_name, self.binary_threshold_min)
            self._log('warn', f'最小阈值调整: {val} -> {self.binary_threshold_min} (超过最大值)')
        if old_min != self.binary_threshold_min:
            pass
    
    def on_binary_threshold_max_change(self, val):
        """二值化最大阈值滑动条回调"""
        old_max = self.binary_threshold_max
        self.binary_threshold_max = val
        if self.depth_scale > 0:
            self.binary_threshold_max_mm = int(self.binary_threshold_max * self.depth_scale)
        else:
            self.binary_threshold_max_mm = 0
        if self.binary_threshold_max < self.binary_threshold_min:
            self.binary_threshold_max = self.binary_threshold_min
            cv2.setTrackbarPos('Max Depth (raw)', self.binary_window_name, self.binary_threshold_max)
            self._log('warn', f'最大阈值调整: {val} -> {self.binary_threshold_max} (小于最小值)')
        if old_max != self.binary_threshold_max:
            pass
    
    def adjust_binary_threshold_min(self, delta_raw):
        """调整最小阈值"""
        new_val = max(0, min(65535, self.binary_threshold_min + delta_raw))
        cv2.setTrackbarPos('Min Depth (raw)', self.binary_window_name, new_val)
        self.on_binary_threshold_min_change(new_val)
    
    def adjust_binary_threshold_max(self, delta_raw):
        """调整最大阈值"""
        new_val = max(0, min(65535, self.binary_threshold_max + delta_raw))
        cv2.setTrackbarPos('Max Depth (raw)', self.binary_window_name, new_val)
        self.on_binary_threshold_max_change(new_val)
    
    def on_binary_enable_change(self, val):
        """二值化开关滑动条回调"""
        self.binary_enabled = (val == 1)
    
    def on_show_contour_change(self, val):
        """显示轮廓开关滑动条回调"""
        self.show_contour = (val == 1)
    
    def on_contour_min_area_change(self, val):
        """轮廓最小面积阈值滑动条回调"""
        self.contour_min_area = val
        if self.contour_min_area > self.contour_max_area:
            self.contour_min_area = self.contour_max_area
            cv2.setTrackbarPos('Contour Min Area', self.binary_window_name, self.contour_min_area)
    
    def on_contour_max_area_change(self, val):
        """轮廓最大面积阈值滑动条回调"""
        self.contour_max_area = val
        if self.contour_max_area < self.contour_min_area:
            self.contour_max_area = self.contour_min_area
            cv2.setTrackbarPos('Contour Max Area', self.binary_window_name, self.contour_max_area)
    
    def on_component_min_aspect_change(self, val):
        """连通域最小宽高比滑动条回调"""
        self.component_min_aspect_ratio = val / 10.0
        if self.component_min_aspect_ratio > self.component_max_aspect_ratio:
            self.component_min_aspect_ratio = self.component_max_aspect_ratio
            cv2.setTrackbarPos('Min Aspect (x10)', self.binary_window_name, int(self.component_min_aspect_ratio * 10))
    
    def on_component_max_aspect_change(self, val):
        """连通域最大宽高比滑动条回调"""
        self.component_max_aspect_ratio = val / 10.0
        if self.component_max_aspect_ratio < self.component_min_aspect_ratio:
            self.component_max_aspect_ratio = self.component_min_aspect_ratio
            cv2.setTrackbarPos('Max Aspect (x10)', self.binary_window_name, int(self.component_max_aspect_ratio * 10))
    
    def on_component_min_width_change(self, val):
        """连通域最小宽度滑动条回调"""
        self.component_min_width = val
    
    def on_component_min_height_change(self, val):
        """连通域最小高度滑动条回调"""
        self.component_min_height = val
    
    def on_component_max_count_change(self, val):
        """连通域最大数量滑动条回调"""
        self.component_max_count = max(1, val)
    
    def on_mouse_click(self, event, x, y, flags, param):
        """鼠标点击回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                if self.latest_depth_image is not None:
                    depth_image = self.latest_depth_image
                    if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                        if len(depth_image.shape) == 2:
                            depth_value = depth_image[y, x]
                        elif len(depth_image.shape) == 3:
                            depth_value = depth_image[y, x, 0]
                        else:
                            return
                        self.clicked_position = (x, y)
                        self.clicked_depth_value = depth_value
                        self.clicked_depth_mm, self.clicked_depth_m = self._get_depth_at_point(depth_image, x, y)
    
    
    def _get_depth_at_point(self, depth_image, x, y):
        """获取指定点的深度值（返回毫米和米）"""
        if not (0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]):
            return None, None
        depth_raw = depth_image[y, x]
        if depth_image.dtype == np.uint16:
            if 0 < depth_raw < 65535:
                depth_mm = depth_raw * self.depth_scale
                depth_m = depth_mm / 1000.0
                return depth_mm, depth_m
        elif depth_image.dtype in (np.float32, np.float64):
            if depth_raw > 0 and not np.isnan(depth_raw):
                depth_mm = depth_raw * self.depth_scale * 1000.0
                depth_m = depth_raw * self.depth_scale
                return depth_mm, depth_m
        return None, None
    
    def run(self):
        """运行主循环（OpenCV窗口循环，ROS回调在另一个线程）"""
        try:
            
            window_closed = False
            while not window_closed:
                try:
                    # 检查窗口是否还存在
                    try:
                        for window_name in [self.depth_window_name, self.color_window_name, 
                                           self.binary_window_name, self.preprocessed_window_name]:
                            prop = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
                            if prop < 0:
                                window_closed = True
                                break
                    except cv2.error:
                        window_closed = True
                    
                    if window_closed:
                        break
                    
                    try:
                        if hasattr(self, 'display_depth_image'):
                            self.display_depth_image()
                    except Exception as e:
                        self._log('error', f'显示深度图像时出错: {e}')
                    
                    try:
                        if hasattr(self, 'display_color_image'):
                            self.display_color_image()
                    except Exception as e:
                        self._log('error', f'显示彩色图像时出错: {e}')
                    
                    try:
                        if hasattr(self, 'display_binary_depth_image'):
                            self.display_binary_depth_image()
                    except Exception as e:
                        self._log('error', f'显示二值化图像时出错: {e}')
                    
                    try:
                        if hasattr(self, 'display_preprocessed_image'):
                            self.display_preprocessed_image()
                    except Exception as e:
                        self._log('error', f'显示预处理图像时出错: {e}')
                    
                    # 处理键盘输入
                    try:
                        key = cv2.waitKey(1) & 0xFF
                    except Exception as e:
                        self._log('error', f'等待键盘输入时出错: {e}')
                        break
                    
                    if key == ord('q') or key == ord('Q'):
                        break
                    elif key == ord('+') or key == ord('='):
                        self.adjust_binary_threshold_min(1)
                    elif key == ord('-') or key == ord('_'):
                        self.adjust_binary_threshold_min(-1)
                    elif key == ord(']') or key == ord('}'):
                        self.adjust_binary_threshold_max(1)
                    elif key == ord('[') or key == ord('{'):
                        self.adjust_binary_threshold_max(-1)
                    elif key == ord('a') or key == ord('A'):
                        self.show_all_contours = not self.show_all_contours
                    elif key == ord('s') or key == ord('S'):
                        self.save_thresholds()
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self._log('error', f'主循环异常: {e}')
                    time.sleep(0.1)
        
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self._log('error', f'主循环外层异常: {e}')
        finally:
            self.save_thresholds()
            cv2.destroyAllWindows()
    
    def _process_depth_image(self, depth_image, enable_zero_interp=True):
        """
        深度图处理函数，0值处理（补齐法）
        
        参数:
            depth_image: 输入的深度图像 (numpy array)
            enable_zero_interp: 是否启用0值插值处理（默认True）
        
        返回:
            processed_image: 处理后的深度图像
            stats: 处理统计信息字典，包含处理的点数等信息
        """
        if depth_image is None or not isinstance(depth_image, np.ndarray):
            return depth_image, {}
        
        stats = {
            'filled_zeros': 0
        }
        
        try:
            if len(depth_image.shape) == 3:
                depth_data = depth_image[:, :, 0].copy()
            else:
                depth_data = depth_image.copy()
            
            original_shape = depth_data.shape
            original_dtype = depth_data.dtype
            
            if enable_zero_interp:
                depth_data, filled_zeros = self._interp_data(depth_data)
                stats['filled_zeros'] = filled_zeros
                assert depth_data.shape == original_shape
            
            if depth_data.shape != original_shape:
                depth_data = cv2.resize(depth_data, (original_shape[1], original_shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
            
            if original_dtype != depth_data.dtype:
                depth_data = depth_data.astype(original_dtype)
            
            return depth_data, stats
            
        except Exception as e:
            return depth_image, stats
    
    def _interp_data(self, depth_image):
        """
        0值处理（补齐法）
        针对列或行中出现的个别深度数据为0的点，采用补齐法进行处理
        注意：不处理整行整列的0值，只处理个别0值点
        """
        if depth_image.size == 0:
            return depth_image, 0
        
        new_data = depth_image.copy()
        zero_mask = (new_data == 0)
        fn = np.sum(zero_mask)
        
        if fn == 0:
            return new_data, 0
        
        # 检测整行整列为0的情况
        row_means = np.mean(new_data, axis=1)
        invalid_rows = row_means == 0  # 整行为0的行
        
        col_means = np.mean(new_data, axis=0)
        invalid_cols = col_means == 0  # 整列为0的列
        
        try:
            # 处理行的个别0值点（跳过整行为0的行）
            for i in range(new_data.shape[0]):
                # 如果整行都是0，跳过该行
                if invalid_rows[i]:
                    continue
                
                row = new_data[i, :]
                zero_indices = np.where(row == 0)[0]
                
                if len(zero_indices) == 0:
                    continue
                
                # 只处理该行中的个别0值点（不是整列都是0的列）
                for idx in zero_indices:
                    # 如果该列整列都是0，跳过
                    if invalid_cols[idx]:
                        continue
                    
                    if idx == 0:
                        non_zero_idx = np.where(row[idx+1:] > 0)[0]
                        if len(non_zero_idx) > 0:
                            new_data[i, idx] = row[idx + 1 + non_zero_idx[0]]
                        elif i > 0:
                            new_data[i, idx] = new_data[i - 1, idx]
                    else:
                        new_data[i, idx] = new_data[i, idx - 1]
            
            # 处理列的个别0值点（跳过整列为0的列）
            remaining_zeros = (new_data == 0)
            if np.any(remaining_zeros):
                for j in range(new_data.shape[1]):
                    # 如果整列都是0，跳过该列
                    if invalid_cols[j]:
                        continue
                    
                    col = new_data[:, j]
                    zero_indices = np.where(col == 0)[0]
                    
                    if len(zero_indices) == 0:
                        continue
                    
                    # 只处理该列中的个别0值点（不是整行都是0的行）
                    for idx in zero_indices:
                        # 如果该行整行都是0，跳过
                        if invalid_rows[idx]:
                            continue
                        
                        if idx == 0:
                            non_zero_idx = np.where(col[idx+1:] > 0)[0]
                            if len(non_zero_idx) > 0:
                                new_data[idx, j] = col[idx + 1 + non_zero_idx[0]]
                            elif j > 0:
                                new_data[idx, j] = new_data[idx, j - 1]
                        else:
                            new_data[idx, j] = new_data[idx - 1, j]
            
            return new_data, fn
            
        except Exception as e:
            return new_data, fn
    
    def _get_valid_depth_region(self, depth_image):
        """
        检测有效深度区域（去除整行整列为0的部分）
        
        参数:
            depth_image: 深度图像 (numpy array)
        
        返回:
            valid_bbox: 有效区域的边界框 (x, y, w, h)，如果没有有效区域则返回None
        """
        if depth_image is None or not isinstance(depth_image, np.ndarray) or depth_image.size == 0:
            return None
        
        # 处理3通道图像
        if len(depth_image.shape) == 3:
            depth_data = depth_image[:, :, 0]
        else:
            depth_data = depth_image
        
        # 检测整行整列为0的情况
        # 计算每列的平均值
        col_means = np.mean(depth_data, axis=0)
        # 计算每行的平均值
        row_means = np.mean(depth_data, axis=1)
        
        # 找出有效列（平均值不为0的列）
        valid_cols = col_means > 0
        # 找出有效行（平均值不为0的行）
        valid_rows = row_means > 0
        
        # 如果没有有效区域，返回None
        if not np.any(valid_cols) or not np.any(valid_rows):
            return None
        
        # 找出有效区域的边界
        valid_col_indices = np.where(valid_cols)[0]
        valid_row_indices = np.where(valid_rows)[0]
        
        if len(valid_col_indices) == 0 or len(valid_row_indices) == 0:
            return None
        
        x = int(valid_col_indices[0])
        y = int(valid_row_indices[0])
        w = int(valid_col_indices[-1] - valid_col_indices[0] + 1)
        h = int(valid_row_indices[-1] - valid_row_indices[0] + 1)
        
        return (x, y, w, h)
    
    def _get_invalid_mask(self, depth_image):
        """获取无效深度值掩码"""
        if depth_image.dtype == np.uint16:
            return (depth_image == 0) | (depth_image == 65535)
        elif depth_image.dtype in (np.float32, np.float64):
            return (depth_image == 0) | np.isnan(depth_image)
        else:
            return depth_image == 0
    
    def _get_invalid_reason(self, depth_image, depth_value):
        """获取无效深度值的原因描述"""
        if depth_image.dtype == np.uint16:
            if depth_value == 0:
                return "0 (无深度数据)"
            elif depth_value == 65535:
                return "65535 (超出测量范围)"
            else:
                return "未知"
        elif depth_image.dtype in (np.float32, np.float64):
            if depth_value == 0:
                return "0.0 (无深度数据)"
            elif np.isnan(depth_value):
                return "NaN (非数字)"
            else:
                return "未知"
        else:
            if depth_value == 0:
                return "0 (无深度数据)"
            else:
                return "未知"
    
    def _clamp_coordinate(self, coord, max_val):
        """将坐标限制在有效范围内"""
        return min(max(coord, 0), max_val - 1)
    
    def _normalize_depth_to_uint8(self, depth_image):
        """将深度图像归一化为uint8格式用于显示"""
        if depth_image.dtype == np.uint16:
            valid_mask = depth_image > 0
            if np.any(valid_mask):
                min_val = np.min(depth_image[valid_mask])
                max_val = np.max(depth_image[valid_mask])
                if max_val > min_val:
                    depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
                    depth_normalized[valid_mask] = np.clip(
                        ((depth_image[valid_mask] - min_val) * 255.0 / (max_val - min_val)).astype(np.uint8),
                        0, 255
                    )
                else:
                    depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            else:
                depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            return depth_normalized, self._get_invalid_mask(depth_image)
        elif depth_image.dtype == np.uint8:
            return depth_image, depth_image == 0
        elif depth_image.dtype in (np.float32, np.float64):
            valid_mask = depth_image > 0
            if np.any(valid_mask):
                min_val = np.min(depth_image[valid_mask])
                max_val = np.max(depth_image[valid_mask])
                if max_val > min_val:
                    depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
                    depth_normalized[valid_mask] = np.clip(
                        ((depth_image[valid_mask] - min_val) * 255.0 / (max_val - min_val)).astype(np.uint8),
                        0, 255
                    )
                else:
                    depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            else:
                depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)
            return depth_normalized, self._get_invalid_mask(depth_image)
        else:
            depth_normalized = cv2.normalize(
                depth_image.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            return depth_normalized, depth_normalized == 0
    
    def _calculate_depth_statistics(self, depth_image):
        """计算深度图像统计信息"""
        invalid_mask = self._get_invalid_mask(depth_image)
        invalid_count = np.sum(invalid_mask)
        total_pixels = depth_image.size
        invalid_percentage = (invalid_count / total_pixels) * 100.0
        
        valid_mask = ~invalid_mask
        min_depth_raw = max_depth_raw = None
        min_depth_mm = max_depth_mm = None
        
        if np.any(valid_mask):
            min_depth_raw = np.min(depth_image[valid_mask])
            max_depth_raw = np.max(depth_image[valid_mask])
            
            if depth_image.dtype == np.uint16:
                min_depth_mm = min_depth_raw * self.depth_scale
                max_depth_mm = max_depth_raw * self.depth_scale
            elif depth_image.dtype in (np.float32, np.float64):
                min_depth_mm = min_depth_raw * self.depth_scale * 1000.0
                max_depth_mm = max_depth_raw * self.depth_scale * 1000.0
            
        
        # 根据无效点比例选择颜色
        if invalid_percentage < 10.0:
            invalid_color = (0, 255, 0)  # 绿色
        elif invalid_percentage < 30.0:
            invalid_color = (0, 255, 255)  # 黄色
        else:
            invalid_color = (0, 0, 255)  # 红色
        
        return {
            'invalid_count': invalid_count,
            'total_pixels': total_pixels,
            'invalid_percentage': invalid_percentage,
            'invalid_color': invalid_color,
            'min_depth_raw': min_depth_raw,
            'max_depth_raw': max_depth_raw,
            'min_depth_mm': min_depth_mm,
            'max_depth_mm': max_depth_mm
        }
    
    def _create_binary_image(self, depth_image):
        """创建二值化图像"""
        invalid_mask = self._get_invalid_mask(depth_image)
        
        valid_mask = ~invalid_mask
        if not np.any(valid_mask):
            self._log('warn', f'二值化处理: 没有有效深度值，阈值=[{self.binary_threshold_min}, {self.binary_threshold_max}]')
        
        binary_mask = (depth_image >= self.binary_threshold_min) & (depth_image <= self.binary_threshold_max)
        binary_mask = binary_mask & ~invalid_mask
        
        
        binary_image = np.zeros_like(depth_image, dtype=np.uint8)
        binary_image[binary_mask] = 255
        binary_image[invalid_mask] = 0  # 无效深度值直接设为0（黑色）
        return binary_image
    
    def _extract_connected_components(self, binary_image):
        """提取连通域"""
        if binary_image is None or binary_image.size == 0:
            return []
        if len(binary_image.shape) == 3:
            contour_image = binary_image[:, :, 0] if binary_image.shape[2] > 1 else binary_image[:, :, 0]
        elif len(binary_image.shape) == 2:
            contour_image = binary_image
        else:
            return []
        if contour_image.dtype != np.uint8:
            contour_image = contour_image.astype(np.uint8)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(contour_image, connectivity=8)
        components = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 0:
                component_mask = np.zeros_like(contour_image, dtype=np.uint8)
                component_mask[labels == i] = 255
                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                bbox = (x, y, w, h)
                components.append((component_mask, area, bbox))
        return components
    
    def _filter_components(self, components):
        """筛选连通域"""
        if not components:
            return []
        self._update_contour_thresholds()
        total_before_filter = len(components)
        candidates = []
        filtered_by_area = 0
        filtered_by_aspect = 0
        filtered_by_size = 0
        for component_mask, area, bbox in components:
            x, y, w, h = bbox
            if area < self.contour_min_area or area > self.contour_max_area:
                filtered_by_area += 1
                continue
            if w < self.component_min_width or h < self.component_min_height:
                filtered_by_size += 1
                continue
            aspect_ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0.0
            if aspect_ratio < self.component_min_aspect_ratio or aspect_ratio > self.component_max_aspect_ratio:
                filtered_by_aspect += 1
                continue
            candidates.append((component_mask, area, bbox))
        if len(candidates) > self.component_max_count:
            candidates.sort(key=lambda x: x[1], reverse=True)
            candidates = candidates[:self.component_max_count]
        if not hasattr(self, '_last_filter_state'):
            self._last_filter_state = {
                'total': 0, 'filtered_by_area': 0, 'filtered_by_aspect': 0, 
                'filtered_by_size': 0, 'passed': 0
            }
        self._last_filter_state = {
            'total': total_before_filter,
            'filtered_by_area': filtered_by_area,
            'filtered_by_aspect': filtered_by_aspect,
            'filtered_by_size': filtered_by_size,
            'passed': len(candidates)
        }
        return candidates
    
    def _find_all_contours(self, binary_image):
        """查找所有连通域（使用connectedComponentsWithStats）"""
        all_components_data = self._extract_connected_components(binary_image)
        if not all_components_data:
            return [], [], [], [], [], []
        all_components = [comp[0] for comp in all_components_data]
        all_areas = [comp[1] for comp in all_components_data]
        all_bboxes = [comp[2] for comp in all_components_data]
        filtered_components_data = self._filter_components(all_components_data)
        if not filtered_components_data:
            return all_components, all_areas, all_bboxes, [], [], []
        filtered_components = [comp[0] for comp in filtered_components_data]
        filtered_areas = [comp[1] for comp in filtered_components_data]
        filtered_bboxes = [comp[2] for comp in filtered_components_data]
        return all_components, all_areas, all_bboxes, filtered_components, filtered_areas, filtered_bboxes
    
    def _find_largest_contour(self, binary_image, verbose=False):
        """查找最大轮廓"""
        if binary_image is None or binary_image.size == 0:
            return None, None, 0, 0, 0
        if len(binary_image.shape) == 3:
            contour_image = binary_image[:, :, 0] if binary_image.shape[2] > 1 else binary_image[:, :, 0]
        elif len(binary_image.shape) == 2:
            contour_image = binary_image
        else:
            self._log('error', f'不支持的图像维度: {binary_image.shape}')
            return None, None, 0, 0, 0
        if contour_image.dtype != np.uint8:
            contour_image = contour_image.astype(np.uint8)
        contours, _ = cv2.findContours(contour_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self._update_contour_thresholds()
        if len(contours) == 0:
            return None, None, 0, 0, 0
        contour_areas = [cv2.contourArea(c) for c in contours]
        filtered_contours = []
        filtered_areas = []
        for i, (c, area) in enumerate(zip(contours, contour_areas)):
            if self.contour_min_area <= area <= self.contour_max_area:
                filtered_contours.append(c)
                filtered_areas.append(area)
        if not filtered_contours:
            return None, None, 0, len(contours), len(filtered_contours)
        max_idx = filtered_areas.index(max(filtered_areas))
        max_contour = filtered_contours[max_idx]
        max_area = filtered_areas[max_idx]
        max_bbox = cv2.boundingRect(max_contour)
        return max_contour, max_bbox, max_area, len(contours), len(filtered_contours)
    
    def _update_contour_thresholds(self):
        """从滑动条更新轮廓面积阈值"""
        contour_min_area = cv2.getTrackbarPos('Contour Min Area', self.binary_window_name)
        contour_max_area = cv2.getTrackbarPos('Contour Max Area', self.binary_window_name)
        if contour_min_area >= 0:
            self.contour_min_area = contour_min_area
        if contour_max_area >= 0:
            self.contour_max_area = contour_max_area
        if self.contour_min_area > self.contour_max_area:
            self.contour_min_area = self.contour_max_area
            cv2.setTrackbarPos('Contour Min Area', self.binary_window_name, self.contour_min_area)
        min_aspect_raw = cv2.getTrackbarPos('Min Aspect (x10)', self.binary_window_name)
        max_aspect_raw = cv2.getTrackbarPos('Max Aspect (x10)', self.binary_window_name)
        min_width = cv2.getTrackbarPos('Min Width', self.binary_window_name)
        min_height = cv2.getTrackbarPos('Min Height', self.binary_window_name)
        max_count = cv2.getTrackbarPos('Max Count', self.binary_window_name)
        if min_aspect_raw >= 0:
            self.component_min_aspect_ratio = min_aspect_raw / 10.0
        if max_aspect_raw >= 0:
            self.component_max_aspect_ratio = max_aspect_raw / 10.0
        if min_width >= 0:
            self.component_min_width = min_width
        if min_height >= 0:
            self.component_min_height = min_height
        if max_count >= 0:
            self.component_max_count = max(max_count, 1)
    
    def _draw_contour_bbox(self, display_image, contour, bbox, area, depth_image):
        """绘制轮廓和边界框"""
        x, y, w, h = bbox
        cv2.drawContours(display_image, [contour], -1, (0, 255, 0), 2)
        cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
        center_x = self._clamp_coordinate(x + w // 2, display_image.shape[1])
        center_y = self._clamp_coordinate(y + h // 2, display_image.shape[0])
        cv2.circle(display_image, (center_x, center_y), 5, (255, 0, 0), -1)
        bbox_text = f'BBox: ({x}, {y}) {w}x{h} | Area: {area:.0f}'
        cv2.putText(display_image, bbox_text, (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        center_text = f'Center: ({center_x}, {center_y})'
        cv2.putText(display_image, center_text, (x, y + h + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        depth_mm, depth_m = self._get_depth_at_point(depth_image, center_x, center_y)
        if depth_m is not None:
            depth_text = f'Depth: {depth_m:.3f}m ({int(depth_image[center_y, center_x])})'
            cv2.putText(display_image, depth_text, (x, y + h + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    def _draw_contour_stats(self, display_image, max_area, total_contours=0, filtered_count=0):
        """绘制轮廓统计信息"""
        y_offset = 135
        if filtered_count > 0:
            contour_text = f'Contours: {filtered_count}/{total_contours} (filtered) | Max Area: {max_area:.0f}'
            cv2.putText(display_image, contour_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        elif total_contours > 0:
            contour_text = f'Contours: {total_contours} (all filtered)'
            cv2.putText(display_image, contour_text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            cv2.putText(display_image, 'No contours found', (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        y_offset += 20
        filter_params_text = f'Filter: Area[{self.contour_min_area}, {self.contour_max_area}]'
        cv2.putText(display_image, filter_params_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 15
        aspect_text = f'Aspect[{self.component_min_aspect_ratio:.2f}, {self.component_max_aspect_ratio:.2f}] | Size[{self.component_min_width}, {self.component_min_height}] | MaxCount:{self.component_max_count}'
        cv2.putText(display_image, aspect_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def _extract_workpiece_circle(self, mask, combine_contours=True, min_area=100):
        """提取工件外接圆"""
        if mask is None or mask.size == 0:
            return (None, 0.0)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return (None, 0.0)
        if combine_contours and len(contours) > 1:
            filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
            if filtered_contours:
                all_points = np.vstack(filtered_contours)
                contours = [all_points]
            else:
                largest_contour = max(contours, key=cv2.contourArea)
                contours = [largest_contour]
        else:
            filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
            if filtered_contours:
                contours = filtered_contours
            else:
                all_contours = [c for c in contours if cv2.contourArea(c) > 0]
                if all_contours:
                    largest_contour = max(all_contours, key=cv2.contourArea)
                    contours = [largest_contour]
                else:
                    contours = []
        if not contours:
            return (None, 0.0)
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (float(x), float(y))
        radius_float = float(radius)
        return (center, radius_float)
    
    def _extract_valve_circle(self, mask):
        """提取阀体外接圆"""
        if mask is None or mask.size == 0:
            return (None, 0.0, None)
        original_area = cv2.countNonZero(mask)
        if original_area == 0:
            return (None, 0.0, None)
        erode_kernel = self.small_circle_erode_kernel
        dilate_kernel = self.small_circle_dilate_kernel
        if erode_kernel % 2 == 0:
            erode_kernel += 1
        if dilate_kernel % 2 == 0:
            dilate_kernel += 1
        kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_kernel, erode_kernel))
        eroded = cv2.erode(mask, kernel_erode, iterations=self.small_circle_erode_iterations)
        eroded_area = cv2.countNonZero(eroded)
        if eroded_area == 0:
            self._log('warn', f'腐蚀后掩码面积为0，尝试降低腐蚀强度 (原始参数: 核={erode_kernel}, 迭代={self.small_circle_erode_iterations})')
            for reduced_iterations in range(self.small_circle_erode_iterations - 1, 0, -1):
                eroded = cv2.erode(mask, kernel_erode, iterations=reduced_iterations)
                eroded_area = cv2.countNonZero(eroded)
                if eroded_area > 0:
                    self._log('info', f'降低腐蚀迭代次数成功: {reduced_iterations}次迭代后面积={eroded_area}')
                    break
            if eroded_area == 0:
                for reduced_kernel in range(erode_kernel - 2, 3, -2):
                    if reduced_kernel < 3:
                        break
                    kernel_erode_reduced = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (reduced_kernel, reduced_kernel))
                    eroded = cv2.erode(mask, kernel_erode_reduced, iterations=1)
                    eroded_area = cv2.countNonZero(eroded)
                    if eroded_area > 0:
                        self._log('info', f'降低腐蚀核大小成功: 核={reduced_kernel}, 1次迭代后面积={eroded_area}')
                        kernel_erode = kernel_erode_reduced
                        break
            if eroded_area == 0:
                self._log('warn', f'阀体外接圆提取失败: 即使降低腐蚀强度后掩码面积仍为0 (原始面积={original_area})')
                self._log('warn', f'   建议: 减小腐蚀核大小或迭代次数，或检查掩码是否包含阀体区域')
                return (None, 0.0, eroded)
        processed = eroded.copy()
        if self.small_circle_largest_cc:
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(processed, connectivity=8)
            if num_labels > 1:
                max_area = 0
                max_label = 1
                for i in range(1, num_labels):
                    area = stats[i, cv2.CC_STAT_AREA]
                    if area > max_area:
                        max_area = area
                        max_label = i
                largest_cc = np.zeros_like(processed, dtype=np.uint8)
                largest_cc[labels == max_label] = 255
                processed = largest_cc
            else:
                self._log('warn', f'阀体外接圆提取失败: 腐蚀后没有连通域 (标签数: {num_labels})')
                return (None, 0.0, processed)
        if self.small_circle_dilate_iterations > 0 and dilate_kernel > 0:
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_kernel, dilate_kernel))
            processed = cv2.dilate(processed, kernel_dilate, iterations=self.small_circle_dilate_iterations)
            dilated_area = cv2.countNonZero(processed)
        else:
            dilated_area = cv2.countNonZero(processed)
        if dilated_area == 0:
            self._log('warn', f'阀体外接圆提取失败: 膨胀后掩码面积为0')
            return (None, 0.0, processed)
        center, radius = self._extract_workpiece_circle(processed, False, 0)
        if center is None:
            contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                self._log('warn', f'阀体外接圆提取失败: 膨胀后没有轮廓 (面积: {dilated_area})')
            else:
                contour_areas = [cv2.contourArea(c) for c in contours]
                self._log('warn', f'阀体外接圆提取失败: 有{len(contours)}个轮廓但无法提取外接圆 (轮廓面积: {[f"{a:.0f}" for a in contour_areas]})')
        return (center, radius, processed)
    
    def _calculate_standardized_angle(self, workpiece_center, valve_center):
        """计算标准化角度"""
        if workpiece_center is None or valve_center is None:
            return 0.0
        dx = valve_center[0] - workpiece_center[0]
        dy = valve_center[1] - workpiece_center[1]
        angle = np.arctan2(dy, dx)
        return angle
    
    def _extract_features_from_mask(self, mask):
        """从掩码中提取特征"""
        if mask is None or mask.size == 0:
            self._log('warn', '特征提取失败: 掩码为空')
            return None
        feature = {}
        feature['workpiece_area'] = cv2.countNonZero(mask)
        workpiece_center, workpiece_radius = self._extract_workpiece_circle(
            mask, self.big_circle_combine_contours, self.big_circle_min_area)
        feature['workpiece_center'] = workpiece_center
        feature['workpiece_radius'] = workpiece_radius if workpiece_center is not None else 0.0
        if workpiece_center is None:
            self._log('warn', '特征提取失败: 无法提取工件外接圆')
            return None
        valve_center, valve_radius, valve_mask = self._extract_valve_circle(mask)
        feature['valve_center'] = valve_center
        feature['valve_radius'] = valve_radius if valve_center is not None else 0.0
        feature['valve_area'] = cv2.countNonZero(valve_mask) if valve_mask is not None else 0
        if valve_center is None:
            self._log('warn', '阀体外接圆提取失败: 无法提取阀体外接圆')
        standardized_angle = self._calculate_standardized_angle(workpiece_center, valve_center)
        feature['standardized_angle'] = standardized_angle
        feature['standardized_angle_deg'] = standardized_angle * 180.0 / np.pi
        return feature
    
    def display_depth_image(self):
        """显示深度图像"""
        try:
            with self.lock:
                if self.latest_depth_image is not None:
                    depth_image = self.latest_depth_image.copy()
                else:
                    depth_image = None
            if depth_image is None:
                display_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display_image, 'Waiting for depth image...', (50, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(display_image, f'Topic: /{self.camera_name}/depth/image_raw', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow(self.depth_window_name, display_image)
                return
            if not isinstance(depth_image, np.ndarray):
                self._log('error', '深度图像不是numpy数组格式')
                return
            if len(depth_image.shape) == 2:
                pass
            elif len(depth_image.shape) == 3:
                if depth_image.shape[2] > 1:
                    depth_image = depth_image[:, :, 0]
            else:
                self._log('error', f'不支持的图像维度: {depth_image.shape}')
                return
            try:
                depth_normalized, invalid_mask = self._normalize_depth_to_uint8(depth_image)
                display_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                display_image[invalid_mask] = [0, 0, 0]
            except Exception as e:
                self._log('error', f'深度图像转换失败: {e}')
                import traceback
                traceback.print_exc()
                if depth_image.dtype == np.uint8:
                    display_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
                else:
                    display_image = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
                    cv2.putText(display_image, 'Image conversion failed', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            info_text = f'Camera: {self.camera_name}'
            cv2.putText(display_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            latest_depth_image_copy = None
            clicked_pos = None
            clicked_depth_m = None
            clicked_depth_mm = None
            with self.lock:
                if self.latest_depth_image is not None:
                    latest_depth_image_copy = self.latest_depth_image.copy()
                clicked_pos = self.clicked_position
                clicked_depth_m = self.clicked_depth_m
                clicked_depth_mm = self.clicked_depth_mm
            if latest_depth_image_copy is not None:
                depth_type_text = f'Type: {latest_depth_image_copy.dtype} | Shape: {latest_depth_image_copy.shape}'
                cv2.putText(display_image, depth_type_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                y_offset = 85
                stats = self._calculate_depth_statistics(latest_depth_image_copy)
                if stats['min_depth_mm'] is not None and stats['max_depth_mm'] is not None:
                    range_text = f'Depth: {stats["min_depth_mm"]/1000:.3f}-{stats["max_depth_mm"]/1000:.3f} m (scale: {self.depth_scale})'
                    cv2.putText(display_image, range_text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    y_offset += 20
                invalid_text = f'Invalid: {stats["invalid_count"]}/{stats["total_pixels"]} ({stats["invalid_percentage"]:.1f}%)'
                cv2.putText(display_image, invalid_text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, stats['invalid_color'], 1)
            # 绘制有效深度检查框
            if latest_depth_image_copy is not None:
                valid_bbox = self._get_valid_depth_region(latest_depth_image_copy)
                if valid_bbox is not None:
                    x, y, w, h = valid_bbox
                    # 绘制有效深度区域的边界框（绿色，线宽2）
                    cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # 在框的上方显示标签
                    bbox_label = f'Valid Region: ({x}, {y}) {w}x{h}'
                    label_y = max(20, y - 10)
                    cv2.putText(display_image, bbox_label, (x, label_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            legend_y = display_image.shape[0] - 40
            legend_text = 'Color: Blue=Near | Red=Far | Black=Invalid'
            cv2.putText(display_image, legend_text, (10, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            hint_text = 'Tip: Click on image to get depth value'
            cv2.putText(display_image, hint_text, (10, legend_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            if clicked_pos is not None:
                x, y = clicked_pos
                cv2.drawMarker(display_image, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                if clicked_depth_m is not None:
                    depth_text = f'({x},{y}): {clicked_depth_m:.3f}m ({clicked_depth_mm:.1f}mm)'
                    text_y = max(30, y - 10)
                    cv2.putText(display_image, depth_text, (x + 15, text_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    invalid_reason = "未知"
                    with self.lock:
                        if self.latest_depth_image is not None and self.clicked_depth_value is not None:
                            invalid_reason = self._get_invalid_reason(self.latest_depth_image, self.clicked_depth_value)
                    depth_text = f'({x},{y}): Invalid ({invalid_reason})'
                    text_y = max(30, y - 10)
                    cv2.putText(display_image, depth_text, (x + 15, text_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if 'display_image' not in locals() or display_image is None:
                display_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display_image, 'Image processing error', (50, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(self.depth_window_name, display_image)
        except Exception as e:
            self._log('error', f'显示深度图像失败: {e}')
            import traceback
            traceback.print_exc()
            error_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_image, f'Error: {str(e)[:50]}', (10, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(self.depth_window_name, error_image)
    
    def display_color_image(self):
        """显示彩色图像"""
        try:
            with self.lock:
                if self.latest_color_image is not None:
                    color_image = self.latest_color_image.copy()
                    detected_bbox = self.detected_bbox
                    show_contour = self.show_contour
                else:
                    color_image = None
                    detected_bbox = None
                    show_contour = False
            if color_image is None:
                color_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(color_image, 'Waiting for color image...', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(self.color_window_name, color_image)
                return
            info_text = f'Camera: {self.camera_name} - Color Image'
            cv2.putText(color_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            image_info = f'Size: {color_image.shape[1]}x{color_image.shape[0]} | Channels: {color_image.shape[2]}'
            cv2.putText(color_image, image_info, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            if detected_bbox is not None and show_contour:
                x, y, w, h = detected_bbox
                bbox_center_x = x + w // 2
                bbox_center_y = y + h // 2
                recognition_w = int(w * 1.2)
                recognition_h = int(h * 1.2)
                recognition_x = bbox_center_x - recognition_w // 2
                recognition_y = bbox_center_y - recognition_h // 2
                if 0 <= x < color_image.shape[1] and 0 <= y < color_image.shape[0]:
                    cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.rectangle(color_image, 
                                (recognition_x, recognition_y), 
                                (recognition_x + recognition_w, recognition_y + recognition_h), 
                                (0, 255, 0), 2)
                    center_x = self._clamp_coordinate(bbox_center_x, color_image.shape[1])
                    center_y = self._clamp_coordinate(bbox_center_y, color_image.shape[0])
                    cv2.circle(color_image, (center_x, center_y), 5, (255, 0, 0), -1)
                    bbox_text = f'Detect: ({x}, {y}) {w}x{h}'
                    cv2.putText(color_image, bbox_text, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    recognition_text = f'Recognition: ({recognition_x}, {recognition_y}) {recognition_w}x{recognition_h}'
                    cv2.putText(color_image, recognition_text, (recognition_x, recognition_y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    center_text = f'Center: ({center_x}, {center_y})'
                    cv2.putText(color_image, center_text, (x, y + h + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.imshow(self.color_window_name, color_image)
        except Exception as e:
            self._log('error', f'显示彩色图像失败: {e}')
            import traceback
            traceback.print_exc()
            error_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_image, f'Error: {str(e)[:50]}', (10, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(self.color_window_name, error_image)
    
    def display_binary_depth_image(self):
        """显示二值化深度图像"""
        try:
            with self.lock:
                if self.latest_depth_image is not None:
                    depth_image = self.latest_depth_image.copy()
                else:
                    depth_image = None
            if depth_image is None:
                binary_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(binary_image, 'Waiting for depth image...', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(self.binary_window_name, binary_image)
                return
            if not isinstance(depth_image, np.ndarray):
                error_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(error_image, 'Invalid depth image format', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow(self.binary_window_name, error_image)
                return
            if len(depth_image.shape) == 3:
                if depth_image.shape[2] > 1:
                    depth_image = depth_image[:, :, 0]
            elif len(depth_image.shape) != 2:
                error_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(error_image, f'Unsupported image shape: {depth_image.shape}', (10, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow(self.binary_window_name, error_image)
                return
            binary_image = self._create_binary_image(depth_image)
            all_components, all_areas, all_bboxes, filtered_components, filtered_areas, filtered_bboxes = self._find_all_contours(binary_image)
            total_contours = len(all_components)
            filtered_count = len(filtered_components)
            max_component = None
            max_bbox = None
            max_area = 0
            max_contour = None
            if filtered_components and filtered_areas and filtered_bboxes:
                max_idx = filtered_areas.index(max(filtered_areas))
                max_component = filtered_components[max_idx]
                max_bbox = filtered_bboxes[max_idx]
                max_area = filtered_areas[max_idx]
                max_contours, _ = cv2.findContours(max_component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                max_contour = max_contours[0] if max_contours else None
                binary_image_only_max = np.zeros_like(binary_image, dtype=np.uint8)
                binary_image_only_max[max_component == 255] = 255
                with self.lock:
                    self.detected_bbox = max_bbox
            else:
                binary_image_only_max = np.zeros_like(binary_image, dtype=np.uint8)
            binary_display = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            if not hasattr(self, '_last_contour_state'):
                self._last_contour_state = {'total': 0, 'filtered': 0, 'has_max': False}
            self._last_contour_state = {
                'total': total_contours,
                'filtered': filtered_count,
                'has_max': max_contour is not None
            }
            if self.show_all_contours and filtered_components:
                max_idx = filtered_areas.index(max(filtered_areas)) if filtered_areas else -1
                for i, (component_mask, area, bbox) in enumerate(zip(filtered_components, filtered_areas, filtered_bboxes)):
                    contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if i == max_idx:
                        color = (0, 255, 0)
                        thickness = 2
                    else:
                        color = (255, 0, 255)
                        thickness = 1
                    if contours:
                        cv2.drawContours(binary_display, contours, -1, color, thickness)
                    x, y, w, h = bbox
                    cv2.rectangle(binary_display, (x, y), (x + w, y + h), color, thickness)
                    cv2.putText(binary_display, f'#{i+1}:{area:.0f}', (x, y - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            if max_contour is not None and max_bbox is not None and self.show_contour and not self.show_all_contours:
                x, y, w, h = max_bbox
                bbox_center_x = x + w // 2
                bbox_center_y = y + h // 2
                recognition_w = int(w * 1.2)
                recognition_h = int(h * 1.2)
                recognition_x = bbox_center_x - recognition_w // 2
                recognition_y = bbox_center_y - recognition_h // 2
                img_h, img_w = binary_display.shape[:2]
                recognition_x1 = max(0, recognition_x)
                recognition_y1 = max(0, recognition_y)
                recognition_x2 = min(img_w, recognition_x + recognition_w)
                recognition_y2 = min(img_h, recognition_y + recognition_h)
                roi_binary = binary_image_only_max[recognition_y1:recognition_y2, recognition_x1:recognition_x2].copy()
                if roi_binary.size > 0:
                    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                    roi_smoothed = cv2.morphologyEx(roi_binary, cv2.MORPH_OPEN, kernel_open)
                    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                    roi_smoothed = cv2.morphologyEx(roi_smoothed, cv2.MORPH_CLOSE, kernel_close)
                    roi_smoothed_bgr = cv2.cvtColor(roi_smoothed, cv2.COLOR_GRAY2BGR)
                    binary_display[recognition_y1:recognition_y2, recognition_x1:recognition_x2] = roi_smoothed_bgr
                self._draw_contour_bbox(binary_display, max_contour, max_bbox, max_area, depth_image)
                cv2.rectangle(binary_display, (recognition_x1, recognition_y1), 
                            (recognition_x2, recognition_y2), (0, 255, 0), 2)
            # 绘制有效深度检查框
            valid_bbox = self._get_valid_depth_region(depth_image)
            if valid_bbox is not None:
                x, y, w, h = valid_bbox
                # 绘制有效深度区域的边界框（青色，线宽2）
                cv2.rectangle(binary_display, (x, y), (x + w, y + h), (255, 255, 0), 2)
                # 在框的上方显示标签
                bbox_label = f'Valid Region: ({x}, {y}) {w}x{h}'
                label_y = max(20, y - 10)
                cv2.putText(binary_display, bbox_label, (x, label_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            info_text = f'Binary Depth (Min: {self.binary_threshold_min}, Max: {self.binary_threshold_max})'
            cv2.putText(binary_display, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            self._draw_contour_stats(binary_display, max_area, total_contours, filtered_count)
            in_range_count = 0
            if depth_image.dtype == np.uint16:
                total_pixels = depth_image.size
                invalid_mask = self._get_invalid_mask(depth_image)
                in_range_mask = (depth_image >= self.binary_threshold_min) & (depth_image <= self.binary_threshold_max)
                in_range_mask = in_range_mask & ~invalid_mask
                in_range_count = np.sum(in_range_mask)
                invalid_count = np.sum(invalid_mask)
                out_range_count = total_pixels - in_range_count - invalid_count
                in_range_percentage = (in_range_count / total_pixels) * 100.0
                stats_text = f'In Range: {in_range_count} ({in_range_percentage:.1f}%)'
                cv2.putText(binary_display, stats_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                out_range_text = f'Out Range: {out_range_count}'
                cv2.putText(binary_display, out_range_text, (10, 85),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                invalid_text = f'Invalid: {invalid_count}'
                cv2.putText(binary_display, invalid_text, (10, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            legend_y = binary_display.shape[0] - 80
            cv2.putText(binary_display, 'White: In Range or Invalid', (10, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(binary_display, 'Black: Out Range', (10, legend_y + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            if self.show_contour:
                if self.show_all_contours:
                    cv2.putText(binary_display, 'Green: Max Contour | Magenta: Other | Press A: Toggle All', (10, legend_y + 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    cv2.putText(binary_display, 'Green: Contour/Recognition | Yellow: BBox | Blue: Center', (10, legend_y + 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(binary_display, 'Press A: Toggle Show All Contours', (10, legend_y + 45),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            if not self.binary_enabled:
                overlay = binary_display.copy()
                cv2.rectangle(overlay, (0, 0), (binary_display.shape[1], binary_display.shape[0]), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.5, binary_display, 0.5, 0, binary_display)
                cv2.putText(binary_display, 'Enable Binary to see result', 
                           (binary_display.shape[1]//2 - 150, binary_display.shape[0]//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(self.binary_window_name, binary_display)
        except Exception as e:
            self._log('error', f'二值化深度图像处理失败: {e}')
            import traceback
            traceback.print_exc()
            error_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_image, f'Error: {str(e)[:50]}', (10, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(self.binary_window_name, error_image)
    
    def display_preprocessed_image(self):
        """显示预处理图像"""
        try:
            with self.lock:
                if self.latest_color_image is not None:
                    color_image = self.latest_color_image.copy()
                    depth_image = self.latest_depth_image.copy() if self.latest_depth_image is not None else None
                    detected_bbox = self.detected_bbox
                    show_contour = self.show_contour
                else:
                    color_image = None
                    depth_image = None
                    detected_bbox = None
                    show_contour = False
            if color_image is None:
                preprocessed_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(preprocessed_image, 'Waiting for color image...', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(self.preprocessed_window_name, preprocessed_image)
                return
            img_h, img_w = color_image.shape[:2]
            preprocessed_image = color_image.copy()
            if detected_bbox is not None and show_contour and depth_image is not None:
                x, y, w, h = detected_bbox
                bbox_center_x = x + w // 2
                bbox_center_y = y + h // 2
                recognition_w = int(w * 1.2)
                recognition_h = int(h * 1.2)
                recognition_x = bbox_center_x - recognition_w // 2
                recognition_y = bbox_center_y - recognition_h // 2
                img_h, img_w = color_image.shape[:2]
                detection_x1 = max(0, x)
                detection_y1 = max(0, y)
                detection_x2 = min(img_w, x + w)
                detection_y2 = min(img_h, y + h)
                recognition_x1 = max(0, recognition_x)
                recognition_y1 = max(0, recognition_y)
                recognition_x2 = min(img_w, recognition_x + recognition_w)
                recognition_y2 = min(img_h, recognition_y + recognition_h)
                depth_for_mask = depth_image.copy()
                if len(depth_for_mask.shape) == 3:
                    depth_for_mask = depth_for_mask[:, :, 0]
                binary_mask = self._create_binary_image(depth_for_mask)
                preprocessed_image = np.full((img_h, img_w, 3), (255, 255, 255), dtype=np.uint8)
                recognition_roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
                cv2.rectangle(recognition_roi_mask, (recognition_x1, recognition_y1), 
                             (recognition_x2, recognition_y2), 255, -1)
                workpiece_mask = (binary_mask == 255) & (recognition_roi_mask == 255)
                kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                workpiece_mask_dilated = cv2.dilate(workpiece_mask.astype(np.uint8), kernel_dilate, iterations=1)
                workpiece_mask_dilated = workpiece_mask_dilated.astype(bool)
                workpiece_mask_dilated = workpiece_mask_dilated & (recognition_roi_mask == 255)
                preprocessed_image[workpiece_mask_dilated] = color_image[workpiece_mask_dilated]
                workpiece_mask_for_feature = np.zeros((img_h, img_w), dtype=np.uint8)
                workpiece_mask_for_feature[workpiece_mask_dilated] = 255
                try:
                    if workpiece_mask_for_feature.size > 0 and np.any(workpiece_mask_for_feature > 0):
                        feature = self._extract_features_from_mask(workpiece_mask_for_feature)
                        with self.lock:
                            self.extracted_features = feature
                    else:
                        with self.lock:
                            self.extracted_features = None
                    feature_to_draw = None
                    with self.lock:
                        if self.extracted_features is not None:
                            feature_to_draw = self.extracted_features
                    if feature_to_draw is not None and feature_to_draw.get('workpiece_center') is not None:
                        wp_center = feature_to_draw['workpiece_center']
                        wp_radius = feature_to_draw['workpiece_radius']
                        if wp_center is not None and wp_radius > 0:
                                wp_x = int(wp_center[0])
                                wp_y = int(wp_center[1])
                                wp_r = int(wp_radius)
                                if 0 <= wp_x < img_w and 0 <= wp_y < img_h and wp_r > 0:
                                    wp_r = min(wp_r, max(img_w, img_h))
                                    cv2.circle(preprocessed_image, 
                                              (wp_x, wp_y), 
                                              wp_r, 
                                              (0, 255, 0), 2)
                                    cv2.circle(preprocessed_image, 
                                              (wp_x, wp_y), 
                                              5, (0, 255, 0), -1)
                                    if feature_to_draw.get('valve_center') is not None and feature_to_draw.get('valve_radius', 0) > 0:
                                        valve_center = feature_to_draw['valve_center']
                                        valve_radius = feature_to_draw['valve_radius']
                                        if valve_center is not None:
                                            valve_x = int(valve_center[0])
                                            valve_y = int(valve_center[1])
                                            valve_r = int(valve_radius)
                                            if 0 <= valve_x < img_w and 0 <= valve_y < img_h and valve_r > 0:
                                                valve_r = min(valve_r, max(img_w, img_h))
                                                cv2.circle(preprocessed_image, 
                                                          (valve_x, valve_y), 
                                                          valve_r, 
                                                          (255, 0, 0), 2)
                                                cv2.circle(preprocessed_image, 
                                                          (valve_x, valve_y), 
                                                          5, (255, 0, 0), -1)
                                                cv2.line(preprocessed_image, 
                                                        (wp_x, wp_y),
                                                        (valve_x, valve_y),
                                                        (0, 255, 255), 2)
                                                angle_deg = feature_to_draw.get('standardized_angle_deg', 0.0)
                                                angle_text = f'Angle: {angle_deg:.1f}°'
                                                text_x = min(valve_x + 10, img_w - 100)
                                                text_y = max(valve_y, 20)
                                                cv2.putText(preprocessed_image, angle_text, 
                                                          (text_x, text_y),
                                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                except Exception as e:
                    self._log('error', f'特征提取失败: {e}')
                    import traceback
                    traceback.print_exc()
            info_text = f'预处理图像 - Camera: {self.camera_name}'
            cv2.putText(preprocessed_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            extracted_features_copy = None
            with self.lock:
                if self.extracted_features is not None:
                    extracted_features_copy = self.extracted_features
            if extracted_features_copy is not None:
                feature = extracted_features_copy
                y_offset = 60
                max_y = img_h - 20
                try:
                    if feature.get('workpiece_center') is not None:
                        wp_center = feature['workpiece_center']
                        wp_radius = feature.get('workpiece_radius', 0)
                        wp_area = feature.get('workpiece_area', 0)
                        if wp_center is not None:
                            feature_text = f'Workpiece: Center({wp_center[0]:.1f}, {wp_center[1]:.1f}) Radius:{wp_radius:.1f} Area:{wp_area}'
                            if y_offset <= max_y:
                                cv2.putText(preprocessed_image, feature_text, (10, y_offset),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                y_offset += 20
                    if feature.get('valve_center') is not None:
                        valve_center = feature['valve_center']
                        valve_radius = feature.get('valve_radius', 0)
                        valve_area = feature.get('valve_area', 0)
                        angle_deg = feature.get('standardized_angle_deg', 0.0)
                        if valve_center is not None:
                            valve_text = f'Valve: Center({valve_center[0]:.1f}, {valve_center[1]:.1f}) Radius:{valve_radius:.1f} Area:{valve_area}'
                            if y_offset <= max_y:
                                cv2.putText(preprocessed_image, valve_text, (10, y_offset),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                                y_offset += 20
                            angle_text = f'Standardized Angle: {angle_deg:.2f}° ({feature.get("standardized_angle", 0.0):.4f} rad)'
                            if y_offset <= max_y:
                                cv2.putText(preprocessed_image, angle_text, (10, y_offset),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                except Exception as e:
                    self._log('warn', f'显示特征信息时出错: {e}')
            cv2.imshow(self.preprocessed_window_name, preprocessed_image)
        except Exception as e:
            self._log('error', f'显示预处理图像失败: {e}')
            import traceback
            traceback.print_exc()
            error_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_image, f'Error: {str(e)[:50]}', (10, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(self.preprocessed_window_name, error_image)


class TriggerDepthSubscriber(Node):
    """ROS订阅节点"""
    
    def __init__(self, image_processor):
        """初始化ROS订阅节点"""
        super().__init__('trigger_depth_node')
        self.image_processor = image_processor
        self.cv_bridge = CvBridge()
        self.depth_image_received = False
        self.color_image_received = False
        self.current_camera_status = None
        self.subscription_check_count = 0
        
        self._init_subscriptions()
    
    def _create_image_qos(self):
        """创建图像订阅的QoS配置"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    def _init_subscriptions(self):
        """初始化ROS订阅"""
        image_qos = self._create_image_qos()
        camera_name = self.image_processor.camera_name
        
        depth_topic = f'/{camera_name}/depth/image_raw'
        self.depth_subscription = self.create_subscription(
            Image, depth_topic, self.depth_image_callback, image_qos
        )
        
        color_topic = f'/{camera_name}/color/image_raw'
        self.color_subscription = self.create_subscription(
            Image, color_topic, self.color_image_callback, image_qos
        )
        
        self.camera_status_subscription = self.create_subscription(
            CameraStatus, '/camera_status', self.camera_status_callback, 10
        )
        
        self.subscription_check_timer = self.create_timer(5.0, self.check_subscription_status)
    
    def depth_image_callback(self, msg):
        """深度图像回调函数"""
        try:
            if not self.depth_image_received:
                self.depth_image_received = True
            
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if not isinstance(depth_image, np.ndarray):
                self.get_logger().error('图像转换失败：不是numpy数组')
                return
            
            if self.image_processor.enable_depth_processing:
                processed_depth_image, _ = self.image_processor._process_depth_image(
                    depth_image,
                    enable_zero_interp=self.image_processor.enable_zero_interp
                )
            else:
                processed_depth_image = depth_image
            
            self.image_processor.update_depth_image(processed_depth_image)
                
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {e}')
    
    def color_image_callback(self, msg):
        """彩色图像回调函数"""
        try:
            if not self.color_image_received:
                self.color_image_received = True
            
            color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if not isinstance(color_image, np.ndarray):
                return
            
            self.image_processor.update_color_image(color_image)
            
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {e}')
    
    def camera_status_callback(self, msg):
        """相机状态回调函数"""
        self.current_camera_status = msg
    
    def check_subscription_status(self):
        """检查订阅状态"""
        self.subscription_check_count += 1
        
        if self.subscription_check_count % 10 == 0:
            if not self.depth_image_received:
                camera_name = self.image_processor.camera_name
                self.get_logger().warn(f'尚未收到深度图像: /{camera_name}/depth/image_raw')
            
            if not self.color_image_received:
                self.get_logger().warn('尚未收到彩色图像')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        image_processor = ImageProcessor(camera_name='camera', depth_scale=0.25, logger=None)
        subscriber_node = TriggerDepthSubscriber(image_processor)
        image_processor.logger = subscriber_node.get_logger()
        
        def ros_spin_thread():
            try:
                rclpy.spin(subscriber_node)
            finally:
                rclpy.shutdown()
        
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()
        
        image_processor.run()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
