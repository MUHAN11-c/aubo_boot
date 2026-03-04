#!/usr/bin/env python3
"""
使用软件触发服务获取深度图像的脚本

功能：
1. 调用 SoftwareTrigger 服务触发相机
2. 订阅深度图像话题并显示
3. 支持手动触发（按空格键）或自动触发（定时）

使用方法：
    python3 trigger_depth.py [camera_name] [--auto] [--interval SECONDS]

参数：
    camera_name: 相机名称，默认为 'camera'
    --auto: 启用自动触发模式
    --interval: 自动触发间隔（秒），默认 1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from percipio_camera_interface.srv import SoftwareTrigger
from percipio_camera_interface.msg import CameraStatus
import cv2
from cv_bridge import CvBridge
import sys
import argparse
import threading
import time
import numpy as np
import json
import os


class TriggerDepthNode(Node):
    """使用软件触发获取深度图像的节点"""
    
    def __init__(self, camera_name='camera', auto_trigger=False, trigger_interval=1.0, depth_scale=0.25):
        super().__init__('trigger_depth_node')
        
        self.camera_name = camera_name
        self.auto_trigger = auto_trigger
        self.trigger_interval = trigger_interval
        self.depth_scale = depth_scale  # 深度缩放因子，用于将原始深度值转换为实际深度
        self.latest_depth_image = None
        self.latest_color_image = None
        self.depth_image_received = False
        self.color_image_received = False
        self.lock = threading.Lock()
        
        # 相机状态（参考手眼标定实现）
        self.current_camera_status = None
        
        # 检测到的边界框（用于在彩色图像上绘制）
        self.detected_bbox = None  # (x, y, w, h)
        
        # 鼠标点击位置和深度值
        self.clicked_position = None
        self.clicked_depth_value = None
        self.clicked_depth_mm = None
        self.clicked_depth_m = None
        
        # 二值化参数（使用实际深度值，单位：毫米）
        # 最大实际深度 = 65535 * depth_scale（毫米）
        max_depth_mm = int(65535 * depth_scale)
        
        # 阈值配置文件路径
        self.threshold_config_file = f'/tmp/trigger_depth_thresholds_{camera_name}.json'
        
        # 尝试从文件加载保存的阈值
        self.binary_threshold_min_mm, self.binary_threshold_max_mm = self.load_thresholds()
        
        # 如果加载失败，使用默认值
        if self.binary_threshold_min_mm is None:
            self.binary_threshold_min_mm = 0  # 最小阈值（实际深度值，毫米）
        if self.binary_threshold_max_mm is None:
            self.binary_threshold_max_mm = max_depth_mm  # 最大阈值（实际深度值，毫米）
        
        # 转换为原始深度值用于比较
        if self.depth_scale > 0:
            self.binary_threshold_min = int(self.binary_threshold_min_mm / self.depth_scale)
            self.binary_threshold_max = int(self.binary_threshold_max_mm / self.depth_scale)
        else:
            self.binary_threshold_min = 0
            self.binary_threshold_max = 65535
        
        self.binary_enabled = True  # 默认启用二值化显示
        self.show_contour = True  # 是否显示轮廓框选
        self.contour_min_area = 100  # 轮廓最小面积阈值（像素²）
        self.contour_max_area = 100000  # 轮廓最大面积阈值（像素²）
        
        # 创建服务客户端（参考手眼标定实现）
        self.trigger_client = self.create_client(SoftwareTrigger, '/software_trigger')
        
        # 等待服务就绪（参考手眼标定实现）
        # 注意：wait_for_service 需要 spin 来处理服务发现消息
        service_ready = False
        start_time = time.time()
        timeout = 10.0  # 增加超时时间到10秒
        
        self.get_logger().info(f'等待软件触发服务就绪: /software_trigger（最多等待{timeout}秒）...')
        
        while time.time() - start_time < timeout:
            if self.trigger_client.wait_for_service(timeout_sec=0.5):
                service_ready = True
                break
            # 处理 ROS2 回调，以便服务发现能够工作
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if service_ready:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f'✓ 软件触发服务已连接: /software_trigger（等待{elapsed_time:.2f}秒）')
        else:
            self.get_logger().warn(f'⚠️  软件触发服务不可用: /software_trigger（等待{timeout}秒后超时）')
            self.get_logger().warn('   提示: 请确保相机节点已启动并提供了 /software_trigger 服务')
            self.get_logger().warn('   可以使用以下命令检查服务: ros2 service list | grep software_trigger')
        
        # 保存服务就绪状态
        self.service_ready = service_ready
        
        # 创建深度图像订阅者
        # 注意：相机节点使用 image_transport，话题名称是 depth/image_raw（在节点命名空间下）
        depth_topic = f'/{camera_name}/depth/image_raw'
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_image_callback,
            10
        )
        self.get_logger().info(f'订阅深度图像话题: {depth_topic}')
        
        # 添加订阅者数量检查定时器
        self.subscription_check_timer = None
        self.subscription_check_count = 0
        
        # 创建彩色图像订阅者
        color_topic = f'/{camera_name}/color/image_raw'
        self.color_subscription = self.create_subscription(
            Image,
            color_topic,
            self.color_image_callback,
            10
        )
        self.get_logger().info(f'订阅彩色图像话题: {color_topic}')
        
        # 订阅相机状态（参考手眼标定实现）
        self.camera_status_subscription = self.create_subscription(
            CameraStatus,
            '/camera_status',
            self.camera_status_callback,
            10
        )
        
        self.cv_bridge = CvBridge()
        
        # 创建窗口
        self.depth_window_name = 'Depth Image (SPACE:trigger, Q:quit, Click:depth, +/-:min, [/]:max)'
        self.color_window_name = 'Color Image'
        self.binary_window_name = 'Binary Depth Image (S:save thresholds)'
        cv2.namedWindow(self.depth_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.color_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.binary_window_name, cv2.WINDOW_NORMAL)
        
        # 设置鼠标回调函数
        cv2.setMouseCallback(self.depth_window_name, self.on_mouse_click)
        
        # 调整窗口位置，让它们并排显示
        cv2.moveWindow(self.depth_window_name, 50, 50)
        cv2.moveWindow(self.color_window_name, 700, 50)
        cv2.moveWindow(self.binary_window_name, 1350, 50)
        
        # 创建二值化滑动条（使用实际深度值，单位：毫米）
        # 最大实际深度 = 65535 * depth_scale（毫米）
        max_depth_mm = int(65535 * depth_scale)
        # 使用加载的阈值或默认值初始化滑动条
        cv2.createTrackbar('Min Depth (mm)', self.binary_window_name, self.binary_threshold_min_mm, max_depth_mm, self.on_binary_threshold_min_change)
        cv2.createTrackbar('Max Depth (mm)', self.binary_window_name, self.binary_threshold_max_mm, max_depth_mm, self.on_binary_threshold_max_change)
        cv2.createTrackbar('Enable Binary', self.binary_window_name, 1, 1, self.on_binary_enable_change)  # 默认启用（1）
        cv2.createTrackbar('Show Contour', self.binary_window_name, 1, 1, self.on_show_contour_change)
        # 轮廓面积阈值滑动条（范围：0-10000像素²）
        cv2.createTrackbar('Contour Min Area', self.binary_window_name, self.contour_min_area, 10000, self.on_contour_min_area_change)
        # 轮廓最大面积阈值滑动条（范围：0-1000000像素²）
        cv2.createTrackbar('Contour Max Area', self.binary_window_name, self.contour_max_area, 1000000, self.on_contour_max_area_change)
        
        # 如果启用自动触发，创建定时器
        if self.auto_trigger:
            self.trigger_timer = self.create_timer(trigger_interval, self.auto_trigger_callback)
            self.get_logger().info(f'自动触发模式: 间隔 {trigger_interval} 秒')
        else:
            self.get_logger().info('手动触发模式: 空格键触发, Q键退出')
        
        # 创建订阅检查定时器（每5秒检查一次）
        self.subscription_check_timer = self.create_timer(5.0, self.check_subscription_status)
        
        # 如果服务可用，在初始化完成后自动触发一次（给相机一些时间准备）
        self.initial_trigger_done = False
        if self.service_ready:
            self.initial_trigger_timer = self.create_timer(2.0, self.initial_trigger_callback)
    
    def on_binary_threshold_min_change(self, val):
        """二值化最小阈值滑动条回调（输入值为实际深度，单位：毫米）"""
        self.binary_threshold_min_mm = val
        # 转换为原始深度值：原始值 = 实际值(毫米) / depth_scale
        if self.depth_scale > 0:
            self.binary_threshold_min = int(val / self.depth_scale)
        else:
            self.binary_threshold_min = 0
        
        if self.binary_threshold_min > self.binary_threshold_max:
            self.binary_threshold_min = self.binary_threshold_max
            self.binary_threshold_min_mm = int(self.binary_threshold_max * self.depth_scale)
            cv2.setTrackbarPos('Min Depth (mm)', self.binary_window_name, self.binary_threshold_min_mm)
    
    def on_binary_threshold_max_change(self, val):
        """二值化最大阈值滑动条回调（输入值为实际深度，单位：毫米）"""
        self.binary_threshold_max_mm = val
        # 转换为原始深度值：原始值 = 实际值(毫米) / depth_scale
        if self.depth_scale > 0:
            self.binary_threshold_max = int(val / self.depth_scale)
        else:
            self.binary_threshold_max = 65535
        
        if self.binary_threshold_max < self.binary_threshold_min:
            self.binary_threshold_max = self.binary_threshold_min
            self.binary_threshold_max_mm = int(self.binary_threshold_min * self.depth_scale)
            cv2.setTrackbarPos('Max Depth (mm)', self.binary_window_name, self.binary_threshold_max_mm)
    
    def adjust_binary_threshold_min(self, delta_mm):
        """调整最小阈值（微调，单位：毫米）"""
        max_depth_mm = int(65535 * self.depth_scale)
        new_val = max(0, min(max_depth_mm, self.binary_threshold_min_mm + delta_mm))
        cv2.setTrackbarPos('Min Depth (mm)', self.binary_window_name, new_val)
        # 触发回调函数更新内部值
        self.on_binary_threshold_min_change(new_val)
    
    def adjust_binary_threshold_max(self, delta_mm):
        """调整最大阈值（微调，单位：毫米）"""
        max_depth_mm = int(65535 * self.depth_scale)
        new_val = max(0, min(max_depth_mm, self.binary_threshold_max_mm + delta_mm))
        cv2.setTrackbarPos('Max Depth (mm)', self.binary_window_name, new_val)
        # 触发回调函数更新内部值
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
        # 确保最小值不超过最大值
        if self.contour_min_area > self.contour_max_area:
            self.contour_min_area = self.contour_max_area
            cv2.setTrackbarPos('Contour Min Area', self.binary_window_name, self.contour_min_area)
    
    def on_contour_max_area_change(self, val):
        """轮廓最大面积阈值滑动条回调"""
        self.contour_max_area = val
        # 确保最大值不小于最小值
        if self.contour_max_area < self.contour_min_area:
            self.contour_max_area = self.contour_min_area
            cv2.setTrackbarPos('Contour Max Area', self.binary_window_name, self.contour_max_area)
    
    def load_thresholds(self):
        """从文件加载保存的阈值"""
        try:
            if os.path.exists(self.threshold_config_file):
                with open(self.threshold_config_file, 'r') as f:
                    config = json.load(f)
                    min_mm = config.get('min_depth_mm')
                    max_mm = config.get('max_depth_mm')
                    contour_min_area = config.get('contour_min_area')
                    contour_max_area = config.get('contour_max_area')
                    if contour_min_area is not None:
                        self.contour_min_area = int(contour_min_area)
                    if contour_max_area is not None:
                        self.contour_max_area = int(contour_max_area)
                    return min_mm, max_mm
            return None, None
        except Exception as e:
            return None, None
    
    def save_thresholds(self):
        """保存当前阈值到文件"""
        try:
            config = {
                'min_depth_mm': self.binary_threshold_min_mm,
                'max_depth_mm': self.binary_threshold_max_mm,
                'contour_min_area': self.contour_min_area,
                'contour_max_area': self.contour_max_area,
                'camera_name': self.camera_name,
                'depth_scale': self.depth_scale
            }
            with open(self.threshold_config_file, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f'✓ 阈值已保存: {self.threshold_config_file}')
            return True
        except Exception as e:
            self.get_logger().error(f'保存阈值文件失败: {e}')
            return False
    
    def on_mouse_click(self, event, x, y, flags, param):
        """鼠标点击回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 获取点击位置的深度值
            with self.lock:
                if self.latest_depth_image is not None:
                    depth_image = self.latest_depth_image
                    
                    # 检查坐标是否在图像范围内
                    if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                        # 获取深度值
                        if len(depth_image.shape) == 2:
                            depth_value = depth_image[y, x]
                        elif len(depth_image.shape) == 3:
                            depth_value = depth_image[y, x, 0]
                        else:
                            return
                        
                        # 保存点击位置和深度值
                        self.clicked_position = (x, y)
                        self.clicked_depth_value = depth_value
                        
                        # 计算实际深度（应用缩放因子）
                        if depth_image.dtype == np.uint16:
                            if depth_value > 0 and depth_value < 65535:
                                self.clicked_depth_mm = depth_value * self.depth_scale
                                self.clicked_depth_m = self.clicked_depth_mm / 1000.0
                            else:
                                self.clicked_depth_mm = None
                                self.clicked_depth_m = None
                        elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
                            if depth_value > 0 and not np.isnan(depth_value):
                                self.clicked_depth_mm = depth_value * self.depth_scale * 1000.0
                                self.clicked_depth_m = depth_value * self.depth_scale
                            else:
                                self.clicked_depth_mm = None
                                self.clicked_depth_m = None
                        else:
                            self.clicked_depth_mm = None
                            self.clicked_depth_m = None
                        
                        # 输出深度值信息
                        if self.clicked_depth_m is not None:
                            self.get_logger().info(f'点击位置 ({x}, {y}): {self.clicked_depth_m:.3f}m ({self.clicked_depth_mm:.1f}mm)')
    
    def check_subscription_status(self):
        """检查订阅状态"""
        self.subscription_check_count += 1
        if self.subscription_check_count == 1:  # 仅在第一次检查时输出
            with self.lock:
                has_received = self.depth_image_received
            
            if not has_received:
                self.get_logger().warn('⚠️  尚未收到深度图像，请检查相机节点和话题')
    
    def depth_image_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 记录收到深度图（仅首次记录）
            if not self.depth_image_received:
                self.get_logger().info(f'✓ 收到深度图像: {msg.width}x{msg.height}, 编码={msg.encoding}')
            
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 确保是numpy数组格式
            if not isinstance(depth_image, np.ndarray):
                self.get_logger().error('图像转换失败：不是numpy数组')
                return
            
            # 计算无效深度点比例（仅在比例过高时警告）
            if depth_image.dtype == np.uint16:
                invalid_mask = (depth_image == 0) | (depth_image == 65535)
            elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
                invalid_mask = (depth_image == 0) | np.isnan(depth_image)
            else:
                invalid_mask = depth_image == 0
            
            invalid_count = np.sum(invalid_mask)
            invalid_percentage = (invalid_count / depth_image.size) * 100.0
            
            # 仅在无效点比例过高时警告
            if invalid_percentage > 50.0:
                self.get_logger().warn(f'⚠️  无效深度点比例过高: {invalid_percentage:.2f}%')
            
            with self.lock:
                self.latest_depth_image = depth_image
                self.depth_image_received = True
                
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {e}')
            import traceback
            traceback.print_exc()
    
    def color_image_callback(self, msg):
        """彩色图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 确保是numpy数组格式
            if not isinstance(color_image, np.ndarray):
                self.get_logger().error('彩色图像转换失败：不是numpy数组')
                return
            
            with self.lock:
                self.latest_color_image = color_image
                self.color_image_received = True
                
            # 仅在首次收到时记录
            if not hasattr(self, '_color_image_logged'):
                self.get_logger().info(f'✓ 收到彩色图像: {color_image.shape[1]}x{color_image.shape[0]}')
                self._color_image_logged = True
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {e}')
            import traceback
            traceback.print_exc()
    
    def camera_status_callback(self, msg):
        """相机状态回调函数（参考手眼标定实现）"""
        self.current_camera_status = msg
    
    def trigger_camera(self):
        """
        触发相机（参考手眼标定实现）
        
        参考: /home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/hand_eye_calibration/hand_eye_calibration_node.py
        """
        # 检查服务是否可用（参考手眼标定实现）
        if self.trigger_client is None:
            return False
        
        if not self.trigger_client.wait_for_service(timeout_sec=1.0):
            return False
        
        # 获取相机ID（参考手眼标定实现）
        camera_id = '207000152740'  # 默认相机ID
        if self.current_camera_status is not None:
            camera_id = self.current_camera_status.camera_id
        
        # 创建请求消息（参考手眼标定实现）
        request_msg = SoftwareTrigger.Request()
        request_msg.camera_id = camera_id
        
        # 使用异步调用（参考手眼标定实现）
        try:
            future = self.trigger_client.call_async(request_msg)
            
            # 使用 spin_until_future_complete 等待响应（参考手眼标定实现）
            timeout_sec = 5.0
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            
            # 检查 future 是否完成（参考手眼标定实现）
            if future.result() is not None:
                response = future.result()
                if response.success:
                    return True
                else:
                    self.get_logger().warn(f'触发失败: {response.message}')
                    return False
            else:
                return False
                    
        except Exception as e:
            self.get_logger().error(f'触发拍照失败: {str(e)}')
            return False
    
    def initial_trigger_callback(self):
        """初始触发回调（仅执行一次）"""
        if not self.initial_trigger_done:
            self.trigger_camera()
            self.initial_trigger_done = True
            # 取消定时器
            if hasattr(self, 'initial_trigger_timer'):
                self.initial_trigger_timer.cancel()
    
    def auto_trigger_callback(self):
        """自动触发回调"""
        self.trigger_camera()
    
    def display_depth_image(self):
        """显示深度图像"""
        with self.lock:
            if self.latest_depth_image is not None:
                depth_image = self.latest_depth_image.copy()
            else:
                return
        
        # 确保是numpy数组
        if not isinstance(depth_image, np.ndarray):
            self.get_logger().error('深度图像不是numpy数组格式')
            return
        
        # 检查图像维度
        if len(depth_image.shape) == 2:
            # 单通道图像（深度图通常是这样的）
            pass
        elif len(depth_image.shape) == 3:
            # 多通道图像，转换为单通道（取第一个通道）
            if depth_image.shape[2] > 1:
                depth_image = depth_image[:, :, 0]
        else:
            self.get_logger().error(f'不支持的图像维度: {depth_image.shape}')
            return
        
        # 根据数据类型转换深度图像为8位用于显示
        try:
            if depth_image.dtype == np.uint16:
                # 16位深度图像：归一化到0-255
                # 过滤掉无效值（通常深度为0表示无效）
                valid_mask = depth_image > 0
                if np.any(valid_mask):
                    # 只对有效值进行归一化
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
                
                # 将无效值（0 或 65535）明确标记为特殊值，避免显示为蓝色
                # JET颜色映射：蓝色(0)表示最小值，红色(255)表示最大值
                # 为了区分无效值，我们将无效值设为255（红色），然后手动改为黑色
                invalid_mask_zero = depth_image == 0
                invalid_mask_max = depth_image == 65535
                invalid_mask = invalid_mask_zero | invalid_mask_max
                
                # 应用颜色映射以便更好地可视化
                # 使用JET：蓝色=近，红色=远
                display_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                
                # 将无效值区域明确标记为黑色（覆盖颜色映射）
                # 这样无效深度值就不会显示为蓝色了
                display_image[invalid_mask] = [0, 0, 0]  # 黑色表示无效深度
                
            elif depth_image.dtype == np.uint8:
                # 如果已经是8位，直接应用颜色映射
                display_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
                
            elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
                # 浮点型深度图像
                valid_mask = depth_image > 0
                invalid_mask = depth_image == 0
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
                display_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                # 将无效值区域标记为黑色
                display_image[invalid_mask] = [0, 0, 0]
                
            else:
                # 其他类型，尝试转换为uint8
                self.get_logger().warn(f'不支持的深度图像类型: {depth_image.dtype}，尝试转换')
                depth_normalized = cv2.normalize(
                    depth_image.astype(np.float32), 
                    None, 0, 255, 
                    cv2.NORM_MINMAX
                ).astype(np.uint8)
                display_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {e}')
            import traceback
            traceback.print_exc()
            # 如果转换失败，尝试直接显示（可能无法正确显示）
            if depth_image.dtype == np.uint8:
                display_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
            else:
                display_image = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
                cv2.putText(display_image, 'Image conversion failed', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 添加信息文本
        info_text = f'Camera: {self.camera_name}'
        if self.auto_trigger:
            info_text += f' | Auto Trigger: {self.trigger_interval}s'
        else:
            info_text += ' | Press SPACE to trigger'
        
        cv2.putText(display_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加深度图类型信息
        with self.lock:
            if self.latest_depth_image is not None:
                depth_type_text = f'Type: {self.latest_depth_image.dtype} | Shape: {self.latest_depth_image.shape}'
                cv2.putText(display_image, depth_type_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # 显示深度值范围（如果有有效值，应用缩放因子）
                y_offset = 85
                if self.latest_depth_image.dtype == np.uint16:
                    # 检查无效点
                    invalid_mask = (self.latest_depth_image == 0) | (self.latest_depth_image == 65535)
                    invalid_count = np.sum(invalid_mask)
                    total_pixels = self.latest_depth_image.size
                    invalid_percentage = (invalid_count / total_pixels) * 100.0
                    
                    valid_mask = (self.latest_depth_image > 0) & (self.latest_depth_image < 65535)
                    if np.any(valid_mask):
                        min_d_raw = np.min(self.latest_depth_image[valid_mask])
                        max_d_raw = np.max(self.latest_depth_image[valid_mask])
                        # 应用深度缩放因子
                        min_d = min_d_raw * self.depth_scale
                        max_d = max_d_raw * self.depth_scale
                        range_text = f'Depth: {min_d/1000:.3f}-{max_d/1000:.3f} m (scale: {self.depth_scale})'
                        cv2.putText(display_image, range_text, (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        y_offset += 20
                    
                    # 显示无效点统计
                    invalid_text = f'Invalid: {invalid_count}/{total_pixels} ({invalid_percentage:.1f}%)'
                    # 根据无效点比例选择颜色：绿色(<10%), 黄色(10-30%), 红色(>30%)
                    if invalid_percentage < 10.0:
                        color = (0, 255, 0)  # 绿色
                    elif invalid_percentage < 30.0:
                        color = (0, 255, 255)  # 黄色
                    else:
                        color = (0, 0, 255)  # 红色
                    cv2.putText(display_image, invalid_text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    
                elif self.latest_depth_image.dtype == np.float32 or self.latest_depth_image.dtype == np.float64:
                    # 检查无效点（0 或 NaN）
                    invalid_mask = (self.latest_depth_image == 0) | np.isnan(self.latest_depth_image)
                    invalid_count = np.sum(invalid_mask)
                    total_pixels = self.latest_depth_image.size
                    invalid_percentage = (invalid_count / total_pixels) * 100.0
                    
                    valid_mask = (self.latest_depth_image > 0) & ~np.isnan(self.latest_depth_image)
                    if np.any(valid_mask):
                        min_d_raw = np.min(self.latest_depth_image[valid_mask])
                        max_d_raw = np.max(self.latest_depth_image[valid_mask])
                        # 应用深度缩放因子
                        min_d = min_d_raw * self.depth_scale
                        max_d = max_d_raw * self.depth_scale
                        range_text = f'Depth: {min_d:.3f}-{max_d:.3f} m (scale: {self.depth_scale})'
                        cv2.putText(display_image, range_text, (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        y_offset += 20
                    
                    # 显示无效点统计
                    invalid_text = f'Invalid: {invalid_count}/{total_pixels} ({invalid_percentage:.1f}%)'
                    if invalid_percentage < 10.0:
                        color = (0, 255, 0)  # 绿色
                    elif invalid_percentage < 30.0:
                        color = (0, 255, 255)  # 黄色
                    else:
                        color = (0, 0, 255)  # 红色
                    cv2.putText(display_image, invalid_text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 添加颜色图例说明
        legend_y = display_image.shape[0] - 40
        legend_text = 'Color: Blue=Near | Red=Far | Black=Invalid'
        cv2.putText(display_image, legend_text, (10, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 添加提示信息
        hint_text = 'Tip: Click on image to get depth value'
        cv2.putText(display_image, hint_text, (10, legend_y + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # 显示鼠标点击位置的深度值
        if self.clicked_position is not None:
            x, y = self.clicked_position
            # 绘制十字标记
            cv2.drawMarker(display_image, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            # 显示深度值信息
            if self.clicked_depth_m is not None:
                depth_text = f'({x},{y}): {self.clicked_depth_m:.3f}m ({self.clicked_depth_mm:.1f}mm)'
                # 在点击位置上方显示文本
                text_y = max(30, y - 10)
                cv2.putText(display_image, depth_text, (x + 15, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                depth_text = f'({x},{y}): Invalid depth'
                text_y = max(30, y - 10)
                cv2.putText(display_image, depth_text, (x + 15, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        if not self.depth_image_received:
            cv2.putText(display_image, 'Waiting for depth image...', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow(self.depth_window_name, display_image)
    
    def display_color_image(self):
        """显示彩色图像"""
        with self.lock:
            if self.latest_color_image is not None:
                color_image = self.latest_color_image.copy()
            else:
                # 如果没有彩色图像，显示占位符
                color_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(color_image, 'Waiting for color image...', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(self.color_window_name, color_image)
                return
        
        # 添加信息文本
        info_text = f'Camera: {self.camera_name} - Color Image'
        cv2.putText(color_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加图像信息
        image_info = f'Size: {color_image.shape[1]}x{color_image.shape[0]} | Channels: {color_image.shape[2]}'
        cv2.putText(color_image, image_info, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 在彩色图像上绘制检测到的边界框（与深度图像相同位置）
        with self.lock:
            if self.detected_bbox is not None and self.show_contour:
                x, y, w, h = self.detected_bbox
                # 检查边界框是否在图像范围内
                if 0 <= x < color_image.shape[1] and 0 <= y < color_image.shape[0]:
                    # 绘制边界框（与深度图像相同的样式）
                    cv2.rectangle(color_image, (x, y), (x + w + 10, y + h + 10), (0, 255, 255), 2)
                    
                    # 绘制中心点（注意：需要括号确保正确的运算符优先级）
                    center_x = x + (w + 10) // 2
                    center_y = y + (h + 10) // 2
                    # 确保坐标在图像范围内
                    center_x = min(max(center_x, 0), color_image.shape[1] - 1)
                    center_y = min(max(center_y, 0), color_image.shape[0] - 1)
                    cv2.circle(color_image, (center_x, center_y), 5, (255, 0, 0), -1)
                    
                    # 显示边界框信息
                    bbox_text = f'BBox: ({x}, {y}) {w}x{h}'
                    cv2.putText(color_image, bbox_text, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    # 显示中心点坐标
                    center_text = f'Center: ({center_x}, {center_y})'
                    cv2.putText(color_image, center_text, (x, y + h + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        cv2.imshow(self.color_window_name, color_image)
    
    def display_binary_depth_image(self):
        """显示二值化深度图像"""
        with self.lock:
            if self.latest_depth_image is None:
                # 如果没有深度图像，显示占位符
                binary_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(binary_image, 'Waiting for depth image...', (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(self.binary_window_name, binary_image)
                return
            
            depth_image = self.latest_depth_image.copy()
        
        # 确保是numpy数组
        if not isinstance(depth_image, np.ndarray):
            return
        
        # 检查图像维度
        if len(depth_image.shape) == 3:
            if depth_image.shape[2] > 1:
                depth_image = depth_image[:, :, 0]
        elif len(depth_image.shape) != 2:
            return
        
        try:
            # 根据数据类型进行二值化
            if depth_image.dtype == np.uint16:
                # 创建二值化掩膜：在阈值范围内的为255，否则为0
                binary_mask = (depth_image >= self.binary_threshold_min) & (depth_image <= self.binary_threshold_max)
                # 排除无效深度值（0 和 65535）
                invalid_mask = (depth_image == 0) | (depth_image == 65535)
                binary_mask = binary_mask & ~invalid_mask
                
                # 创建二值化图像：白色(255)表示在范围内，黑色(0)表示不在范围内
                binary_image = np.zeros_like(depth_image, dtype=np.uint8)
                binary_image[binary_mask] = 255
                
                # 将无效深度值标记为灰色(128)
                binary_image[invalid_mask] = 128
                
            elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
                # 浮点型深度图像
                binary_mask = (depth_image >= self.binary_threshold_min) & (depth_image <= self.binary_threshold_max)
                invalid_mask = (depth_image == 0) | np.isnan(depth_image)
                binary_mask = binary_mask & ~invalid_mask
                
                binary_image = np.zeros_like(depth_image, dtype=np.uint8)
                binary_image[binary_mask] = 255
                binary_image[invalid_mask] = 128
                
            else:
                # 其他类型，尝试转换
                depth_normalized = cv2.normalize(
                    depth_image.astype(np.float32), 
                    None, 0, 65535, 
                    cv2.NORM_MINMAX
                ).astype(np.uint16)
                
                binary_mask = (depth_normalized >= self.binary_threshold_min) & (depth_normalized <= self.binary_threshold_max)
                invalid_mask = depth_normalized == 0
                binary_mask = binary_mask & ~invalid_mask
                
                binary_image = np.zeros_like(depth_normalized, dtype=np.uint8)
                binary_image[binary_mask] = 255
                binary_image[invalid_mask] = 128
            
            # 转换为3通道用于显示
            binary_display = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            
            # 查找轮廓（只处理白色区域，排除灰色无效值）
            contour_image = binary_image.copy()
            # 将灰色(128)也设为黑色，只保留白色(255)用于轮廓检测
            contour_image[contour_image == 128] = 0
            
            # 查找轮廓
            contours, hierarchy = cv2.findContours(contour_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 从滑动条获取当前轮廓面积阈值
            contour_min_area = cv2.getTrackbarPos('Contour Min Area', self.binary_window_name)
            if contour_min_area >= 0:
                self.contour_min_area = contour_min_area
            
            contour_max_area = cv2.getTrackbarPos('Contour Max Area', self.binary_window_name)
            if contour_max_area >= 0:
                self.contour_max_area = contour_max_area
            
            # 确保最小值不超过最大值
            if self.contour_min_area > self.contour_max_area:
                self.contour_min_area = self.contour_max_area
                cv2.setTrackbarPos('Contour Min Area', self.binary_window_name, self.contour_min_area)
            
            # 过滤轮廓：只保留面积在阈值范围内的轮廓
            filtered_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.contour_min_area <= area <= self.contour_max_area:
                    filtered_contours.append(contour)
            
            # 查找最大轮廓（从过滤后的轮廓中）
            max_contour = None
            max_area = 0
            max_bbox = None
            
            if len(filtered_contours) > 0:
                # 找到面积最大的轮廓
                for contour in filtered_contours:
                    area = cv2.contourArea(contour)
                    if area > max_area:
                        max_area = area
                        max_contour = contour
                
                # 如果找到最大轮廓，绘制边界框
                if max_contour is not None and self.show_contour:
                    # 计算最小外接矩形
                    x, y, w, h = cv2.boundingRect(max_contour)
                    max_bbox = (x, y, w, h)
                    # 保存边界框用于在彩色图像上绘制
                    with self.lock:
                        self.detected_bbox = max_bbox
                    
                    # 绘制轮廓
                    cv2.drawContours(binary_display, [max_contour], -1, (0, 255, 0), 2)
                    
                    # 绘制边界框（矩形）
                    cv2.rectangle(binary_display, (x, y), (x + w+10, y + h+10), (0, 255, 255), 2)

                    # 绘制中心点（注意：需要括号确保正确的运算符优先级）
                    center_x = x + (w + 10) // 2
                    center_y = y + (h + 10) // 2
                    # 确保坐标在图像范围内
                    center_x = min(max(center_x, 0), depth_image.shape[1] - 1)
                    center_y = min(max(center_y, 0), depth_image.shape[0] - 1)
                    cv2.circle(binary_display, (center_x, center_y), 5, (255, 0, 0), -1)
                    
                    # 显示边界框信息
                    bbox_text = f'BBox: ({x}, {y}) {w}x{h} | Area: {max_area:.0f}'
                    cv2.putText(binary_display, bbox_text, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    # 显示中心点坐标
                    center_text = f'Center: ({center_x}, {center_y})'
                    cv2.putText(binary_display, center_text, (x, y + h + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                    
                    # 如果深度图可用，显示中心点的深度值
                    if depth_image.dtype == np.uint16:
                        center_depth_raw = depth_image[center_y, center_x]
                        if center_depth_raw > 0 and center_depth_raw < 65535:
                            center_depth_mm = center_depth_raw * self.depth_scale
                            center_depth_m = center_depth_mm / 1000.0
                            depth_text = f'Depth: {center_depth_m:.3f}m ({center_depth_raw})'
                            cv2.putText(binary_display, depth_text, (x, y + h + 40),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            else:
                # 如果没有检测到轮廓，清除边界框
                with self.lock:
                    self.detected_bbox = None
            
            # 添加信息文本（显示实际深度值，单位：毫米）
            info_text = f'Binary Depth (Min: {self.binary_threshold_min_mm}mm, Max: {self.binary_threshold_max_mm}mm)'
            cv2.putText(binary_display, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示轮廓统计信息
            total_contours = len(contours)
            filtered_count = len(filtered_contours)
            if filtered_count > 0:
                contour_count_text = f'Contours: {filtered_count}/{total_contours} (filtered) | Max Area: {max_area:.0f} | Range: [{self.contour_min_area}, {self.contour_max_area}]'
                cv2.putText(binary_display, contour_count_text, (10, 135),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            elif total_contours > 0:
                contour_count_text = f'Contours: {total_contours} (all filtered, range=[{self.contour_min_area}, {self.contour_max_area}])'
                cv2.putText(binary_display, contour_count_text, (10, 135),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            else:
                cv2.putText(binary_display, 'No contours found', (10, 135),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 显示统计信息
            if depth_image.dtype == np.uint16:
                total_pixels = depth_image.size
                in_range_mask = (depth_image >= self.binary_threshold_min) & (depth_image <= self.binary_threshold_max)
                invalid_mask = (depth_image == 0) | (depth_image == 65535)
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
            
            # 添加图例
            legend_y = binary_display.shape[0] - 80
            cv2.putText(binary_display, 'White: In Range', (10, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(binary_display, 'Black: Out Range', (10, legend_y + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(binary_display, 'Gray: Invalid Depth', (10, legend_y + 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            if self.show_contour:
                cv2.putText(binary_display, 'Green: Contour | Yellow: BBox | Blue: Center', (10, legend_y + 45),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 如果未启用，显示提示
            if not self.binary_enabled:
                overlay = binary_display.copy()
                cv2.rectangle(overlay, (0, 0), (binary_display.shape[1], binary_display.shape[0]), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.5, binary_display, 0.5, 0, binary_display)
                cv2.putText(binary_display, 'Enable Binary to see result', 
                           (binary_display.shape[1]//2 - 150, binary_display.shape[0]//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow(self.binary_window_name, binary_display)
            
        except Exception as e:
            self.get_logger().error(f'二值化深度图像处理失败: {e}')
            import traceback
            traceback.print_exc()
    
    def run(self):
        """运行主循环"""
        try:
            while rclpy.ok():
                # 处理ROS2回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # 显示图像
                self.display_depth_image()
                self.display_color_image()
                self.display_binary_depth_image()
                
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                
                # 通用快捷键（手动和自动模式都可用）
                if key == ord('q') or key == ord('Q'):  # Q键退出
                    break
                elif key == ord('+') or key == ord('='):  # +/=键：增加最小阈值
                    self.adjust_binary_threshold_min(1)
                elif key == ord('-') or key == ord('_'):  # -/_键：减少最小阈值
                    self.adjust_binary_threshold_min(-1)
                elif key == ord(']') or key == ord('}'):  # ]/}键：增加最大阈值
                    self.adjust_binary_threshold_max(1)
                elif key == ord('[') or key == ord('{'):  # [/{键：减少最大阈值
                    self.adjust_binary_threshold_max(-1)
                
                # 仅在手动模式下处理触发
                if not self.auto_trigger:
                    if key == ord(' '):  # 空格键触发
                        self.trigger_camera()
        
        except KeyboardInterrupt:
            # 退出前自动保存阈值
            self.save_thresholds()
        finally:
            cv2.destroyAllWindows()


def main(args=None):
    """主函数"""
    parser = argparse.ArgumentParser(
        description='使用软件触发服务获取深度图像',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 手动触发模式（默认）
  python3 trigger_depth.py
  
  # 指定相机名称
  python3 trigger_depth.py my_camera
  
  # 自动触发模式，间隔2秒
  python3 trigger_depth.py --auto --interval 2.0
        """
    )
    
    parser.add_argument(
        'camera_name',
        nargs='?',
        default='camera',
        help='相机名称（默认: camera）'
    )
    
    parser.add_argument(
        '--auto',
        action='store_true',
        help='启用自动触发模式'
    )
    
    parser.add_argument(
        '--interval',
        type=float,
        default=1.0,
        help='自动触发间隔（秒，默认: 1.0）'
    )
    
    parser.add_argument(
        '--depth-scale',
        type=float,
        default=0.25,
        help='深度缩放因子（默认: 0.25）'
    )
    
    args = parser.parse_args()
    
    # rclpy.init() 需要接收 sys.argv 或 None，不能接收 argparse.Namespace
    rclpy.init(args=None)
    
    try:
        node = TriggerDepthNode(
            camera_name=args.camera_name,
            auto_trigger=args.auto,
            trigger_interval=args.interval,
            depth_scale=args.depth_scale
        )
        node.run()
    except Exception as e:
        print(f'错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
