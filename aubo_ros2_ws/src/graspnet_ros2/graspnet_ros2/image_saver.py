#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageSaver(Node):
    """ROS2节点，订阅彩色和深度图像话题并保存为PNG格式"""

    def __init__(self):
        super().__init__('image_saver')
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 保存路径
        self.save_dir = '/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline/doc/pose_1'
        
        # 确保保存目录存在
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 存储接收到的图像
        self.color_image = None
        self.depth_image = None
        self.color_received = False
        self.depth_received = False
        
        # 创建订阅者
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.get_logger().info('ImageSaver节点已启动')
        self.get_logger().info(f'图像将保存到: {self.save_dir}')
        self.get_logger().info('订阅话题: /camera/color/image_raw, /camera/depth/image_raw')
        self.get_logger().info('按 Ctrl+C 保存图像并退出')

    def color_callback(self, msg):
        """彩色图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            # 根据图像编码选择合适的转换方式
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
                # OpenCV使用BGR格式，需要转换
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            self.color_image = cv_image
            
            if not self.color_received:
                self.color_received = True
                self.get_logger().info(f'收到彩色图像: {cv_image.shape}, 编码: {msg.encoding}')
                
        except Exception as e:
            self.get_logger().error(f'彩色图像转换失败: {str(e)}')

    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            if msg.encoding == '16UC1':
                # 16位无符号整数深度图像
                cv_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                # 32位浮点深度图像，需要转换为16位
                cv_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                # 将米转换为毫米，并转换为16位无符号整数
                cv_image = (cv_image * 1000).astype(np.uint16)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                if cv_image.dtype == np.float32 or cv_image.dtype == np.float64:
                    cv_image = (cv_image * 1000).astype(np.uint16)
                elif cv_image.dtype != np.uint16:
                    cv_image = cv_image.astype(np.uint16)
            
            self.depth_image = cv_image
            
            if not self.depth_received:
                self.depth_received = True
                self.get_logger().info(f'收到深度图像: {cv_image.shape}, 编码: {msg.encoding}, 类型: {cv_image.dtype}')
                
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {str(e)}')

    def save_images(self):
        """保存图像到文件"""
        color_path = os.path.join(self.save_dir, 'color.png')
        depth_path = os.path.join(self.save_dir, 'depth.png')
        
        saved = False
        
        if self.color_image is not None:
            cv2.imwrite(color_path, self.color_image)
            self.get_logger().info(f'彩色图像已保存: {color_path}')
            saved = True
        else:
            self.get_logger().warn('未收到彩色图像，无法保存')
        
        if self.depth_image is not None:
            # 保存为16位PNG格式
            cv2.imwrite(depth_path, self.depth_image)
            self.get_logger().info(f'深度图像已保存: {depth_path}')
            saved = True
        else:
            self.get_logger().warn('未收到深度图像，无法保存')
        
        return saved


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到退出信号，正在保存图像...')
    finally:
        node.save_images()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
