#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的图像数据转换节点
将 sensor_msgs/Image 转换为 percipio_camera_interface/ImageData
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from percipio_camera_interface.msg import ImageData


class ImageDataConverterNode(Node):
    def __init__(self):
        super().__init__('image_data_converter_node')
        
        # 声明参数
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/image_data')
        self.declare_parameter('camera_id', 'DA3234363')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.camera_id = self.get_parameter('camera_id').value
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            ImageData,
            output_topic,
            10
        )
        
        self.get_logger().info(f'图像数据转换节点已启动')
        self.get_logger().info(f'  输入话题: {input_topic}')
        self.get_logger().info(f'  输出话题: {output_topic}')
        self.get_logger().info(f'  相机ID: {self.camera_id}')
    
    def image_callback(self, msg):
        """将 Image 转换为 ImageData"""
        try:
            image_data_msg = ImageData()
            image_data_msg.header = msg.header
            image_data_msg.camera_id = self.camera_id
            image_data_msg.image = msg
            
            self.publisher.publish(image_data_msg)
            
            self.get_logger().debug(
                f'转换并发布: 尺寸={msg.width}x{msg.height}, '
                f'编码={msg.encoding}, 大小={len(msg.data)/1024:.1f}KB'
            )
        except Exception as e:
            self.get_logger().error(f'转换失败: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageDataConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

