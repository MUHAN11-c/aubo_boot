#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 发布客户端：手动触发抓取结果发布

功能：
  - 调用 /publish_grasps 服务发布抓取位姿、MarkerArray 和点云

使用方法：
  python3 publish_grasps_client.py
  或
  ./publish_grasps_client.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class PublishGraspsClient(Node):
    """抓取发布客户端节点"""
    
    def __init__(self):
        super().__init__('publish_grasps_client')
        
        # 创建发布服务客户端
        self.client = self.create_client(Trigger, 'publish_grasps')
        
        # 等待服务可用
        self.get_logger().info('等待 /publish_grasps 服务...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，继续等待...')
        
        self.get_logger().info('服务已连接')
    
    def call_service(self):
        """调用发布服务"""
        request = Trigger.Request()
        
        self.get_logger().info('发送发布请求...')
        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'✗ 失败: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {str(e)}')
            return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        client = PublishGraspsClient()
        success = client.call_service()
        
        client.destroy_node()
        rclpy.shutdown()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print('\n用户中断')
        sys.exit(1)
    except Exception as e:
        print(f'错误: {str(e)}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
