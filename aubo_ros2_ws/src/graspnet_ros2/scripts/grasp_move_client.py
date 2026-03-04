#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取移动客户端：调用 /grasp_move_to_pose 服务（独立脚本版本）

功能：
  - 调用 /grasp_move_to_pose 服务触发机械臂移动到抓取位姿

使用方法：
  python3 grasp_move_client.py
  或
  ./grasp_move_client.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class GraspMoveClient(Node):
    """抓取移动客户端节点"""
    
    def __init__(self):
        super().__init__('grasp_move_client')
        
        # 创建服务客户端
        self.client = self.create_client(Trigger, 'grasp_move_to_pose')
        
        # 等待服务可用
        self.get_logger().info('等待 /grasp_move_to_pose 服务...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，继续等待...')
        
        self.get_logger().info('服务已连接')
    
    def call_service(self):
        """调用服务"""
        request = Trigger.Request()
        
        self.get_logger().info('发送抓取移动请求...')
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
        client = GraspMoveClient()
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
