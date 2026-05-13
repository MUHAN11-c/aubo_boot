#!/usr/bin/env python3
"""
工具快换测试脚本 —— 通过 /change_tool 服务触发末端切换

用法：
  # 切换到 gripper0
  ros2 run tool_changer test_tool_change.py gripper0

  # 切换到 gripper2
  ros2 run tool_changer test_tool_change.py gripper2

  # 查询当前工具
  ros2 run tool_changer test_tool_change.py --status

执行后会在终端输出切换结果，同时在 RViz2 中可以看到夹爪模型的附着/脱离变化。
"""

import sys
import rclpy
from rclpy.node import Node
from ivg_interfaces.srv import ChangeTool, GetCurrentTool


class ToolChangeTester(Node):
    def __init__(self):
        super().__init__('tool_change_tester')

        self.change_client = self.create_client(ChangeTool, '/change_tool')
        self.status_client = self.create_client(GetCurrentTool, '/get_current_tool')

    def show_status(self):
        """查询并打印当前工具状态"""
        if not self.status_client.wait_for_service(5.0):
            self.get_logger().error('无法连接 /get_current_tool 服务')
            return False
        req = GetCurrentTool.Request()
        future = self.status_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            if res.tool_id:
                print(f'📌 当前工具: {res.tool_id} ({res.tool_name})')
            else:
                print('📌 当前没有工具附着在机械臂上')
            return True
        else:
            print(f'❌ 查询失败: {res.message}')
            return False

    def change_to(self, tool_id: str):
        """切换到指定工具"""
        print(f'\n⏳ 正在切换到: {tool_id} ...')
        if not self.change_client.wait_for_service(5.0):
            self.get_logger().error('无法连接 /change_tool 服务')
            return
        req = ChangeTool.Request()
        req.tool_id = tool_id
        future = self.change_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            print(f'✅ 切换成功: {res.message}')
        else:
            print(f'❌ 切换失败 (err={res.error_code}): {res.message}')


def main():
    rclpy.init()
    tester = ToolChangeTester()

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == '--status':
            tester.show_status()
        elif arg in ('gripper0', 'gripper2'):
            tester.show_status()
            tester.change_to(arg)
            tester.show_status()
        else:
            print(f'用法: {sys.argv[0]} [gripper0|gripper2|--status]')
    else:
        print(f'用法: {sys.argv[0]} [gripper0|gripper2|--status]')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
