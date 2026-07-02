#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制 demo_driver 的 publish_grasps_client_worker 循环抓取服务。

功能：
  - 调用 SetBool 服务开启循环抓取（data=true）
  - 调用 SetBool 服务关闭循环抓取（data=false）
  - 关闭后可选等待 worker 在当前周期结束后退出
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class PublishGraspsWorkerLoopControlClient(Node):
    def __init__(self, service_name: str):
        super().__init__('publish_grasps_worker_loop_control_client')
        self._service_name = service_name
        self._client = self.create_client(SetBool, service_name)

    def call(self, enable: bool, timeout_sec: float) -> bool:
        self.get_logger().info(
            f"等待服务 {self._service_name} (timeout={timeout_sec:.1f}s)..."
        )
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"服务不可用: {self._service_name}")
            return False

        req = SetBool.Request()
        req.data = enable
        self.get_logger().info(
            f"发送循环控制请求: {'start' if enable else 'stop'}"
        )
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error("服务调用超时")
            return False

        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"服务调用异常: {exc}")
            return False

        if resp is None:
            self.get_logger().error("服务响应为空")
            return False

        if resp.success:
            self.get_logger().info(f"成功: {resp.message}")
            return True

        self.get_logger().error(f"失败: {resp.message}")
        return False

    def wait_worker_exit(self, timeout_sec: float) -> bool:
        """
        关闭请求发送后，等待服务消失，表示 worker 已退出。
        若超时仍存在，返回 False。
        """
        if timeout_sec <= 0.0:
            return True

        self.get_logger().info(
            f"等待 worker 退出（检测服务下线），超时 {timeout_sec:.1f}s..."
        )
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            still_up = self._client.wait_for_service(timeout_sec=1.0)
            if not still_up:
                self.get_logger().info("检测到服务下线，worker 已退出")
                return True
            self.get_logger().info("worker 仍在运行，等待当前周期结束...")
        self.get_logger().error("等待 worker 退出超时")
        return False


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="控制 publish_grasps_client_worker 循环抓取开关"
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--start', action='store_true', help='开启循环抓取')
    mode.add_argument('--stop', action='store_true', help='关闭循环抓取（等待当前周期结束后退出）')
    parser.add_argument(
        '--service',
        default='/publish_grasps_worker_loop_control',
        help='循环控制服务名（std_srvs/SetBool）',
    )
    parser.add_argument(
        '--service-timeout',
        type=float,
        default=5.0,
        help='等待服务与调用超时（秒）',
    )
    parser.add_argument(
        '--wait-exit-timeout',
        type=float,
        default=180.0,
        help='stop 后等待 worker 退出超时（秒），<=0 表示不等待',
    )
    return parser.parse_args(argv)


def main(args=None):
    cli_args = parse_args(sys.argv[1:] if args is None else args)

    rclpy.init(args=None)
    node = PublishGraspsWorkerLoopControlClient(cli_args.service)
    ok = False
    try:
        if cli_args.start:
            ok = node.call(enable=True, timeout_sec=cli_args.service_timeout)
        else:
            ok = node.call(enable=False, timeout_sec=cli_args.service_timeout)
            if ok:
                ok = node.wait_worker_exit(timeout_sec=cli_args.wait_exit_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

