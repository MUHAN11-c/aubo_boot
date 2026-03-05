#!/usr/bin/env python3
"""
简单测试 /movel 服务：发送目标位姿，打印成功/失败与错误码。
需先启动含 move_group + movel_server 的 launch（如 graspnet_demo）。

用法:
  ros2 run demo_driver test_movel_service.py
  ros2 run demo_driver test_movel_service.py --x 0.4 --y 0.0 --z 0.3
  ros2 run demo_driver test_movel_service.py -0.667 0.048 0.258
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from demo_interface.srv import Movel
import sys


def main():
    rclpy.init()
    node = Node("test_movel_client")

    # 默认目标位姿
    x, y, z = -0.4345, 0.0843, 0.3398
    qx, qy, qz, qw = 0.7074, -0.7068, 0.0005, 0.0001
    vel, acc = 0.15, 0.1

    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(argv) >= 3:
        try:
            x, y, z = float(argv[0]), float(argv[1]), float(argv[2])
        except ValueError:
            pass
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg == "--x" and i < len(sys.argv) - 1:
            x = float(sys.argv[i + 1])
        elif arg == "--y" and i < len(sys.argv) - 1:
            y = float(sys.argv[i + 1])
        elif arg == "--z" and i < len(sys.argv) - 1:
            z = float(sys.argv[i + 1])

    client = node.create_client(Movel, "/movel")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("服务 /movel 不可用，请先启动 graspnet_demo 或含 movel_server 的 launch")
        rclpy.shutdown()
        return 1

    req = Movel.Request()
    req.target_pose.position.x = x
    req.target_pose.position.y = y
    req.target_pose.position.z = z
    req.target_pose.orientation.x = qx
    req.target_pose.orientation.y = qy
    req.target_pose.orientation.z = qz
    req.target_pose.orientation.w = qw
    req.velocity_factor = vel
    req.acceleration_factor = acc

    node.get_logger().info(
        "调用 /movel: xyz=[%.3f, %.3f, %.3f] xyzw=[%.2f, %.2f, %.2f, %.2f] vel=%.2f acc=%.2f"
        % (x, y, z, qx, qy, qz, qw, vel, acc)
    )
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)

    if not future.done():
        node.get_logger().error("调用超时")
        rclpy.shutdown()
        return 1
    res = future.result()
    if res is None:
        node.get_logger().error("无响应")
        rclpy.shutdown()
        return 1

    if res.success:
        node.get_logger().info("movel 成功 (error_code=%d)" % res.error_code)
    else:
        node.get_logger().warn("movel 失败 error_code=%d message=%s" % (res.error_code, res.message))
    rclpy.shutdown()
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())
