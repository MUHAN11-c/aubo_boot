#!/usr/bin/env python3
"""
发布夹爪关节状态的简单节点.

用于解决 MoveIt2 缺少 grap0_joint 和 grap1_joint 状态的警告。
这些关节被视为末端执行器的固定部分，发布默认状态 0.0。
2026-09-30 自 aubo_boot 移植（IVG 演示栈；原放 aubo_moveit_config，
按 IVG/peach 隔离裁定迁入本包）。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_state_publisher')

        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # 10Hz 发布
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info('Gripper joint state publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # 夹爪关节（默认 0，视为末端执行器固定部分）
        msg.name = ['grap0_joint', 'grap1_joint']
        msg.position = [0.0, 0.0]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
