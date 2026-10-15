"""
odom→platform_base_link TF 中继（OdometryPublisher 的 gz 帧名不可控）.

gz-sim OdometryPublisher 的 TF 子帧用模型 canonical link 名
（harvester/base_footprint），不能通过 <robot_frame> 参数覆盖为
platform_base_link。本节点订 /odom 并转发正确的 TF——标准 ROS 2
导航栈模式（Nav2 官方 tutorial 同类：odom_tf_broadcaster）。
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):

    def __init__(self):
        super().__init__('odom_tf_relay')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.get_logger().info('odom→platform_base_link TF 中继就绪')

    def _on_odom(self, msg: Odometry):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'platform_base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # SIGINT 时 rclpy 信号处理器已 shutdown，重复调用会抛 RCLError
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
