#!/usr/bin/env python3
"""奶缸嘴标定工具 — 发布可实时调参的 TF + RViz 标记球喵~

用法:
  ros2 run latte_backend spout_calibrator.py

实时微调 (无需重启):
  ros2 param set /spout_calibrator ox -0.065
  ros2 param set /spout_calibrator oy 0.040
  ros2 param set /spout_calibrator oz 0.118

确认后把最终值写入 latte_backend 代码即可喵~
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import math


class SpoutCalibrator(Node):
    def __init__(self):
        super().__init__('spout_calibrator')

        # 默认偏移 (TCP 局部坐标系, 单位 m)
        self.declare_parameter('ox', -0.038)
        self.declare_parameter('oy', 0.095)
        self.declare_parameter('oz', 0.212)
        self.declare_parameter('radius', 0.004)      # 标记球半径
        self.declare_parameter('parent_frame', 'tool_tcp')
        self.declare_parameter('marker_frame', 'spout_marker')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._marker_pub = self.create_publisher(Marker, '/spout_marker', 10)

        self._timer = self.create_timer(0.05, self._publish)  # 20Hz

        self.get_logger().info(
            '奶缸嘴标定器已启动, 用 ros2 param set /spout_calibrator ox/oy/oz <value> 实时微调喵~')

    def _publish(self):
        ox = self.get_parameter('ox').value
        oy = self.get_parameter('oy').value
        oz = self.get_parameter('oz').value
        r  = self.get_parameter('radius').value
        parent = self.get_parameter('parent_frame').value
        child  = self.get_parameter('marker_frame').value

        now = self.get_clock().now().to_msg()

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = ox
        t.transform.translation.y = oy
        t.transform.translation.z = oz
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

        # RViz 绿色标记球
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = parent
        m.ns = 'spout_calibrator'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = ox
        m.pose.position.y = oy
        m.pose.position.z = oz
        m.scale.x = r * 2
        m.scale.y = r * 2
        m.scale.z = r * 2
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        self._marker_pub.publish(m)


def main():
    rclpy.init()
    node = SpoutCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
