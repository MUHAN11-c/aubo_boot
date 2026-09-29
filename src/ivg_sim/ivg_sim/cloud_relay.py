#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_relay：gz rgbd 点云 → IVG 光学系话题中继.

gz 桥出的 PointCloud2（话题 /camera/depth/points、frame_id 为 scoped 传感器
名 camera_rig/link/rgbd、轴系=传感器体系，见 layout.GZ_OPTICAL_CONVENTION）
重写 header.frame_id 为 layout.camera.optical_frame（默认
camera_depth_optical_frame）后，以 RELIABLE 转发到
/camera/depth_registered/points——ivg_graspnet 检测节点零改动接入。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


class CloudRelay(Node):
    """帧名重写 + 可靠转发（订阅端 BEST_EFFORT 兼容任意发布端）."""

    def __init__(self):
        super().__init__('ivg_cloud_relay')
        self.declare_parameter('input_topic', '/camera/depth/points')
        self.declare_parameter('output_topic', '/camera/depth_registered/points')
        self.declare_parameter('optical_frame', 'camera_depth_optical_frame')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.optical_frame = self.get_parameter('optical_frame').value

        self.pub = self.create_publisher(PointCloud2, output_topic, 5)
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_cloud,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            ),
        )
        self.get_logger().info(
            f'点云中继: {input_topic} → {output_topic} '
            f'(frame_id → {self.optical_frame})'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.frame_id = self.optical_frame
        self.pub.publish(msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = CloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001 - SIGINT 后 rclpy 关闭竞态下的兜底
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
