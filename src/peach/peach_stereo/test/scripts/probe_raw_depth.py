#!/usr/bin/env python3
"""原始深度探针：直接订 /camera/depth/image_raw，统计若干帧的覆盖率与数值分布."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self, n=12):
        super().__init__('depth_probe')
        self.n = n
        self.got = 0
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb, qos)

    def cb(self, msg):
        if msg.encoding == '16UC1':
            d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            unit = 0.25
        elif msg.encoding == '32FC1':
            d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
            d = np.round(d * 1000.0 / 0.25).astype(np.uint16)  # 统一到 0.25 单位
            unit = 0.25
        else:
            print('encoding', msg.encoding)
            return
        valid = (d > 4) & (d < 60000)
        win = (d * 0.25 >= 300) & (d * 0.25 <= 1500)
        v = d[valid]
        print(f'frame {self.got}: {msg.encoding} valid={valid.mean():.3f} '
              f'win={win.mean():.3f} '
              f'med={np.median(v) * 0.25 if v.size else 0:.0f}mm '
              f'p99={np.percentile(v, 99) * 0.25 if v.size else 0:.0f}mm '
              f'stamp={msg.header.stamp.sec}')
        self.got += 1
        if self.got >= self.n:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Probe()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 20 and node.got < node.n:
        rclpy.spin_once(node, timeout_sec=0.5)
    return 0


if __name__ == '__main__':
    sys.exit(main())
