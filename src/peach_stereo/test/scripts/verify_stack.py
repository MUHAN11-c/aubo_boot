#!/usr/bin/env python3
"""重启验证：深度健康（30 帧有效占比）+ PointCloud2 字段布局与字节级颜色校验.

用法（域 77）: python3 verify_stack.py
判据：valid_ratio(0.3–1.5m) 均值 <10% 即异常；stereo 档字段应 x/y/z@0/4/8、
rgb@16、confidence@20、point_step=24（percipio 档无 confidence 字段会打异常后退出，属预期）。
"""
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


class Verifier(Node):
    def __init__(self):
        super().__init__('restart_verifier')
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.depth_n = 0
        self.ratios = []
        self.meds = []
        self.points_seen = None
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.on_depth, qos)
        # 点云订户门控：有订户才发布
        self.create_subscription(PointCloud2, '/camera/depth_registered/points',
                                 self.on_points, qos)

    def on_depth(self, msg):
        d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        valid = (d > 0) & (d >= 1200) & (d <= 6000)  # 0.3–1.5m ×0.25mm
        self.ratios.append(float(valid.mean()))
        self.meds.append(float(np.median(d[valid])) * 0.25 if valid.any() else 0.0)
        self.depth_n += 1

    def on_points(self, msg):
        if self.points_seen is None:
            self.points_seen = msg


def main():
    rclpy.init()
    n = Verifier()
    import time
    t0 = time.time()
    while n.depth_n < 30 and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f'depth frames={n.depth_n} valid_ratio mean={np.mean(n.ratios) if n.ratios else -1:.4f} '
          f'min={np.min(n.ratios) if n.ratios else -1:.4f} med_mm={np.mean(n.meds) if n.meds else -1:.1f}')
    t0 = time.time()
    while n.points_seen is None and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = n.points_seen
    if m is None:
        print('POINTS: no message')
    else:
        names = [(f.name, f.offset) for f in m.fields]
        print(f'points: point_step={m.point_step} fields={names} n={m.width}')
        try:
            step = m.point_step
            for i in (0, m.width // 2, m.width - 1):
                b = m.data[i * step:(i + 1) * step]
                x, y, z = struct.unpack_from('<fff', b, 0)
                rgb_u32 = struct.unpack_from('<I', b, 16)[0]
                conf = struct.unpack_from('<f', b, 20)[0]
                print(f'  pt[{i}] xyz=({x:.3f},{y:.3f},{z:.3f}) rgb=0x{rgb_u32:06X} conf={conf:.3f}')
        except struct.error:
            pass  # percipio 档无 confidence 字段
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
