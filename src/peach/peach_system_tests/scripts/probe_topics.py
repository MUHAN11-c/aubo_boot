#!/usr/bin/env python3
"""
话题帧率探针（对已起栈的相机流复核；RELIABLE 订阅）.

09-29 真相机轮勘定：percipio/stereo 发布端是 RELIABLE，BEST_EFFORT 订户
在 FastDDS 大帧下会大量丢包（RViz 正常而 BE 探针 0 帧的假故障），探针
必须与发布端可靠性对齐。收编自 /tmp 手工探针。

用法：
  ros2 run peach_system_tests probe_topics.py --duration 12 \
      /camera/depth/image_raw /camera/depth/points
"""
import argparse
import sys
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from sensor_msgs.msg import Image, PointCloud2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument(
        '--reliable', action='store_true', default=True,
        help='RELIABLE 订阅（默认，对齐发布端；关掉用 --no-reliable）')
    parser.add_argument('--no-reliable', dest='reliable', action='store_false')
    parser.add_argument(
        'topics', nargs='+', default=[], help='话题名（sensor_msgs/Image|PointCloud2）')
    args = parser.parse_args()
    topics = args.topics or [
        '/camera/color/image_raw', '/camera/depth/image_raw',
        '/camera/depth/points',
    ]

    qos = QoSProfile(
        reliability=(ReliabilityPolicy.RELIABLE if args.reliable
                     else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST, depth=5,
    )

    rclpy.init()
    node = Node('probe_topics')
    counts: Dict[str, int] = {t: 0 for t in topics}
    sizes: Dict[str, tuple] = {}
    import time
    deadline = time.monotonic() + args.duration

    def make_cb(topic):
        def cb(msg):
            counts[topic] += 1
            if isinstance(msg, PointCloud2):
                sizes[topic] = (msg.width * msg.height, msg.header.frame_id)
            elif isinstance(msg, Image):
                sizes[topic] = (f'{msg.width}x{msg.height}', msg.encoding)
        return cb

    for topic in topics:
        # 两类都试订：实际只有一类的会静默不匹配（计数为 0 即可判）。
        node.create_subscription(Image, topic, make_cb(topic), qos)
        node.create_subscription(PointCloud2, topic, make_cb(topic), qos)

    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'duration={args.duration:.1f}s reliability={"RELIABLE" if args.reliable else "BEST_EFFORT"}')
    all_zero = True
    for topic in topics:
        count = counts[topic]
        all_zero &= count == 0
        rate = count / args.duration
        info = sizes.get(topic, '-')
        print(f'{topic}: {count} 帧 (~{rate:.1f}Hz) 首帧规格={info}')
    if all_zero:
        print('ALL_TOPICS_ZERO（先查 QoS 匹配与发布端，再怀疑相机）')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
