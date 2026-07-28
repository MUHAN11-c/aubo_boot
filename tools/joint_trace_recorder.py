#!/usr/bin/env python3
"""录制 /joint_states 轨迹到 CSV，用于分析运动前后窗口的速度/位置行为。
用法: python3 tools/joint_trace_recorder.py <out.csv> <seconds>"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self, path, seconds):
        super().__init__("joint_trace_recorder")
        self._f = open(path, "w")
        self._f.write("t,foreArm_pos,foreArm_vel,upperArm_pos,upperArm_vel\n")
        self._end = time.time() + seconds
        self.create_subscription(JointState, "/joint_states", self._cb, 100)

    def _cb(self, msg):
        now = time.time()
        if now > self._end:
            return
        try:
            i_fa = msg.name.index("foreArm_joint")
            i_ua = msg.name.index("upperArm_joint")
        except ValueError:
            return
        vel_fa = msg.velocity[i_fa] if len(msg.velocity) > i_fa else 0.0
        vel_ua = msg.velocity[i_ua] if len(msg.velocity) > i_ua else 0.0
        self._f.write(
            f"{now:.3f},{msg.position[i_fa]:.6f},{vel_fa:.6f},"
            f"{msg.position[i_ua]:.6f},{vel_ua:.6f}\n")


def main():
    rclpy.init()
    node = Recorder(sys.argv[1], float(sys.argv[2]))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
