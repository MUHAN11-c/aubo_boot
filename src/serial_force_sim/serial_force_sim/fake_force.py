#!/usr/bin/env python3
"""仿真 5 点力源：不接硬件，替代 serial_force_node 的 /force/points。

虚拟平面相对压头法线倾斜 (tilt_deg, tilt_yaw_deg)，5 点按线性弹簧模型
f = k·max(0, preload − 压入量) 取力；输出与真机相同顺序的 5 值（1/2/3/5/7）。
详见 serial_force_sim 包 doc/force_align_sim.md；与真机驱动 serial_force 分离。

ROS 参数:
  tilt_deg      虚拟平面倾角，deg（默认 3.0）
  tilt_yaw_deg  倾角方位（绕压头法线），deg（默认 0.0）
  preload_mm    预载压入量，mm（默认 0.30）
  k_n_per_mm    弹簧刚度，N/mm（默认 0.8）
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

from serial_force_sim.common import CH, PADS, Z0

K_N_PER_MM = 0.8
PRELOAD_MM = 0.30
TILT_DEG = 3.0


def axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a) + 1e-12
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FakeForce(Node):
    """把当前 tcp 位姿投影到一张固定虚拟平面上，发布 5 点力。"""

    def __init__(self):
        super().__init__('fake_force_sim')
        self.declare_parameter('tilt_deg', TILT_DEG)
        self.declare_parameter('tilt_yaw_deg', 0.0)
        self.declare_parameter('preload_mm', PRELOAD_MM)
        self.declare_parameter('k_n_per_mm', K_N_PER_MM)
        self.tilt_deg = float(self.get_parameter('tilt_deg').value)
        self.tilt_yaw_deg = float(self.get_parameter('tilt_yaw_deg').value)
        self.preload_mm = float(self.get_parameter('preload_mm').value)
        self.k_n_per_mm = float(self.get_parameter('k_n_per_mm').value)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.pub_f = self.create_publisher(Float64MultiArray, '/force/points', 10)
        self.pub_t = self.create_publisher(Float64MultiArray, '/force/travel', 10)
        self.pub_a = self.create_publisher(Float64MultiArray, '/force/ad', 10)
        self.pub_s = self.create_publisher(UInt8MultiArray, '/force/status', 10)
        self.plane_n = None
        self.plane_p = None
        self.create_timer(0.01, self.tick)
        self.get_logger().info('fake force: waiting for base_link->tcp')

    def tick(self):
        try:
            tf = self.buf.lookup_transform(
                'base_link', 'tcp', rclpy.time.Time())
        except Exception:
            return
        tr = tf.transform
        R = quat_matrix(tr.rotation)
        p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        pad_n = R @ np.array([0.0, 0.0, 1.0])
        pad_c = p + R @ np.array([0.0, 0.0, Z0])
        if self.plane_n is None:
            th = math.radians(self.tilt_deg)
            yaw = math.radians(self.tilt_yaw_deg)
            axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            self.plane_n = axis_angle(axis, th) @ pad_n
            self.plane_n /= np.linalg.norm(self.plane_n)
            self.plane_p = pad_c
            self.get_logger().info(
                f'virtual plane: tilt {self.tilt_deg:+.2f} deg yaw '
                f'{self.tilt_yaw_deg:+.1f} deg, normal '
                f'{np.round(self.plane_n, 4)}')
        forces, ads, trs, sts = [], [], [], []
        for ch in CH:
            x, y = PADS[ch]
            q = p + R @ np.array([x / 1000.0, y / 1000.0, Z0])
            s_mm = float((q - self.plane_p) @ self.plane_n) * 1000.0
            f = self.k_n_per_mm * max(0.0, self.preload_mm - s_mm)
            forces.append(f)
            trs.append(f / 0.647)              # 66 gf/mm -> N/mm
            ads.append(400.0 + f * 450.0)      # fake raw AD for display
            sts.append(1 if f > 3.63 else 0)
        self.pub_f.publish(Float64MultiArray(data=forces))
        self.pub_t.publish(Float64MultiArray(data=trs))
        self.pub_a.publish(Float64MultiArray(data=ads))
        self.pub_s.publish(UInt8MultiArray(data=sts))


def main():
    rclpy.init()
    n = FakeForce()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
