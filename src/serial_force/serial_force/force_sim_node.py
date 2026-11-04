#!/usr/bin/env python3
"""Simulated five-point force source with the same ROS 2 contract as the driver."""
from __future__ import annotations

import math
import uuid

import numpy as np
from peach_interfaces.msg import ForceSample
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt8MultiArray
import tf2_ros

from serial_force.force_sim_common import CH  # noqa: I100
from serial_force.force_sim_common import PADS  # noqa: I100
from serial_force.force_sim_common import Z0  # noqa: I100

K_N_PER_MM = 0.8
PRELOAD_MM = 0.30
TILT_DEG = 3.0


def _axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a) + 1e-12
    cross = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)],
    ])


class ForceSimNode(Node):
    """Publish /force/sample and legacy force arrays from a virtual plane."""

    def __init__(self):
        """Read sim parameters; TF and publishers are created once."""
        super().__init__('serial_force')
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
        self.pub_sample = self.create_publisher(ForceSample, '/force/sample', 10)
        self.pub_force = self.create_publisher(Float64MultiArray, '/force/points', 10)
        self.pub_travel = self.create_publisher(Float64MultiArray, '/force/travel', 10)
        self.pub_ad = self.create_publisher(Float64MultiArray, '/force/ad', 10)
        self.pub_status = self.create_publisher(UInt8MultiArray, '/force/status', 10)
        self.source_uuid = f'sim-{uuid.uuid4()}'
        self.sequence = 0
        self.plane_n = None
        self.plane_p = None
        self.create_timer(0.01, self.tick)
        self.get_logger().info('force sim: waiting for base_link->tcp')

    def tick(self):
        """Sample the virtual plane and publish one atomic frame plus arrays."""
        try:
            tf = self.buf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
        except Exception:
            return
        tr = tf.transform
        rotation = _quat_matrix(tr.rotation)
        position = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        pad_n = rotation @ np.array([0.0, 0.0, 1.0])
        pad_c = position + rotation @ np.array([0.0, 0.0, Z0])
        if self.plane_n is None:
            theta = math.radians(self.tilt_deg)
            yaw = math.radians(self.tilt_yaw_deg)
            axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            self.plane_n = _axis_angle(axis, theta) @ pad_n
            self.plane_n /= np.linalg.norm(self.plane_n)
            self.plane_p = pad_c
            self.get_logger().info(
                f'virtual plane tilt={self.tilt_deg:+.2f} deg '
                f'yaw={self.tilt_yaw_deg:+.1f} deg')
        forces, travels, ads, statuses = [], [], [], []
        for channel in CH:
            x, y = PADS[channel]
            point = position + rotation @ np.array([x / 1000.0, y / 1000.0, Z0])
            depth_mm = float((point - self.plane_p) @ self.plane_n) * 1000.0
            force = self.k_n_per_mm * max(0.0, self.preload_mm - depth_mm)
            forces.append(force)
            travels.append(force / 0.647)
            ads.append(int(max(0, min(65535, 400.0 + force * 450.0))))
            statuses.append(1 if force > 3.63 else 0)
        now = self.get_clock().now().to_msg()
        sample = ForceSample()
        sample.header.stamp = now
        sample.header.frame_id = 'tcp'
        sample.source_uuid = self.source_uuid
        sample.sequence = self.sequence
        sample.channel_ids = CH
        sample.force_n = forces
        sample.travel_mm = travels
        sample.ad = ads
        sample.status = statuses
        self.pub_sample.publish(sample)
        self.pub_force.publish(Float64MultiArray(data=forces))
        self.pub_travel.publish(Float64MultiArray(data=travels))
        self.pub_ad.publish(Float64MultiArray(data=[float(v) for v in ads]))
        self.pub_status.publish(UInt8MultiArray(data=statuses))
        self.sequence += 1


def main(args=None):
    """Run the simulated force source."""
    rclpy.init(args=args)
    node = ForceSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
