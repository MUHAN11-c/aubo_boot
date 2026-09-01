"""串口 IMU 节点：imu_tools 话题布局，只收不发."""
from __future__ import annotations

import os
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, Temperature
import serial
from serial_imu.protocol import feed, ImuSample
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# 协议 mag 单位高斯；sensor_msgs/MagneticField 用特斯拉。
GAUSS_TO_TESLA = 1.0e-4
SERIAL_RETRY_S = 2.0


def _diag9(v: float) -> list[float]:
    """3x3 对角协方差（行优先）."""
    return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]


class SerialImuNode(Node):
    """打开固定串口，解析 0xA4 主动上报，发 /imu/* 与 TF."""

    def __init__(self):
        super().__init__('serial_imu')
        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter(
            'port_fallbacks',
            ['/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
             '/dev/ttyUSB0'])
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('read_period_s', 0.005)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_parent_frame', 'world')
        self.declare_parameter('publish_attitude_tf', True)
        self.declare_parameter('attitude_frame_id', 'imu_attitude')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._tf_parent = str(self.get_parameter('tf_parent_frame').value)
        self._publish_attitude_tf = bool(
            self.get_parameter('publish_attitude_tf').value)
        self._attitude_frame = str(
            self.get_parameter('attitude_frame_id').value)
        qos = qos_profile_sensor_data
        self._pub_data = self.create_publisher(Imu, 'imu/data', qos)
        self._pub_raw = self.create_publisher(Imu, 'imu/data_raw', qos)
        self._pub_mag = self.create_publisher(MagneticField, 'imu/mag', qos)
        self._pub_temp = self.create_publisher(Temperature, 'imu/temp', qos)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._buf = bytearray()
        self._ser = None
        self._last_serial_try = 0.0
        self._logged_serial_fail = False
        if self._publish_tf:
            self._send_mount_tf()
        self._try_open_serial()
        period = float(self.get_parameter('read_period_s').value)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'mount={self._tf_parent}->{self._frame_id}  '
            '话题 imu/data imu/data_raw imu/mag imu/temp')

    def _try_open_serial(self) -> bool:
        """按 port 与 fallbacks 打开；失败不抛，返回 False."""
        self._last_serial_try = time.monotonic()
        port = str(self.get_parameter('port').value)
        fallbacks = list(self.get_parameter('port_fallbacks').value)
        candidates = [port] + [str(p) for p in fallbacks if str(p) != port]
        baud = int(self.get_parameter('baudrate').value)
        last_error = ''
        for path in candidates:
            if not os.path.exists(path):
                continue
            try:
                self._ser = serial.Serial(path, baud, timeout=0.0)
                self.get_logger().info(f'已打开 {path}')
                self._logged_serial_fail = False
                return True
            except (OSError, serial.SerialException) as exc:
                last_error = f'{path}: {exc}'
        if not self._logged_serial_fail:
            self.get_logger().error(
                '打不开 IMU 串口（节点继续发 world TF，稍后重试）。'
                f' 候选: {", ".join(candidates)}'
                + (('；' + last_error) if last_error else '')
                + '。当前终端: sudo usermod -aG dialout $USER && newgrp dialout')
            self._logged_serial_fail = True
        self._ser = None
        return False

    def _send_mount_tf(self):
        """静态安装位：parent→imu_link 单位姿态，姿态不写进 imu_link."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._tf_parent
        msg.child_frame_id = self._frame_id
        msg.transform.rotation.w = 1.0
        self._static_tf.sendTransform(msg)

    def _stamp_header(self, msg):
        """写入当前 stamp 与 frame_id."""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

    def _imu_common(self, sample: ImuSample) -> Imu:
        """加速度与角速度（SI）；不含姿态."""
        msg = Imu()
        self._stamp_header(msg)
        msg.angular_velocity.x = sample.gyro_x
        msg.angular_velocity.y = sample.gyro_y
        msg.angular_velocity.z = sample.gyro_z
        msg.linear_acceleration.x = sample.acc_x
        msg.linear_acceleration.y = sample.acc_y
        msg.linear_acceleration.z = sample.acc_z
        msg.angular_velocity_covariance[:] = _diag9(0.02)
        msg.linear_acceleration_covariance[:] = _diag9(0.04)
        return msg

    def _publish_sample(self, sample: ImuSample):
        """按 imu_tools 布局发 fused / raw / mag / temp，可选姿态 TF."""
        fused = self._imu_common(sample)
        fused.orientation.x = sample.orientation_x
        fused.orientation.y = sample.orientation_y
        fused.orientation.z = sample.orientation_z
        fused.orientation.w = sample.orientation_w
        fused.orientation_covariance[:] = _diag9(0.01)
        self._pub_data.publish(fused)

        raw = self._imu_common(sample)
        raw.orientation_covariance[0] = -1.0
        self._pub_raw.publish(raw)

        mag = MagneticField()
        self._stamp_header(mag)
        mag.magnetic_field.x = sample.mag_x * GAUSS_TO_TESLA
        mag.magnetic_field.y = sample.mag_y * GAUSS_TO_TESLA
        mag.magnetic_field.z = sample.mag_z * GAUSS_TO_TESLA
        mag.magnetic_field_covariance[:] = _diag9(1.0e-10)
        self._pub_mag.publish(mag)

        temp = Temperature()
        self._stamp_header(temp)
        temp.temperature = sample.temp_c
        temp.variance = 0.0
        self._pub_temp.publish(temp)

        if self._publish_attitude_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = fused.header.stamp
            tf_msg.header.frame_id = self._tf_parent
            tf_msg.child_frame_id = self._attitude_frame
            tf_msg.transform.rotation = fused.orientation
            self._tf.sendTransform(tf_msg)

    def _on_timer(self):
        """读串口缓冲并发布完整帧；口未开则定期重试."""
        if self._ser is None:
            if time.monotonic() - self._last_serial_try >= SERIAL_RETRY_S:
                self._try_open_serial()
            return
        waiting = self._ser.in_waiting
        if waiting:
            self._buf.extend(self._ser.read(waiting))
        for sample in feed(self._buf):
            self._publish_sample(sample)

    def destroy_node(self):
        """关串口再销毁节点."""
        if getattr(self, '_ser', None) is not None and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    """入口."""
    rclpy.init(args=args)
    node = SerialImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
