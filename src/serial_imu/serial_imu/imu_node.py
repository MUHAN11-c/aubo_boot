"""串口 IMU 节点：imu_tools 话题布局，只收不发."""
from __future__ import annotations

import os
import time

from geometry_msgs.msg import QuaternionStamped, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField, Temperature
import serial
from serial_imu.protocol import feed, ImuSample
from serial_imu.tool_offset import (
    from_rpy_deg, qangle, ToolOffsetEstimator, zero_gate_ok)
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener)
from tf2_ros import TransformException as Tf2Exception

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
        self.declare_parameter('tool_offset.enabled', False)
        self.declare_parameter('tool_offset.base_frame', 'base_link')
        self.declare_parameter('tool_offset.nominal_frame', 'tool_axis')
        self.declare_parameter('tool_offset.output_frame', 'tcp_actual')
        # 2026-09-09 实测模组倒装 roll≈+177°；pitch/yaw 须在工具竖直零位时标定回填
        self.declare_parameter('tool_offset.mount_rpy_deg', [177.0, 0.0, 0.0])
        self.declare_parameter('tool_offset.pivot_depth_m', 0.0)
        self.declare_parameter('tool_offset.zero_on_start', True)
        self.declare_parameter('tool_offset.max_tilt_deg', 15.0)
        self.declare_parameter('tool_offset.zero_gravity_check', True)
        self.declare_parameter('tool_offset.zero_gravity_tool', [0.0, 0.0, 1.0])
        self.declare_parameter('tool_offset.zero_gravity_tol_deg', 5.0)

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
        self._last_q = None
        self._last_acc = None
        self._setup_tool_offset()
        if self._publish_tf:
            self._send_mount_tf()
        self._try_open_serial()
        period = float(self.get_parameter('read_period_s').value)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'mount={self._tf_parent}->{self._frame_id}  '
            '话题 imu/data imu/data_raw imu/mag imu/temp'
            + self._tool_offset_log())

    def _setup_tool_offset(self):
        """工具偏移（自适应圆柱工具 B）：TF 监听、偏移发布、清零服务."""
        pre = 'tool_offset.'
        self._to_enabled = bool(self.get_parameter(pre + 'enabled').value)
        if not self._to_enabled:
            return
        self._to_base = str(self.get_parameter(pre + 'base_frame').value)
        self._to_nominal = str(self.get_parameter(pre + 'nominal_frame').value)
        self._to_output = str(self.get_parameter(pre + 'output_frame').value)
        self._to_zero_on_start = bool(
            self.get_parameter(pre + 'zero_on_start').value)
        self._to_max_tilt = float(self.get_parameter(pre + 'max_tilt_deg').value)
        self._to_gcheck = bool(
            self.get_parameter(pre + 'zero_gravity_check').value)
        self._to_gtool = tuple(
            float(v) for v in self.get_parameter(pre + 'zero_gravity_tool').value)
        self._to_gtol = float(
            self.get_parameter(pre + 'zero_gravity_tol_deg').value)
        mount = from_rpy_deg(
            [float(v) for v in self.get_parameter(pre + 'mount_rpy_deg').value])
        self._to_mount = mount
        self._est = ToolOffsetEstimator(
            mount, float(self.get_parameter(pre + 'pivot_depth_m').value))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self,
                                              spin_thread=True)
        qos = qos_profile_sensor_data
        self._pub_offset = self.create_publisher(
            QuaternionStamped, 'imu/tool_offset', qos)
        self._srv_zero = self.create_service(
            Trigger, 'imu/tool_offset/zero', self._on_zero_srv)

    def _tool_offset_log(self):
        if not self._to_enabled:
            return ''
        return (f'  工具偏移: {self._to_nominal}->{self._to_output} '
                f'mount_rpy_deg={tuple(self.get_parameter("tool_offset.mount_rpy_deg").value)}')

    def _lookup_base_tool(self):
        """最新 base→tool_axis 姿态四元数；不可得返回 None."""
        try:
            if not self._tf_buffer.can_transform(self._to_base,
                                                 self._to_nominal, Time()):
                return None
            t = self._tf_buffer.lookup_transform(self._to_base,
                                                 self._to_nominal, Time())
        except Tf2Exception:
            return None
        q = t.transform.rotation
        return (q.w, q.x, q.y, q.z)

    def _try_zero(self, q_imu, acc):
        """清零（零位采集）：需 TF 可得且重力自检通过；返回 (ok, 原因)."""
        q_bt = self._lookup_base_tool()
        if q_bt is None:
            return False, f'TF {self._to_base}->{self._to_nominal} 不可得'
        if (self._to_gcheck
                and not zero_gate_ok(acc, self._to_mount, self._to_gtool,
                                     self._to_gtol)):
            return False, ('重力自检未过：实测重力方向与预期零位姿态不符'
                           '（倒装/贴歪/不在零位?）')
        if not self._est.zero(q_imu, q_bt):
            return False, '模组姿态无效'
        return True, '零位已采集'

    def _on_zero_srv(self, request, response):
        """清零服务：用最近一帧 IMU 即时执行，同步返回结果."""
        del request
        if self._last_q is None:
            response.success = False
            response.message = '暂无 IMU 数据'
            return response
        ok, msg = self._try_zero(self._last_q, self._last_acc)
        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info('零位已采集（服务触发）')
        return response

    def _tool_offset_tick(self, stamp):
        """每采样：未清零则尝试自动清零；已清零则解算并发布偏移."""
        if not self._est.zeroed:
            if self._to_zero_on_start:
                ok, msg = self._try_zero(self._last_q, self._last_acc)
                if ok:
                    self.get_logger().info('零位已采集（启动自动清零）')
                else:
                    self.get_logger().warning(
                        f'自动清零未完成: {msg}', throttle_duration_sec=10.0)
            return
        q_bt = self._lookup_base_tool()
        if q_bt is None:
            self.get_logger().warning(
                f'TF {self._to_base}->{self._to_nominal} 不可得，偏移停更'
                '（消费端须按 stamp 新鲜度停止套入）', throttle_duration_sec=5.0)
            return
        res = self._est.update(self._last_q, q_bt)
        if res is None:
            return
        q_off, trans = res
        tilt = qangle(q_off, (1.0, 0.0, 0.0))
        if tilt > self._to_max_tilt:
            self.get_logger().warning(
                f'工具偏移 {tilt:.1f}° 超过 max_tilt_deg={self._to_max_tilt:.0f}'
                '（仍照发，停套入由上层判）', throttle_duration_sec=2.0)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._to_nominal
        tf_msg.child_frame_id = self._to_output
        tf_msg.transform.rotation.w = q_off[0]
        tf_msg.transform.rotation.x = q_off[1]
        tf_msg.transform.rotation.y = q_off[2]
        tf_msg.transform.rotation.z = q_off[3]
        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]
        tf_msg.transform.translation.z = trans[2]
        self._tf.sendTransform(tf_msg)
        off_msg = QuaternionStamped()
        off_msg.header.stamp = stamp
        off_msg.header.frame_id = self._to_nominal
        off_msg.quaternion.w = q_off[0]
        off_msg.quaternion.x = q_off[1]
        off_msg.quaternion.y = q_off[2]
        off_msg.quaternion.z = q_off[3]
        self._pub_offset.publish(off_msg)

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

        self._last_q = (fused.orientation.w, fused.orientation.x,
                        fused.orientation.y, fused.orientation.z)
        self._last_acc = (sample.acc_x, sample.acc_y, sample.acc_z)
        if self._to_enabled:
            self._tool_offset_tick(fused.header.stamp)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
