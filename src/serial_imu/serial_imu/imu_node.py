"""串口 IMU 节点：原始 / 修正两路，只收不发."""
from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater)
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField, Temperature
import serial
from serial_imu.frame import FramePipeline
from serial_imu.protocol import covariance_diag, feed
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from tf2_ros import TransformException as Tf2Exception

# 协议 mag 单位高斯；sensor_msgs/MagneticField 用特斯拉。
GAUSS_TO_TESLA = 1.0e-4
SERIAL_RETRY_S = 2.0
# USB 本机不丢包。RViz MessageFilterDisplay / ros2 topic echo 默认 Reliable。
IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SerialImuNode(Node):
    """打开固定串口，解析 0xA4 主动上报，发原始/修正两路."""

    def __init__(self):
        """声明参数、话题、对齐服务与读串口定时器."""
        super().__init__('serial_imu')
        self._declare_params()
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._tf_parent = str(self.get_parameter('tf_parent_frame').value)
        self._gyro_available = bool(
            self.get_parameter('gyro_available').value)
        self._gyro_var = float(
            self.get_parameter('angular_velocity_variance').value)
        self._acc_var = float(
            self.get_parameter('linear_acceleration_variance').value)
        self._ori_var = float(
            self.get_parameter('orientation_variance').value)
        self._mag_var = float(
            self.get_parameter('magnetic_field_variance').value)
        self._align_to_parent = bool(
            self.get_parameter('align_to_parent').value)
        self._align_on_start = bool(
            self.get_parameter('align_on_start').value)
        self._align_ref = str(
            self.get_parameter('align_reference_frame').value)
        self._pipe = FramePipeline(
            [float(v) for v in self.get_parameter('frame_rpy_deg').value])
        self._pub_data = self.create_publisher(Imu, 'imu/data', IMU_QOS)
        self._pub_raw = self.create_publisher(Imu, 'imu/data_raw', IMU_QOS)
        self._pub_mag = self.create_publisher(
            MagneticField, 'imu/mag', IMU_QOS)
        self._pub_temp = self.create_publisher(Temperature, 'imu/temp', IMU_QOS)
        self._static_tf = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._buf = bytearray()
        self._ser = None
        self._last_serial_try = 0.0
        self._logged_serial_fail = False
        self._last_sample = None
        self._pending_align = False
        self._setup_diagnostics()
        if self._align_to_parent:
            self.create_service(
                Trigger, 'imu/align_to_parent', self._on_align_srv)
        if self._publish_tf:
            self._broadcast_static_tf()
        self.create_timer(
            float(self.get_parameter('read_period_s').value), self._on_timer)
        self.get_logger().info(
            f'串口 IMU  parent={self._tf_parent}  '
            f'align_to_parent={self._align_to_parent}')

    def _p(self, name, default, description):
        """声明带中文 description 的参数."""
        self.declare_parameter(
            name, default, ParameterDescriptor(description=description))

    def _declare_params(self):
        self._p('port', '/dev/imu', '首选串口')
        self._p(
            'port_fallbacks',
            ['/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6060520-if00',
             '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
             '/dev/ttyUSB0'],
            'port 打不开时依次试')
        self._p('baudrate', 115200, '波特率')
        self._p('frame_id', 'imu_link', 'Imu header.frame_id')
        self._p('read_period_s', 0.005, '读串口定时器周期')
        self._p('publish_tf', True, '发静态 parent→imu_link')
        self._p('tf_parent_frame', 'world', 'imu_link 的父坐标系')
        self._p(
            'frame_rpy_deg', [180.0, 0.0, 0.0],
            '模组体轴→imu_link：标准倒装 Rx(180°)')
        self._p(
            'align_to_parent', False,
            '把当前 IMU↔parent 差当误差清掉')
        self._p('align_on_start', True, 'align_to_parent 时启动自动采一次')
        self._p(
            'align_reference_frame', 'base_link',
            '对齐查 TF：reference→parent')
        self._p('gyro_available', False, 'false → 陀螺 covariance[0]=-1')
        self._p('angular_velocity_variance', 0.0, '仅 gyro_available 时用')
        self._p('linear_acceleration_variance', 0.0, '加速度对角方差')
        self._p('orientation_variance', 0.0, '融合姿态对角方差')
        self._p('magnetic_field_variance', 0.0, '磁场对角方差')
        self._p('expected_rate_hz', 75.0, '/diagnostics 帧率期望')

    def _setup_diagnostics(self):
        self._updater = Updater(self)
        self._updater.setHardwareID('serial_imu')
        self._updater.add('serial', self._diag_serial)
        hz = float(self.get_parameter('expected_rate_hz').value)
        self._freq = HeaderlessTopicDiagnostic(
            '/imu/data', self._updater,
            FrequencyStatusParam({'min': hz * 0.6, 'max': hz * 1.4}, 0.1, 10))

    def _diag_serial(self, stat):
        if self._ser is not None and self._ser.is_open:
            stat.summary(DiagnosticStatus.OK, f'open {self._ser.port}')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'serial closed')
        return stat

    def _broadcast_static_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._tf_parent
        msg.child_frame_id = self._frame_id
        msg.transform.rotation.w = 1.0
        self._static_tf.sendTransform(msg)

    def _ports(self):
        seen = []
        for p in ([self.get_parameter('port').value]
                  + list(self.get_parameter('port_fallbacks').value)):
            p = str(p)
            if p and p not in seen:
                seen.append(p)
        return seen

    def _ensure_serial(self):
        if self._ser is not None and self._ser.is_open:
            return
        now = time.monotonic()
        if now - self._last_serial_try < SERIAL_RETRY_S:
            return
        self._last_serial_try = now
        baud = int(self.get_parameter('baudrate').value)
        last_err = None
        for port in self._ports():
            try:
                self._ser = serial.Serial(port, baud, timeout=0)
                self._logged_serial_fail = False
                self.get_logger().info(f'已打开 {port}')
                return
            except OSError as exc:
                last_err = exc
                self._ser = None
        if not self._logged_serial_fail:
            self._logged_serial_fail = True
            hint = ''
            if isinstance(last_err, PermissionError) or (
                    last_err and getattr(last_err, 'errno', None) == 13):
                hint = '  当前终端加 dialout：newgrp dialout'
            self.get_logger().error(
                f'打不开口 {self._ports()}: {last_err}{hint}')

    def _lookup_parent_wxyz(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._align_ref, self._tf_parent, Time())
        except Tf2Exception:
            return None
        q = tf.transform.rotation
        return (q.w, q.x, q.y, q.z)

    def _try_capture(self, sample):
        q_parent = self._lookup_parent_wxyz()
        if q_parent is None:
            return False, f'查不到 TF {self._align_ref}→{self._tf_parent}'
        framed = self._pipe.framed(sample)
        if not framed.q_valid:
            return False, 'IMU 姿态无效'
        return self._pipe.capture(q_parent, framed.q_wxyz)

    def _on_align_srv(self, _req, resp):
        del _req
        if self._last_sample is None:
            self._pending_align = True
            resp.success = False
            resp.message = '尚无样本；等下一帧再采'
            return resp
        ok, msg = self._try_capture(self._last_sample)
        resp.success = ok
        resp.message = msg
        self._pending_align = not ok
        return resp

    def _maybe_align(self, sample):
        if not self._align_to_parent:
            return
        need = self._pending_align or (
            self._align_on_start and not self._pipe.aligned)
        if not need:
            return
        ok, msg = self._try_capture(sample)
        if ok:
            self._pending_align = False
            self.get_logger().info(f'已对齐到 parent {msg}')
        else:
            self.get_logger().warn(
                f'对齐未采到: {msg}', throttle_duration_sec=2.0)

    def _on_timer(self):
        self._ensure_serial()
        if self._ser is None:
            return
        try:
            chunk = self._ser.read(self._ser.in_waiting or 1)
        except OSError as exc:
            self.get_logger().error(f'串口读失败: {exc}')
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None
            return
        if not chunk:
            return
        self._buf.extend(chunk)
        for sample in feed(self._buf):
            self._publish_sample(sample)

    def _publish_sample(self, sample):
        self._last_sample = sample
        self._maybe_align(sample)
        stamp = self.get_clock().now().to_msg()
        raw = self._pipe.raw(sample)
        corr = self._pipe.corrected(sample)
        self._pub_raw.publish(self._to_imu(raw, stamp))
        self._pub_data.publish(self._to_imu(corr, stamp))
        self._freq.tick()
        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = self._frame_id
        mag.magnetic_field.x = corr.mag[0] * GAUSS_TO_TESLA
        mag.magnetic_field.y = corr.mag[1] * GAUSS_TO_TESLA
        mag.magnetic_field.z = corr.mag[2] * GAUSS_TO_TESLA
        mag.magnetic_field_covariance = covariance_diag(self._mag_var)
        self._pub_mag.publish(mag)
        temp = Temperature()
        temp.header.stamp = stamp
        temp.header.frame_id = self._frame_id
        temp.temperature = sample.temp_c
        self._pub_temp.publish(temp)

    def _to_imu(self, frame, stamp) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.linear_acceleration.x, msg.linear_acceleration.y, \
            msg.linear_acceleration.z = frame.acc
        msg.angular_velocity.x, msg.angular_velocity.y, \
            msg.angular_velocity.z = frame.gyro
        w, x, y, z = frame.q_wxyz
        msg.orientation.x, msg.orientation.y, msg.orientation.z, \
            msg.orientation.w = x, y, z, w
        msg.linear_acceleration_covariance = covariance_diag(self._acc_var)
        if self._gyro_available:
            msg.angular_velocity_covariance = covariance_diag(self._gyro_var)
        else:
            msg.angular_velocity_covariance = covariance_diag(-1.0)
        if frame.q_valid:
            msg.orientation_covariance = covariance_diag(self._ori_var)
        else:
            msg.orientation_covariance = covariance_diag(-1.0)
        return msg


def main(args=None):
    """节点入口."""
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
