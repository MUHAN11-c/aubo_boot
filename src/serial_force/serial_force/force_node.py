"""串口 5 点力传感器节点：逐帧打印 N/行程/AD + 话题，只收不发."""
from __future__ import annotations

import time
import uuid

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater)
from peach_interfaces.msg import ForceSample as ForceSampleMsg
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
import serial
from serial_force.force_protocol import (
    feed_force, FORCE_CHANNELS, G_TO_N)
from serial_force.force_status import (
    classify, NAMES, OPEN, SATURATED)
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

SERIAL_RETRY_S = 2.0
FORCE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SerialForceNode(Node):
    """打开固定串口，解析 0xA5 力帧，逐帧打印并发布."""

    def __init__(self):
        """声明参数、话题、诊断与读串口定时器."""
        super().__init__('serial_force')
        self._declare_params()
        self._print_data = bool(self.get_parameter('print_data').value)
        self._print_decimate = max(
            1, int(self.get_parameter('print_decimate').value))
        self._open_ad = float(
            self.get_parameter('open_ad_threshold').value)
        self._full_mm = float(
            self.get_parameter('full_travel_mm').value)
        self._labels = [str(v)
                        for v in self.get_parameter('channel_labels').value]
        if len(self._labels) != len(FORCE_CHANNELS):
            raise ValueError(
                f'channel_labels 须 {len(FORCE_CHANNELS)} 项（通道 '
                f'{FORCE_CHANNELS}），当前 {len(self._labels)} 项')
        self._pub_force = self.create_publisher(
            Float64MultiArray, 'force/points', FORCE_QOS)
        self._pub_travel = self.create_publisher(
            Float64MultiArray, 'force/travel', FORCE_QOS)
        self._pub_ad = self.create_publisher(
            Float64MultiArray, 'force/ad', FORCE_QOS)
        self._pub_status = self.create_publisher(
            UInt8MultiArray, 'force/status', FORCE_QOS)
        self._pub_sample = self.create_publisher(
            ForceSampleMsg, 'force/sample', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE, depth=100))
        self._source_uuid = str(uuid.uuid4())
        self._buf = bytearray()
        self._ser = None
        self._last_serial_try = 0.0
        self._logged_serial_fail = False
        self._frame_count = 0
        self._open_frames = 0
        self._sat_frames = 0
        self._last_open_warn = 0.0
        self._setup_diagnostics()
        self.create_timer(
            float(self.get_parameter('read_period_s').value), self._on_timer)
        self.get_logger().info(
            f'5 点力 IMS-C04A（单点 50–400 g，合计 2 kg）通道 {FORCE_CHANNELS} '
            f'点位 {self._labels}；打印 N/行程/AD/状态，话题 N / mm / AD / status')

    def _p(self, name, default, description):
        """声明带中文 description 的参数."""
        self.declare_parameter(
            name, default, ParameterDescriptor(description=description))

    def _declare_params(self):
        self._p('port', '/dev/force', '首选串口')
        self._p(
            'port_fallbacks', ['/dev/ttyUSB0', '/dev/ttyACM0'],
            'port 打不开时依次试')
        self._p('baudrate', 115200, '波特率')
        self._p('read_period_s', 0.005, '读串口定时器周期')
        self._p('open_ad_threshold', 3000.0,
                'AD 大于等于此值判断线/悬空（有效 370 g 约 1700–2200）')
        self._p('full_travel_mm', 5.95, 'mm 大于等于此值判满量程饱和（6.0）')
        self._p('expected_rate_hz', 100.0, '/diagnostics 帧率期望')
        self._p('print_data', True, '逐帧打印 5 通道力值/行程（N / mm）')
        self._p('print_decimate', 1, '每 N 帧打印一条；1=全部（≈100Hz）')
        self._p(
            'channel_labels',
            # 按果实侧命名；臂侧镜像（1 ↔ 7、2 ↔ 3）。
            ['右上', '右下', '左下', '上', '左上'],
            '通道 1/2/3/5/7 对应咬合末端圆盘点位（与 config 一致，'
            '现场按压校对后改这里）')

    def _setup_diagnostics(self):
        self._updater = Updater(self)
        self._updater.setHardwareID('serial_force')
        self._updater.add('serial', self._diag_serial)
        hz = float(self.get_parameter('expected_rate_hz').value)
        self._freq = HeaderlessTopicDiagnostic(
            '/force/points', self._updater,
            FrequencyStatusParam({'min': hz * 0.6, 'max': hz * 1.4}, 0.1, 10))

    def _diag_serial(self, stat):
        if self._ser is not None and self._ser.is_open:
            stat.summary(DiagnosticStatus.OK, f'open {self._ser.port}')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'serial closed')
        return stat

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
                # exclusive：防两个节点静默共口抢字节。
                self._ser = serial.Serial(port, baud, timeout=0,
                                          exclusive=True)
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
        for sample in feed_force(self._buf):
            self._publish_sample(sample)

    def _publish_sample(self, sample):
        self._frame_count += 1
        statuses = [classify(a, t, self._open_ad, self._full_mm)
                    for a, t in zip(sample.ad, sample.travel_mm)]
        frame = ForceSampleMsg()
        stamp = self.get_clock().now().to_msg()
        frame.header.stamp.sec = stamp.sec
        frame.header.stamp.nanosec = stamp.nanosec
        frame.header.frame_id = 'tcp'
        frame.source_uuid = self._source_uuid
        frame.sequence = self._frame_count
        frame.channel_ids = list(FORCE_CHANNELS)
        frame.force_n = list(sample.force_n)
        frame.travel_mm = list(sample.travel_mm)
        frame.ad = list(sample.ad)
        frame.status = statuses
        self._pub_sample.publish(frame)
        msg = Float64MultiArray()
        msg.data = list(sample.force_n)
        self._pub_force.publish(msg)
        tmsg = Float64MultiArray()
        tmsg.data = list(sample.travel_mm)
        self._pub_travel.publish(tmsg)
        amsg = Float64MultiArray()
        amsg.data = [float(a) for a in sample.ad]
        self._pub_ad.publish(amsg)
        smsg = UInt8MultiArray()
        smsg.data = statuses
        self._pub_status.publish(smsg)
        self._freq.tick()
        if OPEN in statuses:
            self._open_frames += 1
            now = time.monotonic()
            if now - self._last_open_warn > 1.0:
                self._last_open_warn = now
                bad = [str(ch) for ch, st in
                       zip(FORCE_CHANNELS, statuses) if st == OPEN]
                self.get_logger().warning(
                    f'力帧断线/悬空通道 {",".join(bad)}；'
                    f'累计 {self._open_frames} 帧')
        if SATURATED in statuses:
            self._sat_frames += 1
        if not self._print_data:
            return
        if self._frame_count % self._print_decimate != 0:
            return
        parts = []
        for ch, label, g, t, a, st in zip(
                FORCE_CHANNELS, self._labels, sample.grams,
                sample.travel_mm, sample.ad, statuses):
            parts.append(
                f'F{ch}[{label}]={g * G_TO_N:+.2f}N/{t:.1f}mm/AD{a}/'
                f'{NAMES[st]}')
        print(f'#{self._frame_count} ' + ' '.join(parts), flush=True)


def main(args=None):
    """节点入口."""
    rclpy.init(args=args)
    node = SerialForceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT 可能在收尾中再次抵达主线程，吞掉以免 traceback 逃逸。
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
