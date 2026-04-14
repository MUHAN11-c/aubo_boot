#!/usr/bin/env python3
"""将完整 PointCloud2 按均匀步长下采样后发布，专供网页 / rosbridge。

订阅回调内只做校验 + ``bytes`` 快照，**numpy 下采样与 publish 在单独工作线程**执行，避免阻塞 rclpy 单线程执行器。

待处理帧策略：**只保留最新一帧**；工作线程取出任务前在锁内**合并突发**（丢弃中间帧），避免在相机快于 worker 时对多帧连续做重计算，从而在保持低延迟的同时提高 **points_web 的有效发布率**。

参数：
  - ``input_topic`` / ``output_topic``：输入输出话题
  - ``max_points``：单帧最多保留点数（均匀步长覆盖整幅）
  - ``min_publish_period_sec``：工作线程侧最小发布间隔（0 表示不额外限频）
"""
from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
	DurabilityPolicy,
	HistoryPolicy,
	QoSProfile,
	ReliabilityPolicy,
	qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2

# 输入仍用 sensor_data（易与相机 PointCloud2 发布端匹配）。
# 输出改用 RELIABLE：rosbridge / 多数 rclpy 默认订户为 RELIABLE，若此处用 BEST_EFFORT，
# DDS 与订户不兼容，浏览器侧会长期收不到点云（bridge 日志仍可能显示 Subscribed）。
_QOS_WEB_PC2 = QoSProfile(
	depth=10,
	reliability=ReliabilityPolicy.RELIABLE,
	durability=DurabilityPolicy.VOLATILE,
	history=HistoryPolicy.KEEP_LAST,
)


def _snap_from_msg(msg: PointCloud2) -> dict[str, Any]:
	"""回调返回后原 msg 可能失效，故拷贝一份轻量快照（大头是 data 的 memcpy）。"""
	return {
		'header': msg.header,
		'height': int(msg.height),
		'width': int(msg.width),
		'fields': list(msg.fields),
		'is_bigendian': bool(msg.is_bigendian),
		'point_step': int(msg.point_step),
		'row_step': int(msg.row_step),
		'is_dense': bool(msg.is_dense),
		'data': bytes(memoryview(msg.data)),
	}


def _build_pc2(
	snap: dict[str, Any],
	height: int,
	width: int,
	data: bytes,
) -> PointCloud2:
	out = PointCloud2()
	out.header = snap['header']
	out.height = height
	out.width = width
	out.fields = snap['fields']
	out.is_bigendian = snap['is_bigendian']
	out.point_step = snap['point_step']
	if height == snap['height'] and width == snap['width']:
		out.row_step = snap['row_step']
	else:
		out.row_step = width * snap['point_step']
	out.is_dense = snap['is_dense']
	out.data = data
	return out


class PointCloudWebThrottle(Node):
	def __init__(self) -> None:
		super().__init__('ivg_pointcloud_web_throttle')
		self.declare_parameter('input_topic', '/camera/depth_registered/points')
		self.declare_parameter('output_topic', '/camera/depth_registered/points_web')
		self.declare_parameter('max_points', 32000)
		self.declare_parameter('min_publish_period_sec', 0.0)

		it = self.get_parameter('input_topic').get_parameter_value().string_value
		ot = self.get_parameter('output_topic').get_parameter_value().string_value
		mp = int(self.get_parameter('max_points').get_parameter_value().integer_value)
		self._max_points = max(1000, mp)
		self._min_pub_period = float(
			self.get_parameter('min_publish_period_sec').get_parameter_value().double_value
		)
		if self._min_pub_period < 0.0:
			self._min_pub_period = 0.0

		self._pub = self.create_publisher(PointCloud2, ot, _QOS_WEB_PC2)
		self.create_subscription(PointCloud2, it, self._cb, qos_profile_sensor_data)

		self._snap_lock = threading.Lock()
		self._snap_cv = threading.Condition(self._snap_lock)
		self._pending_snap: dict[str, Any] | None = None
		self._worker_stop = False
		self._worker_joined = False
		self._last_pub_mono = 0.0
		self._perf_frames = 0
		self._perf_total_process_ms = 0.0
		self._perf_total_end_to_end_ms = 0.0
		self._perf_last_log_mono = time.monotonic()
		self._worker = threading.Thread(target=self._worker_loop, name='ivg_pc2_web_worker', daemon=True)
		self._worker.start()

		self.get_logger().info(
			f'IVG pointcloud web throttle: {it!r} -> {ot!r}, max_points={self._max_points}, '
			f'min_publish_period_sec={self._min_pub_period} (worker thread, latest-frame coalesce)'
		)

	def _submit_snap(self, snap: dict[str, Any]) -> None:
		with self._snap_cv:
			if self._worker_stop:
				return
			snap['_submitted_mono'] = time.monotonic()
			self._pending_snap = snap
			self._snap_cv.notify()

	def _wait_next_snap(self) -> dict[str, Any] | None:
		"""阻塞直到有新快照；返回前在锁内合并突发，只保留最新一帧。"""
		with self._snap_cv:
			while self._pending_snap is None and not self._worker_stop:
				self._snap_cv.wait(timeout=0.2)
			if self._worker_stop and self._pending_snap is None:
				return None
			snap = self._pending_snap
			self._pending_snap = None
			while self._pending_snap is not None:
				snap = self._pending_snap
				self._pending_snap = None
			return snap

	def _cb(self, msg: PointCloud2) -> None:
		total = int(msg.width) * int(msg.height)
		ps = int(msg.point_step)
		if total <= 0 or ps <= 0:
			return
		expected = total * ps
		raw = memoryview(msg.data)
		if len(raw) < expected:
			self.get_logger().warning(
				f'short point cloud: len(data)={len(raw)} expected>={expected} (w={msg.width} h={msg.height} step={ps})'
			)
			return
		self._submit_snap(_snap_from_msg(msg))

	def _process_snap(self, snap: dict[str, Any]) -> PointCloud2 | None:
		total = snap['width'] * snap['height']
		ps = snap['point_step']
		expected = total * ps
		raw = memoryview(snap['data'])
		if len(raw) < expected:
			return None

		if total <= self._max_points:
			return _build_pc2(snap, snap['height'], snap['width'], snap['data'])

		stride = int(np.ceil(total / self._max_points))
		n_out = int(min(self._max_points, (total + stride - 1) // stride))
		idx = np.arange(0, min(n_out * stride, total), stride, dtype=np.int64)[:n_out]
		arr = np.frombuffer(raw, dtype=np.uint8, count=expected).reshape((total, ps))
		sel = arr[idx].ravel()
		return _build_pc2(snap, 1, n_out, sel.tobytes())

	def _worker_loop(self) -> None:
		while True:
			snap = self._wait_next_snap()
			if snap is None:
				if self._worker_stop:
					break
				continue
			t0 = time.perf_counter()
			out = self._process_snap(snap)
			process_ms = (time.perf_counter() - t0) * 1000.0
			if out is None:
				continue
			if self._min_pub_period > 0.0:
				now = time.monotonic()
				if now - self._last_pub_mono < self._min_pub_period:
					continue
				self._last_pub_mono = now
			try:
				self._pub.publish(out)
				self._perf_frames += 1
				self._perf_total_process_ms += process_ms
				submitted = float(snap.get('_submitted_mono', 0.0) or 0.0)
				if submitted > 0.0:
					self._perf_total_end_to_end_ms += (time.monotonic() - submitted) * 1000.0
				now_mono = time.monotonic()
				if now_mono - self._perf_last_log_mono >= 5.0 and self._perf_frames > 0:
					avg_process_ms = self._perf_total_process_ms / self._perf_frames
					avg_e2e_ms = self._perf_total_end_to_end_ms / self._perf_frames
					self.get_logger().info(
						f'points_web perf: frames={self._perf_frames}, avg_process_ms={avg_process_ms:.1f}, '
						f'avg_end_to_end_ms={avg_e2e_ms:.1f}, max_points={self._max_points}'
					)
					self._perf_frames = 0
					self._perf_total_process_ms = 0.0
					self._perf_total_end_to_end_ms = 0.0
					self._perf_last_log_mono = now_mono
			except Exception as ex:  # noqa: BLE001
				self.get_logger().error(f'publish failed: {ex!s}')

	def shutdown_worker(self) -> None:
		if self._worker_joined:
			return
		with self._snap_cv:
			self._worker_stop = True
			self._snap_cv.notify_all()
		self._worker.join(timeout=3.0)
		self._worker_joined = True
		if self._worker.is_alive():
			self.get_logger().warning('worker thread did not exit within 3s')


def main() -> None:
	rclpy.init()
	node = PointCloudWebThrottle()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.shutdown_worker()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
