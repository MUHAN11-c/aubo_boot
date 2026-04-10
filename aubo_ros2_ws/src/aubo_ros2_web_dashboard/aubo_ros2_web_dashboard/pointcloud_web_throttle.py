#!/usr/bin/env python3
"""将完整 PointCloud2 按均匀步长下采样后发布，专供网页 / rosbridge。

订阅回调内只做校验 + ``bytes`` 快照并入队，**numpy 下采样与 publish 在单独工作线程**执行，避免阻塞 rclpy 单线程执行器（其它回调、同进程节点更不易卡顿）。

参数：
  - ``input_topic`` / ``output_topic``：输入输出话题
  - ``max_points``：单帧最多保留点数
  - ``min_publish_period_sec``：工作线程侧最小发布间隔（0 表示不额外限频）
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


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
		self.declare_parameter('max_points', 24000)
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

		self._pub = self.create_publisher(PointCloud2, ot, qos_profile_sensor_data)
		self.create_subscription(PointCloud2, it, self._cb, qos_profile_sensor_data)

		self._q: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=1)
		self._shutdown = threading.Event()
		self._worker_joined = False
		self._last_pub_mono = 0.0
		self._worker = threading.Thread(target=self._worker_loop, name='ivg_pc2_web_worker', daemon=True)
		self._worker.start()

		self.get_logger().info(
			f'IVG pointcloud web throttle: {it!r} -> {ot!r}, max_points={self._max_points}, '
			f'min_publish_period_sec={self._min_pub_period} (worker thread)'
		)

	def _enqueue(self, snap: dict[str, Any]) -> None:
		try:
			self._q.put_nowait(snap)
		except queue.Full:
			try:
				self._q.get_nowait()
			except queue.Empty:
				pass
			try:
				self._q.put_nowait(snap)
			except queue.Full:
				pass

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
		self._enqueue(_snap_from_msg(msg))

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
		while not self._shutdown.is_set():
			try:
				snap = self._q.get(timeout=0.2)
			except queue.Empty:
				continue
			if snap is None:
				break
			out = self._process_snap(snap)
			if out is None:
				continue
			if self._min_pub_period > 0.0:
				now = time.monotonic()
				if now - self._last_pub_mono < self._min_pub_period:
					continue
				self._last_pub_mono = now
			try:
				self._pub.publish(out)
			except Exception as ex:  # noqa: BLE001
				self.get_logger().error(f'publish failed: {ex!s}')

	def shutdown_worker(self) -> None:
		if self._worker_joined:
			return
		self._shutdown.set()
		try:
			self._q.put_nowait(None)
		except queue.Full:
			try:
				self._q.get_nowait()
			except queue.Empty:
				pass
			try:
				self._q.put_nowait(None)
			except queue.Full:
				pass
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
