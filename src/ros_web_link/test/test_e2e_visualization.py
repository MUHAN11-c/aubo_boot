"""E2E 可视化消息独立测试 — 常用可视化内容经网关全链路 + rclpy 交叉解码.

被测链路与 test_e2e_comm_matrix 相同（pytest → 网关 /ws/foxglove → 桥 → ROS 图），
被测端为 rwl_viz_fixture（/rwl_viz/*，15 类常用可视化消息 @1Hz TRANSIENT_LOCAL）。

覆盖 RViz/Foxglove 生态常用可视化消息的 CDR 形态难点：
  blob（Image/PointCloud2/CompressedImage）、定长数组 float64[36]（Odometry 协方差）、
  变长并行数组（JointState）、嵌套数组（Path/MarkerArray/DiagnosticArray）、
  特殊浮点（LaserScan inf）。
"""
from __future__ import annotations

import asyncio
import math

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped, WrenchStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, JointState,  # noqa: E402
                             LaserScan, PointCloud2, Range)
from tf2_msgs.msg import TFMessage  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from .e2e_harness import Connected, Stack, loopback_client  # noqa: E402

_TIMEOUT = 10.0


@pytest.fixture(scope='module')
def stack():
    st = Stack(fixtures=('ros_web_link.viz_fixture',))
    try:
        st.start()
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


def _run(coro):
    return asyncio.run(coro)


# ── blob / 二进制 ───────────────────────────────────────────────────────────

def test_image_blob(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/image', Image, timeout=_TIMEOUT)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.width == 64 and m.height == 48
                assert m.encoding == 'mono8' and m.step == 64
                assert len(m.data) == 64 * 48
                # 棋盘内容抽验：相邻 8px 块反色、值域二值
                assert m.data[1] != m.data[9]
                assert set(m.data[1:17]) <= {0, 255}
            finally:
                await sub.cancel()
    _run(run())


def test_compressed_image_bytes(stack):
    async def run():
        from ros_web_link.viz_fixture import _JPEG_FRAMES
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/image_compressed', CompressedImage)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.format == 'jpeg'
                d = bytes(m.data)
                # 真实可解码棋盘 JPEG（两相位交替；web 端 createImageBitmap 实解同源字节）
                assert d[:3] == b'\xff\xd8\xff' and d[-2:] == b'\xff\xd9'
                assert d in _JPEG_FRAMES, '字节与 fixture 内嵌帧不一致'
            finally:
                await sub.cancel()
    _run(run())


def test_pointcloud2_xyz(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/points', PointCloud2)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.width == 128 and m.height == 1 and m.point_step == 12
                names = [f.name for f in m.fields]
                assert names == ['x', 'y', 'z']
                assert len(m.data) == 128 * 12
                # 抽验首点在半径 2 的圆上
                x = struct_unpad(m.data, 0)
                y = struct_unpad(m.data, 4)
                assert abs(math.hypot(x, y) - 2.0) < 1e-4
            finally:
                await sub.cancel()
    _run(run())


def struct_unpad(data, off: int) -> float:
    import struct as _s
    # data 可能是 bytes/序列化数组；CDR blob 内已无 padding（点内 4 字节对齐）
    return _s.unpack_from('<f', bytes(data), off)[0]


# ── 定长数组 / 协方差 ───────────────────────────────────────────────────────

def test_odometry_covariance(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/odom', Odometry)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.child_frame_id == 'rwl_viz_base'
                # 圆周轨迹（半径 2m）：XY 到原点距离恒为 2
                x, y = m.pose.pose.position.x, m.pose.pose.position.y
                assert math.hypot(x, y) == pytest.approx(2.0, abs=1e-6)
                assert len(m.pose.covariance) == 36 and m.pose.covariance[0] == pytest.approx(0.01)
                assert len(m.twist.covariance) == 36 and m.twist.covariance[35] == pytest.approx(0.02)
                assert m.twist.twist.linear.x == pytest.approx(0.5)
            finally:
                await sub.cancel()
    _run(run())


def test_camera_info_matrices(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/camera_info', CameraInfo)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.distortion_model == 'plumb_bob'
                assert m.k[0] == pytest.approx(600.0) and m.k[4] == pytest.approx(600.0)
                assert len(m.p) == 12 and m.p[11] == 0.0
                assert m.d[0] == pytest.approx(0.1)
            finally:
                await sub.cancel()
    _run(run())


# ── 变长数组 / 特殊浮点 ─────────────────────────────────────────────────────

def test_laserscan_inf(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/scan', LaserScan)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert len(m.ranges) == 181 and len(m.intensities) == 181
                assert m.range_min == pytest.approx(0.2)
                # 每 17 个一个 inf（无回波语义），其余为有限正距离
                assert math.isinf(m.ranges[0])
                finites = [r for r in m.ranges if not math.isinf(r)]
                assert len(finites) == 181 - 11  # 0,17,...,170 → 11 个 inf
                assert min(finites) > 0.5
            finally:
                await sub.cancel()
    _run(run())


def test_joint_state_parallel_arrays(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/joint_states', JointState)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.name == [f'joint_{i}' for i in range(6)]
                assert len(m.position) == len(m.velocity) == len(m.effort) == 6
                assert m.effort[5] == pytest.approx(15.0)
                assert -0.5 <= min(m.position) and max(m.position) <= 0.5
            finally:
                await sub.cancel()
    _run(run())


def test_range_scalar(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/range', Range)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.radiation_type == Range.ULTRASOUND
                assert 1.5 <= m.range <= 2.5
                assert m.header.frame_id == 'rwl_viz'
            finally:
                await sub.cancel()
    _run(run())


# ── 嵌套数组结构 ───────────────────────────────────────────────────────────

def test_path_poses(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/path', Path)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert len(m.poses) == 5
                assert [p.pose.position.x for p in m.poses] == pytest.approx([0.0, 0.5, 1.0, 1.5, 2.0])
                assert m.poses[0].header.frame_id == 'rwl_viz'
            finally:
                await sub.cancel()
    _run(run())


def test_pose_array_and_stamped(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/pose_array', PoseArray)
            ps = await c.subscribe('/rwl_viz/pose', PoseStamped)
            try:
                a = await sub.next(timeout=_TIMEOUT)
                p = await ps.next(timeout=_TIMEOUT)
                assert len(a.poses) == 4
                assert a.poses[0].orientation.w == pytest.approx(1.0)
                assert p.pose.position.x == pytest.approx(0.3)
                assert p.pose.position.y == pytest.approx(0.4)
                assert 0.5 <= p.pose.position.z <= 0.57
                assert p.header.frame_id == 'rwl_viz'
            finally:
                await sub.cancel()
                await ps.cancel()
    _run(run())


def test_pose_with_covariance(stack):
    """PoseWithCovarianceStamped：协方差椭圆显示链路的传输真值."""
    async def run():
        from geometry_msgs.msg import PoseWithCovarianceStamped
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/pose_cov', PoseWithCovarianceStamped)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.header.frame_id == 'rwl_viz'
                assert m.pose.pose.position.x == pytest.approx(-0.6)
                cov = list(m.pose.covariance)
                assert len(cov) == 36
                assert cov[0] == pytest.approx(0.04)
                assert cov[7] == pytest.approx(0.01)
                assert abs(cov[1]) <= 0.015 + 1e-9      # 主轴摆动项
                # 正定性：主轴方差 (a/2)² > (b/2)² > 0
                trace, det = cov[0] + cov[7], cov[0] * cov[7] - cov[1] ** 2
                l1 = trace / 2 + math.sqrt(trace * trace / 4 - det)
                l2 = trace / 2 - math.sqrt(trace * trace / 4 - det)
                assert l1 > l2 > 0
            finally:
                await sub.cancel()
    _run(run())


def test_marker_sphere_list(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/marker', Marker)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert m.type == Marker.SPHERE_LIST and m.action == Marker.ADD
                assert len(m.points) == 6 and len(m.colors) == 6
                assert m.points[0].x == pytest.approx(0.0)
                assert m.points[5].x == pytest.approx(1.0)
                assert m.colors[0].a == pytest.approx(0.8)
            finally:
                await sub.cancel()
    _run(run())


def test_marker_array_text(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/markers', MarkerArray)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert len(m.markers) == 2
                cube, text = m.markers
                assert cube.type == Marker.CUBE
                assert text.type == Marker.TEXT_VIEW_FACING
                assert text.text.startswith('rwl_viz #')
            finally:
                await sub.cancel()
    _run(run())


def test_diagnostics_nested(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_viz/diagnostics', DiagnosticArray)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                # rosidl 将 int8 标量映射为 bytes（b'\x00' = OK）
                assert m.status[0].level in (0, b'\x00')
                assert m.status[0].name == 'rwl_viz/fixture'
                kv = m.status[0].values[0]
                assert kv.key == 'seq' and kv.value.isdigit()
            finally:
                await sub.cancel()
    _run(run())


def test_wrench_and_tf(stack):
    async def run():
        async with Connected(stack) as c:
            w = await c.subscribe('/rwl_viz/wrench', WrenchStamped)
            t = await c.subscribe('/tf', TFMessage)
            try:
                wm = await w.next(timeout=_TIMEOUT)
                assert wm.wrench.force.x == pytest.approx(1.5)
                assert wm.wrench.torque.z > 0.25
                tf = await t.next(timeout=_TIMEOUT)
                frames = {tr.child_frame_id for tr in tf.transforms}
                # 本栈只起 viz fixture；其动态 TF 帧应可见
                assert 'rwl_viz_marker' in frames, frames
            finally:
                await w.cancel()
                await t.cancel()
    _run(run())
