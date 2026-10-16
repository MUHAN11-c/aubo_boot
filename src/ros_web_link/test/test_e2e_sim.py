"""E2E 官方示例真实数据测试 — GitHub 官方栈为数据面，web 传输为被测面.

栈 = ``sim_demo.launch.py``（gui:=false，整官方组合）：
  turtlebot3 官方仿真（ROBOTIS-GIT/turtlebot3_simulations，已克隆 src/）
    /robot_description /joint_states /odom /tf /imu /scan /camera/image_raw
    /camera/camera_info
  官方 nav2（apt nav2_bringup；navigation2 克隆于 external/）
    /map /amcl_pose /particle_cloud /plan /diagnostics /initialpose /goal_pose
  ros2/examples 官方示例（克隆 src/ros2_examples）
    /test_cloud（官方点云示例） /topic（官方 talker 示例）
  官方 image_transport republish
    /camera/image_raw/compressed

全部经网关 /ws/foxglove（与浏览器同路）。本地自写合成流不参与——
viz_fixture 已删除（2026-10-15 用户裁定），本文件替代 test_e2e_visualization.py。
"""
from __future__ import annotations

import asyncio
import math
import time
import uuid as uuid_mod

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from builtin_interfaces.msg import Time  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PoseArray  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402

from .e2e_harness import Connected, LaunchStack  # noqa: E402

_TIMEOUT = 20.0
_ROBOT_READY = False


@pytest.fixture(scope='module')
def stack():
    st = LaunchStack('sim_demo.launch.py', args={'gui': 'false'})
    try:
        st.start(readiness_topics=(
            '/scan', '/odom', '/joint_states', '/camera/image_raw',
            '/test_cloud', '/topic', '/map'))
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


def _run(coro):
    return asyncio.run(coro)


def _sub(c, topic, msg_type, timeout=_TIMEOUT):
    return c.subscribe(topic, msg_type, timeout=timeout)


async def _first(c, topic, msg_type, timeout=_TIMEOUT):
    """订阅并取首帧（用后即订即取；覆盖多数单帧断言）."""
    sub = await _sub(c, topic, msg_type)
    try:
        return await sub.next(timeout=timeout)
    finally:
        await sub.cancel()


# ── turtlebot3 官方仿真数据面 ──────────────────────────────────────────────


def test_tb3_robot_description(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/robot_description', String)
            assert 'turtlebot3_waffle_pi' in m.data or 'base_footprint' in m.data
    _run(run())


def test_tb3_joint_states(stack):
    """官方 gz 关节桥：轮关节流（waffle_pi: wheel_left/right_joint）。"""
    async def run():
        async with Connected(stack) as c:
            sub = await _sub(c, '/joint_states', JointState)
            try:
                names = set()
                for _ in range(5):
                    m = await sub.next(timeout=_TIMEOUT)
                    names |= set(m.name)
                    if {'wheel_left_joint', 'wheel_right_joint'} <= names:
                        return
                pytest.fail(f'轮关节缺失: {sorted(names)}')
            finally:
                await sub.cancel()
    _run(run())


def test_tb3_scan(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/scan', LaserScan)
            assert m.header.frame_id == 'base_scan'
            assert len(m.ranges) > 100
            assert m.angle_max > m.angle_min
    _run(run())


def test_tb3_odom(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/odom', __import__(
                'nav_msgs.msg', fromlist=['Odometry']).Odometry)
            assert m.header.frame_id == 'odom'
            assert m.child_frame_id in ('base_footprint', 'base_link')
            # gz DiffDrive 协方差字段为零是上游常态，不作为断言
    _run(run())


def test_tb3_imu(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/imu', Imu)
            assert m.header.frame_id, 'IMU 帧名缺失'
    _run(run())


def test_tb3_tf(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await _sub(c, '/tf', TFMessage)
            try:
                children = set()
                parents = set()
                for _ in range(6):
                    m = await sub.next(timeout=_TIMEOUT)
                    children |= {t.child_frame_id for t in m.transforms}
                    parents |= {t.header.frame_id for t in m.transforms}
                    base_ok = {'base_footprint', 'base_link'} & children
                    # gz 桥帧名可带前导斜杠（'/wheel_left_link'），归一后比对
                    if ('odom' in parents or 'odom' in {p.lstrip('/') for p in parents}) \
                            and base_ok:
                        return
                pytest.fail(
                    f'/tf 未见 odom 链: parents={sorted(parents)} '
                    f'children={sorted(children)}')
            finally:
                await sub.cancel()
    _run(run())


def test_tb3_camera_image(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/camera/image_raw', Image)
            assert m.width == 640 and m.height == 480
            assert m.encoding in ('rgb8', 'bgr8', 'mono8')
            assert len(m.data) == m.width * m.height * (
                3 if m.encoding in ('rgb8', 'bgr8') else 1)
            # 首帧可能落在 sim 第 1 秒内（stamp.sec==0 属上游相机桥常态），
            # 只断言帧名与数据面
            assert m.header.frame_id == 'camera_rgb_frame'
    _run(run())


def test_tb3_camera_info(stack):
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/camera/camera_info', CameraInfo)
            assert m.k[0] > 0 and m.k[8] > 0, '相机内参 K 全零'
            assert m.width == 640
    _run(run())


def test_compressed_image(stack):
    """官方 image_transport republish：raw → compressedJPEG。"""
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/camera/image_raw/compressed', CompressedImage)
            assert 'jpeg' in m.format, m.format
            assert len(m.data) > 1000, '压缩图数据过小'
    _run(run())


# ── ros2/examples 官方示例节点 ─────────────────────────────────────────────


def test_official_pointcloud(stack):
    """ros2/examples 官方点云示例：/test_cloud。"""
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/test_cloud', PointCloud2)
            assert m.width * m.height > 0
            fields = {f.name for f in m.fields}
            assert {'x', 'y', 'z'} <= fields
    _run(run())


def test_official_talker(stack):
    """ros2/examples 官方 talker 示例：/topic。"""
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/topic', String)
            assert m.data.startswith('Hello World') or m.data, '官方 talker 无输出'
    _run(run())


# ── 官方 nav2（web ↔ 官方闭环） ────────────────────────────────────────────


async def _publish_initialpose(c) -> None:
    """web 发布 /initialpose（TB3 出生位，官方 AMCL 激活口径），双发抗丢首帧."""
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = Time(sec=int(time.time()), nanosec=0)
    msg.pose.pose.position.x = -2.0
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    await c.publish('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                    msg)
    await asyncio.sleep(0.15)
    await c.publish('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                    msg)


def test_web_initialpose_activates_amcl(stack):
    """web 发布 /initialpose → 官方 AMCL 出 /amcl_pose（web↔官方定位闭环）."""
    global _ROBOT_READY

    async def run():
        async with Connected(stack) as c:
            await _publish_initialpose(c)
            sub = await _sub(c, '/amcl_pose', PoseWithCovarianceStamped)
            try:
                m = await sub.next(timeout=30.0)
                assert m.header.frame_id == 'map'
            finally:
                await sub.cancel()

    _run(run())
    _ROBOT_READY = True


def test_particle_cloud(stack):
    """AMCL 粒子云（PoseArray）——initialpose 激活后粒子应流出."""
    if not _ROBOT_READY:
        pytest.skip('依赖 web initialpose 先激活 AMCL')

    async def run():
        async with Connected(stack) as c:
            sub = await _sub(c, '/particle_cloud', PoseArray)
            try:
                # 粒子采样随激光更新，偶发慢一拍：无粒子时重发 initialpose 重试
                for _ in range(3):
                    try:
                        m = await sub.next(timeout=8.0)
                        assert len(m.poses) > 10, '粒子云过小'
                        return
                    except asyncio.TimeoutError:
                        await _publish_initialpose(c)
                pytest.fail('30s 内 /particle_cloud 无消息')
            finally:
                await sub.cancel()
    _run(run())


def test_web_compute_path_plan(stack):
    """web 调官方 NavFn 规划器 action → /plan（web↔官方规划闭环）。"""
    if not _ROBOT_READY:
        pytest.skip('依赖 web initialpose 先激活 AMCL（costmap TF）')

    async def run():
        async with Connected(stack) as c:
            plan_sub = await _sub(c, '/plan', __import__(
                'nav_msgs.msg', fromlist=['Path']).Path)
            # AMCL 需数秒收敛（首条 amcl_pose 是 initialpose 回执，滤波未稳）
            await _publish_initialpose(c)
            await asyncio.sleep(6.0)

            S = ComputePathToPose.Impl.SendGoalService
            G = ComputePathToPose.Impl.GetResultService
            res = None
            for _ in range(3):  # 收敛/首帧 TF 偶发抖动：整条 goal 退避重试
                gid = uuid_mod.uuid4().bytes
                greq = S.Request()
                greq.goal_id = UUID()
                greq.goal_id.uuid = list(gid)
                greq.goal.goal.header.frame_id = 'map'
                greq.goal.goal.pose.position.x = 1.0
                greq.goal.goal.pose.position.y = 1.0
                greq.goal.goal.pose.orientation.w = 1.0
                accepted = None
                for _ in range(2):
                    try:
                        accepted = await c.call_service(
                            '/compute_path_to_pose/_action/send_goal',
                            greq, S.Response, timeout=30.0)
                        if accepted.accepted:
                            break
                    except Exception:
                        pass
                    await asyncio.sleep(3.0)
                assert accepted is not None and accepted.accepted, '规划 goal 被拒'

                rreq = G.Request()
                rreq.goal_id = UUID()
                rreq.goal_id.uuid = list(gid)
                res = await c.call_service(
                    '/compute_path_to_pose/_action/get_result',
                    rreq, G.Response, timeout=30.0)
                if res.status == 4:
                    break
                await asyncio.sleep(4.0)
            assert res is not None and res.status == 4, \
                f'规划终态 {res.status if res else None}'
            assert len(res.result.path.poses) >= 2, '规划路径过短'

            saw_plan = False
            try:
                deadline = asyncio.get_event_loop().time() + 10.0
                while asyncio.get_event_loop().time() < deadline:
                    try:
                        m = await plan_sub.next(timeout=3.0)
                    except asyncio.TimeoutError:
                        continue
                    if len(m.poses) >= 2:
                        saw_plan = True
                        break
            finally:
                await plan_sub.cancel()
            assert saw_plan, '官方规划器未发布 /plan'
    _run(run())


def test_nav2_map(stack):
    """官方 nav2 map_server：ROBOTIS 配套 turtlebot3_world 图 384²@0.05m.

    （2026-10-16 修正：tb3_sandbox 与仿真世界非同一环境，换官方配套图后
    激光点与地图墙可对齐；尺寸/分辨率/origin 约定与 tb3_sandbox 相同）
    """
    async def run():
        async with Connected(stack) as c:
            m = await _first(c, '/map', OccupancyGrid)
            assert m.info.width == 384 and m.info.height == 384
            assert m.info.resolution == pytest.approx(0.05)
            assert m.header.frame_id == 'map'
    _run(run())


def test_nav2_diagnostics(stack):
    """官方 nav2 诊断流（lifecycle_manager Nav2 Health）经网关可订阅."""
    async def run():
        async with Connected(stack) as c:
            sub = await _sub(c, '/diagnostics', DiagnosticArray)
            try:
                names = set()
                deadline = time.monotonic() + 40.0
                while time.monotonic() < deadline:
                    try:
                        m = await sub.next(timeout=5.0)
                    except asyncio.TimeoutError:
                        continue
                    names |= {s.name for s in m.status}
                    if any(k in n for n in names
                           for k in ('amcl', 'map_server', 'planner',
                                     'lifecycle_manager', 'Nav2')):
                        return
                pytest.fail(f'nav2 诊断缺失: {sorted(names)[:10]}')
            finally:
                await sub.cancel()
    _run(run())


# ── 真实大数据流速率与新鲜度（web↔官方仿真链路"无延迟"的直接证据） ────────


def _measure_real_stream(stack, topic: str, duration_s: float = 6.0):
    """裸 WS 持续消费真实流 duration_s，返回 (fps, 各帧年龄列表).

    复用 bigdata 基准的裸 websockets 计数/测龄助手（max_queue=2，消费速率
    掌握在测试手里）；帧年龄 = 本地墙钟 − 桥接收时刻（logTime，桥不设
    use_sim_time，恒为墙钟）——无界积压会随时间线性增长，健康链路稳定在
    百毫秒级。
    """
    from .test_e2e_bigdata import (_next_data_frame, _raw_connect,
                                   _subscribe, _wait_channel)

    async def run():
        ws = await _raw_connect(stack.ws_url)
        channels: dict = {}
        try:
            ch = (await _wait_channel(ws, topic, channels))['id']
            await _subscribe(ws, ch, 1)
            frames = 0
            ages = []
            t_end = time.monotonic() + duration_s
            while time.monotonic() < t_end:
                _, log_ns, _size = await _next_data_frame(ws, channels, 5.0)
                frames += 1
                ages.append(time.time() - log_ns / 1e9)
        finally:
            await ws.close()
        return frames / duration_s, ages

    return asyncio.run(run())


def test_tb3_camera_stream_rate_and_freshness(stack):
    """真实 gz 相机流（640×480 rgb8 ≈0.92MB/帧，名义 30Hz）持续 6s 经网关.

    与 bigdata 合成基准互补：这是官方仿真真实产流（gz 传感器线程 → gz 桥 →
    DDS → foxglove 桥 → 网关 → WS 全链）。阈值取名义率 50%——gz 实时因子
    受机器负载影响；首帧含传感器慢启动，6s 窗口足以摊薄。
    """
    fps, ages = _measure_real_stream(stack, '/camera/image_raw')
    assert fps >= 15.0, f'相机流 {fps:.1f}fps < 15'
    assert sum(ages) / len(ages) < 0.3, f'mean age {sum(ages) / len(ages):.3f}s'
    assert max(ages) < 1.0, f'max age {max(ages):.3f}s'


def test_tb3_pointcloud_stream_rate_and_freshness(stack):
    """官方点云示例流（ros2/examples 100×100 点，名义 30Hz）持续 6s 经网关."""
    fps, ages = _measure_real_stream(stack, '/test_cloud')
    assert fps >= 18.0, f'点云流 {fps:.1f}fps < 18'
    assert sum(ages) / len(ages) < 0.3, f'mean age {sum(ages) / len(ages):.3f}s'
    assert max(ages) < 1.0, f'max age {max(ages):.3f}s'


def test_teleop_drives_tb3(stack):
    """web 发 /cmd_vel（TwistStamped）→ gz DiffDrive → /odom 位移（teleop 物理闭环）.

    teleop 前端同款节奏：50ms 节流 0.2 m/s 前进 ~2.4s 后急停，/odom 位移 > 0.05m。
    机器负载会拉低 gz 实时因子，阈值取保守值；出生位姿卡障碍问题已在
    sim_demo 显式传 x/y_pose 修复（2026-10-15）。
    """
    async def run():
        async with Connected(stack) as c:
            sub = await _sub(c, '/odom', __import__(
                'nav_msgs.msg', fromlist=['Odometry']).Odometry)

            async def pose_xy():
                m = await sub.next(timeout=_TIMEOUT)
                return (float(m.pose.pose.position.x), float(m.pose.pose.position.y))

            def stamped(vx):
                ts = TwistStamped()
                ts.header.frame_id = 'base_link'
                ts.twist.linear.x = vx
                return ts

            p0 = await pose_xy()
            fwd = stamped(0.2)
            stop = stamped(0.0)
            # 机器负载会拉低 gz 实时因子：4.8s 墙钟窗口 + 低阈值（仅验"真动过"）
            for _ in range(96):
                await c.publish('/cmd_vel', 'geometry_msgs/msg/TwistStamped', fwd)
                await asyncio.sleep(0.05)
            await c.publish('/cmd_vel', 'geometry_msgs/msg/TwistStamped', stop)
            await asyncio.sleep(0.6)
            p1 = p0
            for _ in range(4):   # 丢掉停前缓存帧，读停止后的位姿
                p1 = await pose_xy()
            await sub.cancel()
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            assert dist > 0.05, f'TB3 未移动: dist={dist:.3f}'
    _run(run())
