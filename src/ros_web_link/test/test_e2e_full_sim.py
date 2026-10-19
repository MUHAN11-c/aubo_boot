"""E2E：完整官方仿真项目正常启动 + ros_web_link 独立监控附着.

测试面（2026-10-19 用户裁定：只用 ``src/examples`` 官方示例，本地 peach 不参与）：
1. ``ros2 launch ros_web_link sim_demo.launch.py transport:=false`` —— 正常启动
   完整的官方仿真项目（turtlebot3_simulations + turtlebot3_navigation2 +
   ros2/examples + 官方 image_transport），**不带**传输栈；
2. 另起 ``Stack(domain=<仿真域>)`` = 独立桥 + 网关（不拥有、不干预仿真）；
3. 全部数据经 ``/ws/foxglove`` 读取，验证 ros_web_link 作为独立监控包的
   HTTP 面、真实话题面与 robot-mesh 可视化链路。

运行（较慢，默认跳过）：

    RWL_FULL_SIM=1 ./aubo_py3.12/bin/python -m pytest \\
        src/ros_web_link/test -k full_sim -v

CI/无 X 环境可设置 ``RWL_GZ_WRAPPER_DIR`` 指向一个给 ``gz sim`` 注入
``--headless-rendering`` 的包装器目录（本机测试脚本见 doc/testing.md）。
"""
from __future__ import annotations

import asyncio
import os
import re
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from nav_msgs.msg import OccupancyGrid  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from .e2e_harness import Connected, LaunchStack, Stack, loopback_client  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get('RWL_FULL_SIM') != '1',
    reason='完整官方仿真 E2E 较慢：RWL_FULL_SIM=1 才运行')

_TIMEOUT = 25.0


@pytest.fixture(scope='module')
def monitor():
    """正常启动官方完整仿真（无传输栈），再独立起监控栈（桥+网关）."""
    sim = LaunchStack(
        'sim_demo.launch.py',
        args={'gui': 'false', 'transport': 'false'},
    )
    mon = None
    try:
        sim.start(
            readiness_topics=(
                '/scan', '/odom', '/joint_states', '/camera/image_raw',
                '/test_cloud', '/topic', '/map'),
            wait_gateway=False)
        # 独立监控：与仿真同域，但不由仿真 launch 启动/管理
        mon = Stack(domain=sim.domain)
        mon.start()

        async def ready():
            async with Connected(mon) as c:
                try:
                    await c.wait_channel('/joint_states', timeout=30.0)
                    return True
                except asyncio.TimeoutError:
                    return False

        if not asyncio.run(ready()):
            pytest.skip(f'官方仿真域 {sim.domain} 未发现 /joint_states')
    except Exception:
        if mon is not None:
            mon.stop()
        sim.stop()
        raise
    yield SimpleNamespace(mon=mon, sim=sim, domain=sim.domain)
    mon.stop()
    sim.stop()


def _run(coro):
    return asyncio.run(coro)


async def _first(c, topic, msg_type, timeout=_TIMEOUT):
    sub = await c.subscribe(topic, msg_type, timeout=timeout)
    try:
        return await sub.next(timeout=timeout)
    finally:
        await sub.cancel()


# ── 独立入口 / 网关 HTTP 面 ────────────────────────────────────────────────


def test_full_sim_gateway_health_and_entry(monitor):
    with loopback_client(monitor.mon.gateway_port) as c:
        r = c.get('/health')
        assert r.status_code == 200
        body = r.json()
        assert body['status'] == 'ok', body
        assert body['foxglove_bridge'] == 'ok'
        # 独立入口：/ → /console.html
        r2 = c.get('/')
        assert r2.status_code in (307, 308)
        assert r2.headers['location'].endswith('/console.html')
        page = c.get('/console.html')
        assert page.status_code == 200
        assert '传输诊断台' in page.text
        # 白色二次元商业主题的品牌锚点 + 自动数据面说明
        assert 'LIVE STUDIO' in page.text
        assert '自动数据面' in page.text
        # 数据面自动适配逻辑随包发布
        js = c.get('/js/console/viz_core.js')
        assert js.status_code == 200 and 'adaptInstancesToDataPlane' in js.text


def test_full_sim_ws_stats_and_console_assets(monitor):
    with loopback_client(monitor.mon.gateway_port) as c:
        r = c.get('/api/ivg/ws/stats')
        assert r.status_code == 200
        assert 'totals' in r.json()
        for path in ('/js/console/app.js', '/js/console/viz_core.js',
                     '/js/transport/foxglove/ros_client.js'):
            assert c.get(path).status_code == 200, path


# ── 官方仿真真实数据（全部经网关 /ws/foxglove）─────────────────────────────


def test_full_sim_robot_description_and_joints(monitor):
    async def run():
        async with Connected(monitor.mon) as c:
            desc = await _first(c, '/robot_description', String)
            assert 'turtlebot3' in desc.data or 'base_footprint' in desc.data
            js = await _first(c, '/joint_states', JointState)
            names = set(js.name)
            assert {'wheel_left_joint', 'wheel_right_joint'} <= names, sorted(names)
            assert len(js.position) == len(js.name)
    _run(run())


def test_full_sim_tf_tree(monitor):
    async def run():
        async with Connected(monitor.mon) as c:
            sub = await c.subscribe('/tf', TFMessage, timeout=_TIMEOUT)
            try:
                seen = set()
                deadline = time.monotonic() + _TIMEOUT
                while time.monotonic() < deadline:
                    m = await sub.next(timeout=_TIMEOUT)
                    for t in m.transforms:
                        seen.add(t.header.frame_id.lstrip('/'))
                        seen.add(t.child_frame_id.lstrip('/'))
                    if {'odom', 'base_footprint'} <= seen:
                        return
                pytest.fail(f'/tf 未见 odom→base_footprint 链: {sorted(seen)}')
            finally:
                await sub.cancel()
    _run(run())


def test_full_sim_scan_and_camera(monitor):
    async def run():
        async with Connected(monitor.mon) as c:
            scan = await _first(c, '/scan', LaserScan)
            assert len(scan.ranges) > 100
            assert scan.angle_max > scan.angle_min
            img = await _first(c, '/camera/image_raw', Image)
            assert (img.width, img.height) == (640, 480)
            assert img.encoding in ('rgb8', 'bgr8', 'mono8')
            assert len(img.data) >= img.width * img.height
            info = await _first(c, '/camera/camera_info', CameraInfo)
            assert info.width == img.width and len(info.k) == 9
            comp = await _first(c, '/camera/image_raw/compressed', CompressedImage)
            # image_transport 的 format 形如 'rgb8; jpeg compressed bgr8'
            assert 'jpeg' in comp.format.lower() or 'png' in comp.format.lower()
            assert len(comp.data) > 1024
            # JPEG SOI / PNG magic（真实解码面，不是标记字节）
            assert bytes(comp.data[:2]) == b'\xff\xd8' or bytes(comp.data[:8]) == b'\x89PNG\r\n\x1a\n'
    _run(run())


def test_full_sim_pointcloud_and_map(monitor):
    async def run():
        async with Connected(monitor.mon) as c:
            cloud = await _first(c, '/test_cloud', PointCloud2)
            assert cloud.width > 0 and cloud.height > 0
            assert cloud.point_step > 0
            assert {f.name for f in cloud.fields} >= {'x', 'y', 'z'}
            gm = await _first(c, '/map', OccupancyGrid)
            assert gm.info.width > 0 and gm.info.height > 0
            assert len(gm.data) == gm.info.width * gm.info.height
            topic = await _first(c, '/topic', String)
            assert isinstance(topic.data, str)
    _run(run())


def test_full_sim_connection_graph(monitor):
    async def run():
        async with Connected(monitor.mon) as c:
            await c.subscribe_connection_graph()
            deadline = time.monotonic() + _TIMEOUT
            snap = {}
            while time.monotonic() < deadline:
                snap = c.graph_snapshot()
                if '/joint_states' in snap['topics']:
                    break
                await asyncio.sleep(0.4)
            assert '/joint_states' in snap['topics']
            assert any('turtlebot3' in n or 'amcl' in n or 'nav2' in n
                       for n in snap['nodes'])
    _run(run())


def test_full_sim_robot_mesh_route(monitor):
    """URDF mesh 经 robot-mesh 路由取到真实字节（独立可视化硬证据）."""

    async def run():
        async with Connected(monitor.mon) as c:
            desc = await _first(c, '/robot_description', String)
            m = re.search(r'package://([^/]+)/([^"\s<]+)', desc.data)
            assert m, 'URDF 无 package:// mesh'
            return m.group(1), m.group(2)

    pkg, rel = _run(run())
    with loopback_client(monitor.mon.gateway_port) as c:
        r = c.get(f'/api/ivg/robot-mesh/{pkg}/{rel}')
        assert r.status_code == 200, (pkg, rel, r.status_code)
        assert len(r.content) > 1024
