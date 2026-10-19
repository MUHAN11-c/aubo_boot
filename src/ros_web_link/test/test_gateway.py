"""ros_web_link 网关配置与路由单元测试."""
import os
import sys

import pytest

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from ros_web_link import config as cfg


def test_gateway_port_default():
    assert cfg.gateway_port() == 8095


def test_foxglove_bridge_port():
    assert cfg.foxglove_bridge_port() == 8765
    # vendored ivg_foxglove_bridge 服务于根路径；旧默认 "/ws/foxglove" 是悬空旋钮（已修）
    assert cfg.foxglove_bridge_ws_path() == "/"


def test_web_video_port():
    assert cfg.web_video_port() == 8089


def test_transport_defaults():
    """传输默认值对齐 foxglove_bridge 官方建议：ping 开 / deflate 关 / 有限重试."""
    assert cfg.gateway_ws_ping_interval() == 20.0
    assert cfg.gateway_ws_ping_timeout() == 20.0
    assert cfg.gateway_ws_per_message_deflate() is False
    assert cfg.foxglove_bridge_compression() is False
    assert cfg.proxy_upstream_connect_attempts() == 3
    assert cfg.proxy_upstream_connect_retry_ms() == 250.0


def test_proxy_lane_defaults():
    """有界数据通道默认值：4MiB 预算 + 上游 max_queue=4（慢客户端背压前置）."""
    assert cfg.proxy_data_lane_bytes() == 4194304
    assert cfg.proxy_upstream_max_queue() == 4


def _lane_cls():
    from ros_web_link.routes.foxglove_proxy import _BoundedForwardLane
    return _BoundedForwardLane


def test_bounded_lane_drop_oldest_keeps_control():
    """超字节预算丢最旧数据帧；JSON 控制帧永不丢、相对顺序保持."""
    lane = _lane_cls()(300)
    lane.put(b'A' * 100)
    lane.put('mid')           # 控制帧
    lane.put(b'B' * 100)
    lane.put(b'C' * 100)
    lane.put(b'D' * 50)       # 350 > 300 预算 → 触发丢最旧（丢至 80%）
    assert lane.dropped_data >= 1
    seq = [it[1] if isinstance(it[1], str) else it[1][:1] for it in lane._dq]
    assert seq[0] == 'mid'                        # 控制帧保留在原位
    assert seq[1:] == [b'C', b'D']                # 最旧的 A/B 被丢
    assert lane.queued_bytes == 150


def test_bounded_lane_oversized_frame_drops_newest():
    """单帧大于预算且无可丢旧帧 → 丢该新帧（rosbridge 丢出站语义），控制帧不碰."""
    lane = _lane_cls()(100)
    lane.put('c' * 150)
    lane.put(b'D' * 150)
    assert lane.dropped_data == 1
    assert len(lane) == 1 and lane._dq[0][1] == 'c' * 150


def test_bounded_lane_unlimited_when_budget_zero():
    """预算 0 = 旧版无界行为（供对拍/特殊部署）。"""
    lane = _lane_cls()(0)
    for _ in range(100):
        lane.put(b'x' * 1000)
    assert lane.dropped_data == 0 and len(lane) == 100


def test_ws_stats_route_registered():
    """连接级/累计转发统计端点注册（诊断台「网关丢帧/积压」数据面）."""
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    pytest.importorskip('fastapi')
    from ros_web_link.app import create_app
    app = create_app()
    paths = {r.path for r in app.routes}
    assert '/api/ivg/ws/stats' in paths


def test_config_defaults_without_yaml(monkeypatch):
    """yaml 全空时访问器回退硬编码默认（PyYAML 缺失场景）."""
    monkeypatch.setattr(cfg, '_cfg', {})
    assert cfg.gateway_port() == 8095
    assert cfg.foxglove_bridge_ws_path() == '/'
    assert cfg.gateway_ws_ping_interval() == 20.0
    assert cfg.foxglove_bridge_compression() is False


def _fastapi_ok() -> bool:
    """检查 fastapi/starlette 版本兼容（系统包路径优先问题导致不兼容时跳过）."""
    try:
        from fastapi import APIRouter
        APIRouter()
        return True
    except (TypeError, ImportError):
        return False


def test_create_app_routes():
    """create_app 挂载核心传输路由 + 纯传输模式 HTML 根（console 页）."""
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    fastapi = pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    pytest.importorskip('websockets')
    from ros_web_link.app import create_app
    app = create_app()
    paths = {r.path for r in app.routes}
    assert '/health' in paths
    assert '/ws/foxglove' in paths
    assert any('robot-mesh' in p for p in paths)
    assert '/ws/rosbridge' not in paths
    # 纯传输模式：本包 web/public 挂 /（console.html 诊断台可达；'/' 归一为 ''）
    mount_names = {getattr(r, 'name', '') for r in app.routes
                   if type(r).__name__ == 'Mount'}
    assert 'js' in mount_names and 'html' in mount_names
    assert fastapi is not None


def test_composed_mode_mounts_console_under_rwl(tmp_path):
    """组合模式（html_root 给定）：UI 独占 /，本包诊断页挂 /rwl."""
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    pytest.importorskip('fastapi')
    from fastapi import APIRouter
    from ros_web_link.app import create_app
    ui_root = tmp_path / 'ui'
    (ui_root / 'js').mkdir(parents=True)
    (ui_root / 'index.html').write_text('<html>ui</html>', encoding='utf-8')

    bff = APIRouter(prefix='/api/v1')

    @bff.get('/custom')
    async def custom():
        return {'ok': True}

    app = create_app(html_root=str(ui_root), sub_routers=[bff])
    paths = {r.path for r in app.routes}
    assert '/api/v1/custom' in paths
    mount_names = {getattr(r, 'name', '') for r in app.routes
                   if type(r).__name__ == 'Mount'}
    assert 'html' in mount_names and 'rwl' in mount_names


def test_health_endpoint_via_testclient(monkeypatch):
    """/health 返回 200 且不依赖未初始化的 app.state（static_root 回归）.

    组件探测 monkeypatch 为确定值（TestClient 环境无真实桥，直接探测会因
    开发机上是否跑着 8765 桥而抖动）。
    """
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    try:
        from fastapi.testclient import TestClient
    except Exception:
        pytest.skip('TestClient 依赖（requests）不可用')
    from ros_web_link.app import create_app
    from ros_web_link.routes import health as health_mod

    async def probe_ok(key, host, port):
        return key == 'foxglove_bridge'

    async def probe_all_down(key, host, port):
        return False

    monkeypatch.setattr(health_mod, '_probe_cached', probe_ok)
    app = create_app()
    with TestClient(app) as client:
        r = client.get('/health')
        assert r.status_code == 200
        body = r.json()
        assert body['status'] == 'ok', body
        assert body['foxglove_bridge'] == 'ok'
        assert body['web_video'] == 'down'
        # 纯传输模式：console.html 应可从 HTML 根取到
        page = client.get('/console.html')
        assert page.status_code == 200
        assert '传输诊断台' in page.text
        # 独立入口 / → /console.html；白色二次元商业主题品牌锚点
        root = client.get('/', follow_redirects=False)
        assert root.status_code in (307, 308)
        assert root.headers['location'].endswith('/console.html')
        assert 'LIVE STUDIO' in page.text
        assert '自动数据面' in page.text

        # 桥探测失败 → 整体 degraded（仍 200，不破坏现有监控）
        monkeypatch.setattr(health_mod, '_probe_cached', probe_all_down)
        r2 = client.get('/health')
        assert r2.status_code == 200
        body2 = r2.json()
        assert body2['status'] == 'degraded', body2
        assert body2['foxglove_bridge'] == 'down'


def test_sub_router_injection():
    """UI 层 BFF 子路由可注入."""
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    fastapi = pytest.importorskip('fastapi')
    from fastapi import APIRouter
    from ros_web_link.app import create_app

    bff = APIRouter(prefix="/api/v1")

    @bff.get("/custom")
    async def custom():
        return {"ok": True}

    app = create_app(sub_routers=[bff])
    paths = {r.path for r in app.routes}
    assert '/api/v1/custom' in paths


# ═══════════════════════════════ robot_mesh 解析 ═══════════════════════════════

def test_robot_mesh_resolve(tmp_path):
    pytest.importorskip('ament_index_python')
    from ros_web_link.routes.robot_mesh import _is_under, _resolve_mesh

    share = tmp_path / 'share'
    (share / 'meshes' / 'Links').mkdir(parents=True)
    (share / 'meshes' / 'Links' / 'hand.dae').write_text('dae', encoding='utf-8')

    # 精确路径命中
    hit = _resolve_mesh(share, 'meshes/Links/hand.dae')
    assert hit is not None and hit.name == 'hand.dae'
    # 大小写回退命中（Windows 导出 URDF 常见）
    hit_ci = _resolve_mesh(share, 'meshes/links/HAND.DAE')
    assert hit_ci is not None and hit_ci.name == 'hand.dae'
    # 不存在
    assert _resolve_mesh(share, 'meshes/Links/nope.dae') is None
    # 空路径
    assert _resolve_mesh(share, '') is None


def test_robot_mesh_symlink_layout_and_traversal(tmp_path):
    """词法 containment：colcon symlink-install 合法布局放行，`..` 穿越仍拒.

    resolve-based containment 会把 install share 树内指回源码树的合法符号
    链接误判越界（TB3 URDF mesh 全 404 实证，2026-10-16）；穿越防线 =
    URL `..` 拒绝 + normpath 树下检查，与 SymlinkStaticFiles 同款裁量和先例。
    """
    pytest.importorskip('ament_index_python')
    from ros_web_link.routes.robot_mesh import _is_under, _resolve_mesh

    share = tmp_path / 'share'
    src_tree = tmp_path / 'src' / 'pkg'
    outside = tmp_path / 'outside'
    (share / 'meshes').mkdir(parents=True)
    (src_tree / 'meshes').mkdir(parents=True)
    outside.mkdir()
    (src_tree / 'meshes' / 'base.stl').write_text('ok', encoding='utf-8')
    (outside / 'evil.stl').write_text('evil', encoding='utf-8')
    # colcon --symlink-install：资源目录 symlink 回源码树 → 应命中
    (share / 'models').symlink_to(src_tree / 'meshes')
    # share 树内单文件 symlink 指向外部（词法在 share 下）→ 按布局语义放行
    (share / 'meshes' / 'linked.stl').symlink_to(outside / 'evil.stl')

    assert _resolve_mesh(share, 'models/base.stl') is not None
    assert _is_under(share / 'meshes' / 'linked.stl', share)
    # URL 穿越仍拒：`..` 经 normpath 折叠后逃出 share 树
    assert _resolve_mesh(share, '../outside/evil.stl') is None
    assert not _is_under(share / '..' / 'outside' / 'evil.stl', share)
    assert _resolve_mesh(share, 'meshes/../..') is None


# ═══════════════════════════════ 静态文件类 ═══════════════════════════════

def test_multi_static_files_lookup(tmp_path):
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    from ros_web_link.app import MultiStaticFiles

    root_a = tmp_path / 'a'
    root_b = tmp_path / 'b'
    (root_a / 'core').mkdir(parents=True)
    (root_b / 'ui').mkdir(parents=True)
    (root_b / 'core').mkdir()
    (root_a / 'core' / 'x.js').write_text('A', encoding='utf-8')
    (root_b / 'core' / 'x.js').write_text('B-override', encoding='utf-8')  # 后根不抢先生效
    (root_b / 'ui' / 'y.js').write_text('B', encoding='utf-8')

    msf = MultiStaticFiles(directories=[str(root_a), str(root_b)])
    full, stat = msf.lookup_path('core/x.js')
    assert full.endswith('x.js')
    with open(full, encoding='utf-8') as f:
        assert f.read() == 'A'  # 先序根优先
    full2, _ = msf.lookup_path('ui/y.js')
    assert full2 is not None
    assert msf.lookup_path('missing.js') == (None, None)


def test_symlink_static_files_lookup(tmp_path):
    """colcon --symlink-install 形态：文件是指向 build/ 的符号链接仍可服务."""
    if not _fastapi_ok():
        pytest.skip("fastapi/starlette 版本不兼容（系统包路径优先）")
    from ros_web_link.app import SymlinkStaticFiles

    real_dir = tmp_path / 'build'
    real_dir.mkdir()
    (real_dir / 'app.js').write_text('content', encoding='utf-8')
    install_dir = tmp_path / 'install'
    (install_dir / 'sub').mkdir(parents=True)
    (install_dir / 'sub' / 'app.js').symlink_to(real_dir / 'app.js')

    ssf = SymlinkStaticFiles(directory=str(install_dir), html=True)
    full, stat = ssf.lookup_path('sub/app.js')
    assert full is not None and stat is not None
    # 穿越拒绝
    assert ssf.lookup_path('../escape') == (None, None)
    assert ssf.lookup_path('a/../../escape') == (None, None)


# ═══════════════════════════════ WS 代理上游连接 ═══════════════════════════════

def test_upstream_url_uses_ws_path(monkeypatch):
    from ros_web_link.routes import foxglove_proxy as proxy

    monkeypatch.setattr(cfg, '_cfg', {
        'foxglove_bridge': {'host': 'h', 'port': 1, 'ws_path': '/x'},
    })
    assert proxy._foxglove_bridge_upstream_url() == 'ws://h:1/x'
    # 无前导斜杠自动补
    monkeypatch.setattr(cfg, '_cfg', {
        'foxglove_bridge': {'host': 'h', 'port': 1, 'ws_path': 'x'},
    })
    assert proxy._foxglove_bridge_upstream_url() == 'ws://h:1/x'


def test_connect_upstream_with_retry(monkeypatch):
    """有限次重试 + 全部失败抛最后一次异常（抖动退避不重试到成功之外的场景）."""
    pytest.importorskip('websockets')
    import asyncio
    from ros_web_link.routes import foxglove_proxy as proxy

    monkeypatch.setattr(cfg, '_cfg', {
        'proxy': {'upstream_connect_attempts': 3, 'upstream_connect_retry_ms': 1},
    })

    calls = {'n': 0}

    class FakeConn:
        async def close(self):
            pass

    async def fail_always(*_a, **_k):
        calls['n'] += 1
        raise ConnectionRefusedError('down')

    async def fail_twice_then_ok(*_a, **_k):
        calls['n'] += 1
        if calls['n'] < 3:
            raise ConnectionRefusedError('down')
        return FakeConn()

    monkeypatch.setattr(proxy.websockets, 'connect', fail_always)
    with pytest.raises(ConnectionRefusedError):
        asyncio.run(proxy._connect_upstream_with_retry('ws://x/', 'foxglove.websocket.v1'))
    assert calls['n'] == 3

    # 第三次成功：返回连接对象
    calls['n'] = 0
    monkeypatch.setattr(proxy.websockets, 'connect', fail_twice_then_ok)
    conn = asyncio.run(proxy._connect_upstream_with_retry('ws://x/', 'foxglove.websocket.v1'))
    assert isinstance(conn, FakeConn)
    assert calls['n'] == 3
