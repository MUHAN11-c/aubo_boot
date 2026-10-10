"""网关配置与应用工厂冒烟测试（零 ROS 图依赖）."""
import os
import sys

import pytest

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from aubo_ros2_web_dashboard import config as cfg  # noqa: E402


def test_gateway_port_8095_not_8090():
    """网关端口必须 8095（8090 归 peach_observability）."""
    assert cfg.gateway_port() == 8095


def test_defaults_yaml_loaded():
    """defaults.yaml 已加载（rosbridge 段非空）."""
    assert cfg.rosbridge_port() == 9090
    assert cfg.foxglove_bridge_port() == 8765
    assert cfg.web_video_port() == 8089


def test_robot_status_topic_mapped():
    """前端默认机械臂状态源指向本仓 aubo_io_controller."""
    vp = cfg.vision_panel_config()
    topics = vp['vision']['topics']
    robot = next(t for t in topics if t['id'] == 'topic-robot')
    assert robot['default'] == '/aubo_io_controller/robot_status'
    assert robot['msg_type'] == 'aubo_msgs/msg/RobotStatus'


def test_create_app_routes():
    """create_app 挂载核心路由（依赖 fastapi/httpx/websockets，缺则跳过）."""
    fastapi = pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    pytest.importorskip('websockets')
    web_root = os.path.join(_PKG_ROOT, 'web', 'public')
    if not os.path.isdir(web_root):
        pytest.skip('web/public 不存在')
    from aubo_ros2_web_dashboard.gateway.app import create_app
    app = create_app(web_root)
    paths = {r.path for r in app.routes}
    assert '/health' in paths
    assert '/api/v1/runtime' in paths
    assert '/api/v1/tool-geometries' in paths
    assert fastapi is not None
