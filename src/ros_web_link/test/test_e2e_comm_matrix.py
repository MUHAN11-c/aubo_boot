"""E2E 通信矩阵 — web↔ROS 全生态通信原语经网关代理全链路验证.

被测链路（与浏览器完全同路）:
  pytest(模拟浏览器) → 网关 :gw/ws/foxglove (uvicorn 代理) → ivg_foxglove_bridge → ROS 图

覆盖 ROS 生态通信原语（对照 rosbridge op 语义）:
  topic subscribe / advertise+publish / service call / action send_goal+feedback
  +cancel+get_result / 参数 get+set（标准参数服务）/ TF / connectionGraph 图谱
另含网关 HTTP 面: /health（含桥组件探测）、视频代理负例（上游不在时 502/504）。

运行前提（二选一）:
  1) aubo_py3.12 venv 下直接 pytest（venv 含 fastapi+websockets+rclpy 全栈）
  2) 已 source Jazzy+工作区、且当前 python 有 websockets（否则本文件 skip）

被测端为 rwl_comm_fixture 节点（本包自带），不依赖任何真机/演示栈。
栈编排复用 e2e_harness.Stack（桥 → 网关 → fixture 子进程组，SIGINT 整组收尾）；
网络边界：本测试只访问 127.0.0.1 回环（端口由内核分配），不出本机。
"""
from __future__ import annotations

import asyncio
import time
import uuid as uuid_mod

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from example_interfaces.action import Fibonacci  # noqa: E402
from example_interfaces.srv import AddTwoInts  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402

from .e2e_harness import Connected, Stack, loopback_client  # noqa: E402

_HEALTH_PATH = '/health'
_VIDEO_PROXY_PATH = '/api/ivg/proxy/web-video/stream?topic=/x'


@pytest.fixture(scope='module')
def stack():
    st = Stack(fixtures=('ros_web_link.comm_fixture',))
    try:
        st.start()
    except FileNotFoundError as e:
        pytest.skip(f'ivg_foxglove_bridge 不在环境中: {e}')
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


# ═══════════════════════════════ HTTP 面 ═══════════════════════════════

def test_health(stack):
    with loopback_client(stack.gateway_port) as client:
        r = client.get(_HEALTH_PATH)
    assert r.status_code == 200
    body = r.json()
    # Stack 的桥在跑：整体 ok 且桥组件探测为绿（web_video 不在 Stack 内，down 属预期）
    assert body['status'] == 'ok', body
    assert body['foxglove_bridge'] == 'ok', body


def test_video_proxy_negative_when_upstream_absent(stack):
    """web_video_server 不在时，视频代理必须给出明确 5xx（不得挂死）."""
    with loopback_client(stack.gateway_port) as client:
        r = client.get(_VIDEO_PROXY_PATH)
    assert r.status_code in (502, 504), r.status_code


# ═════════════════════ WS 传输面：协议握手 ═════════════════════

def test_server_info_capabilities(stack):
    async def run():
        async with Connected(stack) as c:
            info = c.server_info
            assert info.get('op') == 'serverInfo'
            caps = set(info.get('capabilities', []))
            assert {'clientPublish', 'services', 'connectionGraph'} <= caps, caps
            assert info.get('sessionId')
    asyncio.run(run())


# ═════════════════════ 话题：订阅 ═════════════════════

def test_topic_subscribe_chatter(stack):
    async def run():
        async with Connected(stack) as c:
            sub = await c.subscribe('/rwl_demo/chatter', String)
            try:
                msg = await sub.next(timeout=8.0)
                assert msg.data.startswith('rwl hello #'), msg.data
            finally:
                await sub.cancel()
    asyncio.run(run())


# ═════════════════════ 话题：客户端发布（roundtrip） ═════════════════════

def test_topic_publish_roundtrip(stack):
    async def run():
        async with Connected(stack) as c:
            reply = await c.subscribe('/rwl_demo/echo_reply', String)
            token = 'web-' + uuid_mod.uuid4().hex[:8]
            try:
                # advertise 后存在 DDS 匹配竞态：重试窗口内发到收到 echo 为止
                deadline = time.monotonic() + 10.0
                got = None
                while got is None and time.monotonic() < deadline:
                    await c.publish('/rwl_demo/echo', 'std_msgs/msg/String',
                                    String(data=token))
                    try:
                        msg = await reply.next(timeout=1.0)
                        got = msg.data
                    except asyncio.TimeoutError:
                        continue
                assert got == 'echo:' + token, got
            finally:
                await reply.cancel()
    asyncio.run(run())


# ═════════════════════ 服务调用 ═════════════════════

def test_service_call_add_two_ints(stack):
    async def run():
        async with Connected(stack) as c:
            req = AddTwoInts.Request()
            req.a, req.b = 20, 22
            resp = await c.call_service(
                '/rwl_demo/add_two_ints', req, AddTwoInts.Response)
            assert resp.sum == 42
    asyncio.run(run())


# ═════════════════════ 参数（标准参数服务 get/set） ═════════════════════

def test_params_get_set(stack):
    async def run():
        async with Connected(stack) as c:
            # get
            g = GetParameters.Request()
            g.names = ['rwl.answer']
            resp = await c.call_service(
                '/comm_fixture/get_parameters', g, GetParameters.Response)
            assert len(resp.values) == 1
            assert resp.values[0].type == ParameterType.PARAMETER_INTEGER
            assert resp.values[0].integer_value == 42

            # set 43
            p = Parameter()
            p.name = 'rwl.answer'
            p.value.type = ParameterType.PARAMETER_INTEGER
            p.value.integer_value = 43
            s = SetParameters.Request()
            s.parameters = [p]
            sresp = await c.call_service(
                '/comm_fixture/set_parameters', s, SetParameters.Response)
            assert sresp.results[0].successful, sresp.results[0].reason

            # 回读
            resp2 = await c.call_service(
                '/comm_fixture/get_parameters', g, GetParameters.Response)
            assert resp2.values[0].integer_value == 43

            # 还原默认
            p.value.integer_value = 42
            s2 = SetParameters.Request()
            s2.parameters = [p]
            await c.call_service(
                '/comm_fixture/set_parameters', s2, SetParameters.Response)
    asyncio.run(run())


# ═════════════════════ Action：goal + feedback + result ═════════════════════

def _fib_send_goal(goal_id: bytes, order: int):
    req = Fibonacci.Impl.SendGoalService.Request()
    req.goal_id = UUID()
    req.goal_id.uuid = list(goal_id)  # type: ignore[assignment]
    req.goal.order = order
    return req


def _fib_cancel(goal_id: bytes):
    from builtin_interfaces.msg import Time
    req = Fibonacci.Impl.CancelGoalService.Request()
    req.goal_info.goal_id = UUID()
    req.goal_info.goal_id.uuid = list(goal_id)  # type: ignore[assignment]
    req.goal_info.stamp = Time()  # 0 = 取消该 goal_id 本身
    return req


def _fib_get_result(goal_id: bytes):
    req = Fibonacci.Impl.GetResultService.Request()
    req.goal_id = UUID()
    req.goal_id.uuid = list(goal_id)  # type: ignore[assignment]
    return req


def test_action_result_and_feedback(stack):
    async def run():
        async with Connected(stack) as c:
            gid = uuid_mod.uuid4().bytes
            # 1) 订 feedback（action server 生命周期内持续发布）
            fb = await c.subscribe(
                '/rwl_demo/fibonacci/_action/feedback',
                Fibonacci.Impl.FeedbackMessage)
            # 2) send_goal
            accepted = await c.call_service(
                '/rwl_demo/fibonacci/_action/send_goal',
                _fib_send_goal(gid, 8), Fibonacci.Impl.SendGoalService.Response)
            assert accepted.accepted
            # 3) 收 feedback（0.15s/步；order=8 → 终态序列 10 项）
            seen_fb = 0
            try:
                while True:
                    m = await fb.next(timeout=3.0)
                    seq = list(m.feedback.sequence)
                    assert seq[0] == 0 and seq[1] == 1
                    seen_fb += 1
                    if len(seq) >= 10:
                        break
            finally:
                await fb.cancel()
            assert seen_fb >= 1
            # 4) get_result（应已完成）
            res = await c.call_service(
                '/rwl_demo/fibonacci/_action/get_result',
                _fib_get_result(gid), Fibonacci.Impl.GetResultService.Response)
            assert res.status == 4, '期望 SUCCEEDED(4)，实际 ' + str(res.status)
            assert list(res.result.sequence) == [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
    asyncio.run(run())


def test_action_cancel(stack):
    async def run():
        async with Connected(stack) as c:
            gid = uuid_mod.uuid4().bytes
            accepted = await c.call_service(
                '/rwl_demo/fibonacci/_action/send_goal',
                _fib_send_goal(gid, 40), Fibonacci.Impl.SendGoalService.Response)
            assert accepted.accepted
            await asyncio.sleep(0.8)  # 让 goal 进入 EXECUTING
            cancel = await c.call_service(
                '/rwl_demo/fibonacci/_action/cancel_goal',
                _fib_cancel(gid), Fibonacci.Impl.CancelGoalService.Response)
            assert cancel.return_code == 0
            assert len(cancel.goals_canceling) >= 1
            res = await c.call_service(
                '/rwl_demo/fibonacci/_action/get_result',
                _fib_get_result(gid), Fibonacci.Impl.GetResultService.Response,
                timeout=10.0)
            assert res.status == 5, '期望 CANCELED(5)，实际 ' + str(res.status)
    asyncio.run(run())


# ═════════════════════ TF ═════════════════════

def test_tf_stream(stack):
    async def run():
        from tf2_msgs.msg import TFMessage
        async with Connected(stack) as c:
            # /tf 上发布的是 tf2_msgs/TFMessage（变换数组）
            sub = await c.subscribe('/tf', TFMessage)
            try:
                msg = await sub.next(timeout=8.0)
                frames = {t.child_frame_id for t in msg.transforms}
                assert 'rwl_demo_marker' in frames, frames
            finally:
                await sub.cancel()
    asyncio.run(run())


# ═════════════════════ 连接图谱 ═════════════════════

def test_connection_graph(stack):
    async def run():
        async with Connected(stack) as c:
            await c.subscribe_connection_graph()
            deadline = time.monotonic() + 10.0
            snap = {}
            while time.monotonic() < deadline:
                await asyncio.sleep(0.4)
                snap = c.graph_snapshot()
                if '/rwl_demo/chatter' in snap['topics'] and \
                        '/rwl_demo/add_two_ints' in snap['services']:
                    break
            assert '/rwl_demo/chatter' in snap['topics'], snap
            assert '/rwl_demo/add_two_ints' in snap['services'], snap
            assert any('comm_fixture' in n for n in snap['nodes']), snap
    asyncio.run(run())
