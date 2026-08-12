# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""桃子感知与重建的只读 HTTP 可视化网关."""

from __future__ import annotations

from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
from peach_harvest_msgs.msg import HarvestState
from peach_harvest_msgs.srv import ControlHarvest, SetOperationPolicy
from peach_pose_msgs.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    PeachTargetObservationArray,
)
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .codec import (
    candidate_array,
    finite_or_none,
    fitting_array,
    parse_json_text,
    target_observations,
    vector_stamped,
)
from .control import CommandGuard, ProfileStore


class DashboardState:
    """HTTP 与 ROS 回调之间的线程安全最新值缓存."""

    def __init__(self):
        """创建空状态."""
        self._lock = threading.Lock()
        self._revision = 0
        self._started = time.time()
        self._values = {
            'perception': {
                'harvest': {},
                'targets': {},
            },
            'reconstruction': {
                'status': {},
                'diagnostics': {},
                'grasp_decision': {},
            },
            'refined': {
                'pose': {},
                'axis': {},
                'diagnostics': {},
            },
            'approach': {
                'status': {},
            },
            'orchestration': {
                'state': {},
                'audit': [],
            },
        }
        self._updated = {}

    def update(self, section: str, key: str, value) -> None:
        """更新一个结构化状态区段."""
        now = time.time()
        with self._lock:
            self._values[section][key] = finite_or_none(value)
            self._updated[f'{section}.{key}'] = now
            self._revision += 1

    def snapshot(self) -> dict:
        """返回浏览器状态快照与话题年龄."""
        now = time.time()
        with self._lock:
            result = json.loads(json.dumps(self._values, ensure_ascii=False))
            result['system'] = {
                'revision': self._revision,
                'server_time': now,
                'uptime_s': now - self._started,
                'topic_age_s': {
                    key: round(now - stamp, 3)
                    for key, stamp in self._updated.items()
                },
            }
            return result

    def orchestrator_revision(self) -> int:
        """返回业务状态 revision，不与页面缓存 revision 混用。."""
        with self._lock:
            return int(self._values['orchestration']['state'].get(
                'revision', 0))

    def operation_mode(self) -> int:
        """返回编排器操作模式。."""
        with self._lock:
            return int(self._values['orchestration']['state'].get(
                'operation_mode', 0))

    def audit(self, action: str, accepted: bool, message: str) -> None:
        """记录有界 Web 操作审计。."""
        with self._lock:
            events = self._values['orchestration']['audit']
            events.append({
                'time': time.time(), 'action': action,
                'accepted': accepted, 'message': message,
            })
            del events[:-200]
            self._revision += 1


class PeachPerceptionWeb(Node):
    """订阅两视觉包输出并提供只读 HTTP API."""

    def __init__(self):
        """声明参数、订阅话题并初始化共享缓存."""
        super().__init__('peach_perception_web')
        self._declare_parameters()
        self._state = DashboardState()
        self._session_nonce = secrets.token_urlsafe(32)
        self._guard = CommandGuard(self._session_nonce)
        share = Path(get_package_share_directory('peach_perception_web'))
        configured_profiles = str(
            self.get_parameter('profile_directory').value).strip()
        profile_root = (Path(configured_profiles) if configured_profiles else
                        share.parents[3] / 'peach_profiles')
        self._profiles = ProfileStore(profile_root)
        self._http = None
        self._create_subscriptions()
        self._create_control_clients()

    def _declare_parameters(self) -> None:
        """集中声明 Web 和话题参数."""
        parameters = {
            'host': ('127.0.0.1', 'HTTP 监听地址；局域网访问显式设 0.0.0.0'),
            'port': (8090, 'HTTP 监听端口'),
            'profile_directory': (
                '', '参数档案目录；空值使用工作区 peach_profiles'),
            'target_observations_topic': (
                '/peach/perception/target_observations', '全局目标快照话题'),
            'harvest_state_topic': (
                '/peach/perception/harvest_state', '采摘计划 JSON 话题'),
            'reconstruction_status_topic': (
                '/peach/reconstruction/status', '重建状态话题'),
            'reconstruction_diagnostics_topic': (
                '/peach/reconstruction/diagnostics', '重建诊断 JSON 话题'),
            'grasp_decision_topic': (
                '/peach/reconstruction/grasp_decision', '抓取许可 JSON 话题'),
            'refined_pose_topic': (
                '/peach/reconstruction/refined_pose', '精化位姿话题'),
            'refined_axis_topic': (
                '/peach/reconstruction/refined_axis', '精化轴线话题'),
            'refined_diagnostics_topic': (
                '/peach/reconstruction/refined_diagnostics',
                '精化质量话题'),
            'approach_status_topic': (
                '/peach_approach_grasp_node/status',
                '主动视觉靠近与抓取编排 JSON 话题'),
            'orchestrator_state_topic': (
                '/peach_harvest_orchestrator/state',
                '采摘编排器类型化状态话题'),
        }
        for name, (default, description) in parameters.items():
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description))

    def _topic(self, parameter: str) -> str:
        return str(self.get_parameter(parameter).value)

    def _create_subscriptions(self) -> None:
        """建立全部只读订阅."""
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PeachTargetObservationArray,
            self._topic('target_observations_topic'),
            self._targets_callback, reliable_qos)
        self.create_subscription(
            String, self._topic('harvest_state_topic'),
            lambda msg: self._state.update(
                'perception', 'harvest', parse_json_text(msg.data)),
            latched_qos)
        self.create_subscription(
            String, self._topic('reconstruction_status_topic'),
            lambda msg: self._state.update(
                'reconstruction', 'status',
                parse_json_text(msg.data, 'state')),
            latched_qos)
        self.create_subscription(
            String, self._topic('reconstruction_diagnostics_topic'),
            lambda msg: self._state.update(
                'reconstruction', 'diagnostics',
                parse_json_text(msg.data)),
            latched_qos)
        self.create_subscription(
            String, self._topic('grasp_decision_topic'),
            lambda msg: self._state.update(
                'reconstruction', 'grasp_decision',
                parse_json_text(msg.data)),
            latched_qos)
        self.create_subscription(
            BagGraspCandidateArray, self._topic('refined_pose_topic'),
            lambda msg: self._state.update(
                'refined', 'pose', candidate_array(msg)), latched_qos)
        self.create_subscription(
            Vector3Stamped, self._topic('refined_axis_topic'),
            lambda msg: self._state.update(
                'refined', 'axis', vector_stamped(msg)), latched_qos)
        self.create_subscription(
            BagFittingArray, self._topic('refined_diagnostics_topic'),
            lambda msg: self._state.update(
                'refined', 'diagnostics', fitting_array(msg)), latched_qos)
        self.create_subscription(
            String, self._topic('approach_status_topic'),
            lambda msg: self._state.update(
                'approach', 'status', parse_json_text(msg.data)),
            latched_qos)
        self.create_subscription(
            HarvestState, self._topic('orchestrator_state_topic'),
            self._orchestrator_callback, latched_qos)

    def _orchestrator_callback(self, message: HarvestState) -> None:
        """把类型化业务状态转换为稳定的浏览器对象。."""
        self._state.update('orchestration', 'state', {
            'revision': message.revision,
            'run_id': message.run_id,
            'cycle_id': message.cycle_id,
            'target_id': message.target_id,
            'operation_mode': message.operation_mode,
            'batch_state': message.batch_state,
            'target_phase': message.target_phase,
            'action_active': message.action_active,
            'auto_start_enabled': message.auto_start_enabled,
            'execution_enabled': message.execution_enabled,
            'grasp_enabled': message.grasp_enabled,
            'tool_enabled': message.tool_enabled,
            'recovery_required': message.recovery_required,
            'progress': message.progress,
            'message': message.message,
            'blockers': list(message.blockers),
        })

    def _targets_callback(self, message) -> None:
        try:
            value = target_observations(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'目标快照转换失败: {error}')
            return
        self._state.update('perception', 'targets', value)

    def _create_control_clients(self) -> None:
        """创建编排控制和全部兼容手动调试服务客户端。."""
        self._control_client = self.create_client(
            ControlHarvest, '/peach_harvest_orchestrator/control')
        self._policy_client = self.create_client(
            SetOperationPolicy,
            '/peach_harvest_orchestrator/set_operation_policy')
        trigger_names = {
            'pose_reset': '/peach_pose_node/reset_global_targets',
            'pose_query': '/peach_pose_node/query_harvest_state',
            'pose_complete': '/peach_pose_node/complete_selected_target',
            'recon_start': '/peach_reconstruction_node/start_reconstruction',
            'recon_capture': '/peach_reconstruction_node/capture_frame',
            'recon_remove': '/peach_reconstruction_node/remove_last_frame',
            'recon_reset': '/peach_reconstruction_node/reset_reconstruction',
            'recon_finalize': '/peach_reconstruction_node/finalize_reconstruction',
            'recon_save': '/peach_reconstruction_node/save_session',
            'recon_query': (
                '/peach_reconstruction_node/query_reconstruction_state'),
            'approach_start': '/peach_approach_grasp_node/start_cycle',
            'approach_preview': (
                '/peach_approach_grasp_node/preview_approach_insert'),
            'approach_contact': (
                '/peach_approach_grasp_node/preview_full_contact'),
            'approach_cancel': '/peach_approach_grasp_node/cancel_cycle',
            'approach_recovery': (
                '/peach_approach_grasp_node/acknowledge_recovery'),
            'approach_query': '/peach_approach_grasp_node/query_state',
        }
        self._debug_clients = {
            key: self.create_client(Trigger, service)
            for key, service in trigger_names.items()
        }
        self._arm_client = self.create_client(
            SetBool, '/peach_approach_grasp_node/set_execution_armed')

    @staticmethod
    def _wait_future(future, timeout: float = 5.0):
        """从 HTTP worker 等待 ROS future，不占用执行器线程。."""
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            raise TimeoutError('ROS 服务响应超时')
        return future.result()

    def call_debug(self, action: str, armed: bool = False) -> tuple[bool, str]:
        """调用一个维护模式调试服务并统一响应。."""
        if action == 'approach_arm':
            client = self._arm_client
            request = SetBool.Request()
            request.data = bool(armed)
        else:
            client = self._debug_clients.get(action)
            if client is None:
                return False, '未知调试命令'
            request = Trigger.Request()
        if not client.service_is_ready():
            return False, '目标 ROS 服务不可用'
        try:
            response = self._wait_future(client.call_async(request))
        except (TimeoutError, RuntimeError) as error:
            return False, str(error)
        return bool(response.success), str(response.message)

    def call_control(
            self, command: int, request_id: str,
            expected_revision: int, reason: str) -> tuple[bool, str]:
        """调用带 revision 的编排控制服务。."""
        if not self._control_client.service_is_ready():
            return False, '编排控制服务不可用'
        request = ControlHarvest.Request()
        request.command = command
        request.request_id = request_id
        request.expected_revision = expected_revision
        request.reason = reason
        try:
            response = self._wait_future(
                self._control_client.call_async(request))
        except (TimeoutError, RuntimeError) as error:
            return False, str(error)
        return bool(response.accepted), str(response.message)

    def call_policy(self, body: dict) -> tuple[bool, str]:
        """原子更新自动运行与三级运动使能策略。."""
        if not self._policy_client.service_is_ready():
            return False, '编排策略服务不可用'
        request = SetOperationPolicy.Request()
        request.request_id = str(body.get('request_id', ''))
        request.expected_revision = int(body.get('expected_revision', 0))
        request.auto_start_enabled = bool(body.get(
            'auto_start_enabled', True))
        request.execution_enabled = bool(body.get(
            'execution_enabled', False))
        request.grasp_enabled = bool(body.get('grasp_enabled', False))
        request.tool_enabled = bool(body.get('tool_enabled', False))
        try:
            response = self._wait_future(
                self._policy_client.call_async(request))
        except (TimeoutError, RuntimeError) as error:
            return False, str(error)
        return bool(response.accepted), str(response.message)

    def start_http(self) -> None:
        """启动静态文件和只读 API 服务."""
        web_root = Path(get_package_share_directory(
            'peach_perception_web')) / 'web'
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            """该网关专用 HTTP handler."""

            def log_message(self, fmt, *args):
                gateway.get_logger().debug(fmt % args)

            def _send(
                    self, status, content_type, data, cache='no-store',
                    set_session=False):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', cache)
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('X-Frame-Options', 'DENY')
                self.send_header(
                    'Content-Security-Policy',
                    "default-src 'self'; object-src 'none'; "
                    "frame-ancestors 'none'")
                if set_session:
                    self.send_header(
                        'Set-Cookie',
                        'peach_session=' + gateway._session_nonce +
                        '; Path=/; HttpOnly; SameSite=Strict')
                self.end_headers()
                self.wfile.write(data)

            def _json(self, value, status=HTTPStatus.OK):
                data = json.dumps(
                    value, ensure_ascii=False,
                    separators=(',', ':')).encode('utf-8')
                self._send(status, 'application/json; charset=utf-8', data)

            def _session_value(self):
                cookie = SimpleCookie(self.headers.get('Cookie', ''))
                item = cookie.get('peach_session')
                return item.value if item is not None else ''

            def _same_origin(self):
                origin = self.headers.get('Origin', '')
                if not origin:
                    return False
                parsed = urlparse(origin)
                return (parsed.scheme in ('http', 'https') and
                        parsed.netloc == self.headers.get('Host', ''))

            def _read_json(self):
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0 or length > 32768:
                    raise ValueError('请求体大小无效')
                value = json.loads(self.rfile.read(length).decode('utf-8'))
                if not isinstance(value, dict):
                    raise ValueError('请求体必须是 JSON 对象')
                return value

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path
                if path == '/api/state':
                    self._json(gateway._state.snapshot())
                    return
                if path == '/api/profiles':
                    self._json({'profiles': gateway._profiles.list_names()})
                    return
                assets = {
                    '/': ('index.html', 'text/html; charset=utf-8'),
                    '/index.html': ('index.html', 'text/html; charset=utf-8'),
                    '/app.css': ('app.css', 'text/css; charset=utf-8'),
                    '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                }
                asset = assets.get(path)
                if asset is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = (web_root / asset[0]).read_bytes()
                self._send(
                    HTTPStatus.OK, asset[1], data,
                    cache='public, max-age=60',
                    set_session=path in ('/', '/index.html'))

            def do_POST(self):
                path = urlparse(self.path).path
                if not self._same_origin():
                    self._json({'accepted': False,
                                'message': '请求来源无效'},
                               HTTPStatus.FORBIDDEN)
                    return
                try:
                    body = self._read_json()
                except (ValueError, json.JSONDecodeError) as error:
                    self._json({'accepted': False, 'message': str(error)},
                               HTTPStatus.BAD_REQUEST)
                    return
                revision = int(body.get('expected_revision', -1))
                allowed, reason = gateway._guard.authorize(
                    self._session_value(), revision,
                    gateway._state.orchestrator_revision(),
                    maintenance_required=path == '/api/debug',
                    operation_mode=gateway._state.operation_mode())
                if not allowed:
                    self._json({'accepted': False, 'message': reason},
                               HTTPStatus.CONFLICT)
                    return
                if path == '/api/control':
                    command = int(body.get('command', -1))
                    if command == 4 and body.get('confirmed') is not True:
                        accepted, message = False, '立即取消需要二次确认'
                    else:
                        accepted, message = gateway.call_control(
                            command, str(body.get('request_id', '')),
                            revision, str(body.get('reason', '')))
                elif path == '/api/debug':
                    action = str(body.get('action', ''))
                    high_risk = action in {
                        'pose_reset', 'recon_reset',
                        'approach_start', 'approach_contact'} or (
                            action == 'approach_arm' and
                            bool(body.get('armed', False)))
                    if high_risk and body.get('confirmed') is not True:
                        accepted, message = False, '该调试命令需要二次确认'
                    else:
                        accepted, message = gateway.call_debug(
                            action, bool(body.get('armed', False)))
                elif path == '/api/policy':
                    enabling = any(bool(body.get(key, False)) for key in (
                        'execution_enabled', 'grasp_enabled',
                        'tool_enabled'))
                    if enabling and body.get('confirmed') is not True:
                        accepted, message = False, '开启运动策略需要二次确认'
                    else:
                        accepted, message = gateway.call_policy(body)
                elif path == '/api/profiles/save':
                    try:
                        gateway._profiles.save(
                            str(body.get('name', '')),
                            body.get('values', {}))
                        accepted, message = True, '参数档案已保存'
                    except (OSError, TypeError, ValueError) as error:
                        accepted, message = False, str(error)
                elif path == '/api/profiles/load':
                    try:
                        values = gateway._profiles.load(str(body.get(
                            'name', '')))
                        accepted, message = True, '参数档案已加载，尚未应用'
                    except (OSError, TypeError, ValueError) as error:
                        accepted, message, values = False, str(error), {}
                else:
                    self._json({'accepted': False,
                                'message': '未知 API'},
                               HTTPStatus.NOT_FOUND)
                    return
                gateway._state.audit(path, accepted, message)
                response = {'accepted': accepted, 'message': message,
                            'revision': gateway._state.orchestrator_revision()}
                if path == '/api/profiles/load':
                    response['values'] = values
                self._json(response,
                           HTTPStatus.OK if accepted else HTTPStatus.CONFLICT)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        if not 1 <= port <= 65535:
            raise ValueError('port 必须在 1..65535')
        self._http = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(
            target=self._http.serve_forever,
            name='peach-perception-http', daemon=True)
        thread.start()
        shown_host = '127.0.0.1' if host == '0.0.0.0' else host
        self.get_logger().info(
            f'桃子感知 Web 控制台: http://{shown_host}:{port}')

    def destroy_node(self):
        """停止 HTTP 线程后销毁 ROS 节点."""
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        super().destroy_node()


def main(args=None):
    """运行 ROS 与 HTTP 网关."""
    rclpy.init(args=args)
    node = PeachPerceptionWeb()
    node.start_http()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
