# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""
桃子采摘链路只读 HTTP 监控网关（无控制/调试写入口）.

设计约定（2026-08-13 起）：测试与运行一律走自动全流程，本节点只回答
「现在跑到哪一步、各步数据是什么、当前参数是什么」，
问题定位依靠过程监测（阶段线 + 过程数据 + 参数镜像）。
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
from peach_harvest_msgs.msg import HarvestState
from peach_pose_msgs.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    PeachTargetObservationArray,
)
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from . import http_server
from .codec import (
    candidate_array,
    fitting_array,
    parse_json_text,
    target_observations,
    vector_stamped,
)
from .state import DashboardState


# 过程监测需要回答「当前以什么参数在跑」：按节点分组的只读参数白名单。
PARAM_WATCHLIST = {
    '/peach_pose_node': [
        'yolo_conf', 'min_detection_conf',
        'target_memory.match_radius_m', 'target_memory.recovery_scale',
        'target_memory.confirm_frames',
        'harvest.min_collect_frames', 'harvest.lock_settle_frames',
        'harvest.max_collect_s', 'harvest.priority_prefer_lower_first',
    ],
    '/peach_reconstruction_node': [
        'capture.min_views', 'capture.recommended_views',
        'capture.min_mask_depth_ratio', 'capture.require_target_mask',
        'icp.enable', 'tsdf.enable',
    ],
    '/peach_approach_grasp_node': [
        'moveit.velocity_scaling', 'moveit.acceleration_scaling',
        'moveit.transit_velocity_scaling', 'moveit.transit_acceleration_scaling',
        'scan.observation_radius_m', 'scan.minimum_radius_m',
        'scan.frame_wait_s', 'scan.maximum_moves',
        'quality.minimum_views', 'quality.minimum_baseline_deg',
        'quality.maximum_refined_rmse_m',
        'execution.enabled', 'grasp.enabled', 'tool.enabled',
    ],
    '/peach_harvest_orchestrator': [
        'auto_start_enabled', 'execution_enabled', 'grasp_enabled',
        'tool_enabled', 'harvest.max_rounds', 'harvest.rescan_until_empty',
        'photo_pose.enabled', 'readiness.timeout_s',
    ],
}


def _parameter_scalar(value) -> object:
    """rcl_interfaces/ParameterValue → 标量（数组取列表，未设置给 None）."""
    kind = value.type
    if kind == 1:
        return bool(value.bool_value)
    if kind == 2:
        return int(value.integer_value)
    if kind == 3:
        return float(value.double_value)
    if kind == 4:
        return str(value.string_value)
    if kind == 6:
        return [bool(item) for item in value.bool_array_value]
    if kind == 7:
        return [int(item) for item in value.integer_array_value]
    if kind == 8:
        return [float(item) for item in value.double_array_value]
    if kind == 9:
        return [str(item) for item in value.string_array_value]
    return None


class PeachPerceptionWeb(Node):
    """订阅采摘链路各阶段输出并提供只读 HTTP 监控 API."""

    def __init__(self):
        """声明参数、订阅话题并初始化共享缓存."""
        super().__init__('peach_perception_web')
        self._declare_parameters()
        self._state = DashboardState()
        self._http = None
        self._create_subscriptions()
        self._create_param_watchers()

    def _declare_parameters(self) -> None:
        """集中声明 Web 和话题参数."""
        parameters = {
            'host': ('127.0.0.1', 'HTTP 监听地址；局域网访问显式设 0.0.0.0'),
            'port': (8090, 'HTTP 监听端口'),
            'param_poll_period_s': (3.0, '各节点参数镜像轮询周期（秒）'),
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
        """把类型化业务状态转换为稳定的浏览器对象."""
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

    # ------------------------------------------------------------------
    # 参数镜像：周期轮询白名单节点的 get_parameters，只读不写
    # ------------------------------------------------------------------
    def _create_param_watchers(self) -> None:
        """为白名单节点建 get_parameters 客户端并启动轮询定时器."""
        self._param_clients = {
            name: self.create_client(GetParameters, f'{name}/get_parameters')
            for name in PARAM_WATCHLIST
        }
        # 服务未就绪时静默跳过（节点可能未启动），不刷错误日志
        self._param_inflight = set()
        period = float(self.get_parameter('param_poll_period_s').value)
        self._param_timer = self.create_timer(period, self._poll_params)

    def _poll_params(self) -> None:
        """对就绪的参数服务发起异步查询（在途请求去重）."""
        for name, client in self._param_clients.items():
            if name in self._param_inflight or not client.service_is_ready():
                continue
            request = GetParameters.Request()
            request.names = list(PARAM_WATCHLIST[name])
            self._param_inflight.add(name)
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, node_name=name: self._on_params(node_name, fut))

    def _on_params(self, node_name: str, future) -> None:
        """落参数镜像到状态缓存；异常仅降级为空镜像."""
        self._param_inflight.discard(node_name)
        try:
            response = future.result()
        except (RuntimeError, rclpy.exceptions.RCLError):
            return
        if response is None:
            return
        values = {
            name: _parameter_scalar(value)
            for name, value in zip(
                PARAM_WATCHLIST[node_name], response.values)
        }
        self._state.update_params(node_name, values)

    # ------------------------------------------------------------------
    # HttpBackend 窄接口实现（http_server 只依赖这两个方法）
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """浏览器状态快照（GET /api/state 的载荷）."""
        return self._state.snapshot()

    def start_http(self) -> None:
        """校验监听参数、提示非回环风险并启动 HTTP 服务线程."""
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        if not 1 <= port <= 65535:
            raise ValueError('port 必须在 1..65535')
        if host not in ('127.0.0.1', 'localhost'):
            self.get_logger().warning(
                f'*** 安全提示：Web 监控台监听在非回环地址 {host}:{port}，'
                '该 HTTP 服务无鉴权，严禁暴露到公网或不受信网络 ***')
        web_root = Path(get_package_share_directory(
            'peach_perception_web')) / 'web'
        self._http = http_server.start_http(
            host, port, web_root, self, self.get_logger().debug)
        shown_host = '127.0.0.1' if host == '0.0.0.0' else host
        self.get_logger().info(
            f'桃子采摘监控台（只读）: http://{shown_host}:{port}')

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
