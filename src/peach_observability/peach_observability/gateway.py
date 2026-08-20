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
from aubo_msgs.msg import RobotStatus
from geometry_msgs.msg import Vector3Stamped
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    CanonicalEvent,
    GraspDecision,
    HarvestState,
    PeachTargetObservationArray,
    ReconstructionStatus,
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
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from . import http_server
from .codec import (
    candidate_array,
    fitting_array,
    grasp_decision,
    harvest_event,
    parse_json_text,
    reconstruction_status,
    robot_status,
    target_observations,
    vector_stamped,
)
from .metrics import MetricsSampler
from .params import DEFAULTS as _DEFAULTS
from .params import load_params as _load_params
from .recorder import Recorder
from .state import DashboardState


# 过程监测需要回答「当前以什么参数在跑」：按节点分组的只读参数白名单。
PARAM_WATCHLIST = {
    '/peach_scene_perception_node': [
        'yolo_conf', 'min_detection_conf',
        'target_memory.match_radius_m', 'target_memory.recovery_scale',
        'target_memory.confirm_frames',
        'harvest.min_collect_frames', 'harvest.lock_settle_frames',
        'harvest.max_collect_s', 'harvest.priority_prefer_lower_first',
    ],
    '/peach_target_reconstruction_node': [
        'capture.min_views', 'capture.recommended_views',
        'capture.min_mask_depth_ratio', 'capture.require_target_mask',
        'icp.enable', 'tsdf.enable',
    ],
    '/peach_manipulation_skills_node': [
        'moveit.velocity_scaling', 'moveit.acceleration_scaling',
        'moveit.transit_velocity_scaling', 'moveit.transit_acceleration_scaling',
        'scan.observation_radius_m', 'scan.minimum_radius_m',
        'scan.frame_wait_s', 'scan.maximum_moves',
        'quality.minimum_views', 'quality.minimum_baseline_deg',
        'quality.maximum_refined_rmse_m',
        'execution.enabled', 'grasp.enabled', 'tool.enabled',
    ],
    '/peach_task_executor': [
        'execution_enabled', 'survey_wait_s', 'empty_survey_limit',
        'persist_ledger',
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
        """声明参数、装载不可变快照、订阅话题并初始化共享缓存."""
        super().__init__('peach_observability')
        self._declare_parameters()
        # A9：declare 后立即集中装载+校验；非法值抛 ValueError 不进运行
        self._params = _load_params(self)
        self._state = DashboardState()
        self._http = None
        self._metrics = None
        # 重建镜像合并缓存：调试 JSON 明细（tsdf/registration/refined 等）与
        # 最近一次许可镜像——类型化诊断到达时并入，保持镜像/落盘信息不缩水
        self._recon_debug_extra: dict = {}
        self._recon_decision_value: dict | None = None
        self._recorder = Recorder(
            root_dir=self._params.record_root_dir,
            enabled=self._params.record_enabled,
            save_images=self._params.record_save_images,
            save_clouds=self._params.record_save_clouds,
            on_info=lambda info: self._state.update('record', 'info', info),
            log_warning=lambda msg: self.get_logger().warning(msg))
        self._create_subscriptions()
        self._create_param_watchers()
        self._create_metrics_sampler()

    def _declare_parameters(self) -> None:
        """集中声明 Web 和话题参数（默认值权威源为模块级 _DEFAULTS）."""
        for name, (default, description) in _DEFAULTS.items():
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description))

    def _topic(self, parameter: str) -> str:
        """从不可变快照取话题名（启动期建订阅用）."""
        return self._params.topics[parameter]

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
            self._harvest_callback, latched_qos)
        self.create_subscription(
            String, self._topic('reconstruction_status_topic'),
            self._recon_status_callback, latched_qos)
        self.create_subscription(
            ReconstructionStatus,
            self._topic('reconstruction_diagnostics_topic'),
            self._recon_diagnostics_callback, latched_qos)
        # 调试明细（tsdf/registration/overlap/refined/逐机位）：并入镜像与落盘，
        # 保持「类型化后过程数据不缩水」；类型化字段为准，明细键补充
        self.create_subscription(
            String, self._topic('reconstruction_diagnostics_debug_topic'),
            self._recon_debug_callback, latched_qos)
        self.create_subscription(
            GraspDecision, self._topic('grasp_decision_topic'),
            self._recon_decision_callback, latched_qos)
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
            self._approach_callback, latched_qos)
        self.create_subscription(
            HarvestState, self._topic('orchestrator_state_topic'),
            self._orchestrator_callback, latched_qos)
        self.create_subscription(
            CanonicalEvent, self._topic('orchestrator_events_topic'),
            self._events_callback,
            QoSProfile(
                depth=50,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            RobotStatus, self._topic('robot_status_topic'),
            lambda msg: self._state.update(
                'robot', 'status', robot_status(msg)), reliable_qos)
        # 记录器图像/点云订阅：只在对应开关开启时建立（省带宽）
        if self._params.record_enabled:
            if self._params.record_save_images:
                self.create_subscription(
                    Image, self._topic('debug_image_topic'),
                    self._recorder.handle_image, reliable_qos)
            if self._params.record_save_clouds:
                self.create_subscription(
                    PointCloud2, self._topic('tsdf_cloud_topic'),
                    self._recorder.handle_cloud, latched_qos)

    def _harvest_callback(self, message: String) -> None:
        """
        感知采摘计划：进状态缓存并喂记录器（perception.jsonl 并入）.

        JSON 全量透传：发布侧新增键（如阶段 D1 的 anchor_stale_target_ids/
        out_of_view_target_ids/dropped_target_ids/lighting/low_light_quality）
        无需本侧改动即进入 /api/state 镜像。
        """
        value = parse_json_text(message.data)
        self._state.update('perception', 'harvest', value)
        self._recorder.handle_harvest(value)

    def _recon_status_callback(self, message: String) -> None:
        """重建状态文本：进状态缓存并喂记录器（reconstruction.jsonl）."""
        value = parse_json_text(message.data, 'state')
        self._state.update('reconstruction', 'status', value)
        self._recorder.handle_reconstruction('status', value)

    def _recon_diagnostics_callback(
            self, message: ReconstructionStatus) -> None:
        """
        重建 1Hz 结构化诊断：消息字段重建镜像 dict 并喂记录器.

        合并调试 JSON 明细（tsdf/registration/refined 等）与最近许可镜像，
        保持镜像/落盘键集与旧裸 JSON 契约一致（reconstruction_final 摘要
        与前端 tsdf 计时依赖这些键）。
        """
        merged = dict(self._recon_debug_extra)
        merged.update(reconstruction_status(message))
        if self._recon_decision_value is not None:
            merged['grasp_decision'] = self._recon_decision_value
        self._state.update('reconstruction', 'diagnostics', merged)
        self._recorder.handle_reconstruction('diagnostics', merged)

    def _recon_debug_callback(self, message: String) -> None:
        """重建调试明细 JSON：只更新合并缓存，不直接落盘（防重复记录）."""
        self._recon_debug_extra = parse_json_text(message.data)

    def _recon_decision_callback(self, message: GraspDecision) -> None:
        """重建抓取许可：消息字段重建镜像 dict 并喂记录器."""
        value = grasp_decision(message)
        self._recon_decision_value = value
        self._state.update('reconstruction', 'grasp_decision', value)
        self._recorder.handle_reconstruction('grasp_decision', value)

    def _approach_callback(self, message: String) -> None:
        """靠近抓取状态：进状态缓存并喂记录器（approach.jsonl）."""
        value = parse_json_text(message.data)
        self._state.update('approach', 'status', value)
        self._recorder.handle_approach(value)

    def _events_callback(self, message: CanonicalEvent) -> None:
        """批次事件进环形缓冲，供前端事件时间线消费."""
        try:
            value = harvest_event(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'事件转换失败: {error}')
            return
        # 缓冲上限来自启动期快照（A9）：已校验 >= 1，运行期不再直读参数
        self._state.append_event(value, self._params.event_buffer_size)
        self._recorder.handle_event(value)

    def _orchestrator_callback(self, message: HarvestState) -> None:
        """把类型化业务状态转换为稳定的浏览器对象."""
        value = {
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
        }
        self._state.update('orchestration', 'state', value)
        self._recorder.handle_state(value)

    def _targets_callback(self, message) -> None:
        try:
            value = target_observations(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'目标快照转换失败: {error}')
            return
        self._state.update('perception', 'targets', value)
        self._recorder.handle_targets(value)

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
        self._param_timer = self.create_timer(
            self._params.param_poll_period_s, self._poll_params)

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
    # 性能采样：独立线程写 state，绝不占用 ROS 回调线程
    # ------------------------------------------------------------------
    def _create_metrics_sampler(self) -> None:
        """按快照参数建性能采样线程（GPU 不可用时自动降级为 None）."""
        self._metrics = MetricsSampler(
            self._params.metrics_period_s,
            list(self._params.metrics_process_patterns),
            self._metrics_callback,
            lambda msg: self.get_logger().warning(msg))

    def _metrics_callback(self, sample: dict) -> None:
        """性能采样落状态缓存并喂记录器（metrics.jsonl）."""
        self._state.update('metrics', 'sample', sample)
        self._recorder.handle_metrics(sample)

    # ------------------------------------------------------------------
    # HttpBackend 窄接口实现（http_server 只依赖这两个方法）
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """浏览器状态快照（GET /api/state 的载荷）."""
        return self._state.snapshot()

    def start_http(self) -> None:
        """按快照参数提示非回环风险并启动 HTTP 服务线程."""
        host = self._params.host
        port = self._params.port
        if host not in ('127.0.0.1', 'localhost'):
            self.get_logger().warning(
                f'*** 安全提示：Web 监控台监听在非回环地址 {host}:{port}，'
                '该 HTTP 服务无鉴权，严禁暴露到公网或不受信网络 ***')
        web_root = Path(get_package_share_directory(
            'peach_observability')) / 'web'
        self._http = http_server.start_http(
            host, port, web_root, self, self.get_logger().debug)
        if self._metrics is not None:
            self._metrics.start()
        shown_host = '127.0.0.1' if host == '0.0.0.0' else host
        self.get_logger().info(
            f'桃子采摘监控台（只读）: http://{shown_host}:{port}')

    def destroy_node(self):
        """停止 HTTP/性能采样/记录器后销毁 ROS 节点."""
        if self._metrics is not None:
            self._metrics.stop()
        if self._recorder is not None:
            self._recorder.close()
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
