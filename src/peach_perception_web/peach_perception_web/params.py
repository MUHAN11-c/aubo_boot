# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""
参数层 — GatewayParams：不可变参数快照（A9）.

与 peach_pose_ros2 / peach_reconstruction_ros2 的 params.py 同层：
默认值权威源为 ``config/web.yaml``（双向同步由 test/test_params.py 强制）；
declare 后立即集中装载为 frozen dataclass 并校验，非法值（端口越界、
周期/缓冲非正）抛 ValueError 整包拒绝启动；运行路径只持有快照引用，
不再逐回调 get_parameter 直读。

本模块只依赖标准库与鸭子类型 node（get_parameter(name).value），
不 import rclpy，可被无 ROS 上下文的单测直接装载。
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Tuple

# 参数默认值权威源（config/web.yaml 与之对齐）：name → (default, 中文说明)
DEFAULTS = {
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
        '/peach/reconstruction/diagnostics', '重建结构化诊断话题'),
    'reconstruction_diagnostics_debug_topic': (
        '/peach/reconstruction/diagnostics_debug',
        '重建调试明细 JSON 话题（镜像合并补充）'),
    'grasp_decision_topic': (
        '/peach/reconstruction/grasp_decision', '抓取许可话题'),
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
    'orchestrator_events_topic': (
        '/peach_harvest_orchestrator/events',
        '采摘编排器过程/审计事件话题（事件时间线）'),
    'robot_status_topic': (
        '/aubo_io_controller/robot_status',
        '机械臂上电/急停/运动/错误状态话题'),
    'event_buffer_size': (100, '事件时间线环形缓冲上限（条）'),
    'metrics_period_s': (1.0, '系统/GPU/进程性能采样周期（秒）'),
    'metrics_process_patterns': (
        [
            'peach_pose_node', 'peach_reconstruction_node',
            'peach_approach_grasp_node', 'peach_harvest_orchestrator',
            'ros2_control_node', 'percipio',
        ],
        '进程性能监控的 cmdline 子串匹配关键字列表'),
    'record.enabled': (True, '监控数据分类落盘总开关'),
    'record.root_dir': (
        'web_runs', '记录根目录（相对节点 CWD；按 run_id 分子目录）'),
    'record.save_images': (True, '关键事件时刻保存感知调试图 PNG'),
    'record.save_clouds': (True, 'target 终局时刻保存 TSDF 点云 PLY'),
    'debug_image_topic': (
        '/peach/perception/debug_image', '感知调试叠加图话题（bgr8）'),
    'tsdf_cloud_topic': (
        '/peach/reconstruction/tsdf_cloud', '在线 TSDF 点云话题'),
}

# 话题键（*_topic）集中进 topics 映射，供快照只读索引
TOPIC_NAMES = tuple(k for k in DEFAULTS if k.endswith('_topic'))


@dataclass(frozen=True)
class GatewayParams:
    """
    启动期静态装载的不可变参数快照（运行路径只持有本快照引用）.

    topics 以 MappingProxyType 包装保证不可变；
    metrics_process_patterns 固化为 tuple。
    """

    host: str
    port: int
    param_poll_period_s: float
    event_buffer_size: int
    metrics_period_s: float
    metrics_process_patterns: Tuple[str, ...]
    record_enabled: bool
    record_root_dir: str
    record_save_images: bool
    record_save_clouds: bool
    topics: Mapping[str, str]


def load_params(node) -> GatewayParams:
    """
    从 node 集中读取全部参数，校验后组装为 frozen 快照（declare 后立即调用）.

    Args:
        node: 鸭子类型节点（提供 get_parameter(name).value）.

    Returns
    -------
        GatewayParams（frozen；topics 为 MappingProxyType）.

    Raises
    ------
        ValueError: port 越界、event_buffer_size < 1、轮询/采样周期 ≤ 0.

    """
    g = node.get_parameter
    port = int(g('port').value)
    if not 1 <= port <= 65535:
        raise ValueError(f'port 必须在 1..65535，got {port}')
    event_buffer_size = int(g('event_buffer_size').value)
    if event_buffer_size < 1:
        raise ValueError(
            f'event_buffer_size 必须 >= 1，got {event_buffer_size}')
    param_poll_period_s = float(g('param_poll_period_s').value)
    if param_poll_period_s <= 0:
        raise ValueError(
            f'param_poll_period_s 必须 > 0，got {param_poll_period_s}')
    metrics_period_s = float(g('metrics_period_s').value)
    if metrics_period_s <= 0:
        raise ValueError(f'metrics_period_s 必须 > 0，got {metrics_period_s}')
    topics = MappingProxyType(
        {name: str(g(name).value).strip() for name in TOPIC_NAMES})
    return GatewayParams(
        host=str(g('host').value).strip(),
        port=port,
        param_poll_period_s=param_poll_period_s,
        event_buffer_size=event_buffer_size,
        metrics_period_s=metrics_period_s,
        metrics_process_patterns=tuple(
            str(item) for item in g('metrics_process_patterns').value),
        record_enabled=bool(g('record.enabled').value),
        record_root_dir=str(g('record.root_dir').value),
        record_save_images=bool(g('record.save_images').value),
        record_save_clouds=bool(g('record.save_clouds').value),
        topics=topics,
    )
