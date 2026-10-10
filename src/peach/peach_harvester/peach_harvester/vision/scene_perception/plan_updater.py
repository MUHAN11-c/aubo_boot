"""
计划更新纯核（W3 自 scene_perception_node 下沉）：帧级计划/身份/光照推进.

本模块零 ROS msg import——只产 dict / token / ndarray。ROS 消息组装
（msg_builders）、publish、save_mask 回调与 plan_lock 持有留在节点；
调用约定：PlanUpdater.update 与 harvest_state_dict 均须在节点
plan_lock 持有区调用（锁协议与原节点实现一致，RLock 可重入）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import List, Optional

import numpy as np

from peach_harvester.vision.scene_perception.identity import (
    classify_tracking_status,
    memory_anchor_fields,
    STATUS_DEPTH_VOID,
    STATUS_OUT_OF_VIEW,
)


@dataclass
class TargetObservationSpec:
    """单目标观测组装材料（节点据此填 PeachTargetObservation，零 msg）."""

    target_id: str
    """目标 ID."""
    priority: int
    """计划优先级."""
    selected: bool
    """是否当前选中目标."""
    harvest_status: str
    """计划 harvest_status token（executor 覆盖口径由节点传入）."""
    tracking_token: str
    """identity.classify_tracking_status token（节点查表映射枚举）."""
    payload: Optional[dict]
    """candidate / candidate_2d / fitting / mask / mask_depth_ratio（原样透传）."""
    camera_distance_m: float
    """本帧记录的相机距离 [m]（payload 缺失为 0）."""
    confidence: float
    """本帧记录的置信度（payload 缺失为 0）."""
    diagnostic_flags: List[str]
    """诊断旗标（含 target_out_of_view / mask_unavailable / anchor_stale 等）."""
    mask: Optional[np.ndarray]
    """payload 掩膜（None=无）；节点据此发 imgmsg + save_mask."""
    anchor_fields: Optional[dict]
    """记忆回填字段（identity.memory_anchor_fields；None=不回填）."""


@dataclass
class PlanUpdateOutcome:
    """一帧计划推进结果；节点只做消息组装与 IO，不再内嵌业务."""

    snapshot_id: str
    """计划快照 ID."""
    target_set_locked: bool
    """锁定集是否已锁定."""
    target_count: int
    """锁定集大小."""
    selected_target_id: str
    """本帧选中目标（executor 覆盖口径）."""
    collecting_count: int
    """发现进度：收齐窗口累积已确认数（R-D8）."""
    pending_count: int
    """发现进度：注册表确认中数."""
    stamp_ns: int
    """掩膜 stamp（纳秒，frame 事件与 save_mask 共用）."""
    observations: List[TargetObservationSpec] = field(default_factory=list)
    """逐锁定目标的组装材料（顺序=locked_ids）."""
    observed_ids: List[str] = field(default_factory=list)
    """本帧带掩膜观测的目标 ID（frame 事件用）."""
    dropped_events: List[dict] = field(default_factory=list)
    """锚点超龄移除的 target_dropped 记账事件（节点写 harvest_data）."""
    locked_just_now: bool = False
    """本帧刚锁定 → 节点调 _start_harvest_run 建轮目录."""
    lighting_warning: Optional[str] = None
    """光照低质告警文案（节点带节流打日志；None=不打）."""
    frame_event: Optional[dict] = None
    """frame_observations 记账事件（未锁定为 None）."""


def degenerate_candidate(candidate) -> bool:
    """袋底或袋颈近原点，视为本帧几何失败（duck-typed：读 x/y/z 字段）."""
    bottom, neck = candidate.bag_bottom, candidate.bag_neck
    origin = (abs(bottom.x) < 1e-6 and abs(bottom.y) < 1e-6
              and abs(bottom.z) < 1e-6)
    neck_origin = (abs(neck.x) < 1e-6 and abs(neck.y) < 1e-6
                   and abs(neck.z) < 1e-6)
    return origin or neck_origin


def _empty_candidate():
    """全零默认 candidate 替身：payload 缺失时 degenerate 判定的输入."""
    zero = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(bag_bottom=zero, bag_neck=zero)


def discovery_counts(pipeline) -> tuple:
    """
    发现进度摘要 (collecting_count, pending_count)（缺陷 R-D8；须持锁）.

    collecting_count：锁定前=收齐窗口累积的已确认目标数（plan 透传策略
    累积集大小），锁定后=锁定集大小（与 target_count 一致）；
    pending_count：锁定前=注册表确认中（未转正）记录数，锁定后恒 0；
    身份记忆禁用时恒 0（每帧记录即确认，无攒帧过程）。
    """
    collecting = pipeline.harvest_plan.collecting_count
    if pipeline.harvest_plan.locked or pipeline.target_registry is None:
        return collecting, 0
    return collecting, pipeline.target_registry.pending_count


def harvest_state_dict(pipeline, harvest_run_id: str, scene_epoch: int,
                       selected_id: str, data_query: dict) -> dict:
    """
    全局采摘计划与数据路径的 JSON 快照（原节点 _harvest_state_dict，须持锁）.

    Args:
        pipeline: PerceptionPipeline（plan/lighting/timing/frame_rate）.
        harvest_run_id: 当前轮 ID（节点持有）.
        scene_epoch: BeginScene 计数（节点持有）.
        selected_id: 生效选中目标（executor 覆盖口径，节点算好传入）.
        data_query: HarvestDataStore.query() 结果（节点持有 IO 对象）.

    Returns
    -------
        可 json 序列化的状态 dict（键集与 W3 前一致，零增删）.

    """
    collecting_count, pending_count = discovery_counts(pipeline)
    return {
        'harvest_run_id': harvest_run_id,
        'snapshot_id': pipeline.harvest_plan.snapshot_id,
        'target_set_locked': pipeline.harvest_plan.locked,
        'target_count': pipeline.harvest_plan.target_count,
        # R-D8 发现进度摘要：与 target_observations 同名字段对齐
        'collecting_count': collecting_count,
        'pending_count': pending_count,
        'target_ids': list(pipeline.harvest_plan.locked_ids),
        'completed_target_ids': sorted(pipeline.harvest_plan.completed_ids),
        'priorities': dict(pipeline.harvest_plan.priorities),
        'selected_target_id': selected_id,
        # 阶段 D1（协议 2.4）：锚点陈旧/出视野/已移除目标集（均为
        # 纯增量键，下游只读消费）与光照质量观测指标
        'anchor_stale_target_ids': sorted(
            pipeline.harvest_plan.anchor_stale_ids),
        'out_of_view_target_ids': sorted(
            pipeline.harvest_plan.out_of_view_ids),
        'dropped_target_ids': sorted(pipeline.harvest_plan.dropped_ids),
        'lighting': pipeline.lighting.snapshot(),
        'low_light_quality': pipeline.lighting.low_quality,
        'scene_epoch': scene_epoch,
        # 推理耗时分项 EMA（毫秒）+ 实测 fps；详见 TimingMetrics
        'timing': pipeline.timing.snapshot(fps=pipeline.frame_rate.rate_hz),
        'data': data_query,
    }


class PlanUpdater:
    """
    帧级计划推进器：驱动 pipeline 上的 plan/registry/lighting/bbox_at_edge.

    原 scene_perception_node._publish_target_observations 的纯逻辑段
    （帧率自适应窗伸缩 / OUT_OF_VIEW 预分类 / plan.update+drop 记账 /
    光照统计 / 逐目标 token 分类与组装材料 / 记忆回填字段），逐句搬移、
    行为零变化。节点保留锁持有与 ROS/IO。
    """

    def __init__(self, pipeline, params, clock, log=None):
        """
        装配推进器.

        Args:
            pipeline: PerceptionPipeline（推进对象都在其上）.
            params: ScenePerceptionParams 快照（逐帧键热读）.
            clock: 算法时钟适配器（协议 I3：now 由注入时钟给出）.
            log: 可选 logger（std logging 兼容；缺省静默）.

        Returns
        -------
            无返回值（None）.

        """
        self.pipeline = pipeline
        self.params = params
        self.clock = clock
        self._log = log

    def update(self, records, payloads, stamp_ns: int, selected_id: str,
               executor_id: Optional[str]) -> PlanUpdateOutcome:
        """
        推进一帧计划并产出观测组装材料（须持 plan_lock 调用）.

        Args:
            records: 本帧身份记录（pipeline.process 的 harvest_records）.
            payloads: target_id → {candidate, candidate_2d, fitting,
                mask, mask_depth_ratio}（含 ROS msg 对象，本模块只透传）.
            stamp_ns: 掩膜 stamp（纳秒）.
            selected_id: 生效选中目标（executor 覆盖口径）.
            executor_id: 调度当前目标 ID（harvest_status 覆盖用；None=
                未见执行器状态）.

        Returns
        -------
            PlanUpdateOutcome；锁内完成的全部状态变更即时生效.

        """
        pipeline = self.pipeline
        params = self.params
        plan = pipeline.harvest_plan
        was_locked = plan.locked
        # 帧率自适应收齐兜底：按实测帧间隔 EMA 伸缩 max_collect_s（帧率
        # 以运行状态为准）——低帧率放大防误锁空集，高帧率收紧提速；
        # 配置值 ×0.4 作下限。异常间隔（暂停后首帧）不进 EMA。
        # 协议 I3：now 取节点时钟（同一时钟源同时驱动窗口超时判定）
        now_s = self.clock.now()
        pipeline.frame_rate.update(now_s)
        frame_interval = pipeline.frame_rate.interval
        if frame_interval is not None:
            plan.max_collect_s = (
                pipeline.collect_window_timeout.value(frame_interval))
            # 锚点新鲜度两档时限（阶段 D1，协议 2.4/I4）：秒级上限 ÷
            # 实测帧间隔 EMA = 帧数阈值，逐帧改写；帧率跌落时帧数变少，
            # 墙钟上限保持不变
            plan.anchor_max_age_frames = max(
                1, round(params.target_memory.anchor_max_age_s
                         / frame_interval))
            plan.anchor_drop_frames = max(
                1, round(params.target_memory.anchor_drop_s
                         / frame_interval))
        # OUT_OF_VIEW 预分类（须在 plan.update 前完成：计划按本集合做
        # 不可选/去选判定）：锁定目标本帧无观测且消失前最后检测框触
        # 图像边缘 → 走出视野；单位姿模型下复扫无益，视为不可选（2.4）
        out_of_view_ids = {
            target_id for target_id in plan.locked_ids
            if target_id not in payloads
            and pipeline.bbox_at_edge.get(target_id, False)
        }
        current = plan.update(records, now=now_s,
                              out_of_view_ids=out_of_view_ids)
        # 锚点超龄移除（LOST 超 anchor_drop）：记账 target_dropped 事件，
        # 编排侧据此按 SKIPPED_UNREACHABLE「目标丢失超时」入账（协议 2.4）
        dropped_events = []
        for dropped_id in plan.pop_dropped():
            dropped_events.append({
                'source': 'perception', 'event': 'target_dropped',
                'target_id': dropped_id,
                'reason': 'anchor_drop_timeout（目标丢失超时）',
            })
        locked_just_now = bool(plan.locked and not was_locked)

        # 光照质量统计（阶段 D1；观测指标，不打阻断旗标）：锁定集中
        # 本帧带掩膜观测的目标，逐帧注入掩膜内有效深度占比与置信度
        if plan.locked:
            depth_ratios = []
            confidences = []
            for target_id in plan.locked_ids:
                payload = payloads.get(target_id)
                if payload is None or payload.get('mask') is None:
                    continue
                depth_ratios.append(payload.get('mask_depth_ratio', 0.0))
                confidences.append(float(
                    current.get(target_id, {}).get('confidence', 0.0)))
            pipeline.lighting.update(depth_ratios, confidences)
        lighting_warning = None
        if pipeline.lighting.low_quality:
            lighting_warning = (
                f'光照质量持续偏低（{params.lighting.bad_frames} 帧连击：'
                f'掩膜内有效深度占比 EMA='
                f'{pipeline.lighting.snapshot()["depth_ratio"]} < '
                f'{params.lighting.min_depth_ratio} 或置信度 EMA < '
                f'{params.lighting.min_conf_mean}），建议现场补光/调曝光')

        collecting_count, pending_count = discovery_counts(pipeline)
        observations: List[TargetObservationSpec] = []
        observed_ids: List[str] = []
        anchor_stale_ids = plan.anchor_stale_ids
        for target_id in plan.locked_ids:
            payload = payloads.get(target_id)
            record = current.get(target_id, {})
            # 跟踪状态四分类（阶段 D1，协议 2.4 第 4 条）：分类纯函数
            # 在 assignment.classify_tracking_status，本处只产 token
            token = classify_tracking_status(
                has_observation=payload is not None,
                has_mask=(payload is not None
                          and payload.get('mask') is not None),
                mask_depth_ratio=(
                    None if payload is None
                    else payload.get('mask_depth_ratio')),
                min_depth_ratio=params.lighting.min_depth_ratio,
                last_bbox_touched_edge=pipeline.bbox_at_edge.get(
                    target_id, False))
            flags: List[str] = []
            mask = None
            camera_distance_m = 0.0
            confidence = 0.0
            if payload is None:
                flags.append('target_out_of_view' if token == STATUS_OUT_OF_VIEW
                             else 'target_temporarily_lost')
            else:
                camera_distance_m = float(record.get('camera_distance_m', 0.0))
                confidence = float(record.get('confidence', 0.0))
                flags.extend(record.get('diagnostic_flags', ()))
                mask = payload.get('mask')
                if mask is None:
                    flags.append('mask_unavailable')
                else:
                    if token == STATUS_DEPTH_VOID:
                        flags.append('depth_void')
                    observed_ids.append(target_id)
            # 锚点陈旧旗标（阶段 D1，协议 2.4）：LOST 超 anchor_max_age
            # 的锁定目标已被计划排除出可选集，旗标同步进诊断供下游展示
            if target_id in anchor_stale_ids and 'anchor_stale' not in flags:
                flags.append('anchor_stale')
            # 几何退化且本帧无活体观测：用身份表记忆锚点回填。
            # 已有检测/掩膜时不要回填，否则能力端把 OBSERVED 当成非新鲜
            # （anchor_from_memory 不刷新 received_s），FULL 再确认空等。
            anchor_fields = None
            if payload is None and degenerate_candidate(_empty_candidate()):
                anchor_fields = memory_anchor_fields(
                    None if pipeline.target_registry is None
                    else pipeline.target_registry.get(target_id),
                    params.tool.entry_standoff)
            observations.append(TargetObservationSpec(
                target_id=target_id,
                priority=plan.priority(target_id),
                selected=(target_id == selected_id),
                harvest_status=plan.harvest_status(
                    target_id, executor_id=executor_id),
                tracking_token=token,
                payload=payload,
                camera_distance_m=camera_distance_m,
                confidence=confidence,
                diagnostic_flags=flags,
                mask=mask,
                anchor_fields=anchor_fields))

        frame_event = None
        if plan.locked:
            frame_event = {
                'source': 'perception', 'event': 'frame_observations',
                'stamp_ns': stamp_ns, 'observed_target_ids': observed_ids,
                'selected_target_id': selected_id,
            }
        return PlanUpdateOutcome(
            snapshot_id=plan.snapshot_id,
            target_set_locked=plan.locked,
            target_count=plan.target_count,
            selected_target_id=selected_id,
            collecting_count=collecting_count,
            pending_count=pending_count,
            stamp_ns=stamp_ns,
            observations=observations,
            observed_ids=observed_ids,
            dropped_events=dropped_events,
            locked_just_now=locked_just_now,
            lighting_warning=lighting_warning,
            frame_event=frame_event)
