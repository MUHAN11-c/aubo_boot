from __future__ import annotations
"""世界系身份、锁定窗、记忆锚点。"""

from dataclasses import dataclass
import math
import time
from typing import (
    Dict,
    List,
    Optional,
    Sequence,
    Tuple,
)

import numpy as np

from .assignment import assign_detections
from .interfaces import (
    compute_entry_start,
    LOCK_POLICIES,
    LockEvent,
    LockPolicy,
    MATCHERS,
    MatchResult,
    TargetMatcher,
)
from .pose_pipelines import grasp_frame_from_axis


# === harvest_plan.py ===

_BLOCKING_FLAGS = frozenset({
    'tf_stale', 'tf_unavailable', 'target_untracked',
    # 阶段 D1（协议 2.4）：摆动目标视为不可选——观测残差连续超
    # wind.swing_threshold_m 的锁定目标由注册表置 swinging、节点打入
    # record 旗标；室外风动场景下靠近抓取由能力端 RECONFIRM 等平息
    'target_swinging',
})
_STATUS_REJECT = 2


def _finite_or(value: float, fallback: float) -> float:
    """非有限浮点（nan/inf）回退为 fallback，供排序键防御脏数据."""
    return value if math.isfinite(value) else fallback


def _selectable(record) -> bool:
    """目标是否可作为靠近重建对象."""
    flags = set(record.get('diagnostic_flags', ()))
    return (
        bool(record.get('target_id'))
        and bool(record.get('confirmed'))
        and int(record.get('status', _STATUS_REJECT)) != _STATUS_REJECT
        and not flags & _BLOCKING_FLAGS
    )


class CollectLockPolicy(LockPolicy):
    """
    收齐窗口锁定策略（LockPolicy 默认实现，注册名 'collect_lock'）.

    逐帧累积确认目标（同 ID 后者覆盖，窗口关闭时取最新一帧的质量量）；
    关闭条件二选一：静止关闭（累积帧数 ≥ min_collect_frames 且连续
    lock_settle_frames 帧无新增确认 ID，且当前帧无未确认记录在攒帧）
    或超时兜底（now − 窗口起点 ≥ max_collect_s，空集也关闭）。关闭后
    发一次 LockEvent 即冻结，reset() 后重新开窗。

    生命周期：与 GlobalHarvestPlan 同寿，由节点按 lock.impl 创建注入。
    线程安全：无内部锁，与 plan 同一把外部锁保护（见模块 docstring）。
    可替换性：实现 LockPolicy 即可经 LOCK_POLICIES 注册表替换。
    """

    def __init__(self, min_collect_frames: int = 10,
                 lock_settle_frames: int = 5, max_collect_s: float = 25.0):
        """建未关闭的收齐窗口（帧数≥1/静止帧数≥0/时长>0 校验）."""
        if min_collect_frames < 1:
            raise ValueError('min_collect_frames 须 ≥ 1')
        if lock_settle_frames < 0:
            raise ValueError('lock_settle_frames 须 ≥ 0')
        if max_collect_s <= 0.0:
            raise ValueError('max_collect_s 须 > 0')
        self.min_collect_frames = int(min_collect_frames)
        self.lock_settle_frames = int(lock_settle_frames)
        self.max_collect_s = float(max_collect_s)
        self._accumulated: Dict[str, dict] = {}
        self._collect_frames = 0        # 窗口内已累积帧数（update 调用次数）
        self._last_new_id_frame = 0     # 最近一次出现新增确认 ID 的帧序号
        self._window_start: Optional[float] = None  # 首帧时间戳（懒启动）
        self._closed = False

    @property
    def accumulated_count(self) -> int:
        """收齐窗口累积集大小（已确认目标数；R-D8 发现进度摘要，只读）."""
        return len(self._accumulated)

    def update(self, records: List[dict], now: float) -> Optional[LockEvent]:
        """
        累积本帧确认记录；窗口关闭时返回 LockEvent（仅一次），否则 None.

        Args:
            records: 本帧候选 record dict 列表（须带 target_id；确认判定
                只看 confirmed 键）.
            now: 当前时刻 (s)，调用方注入（协议 I3）；窗口内各帧须同一
                时钟基准.

        Returns
        -------
            LockEvent（records 为累积确认记录快照，每 ID 取最新一帧）；
            窗口未关闭或已关闭过返回 None.

        """
        if self._closed:
            return None
        ts = float(now)
        if self._window_start is None:
            self._window_start = ts
        self._collect_frames += 1
        # 确认目标（含 REOBSERVE，不限于 selectable）并入累积集；
        # 同 ID 后者覆盖，窗口关闭时取最新一帧的质量量
        for record in records:
            if not record.get('confirmed'):
                continue
            target_id = str(record.get('target_id'))
            if target_id not in self._accumulated:
                self._last_new_id_frame = self._collect_frames
            self._accumulated[target_id] = record
        # 静止关闭追加前提"无进行中确认"：当前帧仍有未确认记录（确认
        # 进度攒帧中）时不得关窗，否则低置信/闪烁场景会在确认完成前
        # 锁定空集（2026-08-14 真机：0.3 阈值下检测迟到，窗口提前锁定空集）。
        has_pending_confirmation = any(
            not record.get('confirmed') for record in records)
        settled = (
            self._collect_frames >= self.min_collect_frames
            and self._collect_frames - self._last_new_id_frame
            >= self.lock_settle_frames
            and not has_pending_confirmation)
        timed_out = ts - self._window_start >= self.max_collect_s
        if settled or timed_out:
            self._closed = True
            return LockEvent(records=tuple(self._accumulated.values()))
        return None

    def reset(self) -> None:
        """清空累积集与窗口计时，重新进入收齐窗口."""
        self._accumulated.clear()
        self._collect_frames = 0
        self._last_new_id_frame = 0
        self._window_start = None
        self._closed = False


class GlobalHarvestPlan:
    """
    收齐窗口关闭后一次性锁定全局目标集合，锁定后按固定优先级推进.

    构造参数：max_targets 为锁定目标数量上限（≥1），超出按排序键截断；
    prefer_lower_first 决定排序是否启用高度键（先低后高），False 时高度
    不参与排序；lock_policy 为收齐窗口策略（None 时按
    min_collect_frames / lock_settle_frames / max_collect_s 构造默认
    CollectLockPolicy——后三个参数仅在该路径生效）。
    anchor_max_age_frames / anchor_drop_frames（阶段 D1，协议 2.4）为锁定
    目标 LOST 帧龄的两档阈值（帧，≥1）：超 anchor_max_age → 入
    anchor_stale_ids（打 anchor_stale 旗标、视为不可选但不移除）；超
    anchor_drop → 从计划移除（记 dropped_ids，selected 顺延）。构造默认
    值是 5 fps 名义帧率下 30 s / 120 s 的折算兜底；运行期节点按秒级配置
    ÷ 实测帧间隔 EMA 逐帧改写（协议 I4，帧率以运行状态为准），同名
    property setter 可写。
    """

    def __init__(self, max_targets: int = 20, min_collect_frames: int = 10,
                 lock_settle_frames: int = 5, max_collect_s: float = 25.0,
                 prefer_lower_first: bool = True,
                 anchor_max_age_frames: int = 150,
                 anchor_drop_frames: int = 600,
                 lock_policy: Optional[LockPolicy] = None):
        """建未锁定计划并进入收齐窗口（容量≥1 校验；策略可注入替换）."""
        if max_targets < 1:
            raise ValueError('max_targets 须 ≥ 1')
        self.max_targets = int(max_targets)
        self.prefer_lower_first = bool(prefer_lower_first)
        self._lock_policy = lock_policy or CollectLockPolicy(
            min_collect_frames=min_collect_frames,
            lock_settle_frames=lock_settle_frames,
            max_collect_s=max_collect_s)
        # 锚点帧龄阈值（setter 内含 ≥1 校验；节点逐帧按帧率 EMA 改写）
        self.anchor_max_age_frames = anchor_max_age_frames
        self.anchor_drop_frames = anchor_drop_frames
        self.snapshot_id = 0
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids = set()
        self.current_selectable_ids = set()
        self._locked = False
        # ---- 阶段 D1：锚点新鲜度/出视野/移除记账 ----
        self._frame_index = 0            # update 调用帧计数（LOST 帧龄判定）
        self._last_seen_frame = {}       # target_id → 最近出现在记录的帧序号
        self.anchor_stale_ids = set()    # LOST 超 anchor_max_age：不可选不移除
        self.out_of_view_ids = set()     # 本帧 OUT_OF_VIEW 的锁定 ID（节点注入）
        self.dropped_ids = set()         # 累计被移除 ID（LOST 超 anchor_drop）
        self._dropped_queue = []         # 待节点取走的移除事件（pop_dropped）

    # 窗口参数透传到底层策略（节点帧率自适应读写 max_collect_s 等）
    @property
    def min_collect_frames(self) -> int:
        """收齐窗口最少累积帧数（透传自锁定策略）."""
        return self._lock_policy.min_collect_frames

    @property
    def lock_settle_frames(self) -> int:
        """连续无新增确认 ID 的静止判定帧数（透传自锁定策略）."""
        return self._lock_policy.lock_settle_frames

    @property
    def max_collect_s(self) -> float:
        """收齐窗口最长时长 (s)（透传自锁定策略，可写）."""
        return self._lock_policy.max_collect_s

    @max_collect_s.setter
    def max_collect_s(self, value: float) -> None:
        self._lock_policy.max_collect_s = float(value)

    # 锚点帧龄阈值（节点按秒级配置 ÷ 实测帧间隔 EMA 逐帧改写，协议 I4）
    @property
    def anchor_max_age_frames(self) -> int:
        """LOST 帧龄打 anchor_stale 的阈值（帧，可写；≥1）."""
        return self._anchor_max_age_frames

    @anchor_max_age_frames.setter
    def anchor_max_age_frames(self, value: int) -> None:
        if int(value) < 1:
            raise ValueError(f'anchor_max_age_frames 须 ≥ 1，got {value}')
        self._anchor_max_age_frames = int(value)

    @property
    def anchor_drop_frames(self) -> int:
        """LOST 帧龄从计划移除的阈值（帧，可写；≥1 且应 > anchor_max_age）."""
        return self._anchor_drop_frames

    @anchor_drop_frames.setter
    def anchor_drop_frames(self, value: int) -> None:
        if int(value) < 1:
            raise ValueError(f'anchor_drop_frames 须 ≥ 1，got {value}')
        self._anchor_drop_frames = int(value)

    @property
    def locked(self) -> bool:
        """是否已锁定全局目标集合（与 locked_ids 解耦：空集也算锁定）."""
        return self._locked

    @property
    def target_count(self) -> int:
        """锁定目标数量."""
        return len(self.locked_ids)

    @property
    def collecting_count(self) -> int:
        """
        累积已确认目标数（缺陷 R-D8 发现进度摘要）.

        锁定前透传锁定策略的收齐窗口累积集大小（随窗口攒帧增长）；
        锁定后返回锁定集大小（与 target_count 一致）——窗口关闭后策略
        累积集冻结不再更新，不能继续透传，故切换语义。
        """
        if self._locked:
            return len(self.locked_ids)
        return int(self._lock_policy.accumulated_count)

    def _rank_key(self, record):
        """
        生成多维确定性排序键（升序排最优先）.

        键序设计依据：
          1. status 升序——先 ACCEPT(0) 后 REOBSERVE(1)，REJECT(2) 垫底；
          2. camera_distance_m——先近后远：先清外围目标，减少深入冠层时
             碰枝与自遮挡；
          3. base_height_m（仅 prefer_lower_first）——先低后高：避免摘高处
             目标时碰落低处果实（Xiong 草莓采摘机实证顺序）；
          4. -confidence——同距同高时置信度高者优先；
          5. target_id——字符串确定性 tie-break，保证跨帧排序稳定。
        record 缺键时按最差值兜底（距离/高度 inf、置信度 0、REJECT）。
        """
        status = int(record.get('status', _STATUS_REJECT))
        distance = _finite_or(
            float(record.get('camera_distance_m', math.inf)), math.inf)
        if distance <= 0.0:
            distance = math.inf
        confidence = _finite_or(float(record.get('confidence', 0.0)), 0.0)
        key = [status, distance]
        if self.prefer_lower_first:
            key.append(_finite_or(
                float(record.get('base_height_m', math.inf)), math.inf))
        key.extend([-confidence, str(record.get('target_id', ''))])
        return tuple(key)

    def update(self, records, now: float, out_of_view_ids=None):
        """
        输入当前帧记录；未锁定期间驱动窗口策略，窗口关闭时一次性锁定.

        Args:
            records: 本帧候选 record dict 列表（含 confirmed/status/距离等）.
            now: 当前时刻 (s)，**显式必传**（协议 I3：节点传 ROS clock
                now，消灭 time.monotonic 双时钟）；窗口内各帧须同一时钟
                基准（与 TargetRegistry 的 now 注入同约定）.
            out_of_view_ids: 本帧判定为 OUT_OF_VIEW 的锁定 target_id 集合
                （阶段 D1，节点按「消失前最后检测框触图像边缘」分类）；
                None 视为空集。OUT_OF_VIEW 目标视为不可选（单位姿模型下
                复扫无益——目标已走出视野，回到同一拍照位姿也看不到它）。

        Returns
        -------
            dict：本帧 target_id → record（仅含带 ID 的记录）.

        """
        current = {
            str(record.get('target_id')): record
            for record in records
            if record.get('target_id')
        }
        self.current_selectable_ids = {
            target_id for target_id, record in current.items()
            if _selectable(record)
        }
        # 帧计数与「最近被观测帧」登记：LOST 帧龄 = 当前帧 − 最近观测帧
        self._frame_index += 1
        for target_id in current:
            self._last_seen_frame[target_id] = self._frame_index
        self.out_of_view_ids = {
            str(target_id) for target_id in (out_of_view_ids or ())
        } & set(self.locked_ids)
        if not self._locked:
            event = self._lock_policy.update(list(current.values()), now)
            if event is not None:
                self._lock_now(event.records)
        else:
            self._maintain_anchor_freshness()
            self._maintain_selection(current)
        return current

    def _maintain_anchor_freshness(self) -> None:
        """
        锁定目标的 LOST 帧龄两档处置（阶段 D1，协议 2.4；须已锁定）.

        超 anchor_max_age_frames → anchor_stale_ids（视为不可选但不移除，
        重新被观测帧龄归零自动恢复可选）；超 anchor_drop_frames → 从
        locked_ids/priorities 移除并记 dropped_ids + _dropped_queue（节点
        取走记账 target_dropped；编排侧按 SKIPPED_UNREACHABLE
        「目标丢失超时」入账）。已完成目标不再处置（账目已定）。
        """
        stale = set()
        kept = []
        for target_id in self.locked_ids:
            age = (self._frame_index
                   - self._last_seen_frame.get(target_id, self._frame_index))
            if target_id not in self.completed_ids:
                if age > self._anchor_drop_frames:
                    self.dropped_ids.add(target_id)
                    self._dropped_queue.append(target_id)
                    self.priorities.pop(target_id, None)
                    continue
                if age > self._anchor_max_age_frames:
                    stale.add(target_id)
            kept.append(target_id)
        self.locked_ids = tuple(kept)
        self.anchor_stale_ids = stale

    def _next_selectable(self) -> str:
        """
        按固定优先级选下一个可选目标（完成/移除/陈旧/出视野一律跳过）.

        anchor_stale / out_of_view 是消失态（不在当前帧记录里、本就不在
        current_selectable_ids），此处显式排除是防御性兜底：语义上
        「视为不可选」不依赖记录缺席这一间接事实（协议 2.4）。
        """
        return next(
            (target_id for target_id in self.locked_ids
             if target_id not in self.completed_ids
             and target_id not in self.anchor_stale_ids
             and target_id not in self.out_of_view_ids
             and target_id in self.current_selectable_ids), '')

    def _maintain_selection(self, current) -> None:
        """
        选中目标的失格去选与空位重选（协议 2.4：不可选 → 空串，恢复后重选）.

        去选触发（仅阶段 D1 新增三类，既有 LOST 粘性选中语义不变——
        锚点回填期内 LOST 选中目标保持选中可派发）：
        被移除（LOST 超 anchor_drop）、anchor_stale、OUT_OF_VIEW、
        或当前帧记录带 target_swinging 阻断旗标。去选后立即按固定优先级
        重选（不跳跃）；无可选者 selected 为空串，恢复后下帧重选。
        """
        selected = self.selected_target_id
        if selected:
            record = current.get(selected)
            swinging = (
                record is not None
                and 'target_swinging' in set(record.get('diagnostic_flags', ())))
            if (selected not in self.locked_ids
                    or selected in self.anchor_stale_ids
                    or selected in self.out_of_view_ids
                    or swinging):
                self.selected_target_id = ''
        if not self.selected_target_id:
            self.selected_target_id = self._next_selectable()

    def _lock_now(self, accumulated) -> None:
        """窗口关闭：累积集排序截断，一次性写入锁定状态（允许空集锁定）."""
        ordered = sorted(
            accumulated, key=self._rank_key)[:self.max_targets]
        ids = []
        seen = set()
        for record in ordered:
            target_id = str(record.get('target_id', ''))
            # 去重兜底：累积集 dict 键本已唯一，此处防御外部构造的脏 record
            if target_id and target_id not in seen:
                seen.add(target_id)
                ids.append(target_id)
        self.locked_ids = tuple(ids)
        self.priorities = {
            target_id: index + 1
            for index, target_id in enumerate(self.locked_ids)
        }
        selectable_ids = {
            str(record.get('target_id'))
            for record in accumulated
            if _selectable(record)
        }
        self.selected_target_id = next(
            (target_id for target_id in self.locked_ids
             if target_id in selectable_ids), '')
        self._locked = True
        self.snapshot_id += 1

    def mark_completed(self, target_id: str) -> None:
        """记入完成集；不推进本地 cursor（当前作业以执行器 target_id 为准）."""
        if target_id:
            self.completed_ids.add(target_id)
            if self.selected_target_id == target_id:
                self.selected_target_id = ''

    def pop_dropped(self) -> List[str]:
        """
        取走自上次调用以来被移除的 target_id 队列（阶段 D1 移除入口）.

        节点在 update 后调用一次：对每个返回 ID 记 target_dropped 事件并
        同步 harvest_state；编排侧据此按 SKIPPED_UNREACHABLE（目标丢失
        超时）入账（协议 2.4）。

        Returns
        -------
            按移除先后排序的 target_id 列表；无移除为空列表.

        """
        dropped = list(self._dropped_queue)
        self._dropped_queue.clear()
        return dropped

    def harvest_status(
            self, target_id: str, executor_id: Optional[str] = None) -> str:
        """采摘状态。executor_id 非 None 时当前目标只跟执行器，不回退本地 cursor."""
        if target_id in self.completed_ids:
            return 'HARVESTED'
        current = (
            executor_id if executor_id is not None else self.selected_target_id)
        if current and target_id == current:
            return 'SELECTED'
        if target_id not in self.current_selectable_ids:
            return 'WAITING_QUALITY'
        return 'PLANNED'

    def priority(self, target_id: str) -> int:
        """返回固定优先级；未锁定 ID 返回 0."""
        return int(self.priorities.get(target_id, 0))

    def reset(self) -> None:
        """清空目标集合与收齐窗口状态，允许下一轮全局观测重新锁定."""
        self.locked_ids = ()
        self.priorities = {}
        self.selected_target_id = ''
        self.completed_ids.clear()
        self.current_selectable_ids.clear()
        self._locked = False
        # 阶段 D1 状态一并复位：新一轮目标集合重新锁定，上一轮的锚点
        # 帧龄/出视野/移除账目不带入（_last_seen_frame/_frame_index 保留
        # 无妨——锁定集已空，但一并清零语义更直白）
        self._frame_index = 0
        self._last_seen_frame.clear()
        self.anchor_stale_ids.clear()
        self.out_of_view_ids.clear()
        self.dropped_ids.clear()
        self._dropped_queue.clear()
        self._lock_policy.reset()


# === target_registry.py ===

class SpatialEmaMatcher(TargetMatcher):
    """
    空间最近邻匹配器（TargetMatcher 默认实现，注册名 'spatial_ema'）.

    两段搜索（仅前一段未命中才进下一段）：
      1. 正常匹配：同类、距离 ≤ match_radius 取最近者；
      2. 恢复匹配（同类）：半径放宽到 match_radius × recovery_scale
         （recovery_scale>1 时），抗检测跳动导致的锚点跳变。
    两档优先级：已确认表项优先于未确认表项（瞬时目标不抢稳定身份）。
    帧级路径（match_or_register_frame）只走全局 1-1 同类分配；跨类恢复
    为预留能力（曾以 cross_class_recovery 配置承诺、帧级路径从未生效，
    已删配置；需要时在帧级分配后对未命中项补一次 class 打开的二次分配）。

    生命周期：与 TargetRegistry 同寿，由节点按 matcher.impl 创建注入。
    线程安全：无内部状态（配置不可变），与注册表同一把外部锁保护。
    可替换性：实现 TargetMatcher 即可经 MATCHERS 注册表替换。
    """

    def __init__(self, match_radius: float = 0.06,
                 recovery_scale: float = 1.0):
        """建匹配器；参数校验（半径>0、倍率≥1）."""
        if match_radius <= 0.0:
            raise ValueError(f'match_radius 须 > 0，got {match_radius}')
        if recovery_scale < 1.0:
            raise ValueError(f'recovery_scale 须 ≥ 1，got {recovery_scale}')
        self.match_radius = float(match_radius)
        self.recovery_scale = float(recovery_scale)

    def match(self, anchor: np.ndarray, class_id: int,
              table: Dict[str, dict], frame_used: set) -> MatchResult:
        """
        三段搜索链找命中表项；全部未命中返回 MatchResult(None, radius).

        Args:
            anchor: (3,) 世界系位置（米），候选目标的空间锚点.
            class_id: 候选类别（正常段要求同类）.
            table: target_id → 表项 dict（只读；用 'class_id' /
                'position' / 'confirmed' 三键）.
            frame_used: 本帧已命中的 target_id 集合（同帧去重，跳过）.

        Returns
        -------
            MatchResult：命中给 (target_id, 距离)；未命中给
            (None, match_radius)（距离字段仅供诊断，无语义承诺）.

        """
        dets = [{
            'position': np.asarray(anchor, dtype=float).reshape(3),
            'class_id': int(class_id),
            'covariance': None,
        }]
        (tid, d2, status), = assign_detections(
            dets, table, frame_used, self.match_radius, self.recovery_scale)
        dist = (float(np.sqrt(max(d2, 0.0))) if tid is not None
                else self.match_radius)
        return MatchResult(target_id=tid, distance=dist, status=status)


class TargetRegistry:
    """
    世界系目标表：跨帧维持稳定 target_id，位置/轴/直径 EMA 平滑.

    每条表项::

        {target_id, class_id, position(3,), axis(3,) | None, diameter,
         first_seen, last_seen, obs_count, last_status, confirmed,
         swing_up, swing_down, swinging}

    构造参数：matcher 为目标匹配器（TargetMatcher 接口；None 时按
    match_radius / recovery_scale 构造默认
    SpatialEmaMatcher——后两个参数仅在该路径生效）；max_targets 为表容量
    上限，超限注册新目标时淘汰 last_seen 最旧的表项；position_ema 为
    EMA 系数 α∈(0,1]，new = (1-α)·old + α·obs，position / axis /
    diameter 共用。confirm_frames 为目标确认帧数（≥1）：新注册表项为
    未确认状态，累计命中 ≥ 本值才转正长期记录；tentative_ttl_frames 为
    未确认表项的存活时限（帧，≥1）：连续超本帧数未再命中即在
    begin_frame 时清除——瞬时出现又消失的误检不留记录。按帧计而非墙钟秒：
    帧率以运行状态为准，任何帧率下确认进度都不会被 TTL 误清。
    max_age_s 为全部表项（含已确认）的墙钟龄上限（秒，>0）：begin_frame
    时淘汰 last_seen 超龄表项（阶段 D1，防跨场景陈旧锚点误命中）。
    swing_threshold_m / swing_frames 为摆动判定：观测残差连续
    swing_frames 帧超阈值置 swinging，连续同帧数低于阈值清除。
    """

    def __init__(self, match_radius: float = 0.06, max_targets: int = 50,
                 position_ema: float = 0.3, recovery_scale: float = 1.0,
                 confirm_frames: int = 1,
                 tentative_ttl_frames: int = 5,
                 max_age_s: float = 600.0, swing_threshold_m: float = 0.03,
                 swing_frames: int = 3,
                 matcher: Optional[TargetMatcher] = None):
        """建空表（容量/EMA/确认帧/TTL/max_age/摆动参数校验，匹配器可注入）."""
        if max_targets < 1:
            raise ValueError(f'max_targets 须 ≥ 1，got {max_targets}')
        if not 0.0 < position_ema <= 1.0:
            raise ValueError(f'position_ema 须在 (0, 1]，got {position_ema}')
        if confirm_frames < 1:
            raise ValueError(f'confirm_frames 须 ≥ 1，got {confirm_frames}')
        if tentative_ttl_frames < 1:
            raise ValueError(
                f'tentative_ttl_frames 须 ≥ 1，got {tentative_ttl_frames}')
        if max_age_s <= 0.0:
            raise ValueError(f'max_age_s 须 > 0，got {max_age_s}')
        if swing_threshold_m <= 0.0:
            raise ValueError(f'swing_threshold_m 须 > 0，got {swing_threshold_m}')
        if swing_frames < 1:
            raise ValueError(f'swing_frames 须 ≥ 1，got {swing_frames}')
        self._matcher = matcher or SpatialEmaMatcher(
            match_radius=match_radius, recovery_scale=recovery_scale)
        self.max_targets = int(max_targets)
        self.alpha = float(position_ema)
        self.confirm_frames = int(confirm_frames)
        self.tentative_ttl_frames = int(tentative_ttl_frames)
        self.max_age_s = float(max_age_s)
        self.swing_threshold_m = float(swing_threshold_m)
        self.swing_frames = int(swing_frames)
        self._targets: Dict[str, dict] = {}
        self._next_index = 0          # 单调计数器，不复用已消亡序号
        self._frame_used: set = set()  # 本帧已命中的 target_id（同帧去重）
        self._frame_index = 0         # 帧计数（begin_frame 递增，TTL 按帧判定）
        self._n_matched = 0           # 累计命中次数（诊断用）
        self._n_registered = 0        # 累计新发 ID 次数（诊断用）

    @property
    def match_radius(self) -> float:
        """正常匹配半径 (m)（透传自匹配器，节点日志/诊断用）."""
        return self._matcher.match_radius

    @property
    def pending_count(self) -> int:
        """
        确认中（未转正）表项数（缺陷 R-D8 发现进度摘要，只读）.

        新注册表项累计命中满 confirm_frames 才转正；此处统计尚未转正的
        表项数，供节点在锁定前发布「确认中记录数」进度摘要。
        """
        return sum(1 for t in self._targets.values() if not t['confirmed'])

    def begin_frame(self, now: Optional[float] = None) -> None:
        """
        开始新一帧：清空同帧去重集合，并做两类过期清除.

        未确认表项的存活期按帧计（当前帧序号 - 最后命中帧序号 >
        tentative_ttl_frames 即清除），随运行帧率自适应，不做墙钟假设。
        max_age_s 淘汰（阶段 D1，协议 2.4 语义变更）：任何表项（含已确认）
        距最后命中超 max_age_s 墙钟秒即淘汰——跨场景/跨批次的陈旧锚点会在
        新场景被恢复匹配误命中抢走新身份，长期存活性必须按墙钟判定
        （与帧率无关）；节点经 now 注入与 match_or_register 相同的时钟源，
        测试可注入显式值。

        Args:
            now: 时间戳 (s) 或 None；None 时只做帧 TTL 清除（兼容不注入
                时钟的旧调用），注入时才启用 max_age_s 墙钟淘汰，且须与
                match_or_register_frame 的 now 同一时钟基准（否则 max_age
                比较失真）.

        Returns
        -------
            无返回值（None）；每帧匹配循环前由节点调用一次.

        """
        self._frame_used.clear()
        self._frame_index += 1
        stale = [tid for tid, t in self._targets.items()
                 if not t['confirmed']
                 and self._frame_index - t['last_seen_frame']
                 > self.tentative_ttl_frames]
        ts = None if now is None else float(now)
        if ts is not None:
            # now 未注入时跳过墙钟淘汰：match_or_register_frame 若用了
            # time.monotonic() 兜底，两时钟混比会把表项瞬间误判超龄；
            # 生产路径节点对 begin_frame/match_or_register_frame 注入同一时钟
            stale.extend(
                tid for tid, t in self._targets.items()
                if ts - t['last_seen'] > self.max_age_s)
        for tid in set(stale):
            del self._targets[tid]

    def match_or_register_frame(
        self,
        items: Sequence[dict],
        now: Optional[float] = None,
    ) -> List[Tuple[str, bool]]:
        """
        对本帧全部有锚点检测做一次全局 1-1 分配后再提交.

        无 position 的项标 untracked_{i}、不占表（不用 target_{i}，避免
        与注册表 target_0 撞号）。歧义标 ambiguous_* 且不入表.
        """
        ts = time.monotonic() if now is None else float(now)
        out: List[Optional[Tuple[str, bool]]] = [None] * len(items)
        dets: List[dict] = []
        index_map: List[int] = []
        parsed: List[tuple] = []
        for i, item in enumerate(items):
            pos = item.get('position')
            if pos is None:
                out[i] = (f'untracked_{i}', False)
                continue
            pos = np.asarray(pos, dtype=float).reshape(3)
            if not np.all(np.isfinite(pos)):
                out[i] = (f'untracked_{i}', False)
                continue
            ax = None
            axis = item.get('axis')
            if axis is not None:
                candidate_axis = np.asarray(axis, dtype=float).reshape(3)
                if np.all(np.isfinite(candidate_axis)):
                    nrm = float(np.linalg.norm(candidate_axis))
                    if nrm > 1e-9:
                        ax = candidate_axis / nrm
            diameter_value = float(item.get('diameter') or 0.0)
            if not np.isfinite(diameter_value) or diameter_value <= 0.0:
                diameter_value = 0.0
            parsed.append((pos, int(item.get('class_id', 0)), ax,
                           diameter_value, str(item.get('status', ''))))
            dets.append({
                'position': pos,
                'class_id': int(item.get('class_id', 0)),
                'covariance': item.get('covariance'),
            })
            index_map.append(i)
        if dets:
            assigned = assign_detections(
                dets, self._targets, self._frame_used,
                self._matcher.match_radius,
                float(getattr(self._matcher, 'recovery_scale', 1.0)))
            for local_i, (tid, d2, status) in enumerate(assigned):
                i = index_map[local_i]
                pos, class_id, ax, diameter_value, st = parsed[local_i]
                matched = MatchResult(
                    target_id=tid, distance=float(d2), status=status)
                out[i] = self._commit_match(
                    pos, class_id, ax, diameter_value, st, ts, matched)
        return [row if row is not None else (f'untracked_{i}', False)
                for i, row in enumerate(out)]

    def _commit_match(
        self, pos, class_id: int, ax, diameter_value: float, status: str,
        ts: float, matched: MatchResult,
    ) -> Tuple[str, bool]:
        """把一次分配结果写入表（命中 EMA / 歧义跳过 / 新注册）."""
        best_id = matched.target_id
        if matched.status == 'ambiguous':
            tid = f'ambiguous_{self._frame_index}_{len(self._frame_used)}'
            return tid, True

        if best_id is not None:
            t = self._targets[best_id]
            # 摆动检测（阶段 D1，协议 2.4）：残差取 EMA 更新前的距离——
            # 反映原始观测相对平滑估计的跳动；EMA 更新后残差会被 α 衰减，
            # 灵敏度失真。有观测才投票；目标 LOST 帧不增不清连击（无观测
            # 不是平息证据）。置位/清除对称：连续 swing_frames 帧超阈值置位，
            # 连续 swing_frames 帧低于阈值清除
            residual = float(np.linalg.norm(pos - t['position']))
            if residual > self.swing_threshold_m:
                t['swing_up'] += 1
                t['swing_down'] = 0
            else:
                t['swing_down'] += 1
                t['swing_up'] = 0
            if t['swing_up'] >= self.swing_frames:
                t['swinging'] = True
            elif t['swing_down'] >= self.swing_frames:
                t['swinging'] = False
            a = self.alpha
            t['position'] = (1.0 - a) * t['position'] + a * pos
            if ax is not None:
                if t['axis'] is None:
                    t['axis'] = ax
                else:
                    old = t['axis']
                    # 轴 ± 二义性：符号对齐后 EMA，退化（近零）时保留旧轴
                    if float(np.dot(old, ax)) < 0.0:
                        ax = -ax
                    merged = (1.0 - a) * old + a * ax
                    n = float(np.linalg.norm(merged))
                    if n > 1e-9:
                        t['axis'] = merged / n
            if diameter_value > 0.0:
                t['diameter'] = (
                    (1.0 - a) * t['diameter'] + a * diameter_value)
            t['obs_count'] += 1
            if t['obs_count'] >= self.confirm_frames:
                t['confirmed'] = True
            t['last_seen'] = ts
            t['last_seen_frame'] = self._frame_index
            t['last_status'] = status
            self._frame_used.add(best_id)
            self._n_matched += 1
            return best_id, False

        if len(self._targets) >= self.max_targets:
            oldest = min(self._targets, key=lambda k: self._targets[k]['last_seen'])
            del self._targets[oldest]
        tid = f'target_{self._next_index}'
        self._next_index += 1
        self._targets[tid] = {
            'target_id': tid,
            'class_id': int(class_id),
            'position': pos,
            'axis': ax,
            'diameter': diameter_value,
            'first_seen': ts,
            'last_seen': ts,
            'last_seen_frame': self._frame_index,
            'obs_count': 1,
            'last_status': status,
            # 未确认表项：累计命中满 confirm_frames 才转正；连续超
            # tentative_ttl_frames 帧未再命中由 begin_frame 清除（瞬时误检
            # 不留长期记录；按帧计，帧率以运行状态为准）
            'confirmed': self.confirm_frames <= 1,
            # 摆动检测连击（阶段 D1）：首帧注册无残差可判，从零计起
            'swing_up': 0,
            'swing_down': 0,
            'swinging': False,
        }
        self._frame_used.add(tid)
        self._n_registered += 1
        return tid, True

    def clear(self) -> int:
        """
        清空全部表项（阶段 D1；节点 BeginScene 换场时调用）.

        用途：物理场景切换时清掉上一轮锚点记忆，防陈旧锚点在新场景被
        恢复匹配误命中。计划/锁定集不在本类职责内，由节点保持不动。
        序号计数器 _next_index 不复位：清空后新发 ID 仍全局单调，避免与
        清空前已下发给下游的 target_id 撞号。

        Returns
        -------
            清除前的表项数（服务应答 message 用）.

        """
        count = len(self._targets)
        self._targets.clear()
        self._frame_used.clear()
        return count

    def stats(self) -> dict:
        """
        注册表诊断快照（供节点周期日志）.

        Returns
        -------
            dict：n_targets（当前表规模）、next_index（下一个序号）、
            n_matched（累计命中次数）、n_registered（累计注册次数；
            与 next_index 恒等，序号不复用）.

        """
        return {
            'n_targets': len(self._targets),
            'next_index': self._next_index,
            'n_matched': self._n_matched,
            'n_registered': self._n_registered,
        }

    def get(self, target_id: str) -> Optional[dict]:
        """
        按 target_id 取表项（测试/调试用）.

        Args:
            target_id: 目标 ID.

        Returns
        -------
            表项 dict 或 None.

        """
        return self._targets.get(target_id)

    def target_ids(self) -> List[str]:
        """
        当前表内全部 target_id（测试/调试用）.

        Returns
        -------
            target_id 列表.

        """
        return list(self._targets.keys())


# === anchor_memory.py ===

def first_point(*candidates) -> Optional[np.ndarray]:
    """返回第一个非 None 的 3D 点."""
    for point in candidates:
        if point is not None:
            return np.asarray(point, dtype=np.float64)
    return None


@dataclass(frozen=True)
class MemoryGrasp:
    """身份表记忆还原的袋底/颈/轴/入口."""

    bottom: np.ndarray
    neck: np.ndarray
    axis: np.ndarray
    entry_start: np.ndarray
    rotation: np.ndarray


def memory_grasp(entry: dict, standoff: float) -> Optional[MemoryGrasp]:
    """从 TargetRegistry 条目还原可规划锚点；缺位置则 None."""
    if not entry or entry.get('position') is None:
        return None
    center = np.asarray(entry['position'], dtype=np.float64)
    axis = entry.get('axis')
    axis = (np.array([0.0, 0.0, 1.0]) if axis is None
            else np.asarray(axis, dtype=np.float64))
    half = 0.5 * float(entry.get('diameter') or 0.06)
    rotation = grasp_frame_from_axis(axis)
    zg = rotation[:, 2]
    bottom = center - zg * half
    neck = center + zg * half
    return MemoryGrasp(
        bottom=bottom,
        neck=neck,
        axis=zg,
        entry_start=compute_entry_start(bottom, zg, standoff),
        rotation=rotation,
    )


MATCHERS.register('spatial_ema', SpatialEmaMatcher)
LOCK_POLICIES.register('collect_lock', CollectLockPolicy)
