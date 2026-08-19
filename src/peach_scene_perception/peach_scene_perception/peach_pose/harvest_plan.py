"""
全局采摘目标计划：收齐式窗口锁定、稳定 ID、多维优先级与选中目标（零 ROS import）.

收齐式锁定语义：reset（或构造）后进入收齐窗口，逐帧 ``update()`` 把确认目标
并入累积集；窗口关闭（帧数与静止条件满足，或超时兜底）时对累积集一次性
排序、截断并锁定。锁定后目标集合与优先级冻结，新出现的 ID 不再入集，
仅在当前目标完成后按固定优先级推进。

分层（2.14 装配规则，A3 起）：
  - ``CollectLockPolicy``（LockPolicy 默认实现，注册名 'collect_lock'）：
    收齐窗口的累积/静止/超时判定，窗口关闭时发 LockEvent；
  - ``GlobalHarvestPlan``（调用端）：消费 LockEvent 做排序/截断/锁定记账，
    持有锁定后状态（locked_ids/priorities/selected/completed）。
  节点经 yaml ``lock.impl`` 按名创建策略注入；调用端只依赖 LockPolicy 接口。

协议条款:
  I3（时钟唯一）：``update(records, now)`` 的 now 为显式必传参数（节点传
  ROS clock now），本模块不 import time、禁止 time.monotonic 双时钟。

阶段 D1（协议 2.4 摆动降权/锚点衰减）：
  - 阻断旗标集追加 ``target_swinging``（观测残差连击由 TargetRegistry
    判定并打入 record）；带 ``anchor_stale`` / OUT_OF_VIEW 的锁定目标
    同样「视为不可选但不移除」——两者都是消失态（不在当前帧记录里），
    由本类按帧龄/节点注入的 out_of_view_ids 维护，见 update()；
  - LOST 帧龄超 anchor_max_age_frames → 入 anchor_stale_ids（不可选）；
    超 anchor_drop_frames → 从 locked_ids 移除并记入 dropped_ids
    （selected 自动顺延；节点经 pop_dropped 取走记账 target_dropped）。
    两阈值均为**帧数**，由节点按秒级配置 ÷ 实测帧间隔 EMA 折算逐帧改写
    （协议 I4：时限按帧率 EMA 折算帧数，帧率以运行状态为准）。

线程安全约定：本模块类自身无锁；写路径（update/complete_selected/reset）
与读路径（各公共属性）由编排层（peach_scene_perception_node）的同一把锁统一保护，
保证单写者语义（选型理由见 peach_scene_perception_node 中 ``_plan_lock`` 注释）。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from .interfaces import LockEvent, LockPolicy

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

    def complete_selected(self) -> str:
        """标记当前目标已完成，并按固定优先级推进到下一个未完成 ID."""
        if self.selected_target_id:
            self.completed_ids.add(self.selected_target_id)
        self.selected_target_id = self._next_selectable()
        return self.selected_target_id

    def reopen_target(self, target_id: str) -> str:
        """
        重开已终局目标：移出 completed_ids 恢复可选（E3 残局抬质量配套）.

        编排器对 SKIPPED_QUALITY 残局目标派发 OBSERVE_ONLY 抬质量成功后
        调用（节点经 ~/reopen_target 服务入口）。重开后目标参与
        _next_selectable 的固定优先级竞争，是否立即重新选中由当前帧
        可观测性自决（不在 current_selectable_ids 即等下一帧观测恢复）。

        Args:
            target_id: 待重开目标 ID.

        Returns
        -------
            拒绝原因；空串表示成功。守卫依次：未锁定 / 不在锁定集 /
            未终局（不在 completed_ids，含从未派发与已重开）.

        """
        if not self._locked:
            return '目标集合尚未锁定，无账目可重开'
        if target_id not in self.locked_ids:
            return f'{target_id} 不在本轮锁定集'
        if target_id not in self.completed_ids:
            return f'{target_id} 未终局（无 completed 账目）'
        self.completed_ids.discard(target_id)
        return ''

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

    def harvest_status(self, target_id: str) -> str:
        """返回目标在本轮固定计划中的采摘状态."""
        if target_id in self.completed_ids:
            return 'HARVESTED'
        if target_id == self.selected_target_id:
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
