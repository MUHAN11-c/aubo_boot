"""
目标身份注册表 — 世界系目标记忆（纯 Python + numpy，不依赖 ROS）.

解决的问题:
  节点每帧对检测赋帧内序号 ``target_{i}``（i 为帧内序号），同一物理桃子
  在相机视野中消失再出现后 ID 全变，下游无法追踪具体目标。本模块在
  **世界系（output_frame）** 下按「同类 + 欧氏距离 ≤ match_radius」做最近邻
  匹配：命中则复用历史 target_id 并对位置/轴/直径做 EMA 平滑，未命中发新 ID
  （单调计数 ``target_0`` / ``target_1`` / …，不复用已消亡序号）。

分层（2.14 装配规则，A3 起）：
  - ``SpatialEmaMatcher``（TargetMatcher 默认实现，注册名 'spatial_ema'）：
    正常匹配 + 两段恢复匹配的候选搜索，不持表；
  - ``TargetRegistry``（调用端）：持有目标表，负责同帧去重、TTL 清除、
    确认机制、ID 签发、EMA 平滑与淘汰，匹配段委托给 TargetMatcher 接口。
  节点经 yaml ``matcher.impl`` 按名创建匹配器注入；调用端只依赖接口。

语义边界:
  - 只维护身份与位置平滑，**不影响逐帧独立计算的三态判定**（last_status 仅记录，
    不参与匹配）。
  - 调用方必须保证输入坐标在世界系；TF 不可用的帧（几何退回相机系）由节点侧
    跳过注册，避免相机系坐标污染本表。
  - 同帧去重：一帧内已命中的表项不再参与后续匹配（begin_frame 重置），
    保证同帧两个候选不会撞同一表项。
  - 已确认目标按 max_age_s 淘汰（阶段 D1 语义变更，协议 2.4/阶段 D 第 6 条：
    A3 前「已确认永不 TTL 清除」，但跨场景/跨批次的陈旧锚点会在新场景里
    被恢复匹配误命中——旧锚点位置恰好落在新目标恢复半径内时抢走新身份；
    超时未命中即淘汰，含已确认表项）；注册数超 max_targets 时另按
    「最久未见」淘汰（淘汰优先落在 last_seen 最旧者，未确认表项另有
    TTL 清除，见下）。max_age 按墙钟秒（表项 last_seen 时间戳），与
    未确认 TTL 的帧计数互补：长期存活性是墙钟语义，与帧率无关。
  - 确认机制：新目标先记为未确认（confirmed=False），累计命中
    ≥ confirm_frames 帧转正；未确认表项连续超 tentative_ttl_frames 帧
    未再命中即在 begin_frame 清除——瞬时出现又消失的误检不留长期记录、
    不占匹配优先级（已确认表项匹配优先于未确认表项）。存活期按**帧**计：
    帧率以运行状态为准，低帧率/卡顿时墙钟 TTL 会在确认进度攒满前误清
    表项，帧计数不受帧率影响。
  - 恢复匹配（recovery）：正常匹配（同类 + match_radius）未命中时才启用，
    分两段：① 同类、半径放宽到 match_radius × recovery_scale（吸收检测
    跳动导致的锚点跳变）；② 跨类但半径**不放大**（吸收 bag/nobag 翻类——
    真翻类锚点几乎不动，邻近的异类新目标不会被误并）。
    常态下相邻目标仍在第一段被正确区分，不受影响。
    恢复命中只复用 ID 与做 EMA，**不改表项类别**（身份优先于类别标签）。
  - 摆动检测（阶段 D1，协议 2.4 摆动降权）：命中时先算观测残差
    ‖观测锚点 − EMA 位置‖（EMA 更新前，残差反映原始观测相对平滑估计的
    跳动），连续 swing_frames 帧超 swing_threshold_m 置表项
    ``swinging=True``，连续同帧数低于阈值清除（对称连击）；目标未被
    观测的帧不投票（无观测不是平息证据）。节点据此给 record 打
    ``target_swinging`` 旗标，能力端 RECONFIRM 阶段消费。
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .interfaces import MatchResult, TargetMatcher


class SpatialEmaMatcher(TargetMatcher):
    """
    空间最近邻匹配器（TargetMatcher 默认实现，注册名 'spatial_ema'）.

    三段搜索链（仅前一段未命中才进下一段）：
      1. 正常匹配：同类、距离 ≤ match_radius 取最近者；
      2. 恢复匹配①（同类）：半径放宽到 match_radius × recovery_scale
         （recovery_scale>1 时），抗检测跳动导致的锚点跳变；
      3. 恢复匹配②（跨类）：半径**不放大**仍限 match_radius
         （cross_class_recovery 为真时），抗 bag/nobag 翻类——真翻类
         锚点几乎不动，邻近异类新目标不会被误并。
    两档优先级：已确认表项优先于未确认表项（瞬时目标不抢稳定身份）。

    生命周期：与 TargetRegistry 同寿，由节点按 matcher.impl 创建注入。
    线程安全：无内部状态（配置不可变），与注册表同一把外部锁保护。
    可替换性：实现 TargetMatcher 即可经 MATCHERS 注册表替换。
    """

    def __init__(self, match_radius: float = 0.06,
                 recovery_scale: float = 1.0,
                 cross_class_recovery: bool = False):
        """建匹配器；参数校验（半径>0、倍率≥1）."""
        if match_radius <= 0.0:
            raise ValueError(f'match_radius 须 > 0，got {match_radius}')
        if recovery_scale < 1.0:
            raise ValueError(f'recovery_scale 须 ≥ 1，got {recovery_scale}')
        self.match_radius = float(match_radius)
        self.recovery_scale = float(recovery_scale)
        self.cross_class_recovery = bool(cross_class_recovery)

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
        best_id, best_d = self._find_nearest(
            anchor, class_id, self.match_radius, True, table, frame_used)
        if best_id is None and self.recovery_scale > 1.0:
            # 恢复匹配①（同类）：仅正常匹配未命中才启用，半径放宽抗锚点跳变；
            # 常态相邻目标在第一段已正确区分，走不到这里
            best_id, best_d = self._find_nearest(
                anchor, class_id, self.match_radius * self.recovery_scale,
                True, table, frame_used)
        if best_id is None and self.cross_class_recovery:
            # 恢复匹配②（跨类）：半径**不放大**——真翻类锚点几乎不动，
            # 仍在 match_radius 内；放宽类别约束但不放宽距离，避免邻近的
            # 异类新目标抢已有 ID（身份优先，命中不改表项类别）
            best_id, best_d = self._find_nearest(
                anchor, class_id, self.match_radius, False, table, frame_used)
        return MatchResult(target_id=best_id, distance=best_d)

    @staticmethod
    def _find_nearest(pos, class_id: int, radius: float, same_class: bool,
                      table: Dict[str, dict], frame_used: set):
        """
        在未被本帧占用的表项中找距离 ≤ radius 的最近者（已确认表项优先）.

        两档优先级：已确认表项优先于未确认表项——候选同时落在一个已确认
        目标与一个更近的未确认（疑似误检）目标半径内时，命中已确认者，
        避免瞬时出现的目标抢走稳定身份。

        Returns
        -------
            (target_id | None, 距离)：无候选时 (None, radius).

        """
        best_id, best_d = None, radius        # 已确认档
        tent_id, tent_d = None, radius        # 未确认档
        for tid, t in table.items():
            if tid in frame_used:
                continue
            if same_class and t['class_id'] != class_id:
                continue
            d = float(np.linalg.norm(pos - t['position']))
            if t['confirmed']:
                if d <= best_d:
                    best_id, best_d = tid, d
            elif d <= tent_d:
                tent_id, tent_d = tid, d
        if best_id is not None:
            return best_id, best_d
        return tent_id, tent_d


class TargetRegistry:
    """
    世界系目标表：跨帧维持稳定 target_id，位置/轴/直径 EMA 平滑.

    每条表项::

        {target_id, class_id, position(3,), axis(3,) | None, diameter,
         first_seen, last_seen, obs_count, last_status, confirmed,
         swing_up, swing_down, swinging}

    构造参数：matcher 为目标匹配器（TargetMatcher 接口；None 时按
    match_radius / recovery_scale / cross_class_recovery 构造默认
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
                 cross_class_recovery: bool = False, confirm_frames: int = 1,
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
            match_radius=match_radius, recovery_scale=recovery_scale,
            cross_class_recovery=cross_class_recovery)
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
                match_or_register 的 now 同一时钟基准（否则 max_age 比较
                失真）.

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
            # now 未注入时跳过墙钟淘汰：match_or_register 若用了
            # time.monotonic() 兜底，两时钟混比会把表项瞬间误判超龄；
            # 生产路径节点对 begin_frame/match_or_register 注入同一时钟
            stale.extend(
                tid for tid, t in self._targets.items()
                if ts - t['last_seen'] > self.max_age_s)
        for tid in set(stale):
            del self._targets[tid]

    def match_or_register(
        self,
        position,
        class_id: int,
        axis=None,
        diameter: float = 0.0,
        status: str = '',
        now: Optional[float] = None,
    ) -> Tuple[str, bool]:
        """
        对一个世界系候选做匹配或注册，返回 (target_id, is_new).

        匹配段委托 TargetMatcher 接口（三段搜索链见 SpatialEmaMatcher）。
        命中：复用其 target_id，position/diameter 按 α EMA，
        双方 axis 都存在时先做符号对齐再 EMA 并归一化（轴有 ± 二义性，点积 < 0
        先取反，否则反向轴直接 EMA 会互相抵消）；obs_count+1，last_seen /
        last_status 更新。未命中：发新 ID ``target_{next_index}``（计数单调增），
        必要时先淘汰最久未见表项。

        Args:
            position: (3,) 世界系位置（米），目标身份的空间锚点.
            class_id: 检测类别（不同类永不匹配）.
            axis: (3,) 单位轴方向或 None（如袋轴 translation_direction）.
            diameter: 目标直径 (m)；≤0 视为无效观测，不参与 EMA（防 0 值污染）.
            status: 本帧三态字符串，仅记录到 last_status，不影响匹配.
            now: 时间戳 (s)，None 用 time.monotonic()；测试可注入显式值.

        Returns
        -------
            (target_id, is_new)：is_new=True 表示本次新注册.

        """
        pos = np.asarray(position, dtype=float).reshape(3)
        if not np.all(np.isfinite(pos)):
            raise ValueError('position 必须是有限的三维世界系坐标')
        ax = None
        if axis is not None:
            candidate_axis = np.asarray(axis, dtype=float).reshape(3)
            if np.all(np.isfinite(candidate_axis)):
                n = float(np.linalg.norm(candidate_axis))
                if n > 1e-9:
                    ax = candidate_axis / n
        ts = time.monotonic() if now is None else float(now)
        if not np.isfinite(ts):
            raise ValueError('now 必须是有限时间戳')
        diameter_value = float(diameter)
        if not np.isfinite(diameter_value) or diameter_value <= 0.0:
            diameter_value = 0.0

        best_id = self._matcher.match(
            pos, class_id, self._targets, self._frame_used).target_id

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
        清空全部表项（阶段 D1，~/clear_target_memory 服务的纯核入口）.

        用途（协议 2.3）：批次开局 harvest.fresh_scene=true 时编排器先调
        本服务，清掉上一轮/上一场景的锚点记忆，防陈旧锚点在新场景被恢复
        匹配误命中。计划/锁定集不在本类职责内，由节点保持不动。
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
