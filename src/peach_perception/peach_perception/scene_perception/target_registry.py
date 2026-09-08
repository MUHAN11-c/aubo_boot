from __future__ import annotations
"""世界系身份、锁定窗、记忆锚点。"""

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
from .contracts import MatchResult


class SpatialEmaMatcher:
    """
    空间最近邻匹配器（唯一实现，直接构造）.

    两段搜索（仅前一段未命中才进下一段）：
      1. 正常匹配：同类、距离 ≤ match_radius 取最近者；
      2. 恢复匹配（同类）：半径放宽到 match_radius × recovery_scale
         （recovery_scale>1 时），抗检测跳动导致的锚点跳变。
    两档优先级：已确认表项优先于未确认表项（瞬时目标不抢稳定身份）。
    帧级路径（match_or_register_frame）只走全局 1-1 同类分配；跨类恢复
    为预留能力（曾以 cross_class_recovery 配置承诺、帧级路径从未生效，
    已删配置；需要时在帧级分配后对未命中项补一次 class 打开的二次分配）。

    生命周期：与 TargetRegistry 同寿，由节点直接构造注入。
    线程安全：无内部状态（配置不可变），与注册表同一把外部锁保护。
    现行为唯一实现（直接构造）；换匹配器=改一个类。
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


class TargetRegistry:
    """
    世界系目标表：跨帧维持稳定 target_id，位置/轴/直径 EMA 平滑.

    每条表项::

        {target_id, class_id, position(3,), axis(3,) | None, diameter,
         first_seen, last_seen, obs_count, last_status, confirmed,
         swing_up, swing_down, swinging}

    构造参数：matcher 为 SpatialEmaMatcher（None 时按
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
                 matcher: Optional[SpatialEmaMatcher] = None):
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
        detections: List[dict] = []
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
            detections.append({
                'position': pos,
                'class_id': int(item.get('class_id', 0)),
                'covariance': item.get('covariance'),
            })
            index_map.append(i)
        if detections:
            assigned = assign_detections(
                detections, self._targets, self._frame_used,
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
