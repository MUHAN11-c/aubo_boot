"""
目标身份注册表 — 世界系目标记忆（纯 Python + numpy，不依赖 ROS）.

解决的问题:
  节点每帧对检测赋帧内序号 ``target_{i}``（i 为帧内序号），同一物理桃子
  在相机视野中消失再出现后 ID 全变，下游无法追踪具体目标。本模块在
  **世界系（output_frame）** 下按「同类 + 欧氏距离 ≤ match_radius」做最近邻
  匹配：命中则复用历史 target_id 并对位置/轴/直径做 EMA 平滑，未命中发新 ID
  （单调计数 ``target_0`` / ``target_1`` / …，不复用已消亡序号）。

语义边界:
  - 只维护身份与位置平滑，**不影响逐帧独立计算的三态判定**（last_status 仅记录，
    不参与匹配）。
  - 调用方必须保证输入坐标在世界系；TF 不可用的帧（几何退回相机系）由节点侧
    跳过注册，避免相机系坐标污染本表。
  - 同帧去重：一帧内已命中的表项不再参与后续匹配（begin_frame 重置），
    保证同帧两个候选不会撞同一表项。
  - 已确认目标不删除；注册数超 max_targets 时按「最久未见」淘汰（淘汰
    优先落在 last_seen 最旧者，未确认表项另有 TTL 清除，见下）。
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
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np


class TargetRegistry:
    """
    世界系目标表：跨帧维持稳定 target_id，位置/轴/直径 EMA 平滑.

    每条表项::

        {target_id, class_id, position(3,), axis(3,) | None, diameter,
         first_seen, last_seen, obs_count, last_status, confirmed}

    构造参数：match_radius 为匹配半径 (m)，同类表项中距离 ≤ 本值的最近者
    命中；max_targets 为表容量上限，超限注册新目标时淘汰 last_seen 最旧的
    表项；position_ema 为 EMA 系数 α∈(0,1]，new = (1-α)·old + α·obs，
    position / axis / diameter 共用。recovery_scale 为恢复匹配半径倍率
    （≥1，1.0=关闭，仅放大同类匹配半径），cross_class_recovery 为恢复匹配
    是否允许跨类别（半径不放大，仍限 match_radius 内）。
    confirm_frames 为目标确认帧数（≥1）：新注册表项为未确认状态，累计命中
    ≥ 本值才转正长期记录；tentative_ttl_frames 为未确认表项的存活时限
    （帧，≥1）：连续超本帧数未再命中即在 begin_frame 时清除——瞬时出现又
    消失的误检不留记录。按帧计而非墙钟秒：帧率以运行状态为准，任何帧率下
    确认进度都不会被 TTL 误清。

    """

    def __init__(self, match_radius: float = 0.06, max_targets: int = 50,
                 position_ema: float = 0.3, recovery_scale: float = 1.0,
                 cross_class_recovery: bool = False, confirm_frames: int = 1,
                 tentative_ttl_frames: int = 5):
        """建空表；参数校验（半径>0、容量≥1、0<α≤1、倍率≥1、帧数≥1、TTL≥1帧）."""
        if match_radius <= 0.0:
            raise ValueError(f'match_radius 须 > 0，got {match_radius}')
        if max_targets < 1:
            raise ValueError(f'max_targets 须 ≥ 1，got {max_targets}')
        if not 0.0 < position_ema <= 1.0:
            raise ValueError(f'position_ema 须在 (0, 1]，got {position_ema}')
        if recovery_scale < 1.0:
            raise ValueError(f'recovery_scale 须 ≥ 1，got {recovery_scale}')
        if confirm_frames < 1:
            raise ValueError(f'confirm_frames 须 ≥ 1，got {confirm_frames}')
        if tentative_ttl_frames < 1:
            raise ValueError(
                f'tentative_ttl_frames 须 ≥ 1，got {tentative_ttl_frames}')
        self.match_radius = float(match_radius)
        self.max_targets = int(max_targets)
        self.alpha = float(position_ema)
        self.recovery_scale = float(recovery_scale)
        self.cross_class_recovery = bool(cross_class_recovery)
        self.confirm_frames = int(confirm_frames)
        self.tentative_ttl_frames = int(tentative_ttl_frames)
        self._targets: Dict[str, dict] = {}
        self._next_index = 0          # 单调计数器，不复用已消亡序号
        self._frame_used: set = set()  # 本帧已命中的 target_id（同帧去重）
        self._frame_index = 0         # 帧计数（begin_frame 递增，TTL 按帧判定）
        self._n_matched = 0           # 累计命中次数（诊断用）
        self._n_registered = 0        # 累计新发 ID 次数（诊断用）

    def begin_frame(self) -> None:
        """
        开始新一帧：清空同帧去重集合，并清除超期未命中的未确认表项.

        未确认表项的存活期按帧计（当前帧序号 - 最后命中帧序号 >
        tentative_ttl_frames 即清除），随运行帧率自适应，不做墙钟假设。

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
        for tid in stale:
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

        匹配规则：第一段为同类（class_id 相等）、未被本帧占用、欧氏距离
        ≤ match_radius 的表项中取最近者；未命中时依次尝试两段恢复匹配：
        同类半径 × recovery_scale（recovery_scale>1 时）、跨类但半径不变
        （cross_class_recovery 为真时；跨类命中不改表项类别，只复用 ID）。
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

        best_id, _ = self._find_nearest(pos, class_id, self.match_radius,
                                        same_class=True)
        if best_id is None and self.recovery_scale > 1.0:
            # 恢复匹配①（同类）：仅正常匹配未命中才启用，半径放宽抗锚点跳变；
            # 常态相邻目标在第一段已正确区分，走不到这里
            best_id, _ = self._find_nearest(
                pos, class_id, self.match_radius * self.recovery_scale,
                same_class=True)
        if best_id is None and self.cross_class_recovery:
            # 恢复匹配②（跨类）：半径**不放大**——真翻类锚点几乎不动，
            # 仍在 match_radius 内；放宽类别约束但不放宽距离，避免邻近的
            # 异类新目标抢已有 ID（身份优先，命中不改表项类别）
            best_id, _ = self._find_nearest(pos, class_id, self.match_radius,
                                            same_class=False)

        if best_id is not None:
            t = self._targets[best_id]
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
        }
        self._frame_used.add(tid)
        self._n_registered += 1
        return tid, True

    def _find_nearest(self, pos, class_id: int, radius: float,
                      same_class: bool):
        """
        在未被本帧占用的表项中找距离 ≤ radius 的最近者（私有，已确认表项优先）.

        两档优先级：已确认表项优先于未确认表项——候选同时落在一个已确认
        目标与一个更近的未确认（疑似误检）目标半径内时，命中已确认者，
        避免瞬时出现的目标抢走稳定身份。

        Args:
            pos: (3,) 世界系位置.
            class_id: 候选类别；same_class=True 时仅同类参与.
            radius: 匹配半径 (m).
            same_class: 是否要求类别相等（恢复匹配可放宽）.

        Returns
        -------
            (target_id | None, 距离)：无候选时 (None, radius).

        """
        best_id, best_d = None, radius        # 已确认档
        tent_id, tent_d = None, radius        # 未确认档
        for tid, t in self._targets.items():
            if tid in self._frame_used:
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
