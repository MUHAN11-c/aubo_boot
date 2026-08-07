"""
时序融合器 — 单目标候选流的一致性门控 + 置信度加权 EMA + 一致簇统计.

对 grasp_candidates 流逐帧融合：与当前融合值位置距 > pos_jump_gate_m 或
轴夹角（无向）> axis_gate_deg 的观测判离群剔除；接受的观测按置信度加权
EMA 更新融合位置/轴向/袋径；接受历史经迷你 DBSCAN（eps=dbscan_eps_m）
统计一致簇大小作为 support_frames，并暴露 pos_std/axis_std 收敛性指标。
纯 numpy 实现（colcon test 走系统 python3，不引入 open3d/sklearn）。
"""
from collections import deque
from typing import Deque, Optional

import numpy as np


class TemporalFuser:
    """
    单目标时序融合器（位置/轴向/袋径 EMA + 离群门控 + 一致簇统计）.

    轴向无向处理：融合前按与当前轴向的点积符号对齐（点积 <0 先翻转），
    避免 ±axis 二义性把均值拉到零向量。
    """

    def __init__(self, pos_jump_gate_m: float = 0.05,
                 axis_gate_deg: float = 20.0, dbscan_eps_m: float = 0.03,
                 ema_alpha: float = 0.3, history: int = 200):
        """
        构造融合器.

        Args:
            pos_jump_gate_m: 位置跳变门 (m)，观测距融合值超过即判离群.
            axis_gate_deg: 轴向门 (deg)，无向夹角超过即判离群.
            dbscan_eps_m: 一致簇邻域半径 (m)（迷你 DBSCAN 连通簇）.
            ema_alpha: EMA 基础系数（实际步长再乘观测置信度，钳到 [0,1]）.
            history: 接受观测历史长度（一致簇/标准差统计窗口）.

        Returns
        -------
            无返回值（None）.

        """
        self.pos_jump_gate_m = float(pos_jump_gate_m)
        self.axis_gate_deg = float(axis_gate_deg)
        self.dbscan_eps_m = float(dbscan_eps_m)
        self.ema_alpha = float(ema_alpha)
        self._pos: Optional[np.ndarray] = None     # 融合位置 (3,)，米
        self._axis: Optional[np.ndarray] = None    # 融合轴向（单位向量）
        self._diameter = 0.0                       # 融合袋径 (m)
        self._conf = 0.0                           # 融合置信度
        self._positions: Deque[np.ndarray] = deque(maxlen=history)
        self._axes: Deque[np.ndarray] = deque(maxlen=history)
        self.accepted = 0                          # 累计接受观测数
        self.rejected = 0                          # 累计离群观测数
        self.frame_seq = 0                         # 融合器内部帧序号

    def update(self, position, axis, diameter: float = 0.0,
               confidence: float = 1.0) -> bool:
        """
        灌入一条候选观测.

        Args:
            position: (3,) 观测位置（输出系，米）.
            axis: (3,) 观测套入轴（无需单位化；零向量直接判离群）.
            diameter: 袋径观测 (m).
            confidence: 观测置信度 [0,1]（加权 EMA 步长）.

        Returns
        -------
            True=接受并更新融合值；False=判离群（位置/轴向超门或轴零向量）.

        """
        self.frame_seq += 1
        p = np.asarray(position, dtype=float)
        a = np.asarray(axis, dtype=float)
        norm_a = float(np.linalg.norm(a))
        if norm_a < 1e-9:
            self.rejected += 1
            return False
        a = a / norm_a
        if self._pos is not None:
            if float(np.linalg.norm(p - self._pos)) > self.pos_jump_gate_m:
                self.rejected += 1
                return False
            cos = abs(float(a @ self._axis))
            angle = float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0))))
            if angle > self.axis_gate_deg:
                self.rejected += 1
                return False
            # 置信度加权 EMA：低置信观测步长小，抗抖动
            alpha = float(np.clip(self.ema_alpha * confidence, 0.0, 1.0))
            self._pos = (1.0 - alpha) * self._pos + alpha * p
            if float(a @ self._axis) < 0.0:
                a = -a                           # 轴无向：先对齐符号再融合
            fused = (1.0 - alpha) * self._axis + alpha * a
            self._axis = fused / float(np.linalg.norm(fused))
            self._diameter = (1.0 - alpha) * self._diameter + alpha * float(diameter)
            self._conf = (1.0 - alpha) * self._conf + alpha * float(confidence)
        else:
            self._pos, self._axis = p, a
            self._diameter, self._conf = float(diameter), float(confidence)
        self.accepted += 1
        self._positions.append(p)
        self._axes.append(a)
        return True

    def current(self) -> Optional[dict]:
        """
        当前融合估计.

        Returns
        -------
            dict(position/axis/diameter/confidence/support_frames/pos_std_m/
            axis_std_deg/accepted/rejected/frame_seq)；尚无观测返回 None.

        """
        if self._pos is None:
            return None
        return {
            'position': self._pos.copy(),
            'axis': self._axis.copy(),
            'diameter': self._diameter,
            'confidence': self._conf,
            'support_frames': self.support_frames,
            'pos_std_m': self.pos_std_m,
            'axis_std_deg': self.axis_std_deg,
            'accepted': self.accepted,
            'rejected': self.rejected,
            'frame_seq': self.frame_seq,
        }

    @property
    def support_frames(self) -> int:
        """一致簇大小：接受历史位置经迷你 DBSCAN 连通簇的最大簇点数."""
        return _largest_cluster_size(list(self._positions), self.dbscan_eps_m)

    @property
    def pos_std_m(self) -> float:
        """位置收敛性（标准误）：簇离散度 / √支撑帧数 (m)，随累积下降."""
        n = len(self._positions)
        if n == 0:
            return 0.0
        pts = np.asarray(self._positions)
        spread = float(np.std(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
        return spread / float(np.sqrt(max(1, self.support_frames)))

    @property
    def axis_std_deg(self) -> float:
        """轴向收敛性（标准误）：无向夹角离散度 / √支撑帧数 (deg)."""
        n = len(self._axes)
        if self._axis is None or n == 0:
            return 0.0
        angles = []
        for a in self._axes:
            cos = abs(float(a @ self._axis))
            angles.append(float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0)))))
        return float(np.std(angles)) / float(np.sqrt(max(1, self.support_frames)))

    def reset(self) -> None:
        """清空融合状态（计数器一并归零）."""
        self._pos = None
        self._axis = None
        self._diameter = 0.0
        self._conf = 0.0
        self._positions.clear()
        self._axes.clear()
        self.accepted = 0
        self.rejected = 0
        self.frame_seq = 0


def _largest_cluster_size(points, eps: float) -> int:
    """
    迷你 DBSCAN：eps 邻域图的连通簇（单链近似，无核心点约束）最大簇大小.

    Args:
        points: [(3,), ...] 点列.
        eps: 邻域半径 (m).

    Returns
    -------
        最大连通簇的点数；空输入给 0.

    """
    n = len(points)
    if n == 0:
        return 0
    p = np.asarray(points, dtype=float)
    #  pairwise 平方距离 → eps 邻接矩阵（N≤200，O(N²) 可忽略）
    adj = ((p[:, None, :] - p[None, :, :]) ** 2).sum(axis=2) <= eps * eps
    seen = np.zeros(n, dtype=bool)
    best = 0
    for i in range(n):
        if seen[i]:
            continue
        stack = [i]
        seen[i] = True
        size = 0
        while stack:
            j = stack.pop()
            size += 1
            for k in np.flatnonzero(adj[j]):
                if not seen[k]:
                    seen[k] = True
                    stack.append(int(k))
        best = max(best, size)
    return best
