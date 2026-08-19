"""
耗时统计 — 重建流水线分项耗时的 EMA 累计与诊断投影（纯核，零 ROS）.

职责:
  为阶段 H 效率优化提供耗时基线：每帧 ICP 耗时、每帧 TSDF 在线积分
  耗时、每帧总处理耗时的指数滑动平均（EMA），以及最近一次 refit /
  finalize 的耗时。本类只做数值累计与快照投影，不自行取时间——计时
  打点由编排节点（reconstruction_node.py）用注入时钟完成（协议 I3：
  RclpyClockAdapter.now，禁 time.monotonic）。

输出契约（diagnostics JSON 的 timing 子对象，同时随 session
metadata.yaml 落盘）::

    {
        'icp_ms_ema': float,             # 每帧 ICP refine 耗时 EMA [ms]
        'tsdf_integrate_ms_ema': float,  # 每帧 TSDF 积分+产物刷新耗时 EMA [ms]
        'frame_total_ms_ema': float,     # 每帧成功采帧路径总耗时 EMA [ms]
        'refit_ms_last': float,          # 最近一次 refit 调用耗时 [ms]
        'finalize_ms_last': float,       # 最近一次 _finalize_now 总耗时 [ms]
        'frames_timed': int,             # 计入 frame_total EMA 的成功采帧数
    }

EMA 约定:
  α=0.3（ema ← 0.3·sample + 0.7·ema），首个样本直接播种；未采样前
  各 EMA 键投影为 0.0（消费方以 frames_timed=0 判「尚无数据」）。
  拒帧早退不计入 frame_total/frames_timed；ICP/TSDF 分项只要对应
  阶段执行过即计入（无论该帧最终是否被收）。refit/finalize 只记
  最近一次（last），不做 EMA——每轮 session 至多一次，EMA 无意义。

线程模型:
  无内部锁；全部读写发生在节点 _state_lock 持锁段（采帧/finalize
  路径），与 collector/TSDF 产物同一并发约定。
"""
from __future__ import annotations

from typing import Optional

# EMA 平滑系数（新样本权重）；0.3 在响应速度与抗单帧抖动间取折中
EMA_ALPHA = 0.3


class TimingStats:
    """重建流水线耗时累计器（分项 EMA + 单次 last 值；纯数值，零 ROS）."""

    def __init__(self):
        """清零：EMA 分项未播种（内部 None，快照投影 0.0），计数为 0."""
        self._icp_ms: Optional[float] = None
        self._tsdf_integrate_ms: Optional[float] = None
        self._frame_total_ms: Optional[float] = None
        self._refit_ms_last = 0.0
        self._finalize_ms_last = 0.0
        self._frames_timed = 0

    @staticmethod
    def _ema_update(current: Optional[float], sample_ms: float) -> float:
        """
        EMA 递推：未播种（None）直接取样本，否则 α·sample + (1−α)·current.

        Args:
            current: 该分项当前 EMA；None 表示尚无样本.
            sample_ms: 本次样本耗时 [ms]（调用方保证非负）.

        Returns
        -------
            更新后的 EMA [ms].

        """
        sample = max(0.0, float(sample_ms))
        if current is None:
            return sample
        return EMA_ALPHA * sample + (1.0 - EMA_ALPHA) * current

    def record_icp(self, sample_ms: float) -> None:
        """记录一次 ICP refine 耗时 [ms]（每帧至多一次，拒帧也计入）."""
        self._icp_ms = self._ema_update(self._icp_ms, sample_ms)

    def record_tsdf_integrate(self, sample_ms: float) -> None:
        """记录一次 TSDF 在线积分+产物刷新耗时 [ms]（仅积分成功路径）."""
        self._tsdf_integrate_ms = self._ema_update(
            self._tsdf_integrate_ms, sample_ms)

    def record_frame_total(self, sample_ms: float) -> None:
        """记录一次成功采帧的 _accept_frame 总耗时 [ms]，并递增计数."""
        self._frame_total_ms = self._ema_update(
            self._frame_total_ms, sample_ms)
        self._frames_timed += 1

    def record_refit(self, sample_ms: float) -> None:
        """记录最近一次 refit 调用耗时 [ms]（last 值，含失败路径）."""
        self._refit_ms_last = max(0.0, float(sample_ms))

    def record_finalize(self, sample_ms: float) -> None:
        """记录最近一次 _finalize_now 总耗时 [ms]（last 值，含失败路径）."""
        self._finalize_ms_last = max(0.0, float(sample_ms))

    @staticmethod
    def _project(ema: Optional[float]) -> float:
        """内部 Optional EMA → 快照标量：未播种投影为 0.0."""
        return 0.0 if ema is None else float(ema)

    def snapshot(self) -> dict:
        """
        投影诊断 timing 子对象（JSON 可序列化，键集恒定）.

        Returns
        -------
            六个契约键的 dict（见模块 docstring）；全部数值非负，
            frames_timed=0 表示尚无成功采帧计时样本.

        """
        return {
            'icp_ms_ema': self._project(self._icp_ms),
            'tsdf_integrate_ms_ema': self._project(self._tsdf_integrate_ms),
            'frame_total_ms_ema': self._project(self._frame_total_ms),
            'refit_ms_last': float(self._refit_ms_last),
            'finalize_ms_last': float(self._finalize_ms_last),
            'frames_timed': int(self._frames_timed),
        }
