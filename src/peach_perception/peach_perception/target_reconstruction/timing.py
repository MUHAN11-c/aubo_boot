from __future__ import annotations
"""采帧/积分/refit 墙钟耗时统计（TimingStats）。"""


from peach_perception.common.ema import ScalarEma

# EMA 平滑系数（新样本权重）；0.3 在响应速度与抗单帧抖动间取折中
EMA_ALPHA = 0.3


class TimingStats:
    """重建流水线耗时累计器（分项 EMA + 单次 last 值；纯数值，零 ROS）."""

    def __init__(self):
        """清零：EMA 分项未播种（快照投影 0.0），计数为 0."""
        self._icp_ms = ScalarEma(EMA_ALPHA)
        self._tsdf_integrate_ms = ScalarEma(EMA_ALPHA)
        self._frame_total_ms = ScalarEma(EMA_ALPHA)
        self._refit_ms_last = 0.0
        self._finalize_ms_last = 0.0
        self._frames_timed = 0

    def record_icp(self, sample_ms: float) -> None:
        """记录一次 ICP refine 耗时 [ms]（每帧至多一次，拒帧也计入）."""
        self._icp_ms.update(max(0.0, float(sample_ms)))

    def record_tsdf_integrate(self, sample_ms: float) -> None:
        """记录一次 TSDF 在线积分+产物刷新耗时 [ms]（仅积分成功路径）."""
        self._tsdf_integrate_ms.update(max(0.0, float(sample_ms)))

    def record_frame_total(self, sample_ms: float) -> None:
        """记录一次成功采帧的 _accept_frame 总耗时 [ms]，并递增计数."""
        self._frame_total_ms.update(max(0.0, float(sample_ms)))
        self._frames_timed += 1

    def record_refit(self, sample_ms: float) -> None:
        """记录最近一次 refit 调用耗时 [ms]（last 值，含失败路径）."""
        self._refit_ms_last = max(0.0, float(sample_ms))

    def record_finalize(self, sample_ms: float) -> None:
        """记录最近一次 _finalize_now 总耗时 [ms]（last 值，含失败路径）."""
        self._finalize_ms_last = max(0.0, float(sample_ms))

    @staticmethod
    def _project(ema: ScalarEma) -> float:
        """内部 EMA → 快照标量：未播种投影为 0.0."""
        return 0.0 if not ema.seeded else float(ema.value)

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
