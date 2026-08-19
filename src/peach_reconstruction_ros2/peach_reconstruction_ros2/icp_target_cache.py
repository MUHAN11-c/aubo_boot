"""ICP target 增量复用缓存（协议 2.13-E4，纯核零 ROS）.

职责:
  帧到模型 ICP 的 target 不再每帧从 TSDF 全量 extract（extract_point_cloud
  + ROI 裁剪 + 体素降采样 + 统计离群剔除是采集循环最贵的环节之一，阶段 C
  埋点的 tsdf 分项大头）：两次全量刷新之间，复用「上次全量提取 + 其间各
  已采帧经 ICP 修正后的 base 系点云增量拼接」作为 target；每 k 帧或关键
  事件（会话开始/绑定切换/reset/remove_last 重放/finalize 失败清理）才
  从 TSDF 全量 extract 刷新基线。

  k 自适应：按近期 ICP 修正量 EMA 伸缩——修正量 EMA ≥ max_translation×
  drift_ratio（漂移风险高），或出现 fk 回退/拒帧（对齐风险信号）→ k 立即
  收回下限 min_period；EMA 长期 ≤ 1/4 漂移阈值（稳定）→ k 拉长到上限
  max_period；中间区间保持现周期（迟滞带，防 k 在边界附近逐帧振荡）。
  起步保守：新会话无任何修正量证据前 k=min_period（逐帧全量=旧行为），
  稳定证据攒够后才拉长。

正确性红线:
  本类只决定「target 从哪来」，有界 ICP 的越界拒绝/质量门语义不变
  （icp_refiner.py 零改动）；增量 target 是同 ROI、同 base 系的同模型
  近视（差别仅是刷新窗口内各帧未经 TSDF 加权平均），由
  test_icp_refiner.py 的「复用 target 与全量 target 对齐一致」容差对照
  测试守门。

线程模型:
  无内部锁；全部读写发生在节点 _state_lock 持锁段（采帧/finalize/服务
  路径），与 collector/TSDF 产物同一并发约定。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from peach_reconstruction_ros2.tsdf_volume import LocalTsdf

# 修正量 EMA 平滑系数（与 timing.EMA_ALPHA 同约定：0.3 在响应速度与
# 抗单帧抖动间取折中；首个样本直接播种）
_CORR_EMA_ALPHA = 0.3
# 稳定判据相对漂移阈值的比例：EMA ≤ 1/4×阈值才拉长周期；与「≥1×阈值收回
# 下限」之间留出 (0.25, 1.0) 迟滞保持带，防 k 在边界附近逐帧振荡
_STABLE_RATIO = 0.25


@dataclass(frozen=True)
class IcpTargetRefreshConfig:
    """ICP target 全量刷新周期自适应配置（长度单位 [m]，周期单位 [帧]）.

    min_period/max_period/drift_ratio 来自 ROS 参数
    icp.target_refresh_min_period/max_period/drift_ratio；
    max_translation_m 注入 icp.max_translation（漂移阈值基准）；
    max_incremental_points/downsample_voxel 为容量护栏（后者注入
    tsdf.voxel_length，与模型分辨率同尺度）。
    """

    min_period: int = 1
    max_period: int = 5
    drift_ratio: float = 0.5
    max_translation_m: float = 0.010
    max_incremental_points: int = 200_000
    downsample_voxel: float = 0.003


class IcpTargetCache:
    """ICP target 增量复用缓存：全量基线 + 逐帧增量拼接 + 自适应周期 k.

    生命周期：随节点构造创建、跨会话复用；会话开始/绑定切换/reset/
    remove_last 重放/finalize 清理等关键事件由编排层调 invalidate()
    复位（模型已不存在或已重建，旧 target 一律作废，下帧强制全量刷新）。
    """

    def __init__(self, config: IcpTargetRefreshConfig):
        """保存配置并归一化上下限；初始状态等同 invalidate()（空缓存）."""
        self._config = config
        # 上限小于下限视为配置错误：归一 max>=min>=1 收敛行为，不抛错
        # （启动期参数层不做业务校验是现状语义，此处防御性兜底）
        self._min_period = max(1, int(config.min_period))
        self._max_period = max(self._min_period, int(config.max_period))
        self._drift_thresh = (float(config.max_translation_m)
                              * max(0.0, float(config.drift_ratio)))
        self._max_incremental_points = max(1, int(config.max_incremental_points))
        self._downsample_voxel = float(config.downsample_voxel)
        # 诊断计数（随 diagnostics JSON registration 子对象投影）
        self.full_refreshes = 0
        self.incremental_appends = 0
        self.invalidate()

    def invalidate(self) -> None:
        """关键事件复位：弃缓存、清零计数、自适应状态回保守初值.

        调用后 should_refresh() 恒 True（target 为空），下一次成功采帧
        必走全量 extract；修正量 EMA 与周期一并复位——新会话/新模型的
        漂移历史不遗传。
        """
        self._target: Optional[np.ndarray] = None
        self._frames_since_refresh = 0
        self._period = self._min_period  # 起步保守：无修正量证据前逐帧全量
        self._corr_ema: Optional[float] = None

    @property
    def period(self) -> int:
        """当前自适应刷新周期 k [帧]（诊断观测用）."""
        return self._period

    @property
    def target_size(self) -> int:
        """当前缓存 target 点数；空缓存为 0（诊断观测用）."""
        return 0 if self._target is None else int(self._target.shape[0])

    def current_target(self) -> Optional[np.ndarray]:
        """返回当前可复用 target（base 系 (N,3) [m]）；空缓存给 None."""
        return self._target

    def should_refresh(self) -> bool:
        """是否应从 TSDF 全量 extract 刷新（空缓存或距上次刷新已满 k 帧）."""
        return (self._target is None
                or self._frames_since_refresh >= self._period)

    def set_full(self, xyz: np.ndarray) -> None:
        """以全量 extract 结果重置基线（由 _refresh_tsdf_outputs 单点回调）.

        Args:
            xyz: (N, 3) 全量提取点云（base 系 [m]，已过 ROI/降采样/统计
                滤波后处理）；空数组表示模型尚无点，缓存保持空（下一帧
                should_refresh 仍为 True，持续重试直到模型形成）.

        Returns
        -------
            无返回值（None）；刷新计数归帧零、full_refreshes 递增.

        """
        arr = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        self._target = arr if arr.shape[0] else None
        self._frames_since_refresh = 0
        self.full_refreshes += 1

    def append_frame(self, cloud_base: np.ndarray) -> None:
        """非刷新帧：把本帧 ICP 修正后的 base 系点云增量并入 target.

        仅应在 should_refresh() 为 False 的成功采帧路径调用；拼接结果
        超过 max_incremental_points 时按 downsample_voxel 体素降采样
        （与提取后处理同实现），防 k 偏大时 target 无界增长拖慢 ICP
        内部降采样/KDTree/法向估计。空云帧仍推进刷新计数（帧确已积分）。

        Args:
            cloud_base: (N, 3) 本帧修正后点云（base 系 [m]，已 ROI 裁剪）.

        Returns
        -------
            无返回值（None）.

        """
        pts = np.asarray(cloud_base, dtype=np.float64).reshape(-1, 3)
        if self._target is None:
            # 防御性兜底：正常流程 target 为空时 should_refresh 恒 True，
            # 不会走到本路径；若编排层未来调整调用序，按全量基线处理
            self.set_full(pts)
            return
        if pts.shape[0]:
            self._target = np.vstack((self._target, pts))
        self._frames_since_refresh += 1
        self.incremental_appends += 1
        if self._target.shape[0] > self._max_incremental_points:
            self._target, _ = LocalTsdf.voxel_downsample(
                self._target, None, self._downsample_voxel)

    def note_result(self, mode: str, translation_m: float) -> None:
        """消费一次配准结果，更新修正量 EMA 并自适应伸缩刷新周期 k.

        每帧 refine 后调用（含 fk 回退/拒帧路径）：
          - mode != 'icp'（fk 回退或拒帧）：对齐风险信号，k 立即收回下限
            （下帧尽早全量刷新），且不混入修正量 EMA（fk 的 translation
            恒 0，掺入会虚假拉低 EMA）；
          - mode == 'icp'：平移修正量入 EMA；EMA ≥ 漂移阈值
            （max_translation×drift_ratio）→ k 收回下限；EMA ≤ 1/4×阈值
            → k 拉长到上限；中间迟滞带保持现周期。

        Args:
            mode: IcpResult.mode（'icp'/'fk'/'reject'）.
            translation_m: IcpResult.translation_m（相对 FK 的平移修正 [m]）.

        Returns
        -------
            无返回值（None）.

        """
        if mode != 'icp':
            self._period = self._min_period
            return
        sample = max(0.0, float(translation_m))
        if self._corr_ema is None:
            self._corr_ema = sample
        else:
            self._corr_ema = (_CORR_EMA_ALPHA * sample
                              + (1.0 - _CORR_EMA_ALPHA) * self._corr_ema)
        if self._corr_ema >= self._drift_thresh:
            # 修正量偏大（漂移风险高）→ 缩短 k，更快全量刷新
            self._period = self._min_period
        elif self._corr_ema <= _STABLE_RATIO * self._drift_thresh:
            # 修正量长期远小于漂移阈值（稳定）→ 拉长 k，省全量提取
            self._period = self._max_period
        # 中间区间保持现周期（迟滞带防抖）
