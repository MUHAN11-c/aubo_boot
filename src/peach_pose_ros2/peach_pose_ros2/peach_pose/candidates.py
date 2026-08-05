"""
收敛后的单模式前景估计入口.

项目只保留一条可靠前景路线：``hybrid_dilated`` =
MobileSAM 掩膜 ∩ 膨胀后的实测深度连通域。

位姿几何与刀具安全门控在 ``pipeline.py``，本模块只负责构造前景并路由到
对应管线。GUI 与离线 e2e 都走这一条公共路径。

故意不支持学习深度 / 补全深度：任何掩膜都必须先与有效实测深度求交，
才能进入几何。SAM 缺失时显式 REOBSERVE（``mask_unavailable``），禁止静默回退。
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable, Optional

import cv2
import numpy as np

from .contracts import BagGrasp2D, BagGraspReference3D, BagObservation
from .pipeline import (
    RobustBagPosePipeline, RobustFruitPosePipeline, TargetPoseResult,
)


@dataclass(frozen=True)
class ForegroundMode:
    """前景模式描述（目前仅 hybrid_dilated）."""

    mode_id: str      # 模式 ID（如 'hybrid_dilated'），估计路由键
    label: str        # 中文短标签（报告/界面显示）
    description: str  # 一句话说明


FOREGROUND_MODES = (
    ForegroundMode(
        'hybrid_dilated', 'SAM∩膨胀深度',
        'SAM 掩膜与膨胀后的实测深度连通域求交'),
)
MODE_IDS = tuple(mode.mode_id for mode in FOREGROUND_MODES)
MODE_LABELS = {mode.mode_id: mode.label for mode in FOREGROUND_MODES}


class CandidateEstimator:
    """
    构造收敛前景掩膜，并送入安全管线评估.

    按检测类别分流：
      - ``peach_bag`` (class_id=0) → 圆柱轴袋线 ``RobustBagPosePipeline``
      - ``peach_nobag`` (class_id=1) → 球+梗腔果线 ``RobustFruitPosePipeline``
    两线共用同一圆柱刀具、入口/行程公式与安全门控。
    """

    def __init__(self, pipeline: Optional[RobustBagPosePipeline] = None,
                 dilate_px: int = 5, min_mask_points: int = 50):
        """
        构造估计器；果线复用袋线的 ToolGeometry，保证刀具契约一致.

        Args:
            pipeline: 袋线实例；None 时按默认参数新建（果线共享其 tool）.
            dilate_px: 深度连通域膨胀半径（像素，≥1；核边长 2*(p//2)+1）.
            min_mask_points: 掩膜最小像素数，不足判 mask_unavailable.

        Returns
        -------
            无返回值（None）；果线复用袋线 tool 建于 self.fruit_pipeline.

        """
        self.pipeline = pipeline or RobustBagPosePipeline()
        # 果线复用袋线的 ToolGeometry，保证刀具契约一致
        self.fruit_pipeline = RobustFruitPosePipeline(tool=self.pipeline.tool)
        self.dilate_px = max(1, int(dilate_px))
        self.min_mask_points = max(1, int(min_mask_points))
        self.last_timings_ms: dict[str, float] = {}
        self._last_mask_timings_ms: dict[str, float] = {}

    def _pipeline_for(self, obs: BagObservation) -> RobustBagPosePipeline:
        """
        按首个检测的 class_id 选择袋线 / 果线.

        Args:
            obs: 单帧输入（取 detections[0] 的 class_id；空列表按袋线）.

        Returns
        -------
            class_id==1 → RobustFruitPosePipeline，否则 RobustBagPosePipeline.

        """
        class_id = obs.detections[0].get('class_id', 0) if obs.detections else 0
        return self.fruit_pipeline if class_id == 1 else self.pipeline

    def estimate_modes(self, obs: BagObservation, target_id: str, bbox: tuple,
                       sam_mask: Optional[np.ndarray],
                       modes: Optional[Iterable[str]] = None
                       ) -> dict[str, TargetPoseResult]:
        """
        对请求的前景模式跑同一套几何与安全门控，返回 mode→结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            sam_mask: 全图 SAM 掩膜或 None（None → 各模式 mask_unavailable）.
            modes: 要跑的模式 ID 可迭代；None 跑全部已注册模式；
                含未知 ID 抛 ValueError.

        Returns
        -------
            {mode_id: TargetPoseResult}；掩膜不可用时结果为显式 REOBSERVE；
            副作用：刷新 last_timings_ms（毫秒，含掩膜构造耗时）.

        """
        selected = tuple(modes or MODE_IDS)
        unknown = set(selected) - set(MODE_IDS)
        if unknown:
            raise ValueError(f'unknown foreground modes: {sorted(unknown)}')

        masks = self.build_masks(obs, bbox, sam_mask)
        pipeline = self._pipeline_for(obs)
        kind = 'fruit' if pipeline is self.fruit_pipeline else 'bag'
        results = {}
        self.last_timings_ms = {}
        for mode in selected:
            started = time.perf_counter()
            mask = masks.get(mode)
            if mask is None:
                # SAM 缺失或交后像素不足：显式 REOBSERVE，不走深度-only 回退
                results[mode] = self._unavailable(
                    obs, target_id, bbox, mode, 'mask_unavailable')
            else:
                results[mode] = pipeline.estimate(
                    obs, target_id, bbox, mask, self._source(mode))
            pose = results[mode].grasp_3d
            results[mode].target_kind = kind
            pose.strategy_id = f'robust_{kind}_pose:{mode}'
            pose.model_version = str(obs.metadata.get('model_version', 'unknown'))
            pose.calibration_version = str(obs.metadata.get(
                'calibration_version', 'unknown'))
            pose.tool_version = self.pipeline.tool.version
            geometry_ms = (time.perf_counter() - started) * 1000.0
            # 总耗时 = 掩膜构造 + 本模式几何
            self.last_timings_ms[mode] = (
                self._last_mask_timings_ms.get(mode, 0.0) + geometry_ms)
        return results

    def build_masks(self, obs: BagObservation, bbox: tuple,
                    sam_mask: Optional[np.ndarray]) -> dict[str, Optional[np.ndarray]]:
        """
        在 bbox ROI 内构造 hybrid_dilated 掩膜（实测深度单位：毫米 uint16）.

        hybrid_dilated = (SAM ∩ 有效深度) ∩ 膨胀后的深度连通域；
        交后像素 < min_mask_points 给 None。

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            bbox: (x1, y1, x2, y2) 检测框（像素，自动裁剪到图内）.
            sam_mask: 全图或 ROI 掩膜；None 或裁剪失败则结果为 None.

        Returns
        -------
            {mode_id: (h, w) bool ROI 掩膜或 None}；副作用：刷新
            _last_mask_timings_ms（毫秒）.

        """
        started = time.perf_counter()
        self._last_mask_timings_ms = {mode: 0.0 for mode in MODE_IDS}
        x1, y1, x2, y2 = self.pipeline._clip_bbox(bbox, obs.depth.shape)
        roi = obs.depth[y1:y2, x1:x2]
        if roi.size == 0:
            return {mode: None for mode in MODE_IDS}
        valid = self.pipeline._valid_depth(roi)
        # 深度连通前景：作为「膨胀母体」，限制 SAM 不漂到背景
        depth_mask, _ = self.pipeline._foreground(
            roi, valid, None, bbox, source='depth_fallback')

        mask = None
        sam_roi = self._crop_mask(sam_mask, (x1, y1, x2, y2), obs.depth.shape)
        if sam_roi is not None:
            # 只保留有实测深度的 SAM 像素
            measured_sam = sam_roi & valid
            k = 2 * (self.dilate_px // 2) + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            expanded_depth = cv2.dilate(depth_mask.astype(np.uint8), kernel) > 0
            # SAM ∩ 膨胀深度；像素过少则视为不可用
            mask = self._enough(measured_sam & expanded_depth)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._last_mask_timings_ms = {mode: elapsed_ms for mode in MODE_IDS}
        return {'hybrid_dilated': mask}

    def _crop_mask(self, mask: Optional[np.ndarray], bbox: tuple,
                   image_shape: tuple) -> Optional[np.ndarray]:
        """
        把全图或 ROI 掩膜裁成与 bbox 同尺寸；尺寸不符返回 None.

        Args:
            mask: bool/0-1 掩膜（全图尺寸则裁 ROI）；None 原样返回 None.
            bbox: (x1, y1, x2, y2) 已裁剪到图内的整数框（像素）.
            image_shape: 全图 shape（判全图/ROI 用）.

        Returns
        -------
            (y2-y1, x2-x1) bool 掩膜；尺寸对不上给 None.

        """
        if mask is None:
            return None
        x1, y1, x2, y2 = bbox
        arr = np.asarray(mask, dtype=bool)
        if arr.shape[:2] == image_shape[:2]:
            arr = arr[y1:y2, x1:x2]
        expected = (y2 - y1, x2 - x1)
        return arr if arr.shape == expected else None

    def _enough(self, mask: np.ndarray) -> Optional[np.ndarray]:
        """
        像素数不足 min_mask_points 时丢弃（触发 mask_unavailable）.

        Args:
            mask: (h, w) bool 掩膜.

        Returns
        -------
            原掩膜或 None.

        """
        return mask if int(mask.sum()) >= self.min_mask_points else None

    @staticmethod
    def _source(mode: str) -> str:
        """
        写入结果的 mask_source 标签（便于诊断追溯）.

        Args:
            mode: 已注册模式 ID（未知 ID 抛 KeyError）.

        Returns
        -------
            来源标签字符串.

        """
        return {
            'hybrid_dilated': 'mobile_sam_dilated_depth_intersection',
        }[mode]

    def _unavailable(self, obs: BagObservation, target_id: str, bbox: tuple,
                     mode: str, reason: str) -> TargetPoseResult:
        """
        构造显式失败结果（REOBSERVE + diagnostic_flags）.

        Args:
            obs: 单帧输入（取版本元数据）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mode: 前景模式 ID（写入 strategy_id）.
            reason: 原因标记（如 'mask_unavailable'）.

        Returns
        -------
            TargetPoseResult（status=REOBSERVE，metrics 为空）.

        """
        x1, y1, x2, y2 = map(int, bbox)
        g2d = BagGrasp2D(
            detection_bbox=(x1, y1, x2 - x1, y2 - y1),
            status='REOBSERVE', diagnostic_flags=[reason])
        g3d = BagGraspReference3D(
            status='REOBSERVE', diagnostic_flags=[reason],
            strategy_id=f'robust_bag_pose:{mode}',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.pipeline.tool.version)
        return TargetPoseResult(target_id, g2d, g3d, mode, {})
