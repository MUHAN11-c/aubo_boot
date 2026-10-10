# -*- coding: utf-8 -*-
"""
位姿估计流水线门面：segmenter → matcher → pose 求解 一处编排（纯核）.

节点（ros2_communication）与 Web 进程（web/services/native_api 调试预览）
都经 PosePipeline 取分割/匹配段实例，消除双进程各持一套算法栈的漂移。
位姿合成段实现仍是 PoseEstimator.estimate_pose（见 pose_solver.py 的单位
单源说明）。

配置（config/pose_estimation.yaml 单一事实源）：
  segmenter.backend: depth_band | rembg_u2net | mobile_sam
  matcher.backend:   geometric | dinov2_template
Web 调试面板的 use_rembg 开关经 select_segmenter(use_rembg=...) 覆盖。
"""

from __future__ import annotations

from typing import Optional

from ivg_pose_estimation.pipeline.matchers import (
    available_matchers,
    create_matcher,
)
from ivg_pose_estimation.pipeline.matchers.geometric import GeometricMatcher
from ivg_pose_estimation.pipeline.pose_solver import resolve_depth_scale
from ivg_pose_estimation.pipeline.segmenters import (
    available_segmenters,
    create_segmenter,
)
from ivg_pose_estimation.pipeline.segmenters.rembg_u2net import RembgU2NetSegmenter


class PosePipeline:
    """分段流水线门面：持有当前配置选定的 segmenter / matcher 实例."""

    def __init__(self, segmenter, matcher):
        self.segmenter = segmenter
        self.matcher = matcher

    # ------------------------------------------------------------------
    @classmethod
    def from_config(cls, config_reader, pose_estimator) -> 'PosePipeline':
        """按配置构建（config_reader=ConfigReader；pose_estimator=PoseEstimator）."""
        seg_cfg = config_reader.get_section('segmenter')
        mat_cfg = config_reader.get_section('matcher')
        seg_name = str(seg_cfg.get('backend', 'depth_band'))
        mat_name = str(mat_cfg.get('backend', 'geometric'))
        segmenter = create_segmenter(seg_name, seg_cfg)
        matcher = create_matcher(mat_name, mat_cfg, pose_estimator)
        return cls(segmenter, matcher)

    # ------------------------------------------------------------------
    def select_segmenter(self, use_rembg: Optional[bool] = None):
        """
        按调试开关选段实例：True→rembg_u2net，False→depth_band，
        None→配置默认。rembg 实例惰性常驻（处理器加载一次）。
        """
        if use_rembg is True:
            if not isinstance(self.segmenter, RembgU2NetSegmenter):
                if not hasattr(self, '_rembg_segmenter'):
                    self._rembg_segmenter = RembgU2NetSegmenter()
                return self._rembg_segmenter
            return self.segmenter
        if use_rembg is False:
            if isinstance(self.segmenter, RembgU2NetSegmenter):
                if not hasattr(self, '_depth_band_segmenter'):
                    from ivg_pose_estimation.pipeline.segmenters.depth_band import (
                        DepthBandSegmenter,
                    )

                    self._depth_band_segmenter = DepthBandSegmenter()
                return self._depth_band_segmenter
        return self.segmenter

    def active_segmenter_name(self, use_rembg: Optional[bool] = None) -> str:
        return self.select_segmenter(use_rembg).name

    # ------------------------------------------------------------------
    def begin_request(self) -> None:
        """一次 estimate_pose 请求开始：清各段缓存."""
        self.segmenter.clear_cache()
        if hasattr(self, '_rembg_segmenter'):
            self._rembg_segmenter.clear_cache()

    # ------------------------------------------------------------------
    def describe(self) -> str:
        matcher_label = getattr(self.matcher, 'mode_label', self.matcher.name)
        return (
            f'segmenter={self.segmenter.name}, matcher={matcher_label} '
            f'(可选分割档: {available_segmenters()}；匹配档: {available_matchers()})'
        )

    @staticmethod
    def depth_scale(config_reader) -> float:
        """深度原始值→米的换算系数（camera.depth_scale 单源）."""
        return resolve_depth_scale(config_reader.get_section('camera'))


__all__ = [
    'GeometricMatcher',
    'PosePipeline',
    'available_matchers',
    'available_segmenters',
    'resolve_depth_scale',
]
