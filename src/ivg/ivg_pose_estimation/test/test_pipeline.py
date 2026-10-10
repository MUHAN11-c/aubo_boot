# -*- coding: utf-8 -*-
"""
零 ROS 纯核测试：流水线分段（Segmenter/Matcher）、单位单源与调试参数路由.

不 import rclpy、不加载模型权重。
"""

from __future__ import annotations

from types import SimpleNamespace

from ivg_pose_estimation.config import apply_debug_param, load_config
import numpy as np
import pytest

from ivg_pose_estimation.pipeline import (
    PosePipeline,
    available_matchers,
    available_segmenters,
)
from ivg_pose_estimation.pipeline.matchers import create_matcher
from ivg_pose_estimation.pipeline.matchers.geometric import GeometricMatcher
from ivg_pose_estimation.pipeline.pose_solver import resolve_depth_scale
from ivg_pose_estimation.pipeline.segmenters import create_segmenter
from ivg_pose_estimation.pipeline.segmenters.depth_band import DepthBandSegmenter
from ivg_pose_estimation.pipeline.segmenters.rembg_u2net import (
    RembgU2NetSegmenter,
    resolve_roi_bbox,
)


class _StubEstimator:
    """GeometricMatcher 的委托桩：记录调用并返回可辨识结果."""

    brute_force_matching_enabled = False

    def __init__(self):
        self.called_with = None

    def select_best_template(self, feature, target_mask=None, workpiece_template_dir=''):
        self.called_with = (feature, target_mask, workpiece_template_dir)
        return (2, 0.123, 0.88, -15.0, np.zeros((3, 3), np.uint8))


class _StubReader:
    """ConfigReader 桩：返回固定段字典."""

    def __init__(self, sections):
        self._sections = sections

    def get_section(self, section):
        return dict(self._sections.get(section, {}))


def test_depth_band_passthrough():
    mask = np.zeros((6, 8), np.uint8)
    mask[2:4, 3:6] = 255
    result = DepthBandSegmenter().refine(None, mask)
    assert result.mask is not None and result.mask.shape == mask.shape
    assert np.array_equal(result.mask, mask)
    assert result.cutout is None
    assert DepthBandSegmenter().refine(None, None).mask is None


def test_rembg_segmenter_degrades_without_roi():
    seg = RembgU2NetSegmenter()
    result = seg.refine(np.zeros((8, 8, 3), np.uint8), None, feature=None, key=0)
    assert result.mask is None and result.cutout is None


def test_resolve_roi_bbox_prefers_feature_circle():
    feature = SimpleNamespace(workpiece_center=(5, 5), workpiece_radius=2)
    assert resolve_roi_bbox(feature, None) == (3, 3, 4, 4)
    mask = np.zeros((10, 10), np.uint8)
    mask[4:6, 2:5] = 255
    assert resolve_roi_bbox(None, mask) == (2, 4, 3, 2)


def test_geometric_matcher_delegates_and_wraps():
    stub = _StubEstimator()
    matcher = GeometricMatcher(stub, mode='brute_force')
    assert stub.brute_force_matching_enabled is True
    assert matcher.mode_label == 'geometric/brute_force'

    outcome = matcher.match('feat', target_mask=None, templates=[],
                            workpiece_template_dir='/tmp/t')
    assert stub.called_with[0] == 'feat'
    assert outcome.best_idx == 2
    assert outcome.distance == pytest.approx(0.123)
    assert outcome.confidence == pytest.approx(0.88)
    assert outcome.best_angle_deg == pytest.approx(-15.0)

    GeometricMatcher(stub, mode='distance')
    assert stub.brute_force_matching_enabled is False
    with pytest.raises(ValueError):
        GeometricMatcher(stub, mode='nope')


def test_registry_rejects_unknown_names():
    with pytest.raises(KeyError, match='未知分割后端'):
        create_segmenter('no_such', {})
    with pytest.raises(KeyError, match='未知匹配后端'):
        create_matcher('no_such', {}, _StubEstimator())
    assert 'depth_band' in available_segmenters()
    assert 'geometric' in available_matchers()


def test_pose_pipeline_from_config_selection():
    stub = _StubEstimator()
    reader = _StubReader({'segmenter': {'backend': 'depth_band'},
                          'matcher': {'backend': 'geometric', 'mode': 'auto'}})
    pipeline = PosePipeline.from_config(reader, stub)
    assert pipeline.segmenter.name == 'depth_band'
    assert pipeline.matcher.name == 'geometric'

    # use_rembg 调试覆盖：True→rembg_u2net，False→回配置档
    assert pipeline.active_segmenter_name(use_rembg=True) == 'rembg_u2net'
    assert pipeline.active_segmenter_name(use_rembg=False) == 'depth_band'
    assert pipeline.active_segmenter_name(None) == 'depth_band'


def test_depth_scale_single_source():
    assert resolve_depth_scale({}) == pytest.approx(0.00025)
    assert resolve_depth_scale({'depth_scale': 0.001}) == pytest.approx(0.001)
    assert resolve_depth_scale({'depth_scale': -1}) == pytest.approx(0.00025)
    assert resolve_depth_scale(None) == pytest.approx(0.00025)


def test_debug_param_routing():
    assert apply_debug_param('use_rembg', True) == ['rembg']
    assert apply_debug_param('component_min_area', 5) == [
        'preprocessor', 'feature_extractor'
    ]
    assert apply_debug_param('binary_threshold_min', 1800) == ['preprocessor']


def test_config_single_source_loads_backends():
    cfg = load_config()
    assert cfg.segmenter.get('backend') == 'depth_band'
    assert cfg.matcher.get('backend') == 'geometric'
    assert cfg.camera.get('depth_scale') == pytest.approx(0.00025)
    assert cfg.calibration.get('translation_unit') == 'auto'
