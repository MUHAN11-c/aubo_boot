"""
显式注册清单 — 默认实现按名登记到接口层注册表（2.14 装配规则）.

职责:
  六类可替换组件的默认实现在此集中登记（一处 import 即完成全部注册）：

  ================  ====================  ==========================
  注册表            注册名                默认实现
  ================  ====================  ==========================
  DETECTORS         'yolo'                UltralyticsYolo
  SEGMENTERS        'mobile_sam'          MobileSam
  POSE_PIPELINES    'robust_bag'          RobustBagPosePipeline
  POSE_PIPELINES    'robust_fruit'        RobustFruitPosePipeline
  MATCHERS          'spatial_ema'         SpatialEmaMatcher
  LOCK_POLICIES     'collect_lock'        CollectLockPolicy
  ================  ====================  ==========================

  暂不用 ``@REGISTRY.register`` 自注册装饰器：注册面集中一处便于审阅
  装配关系，也避免「import 副作用隐式决定可用实现」；后续实现增多再
  评估迁移。yaml ``*.impl`` 参数的值即本清单的注册名。

  调用端（candidates 默认构造、peach_scene_perception_node 装配、测试）import 本
  模块即保证注册完成；接口层 interfaces.py 不 import 本模块，避免
  interfaces ↔ 实现 循环 import。

协议条款:
  纯核零 ROS import（test_pure_core.py AST 强制）。
"""
from __future__ import annotations

from .harvest_plan import CollectLockPolicy
from .inference import MobileSam, UltralyticsYolo
from .interfaces import (
    DETECTORS,
    LOCK_POLICIES,
    MATCHERS,
    POSE_PIPELINES,
    SEGMENTERS,
)
from .pipeline import RobustBagPosePipeline, RobustFruitPosePipeline
from .target_registry import SpatialEmaMatcher

DETECTORS.register('yolo', UltralyticsYolo)
SEGMENTERS.register('mobile_sam', MobileSam)
POSE_PIPELINES.register('robust_bag', RobustBagPosePipeline)
POSE_PIPELINES.register('robust_fruit', RobustFruitPosePipeline)
MATCHERS.register('spatial_ema', SpatialEmaMatcher)
LOCK_POLICIES.register('collect_lock', CollectLockPolicy)
