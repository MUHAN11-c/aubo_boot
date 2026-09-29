# -*- coding: utf-8 -*-
"""分割段后端注册表：import 即注册；工厂吃配置 dict."""

from __future__ import annotations

from typing import Callable, Dict, List

from ivg_pose_estimation.pipeline.segmenters.base import RefineResult, Segmenter  # noqa: F401


Factory = Callable[[dict], Segmenter]

_SEGMENTERS: Dict[str, Factory] = {}


def register_segmenter(name: str) -> Callable[[Factory], Factory]:
    def _wrap(factory: Factory) -> Factory:
        if name in _SEGMENTERS:
            raise KeyError(f'分割后端重复注册: {name}')
        _SEGMENTERS[name] = factory
        return factory

    return _wrap


def create_segmenter(name: str, params: dict) -> Segmenter:
    try:
        factory = _SEGMENTERS[name]
    except KeyError:
        raise KeyError(f'未知分割后端 {name!r}，可用: {available_segmenters()}') from None
    return factory(params or {})


def available_segmenters() -> List[str]:
    return sorted(_SEGMENTERS)


# ---- 默认档 ----
from ivg_pose_estimation.pipeline.segmenters.depth_band import DepthBandSegmenter  # noqa: E402,F401
from ivg_pose_estimation.pipeline.segmenters.rembg_u2net import RembgU2NetSegmenter  # noqa: E402,F401


@register_segmenter('depth_band')
def _depth_band(params: dict) -> Segmenter:
    return DepthBandSegmenter()


@register_segmenter('rembg_u2net')
def _rembg_u2net(params: dict) -> Segmenter:
    return RembgU2NetSegmenter(
        model=str(params.get('model', 'u2net')),
        prefer_cuda=bool(params.get('prefer_cuda', True)),
    )


# ---- 前沿档（重型依赖/权重缺失时优雅跳过） ----
try:  # pragma: no cover - 依赖缺失时优雅跳过
    from ivg_pose_estimation.pipeline.segmenters.mobile_sam import MobileSamSegmenter  # noqa: E402,F401

    @register_segmenter('mobile_sam')
    def _mobile_sam(params: dict) -> Segmenter:
        return MobileSamSegmenter(
            weights=str(params.get('weights', '')),
            device=str(params.get('device', 'auto')),
        )
except Exception:  # noqa: BLE001
    pass
