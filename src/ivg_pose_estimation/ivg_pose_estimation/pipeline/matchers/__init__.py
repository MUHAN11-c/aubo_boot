# -*- coding: utf-8 -*-
"""匹配段后端注册表：import 即注册；工厂吃配置 dict + PoseEstimator."""

from __future__ import annotations

from typing import Any, Callable, Dict, List

from ivg_pose_estimation.pipeline.matchers.base import Matcher, MatchOutcome  # noqa: F401

Factory = Callable[[dict, Any], Matcher]

_MATCHERS: Dict[str, Factory] = {}


def register_matcher(name: str) -> Callable[[Factory], Factory]:
    def _wrap(factory: Factory) -> Factory:
        if name in _MATCHERS:
            raise KeyError(f'匹配后端重复注册: {name}')
        _MATCHERS[name] = factory
        return factory

    return _wrap


def create_matcher(name: str, params: dict, pose_estimator) -> Matcher:
    try:
        factory = _MATCHERS[name]
    except KeyError:
        raise KeyError(f'未知匹配后端 {name!r}，可用: {available_matchers()}') from None
    return factory(params or {}, pose_estimator)


def available_matchers() -> List[str]:
    return sorted(_MATCHERS)


from ivg_pose_estimation.pipeline.matchers.geometric import GeometricMatcher  # noqa: E402,F401


@register_matcher('geometric')
def _geometric(params: dict, pose_estimator) -> Matcher:
    return GeometricMatcher(
        pose_estimator, mode=str(params.get('mode', 'auto'))
    )


# ---- 前沿档（重型依赖/索引构建失败时优雅跳过注册） ----
try:  # pragma: no cover - 依赖缺失时优雅跳过
    from ivg_pose_estimation.pipeline.matchers.dinov2_template import Dinov2TemplateMatcher  # noqa: E402,F401

    @register_matcher('dinov2_template')
    def _dinov2_template(params: dict, pose_estimator) -> Matcher:
        return Dinov2TemplateMatcher(
            pose_estimator,
            model=str(params.get('model', 'dinov2_vits14')),
            device=str(params.get('device', 'auto')),
            top_k=int(params.get('top_k', 1)),
            min_similarity=float(params.get('min_similarity', 0.5)),
        )
except Exception:  # noqa: BLE001
    pass
