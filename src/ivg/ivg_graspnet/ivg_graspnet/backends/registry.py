# -*- coding: utf-8 -*-
"""后端注册表：节点按 `backend` 参数名取工厂构建后端实例.

工厂签名统一为 factory(params: dict) -> GraspBackend，params 是节点
declare_parameter 的扁平字典——各后端自取所需键（model_path/device 等），
互不干扰。新增后端：在 backends/ 下写实现模块 + @register('名字') 工厂，
并在 backends/__init__.py import 一次。
"""

from __future__ import annotations

from typing import Callable, Dict, List

from ivg_graspnet.backends.base import GraspBackend

Factory = Callable[[dict], GraspBackend]

_REGISTRY: Dict[str, Factory] = {}


def register(name: str) -> Callable[[Factory], Factory]:
    """类装饰器：注册后端工厂."""

    def _wrap(factory: Factory) -> Factory:
        if name in _REGISTRY:
            raise KeyError(f'后端名重复注册: {name}')
        _REGISTRY[name] = factory
        return factory

    return _wrap


def create_grasp_backend(name: str, params: dict) -> GraspBackend:
    """按名构建后端；未知名抛 KeyError（节点启动即失败，防带错运行）."""
    try:
        factory = _REGISTRY[name]
    except KeyError:
        raise KeyError(
            f'未知抓取后端 {name!r}，可用: {sorted(_REGISTRY)}'
        ) from None
    return factory(params)


def available_backends() -> List[str]:
    """已注册后端名列表."""
    return sorted(_REGISTRY)
