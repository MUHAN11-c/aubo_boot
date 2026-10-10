# -*- coding: utf-8 -*-
"""抓取检测后端集合：import 即注册（registry）.

新增后端在下方追加 import；重型依赖（缺 torch/权重等）导致 import 失败的
可选后端用 try/except 包住——注册表里少一个可选项比整个包不可 import 好。
"""

from ivg_graspnet.backends.graspnet_torch import GraspNetTorchBackend  # noqa: F401
from ivg_graspnet.backends.registry import (  # noqa: F401
    available_backends,
    create_grasp_backend,
    register,
)

# contact_graspnet 后端（vendored，可选）
try:  # pragma: no cover - 依赖/权重缺失时优雅跳过
    from ivg_graspnet.backends import contact_graspnet  # noqa: F401
except Exception:  # noqa: BLE001 - import 期不炸，注册表里少一项
    pass
