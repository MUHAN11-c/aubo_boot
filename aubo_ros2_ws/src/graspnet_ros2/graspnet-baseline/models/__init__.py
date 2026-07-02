# -*- coding: utf-8 -*-
"""GraspNet 模型包。

上游代码使用 "from backbone import ..."（绝对导入）而非 "from .backbone import ..."（相对导入），
需要 models/ 自身在 sys.path 中才能解析内部模块。pip install -e . 仅保证 graspnet-baseline/ 在
sys.path，models/ 不会自动加入——此处补上。
"""
import os as _os
import sys as _sys

_models_dir = _os.path.dirname(_os.path.abspath(__file__))
if _models_dir not in _sys.path:
    _sys.path.insert(0, _models_dir)
