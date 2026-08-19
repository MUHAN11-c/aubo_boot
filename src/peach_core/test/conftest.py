"""
pytest 路径引导：未 colcon 构建时也能直接从源码树 import peach_core.

colcon test 经 install 空间提供包；裸 pytest（AGENTS.md 第 5 节的 venv
直跑方式）从源码树运行时需要把包根加入 sys.path。
"""

from pathlib import Path
import sys

_PKG_PARENT = str(Path(__file__).resolve().parents[1])
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)
