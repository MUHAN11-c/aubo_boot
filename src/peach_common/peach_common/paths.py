"""
路径工具（W1/W6-B 单源）：目录段净化与 runs 根目录解析.

消息来源的 run_id / target_id 会拼进落盘路径（runs/&lt;request_id&gt;/…），
三包此前各持口径不一的净化（supervisor `_safe_run_component` 不滤 NUL/纯点；
vision 侧无净化）。本模块是唯一实现：拒绝分隔符、上跳、NUL、空与纯点，
非法值回退 fallback 目录段（折叠隔离，不拒绝整批——与账本侧既有语义一致）。

runs_root（W6-B）归一 supervisor/batch 与 vision/common/runtime 两份同名
实现：env 键取两者并集（AUBO_RUNS_DIR 优先于 AUBO_HARVEST_DATA_DIR），
探测序=从本文件向上找含 ``src/peach_interfaces`` 的工作区标记 → cwd。
"""
from __future__ import annotations

import os
from pathlib import Path

_INVALID = {'', '.', '..'}
_FORBIDDEN_CHARS = ('/', '\\', '\x00')

_WORKSPACE_MARKER = Path('src') / 'peach_interfaces'
"""工作区根标记：仓库布局 src/&lt;pkg&gt; 下的接口包目录."""


def safe_component(name, fallback: str) -> str:
    """
    净化单个目录段；非法回退 fallback.

    Args:
        name: 任意来源的 id 字符串（消息/参数/调用方拼接）.
        fallback: 非法时使用的目录段（自身须合法，否则进一步退 'unknown'）.

    Returns
    -------
        可安全用作 ``Path(base) / result`` 的单段字符串.

    """
    text = str(name or '').strip()
    if text in _INVALID or any(ch in text for ch in _FORBIDDEN_CHARS):
        text = str(fallback or '').strip()
        if text in _INVALID or any(ch in text for ch in _FORBIDDEN_CHARS):
            return 'unknown'
    return text


def runs_root(configured: str = '') -> Path:
    """
    过程数据根目录（工作区 ``runs/``）唯一解析.

    优先级：configured 为绝对路径 > 环境变量（AUBO_RUNS_DIR 优先于
    AUBO_HARVEST_DATA_DIR）> 从本文件向上找含 ``src/peach_interfaces``
    的工作区根，取其 ``runs/`` > ``Path.cwd()/runs`` 兜底。与归一前
    supervisor/batch 与 vision/common/runtime 两实现语义一致（并集）。

    Args:
        configured: 参数/yaml 给出的候选路径；相对路径不生效（回默认）.

    Returns
    -------
        runs 根目录 Path（不保证已存在）.

    """
    text = str(configured or '').strip()
    if text:
        path = Path(text)
        if path.is_absolute():
            return path
    override = os.environ.get('AUBO_RUNS_DIR') or os.environ.get(
        'AUBO_HARVEST_DATA_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / _WORKSPACE_MARKER).is_dir():
            return parent / 'runs'
    return Path.cwd() / 'runs'
