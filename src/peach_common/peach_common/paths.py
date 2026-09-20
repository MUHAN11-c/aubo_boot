"""路径工具（W1 单源）：目录段净化。

消息来源的 run_id / target_id 会拼进落盘路径（runs/&lt;request_id&gt;/…），
三包此前各持口径不一的净化（supervisor `_safe_run_component` 不滤 NUL/纯点；
vision 侧无净化）。本模块是唯一实现：拒绝分隔符、上跳、NUL、空与纯点，
非法值回退 fallback 目录段（折叠隔离，不拒绝整批——与账本侧既有语义一致）。
"""
from __future__ import annotations

_INVALID = {'', '.', '..'}
_FORBIDDEN_CHARS = ('/', '\\', '\x00')


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
