# Copyright 2026 wjz
"""Web 控制请求的纯校验与命名参数档案持久化。."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import tempfile


class CommandGuard:
    """在 HTTP 线程进入 ROS 服务前执行无状态安全校验。."""

    MAINTENANCE_MODE = 2

    def __init__(self, nonce: str):
        """保存进程级随机 nonce。."""
        self._nonce = nonce

    def authorize(
            self, nonce: str, expected_revision: int,
            current_revision: int, *, maintenance_required: bool,
            operation_mode: int = 0) -> tuple[bool, str]:
        """验证会话、乐观锁和维护模式所有权。."""
        if nonce != self._nonce:
            return False, '会话 nonce 无效'
        if expected_revision != current_revision:
            return False, '状态版本已过期'
        if maintenance_required and operation_mode != self.MAINTENANCE_MODE:
            return False, '手动调试仅在维护模式可用'
        return True, ''


class ProfileStore:
    """在一个固定目录内原子保存 JSON 参数档案。."""

    _VALID_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')

    def __init__(self, root: Path):
        """记录档案根目录，首次保存时再创建。."""
        self._root = Path(root)

    @classmethod
    def _validate_name(cls, name: str) -> None:
        if not cls._VALID_NAME.fullmatch(name):
            raise ValueError('档案名仅允许字母、数字、点、下划线和连字符')

    def _path(self, name: str) -> Path:
        self._validate_name(name)
        return self._root / f'{name}.json'

    def list_names(self) -> list[str]:
        """按名称排序返回全部档案。."""
        if not self._root.exists():
            return []
        return sorted(path.stem for path in self._root.glob('*.json'))

    def load(self, name: str) -> dict:
        """读取并验证档案顶层类型。."""
        value = json.loads(self._path(name).read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            raise ValueError('参数档案顶层必须是对象')
        return value

    def save(self, name: str, values: dict) -> None:
        """同目录临时文件加 replace，避免断电留下半份 JSON。."""
        if not isinstance(values, dict):
            raise ValueError('参数档案顶层必须是对象')
        target = self._path(name)
        self._root.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f'.{name}.', suffix='.tmp', dir=self._root)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(values, stream, ensure_ascii=False, indent=2,
                          sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
