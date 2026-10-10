# -*- coding: utf-8 -*-
"""
推理会话：统一 device 解析、fp16 autocast 与显存观测.

后端经 InferenceSession 拿设备与 autocast 上下文，避免各后端自行散落
`cuda.is_available()` 判断；torch 延迟导入保证无 GPU 环境可 import 本模块。
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass


@dataclass
class SessionConfig:
    """会话配置."""

    device: str = 'auto'  # 'auto' | 'cuda[:idx]' | 'cpu'
    fp16: bool = False    # 仅 cuda 生效（autocast 半精度前向）


class InferenceSession:
    """一次推理会话：设备解析 + autocast 上下文 + 显存摘要."""

    def __init__(self, config: SessionConfig):
        self.config = config
        self.device = self._resolve_device()

    # ------------------------------------------------------------------
    def _resolve_device(self):
        import torch

        spec = self.config.device
        if spec in ('', 'auto'):
            return torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        return torch.device(spec)

    def autocast(self):
        """前向上下文：cuda 且 fp16 开启时半精度，否则空上下文."""
        if self.config.fp16 and self.device.type == 'cuda':
            import torch

            return torch.autocast('cuda', dtype=torch.float16)
        return contextlib.nullcontext()

    def summary(self) -> str:
        """人读摘要（节点启动/日志用）."""
        return f'device={self.device}, fp16={self.config.fp16 and self.device.type == "cuda"}'

    def memory_summary(self) -> str:
        """显存占用摘要；非 cuda 返回 'n/a'."""
        if self.device.type != 'cuda':
            return 'n/a'
        torch = __import__('torch')
        alloc = torch.cuda.memory_allocated() / (1024 ** 2)
        peak = torch.cuda.max_memory_allocated() / (1024 ** 2)
        return f'alloc={alloc:.0f}MiB, peak={peak:.0f}MiB'
