"""
TargetReconstructionParams：GPL 快照之上的薄包装（strip 派生）.

声明 / 类型 / 默认值 / 中文描述 / 范围校验的权威源是
config/target_reconstruction_parameters.yaml（根键=节点名）。
本模块不再照抄嵌套 dataclass：访问语法
``self.params.capture.min_views`` 仍成立（转发到生成结构）；
仅 ``frames.base_frame`` / ``session.root_dir`` 在装载时 strip。
"""
from __future__ import annotations


class _StripProxy:
    """转发嵌套组，覆盖已 strip 的字符串字段."""

    def __init__(self, raw, stripped: dict):
        """Raw GPL nested group; stripped maps field names to whitespace-trimmed values."""
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, '_stripped', stripped)

    def __getattr__(self, name):
        """优先返回 strip 派生，其余转发生成组."""
        if name in self._stripped:
            return self._stripped[name]
        return getattr(self._raw, name)


class TargetReconstructionParams:
    """GPL Params 快照；frames / session 带 strip 派生，其余属性转发."""

    def __init__(self, raw):
        """持有生成快照；frames.base_frame 与 session.root_dir 已 strip."""
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(
            self, 'frames',
            _StripProxy(raw.frames, {
                'base_frame': raw.frames.base_frame.strip()}))
        object.__setattr__(
            self, 'session',
            _StripProxy(raw.session, {
                'root_dir': raw.session.root_dir.strip()}))

    def __getattr__(self, name):
        """未覆盖的属性转发到 GPL Params（含 capture / refit / refitter）."""
        return getattr(self._raw, name)

    @staticmethod
    def declare(node) -> object:
        """
        生成 ParamListener 并集中声明全部参数.

        Args:
            node: rclpy Node（声明参数+挂 on_set 校验）.

        Returns
        -------
            ParamListener.

        """
        from peach_perception.target_reconstruction_parameters import (
            peach_target_reconstruction_node)
        return peach_target_reconstruction_node.ParamListener(node)

    @classmethod
    def from_params(cls, p) -> 'TargetReconstructionParams':
        """
        包装生成快照；字符串 strip 在 frames / session 代理上完成.

        Args:
            p: peach_target_reconstruction_node.Params.

        Returns
        -------
            TargetReconstructionParams.

        """
        return cls(p)
