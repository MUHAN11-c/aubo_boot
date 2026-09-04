"""
ScenePerceptionParams：GPL 快照之上的派生层（gravity 解析 + ToolGeometry）.

声明 / 类型 / 默认值 / 中文描述 / 范围校验的权威源是
config/scene_perception_parameters.yaml（根键=节点名），由
generate_parameter_library_py 生成 peach_perception/scene_perception_parameters.py。
本模块不再照抄 57 个标量字段：其余属性转发到生成嵌套结构
（``self.params.pipeline.bag_impl`` / ``self.params.target_memory.enable``）。
静态装载：启动期一次，不做动态改参回调。
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from peach_perception.scene_perception.contracts import ToolGeometry


class ScenePerceptionParams:
    """
    GPL Params 快照 + tool / gravity_hint 派生.

    标量与分组字段转发到生成结构；tool 为组装后的 ToolGeometry
    （entry_standoff = entry_d_tool + entry_d_s）；gravity_hint 为解析后的
    ndarray 或 None；gravity_mode 经白名单回退。
    """

    def __init__(self, raw, tool: ToolGeometry, gravity_hint, gravity_mode: str):
        """持有生成快照与派生字段（不拷贝标量）."""
        object.__setattr__(self, '_raw', raw)
        self.tool = tool
        self.gravity_hint = gravity_hint
        object.__setattr__(self, '_gravity_mode', gravity_mode)

    def __getattr__(self, name):
        """未覆盖的属性转发到 GPL Params（含 pipeline / target_memory 等嵌套组）."""
        if name == 'gravity_mode':
            return self._gravity_mode
        return getattr(self._raw, name)

    @staticmethod
    def declare(node) -> object:
        """
        生成 ParamListener 并集中声明全部参数.

        Args:
            node: rclpy Node（声明参数+挂 on_set 校验）.

        Returns
        -------
            ParamListener：调用方持有并用于读取 Params 快照.

        """
        from peach_perception.scene_perception_parameters import (
            peach_scene_perception_node)
        return peach_scene_perception_node.ParamListener(node)

    @classmethod
    def from_params(cls, p) -> 'ScenePerceptionParams':
        """
        从生成的 Params 快照解析 gravity、组装 ToolGeometry.

        gravity_hint_xyz 非空必须恰 3 个逗号分隔浮点（否则 ValueError）；
        gravity_mode 不在 {fixed, tf} 告警并回退 fixed；
        tool.entry_standoff = entry_d_tool + entry_d_s。

        Args:
            p: peach_scene_perception_node.Params（declare 后 get_params()）.

        Returns
        -------
            ScenePerceptionParams.

        """
        gh_s: str = p.gravity_hint_xyz.strip()
        if gh_s:
            parts = [float(x) for x in gh_s.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    f'gravity_hint_xyz needs 3 comma-separated floats, got {gh_s!r}')
            gravity_hint: Optional[np.ndarray] = np.asarray(parts, dtype=float)
        else:
            gravity_hint = None
        gravity_mode: str = p.gravity_mode.strip()
        if gravity_mode not in ('fixed', 'tf'):
            import rclpy.logging
            rclpy.logging.get_logger('peach_scene_perception_node').warning(
                f'未知 gravity_mode={gravity_mode!r}，回退 fixed')
            gravity_mode = 'fixed'
        tool = ToolGeometry(
            d_inner_m=float(p.tool.D_inner),
            insert_length_m=float(p.tool.L_insert),
            blade_offset_m=float(p.tool.L_blade),
            entry_d_tool=float(p.tool.entry_d_tool),
            entry_d_s=float(p.tool.entry_d_s),
            entry_standoff=float(p.tool.entry_d_tool)
            + float(p.tool.entry_d_s),
            clearance_min=float(p.tool.clearance_min),
            margin_neck=float(p.tool.margin_neck),
            version=str(p.tool.version),
        )
        return cls(p, tool, gravity_hint, gravity_mode)
