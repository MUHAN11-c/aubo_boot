"""Scene perception parameters backed by generate_parameter_library."""
from __future__ import annotations

from functools import partial

import numpy as np
from peach_scene_perception.contracts import ToolGeometry
from peach_scene_perception.scene_perception_parameters import (
    peach_scene_perception_node as _Generated,
)
from rcl_interfaces.msg import SetParametersResult

_INT_MIN = {
    'sync_queue_size': 1,
    'yolo_imgsz': 32,
    'sam_max_bboxes': 1,
    'sam_min_area': 0,
    'min_mask_points': 1,
    'detection_cloud_stride': 1,
    'pipeline.min_points': 1,
    'target_memory.max_targets': 1,
    'target_memory.confirm_frames': 1,
    'target_memory.tentative_ttl_frames': 1,
    'wind.swing_frames': 1,
    'lighting.bad_frames': 1,
    'harvest.min_collect_frames': 1,
    'harvest.lock_settle_frames': 1,
}


def _gravity_hint(text):
    hint_text = str(text).strip()
    if not hint_text:
        return None
    parts = [float(item) for item in hint_text.split(',')]
    if len(parts) != 3 or not np.isfinite(parts).all():
        raise ValueError('gravity_hint_xyz needs 3 finite comma-separated floats')
    return np.asarray(parts, dtype=float)


def _derive(snapshot):
    """Parse gravity and assemble ToolGeometry; raise on illegal combinations."""
    gravity_hint = _gravity_hint(snapshot.gravity_hint_xyz)
    gravity_mode = str(snapshot.gravity_mode).strip()
    if gravity_mode not in ('fixed', 'tf'):
        gravity_mode = 'fixed'
    if not float(snapshot.pipeline.min_depth_m) < float(
            snapshot.pipeline.max_depth_m):
        raise ValueError('pipeline.min_depth_m must be < pipeline.max_depth_m')
    tool = ToolGeometry(
        d_inner_m=float(snapshot.tool.D_inner),
        insert_length_m=float(snapshot.tool.L_insert),
        blade_offset_m=0.0,
        entry_d_tool=float(snapshot.tool.entry_d_tool),
        entry_d_s=float(snapshot.tool.entry_d_s),
        entry_standoff=float(snapshot.tool.entry_d_tool)
        + float(snapshot.tool.entry_d_s),
        clearance_min=float(snapshot.tool.clearance_min),
        margin_neck=float(snapshot.tool.margin_neck),
        version=str(snapshot.tool.version),
    )
    return tool, gravity_hint, gravity_mode


def _set_constraints(node, parameters):
    """Validate integer floors and the depth window before GPL commits."""
    proposed = {item.name: item.value for item in parameters}
    for name, minimum in _INT_MIN.items():
        if name in proposed and int(proposed[name]) < minimum:
            return SetParametersResult(
                successful=False, reason=f'{name} >= {minimum} required')
    lo_name, hi_name = 'pipeline.min_depth_m', 'pipeline.max_depth_m'
    if lo_name in proposed or hi_name in proposed:
        low = proposed.get(lo_name)
        high = proposed.get(hi_name)
        if low is None and node.has_parameter(lo_name):
            low = node.get_parameter(lo_name).value
        if high is None and node.has_parameter(hi_name):
            high = node.get_parameter(hi_name).value
        if low is not None and high is not None and not float(low) < float(high):
            return SetParametersResult(
                successful=False, reason='pipeline.min_depth_m must be < pipeline.max_depth_m')
    if 'gravity_hint_xyz' in proposed:
        try:
            _gravity_hint(proposed['gravity_hint_xyz'])
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
    return SetParametersResult(successful=True)


class ScenePerceptionParams:
    """Live GPL parameters plus ToolGeometry/gravity derivation."""

    def __init__(self, raw) -> None:
        """Wrap a generated listener or a plain snapshot."""
        object.__setattr__(self, '_listener', None)
        object.__setattr__(self, '_raw', raw)

    def _current(self):
        if self._listener is not None:
            return self._listener.get_params()
        return self._raw

    @property
    def tool(self) -> ToolGeometry:
        """Return derived hollow-cylinder tool geometry."""
        return _derive(self._current())[0]

    @property
    def gravity_hint(self):
        """Camera-frame gravity hint, or None."""
        return _derive(self._current())[1]

    @property
    def gravity_mode(self) -> str:
        """Return normalized gravity source."""
        return _derive(self._current())[2]

    def __getattr__(self, name: str):
        """Read the current generated parameter field."""
        return getattr(self._current(), name)

    @classmethod
    def attach(cls, node) -> 'ScenePerceptionParams':
        """Register constraints, create the GPL listener, validate startup values."""
        validate = partial(_set_constraints, node)
        node.add_on_set_parameters_callback(validate)
        try:
            listener = _Generated.ParamListener(node)
        finally:
            node.remove_on_set_parameters_callback(validate)
        # rclpy prepends callbacks. Validate both declarations and updates before GPL commits.
        node.add_on_set_parameters_callback(validate)
        wrapper = cls(None)
        object.__setattr__(wrapper, '_listener', listener)
        params = listener.get_params()
        if not str(params.yolo_model_path).strip() or not str(
                params.sam_model_path).strip():
            raise ValueError('yolo_model_path and sam_model_path must be non-empty')
        _derive(params)
        return wrapper

    @classmethod
    def from_params(cls, snapshot) -> 'ScenePerceptionParams':
        """Pure snapshot wrapper for tests; no ROS listener is created."""
        return cls(snapshot)
