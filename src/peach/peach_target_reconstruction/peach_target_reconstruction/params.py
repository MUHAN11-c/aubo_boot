"""Target reconstruction parameters backed by generate_parameter_library."""
from __future__ import annotations

from peach_target_reconstruction.target_reconstruction_parameters import (
    peach_target_reconstruction_node as _Generated,
)
from rcl_interfaces.msg import SetParametersResult


_INT_MIN = {
    'capture.min_views': 1,
    'capture.recommended_views': 1,
    'capture.max_views': 1,
    'capture.min_mask_pixels': 1,
    'icp.min_points': 1,
    'icp.coarse_iterations': 1,
    'icp.fine_iterations': 1,
    'icp.target_refresh_min_period': 1,
    'icp.target_refresh_max_period': 1,
}


def _int_bounds(parameters):
    """Enforce integer lower bounds without broken GPL IntegerRange descriptors."""
    for param in parameters:
        minimum = _INT_MIN.get(param.name)
        if minimum is not None and int(param.value) < minimum:
            return SetParametersResult(
                successful=False, reason=f'{param.name} >= {minimum} required')
    return SetParametersResult(successful=True)


class _StripProxy:
    """Forward a generated nested group while stripping string fields."""

    def __init__(self, raw, names: tuple) -> None:
        """Store the group and the fields to strip."""
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, '_names', names)

    def __getattr__(self, name):
        """Strip selected fields; forward the rest."""
        value = getattr(self._raw, name)
        if name in self._names:
            return str(value).strip()
        return value


def _flatten(value, prefix=''):
    """Flatten a generated parameter struct into dotted primitive keys."""
    data = {}
    for name in dir(value):
        if name.startswith('_') or name == 'stamp_':
            continue
        item = getattr(value, name)
        if callable(item):
            continue
        key = f'{prefix}.{name}' if prefix else name
        if hasattr(item, '__dict__') and not isinstance(item, type):
            data.update(_flatten(item, key))
        elif isinstance(item, (str, int, float, bool)):
            data[key] = item
    return data


class TargetReconstructionParams:
    """Live GPL snapshot with stripped frames/session strings."""

    def __init__(self, raw) -> None:
        """Wrap either a generated listener or a plain snapshot."""
        object.__setattr__(self, '_listener', None)
        object.__setattr__(self, '_raw', raw)

    def _current(self):
        if self._listener is not None:
            return self._listener.get_params()
        return self._raw

    def __getattr__(self, name: str):
        """Return the current generated/snapshot field."""
        value = getattr(self._current(), name)
        if name == 'frames':
            return _StripProxy(value, ('base_frame',))
        if name == 'session':
            return _StripProxy(value, ('root_dir',))
        return value

    def snapshot(self) -> dict:
        """Return a dot-key snapshot, with trimmed session/frame values."""
        data = _flatten(self._current())
        for key, value in list(data.items()):
            if (key.startswith('frames.') or key.startswith('session.')) \
                    and isinstance(value, str):
                data[key] = value.strip()
        return data

    @classmethod
    def attach(cls, node) -> 'TargetReconstructionParams':
        """Declare GPL parameters and return the live wrapper."""
        node.add_on_set_parameters_callback(_int_bounds)
        try:
            listener = _Generated.ParamListener(node)
        finally:
            node.remove_on_set_parameters_callback(_int_bounds)
        # rclpy prepends callbacks. Validate both declarations and updates before GPL commits.
        node.add_on_set_parameters_callback(_int_bounds)
        wrapper = cls(None)
        object.__setattr__(wrapper, '_listener', listener)
        return wrapper

    @classmethod
    def from_params(cls, snapshot) -> 'TargetReconstructionParams':
        """Pure snapshot wrapper for tests; no ROS listener is created."""
        return cls(snapshot)
