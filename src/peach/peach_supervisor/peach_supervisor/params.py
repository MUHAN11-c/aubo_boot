"""Supervisor parameters backed by generate_parameter_library."""
from __future__ import annotations

from peach_supervisor.supervisor_parameters import peach_supervisor
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidParameterValueException
from rclpy.exceptions import ParameterNotDeclaredException


_INT_BOUNDS = {
    'empty_survey_limit': (1, None),
    'selection_priority': (0, 30),
    'reconstruction_min_views': (1, None),
}
_WINDOW_PAIRS = (
    ('selection_reach_min_m', 'selection_reach_max_m'),
    ('selection_depth_min_m', 'selection_depth_max_m'),
)


def _int_bounds(parameters):
    """Enforce integer bounds without broken GPL IntegerRange descriptors."""
    for param in parameters:
        bounds = _INT_BOUNDS.get(param.name)
        if bounds is None:
            continue
        low, high = bounds
        value = int(param.value)
        if (low is not None and value < low) or (
                high is not None and value > high):
            return SetParametersResult(
                successful=False, reason=f'{param.name} out of range')
    return SetParametersResult(successful=True)


class SupervisorParams:
    """Read-through view over the generated supervisor ParamListener."""

    def __init__(self, listener) -> None:
        """Store the generated listener."""
        object.__setattr__(self, '_listener', listener)

    def __getattr__(self, name: str):
        """Return the current generated parameter value."""
        return getattr(self._listener.get_params(), name)


def _cross_field(node):
    """Reject min/max window pairs that would become inverted on a set."""
    def check(parameters):
        proposed = {item.name: item.value for item in parameters}

        def value(name):
            if name in proposed:
                return proposed[name]
            try:
                return node.get_parameter(name).value
            except ParameterNotDeclaredException:
                return None

        for low, high in _WINDOW_PAIRS:
            low_value = value(low)
            high_value = value(high)
            if low_value is None or high_value is None:
                continue
            if float(low_value) > float(high_value):
                return SetParametersResult(
                    successful=False, reason=f'{low} <= {high} required')
        return SetParametersResult(successful=True)
    return check


def attach(node) -> SupervisorParams:
    """Validate startup values; run rejection callbacks before GPL commits its view."""
    listener = peach_supervisor.ParamListener(node)
    validators = (_int_bounds, _cross_field(node))
    names = set(_INT_BOUNDS)
    names.update(name for pair in _WINDOW_PAIRS for name in pair)
    current = [node.get_parameter(name) for name in sorted(names)]
    for validate in validators:
        result = validate(current)
        if not result.successful:
            raise InvalidParameterValueException(
                'peach_supervisor', {item.name: item.value for item in current}, result.reason)
        # rclpy 前插回调；后注册的只读验证必须先于 listener.update 执行。
        node.add_on_set_parameters_callback(validate)
    return SupervisorParams(listener)
