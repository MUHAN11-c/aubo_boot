"""Bringup helper nodes backed by generate_parameter_library."""
from __future__ import annotations

from peach_bringup.autostart_parameters import peach_autostart_client
from peach_bringup.flag_bridge_parameters import peach_lifecycle_flag_bridge
from peach_interfaces.action import RunHarvest
from rcl_interfaces.msg import SetParametersResult


class _LiveParams:
    """Read-through view over a generated ParamListener."""

    def __init__(self, listener) -> None:
        """Store the generated listener."""
        object.__setattr__(self, '_listener', listener)

    def __getattr__(self, name: str):
        """Return the current generated value."""
        return getattr(self._listener.get_params(), name)


class FlagBridgeParams(_LiveParams):
    """peach_lifecycle_flag_bridge parameters."""


class AutostartClientParams(_LiveParams):
    """peach_autostart_client parameters."""


def attach_flag_bridge(node) -> FlagBridgeParams:
    """Declare flag-bridge parameters and return a live view."""
    return FlagBridgeParams(
        peach_lifecycle_flag_bridge.ParamListener(node))


def _int_bounds(parameters):
    """Enforce the RunHarvest intent constants before GPL cache updates."""
    for param in parameters:
        if param.name == 'intent' and int(param.value) not in (
                RunHarvest.Goal.INTENT_PICK_ALL,
                RunHarvest.Goal.INTENT_PICK_SELECTED,
                RunHarvest.Goal.INTENT_SURVEY_ONLY):
            return SetParametersResult(
                successful=False, reason='intent must be PICK_ALL, PICK_SELECTED or SURVEY_ONLY')
    return SetParametersResult(successful=True)


def attach_autostart_client(node) -> AutostartClientParams:
    """Declare autostart parameters and return a live view."""
    node.add_on_set_parameters_callback(_int_bounds)
    try:
        listener = peach_autostart_client.ParamListener(node)
    finally:
        node.remove_on_set_parameters_callback(_int_bounds)
    # rclpy prepends callbacks: validate before GPL updates its runtime cache.
    node.add_on_set_parameters_callback(_int_bounds)
    return AutostartClientParams(listener)
