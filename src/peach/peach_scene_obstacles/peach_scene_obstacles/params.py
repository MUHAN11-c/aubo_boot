"""Scene obstacle parameters backed by generate_parameter_library."""
from __future__ import annotations

from peach_scene_obstacles.core import SnapshotParams
from peach_scene_obstacles.scene_obstacles_parameters import peach_scene_obstacles
from rcl_interfaces.msg import SetParametersResult


def _int_bounds(parameters):
    """Enforce max_boxes lower bound without a broken GPL IntegerRange."""
    for param in parameters:
        if param.name == 'max_boxes' and int(param.value) < 1:
            return SetParametersResult(
                successful=False, reason='max_boxes >= 1 required')
    return SetParametersResult(successful=True)


class SceneObstaclesParams:
    """Live parameter view over a generated ParamListener."""

    def __init__(self, listener) -> None:
        """Store the generated listener; values are fetched on access."""
        object.__setattr__(self, '_listener', listener)

    def __getattr__(self, name: str):
        """Read the current generated parameter value."""
        return getattr(self._listener.get_params(), name)

    @staticmethod
    def attach(node) -> 'SceneObstaclesParams':
        """Declare parameters and return the live view."""
        node.add_on_set_parameters_callback(_int_bounds)
        try:
            listener = peach_scene_obstacles.ParamListener(node)
        finally:
            node.remove_on_set_parameters_callback(_int_bounds)
        # rclpy prepends callbacks. Validate both declarations and updates before GPL commits.
        node.add_on_set_parameters_callback(_int_bounds)
        return SceneObstaclesParams(listener)

    def tuning(self) -> SnapshotParams:
        """Return a pure snapshot for build_snapshot."""
        params = self._listener.get_params()
        return SnapshotParams(
            voxel_size_m=params.voxel_size_m,
            self_filter_margin_m=params.self_filter_margin_m,
            capsule_radial_margin_m=params.capsule_radial_margin_m,
            capsule_axial_margin_m=params.capsule_axial_margin_m,
            workspace_radius_m=params.workspace_radius_m,
            max_boxes=params.max_boxes)
