"""Pure launch-time decisions for peach2_system (no ROS imports)."""
from __future__ import annotations

HARDWARE_MODES = ('mock', 'real')
CAMERA_FRONTENDS = ('stereo', 'percipio')
TOOL_IDS = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')

# Lifecycle order: sensing -> model -> scene -> motion -> task; teardown is the reverse.
MANAGED_ORDER = (
    'peach2_perception',
    'peach2_target_model',
    'peach2_scene',
    'peach2_manipulation',
    'peach2_task',
)
CAMERA_NODES = frozenset({'peach2_perception', 'peach2_scene'})


def as_bool(text: str) -> bool:
    """Launch-argument boolean ('true'/'false', case-insensitive); anything else raises."""
    value = str(text).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"expected true/false, got '{text}'")


def managed_node_names(camera_enabled: bool) -> list[str]:
    """Lifecycle-manager node_names: camera-dependent nodes are omitted when not launched."""
    return [n for n in MANAGED_ORDER if camera_enabled or n not in CAMERA_NODES]


def start_stereo(camera_enabled: bool, frontend: str) -> bool:
    """Whether this launch starts the stereo camera driver itself."""
    return camera_enabled and frontend == 'stereo'


def aubo_camera_enabled(camera_enabled: bool, frontend: str) -> bool:
    """aubo_e5_bringup camera_enabled means the Percipio driver; stereo is started separately."""
    return camera_enabled and frontend == 'percipio'


def validate(hardware_mode: str, camera_frontend: str, tool_id: str, bond_timeout_s: float) -> str:
    """Empty string when the combination is valid, else the reason."""
    if hardware_mode not in HARDWARE_MODES:
        return f"hardware_mode '{hardware_mode}' not in {HARDWARE_MODES}"
    if camera_frontend not in CAMERA_FRONTENDS:
        return f"camera_frontend '{camera_frontend}' not in {CAMERA_FRONTENDS}"
    if tool_id not in TOOL_IDS:
        return f"tool_id '{tool_id}' not in {TOOL_IDS}"
    if not bond_timeout_s >= 0.0:
        return f'bond_timeout must be >= 0, got {bond_timeout_s}'
    return ''
