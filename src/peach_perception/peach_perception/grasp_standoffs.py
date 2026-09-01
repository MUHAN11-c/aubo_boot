"""Load grasp_standoffs.yaml (plain keys, not ROS ParameterFile format)."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_grasp_standoffs():
    """Return (entry_standoff_m, pregrasp_standoff_m) from the shared yaml."""
    path = os.path.join(
        get_package_share_directory('peach_perception'),
        'config', 'grasp_standoffs.yaml')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return float(data['entry_standoff_m']), float(data['pregrasp_standoff_m'])


def scene_overlay():
    entry, _ = load_grasp_standoffs()
    return {'tool.entry_d_tool': entry, 'tool.entry_d_s': 0.0}


def reconstruction_overlay():
    entry, approach = load_grasp_standoffs()
    return {
        'refit.entry_standoff_m': entry,
        'refit.pregrasp_standoff_m': approach,
    }


def manipulation_overlay():
    _, approach = load_grasp_standoffs()
    return {'moveit.mtc_approach_along_axis_m': approach}
