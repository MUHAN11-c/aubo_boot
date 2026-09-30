"""
Tool geometry and calibration constants loaders.

Geometry: aubo_description/config/<tool_id>.yaml; constants:
peach2_calibration/results/<tool_id>.yaml. No defaults in code: missing keys raise.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re

import yaml

_TOOL_ID_RE = re.compile(r'^[a-z0-9_]+$')
CALIBRATED = 'calibrated'


@dataclass
class ToolGeometry:
    tool_id: str
    d_inner_m: float
    d_outer_m: float
    l_insert_m: float
    l_blade_m: float
    body_length_m: float
    body_radius_m: float
    wall_clearance_m: float


@dataclass
class ToolCalibration:
    w_capture_m: float
    e_blade_m: float
    e_robot_m: float
    e_tcp_m: float
    e_handeye_m: float
    e_runout_m: float
    c_wall_m: float
    fruit_clearance_m: float
    status: str


def _read(tool_id: str, directory: str) -> dict:
    if not _TOOL_ID_RE.match(tool_id or ''):
        raise ValueError(f'invalid tool_id {tool_id!r}')
    path = os.path.join(directory, f'{tool_id}.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: not a mapping')
    return data


def _number(section: dict, key: str, path: str, positive: bool) -> float:
    if key not in section:
        raise ValueError(f'{path}: missing {key}')
    value = section[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{path}: {key} must be a number')
    value = float(value)
    if not math.isfinite(value) or value < 0.0 or (positive and value == 0.0):
        raise ValueError(f'{path}: {key}={value} out of range')
    return value


def load_tool_geometry(tool_id: str, description_config_dir: str) -> ToolGeometry:
    data = _read(tool_id, description_config_dir)
    where = f'{description_config_dir}/{tool_id}.yaml'
    if data.get('profile_id') != tool_id:
        raise ValueError(f'{where}: profile_id {data.get("profile_id")!r} != {tool_id!r}')
    geo = data.get('geometry_m')
    if not isinstance(geo, dict):
        raise ValueError(f'{where}: missing geometry_m')
    tool = ToolGeometry(
        tool_id=tool_id,
        d_inner_m=_number(geo, 'D_inner', where, True),
        d_outer_m=_number(geo, 'D_outer', where, True),
        l_insert_m=_number(geo, 'L_insert', where, True),
        l_blade_m=_number(geo, 'L_blade', where, False),
        body_length_m=_number(geo, 'body_length', where, True),
        body_radius_m=_number(geo, 'body_radius', where, True),
        wall_clearance_m=_number(geo, 'wall_clearance', where, False))
    if tool.d_inner_m >= tool.d_outer_m:
        raise ValueError(f'{where}: D_inner must be < D_outer')
    return tool


def load_tool_calibration(tool_id: str, calibration_dir: str) -> ToolCalibration:
    data = _read(tool_id, calibration_dir)
    where = f'{calibration_dir}/{tool_id}.yaml'
    if 'tool_id' in data and data['tool_id'] != tool_id:
        raise ValueError(f'{where}: tool_id {data["tool_id"]!r} != {tool_id!r}')
    status = data.get('status')
    if not isinstance(status, str) or not status:
        raise ValueError(f'{where}: missing status')
    return ToolCalibration(
        w_capture_m=_number(data, 'w_capture_m', where, True),
        e_blade_m=_number(data, 'e_blade_m', where, False),
        e_robot_m=_number(data, 'e_robot_m', where, False),
        e_tcp_m=_number(data, 'e_tcp_m', where, False),
        e_handeye_m=_number(data, 'e_handeye_m', where, False),
        e_runout_m=_number(data, 'e_runout_m', where, False),
        c_wall_m=_number(data, 'c_wall_m', where, False),
        fruit_clearance_m=_number(data, 'fruit_clearance_m', where, False),
        status=status)
