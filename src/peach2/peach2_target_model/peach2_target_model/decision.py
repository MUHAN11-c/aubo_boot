"""
Pure GraspDecision logic (no ROS): convergence gates, permission rules and pregrasp geometry.

Permission chain (each level implies the previous one):
  approach = model valid and converged, not expired, swing known (when require_known_swing)
             and <= swing_max, tool radially feasible at all, and
             radial margin >= -approach_radial_slack_m
  sleeve   = approach and budget.sleeve_ok (radial margin > 0)
  cut      = sleeve and budget.cut_ok and close-range neck re-measure passed (when required)
             and no re-measure mismatch
failure_code / reason name the first level that failed; 0 / 'ok' only when cut is allowed.
Decision.flags carries informational tags (e.g. 'fruit_top_proxy', 'swing_unknown').
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

import numpy as np
from peach2_core import codes
from peach2_core.budget import evaluate_budget, radial_structural, tcp_target_for_blade
from peach2_core.fusion import FusedModel
from peach2_core.tool import ToolCalibration, ToolGeometry
from peach2_core.types import unit

from .params import TargetModelParams


class Remeasure(Enum):
    NONE = 'none'
    PASSED = 'passed'
    MISMATCH = 'mismatch'


@dataclass
class ModelRecord:
    """Fused model state for one target (owned by the store, copied out for decisions)."""

    target_id: str
    revision: int
    fused: FusedModel
    swing_amplitude_m: float
    last_obs_s: float
    converged: bool
    unconverged: list[str] = field(default_factory=list)
    remeasure: Remeasure = Remeasure.NONE
    fruit_top_offset_m: float = float('nan')


@dataclass
class Decision:
    target_id: str
    model_revision: int
    tool_id: str
    valid_until_s: float
    approach_allowed: bool
    sleeve_allowed: bool
    cut_allowed: bool
    radial_margin_m: float
    axial_margin_m: float
    pregrasp_position: np.ndarray
    pregrasp_quat_xyzw: np.ndarray
    blade_target: np.ndarray
    insert_travel_m: float
    failure_code: int
    reason: str
    flags: list[str] = field(default_factory=list)


def convergence_failures(model: FusedModel, params: TargetModelParams) -> list[str]:
    """Names of the sigma gates not met; empty means converged."""
    if not model.ok:
        return ['model_invalid']
    failed = []
    if model.n_views < params.min_views:
        failed.append('too_few_views')
    if not model.sigma_lateral95_m < params.converge_sigma_lateral95_m:
        failed.append('sigma_lateral')
    if not model.theta95_deg < params.converge_theta95_deg:
        failed.append('theta')
    if not model.sigma_axial95_m < params.converge_sigma_axial95_m:
        failed.append('sigma_axial')
    return failed


def quat_z_to_axis(axis: np.ndarray) -> np.ndarray:
    """Shortest-arc quaternion (x, y, z, w) rotating +Z onto `axis`; roll is left arbitrary."""
    a = unit(axis)
    if a is None:
        raise ValueError('axis must be a finite non-zero vector')
    w = 1.0 + float(a[2])
    if w < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = np.array([-a[1], a[0], 0.0, w])
    return q / np.linalg.norm(q)


def pregrasp_position(bottom: np.ndarray, axis: np.ndarray, standoff_m: float) -> np.ndarray:
    a = unit(axis)
    if a is None:
        raise ValueError('axis must be a finite non-zero vector')
    return np.asarray(bottom, dtype=np.float64).reshape(3) - standoff_m * a


def cut_to_fruit_m(model: FusedModel, fruit_top_offset_m: float) -> tuple[float, bool]:
    """
    Blade-plane (neck) to fruit-top distance and whether the fallback proxy was used.

    Uses TargetModel.fruit_top_offset_m when known. Otherwise the fruit is assumed to rest on
    the bag bottom with a diameter equal to the bag d95 (length - d95).
    """
    if not model.ok:
        return float('nan'), False
    if math.isfinite(fruit_top_offset_m):
        return float(fruit_top_offset_m), False
    return float(model.length_m - model.d95_m), True


def remeasure_deviation(fused_neck: np.ndarray, close_necks: list[np.ndarray]) -> float:
    """3D distance between the fused neck and the component-wise median close-range neck."""
    if not close_necks:
        return float('nan')
    med = np.median(np.asarray(close_necks, dtype=np.float64).reshape(-1, 3), axis=0)
    return float(np.linalg.norm(med - np.asarray(fused_neck, dtype=np.float64).reshape(3)))


def _blocked(record: ModelRecord, tool_id: str, valid_until_s: float,
             code: int, reason: str) -> Decision:
    zero = np.zeros(3)
    return Decision(
        target_id=record.target_id, model_revision=record.revision, tool_id=tool_id,
        valid_until_s=valid_until_s, approach_allowed=False, sleeve_allowed=False,
        cut_allowed=False, radial_margin_m=float('nan'), axial_margin_m=float('nan'),
        pregrasp_position=zero.copy(), pregrasp_quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        blade_target=zero.copy(), insert_travel_m=0.0, failure_code=int(code), reason=reason)


def build_decision(record: ModelRecord, tool: ToolGeometry, calib: ToolCalibration,
                   params: TargetModelParams, now_s: float) -> Decision:
    valid_until = now_s + params.validity_s
    model = record.fused
    if not model.ok:
        return _blocked(record, tool.tool_id, valid_until, codes.MODEL_NOT_CONVERGED,
                        'model_invalid')
    if now_s - record.last_obs_s > params.validity_s:
        return _blocked(record, tool.tool_id, valid_until, codes.MODEL_EXPIRED, 'model_expired')

    flags: list[str] = []
    swing_known = math.isfinite(record.swing_amplitude_m)
    if not swing_known:
        flags.append('swing_unknown')
    fruit_dist, fruit_proxy = cut_to_fruit_m(model, record.fruit_top_offset_m)
    if fruit_proxy:
        flags.append('fruit_top_proxy')
    swing_for_budget = record.swing_amplitude_m if swing_known else 0.0
    budget = evaluate_budget(model, tool, calib, swing_for_budget, fruit_dist)
    pregrasp = pregrasp_position(model.bottom, model.axis, params.pregrasp_standoff_m)
    tcp_final = tcp_target_for_blade(model.neck, model.axis, tool)
    insert_travel = float((tcp_final - pregrasp) @ unit(model.axis))

    radial_ok = (math.isfinite(budget.radial_margin_m)
                 and budget.radial_margin_m >= -params.approach_radial_slack_m)
    structural_r = radial_structural(tool, calib)

    if not record.converged:
        code, reason = codes.MODEL_NOT_CONVERGED, 'model_not_converged'
    elif not swing_known and params.require_known_swing:
        code, reason = codes.SWING_TOO_LARGE, 'swing_unknown'
    elif swing_known and record.swing_amplitude_m > params.swing_max_m:
        code, reason = codes.SWING_TOO_LARGE, 'swing_too_large'
    elif structural_r:
        code, reason = codes.BUDGET_STRUCTURAL, 'radial_structural'
    elif not radial_ok:
        code, reason = codes.BUDGET_RADIAL_NEGATIVE, 'radial_margin_below_approach_slack'
    else:
        code, reason = codes.NONE, 'ok'
    approach = code == codes.NONE
    sleeve = approach and budget.sleeve_ok
    cut = sleeve and budget.cut_ok
    if approach and not cut:
        code, reason = budget.failure_code, budget.reason
    if cut and record.remeasure is Remeasure.MISMATCH:
        cut = False
        code, reason = codes.NECK_REMEASURE_MISMATCH, 'neck_remeasure_mismatch'
    elif cut and params.cut_requires_remeasure and record.remeasure is not Remeasure.PASSED:
        cut = False
        code, reason = codes.NECK_REMEASURE_PENDING, 'neck_remeasure_pending'

    return Decision(
        target_id=record.target_id, model_revision=record.revision, tool_id=tool.tool_id,
        valid_until_s=valid_until, approach_allowed=approach, sleeve_allowed=sleeve,
        cut_allowed=cut, radial_margin_m=budget.radial_margin_m,
        axial_margin_m=budget.axial_margin_m, pregrasp_position=pregrasp,
        pregrasp_quat_xyzw=quat_z_to_axis(model.axis), blade_target=model.neck.copy(),
        insert_travel_m=insert_travel, failure_code=int(code), reason=reason, flags=flags)
