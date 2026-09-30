"""
Per-target sleeve / cut budget (plan section 5.1): independent 95% terms combined by RSS.

radial: m_r = D_inner/2 - d95/2 - c_wall
              - sqrt(s_lat^2 + (L sin th95)^2 + e_tcp^2 + e_handeye^2 + e_runout^2 + A^2)
axial:  m_a = w_capture - sqrt(s_ax^2 + e_blade^2 + e_robot^2 + A^2)

L is the bag length (the sleeve path is anchored at the fused bottom, so the axis error grows
the lateral offset over bottom -> neck). The swing amplitude A is a scalar peak of unknown
direction, so it is charged to both the radial and the axial budget.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from . import codes
from .fusion import FusedModel
from .tool import CALIBRATED, ToolCalibration, ToolGeometry
from .types import unit


@dataclass
class BudgetResult:
    radial_margin_m: float
    axial_margin_m: float
    sleeve_ok: bool
    cut_ok: bool
    structural: bool
    failure_code: int
    reason: str


def _rss(*terms: float) -> float:
    return math.sqrt(sum(t * t for t in terms))


def radial_structural(tool: ToolGeometry, calib: ToolCalibration) -> bool:
    """No bag (d95 = 0, zero perception error) could be sleeved with these constants."""
    fixed = _rss(calib.e_tcp_m, calib.e_handeye_m, calib.e_runout_m)
    return 0.5 * tool.d_inner_m - calib.c_wall_m <= fixed


def axial_structural(calib: ToolCalibration) -> bool:
    """Tool-only axial terms (neck axial error = 0, no swing) already fill the capture band."""
    return calib.w_capture_m <= _rss(calib.e_blade_m, calib.e_robot_m)


def evaluate_budget(model: FusedModel, tool: ToolGeometry, calib: ToolCalibration,
                    swing_amplitude_m: float = 0.0,
                    cut_to_fruit_m: float = float('inf')) -> BudgetResult:
    """
    Sleeve / cut permission for one fused model and one tool.

    Precedence of failure codes: invalid model (20) > radial structural (25) > radial
    negative (23) > calibration not 'calibrated' (25, cut only) > axial structural (25) >
    fruit clearance (24) > axial negative (24).
    """
    structural = radial_structural(tool, calib) or axial_structural(calib)
    inputs = (model.d95_m, model.length_m, model.sigma_lateral95_m, model.sigma_axial95_m,
              model.theta95_deg, swing_amplitude_m)
    if (not model.ok or not all(math.isfinite(v) for v in inputs) or swing_amplitude_m < 0.0
            or math.isnan(cut_to_fruit_m)):
        return BudgetResult(float('nan'), float('nan'), False, False, structural,
                            codes.MODEL_NOT_CONVERGED, 'model_invalid')
    a = swing_amplitude_m
    available = 0.5 * tool.d_inner_m - 0.5 * model.d95_m - calib.c_wall_m
    lever = max(model.length_m, 0.0) * math.sin(math.radians(max(model.theta95_deg, 0.0)))
    radial_err = _rss(model.sigma_lateral95_m, lever, calib.e_tcp_m, calib.e_handeye_m,
                      calib.e_runout_m, a)
    m_r = available - radial_err
    m_a = calib.w_capture_m - _rss(model.sigma_axial95_m, calib.e_blade_m, calib.e_robot_m, a)
    sleeve_ok = model.d95_m > 0.0 and m_r > 0.0
    fruit_ok = cut_to_fruit_m > calib.fruit_clearance_m
    calibrated = calib.status == CALIBRATED
    cut_ok = sleeve_ok and calibrated and not structural and fruit_ok and m_a > 0.0

    if radial_structural(tool, calib):
        code, reason = codes.BUDGET_STRUCTURAL, 'radial_structural'
    elif model.d95_m <= 0.0:
        code, reason = codes.BUDGET_RADIAL_NEGATIVE, 'bag_d95_missing'
    elif available <= 0.0:
        code, reason = codes.BUDGET_RADIAL_NEGATIVE, 'bag_d95_exceeds_tool'
    elif m_r <= 0.0:
        code, reason = codes.BUDGET_RADIAL_NEGATIVE, 'radial_budget_negative'
    elif not calibrated:
        code, reason = codes.BUDGET_STRUCTURAL, 'calibration_pending'
    elif axial_structural(calib):
        code, reason = codes.BUDGET_STRUCTURAL, 'axial_structural'
    elif not fruit_ok:
        code, reason = codes.BUDGET_AXIAL_NEGATIVE, 'cut_plane_fruit_clearance'
    elif m_a <= 0.0:
        code, reason = codes.BUDGET_AXIAL_NEGATIVE, 'axial_budget_negative'
    else:
        code, reason = codes.NONE, 'ok'
    return BudgetResult(radial_margin_m=float(m_r), axial_margin_m=float(m_a),
                        sleeve_ok=bool(sleeve_ok), cut_ok=bool(cut_ok), structural=structural,
                        failure_code=int(code), reason=reason)


def tcp_target_for_blade(neck: np.ndarray, axis: np.ndarray, tool: ToolGeometry) -> np.ndarray:
    """
    TCP position that puts the blade plane (L_blade behind the mouth along -Z_tcp) on the neck.

    The mouth overshoots the neck by L_blade; collision / feasibility must account for it.
    """
    a = unit(axis)
    if a is None:
        raise ValueError('axis must be a finite non-zero vector')
    return np.asarray(neck, dtype=np.float64).reshape(3) + tool.l_blade_m * a
