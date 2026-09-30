"""Shared synthetic inputs for peach2_target_model pure-logic tests."""
import os

import numpy as np
from peach2_core.fusion import FusedModel
from peach2_core.types import axial_lateral_cov
from peach2_target_model.decision import ModelRecord
from peach2_target_model.params import load_params

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'target_model.yaml')
PROFILES = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'aubo_description', 'config'))
CALIB = os.path.normpath(os.path.join(HERE, '..', '..', 'peach2_calibration', 'results'))
TOOLS = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')

AXIS = np.array([0.0, 0.0, 1.0])
BOTTOM = np.array([0.5, 0.1, 0.8])
LENGTH = 0.13
NECK = BOTTOM + LENGTH * AXIS


def resolve_share(pkg: str) -> str:
    return {'aubo_description': os.path.dirname(PROFILES),
            'peach2_calibration': os.path.dirname(CALIB)}[pkg]


def params():
    return load_params(CONFIG, resolve_share)


def fused(d95=0.05, length=LENGTH, s_lat=0.004, s_ax=0.004, theta=2.0, n_views=2, ok=True,
          axis=AXIS, bottom=BOTTOM):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    bottom = np.asarray(bottom, dtype=float)
    cov = axial_lateral_cov(axis, s_lat / 1.96, s_ax / 1.96)
    return FusedModel(ok=ok, bottom=bottom, neck=bottom + length * axis, tie=None, axis=axis,
                      d95_m=d95, length_m=length, sigma_lateral95_m=s_lat,
                      sigma_axial95_m=s_ax, theta95_deg=theta, n_views=n_views, flags=[],
                      bottom_cov=cov, neck_cov=cov)


def record(model=None, swing=0.0, last_obs_s=100.0, converged=True, revision=3, **kw):
    return ModelRecord(target_id='target_1', revision=revision,
                       fused=model if model is not None else fused(), swing_amplitude_m=swing,
                       last_obs_s=last_obs_s, converged=converged, **kw)
