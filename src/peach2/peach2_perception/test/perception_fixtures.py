"""Synthetic hanging-bag RGB-D scenes and fake model backends for the perception tests."""
from __future__ import annotations

import os

import numpy as np
from peach2_perception.depth_quality import (confident_depth, surrogate_confidence,
                                             valid_depth_mask)
from peach2_perception.detector import Detection
from peach2_perception.observation_builder import BuilderParams, FrameData
from peach2_perception.segmenter import MaskResult

H, W = 480, 640
K = np.array([[549.0, 0.0, 320.0], [0.0, 549.0, 240.0], [0.0, 0.0, 1.0]])
# Camera optical frame looking along base +X: x_cam = -y_base, y_cam = -z_base, z_cam = x_base.
R_BASE_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
DEPTH_UNIT_M = 0.00025
CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config',
                      'perception.yaml')
BUILDER_PARAMS = BuilderParams(
    bag_class_id=0, n_bins_2d=20, n_bins_3d=12, taper_ratio=0.85, point_stride=1,
    cross_check_tol_m=0.015, cross_check_inset_px=3, backproject_win=1, baseline_m=0.0622,
    stereo_fx_px=549.0, disparity_sigma_px=0.25, confirm_hits=3, ttl_s=3.0, gate_chi2=11.34,
    process_accel_sigma=0.5, swing_window_s=2.0, swing_min_span_s=1.0)


def camera_pose(position=(0.0, 0.0, 0.5), R: np.ndarray = R_BASE_CAM) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position
    return T


def _basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = axis / np.linalg.norm(axis)
    ref = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(a, ref)
    u /= np.linalg.norm(u)
    return u, np.cross(a, u)


def bag_radius(t: np.ndarray, body_r: float, body_len: float, taper_len: float,
               neck_r: float) -> np.ndarray:
    """Radius along the axis from the bottom: rounded bottom, body, linear taper, neck."""
    r = np.full_like(t, body_r)
    round_len = 0.3 * body_r
    rb = t < round_len
    r[rb] = body_r * np.sqrt(np.clip(1.0 - ((round_len - t[rb]) / round_len) ** 2, 0.05, 1.0))
    taper = (t > body_len) & (t <= body_len + taper_len)
    r[taper] = body_r + (neck_r - body_r) * (t[taper] - body_len) / taper_len
    r[t > body_len + taper_len] = neck_r
    return r


def bag_points(bottom=(0.6, 0.0, 0.35), axis=(0.0, 0.0, 1.0), body_r=0.04, body_len=0.13,
               taper_len=0.02, neck_r=0.012, neck_len=0.05, n_theta=900,
               n_t=700) -> np.ndarray:
    """Dense surface samples (N, 3) in base_link of a bag of revolution plus its bottom cap."""
    a = np.asarray(axis, dtype=np.float64)
    a /= np.linalg.norm(a)
    u, v = _basis(a)
    length = body_len + taper_len + neck_len
    t = np.linspace(0.0, length, n_t)
    th = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    tt, hh = np.meshgrid(t, th, indexing='ij')
    rr = bag_radius(tt, body_r, body_len, taper_len, neck_r)
    pts = (np.asarray(bottom)[None, None, :] + tt[..., None] * a
           + rr[..., None] * (np.cos(hh)[..., None] * u + np.sin(hh)[..., None] * v))
    cap_r = np.linspace(0.0, bag_radius(np.zeros(1), body_r, body_len, taper_len, neck_r)[0],
                        60)
    cr, ch = np.meshgrid(cap_r, th, indexing='ij')
    cap = (np.asarray(bottom)[None, None, :]
           + cr[..., None] * (np.cos(ch)[..., None] * u + np.sin(ch)[..., None] * v))
    return np.vstack([pts.reshape(-1, 3), cap.reshape(-1, 3)])


def plane_points(centre, normal, half_w: float, half_h: float, step: float = 0.0006):
    n = np.asarray(normal, dtype=np.float64)
    n /= np.linalg.norm(n)
    u, v = _basis(n)
    s = np.arange(-half_w, half_w, step)
    r = np.arange(-half_h, half_h, step)
    ss, rr = np.meshgrid(s, r, indexing='ij')
    return (np.asarray(centre)[None, None, :] + ss[..., None] * u
            + rr[..., None] * v).reshape(-1, 3)


def render(objects: list[np.ndarray], T_base_cam: np.ndarray, wall_depth_m: float | None = 1.4,
           shape=(H, W)) -> tuple[np.ndarray, np.ndarray]:
    """Z-buffer splat: (depth [m] float32 with 0 = no return, label int (-1 = wall/none))."""
    h, w = shape
    depth = np.full(h * w, np.inf)
    T_cam_base = np.linalg.inv(T_base_cam)
    all_idx, all_z, all_lab = [], [], []
    for label, pts in enumerate(objects):
        pc = pts @ T_cam_base[:3, :3].T + T_cam_base[:3, 3]
        pc = pc[pc[:, 2] > 0.05]
        u = np.round(K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]).astype(int)
        v = np.round(K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]).astype(int)
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        all_idx.append(v[ok] * w + u[ok])
        all_z.append(pc[ok, 2])
        all_lab.append(np.full(int(ok.sum()), label))
    idx = np.concatenate(all_idx)
    z = np.concatenate(all_z)
    lab = np.concatenate(all_lab)
    np.minimum.at(depth, idx, z)
    labels = np.full(h * w, -1)
    winner = z <= depth[idx] + 1e-9
    labels[idx[winner]] = lab[winner]
    empty = ~np.isfinite(depth)
    depth[empty] = wall_depth_m if wall_depth_m is not None else 0.0
    return depth.reshape(h, w).astype(np.float32), labels.reshape(h, w)


def to_raw(depth_m: np.ndarray) -> np.ndarray:
    return np.round(depth_m / DEPTH_UNIT_M).astype(np.uint16)


def make_frame(T: np.ndarray, objects: list[np.ndarray],
               stamp: float = 0.0) -> tuple[FrameData, np.ndarray]:
    """Render a frame (surrogate confidence, as with no confidence topic) and its labels."""
    depth, lab = render(objects, T)
    valid = valid_depth_mask(depth, 0.3, 1.5)
    conf = surrogate_confidence(depth, valid, 0.01, 0.04)
    dc = confident_depth(depth, valid, conf, 0.5)
    return FrameData(stamp, K, T, dc, conf), lab


def bag_instance(lab: np.ndarray, label: int = 0,
                 class_id: int = 0) -> tuple[Detection, MaskResult]:
    mask = lab == label
    return (Detection(mask_bbox(mask), class_id, 'x', 0.9),
            MaskResult(mask, 0.9, int(mask.sum()), int(mask.sum())))


def colour_image(labels: np.ndarray) -> np.ndarray:
    img = np.full(labels.shape + (3,), 60, dtype=np.uint8)
    img[labels >= 0] = (40, 180, 220)
    return img


def mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    vs, us = np.nonzero(mask)
    return int(us.min()), int(vs.min()), int(us.max()) + 1, int(vs.max()) + 1


class FakeYolo:
    """Returns fixed rows [x1, y1, x2, y2, score, cls] and records calls."""

    device = 'cpu'

    def __init__(self, rows, names=None) -> None:
        self.rows = np.asarray(rows, dtype=np.float64).reshape(-1, 6)
        self.names = names if names is not None else {0: 'peach_bag', 1: 'peach_nobag'}
        self.calls = 0

    def predict(self, bgr, conf, iou):
        self.calls += 1
        return self.rows.copy()


class FakeSam:
    """Returns, per prompt box, the provided mask whose pixels best fill that box."""

    device = 'cpu'

    def __init__(self, masks: list[np.ndarray]) -> None:
        self.masks = [np.asarray(m, dtype=bool) for m in masks]
        self.set_image_calls = 0
        self.predict_calls = 0
        self.last_prompts = None

    def set_image(self, bgr) -> None:
        self.set_image_calls += 1

    def predict(self, boxes, points, labels):
        self.predict_calls += 1
        self.last_prompts = (np.asarray(boxes), np.asarray(points), np.asarray(labels))
        out = []
        for b in np.asarray(boxes):
            x1, y1, x2, y2 = (int(round(x)) for x in b)
            counts = [int(m[y1:y2, x1:x2].sum()) for m in self.masks]
            out.append(self.masks[int(np.argmax(counts))].copy())
        return np.stack(out), np.full(len(out), 0.95)
