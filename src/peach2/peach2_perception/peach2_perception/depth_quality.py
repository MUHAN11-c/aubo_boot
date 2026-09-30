"""
Depth unit conversion, validity, confidence and in-mask coverage (no ROS).

The depth image is registered to the colour grid. Raw uint16 is converted to metres here, at the
driver boundary (stereo front end 0.25 mm/LSB, Percipio 1 mm/LSB, set by `depth_unit_m`).
Everything downstream works in metres; invalid depth is 0.
"""
from __future__ import annotations

import numpy as np

_UINT16_SATURATED = np.iinfo(np.uint16).max


def depth_to_metres(depth: np.ndarray, depth_unit_m: float) -> np.ndarray:
    """
    Raw depth image -> float32 metres (0 = invalid).

    uint16: raw * depth_unit_m; 0 and the 65535 driver sentinel are invalid. The sentinel is
    tested on raw counts (at 0.25 mm/LSB it is 16.4 m, at 1 mm/LSB 65.5 m, so a range test in
    metres would not catch it consistently). float32/64 images are taken as metres already.
    """
    d = np.asarray(depth)
    if d.ndim != 2:
        raise ValueError('depth image must be single channel')
    if d.dtype == np.uint16:
        if not depth_unit_m > 0.0:
            raise ValueError('depth_unit_m must be > 0')
        out = d.astype(np.float32) * np.float32(depth_unit_m)
        out[d == _UINT16_SATURATED] = 0.0
        return out
    if np.issubdtype(d.dtype, np.floating):
        out = d.astype(np.float32, copy=True)
        out[~np.isfinite(out) | (out < 0.0)] = 0.0
        return out
    raise ValueError(f'unsupported depth dtype {d.dtype}')


def valid_depth_mask(depth_m: np.ndarray, min_depth_m: float, max_depth_m: float) -> np.ndarray:
    """Pixels with finite depth inside [min_depth_m, max_depth_m]."""
    d = np.asarray(depth_m)
    return np.isfinite(d) & (d >= min_depth_m) & (d <= max_depth_m) & (d > 0.0)


def normalise_confidence(confidence: np.ndarray) -> np.ndarray:
    """mono8 (0..255) or float (0..1) confidence -> float32 0..1 (non-finite -> 0)."""
    c = np.asarray(confidence)
    if c.dtype == np.uint8:
        return c.astype(np.float32) / np.float32(255.0)
    if c.dtype == np.uint16:
        return c.astype(np.float32) / np.float32(65535.0)
    if np.issubdtype(c.dtype, np.floating):
        out = np.nan_to_num(c.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(out, 0.0, 1.0)
    raise ValueError(f'unsupported confidence dtype {c.dtype}')


def _shift(a: np.ndarray, dy: int, dx: int, fill) -> np.ndarray:
    out = np.full_like(a, fill)
    h, w = a.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = a[ys, xs]
    return out


def surrogate_confidence(depth_m: np.ndarray, valid: np.ndarray, jump_rel_lo: float,
                         jump_rel_hi: float) -> np.ndarray:
    """
    Confidence stand-in when the front end publishes none.

    For each valid pixel the largest relative depth jump to its valid 4-neighbours is mapped
    linearly from 1 (jump <= jump_rel_lo) to 0 (jump >= jump_rel_hi): stereo depth is least
    reliable at depth edges, where flying pixels sit between foreground and background.
    Pixels with fewer than 2 valid neighbours (isolated speckles) get 0.
    """
    if not 0.0 <= jump_rel_lo < jump_rel_hi:
        raise ValueError('need 0 <= jump_rel_lo < jump_rel_hi')
    z = np.asarray(depth_m, dtype=np.float32)
    v = np.asarray(valid, dtype=bool)
    jump = np.zeros_like(z)
    n_valid = np.zeros(z.shape, dtype=np.int8)
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        zn = _shift(z, dy, dx, 0.0)
        both = v & _shift(v, dy, dx, False)
        jump = np.maximum(jump, np.where(both, np.abs(z - zn), 0.0))
        n_valid += both.astype(np.int8)
    rel = jump / np.maximum(z, 1e-6)
    conf = np.clip((jump_rel_hi - rel) / (jump_rel_hi - jump_rel_lo), 0.0, 1.0)
    conf[(n_valid < 2) | ~v] = 0.0
    return conf.astype(np.float32)


def depth_edges(depth_m: np.ndarray, valid: np.ndarray, jump_rel: float,
                jump_abs_m: float) -> np.ndarray:
    """Mark valid pixels whose depth differs from a valid 4-neighbour by > max(abs, rel * z)."""
    z = np.asarray(depth_m, dtype=np.float32)
    v = np.asarray(valid, dtype=bool)
    thresh = np.maximum(np.float32(jump_abs_m), np.float32(jump_rel) * z)
    edge = np.zeros(z.shape, dtype=bool)
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        both = v & _shift(v, dy, dx, False)
        edge |= both & (np.abs(z - _shift(z, dy, dx, 0.0)) > thresh)
    return edge


def confident_depth(depth_m: np.ndarray, valid: np.ndarray, confidence: np.ndarray,
                    min_confidence: float) -> np.ndarray:
    """Depth in metres with invalid or low-confidence pixels zeroed (the landmark input)."""
    ok = np.asarray(valid, dtype=bool) & (np.asarray(confidence) >= min_confidence)
    return np.where(ok, depth_m, 0.0).astype(np.float32)


def mask_depth_coverage(mask: np.ndarray, depth_ok: np.ndarray) -> float:
    """Fraction of mask pixels with valid, confident depth (0 for an empty mask)."""
    m = np.asarray(mask, dtype=bool)
    n = int(np.count_nonzero(m))
    if n == 0:
        return 0.0
    return float(np.count_nonzero(m & np.asarray(depth_ok, dtype=bool))) / n
