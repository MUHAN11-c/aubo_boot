#!/usr/bin/env python3
"""Offline YOLO→FastSAM cycle stats using official Ultralytics Speed."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from peach_common.yaml_params import (
    dict_to_ns,
    expand_share,
    load_ros_parameters,
    package_yaml,
)
from peach_scene_perception.params import ScenePerceptionParams
from peach_scene_perception.pipeline import (
    PerceptionPipeline,
    SyncedRgbd,
)
from peach_vision_core.geometry import normalize_depth_to_uint16_mm
from std_msgs.msg import Header

_SPEED_KEYS = ('preprocess', 'inference', 'postprocess')


class _Clock:
    """Monotonic clock adapter for the pure pipeline."""

    def now(self) -> float:
        return time.monotonic()


def _load_params() -> ScenePerceptionParams:
    raw = load_ros_parameters(
        package_yaml('peach_scene_perception', 'scene_perception.yaml'),
        'peach_scene_perception_node')
    return ScenePerceptionParams.from_params(dict_to_ns(expand_share(raw)))


def _frame(path: Path, params: ScenePerceptionParams, index: int):
    with np.load(path) as data:
        rgb = data['rgb_bgr'].copy()
        depth = normalize_depth_to_uint16_mm(
            data['depth_raw'], params.depth_scale_unit)
        k = data['K']
        transform = data['T'].copy() if 'T' in data else np.eye(4)
    header = Header()
    header.stamp.sec = 100 + index
    header.frame_id = params.camera_optical_frame or 'camera_color_optical_frame'
    intrinsics = {
        'fx': float(k[0]), 'fy': float(k[4]),
        'cx': float(k[2]), 'cy': float(k[5]),
        'width': int(depth.shape[1]), 'height': int(depth.shape[0]),
    }
    return SyncedRgbd(
        rgb=rgb, depth=depth, K=intrinsics, cam_frame=header.frame_id,
        out_frame=header.frame_id, geometry_stamp=header.stamp,
        img_header=header, header=header, T_out_cam=transform,
        tf_status='ok', gravity_hint=params.gravity_hint)


def _copy_speed(speed, prefix: str) -> dict:
    if not speed:
        return {}
    out = {}
    for key in _SPEED_KEYS:
        if key in speed:
            out[f'{prefix}_{key}_ms'] = float(speed[key])
    return out


def _percentile_block(values) -> dict:
    arr = np.asarray(list(values), dtype=float)
    return {
        'mean': round(float(arr.mean()), 3),
        'p50': round(float(np.percentile(arr, 50)), 3),
        'p95': round(float(np.percentile(arr, 95)), 3),
        'p99': round(float(np.percentile(arr, 99)), 3),
        'max': round(float(arr.max()), 3),
    }


def _row_cycle_ms(row: dict, prefix: str):
    fields = [f'{prefix}_{key}_ms' for key in _SPEED_KEYS]
    if not all(field in row for field in fields):
        return None
    return sum(row[field] for field in fields)


def _official_stage(rows, prefix: str) -> dict:
    out = {}
    for key in _SPEED_KEYS:
        field = f'{prefix}_{key}_ms'
        vals = [row[field] for row in rows if field in row]
        if vals:
            out[key] = _percentile_block(vals)
    cycles = [ms for ms in (_row_cycle_ms(row, prefix) for row in rows)
              if ms is not None]
    if cycles and all(key in out for key in _SPEED_KEYS):
        out['cycle'] = _percentile_block(cycles)
        out['speed_line'] = (
            'Speed: {:.1f}ms preprocess, {:.1f}ms inference, '
            '{:.1f}ms postprocess per image'.format(
                out['preprocess']['p50'],
                out['inference']['p50'],
                out['postprocess']['p50']))
    return out


def _combined_cycle(rows) -> list:
    totals = []
    for row in rows:
        parts = []
        for prefix in ('detect', 'segment'):
            cycle = _row_cycle_ms(row, prefix)
            if cycle is not None:
                parts.append(cycle)
        if parts:
            totals.append(sum(parts))
    return totals


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', required=True, type=Path)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--json-out', type=Path)
    args = parser.parse_args(argv)
    paths = sorted(args.input_dir.glob('*.npz'))
    if not paths:
        parser.error(f'no .npz files in {args.input_dir}')
    if args.max_frames > 0:
        paths = paths[:args.max_frames]
    params = _load_params()
    pipeline = PerceptionPipeline.from_params(params, _Clock())
    pipeline.warmup()
    warm = min(max(args.warmup, 0), len(paths))
    rows = []
    for index, path in enumerate(paths):
        start = time.perf_counter()
        result = pipeline.process(_frame(path, params, index))
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if result is None:
            continue
        row = {
            'frame': path.name,
            'host_ms': round(elapsed_ms, 3),
            'detections': len(result.kept),
            'candidates': len(result.candidates.candidates),
        }
        row.update(_copy_speed(pipeline.engine.last_detect_speed, 'detect'))
        row.update(_copy_speed(pipeline.engine.last_segment_speed, 'segment'))
        rows.append(row)
    steady = rows[warm:]
    if not steady:
        parser.error('no successful steady-state frames')
    detect = _official_stage(steady, 'detect')
    segment = _official_stage(steady, 'segment')
    cycle_vals = _combined_cycle(steady)
    if not cycle_vals:
        parser.error('no official Results.speed samples')
    cycle = _percentile_block(cycle_vals)
    summary = {
        'protocol': (
            'ultralytics model.info() + Results.speed '
            '(preprocess/inference/postprocess ms per image)'),
        'input_dir': str(args.input_dir),
        'frames': len(steady),
        'warmup_frames': warm,
        'model_info': pipeline.engine.model_info(),
        'detect': detect,
        'segment': segment,
        'cycle': cycle,
        'ms': cycle,
        'fps': round(float(1000.0 / cycle['mean']), 2),
        'host_pipeline': _percentile_block(
            [row['host_ms'] for row in steady]),
        'detections_mean': round(float(np.mean(
            [row['detections'] for row in steady])), 2),
        'candidates_mean': round(float(np.mean(
            [row['candidates'] for row in steady])), 2),
        'rows': rows,
    }
    print(json.dumps(summary, ensure_ascii=False))
    if args.json_out:
        args.json_out.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
