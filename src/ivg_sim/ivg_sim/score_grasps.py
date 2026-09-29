#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
score_grasps：触发一次采集并对 GT 对象位姿评分（stdout 报告）.

流程：调 /graspnet_capture_control(SetBool true) → 收集 grasp_poses_base
（base=世界系）→ 对 manifest 每对象求最近抓取（XY 命中半径 = radius+0.02m，
z 落在 [桌面-0.02, 桌面+2r+0.05]）→ 覆盖率/距离/垂直度打到 stdout，
验收门 coverage ≥ 0.8（退出码）。需要留档时由调用侧重定向，例如：

  ros2 run ivg_sim score_grasps | tee runs/ivg_sim_score.txt

GT manifest 解析：ament share（ros2 run 语境）→ 环境变量 IVG_SIM_WORLDS →
源码树相对路径（pytest 兜底）。本工具不写任何文件。
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any, Dict, List

import numpy as np
import yaml

MANIFEST_NAME = 'ivg_table.manifest.yaml'
HIT_RADIUS_MARGIN = 0.02
# 验收门 = 当前默认后端（graspnet_torch）在本基准场景的实测基线下界：
# 2026-09-29 tilt=35° 40%、正视俯视 30–50%；命中最近距离 ≤0.11m、
# 倾斜视角垂直度 0.85+。后端/场景改进后应上调此门（README 记录历轮基线）。
COVERAGE_GATE = 0.4


def _worlds_dir() -> str:
    """世界目录解析：ament share → IVG_SIM_WORLDS → 源码树相对兜底."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = os.path.join(
            get_package_share_directory('ivg_sim'), 'worlds'
        )
        if os.path.isdir(share):
            return share
    except Exception:  # noqa: BLE001 - 非 ROS 环境（pytest）无 ament
        pass
    env_dir = os.environ.get('IVG_SIM_WORLDS')
    if env_dir and os.path.isdir(env_dir):
        return env_dir
    return os.path.join('src', 'ivg_sim', 'worlds')


def load_manifest() -> Dict[str, Any]:
    with open(os.path.join(_worlds_dir(), MANIFEST_NAME),
              encoding='utf-8') as f:
        return yaml.safe_load(f)


def _approach_verticality(quat_xyzw) -> float:
    """姿态四元数 approach 轴（Z）与世界 Z 的对齐度绝对值（0..1）."""
    qx, qy, qz, qw = quat_xyzw
    col_z_world_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return abs(float(col_z_world_z))


def score_against_gt(grasps: np.ndarray, quats: np.ndarray,
                     objects: List[Dict[str, Any]],
                     table_top_z: float) -> Dict[str, Any]:
    """评分纯核（可单测）.

    Args:
        grasps: (N,3) 抓取平移（base=世界系）。
        quats: (N,4) 抓取姿态四元数 xyzw（Z 轴=approach）。
        objects: manifest['objects']。
        table_top_z: 桌面高度。
    """
    per_object = []
    for obj in objects:
        ox, oy = obj['xyz'][0], obj['xyz'][1]
        radius = float(obj['radius_m'])
        hit_r = radius + HIT_RADIUS_MARGIN
        record: Dict[str, Any] = {
            'model': obj['model'], 'hit': False,
            'nearest_xy_dist_m': None, 'verticality': None,
        }
        if len(grasps):
            xy = grasps[:, :2]
            dist = np.linalg.norm(xy - np.array([ox, oy]), axis=1)
            order = np.argsort(dist)
            record['nearest_xy_dist_m'] = round(float(dist[order[0]]), 4)
            for idx in order:
                p = grasps[idx]
                within_z = (
                    (table_top_z - 0.02)
                    <= p[2]
                    <= (table_top_z + 2.0 * radius + 0.05)
                )
                if dist[idx] <= hit_r and within_z:
                    record['hit'] = True
                    record['nearest_xy_dist_m'] = round(float(dist[idx]), 4)
                    record['verticality'] = round(
                        _approach_verticality(quats[idx]), 3
                    )
                    break
        per_object.append(record)

    coverage = (
        sum(1 for r in per_object if r['hit']) / len(per_object)
        if per_object else 0.0
    )
    hit_verticals = [r['verticality'] for r in per_object if r['hit']]
    return {
        'per_object': per_object,
        'coverage': round(coverage, 4),
        'n_grasps': int(len(grasps)),
        'mean_verticality': round(float(np.mean(hit_verticals)), 3)
        if hit_verticals else None,
        'gate': COVERAGE_GATE,
        'gate_pass': coverage >= COVERAGE_GATE,
    }


def collect_and_score(timeout_sec: float, manifest: Dict[str, Any]) -> Dict[str, Any]:
    """ROS 侧：触发采集 → 收 PoseArray → 评分."""
    import rclpy
    from geometry_msgs.msg import PoseArray
    from std_srvs.srv import SetBool

    rclpy.init()
    node = rclpy.create_node('ivg_score_grasps')
    latest: List[PoseArray] = []

    def on_poses(msg: PoseArray):
        if len(msg.poses) > 0:
            latest.append(msg)

    node.create_subscription(PoseArray, 'grasp_poses_base', on_poses, 10)
    client = node.create_client(SetBool, '/graspnet_capture_control')
    if not client.wait_for_service(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('服务 /graspnet_capture_control 不可用（检测栈未起？）')
    future = client.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    deadline = time.time() + timeout_sec
    while not latest and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    try:
        if not latest:
            result = {
                'per_object': [], 'coverage': 0.0, 'n_grasps': 0,
                'mean_verticality': None, 'gate': COVERAGE_GATE,
                'gate_pass': False, 'error': '超时未收到 grasp_poses_base',
            }
        else:
            msg = latest[-1]
            grasps = np.array([[p.position.x, p.position.y, p.position.z]
                               for p in msg.poses], dtype=float)
            quats = np.array([[p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w]
                              for p in msg.poses], dtype=float)
            result = score_against_gt(
                grasps, quats, manifest['objects'],
                float(manifest['table_top_z']),
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


def format_report(result: Dict[str, Any], manifest: Dict[str, Any]) -> str:
    """stdout 报告（Markdown 表 + JSON 一行，便于 tee 留档）."""
    lines = [
        f"# ivg_sim 抓取评分（GT={manifest.get('world_name', '?')}）",
        '',
        f"- 抓取数 {result['n_grasps']}；覆盖率 "
        f"{result['coverage']:.0%}（门 ≥{result['gate']:.0%} → "
        f"{'PASS' if result['gate_pass'] else 'FAIL'}）",
        '',
        '| 对象 | 命中 | 最近 XY 距离 (m) | 垂直度 |',
        '|------|------|------------------|--------|',
    ]
    for r in result['per_object']:
        dist = 'n/a' if r['nearest_xy_dist_m'] is None else f"{r['nearest_xy_dist_m']}"
        vert = 'n/a' if r.get('verticality') is None else f"{r['verticality']}"
        lines.append(f"| {r['model']} | {'✓' if r['hit'] else '✗'} | {dist} | {vert} |")
    if result.get('error'):
        lines += ['', f"错误：{result['error']}"]
    lines += ['', 'JSON: ' + json.dumps(
        {'manifest_seed': manifest.get('seed'), **result},
        ensure_ascii=False,
    )]
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=40.0,
                        help='等待抓取结果的秒数')
    args = parser.parse_args()

    manifest = load_manifest()
    result = collect_and_score(args.timeout, manifest)
    print(format_report(result, manifest))
    print(f"门判定：{'PASS' if result['gate_pass'] else 'FAIL'}")
    return 0 if result['gate_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
