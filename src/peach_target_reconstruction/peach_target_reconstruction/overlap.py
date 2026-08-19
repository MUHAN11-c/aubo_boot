"""
多帧点云刚性对齐的重叠度指标 — 纯 numpy/scipy（无 ROS 依赖）.

语义：对**相邻已采帧**（i-1 与 i）的 base 系点云做最近邻距离统计
（mean/median/p95，单位 [mm]），并输出每帧质心与装配总质心（单位 [m]）。
大云先按固定种子随机抽稀到 ≤max_points 再建树（8 帧 × 45 万点全量
cKDTree 太慢；种子固定保证结果可复现）。

真机判据（README 第「使用方法」节有完整排查顺序）：同一静态目标、4+ 视角
时 overlap mean < 5mm 且 p95 < 15mm 视为刚性对齐合格。
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np
from scipy.spatial import cKDTree

DEFAULT_MAX_POINTS = 20000  # 单侧抽稀上限（cKDTree 建树规模）
DEFAULT_SEED = 0            # 抽稀随机种子（固定保证可复现）


def subsample_points(cloud: np.ndarray, max_points: int = DEFAULT_MAX_POINTS,
                     seed: int = DEFAULT_SEED) -> np.ndarray:
    """
    固定种子随机抽稀（无放回）；点数不超上限时原样返回.

    Args:
        cloud: (N, 3) 点云 [m].
        max_points: 抽稀上限.
        seed: 随机种子（同 N 同 seed 结果一致，可复现）.

    Returns
    -------
        (M, 3) 抽稀后点云，M = min(N, max_points).

    """
    n = int(cloud.shape[0])
    if n <= max_points:
        return cloud
    rng = np.random.default_rng(seed)
    idx = rng.choice(n, size=int(max_points), replace=False)
    return cloud[idx]


def cloud_centroid(cloud: np.ndarray) -> Optional[np.ndarray]:
    """
    点云质心 [m].

    Args:
        cloud: (N, 3) 点云.

    Returns
    -------
        (3,) float64 质心；空云给 None.

    """
    if cloud is None or np.asarray(cloud).size == 0:
        return None
    return np.asarray(cloud, dtype=np.float64).reshape(-1, 3).mean(axis=0)


def nn_distance_stats_mm(cloud_a: np.ndarray, cloud_b: np.ndarray,
                         max_points: int = DEFAULT_MAX_POINTS,
                         seed: int = DEFAULT_SEED) -> Optional[dict]:
    """
    两朵点云抽稀后的最近邻距离统计（a→b 单向），单位 [mm].

    Args:
        cloud_a: (N, 3) 查询侧点云 [m].
        cloud_b: (M, 3) 建树侧点云 [m].
        max_points: 两侧各自抽稀上限.
        seed: 抽稀随机种子.

    Returns
    -------
        {'mean_mm', 'median_mm', 'p95_mm'}；任一侧为空给 None.

    """
    a = subsample_points(np.asarray(cloud_a), max_points, seed)
    b = subsample_points(np.asarray(cloud_b), max_points, seed)
    if a.size == 0 or b.size == 0:
        return None
    dist, _ = cKDTree(b).query(a, k=1)
    return {
        'mean_mm': float(np.mean(dist)) * 1000.0,
        'median_mm': float(np.median(dist)) * 1000.0,
        'p95_mm': float(np.percentile(dist, 95)) * 1000.0,
    }


def assembly_overlap_metrics(frames: List,
                             max_points: int = DEFAULT_MAX_POINTS,
                             seed: int = DEFAULT_SEED) -> dict:
    """
    已采帧列表 → 重叠度指标 dict（finalize 时调用）.

    Args:
        frames: CapturedFrame 列表（读各帧 cloud_base，[m]）.
        max_points: 单侧抽稀上限.
        seed: 抽稀随机种子.

    Returns
    -------
        dict，键：
        - ``pairs``：相邻帧统计列表 [{'i', 'mean_mm', 'median_mm',
          'p95_mm'}]，i 为对中较后帧的下标（对 = i-1 与 i）；不足 2 帧为空
        - ``frame_centroids_base``：每帧质心 [m] 列表（空云帧给 None）
        - ``centroid_base``：装配总质心 [m]；无任何点给 None

    """
    clouds = [f.cloud_base for f in frames]
    pairs = []
    for i in range(1, len(clouds)):
        stats = nn_distance_stats_mm(clouds[i - 1], clouds[i],
                                     max_points=max_points, seed=seed)
        if stats is None:
            continue
        pairs.append({'i': i, **stats})
    centroids = []
    for cloud in clouds:
        c = cloud_centroid(cloud)
        centroids.append(None if c is None else [float(v) for v in c])
    valid = [np.asarray(c) for c in clouds
             if c is not None and np.asarray(c).size]
    assembly_centroid = cloud_centroid(np.vstack(valid)) if valid else None
    return {
        'pairs': pairs,
        'frame_centroids_base': centroids,
        'centroid_base': (None if assembly_centroid is None
                          else [float(v) for v in assembly_centroid]),
    }


def summarize_pairs_mm(pairs: List) -> Optional[dict]:
    """
    聚合相邻对统计为一句话指标：mean 取各对平均，p95 取最差对.

    Args:
        pairs: assembly_overlap_metrics 返回的 pairs 列表.

    Returns
    -------
        {'mean_mm', 'p95_mm'}；空列表给 None.

    """
    if not pairs:
        return None
    return {
        'mean_mm': float(np.mean([p['mean_mm'] for p in pairs])),
        'p95_mm': float(np.max([p['p95_mm'] for p in pairs])),
    }
