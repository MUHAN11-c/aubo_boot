"""
Median-absolute-deviation outlier masks shared by solver and server.

两处历史实现语义不同, 在此逐字保留:
  * 一维 (逐帧重投影 RMS 过滤): 少于 min_values 时全部保留
  * 二维 (平移/旋转残差): 逐列归一化, 任一列超限即判离群
"""

import numpy as np


def mad_mask_1d(values, threshold=3.5, min_values=4):
    """返回一维 MAD 内点掩码; 样本太少时全部保留."""
    values = np.asarray(values, dtype=np.float64)
    if values.size < min_values:
        return np.ones(values.size, dtype=bool)
    median = np.median(values)
    mad = np.median(np.abs(values - median))
    scale = max(1.4826 * mad, 1e-9)
    return np.abs(values - median) / scale <= threshold


def mad_inliers_2d(errors, threshold=3.5, min_values=5):
    """二维 (m, deg) 残差的 MAD 内点掩码; 任一列超限即离群."""
    errors = np.asarray(errors, dtype=np.float64)
    if len(errors) < min_values:
        return np.ones(len(errors), dtype=bool)
    normalized = errors.copy()
    for column in range(2):
        median = np.median(normalized[:, column])
        mad = np.median(np.abs(normalized[:, column] - median))
        scale = max(1.4826 * mad, 1e-9)
        normalized[:, column] = np.abs(
            normalized[:, column] - median) / scale
    return np.max(normalized, axis=1) <= threshold
