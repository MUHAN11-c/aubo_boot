"""分位数统计的逆采样：把 priors.json 的 p10/p50/p90 变成可复现随机源.

袋宽/高/倾角等真实测量分布只有分位数摘要（n 与 p10/p50/p90/mean），
这里用分段线性逆 CDF（p10→p50→p90 三段、段内线性、u 均匀）采样，
并返回所用分位 u，供 manifest 逐实例记录 source_percentile。纯核零
Blender/ROS 依赖。
"""

import math
from typing import Dict, Tuple

# Peach_nobag 框宽含幼果（p10 4.7 cm）。成熟果只取 p50–p90。
MATURE_U_MIN = 0.5
# 纸袋可见宽/高必须包住果并留下扎口与袋底的纸。
FRUIT_WIDTH_MARGIN_M = 0.015
FRUIT_HEIGHT_MARGIN_M = 0.035
# 假设密度，不是实测；只用于清单里的质量字段。
FRUIT_DENSITY_KG_M3 = 970.0


def _interp(x, x0, y0, x1, y1):
    if x1 <= x0:
        return y0
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def sample_percentile(stats: Dict, u: float) -> Tuple[float, float]:
    """Stats 须含 p10/p50/p90（measure_priors 的 _percentiles 产物）.

    u ∈ [0,1)：0→p10，0.5→p50，1→p90；出界自动夹紧。返回 (value, u)。
    """
    if not all(k in stats for k in ('p10', 'p50', 'p90')):
        raise ValueError(f'stats missing percentiles: {sorted(stats)}')
    u = min(max(u, 0.0), 1.0)
    if u < .5:
        return _interp(u, 0.0, stats['p10'], .5, stats['p50']), u
    return _interp(u, .5, stats['p50'], 1.0, stats['p90']), u


def sample_width_height(stats: Dict, rng) -> Dict:
    """宽度与高宽比独立采样，高度 = 宽 × 比；记录两个分位来源.

    stats: priors['splits']['Peach_bag']['classes']['0']，须含
    width_m / aspect_h_over_w 两组分位摘要。确定性来自传入 rng。
    """
    width, u_w = sample_percentile(stats['width_m'], rng.random())
    aspect, u_a = sample_percentile(stats['aspect_h_over_w'], rng.random())
    return {
        'width_m': width,
        'height_m': width * aspect,
        'width_source_percentile': round(u_w, 4),
        'aspect_source_percentile': round(u_a, 4),
        'source': 'Peach_bag priors percentile sampling (n=722)',
    }


def sample_mature_fruit(stats: Dict, rng) -> Dict:
    """成熟果横径取裸桃框宽的上半段（u≥0.5 → p50..p90）.

    高宽比夹到 ≤1，使赤道直径是包围球直径，袋内间隙按它计算。
    质量 = 970 kg/m³ × 椭球体积，密度是假设。
    """
    u = MATURE_U_MIN + (1.0 - MATURE_U_MIN) * rng.random()
    diameter, u_d = sample_percentile(stats['width_m'], u)
    aspect, u_a = sample_percentile(stats['aspect_h_over_w'], rng.random())
    aspect = min(1.0, max(0.85, aspect))
    radius = diameter / 2
    volume = 4 / 3 * math.pi * radius * radius * (radius * aspect)
    return {
        'diameter_m': diameter,
        'aspect': aspect,
        'mass_kg': FRUIT_DENSITY_KG_M3 * volume,
        'diameter_source_percentile': round(u_d, 4),
        'aspect_source_percentile': round(u_a, 4),
        'source': (
            'Peach_nobag class 1 mature half (u>=0.5, n=359); '
            'mass from assumed 970 kg/m3'),
    }


def sample_bag_for_fruit(stats: Dict, fruit: Dict, rng, attempts: int = 12) -> Dict:
    """抽一只装得下这颗果的袋；抽不到就把最后一次抬到下限.

    下限是果径加余量，同时保留原先的 0.22/0.24 m 护栏。
    """
    floor_w = fruit['diameter_m'] + FRUIT_WIDTH_MARGIN_M
    floor_h = fruit['diameter_m'] + FRUIT_HEIGHT_MARGIN_M
    last = None
    for _ in range(attempts):
        last = sample_width_height(stats, rng)
        if floor_w <= last['width_m'] <= .22:
            # 只抬高度。连宽度一起重抽会把袋宽推到分布上半段。
            if last['height_m'] < floor_h:
                last['height_m'] = min(.24, floor_h)
                last['dimension_fit'] = 'height_clamped'
            else:
                last['dimension_fit'] = 'resampled'
            return last
    last['width_m'] = min(.22, max(last['width_m'], floor_w))
    last['height_m'] = min(.24, max(last['height_m'], floor_h))
    last['dimension_fit'] = 'clamped'
    return last
