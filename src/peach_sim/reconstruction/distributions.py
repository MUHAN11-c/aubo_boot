"""分位数统计的逆采样：把 priors.json 的 p10/p50/p90 变成可复现随机源.

袋宽/高/倾角等真实测量分布只有分位数摘要（n 与 p10/p50/p90/mean），
这里用分段线性逆 CDF（p10→p50→p90 三段、段内线性、u 均匀）采样，
并返回所用分位 u，供 manifest 逐实例记录 source_percentile。纯核零
Blender/ROS 依赖。
"""

from typing import Dict, Tuple


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
