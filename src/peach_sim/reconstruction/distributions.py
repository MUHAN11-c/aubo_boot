"""分位数统计的逆采样：把 priors.json 的 p10/p50/p90 变成可复现随机源.

袋宽/高/倾角等真实测量分布只有分位数摘要（n 与 p10/p50/p90/mean），
这里用分段线性逆 CDF（p10→p50→p90 三段、段内线性、u 均匀）采样，
并返回所用分位 u，供 manifest 逐实例记录 source_percentile。纯核零
Blender/ROS 依赖。
"""

import math
from typing import Dict, Tuple

# VOC 0=无遮挡；尺寸上半段作为成熟内果是假设，不是成熟度标签。
MATURE_U_MIN = 0.5
# 纸贴果：径向间隙 5–9 mm。现场袋底到袋口 p50 7.0 cm，约等于果径。
FRUIT_SEAT_M = 0.022
PAPER_GAP_M = 0.005
PAPER_SLACK_M = (0.005, 0.009)
NECK_ABOVE_FRUIT_M = (0.010, 0.018)
NECK_HALF_M = 0.006
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
        'source': ('Peach_bag priors percentile sampling '
                   f"(n={stats['width_m'].get('n', 'not available')})"),
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
            'Peach_nobag non-occluded class 0 mature half '
            f"(u>=0.5, n={stats['width_m'].get('n', 'not available')}); "
            'mass from assumed 970 kg/m3'),
    }


def fruit_center_z(diameter: float) -> float:
    """Bag-local Z of the enclosed fruit centre (origin = paper floor)."""
    return diameter / 2 + FRUIT_SEAT_M


def wrap_radius(fruit_r: float, width: float) -> float:
    """Equator paper radius. Width encodes slack; Peach_bag boxes do not."""
    slack = width / 2 - fruit_r
    slack = min(max(slack, PAPER_GAP_M), PAPER_SLACK_M[1])
    return fruit_r + slack


def wrap_half_at(z: float, fruit_r: float, wrap_r: float, height: float):
    """Paper half-width and superellipse power at bag-local z.

    Cheek is a sphere about the fruit until just above the fruit, then a
    short gathered neck. No teardrop floor: that read as an empty bag.
    """
    z_c = fruit_r + FRUIT_SEAT_M
    fruit_top = z_c + fruit_r
    gather_start = min(height - 0.006, fruit_top + 0.001)
    dz = z - z_c
    sphere = (math.sqrt(max(0.0, wrap_r * wrap_r - dz * dz))
              if abs(dz) < wrap_r else 0.0)
    if z <= gather_start:
        return max(sphere, 0.004), 2.08
    span = max(height - gather_start, 1e-4)
    u = min(1.0, max(0.0, (z - gather_start) / span))
    start_dz = gather_start - z_c
    if abs(start_dz) < wrap_r:
        start_r = math.sqrt(max(1e-8, wrap_r * wrap_r - start_dz * start_dz))
    else:
        start_r = 0.004
    s = u * u * (3.0 - 2.0 * u)
    half = start_r * (1.0 - s) + NECK_HALF_M * s
    if u > 0.88:
        half = NECK_HALF_M + 0.003 * (u - 0.88) / 0.12
    return max(half, 0.004), 2.04


def sample_bag_for_fruit(stats: Dict, fruit: Dict, rng, attempts: int = 12) -> Dict:
    """Sample observed paper dimensions, bounded around an unchanged fruit.

    Bounding boxes are approximate projected dimensions. Their variation is
    retained within plausible paper bounds, not treated as calibrated meshes.
    """
    del attempts
    observed = sample_width_height(stats, rng)
    diameter = fruit['diameter_m']
    width = min(max(observed['width_m'], diameter * 1.25), diameter * 1.65)
    height = min(max(observed['height_m'], diameter + .055), diameter + .08)
    return {
        'width_m': width,
        'height_m': height,
        'width_source_percentile': observed['width_source_percentile'],
        'aspect_source_percentile': observed['aspect_source_percentile'],
        'observed_width_m': observed['width_m'],
        'observed_height_m': observed['height_m'],
        'source': 'Peach_bag class 0 paper quantiles; bounded to contain unchanged fruit',
        'dimension_fit': 'fruit_supported_paper',
    }
