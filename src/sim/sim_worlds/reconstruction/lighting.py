"""室外光照预设：纯数据 + 范围校验，零 Blender 依赖（pytest 可单测）.

五个预设覆盖顺光、侧光、逆光和阴天。Nishita 的空气/尘埃密度分别配置，
太阳只由一个 SUN 发光；阴天使用均匀漫射天空近似，不把沙尘天冒充阴天。
数值是可复现的外观条件，尚非现场辐照度标定。
"""

from dataclasses import asdict, dataclass
from typing import Dict


@dataclass(frozen=True)
class LightingPreset:
    name: str
    sun_elevation_deg: float
    """太阳高度角 [deg]；Nishita 低角自动偏暖."""
    sun_azimuth_deg: float
    """太阳方位角 [deg]（绕 Z，北偏东口径与 Nishita sun_rotation 一致）."""
    air_density: float
    """Nishita 空气密度；1 为清洁大气，不是 Preetham turbidity."""
    sun_size_deg: float
    """太阳视直径 [deg]；越大阴影越软（阴天放大）."""
    sky_strength: float
    """World Background 强度；天空整体亮度."""
    fill_energy: float
    """冠下补光 SUN 能量（穿叶散射的近似）."""
    fill_spread_deg: float
    """补光角半径 [deg]；软硬程度."""
    note: str
    dust_density: float = .5
    cloud_cover: float = 0.0
    exposure_ev: float = .5


PRESETS: Dict[str, LightingPreset] = {
    'noon': LightingPreset(
        'noon', 38.0, 125.0, 1.0, .53, .35, 3.0, .53,
        '清洁日间天空与单一太阳；保留参考太阳方位，重建外观基线'),
    'morning': LightingPreset(
        'morning', 18.0, 170.0, 1.0, .53, .30, 2.3, .8,
        '低角晨光；与 backlit 分开保留，不能替代逆光压力工况', .7),
    'late_afternoon': LightingPreset(
        'late_afternoon', 22.0, 235.0, 1.0, .53, .30, 2.5, .8,
        '午后侧光、长影与局部叶影', .65),
    'backlit': LightingPreset(
        'backlit', 20.0, 60.0, 1.0, .53, .25, 3.0, .53,
        '逆光困难组；保留暗袋面、亮叶缘，不按检出率剔除', .45),
    'overcast': LightingPreset(
        'overcast', 45.0, 150.0, 1.0, .53, .8, 0.0, .53,
        '均匀冷灰漫射天空近似，关闭太阳；无云层体积/气象标定', .5, 1.0),
}

_RANGES = {
    'sun_elevation_deg': (5.0, 80.0),
    'sun_azimuth_deg': (0.0, 360.0),
    'air_density': (0.0, 2.0),
    'dust_density': (0.0, 10.0),
    'cloud_cover': (0.0, 1.0),
    'exposure_ev': (-6.0, 6.0),
    'sun_size_deg': (0.5, 15.0),
    'sky_strength': (.05, 2.0),
    'fill_energy': (0.0, 3.0),
    'fill_spread_deg': (0.5, 20.0),
}


def validate(preset: LightingPreset) -> None:
    """范围校验，导入期对 PRESETS 全量执行（非法即拒绝启动）."""
    for field, (low, high) in _RANGES.items():
        value = getattr(preset, field)
        if not low <= value <= high:
            raise ValueError(
                f'{preset.name}.{field}={value} outside [{low}, {high}]')


for _preset in PRESETS.values():
    validate(_preset)


def get(name: str) -> LightingPreset:
    if name not in PRESETS:
        raise KeyError(
            f'unknown lighting preset {name!r}; '
            f'choose from {sorted(PRESETS)}')
    return PRESETS[name]


def manifest_entry(name: str) -> dict:
    """Manifest 里记录的完整光照参数（可追溯）."""
    return {'preset': name, **asdict(get(name))}
