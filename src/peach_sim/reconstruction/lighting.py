"""室外光照预设：纯数据 + 范围校验，零 Blender 依赖（pytest 可单测）.

四个预设覆盖果园一天的典型光照：晨光（低角暖光、长影）、正午（现行
基线值）、午后侧逆光、阴天（高浑浊度漫射、软影无直射）。Nishita 物理
天空承担色温与大气，预设只给参数；应用端在 build_scene.setup_render。
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
    turbidity: float
    """大气浑浊度 (1 清朗 – 10 浓雾/阴)；阴天用高值压掉直射感."""
    sun_size_deg: float
    """太阳视直径 [deg]；越大阴影越软（阴天放大）."""
    sky_strength: float
    """World Background 强度；天空整体亮度."""
    fill_energy: float
    """冠下补光 SUN 能量（穿叶散射的近似）."""
    fill_spread_deg: float
    """补光角半径 [deg]；软硬程度."""
    note: str


PRESETS: Dict[str, LightingPreset] = {
    # 正午 = 合并前基线原值（elevation 38 / rotation 125 / bg .11 / fill .75）
    'noon': LightingPreset(
        'noon', 38.0, 125.0, 2.7, 2.0, .11, .75, 3.4,
        '合并轮基线原值，作回归锚'),
    'morning': LightingPreset(
        'morning', 18.0, 170.0, 3.2, 2.5, .13, .55, 3.4,
        '低高度角暖光（顺光侧：相机在 -Y 作业道，azimuth 170° 太阳在 '
        '相机侧后方；60° 会把袋拍成逆光剪影，冒烟轮实测剔除）'),
    'late_afternoon': LightingPreset(
        'late_afternoon', 22.0, 235.0, 3.6, 3.0, .12, .65, 3.4,
        '西向侧光（azimuth 235° 同在相机侧），照向树冠侧面'),
    'overcast': LightingPreset(
        'overcast', 45.0, 150.0, 10.0, 10.0, .22, 1.1, 12.0,
        '高浑浊度漫射天空，太阳盘放大成软影，无硬直射'),
}

_RANGES = {
    'sun_elevation_deg': (5.0, 80.0),
    'sun_azimuth_deg': (0.0, 360.0),
    'turbidity': (1.0, 10.0),
    'sun_size_deg': (0.5, 15.0),
    'sky_strength': (.05, .6),
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
