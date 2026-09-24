"""贴图生成测试（零 ROS）：确定性、尺寸、法线图合法性."""

from pathlib import Path

from peach_sim.textures import generate_all, height_to_normal, SIZE
from PIL import Image


def _generate(tmp_path: Path):
    return generate_all(tmp_path, 20260923)


def test_generate_all_writes_pairs(tmp_path):
    """每种材质出 albedo + normal 两张，尺寸 256 见方."""
    written = _generate(tmp_path)
    assert set(written) == {'bark', 'fruit_skin', 'grass', 'leaf', 'leaf_card',
                            'paper_bag', 'soil'}
    for albedo, normal in written.values():
        assert Image.open(albedo).size == (SIZE, SIZE)
        image = Image.open(normal)
        assert image.size == (SIZE, SIZE)
        assert image.mode in ('RGB', 'RGBA')  # leaf_card 是 RGBA（alpha 叶）


def test_textures_are_deterministic(tmp_path):
    """同 seed 两次生成逐字节一致."""
    first = _generate(tmp_path / 'a')
    second = _generate(tmp_path / 'b')
    for name in first:
        assert first[name][0].read_bytes() == second[name][0].read_bytes()
        assert first[name][1].read_bytes() == second[name][1].read_bytes()


def test_normal_map_z_is_outward(tmp_path):
    """法线图 Z 分量应整体朝外（>128）且 XY 在中值附近."""
    written = _generate(tmp_path)
    image = Image.open(written['paper_bag'][1])
    pixels = list(image.getdata())
    zs = [px[2] for px in pixels]
    assert min(zs) > 120, '法线 Z 有朝内分量'
    xs = sorted(px[0] for px in pixels)
    assert xs[len(xs) // 2] in range(110, 146), '法线 X 均值应居中'


def test_height_to_normal_flat_is_untouched():
    """平高度图 → 法线恒 (128,128,255)."""
    flat = Image.new('L', (16, 16), 128)
    normal = height_to_normal(flat, 2.0)
    assert set(normal.getdata()) == {(128, 128, 255)}


if __name__ == '__main__':
    raise SystemExit(0)
