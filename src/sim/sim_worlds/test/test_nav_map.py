"""导航占据栅格生成测试（零 ROS）：内容、确定性、入库产物一致."""

import hashlib
from pathlib import Path

from sim_worlds.nav_map import (
    build_nav_map,
    FREE,
    map_yaml_text,
    OCCUPIED,
    write_pgm,
)
from sim_worlds.params import params_from_dict
import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
CONFIG = PKG_DIR / 'config' / 'orchard.yaml'
MAPS_DIR = PKG_DIR / 'maps'


def _params():
    raw = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw)
    assert problems == []
    return params


def test_map_dimensions_cover_ground():
    """栅格正好覆盖地面（32×22 m → 640×440 格），原点 = 左下角."""
    nav_map = build_nav_map(_params())
    assert nav_map.width == 640 and nav_map.height == 440
    assert nav_map.resolution == 0.05
    assert nav_map.origin == (-16.0, -11.0)


def test_trunks_and_posts_occupied_aisle_free():
    """树干/棚架立柱格占据，行间作业道中心自由."""
    nav_map = build_nav_map(_params())

    def cell(x: float, y: float) -> int:
        col, row = nav_map.world_to_cell(x, y)
        return nav_map.cells[row][col]

    # tree_r2_t3 干位（行 2 → x=0；株 3 → y=0）
    assert cell(0.0, 0.0) == OCCUPIED
    # 棚架端柱（行 0 → x=-8；y=-7.7）
    assert cell(-8.0, -7.7) == OCCUPIED
    # 行间作业道中心（x=-6 远离树行，y=0）
    assert cell(-6.0, 0.0) == FREE


def test_border_ring_is_lethal():
    """地面外沿致命环占据（0.2 m），防把车开出有限地面."""
    nav_map = build_nav_map(_params())
    border = 4  # 0.20 m / 0.05 m
    assert all(value == OCCUPIED for value in nav_map.cells[0][:20])
    assert all(value == OCCUPIED for value in nav_map.cells[-1][:20])
    assert all(nav_map.cells[row][0] == OCCUPIED
               for row in range(nav_map.height))
    assert nav_map.cells[nav_map.height // 2][border + 2] in (FREE, OCCUPIED)


def test_map_is_deterministic():
    """同参数两次生成字节一致."""
    first = build_nav_map(_params())
    second = build_nav_map(_params())
    assert first.cells == second.cells


def test_committed_map_matches_generator():
    """入库 maps/orchard_map.{pgm,yaml} 与重生成一致（防漂移）."""
    params = _params()
    nav_map = build_nav_map(params)
    pgm_path = MAPS_DIR / 'orchard_map.pgm'
    yaml_path = MAPS_DIR / 'orchard_map.yaml'
    assert pgm_path.exists() and yaml_path.exists()

    expected = bytearray()
    expected += f'P5\n{nav_map.width} {nav_map.height}\n255\n'.encode('ascii')
    for row in nav_map.cells:
        expected += bytes(row)
    assert hashlib.sha256(pgm_path.read_bytes()).hexdigest() \
        == hashlib.sha256(bytes(expected)).hexdigest()

    loaded = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert loaded['resolution'] == nav_map.resolution
    assert loaded['origin'][:2] == list(nav_map.origin)
    assert loaded['image'] == 'orchard_map.pgm'


def test_pgm_write_roundtrip(tmp_path):
    """write_pgm/map_yaml_text 输出可被 map_server 约定消费."""
    nav_map = build_nav_map(_params())
    pgm = tmp_path / 'm.pgm'
    write_pgm(nav_map, pgm)
    data = pgm.read_bytes()
    header = f'P5\n{nav_map.width} {nav_map.height}\n255\n'.encode('ascii')
    assert data.startswith(header)
    assert len(data) == len(header) + nav_map.width * nav_map.height
    text = map_yaml_text(nav_map, image_name='m.pgm')
    assert 'mode: trinary' in text and 'image: m.pgm' in text
