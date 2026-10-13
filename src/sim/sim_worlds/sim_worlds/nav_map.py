"""
果园导航栅格地图确定性生成（纯核，零 ROS）.

与 ``scene.py`` 同源消费 ``OrchardParams``：占据物取 2D 激光扫描平面
（z≈0.45 m）可见的立面——树干（保守上界半径）与棚架立柱；地面外沿加
致命边环防止把车开出有限地面。产物 PGM(P5)+YAML 与世界同步重生成，
不做 SLAM 也能起 AMCL；``slam:=True`` 模式可另做真实激光校验。
坐标约定 REP-103：行沿世界 Y，行距沿 X；栅格原点 = 地面左下角。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from .params import OrchardParams
from .scene import tree_positions

RESOLUTION = 0.05
FREE, OCCUPIED = 254, 0  # PGM：白=自由，黑=占据（map_server trinary）
BORDER_M = 0.20  # 地面外沿致命环宽（车体不越出有限地面）
MARGIN_M = 0.06  # 树干/立柱占据外扩（含栅格化离散余量）
TRELLIS_POST_RADIUS = 0.045  # 与 scene._trellis 立柱半径同源


@dataclass(frozen=True)
class NavMap:
    """占据栅格：cells 行主序，行 0 = 最高 y（PGM 顶行约定）."""

    width: int
    height: int
    resolution: float
    origin: tuple[float, float]
    cells: tuple[tuple[int, ...], ...]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """世界坐标 → (列, PGM 行索引)."""
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row_bottom = int(math.floor((y - self.origin[1]) / self.resolution))
        return col, self.height - 1 - row_bottom

    def cell_world(self, col: int, row: int) -> tuple[float, float]:
        """(列, PGM 行索引) → 该格中心世界坐标."""
        y = self.origin[1] + (self.height - row - 0.5) * self.resolution
        x = self.origin[0] + (col + 0.5) * self.resolution
        return x, y


def _trellis_post_positions(params: OrchardParams) -> list[tuple[float, float]]:
    """复刻 scene._trellis 的立柱布置（确定性，无随机）."""
    rows = params.rows
    posts: list[tuple[float, float]] = []
    for row in range(rows.count):
        row_x = (row - (rows.count - 1) / 2.0) * rows.spacing
        post_ys = [(-0.5 - (rows.trees_per_row - 1) / 2.0) * rows.tree_spacing,
                   ((rows.trees_per_row - 1) / 2.0 + 0.5) * rows.tree_spacing]
        post_ys += [
            (index - (rows.trees_per_row - 1) / 2.0) * rows.tree_spacing
            + rows.tree_spacing / 2.0
            for index in range(0, rows.trees_per_row - 1, 3)
        ]
        posts.extend((row_x, y) for y in post_ys)
    return posts


def build_nav_map(params: OrchardParams) -> NavMap:
    """由场景参数确定性生成占据栅格."""
    ground = params.ground
    width = int(round(ground.size_x / RESOLUTION))
    height = int(round(ground.size_y / RESOLUTION))
    origin = (-ground.size_x / 2.0, -ground.size_y / 2.0)

    grid = [[FREE] * width for _ in range(height)]

    def stamp(x: float, y: float, radius: float) -> None:
        col0 = int(math.floor((x - radius - origin[0]) / RESOLUTION))
        col1 = int(math.ceil((x + radius - origin[0]) / RESOLUTION))
        row0 = int(math.floor((y - radius - origin[1]) / RESOLUTION))
        row1 = int(math.ceil((y + radius - origin[1]) / RESOLUTION))
        for row_bottom in range(max(row0, 0), min(row1 + 1, height)):
            row = height - 1 - row_bottom
            for col in range(max(col0, 0), min(col1 + 1, width)):
                cx = origin[0] + (col + 0.5) * RESOLUTION
                cy = origin[1] + (row_bottom + 0.5) * RESOLUTION
                if (cx - x) ** 2 + (cy - y) ** 2 <= radius ** 2:
                    grid[row][col] = OCCUPIED

    trunk_radius = params.tree.trunk_radius[1] + MARGIN_M
    for _, _, _, trunk in tree_positions(params):
        stamp(trunk[0], trunk[1], trunk_radius)
    for post in _trellis_post_positions(params):
        stamp(post[0], post[1], TRELLIS_POST_RADIUS + MARGIN_M)

    # 地面外沿致命环：越界即坠落，导航层面按不可通行处理
    border_cells = int(round(BORDER_M / RESOLUTION))
    for row in range(height):
        for col in range(width):
            if row < border_cells or row >= height - border_cells \
                    or col < border_cells or col >= width - border_cells:
                grid[row][col] = OCCUPIED

    return NavMap(
        width=width, height=height, resolution=RESOLUTION, origin=origin,
        cells=tuple(tuple(row) for row in grid))


def write_pgm(nav_map: NavMap, path: Path) -> None:
    """PGM P5 二进制写出."""
    data = bytearray()
    data += f'P5\n{nav_map.width} {nav_map.height}\n255\n'.encode('ascii')
    for row in nav_map.cells:
        data += bytes(row)
    path.write_bytes(bytes(data))


def map_yaml_text(nav_map: NavMap, image_name: str = 'orchard_map.pgm') -> str:
    """map_server 侧 YAML 文本."""
    return (
        f'image: {image_name}\n'
        f'resolution: {nav_map.resolution}\n'
        f'origin: [{nav_map.origin[0]}, {nav_map.origin[1]}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
        'mode: trinary\n')
