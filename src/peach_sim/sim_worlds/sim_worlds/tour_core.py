"""
巡航工位序列纯核（零 ROS）：每株树一个停靠位，蛇形穿行树行.

停靠位几何与 ``scene.work_pose`` 同口径：车体中心在树行作业道侧
（行线 + side*row_gap），车体 +X 沿行（行驶方向），逐行蛇形换向。
行末换行走 headland 模式（农田机器人标准节拍，参考 Field Robot Event
virtual_maize_field 一类实现）：越过末株在开放区插一个走廊换行航点，
把大角度换向从树行内挪到无树区，避免窄过道掉头卡死。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .params import OrchardParams
from .scene import tree_positions

HEADLAND_CLEAR_M = 1.8  # 末株外移（行末棚架端柱在 1.1 m 处，须再让开）


@dataclass(frozen=True)
class Station:
    """一个巡航停靠位（map/世界系）."""

    tree_id: str
    row: int
    index: int
    x: float
    y: float
    yaw: float


def _headland_y(params: OrchardParams, row: int) -> float:
    """行末开放区 y：偶数行从 +Y 端出、奇数行从 -Y 端出（与蛇形方向一致）."""
    rows = params.rows
    last_tree_y = (rows.trees_per_row - 1) / 2.0 * rows.tree_spacing
    sign = 1.0 if row % 2 == 0 else -1.0
    return sign * (last_tree_y + HEADLAND_CLEAR_M)


def stations(params: OrchardParams) -> list[Station]:
    """
    全部树的停靠位 + 行末换行航点，蛇形排序.

    偶数行 +Y 升、奇数行 -Y 降；车头朝向 = 行进方向（+X 车体沿行）；
    侧向偏移统一取作业道侧 ``platform.side``。换行航点在两行之间的
    走廊中心、越过末株的开放区，朝向 = 横穿方向（+X）。
    """
    platform = params.platform
    ground = params.ground
    by_row: dict[int, list[tuple[str, int, tuple[float, float, float]]]] = {}
    for tree_id, row, index, trunk in tree_positions(params):
        by_row.setdefault(row, []).append((tree_id, index, trunk))

    def within(x: float, y: float) -> bool:
        return (-ground.size_y / 2.0 < y < ground.size_y / 2.0
                and -ground.size_x / 2.0 < x < ground.size_x / 2.0)

    out: list[Station] = []
    for row in sorted(by_row):
        trees = sorted(by_row[row], key=lambda item: item[1])
        ascending = row % 2 == 0
        if not ascending:
            trees = list(reversed(trees))
        row_x = (row - (params.rows.count - 1) / 2.0) * params.rows.spacing
        x = row_x + platform.side * platform.row_gap
        yaw = (math.pi / 2.0) if ascending else (-math.pi / 2.0)
        for tree_id, index, trunk in trees:
            y = trunk[1]
            if not within(x, y):
                raise ValueError(f'停靠位越出地面边界：{tree_id} ({x:.2f}, {y:.2f})')
            out.append(Station(
                tree_id=tree_id, row=row, index=index,
                x=round(x, 6), y=round(y, 6), yaw=round(yaw, 6)))
        # headland 换行航点（最后一行之后不出界，无需换行）
        if row != sorted(by_row)[-1]:
            corridor_x = row_x + params.rows.spacing / 2.0
            head_y = _headland_y(params, row)
            if not within(corridor_x, head_y):
                raise ValueError(
                    f'换行航点越出地面边界：({corridor_x:.2f}, {head_y:.2f})')
            out.append(Station(
                tree_id=f'headland_r{row}_exit', row=row, index=-1,
                x=round(corridor_x, 6), y=round(head_y, 6), yaw=0.0))
    return out


def stations_by_row(params: OrchardParams, rows: list[int]) -> list[Station]:
    """按行号过滤停靠位（巡航命令行 --rows 用，含“进入集内下一行”的换行航点）."""
    wanted = set(rows)
    selected: list[Station] = []
    for item in stations(params):
        if item.index == -1:
            # 换行航点只保留“从集内行出、进集内下一行”的那个
            if item.row in wanted and item.row + 1 in wanted:
                selected.append(item)
        elif item.row in wanted:
            selected.append(item)
    return selected
