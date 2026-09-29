"""Checkerboard definition shared by the detector and the server."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Checkerboard:
    """Inner-corner grid of a planar checkerboard target."""

    # 运行时事实源是 config/calibration.yaml (server) 与内参标定 launch;
    # 此处默认值仅作纯核构造兜底 (单测依赖), 换板改 yaml 不改这里
    columns: int = 11
    rows: int = 8
    square_size_m: float = 0.020

    @property
    def pattern_size(self):
        return (int(self.columns), int(self.rows))

    @property
    def corner_count(self):
        return int(self.columns) * int(self.rows)

    @property
    def object_points(self):
        grid = np.zeros((self.corner_count, 3), dtype=np.float32)
        grid[:, :2] = np.mgrid[0:self.columns, 0:self.rows].T.reshape(-1, 2)
        grid[:, :2] *= float(self.square_size_m)
        return grid

    def describe(self):
        return (
            f'{self.columns}x{self.rows} inner corners, '
            f'square {self.square_size_m * 1000:.0f} mm'
        )

    def metadata(self):
        return {
            'inner_corners': [int(self.columns), int(self.rows)],
            'square_size_m': float(self.square_size_m),
        }


def board_from_node(node):
    """Build a Checkerboard from a node's declared parameters."""
    return Checkerboard(
        columns=int(node.get_parameter('board_columns').value),
        rows=int(node.get_parameter('board_rows').value),
        square_size_m=float(node.get_parameter('board_square_size_m').value),
    )
