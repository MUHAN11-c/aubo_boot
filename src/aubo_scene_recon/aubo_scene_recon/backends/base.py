"""融合后端抽象基类."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np


class FusionBackend(ABC):
    @abstractmethod
    def integrate(
        self,
        xyz_cam: np.ndarray,
        colors: Optional[np.ndarray],
        T_map_cam: np.ndarray,
        min_range: float,
        max_range: float,
    ) -> None:
        ...

    @abstractmethod
    def get_map(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def num_points(self) -> int:
        ...

    def get_o3d_cloud(self):
        """可选：返回 open3d.geometry.PointCloud（用于 save）."""
        raise NotImplementedError
