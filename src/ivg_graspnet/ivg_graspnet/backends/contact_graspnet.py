# -*- coding: utf-8 -*-
"""
contact_graspnet 后端：vendored contact_graspnet_pytorch.

Contact-GraspNet PyTorch 移植，权重随 vendor 携带——clutter 场景抓取分布
优于 GraspNet-baseline，作为第二后端。

vendor 两处最小补丁（均在 contact_graspnet_lib 内，带 [ivg vendor patch]
注释）：pyrender 惰化导入（仅训练期需要）、torch>=2.6 weights_only 白名单。
输出转 GraspList 17 维契约：Contact-GraspNet 抓取系旋转列约定
[approach, width, height] 与 GraspNet-baseline 同族；开口取每抓取的
gripper_openings；height/depth 为固定夹爪近似值（可配）。

⚠️ approach 方向与末端约定是否需要 180° 翻转（approach_flip_z180）未经
真机验证，默认 False——执行端 publish_grasps_client.apply_grasp_z_flip
须与所选后端一致；P3 评估轮先在 RViz/Marker 上核对方向再授权执行。
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import sys
from typing import Optional

from ivg_graspnet.backends.base import BackendInfo, GraspBackend
from ivg_graspnet.backends.registry import register
from ivg_graspnet.grasp_core import GRASP_ARRAY_LEN, GraspList
import numpy as np


def _vendor_dir() -> str:
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg_root, 'contact_graspnet_lib')


def _default_ckpt_dir() -> str:
    return os.path.join(_vendor_dir(), 'checkpoints', 'contact_graspnet')


@dataclass
class ContactGraspnetConfig:
    """contact_graspnet 后端配置."""

    ckpt_dir: str = ''
    forward_passes: int = 1
    local_regions: bool = False   # 需 segmap；点云直入模式关闭
    filter_grasps: bool = False   # 需 segmap；点云直入模式关闭
    approach_flip_z180: bool = False  # 未经真机验证，默认不翻转
    assumed_height: float = 0.05  # 固定夹爪指高近似（GraspList height 语义）
    assumed_depth: float = 0.03   # approach 侧指深近似（GraspList depth 语义）


class ContactGraspnetBackend(GraspBackend):
    """Contact-GraspNet 推理后端（vendored，纯点云输入）."""

    def __init__(self, config: ContactGraspnetConfig):
        super().__init__(BackendInfo(
            name='contact_graspnet',
            description=(
                'vendored contact_graspnet_pytorch（Contact-GraspNet 移植，'
                'NVIDIA 非商用系许可，内部研究用）'
            ),
            approach_flip_z180=config.approach_flip_z180,
        ))
        self.config = config
        self._estimator = None

    # ------------------------------------------------------------------
    def _get_estimator(self):
        if self._estimator is not None:
            return self._estimator
        vendor = _vendor_dir()
        if vendor not in sys.path:
            sys.path.insert(0, vendor)
        from contact_graspnet_pytorch import config_utils
        from contact_graspnet_pytorch.checkpoints import CheckpointIO
        from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator

        ckpt_dir = self.config.ckpt_dir or _default_ckpt_dir()
        if not os.path.isdir(ckpt_dir):
            raise FileNotFoundError(f'contact_graspnet checkpoint 目录不存在: {ckpt_dir}')
        global_config = config_utils.load_config(
            ckpt_dir, batch_size=self.config.forward_passes
        )
        estimator = GraspEstimator(global_config)
        checkpoint_io = CheckpointIO(
            checkpoint_dir=os.path.join(ckpt_dir, 'checkpoints'),
            model=estimator.model,
        )
        checkpoint_io.load('model.pt')
        self._estimator = estimator
        return estimator

    # ------------------------------------------------------------------
    def detect(self, points_m: np.ndarray,
               rgb: Optional[np.ndarray] = None) -> GraspList:
        """
        对一帧点云做抓取检测（原始结果，未后处理）.

        Args:
            points_m: (N, 3) float，点云坐标系下的坐标（米），已过工作区过滤。
            rgb: 忽略（当前纯点云前向）。
        """
        estimator = self._get_estimator()
        points = np.asarray(points_m, dtype=np.float32).reshape(-1, 3)
        if points.shape[0] == 0:
            return GraspList()

        pred, scores, _contact_pts, openings = estimator.predict_scene_grasps(
            points,
            pc_segments={},
            local_regions=self.config.local_regions,
            filter_grasps=self.config.filter_grasps,
            forward_passes=self.config.forward_passes,
        )
        grasps = pred.get(-1) if hasattr(pred, 'get') else pred[-1]
        score_arr = scores.get(-1) if hasattr(scores, 'get') else scores[-1]
        opening_arr = (
            openings.get(-1) if hasattr(openings, 'get') else openings[-1]
        )
        if grasps is None or len(grasps) == 0:
            return GraspList()

        rows = np.zeros((len(grasps), GRASP_ARRAY_LEN), dtype=np.float64)
        rows[:, 0] = np.asarray(score_arr, dtype=np.float64)
        rows[:, 1] = np.asarray(opening_arr, dtype=np.float64)
        rows[:, 2] = self.config.assumed_height
        rows[:, 3] = self.config.assumed_depth
        rows[:, 4:13] = np.asarray(grasps, dtype=np.float64)[:, :3, :3].reshape(-1, 9)
        rows[:, 13:16] = np.asarray(grasps, dtype=np.float64)[:, :3, 3]
        rows[:, 16] = -1
        return GraspList(rows)

    def close(self) -> None:
        self._estimator = None


@register('contact_graspnet')
def _factory(params: dict) -> GraspBackend:
    model_path = str(params.get('model_path', ''))
    return ContactGraspnetBackend(ContactGraspnetConfig(
        ckpt_dir=model_path if model_path else '',
        forward_passes=int(params.get('forward_passes', 1)),
    ))
