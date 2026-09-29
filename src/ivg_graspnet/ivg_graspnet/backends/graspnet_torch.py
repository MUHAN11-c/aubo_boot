# -*- coding: utf-8 -*-
"""
graspnet_torch 后端：vendored GraspNet-baseline（纯 torch 算子）前向 + 解码.

网络结构超参默认对应 models/checkpoint-rs.tar；权重同目录同名 .yaml
manifest（checkpoint manifest）存在时以 manifest 覆盖默认值——换 checkpoint
只换（权重 + manifest）两个文件，零改代码（对齐 contact_graspnet_pytorch
的 per-checkpoint config 模式）。

本后端只做「点云 → 原始 GraspList」；碰撞/NMS/topK 在 postprocess 模块。
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, replace
from typing import Optional

from ivg_graspnet.backends.base import BackendInfo, GraspBackend
from ivg_graspnet.backends.registry import register
from ivg_graspnet.grasp_core import GraspList
from ivg_graspnet.inference_session import InferenceSession, SessionConfig
import numpy as np

_MANIFEST_KEYS = (
    'num_point', 'num_view', 'num_angle', 'num_depth',
    'cylinder_radius', 'hmin', 'hmax_list',
)


@dataclass
class GraspNetTorchConfig:
    """graspnet_torch 后端配置（网络超参与 checkpoint 耦合，勿随意改动）."""

    checkpoint_path: str = ''
    device: str = 'auto'  # 'auto' | 'cuda[:idx]' | 'cpu'
    fp16: bool = False
    num_point: int = 20000
    num_view: int = 300
    num_angle: int = 12
    num_depth: int = 4
    cylinder_radius: float = 0.05
    hmin: float = -0.02
    hmax_list: tuple = (0.01, 0.02, 0.03, 0.04)
    # 未发现 manifest 时的提示级别（节点首次加载只 WARN 一次由调用方控制）
    warn_no_manifest: bool = field(default=True, repr=False)


def load_checkpoint_manifest(checkpoint_path: str) -> dict:
    """读权重同目录同名 .yaml manifest；不存在返回空 dict."""
    import os

    import yaml

    stem = os.path.splitext(checkpoint_path)[0]
    manifest_path = stem + '.yaml'
    if not os.path.exists(manifest_path):
        return {}
    with open(manifest_path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data


def config_with_manifest(config: GraspNetTorchConfig) -> GraspNetTorchConfig:
    """用 manifest 覆盖配置中与 checkpoint 耦合的网络超参."""
    manifest = load_checkpoint_manifest(config.checkpoint_path)
    if not manifest:
        return config
    if manifest.get('backend') not in (None, 'graspnet_torch'):
        raise ValueError(
            f'manifest 声明 backend={manifest.get("backend")!r}，'
            f'与 graspnet_torch 后端不匹配: {config.checkpoint_path}'
        )
    overrides = {k: manifest[k] for k in _MANIFEST_KEYS if k in manifest}
    if 'hmax_list' in overrides:
        overrides['hmax_list'] = tuple(overrides['hmax_list'])
    valid = {f.name for f in fields(GraspNetTorchConfig)}
    unknown = set(overrides) - valid
    if unknown:
        raise ValueError(f'manifest 含未知键: {sorted(unknown)}')
    return replace(config, **overrides)


class GraspNetTorchBackend(GraspBackend):
    """GraspNet-baseline 推理后端."""

    def __init__(self, config: GraspNetTorchConfig):
        super().__init__(BackendInfo(
            name='graspnet_torch',
            description='vendored GraspNet-baseline，纯 torch 算子（无 CUDA 扩展）',
            approach_flip_z180=True,
        ))
        self.config = config_with_manifest(config)
        self.session = InferenceSession(SessionConfig(
            device=self.config.device, fp16=self.config.fp16,
        ))
        self.net = self._load_model()

    # ------------------------------------------------------------------
    def _load_model(self):
        import os

        import torch

        from ivg_graspnet.graspnet_lib import GraspNet

        path = self.config.checkpoint_path
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f'GraspNet 权重不存在: {path!r}')

        net = GraspNet(
            input_feature_dim=0,
            num_view=self.config.num_view,
            num_angle=self.config.num_angle,
            num_depth=self.config.num_depth,
            cylinder_radius=self.config.cylinder_radius,
            hmin=self.config.hmin,
            hmax_list=list(self.config.hmax_list),
        )
        net.to(self.session.device)
        # 安全加载：weights_only=True 只允许张量/基础类型，防止 pickle 代码执行
        checkpoint = torch.load(
            path, map_location=self.session.device, weights_only=True
        )
        net.load_state_dict(checkpoint['model_state_dict'])
        net.eval()
        return net

    # ------------------------------------------------------------------
    def _sample_points(self, points: np.ndarray) -> np.ndarray:
        """采样/补齐到固定 num_point（与上游 demo 一致：固定种子、不足重复补齐）."""
        n_target = self.config.num_point
        rng = np.random.default_rng(1)
        if len(points) >= n_target:
            idxs = rng.choice(len(points), n_target, replace=False)
        else:
            idxs1 = np.arange(len(points))
            idxs2 = rng.choice(len(points), n_target - len(points), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        return points[idxs].astype(np.float32)

    # ------------------------------------------------------------------
    def detect(self, points_m: np.ndarray,
               rgb: Optional[np.ndarray] = None) -> GraspList:
        """
        对一帧点云做抓取检测（原始结果，未后处理）.

        Args:
            points_m: (N, 3) float，点云坐标系下的坐标（米），已过工作区过滤。
            rgb: 忽略（GraspNet-baseline 点云版不用颜色）。

        Returns:
            GraspList（decode 后、未经碰撞/NMS/topK 的原始抓取）。
        """
        import torch

        from ivg_graspnet.graspnet_lib import pred_decode

        points = np.asarray(points_m, dtype=np.float32).reshape(-1, 3)
        if points.shape[0] == 0:
            return GraspList()

        sampled = self._sample_points(points)
        cloud_tensor = torch.from_numpy(sampled[np.newaxis]).to(self.session.device)
        with torch.no_grad(), self.session.autocast():
            end_points = self.net({'point_clouds': cloud_tensor})
            grasp_preds = pred_decode(end_points)
        return GraspList(grasp_preds[0].detach().cpu().numpy())


@register('graspnet_torch')
def _factory(params: dict) -> GraspNetTorchBackend:
    return GraspNetTorchBackend(GraspNetTorchConfig(
        checkpoint_path=params['model_path'],
        device=params.get('device', 'auto'),
        fp16=bool(params.get('fp16', False)),
        num_point=int(params.get('num_point', 20000)),
    ))
