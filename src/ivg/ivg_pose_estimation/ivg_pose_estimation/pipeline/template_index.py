# -*- coding: utf-8 -*-
"""
DINOv2 模板嵌入索引：离线建库、在线最近邻检索（FoundPose/CNOS 检索段工程化）.

对模板库每个 pose 目录的标准化图像提 DINOv2 CLS 嵌入，落盘
`<object_dir>/.dinov2_index.npz`（embeddings/template_ids/angles_deg）；
模板目录有更新（mtime 新于索引）自动重建。在线查询为余弦最近邻。

嵌入器可注入（测试用假模型）；缺 torch/网络时构建失败由调用方兜底。
"""

from __future__ import annotations

import os
from typing import Callable, List, Optional

import numpy as np

INDEX_FILENAME = '.dinov2_index.npz'
# ImageNet 统计量（DINOv2 预训练输入约定）
_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
_STD = np.array([0.229, 0.224, 0.225], np.float32)
_INPUT_SIZE = 224  # 14 的倍数

# 模板图优先级：标准化白底图 > 原图
_TEMPLATE_IMAGES = (
    'preprocessed_image.jpg', 'image.jpg', 'original_image.jpg',
)


def _load_template_image(pose_dir: str) -> Optional[np.ndarray]:
    import cv2

    for name in _TEMPLATE_IMAGES:
        path = os.path.join(pose_dir, name)
        if os.path.exists(path):
            bgr = cv2.imread(path, cv2.IMREAD_COLOR)
            if bgr is not None:
                return bgr
    return None


class TemplateEmbeddingIndex:
    """单工件模板库的 DINOv2 嵌入索引."""

    def __init__(self, model: str = 'dinov2_vits14', device: str = 'auto'):
        self.model_name = model
        self.device = device
        self._model = None
        self._load_error: Optional[Exception] = None
        # 注入式嵌入函数（测试）；生产走 _embed_dinov2
        self.embed_fn: Optional[Callable[[np.ndarray], np.ndarray]] = None
        self.embeddings: Optional[np.ndarray] = None
        self.template_ids: List[str] = []
        self.angles_deg: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    def _get_model(self):
        if self._model is not None or self._load_error is not None:
            return self._model
        try:
            import torch

            net = torch.hub.load(
                'facebookresearch/dinov2', self.model_name, verbose=False
            )
            device = (
                'cuda:0' if self.device in ('', 'auto') and torch.cuda.is_available()
                else (self.device if self.device not in ('', 'auto') else 'cpu')
            )
            self._model = net.to(device).eval()
            self._device = device
        except Exception as exc:  # noqa: BLE001 - 无 torch/网络/权重时兜底
            self._load_error = exc
            return None
        return self._model

    def preprocess(self, bgr: np.ndarray) -> np.ndarray:
        """BGR uint8 → (3,224,224) ImageNet 归一化 float32."""
        import cv2

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (_INPUT_SIZE, _INPUT_SIZE))
        arr = resized.astype(np.float32) / 255.0
        arr = (arr - _MEAN) / _STD
        return arr.transpose(2, 0, 1)

    def _embed_dinov2(self, bgr: np.ndarray) -> np.ndarray:
        import torch

        net = self._get_model()
        if net is None:
            raise RuntimeError(f'DINOv2 模型不可用: {self._load_error}')
        tensor = torch.from_numpy(self.preprocess(bgr))[None].to(self._device)
        with torch.no_grad():
            feat = net(tensor)
        vec = feat[0].float().cpu().numpy()
        return vec / (np.linalg.norm(vec) + 1e-9)

    def embed(self, bgr: np.ndarray) -> np.ndarray:
        """单图 L2 归一化嵌入."""
        if self.embed_fn is not None:
            vec = np.asarray(self.embed_fn(bgr), np.float64)
            return vec / (np.linalg.norm(vec) + 1e-9)
        return self._embed_dinov2(bgr)

    # ------------------------------------------------------------------
    def build(self, object_dir: str) -> bool:
        """扫描 pose_* 目录建索引并落盘；无可用模板图返回 False."""
        pose_dirs = []
        if os.path.isdir(object_dir):
            pose_dirs = sorted(
                os.path.join(object_dir, n)
                for n in os.listdir(object_dir)
                if os.path.isdir(os.path.join(object_dir, n)) and n_is_pose(n)
            )
        embeddings, template_ids, angles = [], [], []
        for pose_dir in pose_dirs:
            image = _load_template_image(pose_dir)
            if image is None:
                continue
            try:
                embeddings.append(self.embed(image))
            except Exception:  # noqa: BLE001 - 单模板失败不阻塞建库
                continue
            template_ids.append(os.path.basename(pose_dir))
            angles.append(_load_template_angle(pose_dir))
        if not embeddings:
            return False
        self.embeddings = np.stack(embeddings).astype(np.float32)
        self.template_ids = template_ids
        self.angles_deg = np.array(angles, np.float64)
        np.savez(
            os.path.join(object_dir, INDEX_FILENAME),
            embeddings=self.embeddings,
            template_ids=np.array(template_ids),
            angles_deg=self.angles_deg,
        )
        return True

    def load(self, object_dir: str) -> bool:
        """读盘载入（模板目录有更新则失效返回 False）."""
        index_path = os.path.join(object_dir, INDEX_FILENAME)
        if not os.path.exists(index_path):
            return False
        pose_dirs = [
            os.path.join(object_dir, n) for n in os.listdir(object_dir)
            if os.path.isdir(os.path.join(object_dir, n)) and n_is_pose(n)
        ]
        latest = max(
            (os.path.getmtime(d) for d in pose_dirs), default=0.0
        )
        if os.path.getmtime(index_path) < latest:
            return False  # 模板更新过 → 重建
        data = np.load(index_path, allow_pickle=False)
        self.embeddings = data['embeddings'].astype(np.float32)
        self.template_ids = [str(x) for x in data['template_ids'].tolist()]
        self.angles_deg = data['angles_deg'].astype(np.float64)
        return len(self.template_ids) > 0

    # ------------------------------------------------------------------
    def query(self, embedding: np.ndarray, top_k: int = 1):
        """余弦最近邻；返回 [(template_id, similarity, angle_deg), ...]."""
        if self.embeddings is None or not len(self.template_ids):
            return []
        query = np.asarray(embedding, np.float32)
        query = query / (np.linalg.norm(query) + 1e-9)
        sims = self.embeddings @ query
        order = np.argsort(-sims)[:max(1, int(top_k))]
        return [
            (self.template_ids[i], float(sims[i]), float(self.angles_deg[i]))
            for i in order
        ]


def n_is_pose(name: str) -> bool:
    return name.startswith('pose_')


def _load_template_angle(pose_dir: str) -> float:
    """读模板标准化角度（template_info.json 的 feature_parameters 段；缺省 0）."""
    import json

    info_path = os.path.join(pose_dir, 'template_info.json')
    try:
        with open(info_path, encoding='utf-8') as f:
            info = json.load(f)
        feature_params = info.get('feature_parameters', info)
        angle = feature_params.get(
            'standardized_angle_deg', feature_params.get('angle_deg', 0.0)
        )
        return float(angle)
    except Exception:  # noqa: BLE001 - 角度缺失按 0 处理
        return 0.0


def normalize_angle_180(angle_deg: float) -> float:
    """归一化到 [-180, 180]."""
    import math

    return math.atan2(math.sin(math.radians(angle_deg)),
                      math.cos(math.radians(angle_deg))) * 180.0 / math.pi
