"""推理引擎 — YOLO 检测 + SAM 分割的生命周期管理。

职责:
  封装 Ultralytics YOLO / SAM 的懒加载、推理与结果解析，为感知管线上游提供
  检测框与实例掩码，输出纯数据结构 (dict / ndarray)，不耦合 GUI。

在管线中的位置:
  **① YOLO 检测** (detect) → **② SAM 分割** (segment) → 下游收敛前景与几何拟合

核心理论要点:
  - 懒加载: 首次 detect/segment 时才加载权重，缩短 GUI 冷启动
  - 依赖注入: 模型路径与阈值经构造函数传入，不读全局 config
  - CUDA 线程安全: threading.Lock 序列化所有模型 forward，避免多线程竞态
  - YOLO 输出按 conf 降序；SAM 以 bbox 为 prompt，过滤过小掩码

主要对外 API:
  - InferenceEngine.detect(rgb) — 返回检测 dict 列表
  - InferenceEngine.segment(rgb, bboxes) — 返回 [(mask, bbox), ...]
  - InferenceEngine.segment_detections(...) — 从 detect 结果筛选后分割
  - InferenceEngine.reset() — 释放模型缓存

线程安全说明:
  YOLO/SAM 在 GPU 上持有 CUDA context 与内部缓冲；多线程并发 forward 可能导致
  结果错乱或 CUDA 错误。因此 with self._lock 包裹整个加载+推理路径。
  GUI 侧 Open3D/Qt 应在主线程；若需后台推理，用 QThread + 信号槽，勿裸线程共享 engine。

详见 docs/architecture.md「推理与模型」章节 (若已撰写)。
"""

import threading
import numpy as np
from typing import List, Optional, Tuple
from pathlib import Path


class InferenceEngine:
    """管理 YOLO + SAM 的加载、推理和缓存。

    所有推理方法通过 self._lock 序列化，确保同一时刻仅一个线程占用 GPU 模型。

    用法:
        engine = InferenceEngine(
            yolo_model="best.pt",
            sam_model="mobile_sam.pt",
        )
        dets = engine.detect(rgb)               # → list[dict]
        masks = engine.segment(rgb, bboxes)     # → list[(mask, bbox)]
    """

    def __init__(
        self,
        yolo_model: str = "",
        sam_model: str = "mobile_sam.pt",
        yolo_conf: float = 0.3,
        yolo_iou: float = 0.5,
        sam_max_bboxes: int = 8,
        sam_min_area: int = 100,
        class_names: dict = None,
    ):
        self._yolo_model_path = yolo_model
        self._sam_model_name = sam_model
        self._yolo_conf = yolo_conf
        self._yolo_iou = yolo_iou
        self._sam_max_bboxes = sam_max_bboxes
        self._sam_min_area = sam_min_area
        self._class_names = class_names or {0: "peach_bag", 1: "peach_nobag"}

        # 懒加载: None 表示尚未 load 权重
        self._yolo = None
        self._sam = None
        # 推理设备：默认优先 CUDA（peach_pose_ros2 要求 GPU）；无卡时回退 CPU
        self._device = self._resolve_device()

        # CUDA 线程安全: 全局锁序列化 YOLO/SAM 的 load + forward
        self._lock = threading.Lock()

    @staticmethod
    def _resolve_device() -> str:
        try:
            import torch
            if torch.cuda.is_available():
                return "cuda:0"
        except ImportError:
            pass
        return "cpu"
    # ═══════════════════════════════════════════════════════════════
    # YOLO 检测
    # ═══════════════════════════════════════════════════════════════

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """对 RGB 图像运行 YOLO 目标检测 (管线步骤 ①)。

        Args:
            rgb: (H, W, 3) BGR 图像 (OpenCV 惯例)

        Returns:
            [{"class_id", "class_name", "bbox": (x1,y1,x2,y2), "conf"}, ...]
            按置信度降序排列
        """
        with self._lock:
            if self._yolo is None:
                from ultralytics import YOLO
                self._yolo = YOLO(self._yolo_model_path)
                # 权重迁到目标设备；后续 predict 显式传 device，避免默认漂到 CPU
                try:
                    self._yolo.to(self._device)
                except Exception:
                    pass

            results = self._yolo(
                rgb, conf=self._yolo_conf, iou=self._yolo_iou,
                device=self._device, verbose=False)

        # 锁外解析: 纯 CPU 后处理，不涉及 CUDA
        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for i in range(len(r.boxes)):
                ci = int(r.boxes.cls[i])
                cf = float(r.boxes.conf[i])
                x1, y1, x2, y2 = r.boxes.xyxy[i].tolist()
                dets.append({
                    "class_id": ci,
                    "class_name": self._class_names.get(ci, f"cls_{ci}"),
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "conf": cf,
                })

        dets.sort(key=lambda d: d["conf"], reverse=True)
        return dets

    # ═══════════════════════════════════════════════════════════════
    # SAM 分割
    # ═══════════════════════════════════════════════════════════════

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """对 RGB 图像运行 SAM 实例分割 (管线步骤 ②)。

        SAM 以 bbox 为 box prompt，在框内生成二值前景掩码。

        Args:
            rgb: (H, W, 3) BGR 图像
            bboxes: [(x1, y1, x2, y2), ...]，超过 sam_max_bboxes 时截断

        Returns:
            [(binary_mask, bbox), ...]，面积 < sam_min_area 的掩码被丢弃
        """
        if not bboxes:
            return []

        with self._lock:
            if self._sam is None:
                from ultralytics import SAM
                self._sam = SAM(self._sam_model_name)
                try:
                    self._sam.to(self._device)
                except Exception:
                    pass

            # 限制 bbox 数量: SAM 批量推理显存与耗时随 N 增长
            if len(bboxes) > self._sam_max_bboxes:
                bboxes = bboxes[:self._sam_max_bboxes]

            try:
                results = self._sam(
                    rgb, bboxes=bboxes, device=self._device, verbose=False)
            except Exception as e:
                print(f"[InferenceEngine] SAM 分割失败: {e}")
                return []

        if not results or results[0].masks is None:
            return []

        masks = results[0].masks.data.cpu().numpy()  # GPU→CPU: (N, H, W) 概率图

        output = []
        for i, mask in enumerate(masks):
            bin_mask = mask > 0.5  # 阈值化得布尔前景掩码
            if bin_mask.sum() > self._sam_min_area:
                output.append((bin_mask, bboxes[i]))

        return output

    # ═══════════════════════════════════════════════════════════════
    # 便捷方法: 对高置信度检测结果做 SAM 分割
    # ═══════════════════════════════════════════════════════════════

    def segment_detections(
        self,
        rgb: np.ndarray,
        detections: List[dict],
        min_conf: float = 0.5,
        target_classes: set = None,
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """从 detect() 结果中筛选高置信度目标，再调用 segment()。

        典型用法: detect → 过滤 peach_bag/peach_nobag → segment 得掩码供前景管线。

        Args:
            rgb: BGR 图像
            detections: detect() 返回值
            min_conf: 置信度下限
            target_classes: 参与分割的 class_id 集合，None 时默认 {0, 1}

        Returns:
            同 segment()
        """
        if target_classes is None:
            target_classes = {0, 1}  # peach_bag, peach_nobag

        bboxes = [
            d["bbox"] for d in detections
            if d["class_id"] in target_classes and d["conf"] > min_conf
        ]
        return self.segment(rgb, bboxes)

    # ═══════════════════════════════════════════════════════════════
    # 资源管理
    # ═══════════════════════════════════════════════════════════════

    def reset(self):
        """释放 YOLO/SAM 缓存 (切换模型路径或数据集后调用)。线程安全。"""
        with self._lock:
            self._yolo = None
            self._sam = None
