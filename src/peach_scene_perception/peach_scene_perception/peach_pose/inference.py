"""
推理实现 — YOLO 检测器 / SAM 分割器的生命周期管理与组合引擎.

职责:
  封装 Ultralytics YOLO / SAM 的懒加载、推理与结果解析，为感知管线上游提供
  检测框与实例掩码，输出纯数据结构 (dict / ndarray)，不耦合 GUI。

在管线中的位置:
  **① YOLO 检测** (UltralyticsYolo.detect) → **② SAM 分割**
  (MobileSam.segment) → 下游收敛前景与几何拟合

分层（2.14 装配规则，A3 起）：
  - ``UltralyticsYolo``（Detector 默认实现，注册名 'yolo'）；
  - ``MobileSam``（Segmenter 默认实现，注册名 'mobile_sam'）；
  - ``InferenceEngine``（组合调用端）：只持有 Detector/Segmenter 接口引用，
    detect/segment/reset 纯委托；节点经 yaml detector.impl/segmenter.impl
    按名创建注入，离线 CLI 亦可直接装配。
  实现类在 impls.py 显式注册清单登记（暂不用自注册）。

核心理论要点:
  - 懒加载: 首次 detect/segment 时才加载权重，缩短冷启动
  - 依赖注入: 模型路径与阈值经构造函数传入，不读全局 config
  - CUDA 线程安全: 各实现持独立 threading.Lock 序列化自身 forward
    （旧版单锁同时串行 YOLO+SAM；拆分后各自一把锁，现网调用为单 worker
    线程顺序调用，行为等价）
  - YOLO 输出按 conf 降序；SAM 以 bbox 为 prompt，过滤过小掩码

线程安全说明:
  YOLO/SAM 在 GPU 上持有 CUDA context 与内部缓冲；多线程并发 forward 可能导致
  结果错乱或 CUDA 错误，故各实现的加载+推理路径持锁。GUI 侧 Open3D/Qt 应在
  主线程；若需后台推理，用 QThread + 信号槽，勿裸线程共享 engine。
"""

import logging
import threading
from typing import List, Tuple

import numpy as np

from .interfaces import Detector, Segmenter

_logger = logging.getLogger(__name__)


def _resolve_device() -> str:
    """
    选推理设备：有 CUDA 用 'cuda:0'，否则 'cpu'.

    Returns
    -------
        设备字符串（torch 未安装时视为无卡，回退 'cpu'）.

    """
    try:
        import torch
        if torch.cuda.is_available():
            return 'cuda:0'
    except ImportError:
        pass
    return 'cpu'


class UltralyticsYolo(Detector):
    """
    Ultralytics YOLO 检测器（Detector 默认实现，注册名 'yolo'）.

    懒加载：首次 detect 才读权重。所有推理经 self._lock 序列化，确保同一
    时刻仅一个线程占用 GPU 模型。
    """

    def __init__(self, yolo_model: str = '', yolo_conf: float = 0.3,
                 yolo_iou: float = 0.5, class_names: dict = None):
        """
        构造检测器（模型懒加载，首次推理时才读权重）.

        Args:
            yolo_model: YOLO 权重路径（.pt）；空串行为取决于 ultralytics.
            yolo_conf: YOLO 置信度阈值 [0, 1].
            yolo_iou: YOLO NMS IoU 阈值 [0, 1].
            class_names: {class_id: 名称}；None 用默认 {0: peach_bag,
                1: peach_nobag}.

        Returns
        -------
            无返回值（None）.

        """
        self._yolo_model_path = yolo_model
        self._yolo_conf = yolo_conf
        self._yolo_iou = yolo_iou
        self._class_names = class_names or {0: 'peach_bag', 1: 'peach_nobag'}
        # 懒加载: None 表示尚未 load 权重
        self._yolo = None
        # 推理设备：默认优先 CUDA（peach_scene_perception 要求 GPU）；无卡时回退 CPU
        self._device = _resolve_device()
        # CUDA 线程安全: 锁序列化 load + forward
        self._lock = threading.Lock()

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """
        对 RGB 图像运行 YOLO 目标检测 (管线步骤 ①).

        Args:
            rgb: (H, W, 3) BGR 图像 (OpenCV 惯例)

        Returns
        -------
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
                    'class_id': ci,
                    'class_name': self._class_names.get(ci, f'cls_{ci}'),
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'conf': cf,
                })

        dets.sort(key=lambda d: d['conf'], reverse=True)
        return dets

    def reset(self):
        """释放 YOLO 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._yolo = None


class MobileSam(Segmenter):
    """
    Ultralytics MobileSAM 分割器（Segmenter 默认实现，注册名 'mobile_sam'）.

    懒加载：首次 segment 才读权重。SAM 以 bbox 为 box prompt，在框内生成
    二值前景掩码；面积 < sam_min_area 的掩码被丢弃。所有推理经
    self._lock 序列化（CUDA 线程安全，同 UltralyticsYolo）。
    """

    def __init__(self, sam_model: str = 'mobile_sam.pt',
                 sam_max_bboxes: int = 16, sam_min_area: int = 100):
        """
        构造分割器（模型懒加载，首次推理时才读权重）.

        Args:
            sam_model: SAM 权重路径或模型名.
            sam_max_bboxes: 单次 SAM 推理的最大 prompt 框数（超出截断）；
                默认 16（阶段 D1 由 8 上调并参数化为 yaml sam_max_bboxes：
                室外多果场景一帧目标数常超 8，截断目标无掩膜被判 OCCLUDED）.
            sam_min_area: 掩膜最小像素数，过小丢弃.

        Returns
        -------
            无返回值（None）.

        """
        self._sam_model_name = sam_model
        self._sam_max_bboxes = sam_max_bboxes
        self._sam_min_area = sam_min_area
        # 懒加载: None 表示尚未 load 权重
        self._sam = None
        self._device = _resolve_device()
        self._lock = threading.Lock()

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        对 RGB 图像运行 SAM 实例分割 (管线步骤 ②).

        Args:
            rgb: (H, W, 3) BGR 图像
            bboxes: [(x1, y1, x2, y2), ...]，超过 sam_max_bboxes 时截断

        Returns
        -------
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
                # 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）
                _logger.warning('SAM 分割失败: %s', e)
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

    def reset(self):
        """释放 SAM 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._sam = None


class InferenceEngine:
    """
    检测/分割组合引擎（调用端）：只持有 Detector/Segmenter 接口引用.

    detect/segment/reset 全部委托给构造期注入的接口实现；引擎自身不含
    任何模型逻辑，可替换性由接口层注册表（2.14）保证。

    用法::

        engine = InferenceEngine(
            detector=UltralyticsYolo(yolo_model='best.pt'),
            segmenter=MobileSam(sam_model='mobile_sam.pt'),
        )
        dets = engine.detect(rgb)               # → list[dict]
        masks = engine.segment(rgb, bboxes)     # → list[(mask, bbox)]
    """

    def __init__(self, detector: Detector, segmenter: Segmenter):
        """
        装配检测器与分割器（接口引用，不绑死具体实现）.

        Args:
            detector: Detector 接口实现（如 UltralyticsYolo）.
            segmenter: Segmenter 接口实现（如 MobileSam）.

        Returns
        -------
            无返回值（None）.

        """
        self._detector = detector
        self._segmenter = segmenter

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """委托注入的 Detector（签名与语义见 Detector.detect）."""
        return self._detector.detect(rgb)

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """委托注入的 Segmenter（签名与语义见 Segmenter.segment）."""
        return self._segmenter.segment(rgb, bboxes)

    def reset(self):
        """释放两个模型的缓存（逐接口委托；实现方各自保证线程安全）."""
        self._detector.reset()
        self._segmenter.reset()
