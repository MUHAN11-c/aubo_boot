"""
推理层：YOLO 检测 / MobileSAM 分割引擎与候选估计器（看场景算法核）.

torch / ultralytics 惰性导入（无权重环境仍可 import 本模块做几何侧
冒烟）；掩膜门控委托 image_gates，位姿评估委托 pose_pipelines。
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np

from peach_harvester.vision.common.geometry import crop_mask_to_bbox

from .contracts import (
    BagGrasp2D,
    BagGraspReference3D,
    BagObservation,
)
from .image_gates import clip_bbox, foreground_mask, valid_depth_mask
from .pose_pipelines import (
    RobustBagPosePipeline,
    TargetPoseResult,
)

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


class UltralyticsYolo:
    """
    Ultralytics YOLO 检测器（唯一实现，直接构造）.

    懒加载：首次 detect 才读权重。所有推理经 self._lock 序列化，确保同一
    时刻仅一个线程占用 GPU 模型。
    """

    def __init__(self, yolo_model: str = '', yolo_conf: float = 0.3,
                 yolo_iou: float = 0.5, yolo_imgsz: int = 640,
                 yolo_half: bool = False, class_names: dict = None):
        """
        构造检测器（模型懒加载，首次推理时才读权重）.

        Args:
            yolo_model: YOLO 权重路径（.pt）；空串行为取决于 ultralytics.
            yolo_conf: YOLO 置信度阈值 [0, 1].
            yolo_iou: YOLO NMS IoU 阈值 [0, 1].
            yolo_imgsz: 推理输入边长（px，ultralytics imgsz；640=现行为，
                须为 32 的倍数；构造期捕获）.
            yolo_half: FP16 推理开关（档案化：默认 False=现行为，仅在已
                验证半精度的设备上开启；构造期捕获）.
            class_names: {class_id: 名称}；None 用默认 {0: peach_bag,
                1: peach_nobag}.

        Returns
        -------
            无返回值（None）.

        """
        self._yolo_model_path = yolo_model
        self._yolo_conf = yolo_conf
        self._yolo_iou = yolo_iou
        self._yolo_imgsz = int(yolo_imgsz)
        self._yolo_half = bool(yolo_half)
        self._class_names = class_names or {0: 'peach_bag', 1: 'peach_nobag'}
        # 懒加载: None 表示尚未 load 权重
        self._yolo = None
        # 推理设备：默认优先 CUDA（peach_scene_perception 要求 GPU）；无卡时回退 CPU
        self._device = _resolve_device()
        # CUDA 线程安全: 锁序列化 load + forward
        self._lock = threading.Lock()

    def _warmup(self):
        """
        首个真实帧前对 imgsz×imgsz 零图热身一次（PF-6）.

        目的：构造 CUDA 上下文 / 推理引擎缓存，把首个真实帧的首推理
        延迟尖峰从现场帧路径挪到启动路径。失败仅告警不阻断——热身是
        性能优化，真实错误由首个真实帧的正常报告路径暴露。
        """
        try:
            self._yolo(
                np.zeros((self._yolo_imgsz, self._yolo_imgsz, 3),
                         dtype=np.uint8),
                conf=self._yolo_conf, iou=self._yolo_iou,
                imgsz=self._yolo_imgsz, half=self._yolo_half,
                device=self._device, verbose=False)
        except Exception as exc:  # noqa: BLE001 热身失败不阻断推理
            _logger.warning('YOLO warmup 失败（不阻断，首帧照常推理）: %s', exc)

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
                self._warmup()

            results = self._yolo(
                rgb, conf=self._yolo_conf, iou=self._yolo_iou,
                imgsz=self._yolo_imgsz, half=self._yolo_half,
                device=self._device, verbose=False)

        # 锁外解析: 纯 CPU 后处理，不涉及 CUDA
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for i in range(len(r.boxes)):
                class_id = int(r.boxes.cls[i])
                conf = float(r.boxes.conf[i])
                x1, y1, x2, y2 = clip_bbox(
                    r.boxes.xyxy[i].tolist(), rgb.shape)
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append({
                    'class_id': class_id,
                    'class_name': self._class_names.get(class_id, f'cls_{class_id}'),
                    'bbox': (x1, y1, x2, y2),
                    'conf': conf,
                })

        detections.sort(key=lambda d: d['conf'], reverse=True)
        return detections

    def reset(self):
        """释放 YOLO 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._yolo = None


class MobileSam:
    """
    Ultralytics MobileSAM 分割器（唯一实现，直接构造）.

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
                # W2/V1：不吞异常——上层 pipeline 的逐目标回退分支
                # （pipeline.process 的 try/except）依赖本方法抛出才会执行；
                # 吞成 return [] 会让回退成死路径、全帧掩膜静默缺失。
                # 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）。
                _logger.warning('SAM 批量分割失败: %s', e)
                raise

        if not results or results[0].masks is None:
            return []

        masks = results[0].masks.data.cpu().numpy()  # GPU→CPU: (N, H, W) 概率图
        ih, iw = rgb.shape[:2]

        output = []
        for i, mask in enumerate(masks):
            bin_mask = mask > 0.5  # 阈值化得布尔前景掩码
            if bin_mask.shape != (ih, iw):
                bin_mask = cv2.resize(
                    bin_mask.astype(np.uint8), (iw, ih),
                    interpolation=cv2.INTER_NEAREST).astype(bool)
            if bin_mask.sum() > self._sam_min_area:
                output.append((bin_mask, bboxes[i]))

        return output

    def reset(self):
        """释放 SAM 缓存 (切换模型路径或数据集后调用)。线程安全."""
        with self._lock:
            self._sam = None


class InferenceEngine:
    """
    检测/分割组合引擎：持有 UltralyticsYolo + MobileSam（直接构造）.

    detect/segment/reset 全部委托给构造期注入的实现；引擎自身不含模型逻辑。

    用法::

        engine = InferenceEngine(
            detector=UltralyticsYolo(yolo_model='best.pt'),
            segmenter=MobileSam(sam_model='mobile_sam.pt'),
        )
        detections = engine.detect(rgb)               # → list[dict]
        masks = engine.segment(rgb, bboxes)     # → list[(mask, bbox)]
    """

    def __init__(self, detector, segmenter):
        """
        装配检测器与分割器（唯一实现直接注入）.

        Args:
            detector: UltralyticsYolo.
            segmenter: MobileSam.

        Returns
        -------
            无返回值（None）.

        """
        self._detector = detector
        self._segmenter = segmenter

    def detect(self, rgb: np.ndarray) -> List[dict]:
        """委托注入的检测器."""
        return self._detector.detect(rgb)

    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """委托注入的分割器."""
        return self._segmenter.segment(rgb, bboxes)

    def reset(self):
        """释放两个模型的缓存（实现方各自保证线程安全）."""
        self._detector.reset()
        self._segmenter.reset()


# W13-A 压缩：原 ForegroundMode dataclass + FOREGROUND_MODES 单模式
# 注册表 + MODE_LABELS 已删（三者全仓零消费者）；估计路由/计时键
# 单源本常量，语义与旧 tuple(mode.mode_id ...) 完全一致。
MODE_IDS = ('hybrid_dilated',)


class CandidateEstimator:
    """
    构造收敛前景掩膜，并送入安全管线评估.

    按检测类别分流：
      - ``peach_bag`` (class_id=0) → 圆柱轴袋线 ``RobustBagPosePipeline``
      - ``peach_nobag`` (class_id=1) → 球+梗腔果线（仅 ``fruit_pipeline`` 已注入）
    两线共用同一圆柱刀具、入口/行程公式与安全门控。
    ``fruit_pipeline=None`` 时不构造果线；误入的裸果不走袋线。
    """

    pipeline: RobustBagPosePipeline
    """袋装线；None 构造时默认 RobustBagPosePipeline."""
    fruit_pipeline: Optional[RobustBagPosePipeline]
    """裸果线；None=关闭，class_id=1 不走袋线."""
    dilate_px: int
    """深度连通域膨胀半径 [px]，≥1."""
    min_mask_points: int
    """掩膜最少像素，不足 mask_unavailable."""
    last_timings_ms: dict
    """最近一拍几何耗时 [ms]."""

    def __init__(self, pipeline: Optional[RobustBagPosePipeline] = None,
                 fruit_pipeline: Optional[RobustBagPosePipeline] = None,
                 dilate_px: int = 5, min_mask_points: int = 50):
        """
        构造估计器；袋/果两条线由节点经 PIPELINES_BY_IMPL 注入.

        Args:
            pipeline: 袋线实例；None 时新建 RobustBagPosePipeline.
            fruit_pipeline: 果线实例；None=裸果线关闭（不默认新建）.
            dilate_px: 深度连通域膨胀半径（像素，≥1；核边长 2*(p//2)+1）.
            min_mask_points: 掩膜最小像素数，不足判 mask_unavailable.

        Returns
        -------
            无返回值（None）.

        """
        self.pipeline = pipeline or RobustBagPosePipeline()
        self.fruit_pipeline = fruit_pipeline
        # 类别路由：class_id==1 → 'fruit'（仅注入时），其余 → 'bag'
        self._estimator_by_kind = {'bag': self.pipeline}
        if fruit_pipeline is not None:
            self._estimator_by_kind['fruit'] = fruit_pipeline
        self.dilate_px = max(1, int(dilate_px))
        self.min_mask_points = max(1, int(min_mask_points))
        self.last_timings_ms: dict[str, float] = {}
        self._last_mask_timings_ms: dict[str, float] = {}
        # W13-A：膨胀椭圆核按边长缓存（原每目标 getStructuringElement 重建
        # 是纯重复分配；dilate_px 为公开属性可后改，故按核边长字典缓存）.
        self._dilate_kernel_cache: dict[int, np.ndarray] = {}

    def _pipeline_for(self, obs: BagObservation) -> tuple:
        """
        按检测 class_id 选择袋线 / 果线.

        Args:
            obs: 单帧输入（class_id 取 detections[0]）.

        Returns
        -------
            (kind, pipeline)：class_id==1 → fruit，否则 bag；
            果线未注入时 pipeline 为 None（不退回袋线）.

        """
        class_id = 0
        detections = list(obs.detections or [])
        if detections:
            class_id = int(detections[0].get('class_id', 0))
        kind = 'fruit' if class_id == 1 else 'bag'
        return kind, self._estimator_by_kind.get(kind)

    def estimate_modes(self, obs: BagObservation, target_id: str, bbox: tuple,
                       sam_mask: Optional[np.ndarray],
                       modes: Optional[Iterable[str]] = None
                       ) -> dict[str, TargetPoseResult]:
        """
        对请求的前景模式跑同一套几何与安全门控，返回 mode→结果.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            sam_mask: 全图 SAM 掩膜或 None（None → 各模式 mask_unavailable）.
            modes: 要跑的模式 ID 可迭代；None 跑全部已注册模式；
                含未知 ID 抛 ValueError.

        Returns
        -------
            {mode_id: TargetPoseResult}；掩膜不可用时结果为显式 REOBSERVE；
            副作用：刷新 last_timings_ms（毫秒，含掩膜构造耗时）.

        """
        selected = tuple(modes or MODE_IDS)
        unknown = set(selected) - set(MODE_IDS)
        if unknown:
            raise ValueError(f'unknown foreground modes: {sorted(unknown)}')

        bbox = clip_bbox(bbox, obs.depth.shape)
        kind, pipeline = self._pipeline_for(obs)
        if pipeline is None:
            # 果线未注入：不走袋线，避免裸果被当袋进身份表。
            self.last_timings_ms = {}
            results = {}
            for mode in selected:
                results[mode] = self._unavailable(
                    obs, target_id, bbox, mode, 'fruit_pipeline_disabled')
                results[mode].target_kind = kind
            return results
        masks, valid_roi, sam_yield, raw_sam_roi = self.build_masks(
            obs, bbox, sam_mask)
        # 袋线透传原始 SAM 掩膜（未经深度门控）：2D+3D 融合补全袋长用；
        # 果线签名无此参数（球心/半径来自点云，不受剪影影响）
        est_kwargs = {'raw_mask': raw_sam_roi} if kind == 'bag' else {}
        results = {}
        self.last_timings_ms = {}
        for mode in selected:
            started = time.perf_counter()
            mask = masks.get(mode)
            if mask is None:
                # SAM 缺失或交后像素不足：显式 REOBSERVE，不走深度-only 回退
                results[mode] = self._unavailable(
                    obs, target_id, bbox, mode, 'mask_unavailable')
            else:
                results[mode] = pipeline.estimate(
                    obs, target_id, bbox, mask, self._source(mode),
                    valid_roi=valid_roi, target_valid_ratio=sam_yield,
                    **est_kwargs)
            pose = results[mode].grasp_3d
            results[mode].target_kind = kind
            pose.strategy_id = f'robust_{kind}_pose:{mode}'
            pose.model_version = str(obs.metadata.get('model_version', 'unknown'))
            pose.calibration_version = str(obs.metadata.get(
                'calibration_version', 'unknown'))
            pose.tool_version = self.pipeline.tool.version
            geometry_ms = (time.perf_counter() - started) * 1000.0
            # 总耗时 = 掩膜构造 + 本模式几何
            self.last_timings_ms[mode] = (
                self._last_mask_timings_ms.get(mode, 0.0) + geometry_ms)
        return results

    def build_masks(self, obs: BagObservation, bbox: tuple,
                    sam_mask: Optional[np.ndarray]
                    ) -> tuple[dict[str, Optional[np.ndarray]],
                               Optional[np.ndarray], Optional[float],
                               Optional[np.ndarray]]:
        """
        在 bbox ROI 内构造 hybrid_dilated 掩膜（实测深度单位：毫米 uint16）.

        hybrid_dilated = (SAM ∩ 有效深度) ∩ 膨胀后的深度连通域；
        交后像素 < min_mask_points 给 None。SAM 缺失时跳过深度连通域白算.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            bbox: (x1, y1, x2, y2) 已裁到图内的检测框（像素）.
            sam_mask: 全图或 ROI 掩膜；None 或裁剪失败则结果为 None.

        Returns
        -------
            ({mode_id: ROI 掩膜或 None}, ROI 有效深度掩膜或 None,
            sam_yield 或 None, 原始 SAM ROI 掩膜或 None)：
            sam_yield = |SAM∩valid|/|SAM|（ROI 内），
            即「分割声称的目标里有多少具备有效深度」。作为
            target_valid_ratio 传给位姿管线，取代旧 ROI 均值口径——
            检测框背景超窗会把 ROI 均值压到掩膜占比量级（实测 0.09 vs
            目标真实 ~0.9），连带压垮 confidence/σ/遮挡分类；
            原始 SAM ROI（未经深度门控）透传给袋线做 2D+3D 融合补全
            袋长；副作用：刷新 _last_mask_timings_ms（毫秒）.

        """
        started = time.perf_counter()
        self._last_mask_timings_ms = {mode: 0.0 for mode in MODE_IDS}
        x1, y1, x2, y2 = bbox
        roi = obs.depth[y1:y2, x1:x2]
        empty = {mode: None for mode in MODE_IDS}
        if roi.size == 0:
            return empty, None, None, None
        sam_roi = self._crop_mask(sam_mask, (x1, y1, x2, y2), obs.depth.shape)
        if sam_roi is None:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._last_mask_timings_ms = {mode: elapsed_ms for mode in MODE_IDS}
            return empty, None, None, None
        valid = valid_depth_mask(
            roi, self.pipeline.min_depth_m, self.pipeline.max_depth_m)
        depth_mask, _ = foreground_mask(
            roi, valid, None, bbox, source='depth_fallback')
        measured_sam = sam_roi & valid
        n_sam = int(sam_roi.sum())
        sam_yield = (
            float(measured_sam.sum() / n_sam) if n_sam > 0 else None)
        k = 2 * (self.dilate_px // 2) + 1
        kernel = self._dilate_kernel_cache.get(k)
        if kernel is None:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            self._dilate_kernel_cache[k] = kernel
        expanded_depth = cv2.dilate(depth_mask.astype(np.uint8), kernel) > 0
        mask = self._enough(measured_sam & expanded_depth)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._last_mask_timings_ms = {mode: elapsed_ms for mode in MODE_IDS}
        return {'hybrid_dilated': mask}, valid, sam_yield, sam_roi

    def _crop_mask(self, mask: Optional[np.ndarray], bbox: tuple,
                   image_shape: tuple) -> Optional[np.ndarray]:
        """
        把全图或 ROI 掩膜裁成与 bbox 同尺寸；尺寸不符返回 None.

        裁剪核单源 geometry.crop_mask_to_bbox（W3；与 pipeline 侧同一原语）。

        Args:
            mask: bool/0-1 掩膜（全图尺寸则裁 ROI）；None 原样返回 None.
            bbox: (x1, y1, x2, y2) 已裁剪到图内的整数框（像素）.
            image_shape: 全图 shape（判全图/ROI 用）.

        Returns
        -------
            (y2-y1, x2-x1) bool 掩膜；尺寸对不上给 None.

        """
        if mask is None:
            return None
        x1, y1, x2, y2 = bbox
        arr = np.asarray(mask, dtype=bool)
        if arr.shape[:2] == image_shape[:2]:
            arr = crop_mask_to_bbox(arr, bbox)
        expected = (y2 - y1, x2 - x1)
        return arr if arr.shape == expected else None

    def _enough(self, mask: np.ndarray) -> Optional[np.ndarray]:
        """
        像素数不足 min_mask_points 时丢弃（触发 mask_unavailable）.

        Args:
            mask: (h, w) bool 掩膜.

        Returns
        -------
            原掩膜或 None.

        """
        return mask if int(mask.sum()) >= self.min_mask_points else None

    @staticmethod
    def _source(mode: str) -> str:
        """
        写入结果的 mask_source 标签（便于诊断追溯）.

        Args:
            mode: 已注册模式 ID（未知 ID 抛 KeyError）.

        Returns
        -------
            来源标签字符串.

        """
        return {
            'hybrid_dilated': 'mobile_sam_dilated_depth_intersection',
        }[mode]

    def _unavailable(self, obs: BagObservation, target_id: str, bbox: tuple,
                     mode: str, reason: str) -> TargetPoseResult:
        """
        构造显式失败结果（REOBSERVE + diagnostic_flags）.

        Args:
            obs: 单帧输入（取版本元数据）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mode: 前景模式 ID（写入 strategy_id）.
            reason: 原因标记（如 'mask_unavailable'）.

        Returns
        -------
            TargetPoseResult（status=REOBSERVE，metrics 为空）.

        """
        x1, y1, x2, y2 = map(int, bbox)
        grasp_2d = BagGrasp2D(
            detection_bbox=(x1, y1, x2 - x1, y2 - y1),
            status='REOBSERVE', diagnostic_flags=[reason])
        grasp_3d = BagGraspReference3D(
            status='REOBSERVE', diagnostic_flags=[reason],
            # strategy_id 与成功路径同名（袋线/果线经管线 kind 区分，不恒为 bag）
            strategy_id=f'robust_{self.pipeline.kind}_pose:{mode}',
            model_version=str(obs.metadata.get('model_version', 'unknown')),
            calibration_version=str(obs.metadata.get(
                'calibration_version', 'unknown')),
            tool_version=self.pipeline.tool.version)
        return TargetPoseResult(target_id, grasp_2d, grasp_3d, mode, {})


def dedup_overlapping_detections(
        detections, ios_threshold: float = 0.6,
        frag_ios_threshold: float = 0.2,
        frag_area_ratio: float = 0.5) -> list:
    """
    重叠检测框去重：IoS（交集/较小框面积）≥ 阈值判同一物理目标，保留大框.

    规则 1（基本包含）：IoS ≥ ios_threshold → 抑制小框（原有）；
    规则 2（碎片残枝，09-01「先做大框」定夺）：IoS ≥ frag_ios_threshold
    且面积比（小/大）≤ frag_area_ratio → 抑制小框——叶片遮挡碎片框 IoS
    达不到包含阈值，但「明显更小+可见重叠」足以判为同一颗的残片；相邻
    两颗袋面积相当（比值≈1）不会被误删。面积并列时保留置信度高者；
    跨类别同样生效——YOLO 按类 NMS，同一颗桃可同时出 bag/nobag 两框，
    都会在身份注册表上重复占号。用 IoS 而非 IoU：部分重叠的相邻两颗桃
    IoS 低不误删。
    贪心顺序为面积降序（置信度次之），后遍历到的高重叠框被抑制。

    Args:
        detections: 检测 dict 列表（须含 'bbox'=(x1,y1,x2,y2)；'conf' 可选）.
        ios_threshold: IoS 阈值；≥1.0 时永不命中，等效关闭去重.

    Returns
    -------
        去重后的检测 dict 列表（按面积降序；元素为原 dict 引用，不改原对象）.

    """
    if not detections or ios_threshold >= 1.0:
        return list(detections)
    boxes = np.asarray([d['bbox'] for d in detections], dtype=float).reshape(-1, 4)
    areas = (np.maximum(0.0, boxes[:, 2] - boxes[:, 0])
             * np.maximum(0.0, boxes[:, 3] - boxes[:, 1]))
    confs = np.array([float(d.get('conf', 0.0)) for d in detections])
    order = sorted(range(len(detections)), key=lambda i: (-areas[i], -confs[i]))
    kept: list = []
    for i in order:
        suppress = False
        for j in kept:
            if areas[i] <= 0.0 or areas[j] <= 0.0:
                continue
            ix1 = max(boxes[i, 0], boxes[j, 0])
            iy1 = max(boxes[i, 1], boxes[j, 1])
            ix2 = min(boxes[i, 2], boxes[j, 2])
            iy2 = min(boxes[i, 3], boxes[j, 3])
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            if inter / min(areas[i], areas[j]) >= ios_threshold:
                suppress = True
                break
            if frag_area_ratio > 0.0:
                small_over_big = min(areas[i], areas[j]) / max(
                    areas[i], areas[j])
                if (small_over_big <= frag_area_ratio
                        and inter / areas[i] >= frag_ios_threshold):
                    suppress = True
                    break
        if not suppress:
            kept.append(i)
    return [detections[i] for i in kept]
