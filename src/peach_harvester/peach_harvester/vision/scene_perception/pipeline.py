"""
感知纯核门面：from_params + process(frame) → PerceptionResult.

节点只做 ROS 解码/发布。袋/果 impl 字典仍在 pose_pipelines.make_pipeline。
"""
from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
from typing import List, Optional, Tuple

import numpy as np
from peach_harvester.vision.common.geometry import (
    crop_mask_to_bbox,
    invert_transform,
)
from peach_harvester.vision.domain.evidence import may_commit_identity
from peach_harvester.vision.scene_perception.contracts import BagObservation
from peach_harvester.vision.scene_perception.debug_draw import draw_debug
from peach_harvester.vision.scene_perception.identity import (
    bbox_touches_image_edge,
    CollectLockPolicy,
    first_point,
    GlobalHarvestPlan,
    SpatialEmaMatcher,
    TargetRegistry,
)
from peach_harvester.vision.scene_perception.image_gates import (
    plan_segmentation_bboxes,
    project_positions_to_pixels,
    valid_depth_mask,
)
from peach_harvester.vision.scene_perception.inference import (
    CandidateEstimator,
    dedup_overlapping_detections,
    InferenceEngine,
    MobileSam,
    UltralyticsYolo,
)
from peach_harvester.vision.scene_perception.msg_builders import (
    to_candidate,
    to_candidate_2d,
    to_fitting,
    to_markers,
)
from peach_harvester.vision.scene_perception.pose_pipelines import (
    apply_transform_to_reference,
    make_pipeline,
)
from peach_harvester.vision.scene_perception.stream_metrics import (
    AdaptiveTimeout,
    LightingMeter,
    RateEstimator,
    TimingMetrics,
)
from peach_interfaces.msg import BagFittingArray, BagGraspCandidateArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

_LOG = logging.getLogger(__name__)


@dataclass
class SyncedRgbd:
    """一帧已解码 RGB-D + TF 查询结果（节点 decode，管线 process）."""

    rgb: np.ndarray
    """BGR uint8 (H, W, 3)，OpenCV 惯例."""
    depth: np.ndarray
    """uint16 毫米，与 RGB 同尺寸对齐."""
    K: dict
    """内参 {'fx','fy','cx','cy','width','height'}，像素."""
    cam_frame: str
    """相机光学系 frame_id."""
    out_frame: str
    """几何输出系；通常 base / output_frame."""
    geometry_stamp: object
    """深度图 stamp，用于精确时刻 TF."""
    img_header: Header
    """彩色图 Header（检测/掩膜话题用）."""
    header: Header
    """输出 Header：stamp=geometry_stamp，frame_id=out_frame."""
    T_out_cam: np.ndarray
    """4×4 齐次：out_frame ← camera；失败时节点给单位阵或 None."""
    tf_status: str
    """'ok' | 'stale' | 'unavailable'。仅 ok 才进身份链."""
    gravity_hint: np.ndarray
    """相机系重力方向 (3,)；tf 模式由 R 反推."""


@dataclass
class PerceptionResult:
    """process() 一拍产物。节点发布；本对象不碰 publisher."""

    kept: list
    """过滤+去重后的检测 dict 列表（bbox/class_id/conf）."""
    mask_canvas: np.ndarray
    """全图 uint16 实例掩膜，像素值=检测序号+1."""
    debug: Optional[np.ndarray]
    """仅已确认目标叠加；None=未开 publish_debug_image."""
    debug_raw: Optional[np.ndarray]
    """全量检测叠加（含未确认灰框）."""
    candidates: BagGraspCandidateArray
    """已确认目标的 3D 抓取候选（/initial_pose）."""
    fittings: BagFittingArray
    """拟合诊断（target_kind、内点率等）."""
    markers: MarkerArray
    """RViz 袋轴/入口标记."""
    confirmed_bboxes: list
    """已确认检测框，用于检测点云."""
    harvest_records: list
    """本帧身份记录，喂 harvest_plan.update."""
    harvest_payloads: dict
    """target_id → {candidate, candidate_2d, fitting, mask, mask_depth_ratio}."""
    frame_axes: List[Tuple[str, Optional[np.ndarray]]]
    """(ACCEPT/REOBSERVE/REJECT, 袋底→袋口方向)；节点取最佳轴发布."""
    mask_header: Header
    """掩膜 stamp=深度时刻，frame_id=相机系."""
    header: Header
    """与 candidates 相同的输出 Header."""
    img_header: Header
    """彩色图 Header."""


def filter_detections(detections, params, enable_fruit: bool = False):
    """
    Apply confidence gate, optional fruit drop, then IoS dedup.

    Args:
        detections: YOLO 输出 dict 列表（须含 conf / class_id / bbox）.
        params: 须有 min_detection_conf、detection_dedup_ios、
            detection_dedup_area_ratio.
        enable_fruit: False 时丢掉 peach_nobag（class_id=1）.

    Returns
    -------
        过滤+去重后的检测 dict 列表.

    """
    kept = [d for d in detections
            if float(d.get('conf', 0.0)) >= params.min_detection_conf]
    if not enable_fruit:
        kept = [d for d in kept if int(d.get('class_id', 0)) != 1]
    return dedup_overlapping_detections(
        kept, params.detection_dedup_ios,
        frag_area_ratio=params.detection_dedup_area_ratio)


def _crop_mask_to_bbox(sam_mask, bbox):
    """全图掩膜 bbox 外清零（裁剪核单源 geometry.crop_mask_to_bbox；W3）."""
    cropped = crop_mask_to_bbox(sam_mask, bbox)
    x1, y1, x2, y2 = (int(v) for v in bbox)
    out = np.zeros_like(sam_mask)
    out[max(y1, 0):max(y2, 0), max(x1, 0):max(x2, 0)] = cropped
    return out


class PerceptionPipeline:
    """检测→分割→袋位姿门面。不持有 Node，不发话题."""

    params: object
    """ScenePerceptionParams 快照（yaml 直读）."""
    clock: object
    """算法时钟适配器（秒，float）；与节点 get_clock 同源."""
    enable_fruit: bool
    """False=不构造裸果线，class_id=1 在 SAM 前丢弃."""
    engine: InferenceEngine
    """YOLO 检测 + MobileSAM 分割."""
    estimator: CandidateEstimator
    """袋/果位姿；fruit_pipeline=None 时裸果不走袋线."""
    target_registry: Optional[TargetRegistry]
    """世界系身份表；target_memory.enable=False 时为 None."""
    harvest_plan: GlobalHarvestPlan
    """收齐窗口与锁定集，不选下一颗."""
    plan_lock: threading.RLock
    """保护 harvest_plan / registry / bbox_at_edge."""
    lighting: LightingMeter
    """锁定后掩膜深度占比与置信度 EMA."""
    bbox_at_edge: dict
    """target_id → 本帧检测框是否贴图像边缘."""
    frame_rate: RateEstimator
    """帧间隔 EMA，用于伸缩收齐超时."""
    collect_window_timeout: AdaptiveTimeout
    """按实测帧间隔伸缩 harvest_plan.max_collect_s."""
    timing: TimingMetrics
    """detect/segment/geometry 毫秒 EMA."""

    def __init__(self, params, clock, logger=None,
                 enable_fruit: bool = False):
        self.params = params
        self.clock = clock
        self._log = logger or _LOG
        self.enable_fruit = bool(enable_fruit)
        self.engine = InferenceEngine(
            detector=UltralyticsYolo(
                yolo_model=params.yolo_model_path,
                yolo_conf=params.yolo_conf,
                yolo_iou=params.yolo_nms_iou,
                yolo_imgsz=int(getattr(params, 'yolo_imgsz', 640)),
                yolo_half=bool(getattr(params, 'yolo_half', False))),
            segmenter=MobileSam(
                sam_model=params.sam_model_path,
                sam_max_bboxes=params.sam_max_bboxes,
                sam_min_area=params.sam_min_area))
        pipe_kw = {
            'min_depth_m': params.pipeline.min_depth_m,
            'max_depth_m': params.pipeline.max_depth_m,
            'min_points': params.pipeline.min_points,
        }
        bag = make_pipeline(
            params.pipeline.bag_impl, tool=params.tool, **pipe_kw)
        fruit = None
        if self.enable_fruit:
            fruit = make_pipeline(
                params.pipeline.fruit_impl, tool=params.tool, **pipe_kw)
        self.estimator = CandidateEstimator(
            pipeline=bag, fruit_pipeline=fruit,
            min_mask_points=params.min_mask_points)
        if params.target_memory.enable:
            matcher = SpatialEmaMatcher(
                match_radius=params.target_memory.match_radius_m,
                recovery_scale=params.target_memory.recovery_scale)
            self.target_registry = TargetRegistry(
                matcher=matcher,
                max_targets=params.target_memory.max_targets,
                position_ema=params.target_memory.position_ema,
                confirm_frames=params.target_memory.confirm_frames,
                tentative_ttl_frames=params.target_memory.tentative_ttl_frames,
                max_age_s=params.target_memory.max_age_s,
                swing_threshold_m=params.wind.swing_threshold_m,
                swing_frames=params.wind.swing_frames)
        else:
            self.target_registry = None
        self.plan_lock = threading.RLock()
        self.harvest_plan = GlobalHarvestPlan(
            max_targets=params.target_memory.max_targets,
            prefer_lower_first=params.harvest.priority_prefer_lower_first,
            anchor_max_age_frames=max(
                1, round(params.target_memory.anchor_max_age_s / 0.2)),
            anchor_drop_frames=max(
                1, round(params.target_memory.anchor_drop_s / 0.2)),
            lock_policy=CollectLockPolicy(
                min_collect_frames=params.harvest.min_collect_frames,
                lock_settle_frames=params.harvest.lock_settle_frames,
                max_collect_s=params.harvest.max_collect_s))
        self.lighting = LightingMeter(
            alpha=0.3,
            min_depth_ratio=params.lighting.min_depth_ratio,
            min_conf_mean=params.lighting.min_conf_mean,
            bad_frames=params.lighting.bad_frames)
        self.bbox_at_edge = {}
        self.frame_rate = RateEstimator(alpha=0.3)
        self.collect_window_timeout = AdaptiveTimeout(
            lower=0.4 * params.harvest.max_collect_s, upper=float('inf'),
            factor=(params.harvest.min_collect_frames
                    + params.harvest.lock_settle_frames + 3))
        self.timing = TimingMetrics(alpha=0.3)

    @classmethod
    def from_params(cls, params, clock, logger=None,
                    enable_fruit: bool = False):
        """Build from a yaml snapshot and clock adapter."""
        return cls(params, clock, logger=logger, enable_fruit=enable_fruit)

    def begin_scene(self, scene_changed: bool) -> int:
        """Reset lock window; clear identity only on physical scene change."""
        self.harvest_plan.reset()
        cleared = 0
        if self.target_registry is not None and scene_changed:
            cleared = self.target_registry.clear()
        self.bbox_at_edge.clear()
        return cleared

    def segmentation_bboxes(self, kept, T_out_cam, K, executor_target_id: str):
        """Choose SAM boxes: locked-set (or selected) when the plan is locked."""
        with self.plan_lock:
            locked = self.harvest_plan.locked
            anchor_px = None
            if (self.params.pipeline.locked_only_segmentation and locked
                    and self.target_registry is not None
                    and T_out_cam is not None):
                positions = {}
                for target_id in self.harvest_plan.locked_ids:
                    if target_id in self.harvest_plan.completed_ids:
                        continue
                    if executor_target_id and target_id != executor_target_id:
                        continue
                    entry = self.target_registry.get(target_id)
                    if entry is not None and entry.get('position') is not None:
                        positions[target_id] = np.asarray(
                            entry['position'], dtype=float)
                # T_camera_base = inv(T_base_camera)：W13-A 走 geometry
                # 单源 invert_transform（数值同 np.linalg.inv，口径有测试锚点）
                anchor_px = project_positions_to_pixels(
                    positions, invert_transform(T_out_cam), K)
            return plan_segmentation_bboxes(
                kept, self.params.pipeline.locked_only_segmentation,
                locked, anchor_px)

    def process(self, frame: SyncedRgbd,
                executor_target_id: str = '') -> Optional[PerceptionResult]:
        """Detect → segment → pose → identity. Returns None if YOLO fails."""
        params = self.params
        rgb, depth, K = frame.rgb, frame.depth, frame.K
        cam_frame, out_frame = frame.cam_frame, frame.out_frame
        header, img_header = frame.header, frame.img_header
        T_out_cam, tf_status = frame.T_out_cam, frame.tf_status
        gravity_hint = frame.gravity_hint

        t_detect = self.clock.now()
        try:
            detections = self.engine.detect(rgb)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'YOLO 检测异常，跳过本帧: {exc}')
            return None
        kept = filter_detections(detections, params, self.enable_fruit)
        self.timing.record('detect_ms', (self.clock.now() - t_detect) * 1e3)

        mask_canvas = np.zeros(depth.shape[:2], dtype=np.uint16)
        debug = rgb.copy() if params.publish_debug_image else None
        debug_raw = rgb.copy() if params.publish_debug_image else None
        cand_arr = BagGraspCandidateArray()
        cand_arr.header = header
        fit_arr = BagFittingArray()
        fit_arr.header = header
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        frame_axes: List[Tuple[str, Optional[np.ndarray]]] = []
        harvest_records = []
        harvest_payloads = {}
        confirmed_bboxes = []
        mask_header = Header()
        mask_header.stamp = frame.geometry_stamp
        mask_header.frame_id = cam_frame

        track = (
            self.target_registry is not None
            and may_commit_identity(tf_status == 'ok'))
        if track:
            with self.plan_lock:
                # W2/V2：注册表淘汰的 id 须同步清出 bbox_at_edge 旁路缓存，
                # 否则键只增不减（长运行缓慢泄漏）。
                for evicted_id in self.target_registry.begin_frame(
                        now=self.clock.now()):
                    self.bbox_at_edge.pop(evicted_id, None)

        t_seg = self.clock.now()
        frame_bboxes = self.segmentation_bboxes(
            kept, T_out_cam, K, executor_target_id)
        mask_by_bbox = {}
        if frame_bboxes:
            try:
                segs = self.engine.segment(rgb, frame_bboxes)
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f'SAM 批量分割异常，回退逐目标调用: {exc}')
                segs = []
                for fallback_bbox in frame_bboxes:
                    try:
                        segs.extend(self.engine.segment(rgb, [fallback_bbox]))
                    except Exception as exc_single:  # noqa: BLE001
                        self._log.warning(f'SAM 单目标分割异常: {exc_single}')
            mask_by_bbox = {bbox: mask for mask, bbox in segs}
        self.timing.record('segment_ms', (self.clock.now() - t_seg) * 1e3)

        t_geom = self.clock.now()
        valid_depth_full = valid_depth_mask(
            depth, params.pipeline.min_depth_m, params.pipeline.max_depth_m)
        pending = []
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = mask_by_bbox.get(bbox)
            if sam_mask is not None:
                sam_mask = _crop_mask_to_bbox(sam_mask, bbox)
                mask_canvas[sam_mask > 0] = np.uint16(i + 1)
            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=gravity_hint, detections=[det],
                metadata={
                    'model_version': params.model_version,
                    'calibration_version': params.calibration_version,
                })
            result = self.estimator.estimate_modes(
                obs, f'target_{i}', bbox, sam_mask)['hybrid_dilated']
            if tf_status != 'ok':
                flag = 'tf_stale' if tf_status == 'stale' else 'tf_unavailable'
                if flag not in result.grasp_3d.diagnostic_flags:
                    result.grasp_3d.diagnostic_flags.append(flag)
            camera_anchor = first_point(
                result.grasp_3d.points_centroid, result.grasp_3d.bag_bottom,
                result.grasp_3d.position, result.grasp_3d.entry_start)
            camera_distance_m = (
                0.0 if camera_anchor is None
                else float(np.linalg.norm(camera_anchor)))
            if T_out_cam is not None and out_frame != cam_frame:
                apply_transform_to_reference(result.grasp_3d, T_out_cam)
            frame_axes.append((result.grasp_3d.status,
                               result.grasp_3d.translation_direction))
            pending.append({
                'i': i, 'det': det, 'bbox': bbox, 'sam_mask': sam_mask,
                'result': result, 'camera_distance_m': camera_distance_m,
                # W3 去重：贴边判定同帧只算一次（at_edge 双消费：
                # 身份分配入参 + bbox_at_edge 旁路缓存）
                'at_edge': bbox_touches_image_edge(
                    bbox, depth.shape[1], depth.shape[0]),
            })

        assigned_ids = [(f'untracked_{p["i"]}', False) for p in pending]
        if self.target_registry is not None and track:
            assign_items = []
            for p in pending:
                g3 = p['result'].grasp_3d
                assign_items.append({
                    'position': first_point(
                        g3.points_centroid, g3.bag_bottom,
                        g3.position, g3.entry_start),
                    'class_id': int(p['det'].get('class_id', 0)),
                    'axis': g3.translation_direction,
                    'diameter': float(g3.bag_diameter_upper_m or 0.0),
                    'status': g3.status,
                    'at_edge': p['at_edge'],
                })
            with self.plan_lock:
                assigned_ids = self.target_registry.match_or_register_frame(
                    assign_items, now=self.clock.now())

        for p, (tid, is_new) in zip(pending, assigned_ids):
            det, bbox, sam_mask = p['det'], p['bbox'], p['sam_mask']
            result = p['result']
            if self.target_registry is not None:
                if track:
                    result.grasp_3d.diagnostic_flags.append(
                        'target_new' if is_new else 'target_matched')
                    if str(tid).startswith('ambiguous_'):
                        result.grasp_3d.diagnostic_flags.append(
                            'target_ambiguous')
                    tracked = self.target_registry.get(tid)
                    if tracked is not None and tracked.get('swinging'):
                        result.grasp_3d.diagnostic_flags.append(
                            'target_swinging')
                    if not str(tid).startswith('ambiguous_'):
                        at_edge = p['at_edge']
                        self.bbox_at_edge[tid] = at_edge
                        if at_edge:
                            result.grasp_3d.diagnostic_flags.append('bbox_edge')
                else:
                    result.grasp_3d.diagnostic_flags.append('target_untracked')
            g3, g2 = result.grasp_3d, result.grasp_2d
            candidate_msg = to_candidate(
                header, tid, g3, model_version=params.model_version,
                calibration_version=params.calibration_version,
                tool_version=params.tool.version)
            candidate_2d_msg = to_candidate_2d(header, tid, g2)
            fitting_msg = to_fitting(header, tid, result)
            registry_item = (
                None if self.target_registry is None
                else self.target_registry.get(tid))
            confirmed = (
                True if self.target_registry is None
                else bool(registry_item and registry_item['confirmed']))
            if str(tid).startswith(('untracked_', 'ambiguous_')):
                confirmed = False
            base_anchor = first_point(
                g3.bag_bottom, g3.points_centroid, g3.position, g3.entry_start)
            record = {
                'target_id': tid, 'status': int(candidate_msg.status),
                'confidence': float(candidate_msg.confidence),
                'camera_distance_m': p['camera_distance_m'],
                'confirmed': confirmed,
                'diagnostic_flags': list(candidate_msg.diagnostic_flags),
            }
            if base_anchor is not None:
                record['base_height_m'] = float(base_anchor[2])
            harvest_records.append(record)
            if confirmed:
                confirmed_bboxes.append(det['bbox'])
                mask_depth_ratio = 0.0
                if sam_mask is not None:
                    fg = np.asarray(sam_mask) > 0
                    n_fg = int(np.count_nonzero(fg))
                    if n_fg > 0:
                        mask_depth_ratio = float(
                            np.count_nonzero(fg & valid_depth_full) / n_fg)
                harvest_payloads[tid] = {
                    'candidate': candidate_msg,
                    'candidate_2d': candidate_2d_msg,
                    'fitting': fitting_msg,
                    'mask': sam_mask,
                    'mask_depth_ratio': mask_depth_ratio,
                }
                cand_arr.candidates.append(candidate_msg)
                fit_arr.fittings.append(fitting_msg)
                markers.markers.extend(to_markers(
                    header, tid, p['i'], result,
                    tool_d_inner=float(params.tool.d_inner_m)))
            if debug_raw is not None:
                draw_debug(debug_raw, det, g2, sam_mask, tid, confirmed=confirmed)
            if debug is not None and confirmed:
                draw_debug(debug, det, g2, sam_mask, tid, confirmed=True)

        self.timing.record('geometry_ms', (self.clock.now() - t_geom) * 1e3)
        return PerceptionResult(
            kept=kept, mask_canvas=mask_canvas, debug=debug, debug_raw=debug_raw,
            candidates=cand_arr, fittings=fit_arr, markers=markers,
            confirmed_bboxes=confirmed_bboxes,
            harvest_records=harvest_records, harvest_payloads=harvest_payloads,
            frame_axes=frame_axes, mask_header=mask_header,
            header=header, img_header=img_header)
