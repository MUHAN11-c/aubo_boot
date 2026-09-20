"""
重建共享状态与方法宿主（W4：三个 Mixin 的方法本体迁入）.

``ReconstructionCore`` 按 §3.3 宿主属性盘点显式持有 FrameStore /
AutoController / Publisher 三面全部状态与方法（数学与分支判据零改动，
仅日志经注入 ``_logger``、N7 帧栈锁内快照、PF-5 心跳点数走增量计数、
N12 ``_locked_*`` 显式初始化）。节点 ``TargetReconstructionNode`` 经
MRO 组合本类；capture/publish 的 Mixin 类保留为过渡薄壳一个提交期。

过渡期已知残留（下一提交收口）：12 个发布器与 ``get_clock()`` 仍在
宿主节点创建/解析（on_configure 之后才可用），Core 方法经 MRO 取到。
"""
from __future__ import annotations

import json
import threading
from typing import Optional, Tuple

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Vector3Stamped
import numpy as np
from peach_harvester.vision.common.runtime import HarvestDataStore
from peach_harvester.vision.domain.model_contract import (
    CAPABILITY_INVALID,
    CAPABILITY_UNKNOWN,
    CAPABILITY_VALID,
)
from peach_harvester.vision.target_reconstruction.capture import (
    classify_skip_reason,
    GATE_ALLOW,
    MaskContext,
    STATE_COLLECTING,
    STATE_IDLE,
)
from peach_harvester.vision.target_reconstruction.integrate import (
    assembly_overlap_metrics,
    LocalTsdf,
    summarize_pairs_mm,
    summarize_view_coverage,
)
from peach_harvester.vision.target_reconstruction.publish import (
    _time_plus,
    build_camera_markers,
    build_mesh_marker,
    build_refined_grasp_markers,
    diagnostics_to_status_msg,
    grasp_decision_to_msg,
    MODEL_VALIDITY_S,
    xyzrgb_to_cloud_msg,
)
from peach_harvester.vision.target_reconstruction.refine import (
    axis_angle_deg,
    candidate_axis_hint,
    RefitResult,
    STATUS_REJECT,
)
from peach_interfaces.msg import (
    BagFitting,
    BagFittingArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
    ShapeHypothesis,
)
from std_msgs.msg import Header, String


class ReconstructionCore:
    """重建共享状态宿主：帧环/掩膜缓存/自动驱动/发布面/产物缓存."""

    def __init__(self, *, collector, mask_gate, kind_memory, params,
                 algo_clock, logger, timing, throttle, icp_target_cache):
        """
        按 §3.3 属性盘点注入组件并显式初始化全部宿主状态（N12）.

        Args:
            collector: FrameCollector（采帧状态机）.
            mask_gate: StrictMaskGate（五道掩膜门）.
            kind_memory: TargetKindMemory（target_id→target_kind）.
            params: TargetReconstructionParams（yaml 直读快照）.
            algo_clock: 纯核 Clock 适配（协议 I3）.
            logger: 宿主 logger（node.get_logger()）.
            timing: TimingStats（耗时累计器）.
            throttle: PublishThrottle（E4 发布节流）.
            icp_target_cache: IcpTargetCache（E4 ICP target 复用）.

        Returns
        -------
            无返回值（None）.

        """
        # —— 注入（§3.3）——
        self.collector = collector
        self._mask_gate = mask_gate
        self._target_kind_memory = kind_memory
        self.params = params
        self._algo_clock = algo_clock
        self._logger = logger
        self._timing = timing
        self._publish_throttle = throttle
        self._icp_target_cache = icp_target_cache
        # —— 并发收敛（注释随迁自节点：worker 与 executor 双写收敛）——
        # 方案 b：collector/在线 TSDF/派生产物的竞态源是 worker 线程
        # （_process_rgbd→_auto_drive）与 executor 线程（订阅/服务回调）
        # 双写；节点级 RLock 使服务/订阅/自动驱动入口互斥，帧栈、TSDF
        # 积分与全部产物读写均在锁内（RLock 允许 _auto_drive→_finalize_now
        # 等锁内嵌套调用）。唯一锁外段：采帧门禁的阻塞式精确时刻 TF 查询
        # （节点 _gated_capture_* 三段式）。_latest_frame/_latest_candidates/
        # _max_joint_vel 等单字段原子赋值不持锁（CPython 引用赋值原子）。
        self._state_lock = threading.RLock()
        self._lifecycle_active = False
        # —— FrameStore 状态 ——
        self._frame_ring_max = 5
        # 同步 RGB-D 帧环：按 stamp_ns 保留最近若干帧，供掩膜滞后对齐。
        # 元组 (rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame)。
        # 只缓存、不直接累积；手动/自动门禁通过后才会入帧栈。
        # _latest_frame 仍指向环内最新一帧（兼容只读侧）。
        self._frame_ring: dict = {}
        self._latest_frame: Optional[tuple] = None
        self._last_captured_stamp_sec = -1.0  # [s] 上次成功采帧的图像时间戳
        self._latest_candidates: Optional[BagGraspCandidateArray] = None
        self._preferred_target_id = ''
        self._target_masks = {}
        # 锁定集目标锚点缓存 {target_id: (3,) base 系中心 [m]}：E2 邻目标
        # 串扰门数据源（每条 target_observations 全量重建，未锁定恒空）
        self._locked_target_centers = {}
        self._locked_target_areas = {}
        self._harvest_data = HarvestDataStore()
        self._last_tf_latency_ms: Optional[float] = None
        # —— 产物缓存（_reset_products/_refresh_tsdf_outputs 本体同在此类）——
        # 产物版本号（E4 发布节流的 on-change 判据）：_tsdf_cloud_version
        # 随 _tsdf_cloud_cache 每次写入递增；_products_version 随任一
        # 云/Marker 产物（含 refined/mesh）写入递增；清空类事件另置
        # _products_force_publish 强制下一轮 _publish_all 立即透传
        self._tsdf_cloud_version = 0
        self._products_version = 0
        self._products_force_publish = False
        # finalize 时计算的 overlap 指标缓存（帧栈变动即失效置 None）
        self._overlap_cache: Optional[dict] = None
        # TSDF 云缓存：(xyz, colors_bgr) 或 None；finalize 时重建，帧栈变动失效
        self._tsdf_cloud_cache: Optional[tuple] = None
        self._tsdf_info: Optional[dict] = None  # diagnostics 的 tsdf 键内容
        self._tsdf_volume = None  # 每轮 session 持续在线积分
        self._mesh_cache: Optional[dict] = None
        # refit：感知 diagnostics 的 target_id→target_kind 映射；
        # _refined 为 refit 唯一缓存——refit 成功结果 RefitResult 或
        # ok=False 失败记录（None=未跑/已失效），diagnostics JSON 的
        # refined 键由 _refined_info() 投影派生
        self._refined: Optional[RefitResult] = None
        self._bag_model = None
        self._pregrasp_prev: Optional[dict] = None
        # 球体 refit 无法独立恢复姿态轴；绑定时冻结感知侧果梗/凹陷方向先验。
        self._bound_axis_hint = None
        self._harvest_run_id = ''
        self._scene_epoch = 0
        # —— N12：_lock_decision_validity 三属性显式初始化（旧版靠
        # getattr 默认值隐式存在，宿主 __init__ 未声明）——
        self._locked_model_revision: Optional[str] = None
        self._locked_generated_at = None
        self._locked_valid_until = None
        # —— 派生（与节点同源 params；Core 自持不借宿主）——
        self._local_volume = (
            params.local_volume.size_x, params.local_volume.size_y,
            params.local_volume.size_z)

    # ------------------------------------------------------------------
    # 体积与产物缓存（自节点迁入；调用方须持 _state_lock）
    # ------------------------------------------------------------------
    def _create_volume(self):
        """建一个空融合体积（LocalTsdf 唯一实现；I3 注入时钟）."""
        p = self.params
        return LocalTsdf(
            now=self._algo_clock.now,
            voxel_length=p.tsdf.voxel_length,
            sdf_trunc=p.tsdf.sdf_trunc,
            depth_trunc=p.tsdf.depth_trunc)

    def _bump_products_version(self, tsdf_cloud: bool = False) -> None:
        """
        产物版本号递增（E4 发布节流的 on-change 判据；须持 _state_lock）.

        Args:
            tsdf_cloud: True 表示 _tsdf_cloud_cache 也被写入（同步递增
                tsdf_cloud 版本）.

        Returns
        -------
            无返回值（None）.

        """
        self._products_version += 1
        if tsdf_cloud:
            self._tsdf_cloud_version += 1

    def _reset_products(self, create_volume: bool) -> None:
        """清空本轮派生结果；开始新轮时同时创建一个空在线 TSDF."""
        # 预抓取验证的一致性比对基点一并清空：换目标后首拍不得与上一
        # 目标（甚至上一轮）的残差比 frames_consistent。
        self._pregrasp_prev = None
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        self._mesh_cache = None
        self._refined = None
        # 与 _refined 成对清空：换目标后 _bag_model 若残留旧目标融合结果，
        # PregraspVerification/TargetModel 会拿上一颗的袋模型报残差。
        self._bag_model = None
        self._tsdf_volume = None
        # E4：模型已清空，ICP target 缓存作废（下次采帧强制全量刷新）；
        # 版本号递增 + force 标志使下一轮 _publish_all 立即透传空产物
        # （RViz 同步刷新不被 on-change/间隔门抑制）
        self._icp_target_cache.invalidate()
        self._bump_products_version(tsdf_cloud=True)
        self._products_force_publish = True
        if create_volume and self.params.tsdf.enable:
            self._tsdf_volume = self._create_volume()

    def _roi_center(self):
        """返回局部体素盒中心；候选缺失时退到首帧局部云质心."""
        center = self.collector.target_center
        if center is None and self.collector.frames:
            first = self.collector.frames[0].cloud_base
            if first is not None and len(first):
                center = np.asarray(first).mean(axis=0)
        return center

    def _refresh_tsdf_outputs(self, extract_mesh: bool = False) -> None:
        """
        从当前在线体积刷新局部点云；finalize 时额外提取网格.

        E4：本方法是 TSDF 全量 extract 的唯一入口（采帧路径受
        IcpTargetCache 节流，每 k 帧或关键事件才调用）；每次调用同步
        重置 ICP target 复用基线并递增 tsdf_cloud 产物版本号。
        """
        if self._tsdf_volume is None:
            self._tsdf_cloud_cache = None
            self._mesh_cache = None
            # 体积不存在：产物清空 + target 缓存作废（防御性路径，
            # 正常流程采帧/ finalize 前体积必已创建）
            self._icp_target_cache.invalidate()
            self._bump_products_version(tsdf_cloud=True)
            return
        xyz, colors = self._tsdf_volume.extract_cloud()
        center = self._roi_center()
        if center is not None and xyz.size:
            xyz, colors = LocalTsdf.crop_to_box(
                xyz, colors, center, self._local_volume)
        if self.params.cloud_filter.voxel_size > 0.0:
            xyz, colors = LocalTsdf.voxel_downsample(
                xyz, colors, self.params.cloud_filter.voxel_size)
        if self.params.cloud_filter.enable_statistical_filter:
            xyz, colors = LocalTsdf.statistical_filter(xyz, colors)
        self._tsdf_cloud_cache = (xyz, colors)
        # E4：全量提取结果即 ICP target 复用基线（帧到模型语义不变，
        # 仅 extract 频率从逐帧降为每 k 帧）
        self._icp_target_cache.set_full(xyz)
        self._bump_products_version(tsdf_cloud=True)
        if extract_mesh:
            self._mesh_cache = self._tsdf_volume.extract_mesh(
                center=center, size_xyz=self._local_volume)
        self._tsdf_info = {
            'points': int(xyz.shape[0]),
            'integrated_frames': len(self.collector.frames),
            'integrate_time_s': float(self._tsdf_volume.integrate_time_s),
            'voxel_length': self.params.tsdf.voxel_length,
            'sdf_trunc': self.params.tsdf.sdf_trunc,
            'mesh_vertices': int(
                0 if self._mesh_cache is None
                else len(self._mesh_cache['vertices'])),
            'roi_center': (None if center is None
                           else [float(v) for v in center]),
        }

    # ------------------------------------------------------------------
    # FrameStore（自 capture.FrameStoreMixin 迁入，语义零改动）
    # ------------------------------------------------------------------
    def _stamp_ns(self, stamp_msg) -> int:
        """ROS Time → 纳秒整数（与掩膜缓存键一致）."""
        return int(stamp_msg.sec) * 1000000000 + int(stamp_msg.nanosec)

    def _clear_frame_ring(self) -> None:
        """换绑/reset 时丢弃未对齐的陈帧，避免旧目标点云混入."""
        self._frame_ring = {}
        self._latest_frame = None

    def _push_frame_ring(self, frame_tuple) -> None:
        """
        按 stamp_ns 写入帧环，超出容量丢最旧.

        须持 ``_state_lock``：读者（``_select_cached_frame``，持锁）
        对环做 ``list(...)`` 快照迭代，无锁并发插入会触发
        ``RuntimeError: dictionary changed size during iteration``。
        """
        stamp_msg = frame_tuple[3]
        stamp_ns = self._stamp_ns(stamp_msg)
        with self._state_lock:
            self._frame_ring.pop(stamp_ns, None)
            self._frame_ring[stamp_ns] = frame_tuple
            while len(self._frame_ring) > self._frame_ring_max:
                self._frame_ring.pop(next(iter(self._frame_ring)))
        self._latest_frame = frame_tuple

    def _select_cached_frame(
            self, prefer_stamp_sec=None, prefer_cam_frame=None):
        """
        取采帧缓存：优先指定 stamp；否则取「有同戳掩膜的最新帧」.

        严格同戳语义不变：不回退 latest TF。掩膜尚未到达的最新帧留在环
        内，等感知回调再驱动。环空返回 None。
        """
        if prefer_stamp_sec is not None:
            # 持锁遍历：_push_frame_ring 与本读同锁（写侧注释见上）
            for frame in list(self._frame_ring.values()):
                if abs(float(frame[4]) - float(prefer_stamp_sec)) > 1e-9:
                    continue
                if (prefer_cam_frame is not None
                        and (frame[5] or '') != prefer_cam_frame):
                    continue
                return frame
            return None
        for stamp_ns in reversed(list(self._frame_ring.keys())):
            if stamp_ns in self._target_masks:
                return self._frame_ring[stamp_ns]
        if self._frame_ring:
            return next(reversed(list(self._frame_ring.values())))
        return self._latest_frame

    @staticmethod
    def _candidate_center(candidate):
        """候选几何袋底/袋颈中点（base 系 [m]）；非有限或全零时返回 None."""
        bottom = np.array([
            candidate.bag_bottom.x,
            candidate.bag_bottom.y,
            candidate.bag_bottom.z], dtype=np.float64)
        neck = np.array([
            candidate.bag_neck.x,
            candidate.bag_neck.y,
            candidate.bag_neck.z], dtype=np.float64)
        center = 0.5 * (bottom + neck)
        if not np.all(np.isfinite(center)) or not np.any(center):
            return None
        return center

    def _target_mask_for_frame(self, stamp_msg, depth_mm):
        """取严格同时间戳掩膜并过五道质量门（判定本体在 MaskGate 实现）."""
        stamp_ns = self._stamp_ns(stamp_msg)
        # 邻目标锚点=锁定集锚点缓存剔除绑定目标自身（E2 串扰门输入）；
        # 框面积并行携带，供串扰门按面积比豁免小框邻居
        neighbors = tuple(
            (c, self._locked_target_areas.get(tid, 0.0))
            for tid, c in self._locked_target_centers.items()
            if tid != self._preferred_target_id)
        centers = tuple(c for c, _ in neighbors)
        areas = tuple(a for _, a in neighbors)
        result = self._mask_gate.check(MaskContext(
            stamp_ns=stamp_ns,
            depth_mm=depth_mm,
            masks=self._target_masks,
            bound_center=self.collector.target_center,
            neighbor_centers=centers,
            bound_area=self._locked_target_areas.get(
                self._preferred_target_id, 0.0),
            neighbor_areas=areas))
        return result.mask, result.reason

    # ------------------------------------------------------------------
    # AutoController（自 capture.AutoControllerMixin 迁入，语义零改动；
    # 三段式门禁/构云/提交仍在节点——锁协议函数原位保留）
    # ------------------------------------------------------------------
    def _auto_drive(self):
        """
        自动模式驱动：每个新同步帧回调末尾调用一次.

        流程：IDLE 且有候选 → 自动开始；COLLECTING → 满 max_views 自动
        finalize，否则尝试自动采帧；READY/FAILED 停采，等 reset/start 进
        下一轮。所有"不行"都只对当前帧跳过/告警，不打断流程。
        并发收敛：本方法可由 worker 线程（_process_rgbd）与 executor 线程
        （_on_target_observations）并发进入；collector/TSDF/产物读写全程
        持 _state_lock（RLock 允许锁内嵌套调 _finalize_now）。唯一例外是
        采帧的阻塞式 TF 查询：锁内 begin 采集判据 → 锁外查询（最长
        tf_timeout，期间服务/心跳可取锁）→ 锁内 finish 按 stamp 复核收口
        （见节点 _gated_capture_finish 竞态说明）。
        """
        with self._state_lock:
            if self.collector.state == STATE_IDLE and \
                    self.collector.should_auto_start():
                self._auto_start()
            if self.collector.state != STATE_COLLECTING:
                return
            if self.collector.should_auto_finalize():
                ok, message = self._finalize_now()
                if ok:
                    self._logger.info(f'自动完成：{message}')
                return
            if len(self.collector.frames) >= self.params.capture.max_views:
                # 满栈后静默等待 finalize，避免每帧重复构云/ICP和刷屏
                return
            decision, tf_request = self._gated_capture_begin(automatic=True)
            if tf_request is None:
                # 前置门禁已定案（skip），无需 TF 查询。落账须持锁
                # （W2/R1）：collector.note_skip/rejected_views 与其他锁内
                # 写者互斥，锁外调用曾与锁内路径锁契约不一致。
                if decision.reason:
                    self._record_auto_skip(
                        decision.reason,
                        count_reject=decision.count_reject,
                        count_tf_failure=decision.count_tf_failure)
                return
        # 锁外：阻塞式 TF 查询（不得持 _state_lock）。
        tf_result = self._gated_capture_query_tf(tf_request)
        with self._state_lock:
            decision, context = self._gated_capture_finish(
                automatic=True, tf_request=tf_request, tf_result=tf_result)
            bound_target = self.collector.target_id
        # 锁域自管：构云/ICP 重活在锁外（见 _auto_capture_commit）。旧版
        # 全程持 _state_lock，心跳/观测回调被饿死（2026-09-09 真机：
        # 心跳违约 57 次、绑定延迟 73s > build_start_timeout 12s）。
        self._auto_capture_commit(decision, context, bound_target)

    def _auto_start(self):
        """自动开始：绑定当前最优候选进 COLLECTING；无候选则保持 IDLE 等待."""
        target_id, center = self._best_candidate()
        if not target_id:
            self._logger.debug(
                '自动开始：无 initial_pose 候选，保持 IDLE')
            return
        message = self.collector.start(target_id, center)
        self._target_kind_memory.bind(target_id)
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=True)
        self._bound_axis_hint = candidate_axis_hint(
            self._latest_candidates, target_id)
        self._logger.info(f'自动开始：{message}')
        self._publish_all()

    def _auto_capture_commit(self, decision, context, bound_target):
        """
        自动采帧落地段（锁域自管）：锁内判门取上下文，锁外构云/ICP，锁内窄临界提交.

        decision 非 GATE_ALLOW 即按 skip 跳过（按需计 tf_failures）；
        context 为 ALLOW 时的帧上下文（见节点 _gated_capture_finish）。
        bound_target 是 finish 锁内快照的绑定目标，提交前复核会话未变。

        并发说明（2026-09-09 锁手术）：构云+ICP 是每帧最重的纯计算，
        持 _state_lock 执行会把心跳/观测回调饿到分钟级滞后；现移到锁外，
        提交段仅剩 add_frame/TSDF 积分/计数（毫秒级）。准备期间会话被
        reset/切换/收口时按 target/state 复核丢弃陈旧帧，与 finish 的
        stamp 复核构成同一竞态窗口的两道闸。
        """
        if decision.action != GATE_ALLOW:
            with self._state_lock:
                if decision.reason:
                    self._record_auto_skip(
                        decision.reason,
                        count_reject=decision.count_reject,
                        count_tf_failure=decision.count_tf_failure)
                elif decision.count_tf_failure:
                    self.collector.tf_failures += 1
            return
        t_frame0 = self._algo_clock.now()
        with self._state_lock:
            if (self.collector.state != STATE_COLLECTING
                    or self.collector.target_id != bound_target):
                return
            (rgb, depth_mm, K, stamp_sec,
             T_base_camera, tf_status, target_mask) = context
            if self._last_captured_stamp_sec > 0.0:
                since_last = stamp_sec - self._last_captured_stamp_sec
            else:
                since_last = float('inf')  # 首帧不受间隔门限制
            action, reason = self.collector.auto_capture_decision(
                T_base_camera, since_last)
            if action != 'capture':
                self._record_auto_skip(reason)
                return
        prepared, reject = self._prepare_frame(context, bound_target)
        if prepared is None:
            # 构云/ICP 拒帧：与原 _accept_frame 失败路径同语义
            with self._state_lock:
                self.collector.rejected_views += 1
                self._record_auto_skip(reject, count_reject=False)
            self._logger.warning(f'自动采帧未入库：{reject}',
                                 throttle_duration_sec=1000.0)
            return
        with self._state_lock:
            if (self.collector.state != STATE_COLLECTING
                    or self.collector.target_id != bound_target):
                self._logger.debug(
                    '采帧准备期间会话已切换/收口，丢弃陈旧帧 '
                    f'{bound_target or "（空）"}')
                return
            accepted, message = self._commit_prepared_frame(prepared, t_frame0)
            if not accepted:
                self.collector.rejected_views += 1
                self._record_auto_skip(message, count_reject=False)
                self._logger.warning(f'自动采帧未入库：{message}',
                                     throttle_duration_sec=1000.0)
                return
        # 累加云/Marker 组装与序列化是重活，移出锁外（E4 节流仍在）
        self._publish_all()

    def _record_auto_skip(
            self, reason: str, *, count_reject: bool = False,
            count_tf_failure: bool = False) -> None:
        """自动 skip 落账：原因码计数 + 节流 WARN + harvest_data 事件."""
        code = classify_skip_reason(reason)
        self.collector.note_skip(code, reason)
        if count_tf_failure:
            self.collector.tf_failures += 1
        if count_reject:
            self.collector.rejected_views += 1
        self._logger.warning(
            f'自动采帧跳过 [{code}]：{reason}',
            throttle_duration_sec=1.0)
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_skipped',
            'target_id': self.collector.target_id,
            'code': code, 'reason': reason,
        })

    # ------------------------------------------------------------------
    # finalize 编排（W4 自节点迁入；全部为 Core 持有的共享状态操作）
    # ------------------------------------------------------------------
    def _finalize_now(self) -> Tuple[bool, str]:
        """
        共享 finalize：状态迁移、重叠指标、最终网格与几何精化.

        Returns
        -------
            (ok, message)；ok=False 时保持 COLLECTING.

        """
        # finalize 总耗时起点（含重叠指标、TSDF 最终提取、refit 全链；
        # 成功/失败路径都记 last 值）
        t_finalize0 = self._algo_clock.now()
        coverage = summarize_view_coverage(
            self.collector.frames, self.collector.target_center)
        pose_count = int(coverage.get('view_count') or 0)
        min_views = int(self.params.capture.min_views)
        if pose_count < min_views:
            message = (
                f'已采 {pose_count} 机位 < min_views={min_views}，'
                '继续采帧或 reset')
            self._logger.warning(message)
            self._timing.record_finalize(
                (self._algo_clock.now() - t_finalize0) * 1000.0)
            self._publish_all()
            return False, message
        ok, message, _cloud = self.collector.finalize()
        if not ok:
            self._overlap_cache = None
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self._refined = None
            self._bag_model = None
            self._bump_products_version(tsdf_cloud=True)
            self._logger.warning(message)
        else:
            self._overlap_cache = assembly_overlap_metrics(self.collector.frames)
            summary = summarize_pairs_mm(self._overlap_cache['pairs'])
            if summary is None:
                message += '；重叠指标需 ≥2 帧，本批次不可用'
            else:
                message += (f'；重叠 mean={summary["mean_mm"]:.1f}mm '
                            f'p95={summary["p95_mm"]:.1f}mm')
            product_ok = True
            if self.params.tsdf.enable:
                message += self._run_tsdf()
                product_ok = (
                    self._tsdf_cloud_cache is not None
                    and self._tsdf_cloud_cache[0].size)
                if not product_ok:
                    message += '；TSDF 产物为空'
                elif self.params.refit.enable:
                    message += self._run_refit(
                        keep_last_good=False, mark_final=True)
                    if not (self._refined and self._refined.ok):
                        product_ok = False
                        message += '；refit 未产出可用几何'
            if product_ok:
                self._logger.info(message)
                self._harvest_data.append_event({
                    'source': 'reconstruction',
                    'event': 'reconstruction_finalized',
                    'target_id': self.collector.target_id,
                    'captured_views': len(self.collector.frames),
                    'pose_count': pose_count,
                    'refined': self._refined_info(),
                    'grasp_decision': self._grasp_decision(),
                })
            else:
                # 已提取的 TSDF 留给 RViz；状态退回 COLLECTING，Build 失败
                ok = False
                self.collector.state = STATE_COLLECTING
                self._logger.warning(message)
        self._timing.record_finalize(
            (self._algo_clock.now() - t_finalize0) * 1000.0)
        self._publish_all()
        return ok, message

    def _run_tsdf(self) -> str:
        """
        从在线 TSDF 提取最终点云和三角网格.

        每帧已在 _commit_prepared_frame 中完成积分；此处禁止再次批量积分，只做
        ROI 点云后处理与 Open3D marching-cubes 网格提取.

        Returns
        -------
            追加到 finalize message 的片段（如 '；TSDF 123456 点'）.

        """
        try:
            self._refresh_tsdf_outputs(extract_mesh=True)
            xyz = self._tsdf_cloud_cache[0]
            mesh_vertices = (
                0 if self._mesh_cache is None
                else len(self._mesh_cache['vertices']))
        except Exception as exc:  # noqa: BLE001
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self._mesh_cache = None
            # E4：提取失败清空产物，版本号递增保证闩锁话题覆盖旧内容
            self._bump_products_version(tsdf_cloud=True)
            self._logger.error(f'TSDF 最终提取失败: {exc}')
            return f'；TSDF 提取失败（{exc}）'
        return (f'；TSDF {xyz.shape[0]} 点'
                f' / mesh {mesh_vertices} 顶点'
                f'（累计积分 {self._tsdf_volume.integrate_time_s:.2f}s）')

    # ------------------------------------------------------------------
    # Publisher（自 publish.PublisherMixin 迁入；N7 帧栈快照/PF-5 点数）
    # ------------------------------------------------------------------
    def _publish_heartbeat(self):
        """1Hz 活性心跳：状态 + 诊断 + 抓取许可（轻量三件套，不含云/Marker）."""
        if not self._lifecycle_active:
            return
        with self._state_lock:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.params.frames.base_frame
            self._publish_status_trio(header)

    def _publish_status_trio(self, header: Header):
        """
        状态名 + 类型化诊断 + 调试 JSON + 抓取许可统一重发（心跳/状态变化共用）.

        结构化核心走 ReconstructionStatus（/diagnostics），完整明细走
        String JSON（/diagnostics_debug），许可走 GraspDecision——三者同
        transient_local 闩锁，后启动订阅者读到的始终是最新一轮。
        """
        diag = self._diagnostics()
        self.pub_status.publish(String(data=self.collector.state))
        self.pub_diag.publish(diagnostics_to_status_msg(diag, header))
        self.pub_diag_debug.publish(
            String(data=json.dumps(diag, ensure_ascii=False)))
        self.pub_grasp_decision.publish(
            grasp_decision_to_msg(
                self._lock_decision_validity(self._grasp_decision()), header))
        if getattr(self, 'pub_pregrasp', None) is not None:
            self.pub_pregrasp.publish(self._pregrasp_verification_msg(header))

    def _lock_decision_validity(self, decision: dict) -> dict:
        """心跳不得续签 valid_until：同一 model_revision 沿用首次冻结时刻."""
        revision = str(decision.get('model_revision') or '')
        locked = self._locked_model_revision
        if locked != revision or self._locked_valid_until is None:
            now = self.get_clock().now().to_msg()
            self._locked_model_revision = revision
            self._locked_generated_at = now
            self._locked_valid_until = _time_plus(now, MODEL_VALIDITY_S)
        decision['generated_at'] = self._locked_generated_at
        decision['valid_until'] = self._locked_valid_until
        return decision

    def _publish_all(self):
        """
        状态变化后统一重发：累加云 + 状态三件套 + 相机轨迹 Marker.

        E4 发布节流（publish.on_change_only / publish.min_interval_s）：
        local_cloud/tsdf_cloud/markers 三类大消息仅内容版本变化且距上次
        实际发布超过最小间隔才真正组装发布（on-change key 用帧数/末帧
        时间戳/产物版本号等廉价标量，不做内容哈希）；零变化或间隔内抑制
        ——三话题均为 transient_local 闩锁，订阅者/RViz 保留最后一帧不丢
        显示，被抑制的变化留待下次调用补发最新版本。产物清空事件
        （_reset_products 置 force 标志）绕过间隔门立即透传，保证绑定
        切换/reset 时 RViz 同步清屏。心跳/状态/诊断/refit 三件套不节流
        （就绪门/新鲜度门载体与闩锁覆盖防陈旧语义不动）。
        N7：本方法可锁外可达（COMMIT 补发路径），帧栈读取改锁内
        tuple(frames) 快照（RLock：锁内调用方重入安全）。
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.params.frames.base_frame
        # on_change_only=false 回退逐次全发旧行为（不查节流器）
        throttle = (self._publish_throttle
                    if self.params.publish.on_change_only else None)
        force = self._products_force_publish
        self._products_force_publish = False
        # N7：锁内帧栈快照（本方法多在锁内被调，RLock 重入零开销）
        with self._state_lock:
            frames = tuple(self.collector.frames)
        n_frames = len(frames)
        # 累加云内容随帧栈增删变化；同长度同末帧戳即同内容（reset 后帧栈
        # 为空 n=0 自然判变；remove_last 改变 n_frames）
        last_stamp = float(frames[-1].stamp) if frames else -1.0
        if throttle is None or throttle.should_publish(
                'local_cloud', (n_frames, last_stamp), force=force):
            cloud = self.collector.accumulated_cloud()
            self.pub_cloud.publish(xyzrgb_to_cloud_msg(
                cloud, self.collector.accumulated_rgb(), header))
        # TSDF 云仅 finalize 后非空；无缓存发空云（字段布局保持一致）。
        # 内容版本 = _tsdf_cloud_version（每次全量 extract/清空递增）
        if throttle is None or throttle.should_publish(
                'tsdf_cloud', (self._tsdf_cloud_version,), force=force):
            if self._tsdf_cloud_cache is not None:
                tsdf_xyz, tsdf_rgb = self._tsdf_cloud_cache
            else:
                tsdf_xyz, tsdf_rgb = np.zeros((0, 3)), None
            self.pub_tsdf_cloud.publish(
                xyzrgb_to_cloud_msg(tsdf_xyz, tsdf_rgb, header))
        self._publish_status_trio(header)
        # Marker 内容 = 相机轨迹（帧栈）+ refined 抓取示意 + mesh
        # _products_version 覆盖（refit 写入/finalize 清理均递增）
        if throttle is None or throttle.should_publish(
                'markers', (n_frames, last_stamp, self._products_version),
                force=force):
            markers = build_camera_markers(header, frames)
            markers.markers.extend(build_refined_grasp_markers(
                header, self._refined, self.collector.target_id or '',
                tool_d_inner=float(self.params.tool.budget.d_inner)))
            mesh_marker = build_mesh_marker(header, self._mesh_cache)
            if mesh_marker is not None:
                markers.markers.append(mesh_marker)
            self.pub_markers.publish(markers)
        # refit 三件套：闩锁话题每次重发（无结果发空消息，防陈旧数据）
        pose_arr, axis_msg, fit_arr = self._refined_messages(header)
        self.pub_refined_pose.publish(pose_arr)
        self.pub_refined_axis.publish(axis_msg)
        self.pub_refined_diag.publish(fit_arr)
        self.pub_shape.publish(self._shape_hypothesis_msg(header))

    def _shape_hypothesis_msg(self, header: Header) -> ShapeHypothesis:
        """由 refit 缓存组形状假设（几何+协方差占位，不含工具抓取量）."""
        msg = ShapeHypothesis()
        msg.header = header
        msg.target_id = self.collector.target_id or ''
        result = self._refined
        if result is None or not result.ok:
            return msg
        bottom = result.bottom
        neck = result.neck
        center = 0.5 * (bottom + neck)
        msg.center = Point(
            x=float(center[0]), y=float(center[1]), z=float(center[2]))
        msg.axis = Vector3(
            x=float(result.axis[0]),
            y=float(result.axis[1]),
            z=float(result.axis[2]))
        msg.diameter_m = float(result.diameter)
        msg.length_m = float(result.span_m)
        msg.confidence = float(result.inlier_ratio)
        msg.model_kind = str(result.kind)
        return msg

    def _refined_messages(self, header: Header):
        """
        由 refit 缓存组 refined 三话题消息（闩锁重发/清空共用）.

        无结果（未跑 finalize/refit 关闭）→ 全空消息；拟合成功 →
        pose/axis/diagnostics 按结果填充（status=ACCEPT/REOBSERVE 照常
        发布）；拟合失败（REJECT）→ pose 发空数组、axis 发零向量（无效
        占位），diagnostics 发 status=REJECT 单条记录（标量 -1）——闩锁
        话题必须发消息覆盖，防后启动订阅者读到上一轮陈旧结果。

        Args:
            header: 输出头（frame_id=base_frame）.

        Returns
        -------
            (BagGraspCandidateArray, Vector3Stamped, BagFittingArray).

        """
        pose_arr = BagGraspCandidateArray()
        pose_arr.header = header
        axis_msg = Vector3Stamped()
        axis_msg.header = header
        fit_arr = BagFittingArray()
        fit_arr.header = header
        result = self._refined
        if result is not None and result.ok:
            cand = BagGraspCandidate()
            cand.header = header
            cand.target_id = self.collector.target_id
            # entry = bottom − axis×standoff（几何在融合线算好）；
            # 姿态未用（接近方向由 translation_direction 给出），置单位四元数
            cand.entry_pose = Pose(
                position=Point(x=float(result.entry[0]),
                               y=float(result.entry[1]),
                               z=float(result.entry[2])),
                orientation=Quaternion(w=1.0))
            cand.bag_bottom = Point(x=float(result.bottom[0]),
                                    y=float(result.bottom[1]),
                                    z=float(result.bottom[2]))
            cand.bag_neck = Point(x=float(result.neck[0]),
                                  y=float(result.neck[1]),
                                  z=float(result.neck[2]))
            # 剪切行进方向 = refined 轴（bottom→neck 单位向量）
            cand.translation_direction = Vector3(x=float(result.axis[0]),
                                                 y=float(result.axis[1]),
                                                 z=float(result.axis[2]))
            # 圆柱为袋径、球为果径（均 = 2r）
            cand.bag_diameter_upper_m = float(result.diameter)
            # 套入行程 = 入口沿轴到剪切参考；执行端优先用本字段。
            cand.suggested_travel_m = float(result.cut_travel_m or 0.0)
            cand.confidence = float(result.inlier_ratio)
            cand.status = int(result.status)
            cand.diagnostic_flags = list(result.flags)
            cand.strategy_id = f'reconstruction_refit_{result.kind}'
            pose_arr.candidates.append(cand)
            axis_msg.vector = Vector3(x=float(result.axis[0]),
                                      y=float(result.axis[1]),
                                      z=float(result.axis[2]))
        fit = self._refined_fitting_msg(header, result)
        if fit is not None:
            fit_arr.fittings.append(fit)
        return pose_arr, axis_msg, fit_arr

    def _refined_fitting_msg(self, header: Header,
                             result: Optional[RefitResult]
                             ) -> Optional[BagFitting]:
        """
        由 refit 结果组 BagFitting（无效标量 -1，语义对齐感知包 _to_fitting）.

        Args:
            header: 输出头.
            result: refit 唯一缓存 _refined（成功结果或 ok=False 失败
                记录）；None 表示未跑（失败时由 _refined_info() 取原因，
                发 REJECT 记录）.

        Returns
        -------
            peach_interfaces/BagFitting；从未跑过 refit 给 None.

        """
        info = self._refined_info()
        if result is None and not info:
            return None
        m = BagFitting()
        m.header = header
        m.target_id = self.collector.target_id
        m.axis_source = 'reconstruction_refit'
        # 全部标量先置 -1（无效约定），再按拟合线逐项覆盖有效字段
        for attr in ('axis_confidence', 'axis_disagreement_deg',
                     'theta_err_deg', 'error_budget_mm',
                     'radial_clearance_mm', 'valid_depth_ratio',
                     'foreground_ratio', 'boundary_touch_ratio',
                     'bag_length_m', 'bag_diameter_upper_m', 'travel_m',
                     'cylinder_rms_m', 'cylinder_inlier_ratio',
                     'fruit_radius_m', 'sphere_rms_m',
                     'sphere_inlier_ratio', 'cavity_dip_mm'):
            setattr(m, attr, -1.0)
        m.boundary_sides_touched = -1
        m.n_points = -1
        if result is None or not result.ok:
            info = info or {}
            m.target_kind = str(info.get('kind', ''))
            m.status = STATUS_REJECT  # 拟合失败不发 pose/axis，仅留诊断记录
            m.diagnostic_flags = ['refit_failed',
                                  str(info.get('reason', 'unknown'))]
            return m
        m.target_kind = 'fruit' if result.kind == 'sphere' else 'bag'
        m.n_points = int(result.n_points)
        m.bag_diameter_upper_m = float(result.diameter)
        m.travel_m = float(result.span_m)
        if result.kind == 'cylinder':
            m.bag_length_m = float(result.span_m)
            m.cylinder_rms_m = float(result.rmse)
            m.cylinder_inlier_ratio = float(result.inlier_ratio)
        else:
            m.fruit_radius_m = float(result.radius)
            m.sphere_rms_m = float(result.rmse)
            m.sphere_inlier_ratio = float(result.inlier_ratio)
        m.status = int(result.status)
        m.diagnostic_flags = list(result.flags)
        return m

    def _grasp_decision(self) -> dict:
        """把最终精化质量归一成只读抓取许可，不发送运动指令."""
        decision = {
            'harvest_run_id': self._harvest_run_id,
            'target_id': self.collector.target_id,
            'allowed': False,
            'geometry_valid': False,
            'reason': 'reconstruction_not_ready',
        }
        if self.collector.state != 'READY':
            return decision
        result = self._refined
        if result is None or not result.ok:
            decision['reason'] = 'refined_geometry_unavailable'
            return decision
        angle = result.axis_angle_deg
        if angle is None:
            angle = axis_angle_deg(result.axis, self._bound_axis_hint)
        if angle is not None:
            decision['axis_angle_deg'] = float(angle)
            max_deg = float(self.params.refit.max_axis_angle_deg)
            decision['diagnostic_axis_mismatch'] = bool(angle > max_deg)
        entry = [float(v) for v in result.entry]
        axis = [float(v) for v in result.axis]
        pregrasp = result.pregrasp
        if pregrasp is None:
            pregrasp = [
                entry[0] - 0.10 * axis[0],
                entry[1] - 0.10 * axis[1],
                entry[2] - 0.10 * axis[2],
            ]
        cut_src = result.cut_pose
        if cut_src is None:
            cut_src = result.neck if result.neck is not None else entry
        decision.update({
            'geometry_valid': True,
            'entry': entry,
            'axis': axis,
            'pregrasp': [float(v) for v in pregrasp],
            'cut_pose': [float(v) for v in cut_src],
            'diameter_m': float(result.d95_m or result.diameter or 0.0),
            'd95_m': float(result.d95_m or result.diameter or 0.0),
            'travel_m': float(result.cut_travel_m or result.span_m or 0.0),
            'cut_travel_m': float(result.cut_travel_m or 0.0),
            'radial_margin_m': float(result.radial_margin_m or 0.0),
            'axial_margin_m': float(result.axial_margin_m or 0.0),
            'corridor_clear': bool(result.corridor_clear),
            'rmse_m': float(result.rmse or 0.0),
            'inlier_ratio': float(result.inlier_ratio or 0.0),
            'model_revision': str(result.model_revision or ''),
            'tool_profile_id': str(self.params.tool.profile_id),
            'scene_epoch': int(self._scene_epoch or 0),
            'calibration_revision': str(
                getattr(self.params, 'calibration_version', '')
                or 'unspecified'),
            'config_revision': str(getattr(self.params.tool, 'version', '')
                                   or self.params.tool.profile_id),
            'geometry_capability': CAPABILITY_VALID,
            'pregrasp_capability': CAPABILITY_VALID,
        })
        budget = result.budget or {}
        if not budget:
            decision['sleeve_capability'] = CAPABILITY_UNKNOWN
            decision['cut_capability'] = CAPABILITY_UNKNOWN
            decision['reason'] = 'bag_model_unavailable'
            decision['failure_code'] = 3
            return decision
        decision['sleeve_capability'] = int(
            budget.get('sleeve_capability',
                       CAPABILITY_VALID if budget.get('sleeve_ok')
                       else CAPABILITY_INVALID))
        decision['cut_capability'] = int(
            budget.get('cut_capability',
                       CAPABILITY_VALID if budget.get('cut_ok')
                       else CAPABILITY_INVALID))
        decision['reason'] = str(
            budget.get('reason') or 'refined_geometry_accept')
        decision['failure_code'] = int(budget.get('failure_code') or 0)
        return decision

    def _diagnostics(self) -> dict:
        """
        组装完整诊断 dict（调试明细，随 /diagnostics_debug 以 JSON 发出）.

        N7：帧栈读取改锁内 tuple(frames) 快照（心跳路径锁外可达）；
        PF-5：cloud_points 走 collector 增量维护的点数标量（不再全量
        vstack 整帧栈只为数一个数——Nav2 诊断不占热路径同款原则）。
        """
        c = self.collector
        with self._state_lock:
            frames = tuple(c.frames)
        last_ratio = frames[-1].valid_depth_ratio if frames else None
        # registration 摘要不存副本，由帧栈各帧 registration 派生
        registrations = [f.registration for f in frames]
        coverage = summarize_view_coverage(frames, c.target_center)
        return {
            'harvest_run_id': self._harvest_run_id,
            'selected_target_id': self._preferred_target_id,
            'target_mask_cache_size': len(self._target_masks),
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'bound_axis_hint': (None if self._bound_axis_hint is None
                                else [float(v) for v in self._bound_axis_hint]),
            # 实际积分帧数；机位数见 view_coverage.view_count。
            'captured_views': len(frames),
            'pose_count': int(coverage.get('view_count') or 0),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'skipped_views': c.skipped_views,
            'skip_reasons': dict(c.skip_reasons),
            'last_skip_code': c.last_skip_code,
            'last_skip_reason': c.last_skip_reason,
            'frame_ring_size': len(self._frame_ring),
            'tf_latency_ms': self._last_tf_latency_ms,
            'valid_depth_ratio': last_ratio,
            'cloud_points': int(c.accumulated_points_count),
            'last_rel_translation_m': c.last_rel_translation_m,
            'last_rel_rotation_deg': c.last_rel_rotation_deg,
            # finalize 时的重叠度指标（pairs/质心）；未 finalize 或帧栈已变为 None
            'overlap': self._overlap_cache,
            # finalize 时的 TSDF 摘要（points/integrate_time_s/roi_center 等）
            'tsdf': self._tsdf_info,
            'registration': {
                'accepted': len(registrations),
                'latest': (None if not registrations
                           else registrations[-1]),
                # E4 ICP target 增量复用观测：当前自适应刷新周期 k、缓存
                # target 点数、全量刷新/增量拼接累计次数
                'target_refresh_period': self._icp_target_cache.period,
                'target_points': self._icp_target_cache.target_size,
                'target_full_refreshes':
                    self._icp_target_cache.full_refreshes,
                'target_incremental_appends':
                    self._icp_target_cache.incremental_appends,
            },
            # 主动视觉控制器消费精确采帧位姿，而不是回调时刻的 latest TF。
            # 覆盖指标按机位聚类（同机位连帧不稀释角基线）；积分帧数另由
            # captured_views 报告。
            'view_coverage': coverage,
            # refit 摘要（kind/center/axis/diameter/rmse/inlier_ratio/ok）；
            # 未跑为 None，失败为 as_dict 投影（ok=False + reason）
            'refined': self._refined_info(),
            'grasp_decision': self._grasp_decision(),
            # 耗时基线（阶段 C 埋点）：ICP/TSDF/帧总 EMA + refit/finalize
            # last 值 + 计数；键集恒定，随 diagnostics_debug JSON 发出
            'timing': self._timing.snapshot(),
        }

    def _refined_info(self) -> Optional[dict]:
        """
        由唯一 refit 缓存 _refined 投影出 diagnostics JSON 的 refined 键.

        Returns
        -------
            None（未跑/已失效）；否则 RefitResult.as_dict()（JSON 原生
            类型投影；成功/失败同键集，axis_angle_deg 保持旧兼容语义）.

        """
        result = self._refined
        if result is None:
            return None
        return result.as_dict()
