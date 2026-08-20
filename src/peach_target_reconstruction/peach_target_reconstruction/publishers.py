"""
发布面 mixin — 状态/诊断/抓取许可/点云/Marker 的消息组装与发布.

职责边界：PeachReconstructionNode 的发布侧全部入口（心跳三件套、状态变化
全量重发、refit 三话题与 BagFitting 诊断记录组装、diagnostics/grasp_decision
纯数据投影，含 refit 缓存的 JSON 投影 _refined_info/_refined_diag_dict）。
只读节点状态并调用发布者，不修改 collector/TSDF/产物；
采集/精化等写路径留在 reconstruction_node.py。

线程模型：_publish_heartbeat 由 executor 定时器线程进入（自取
_state_lock）；_publish_all 由持锁调用方（服务回调 / _auto_start /
_accept_frame 等）在锁内嵌套调用（RLock）。diagnostics/grasp_decision
组装只读快照字段，无额外同步。

宿主契约（mixin，PeachReconstructionNode 提供）：
  发布者 pub_status/pub_diag/pub_diag_debug/pub_grasp_decision/pub_cloud/
  pub_tsdf_cloud/pub_markers/pub_refined_pose/pub_refined_axis/
  pub_refined_diag；状态 collector/params/_refined/_mesh_cache/
  _tsdf_cloud_cache/_tsdf_info/_overlap_cache/_harvest_run_id/
  _preferred_target_id/_target_masks/_bound_axis_hint/_last_tf_latency_ms/
  _timing/_state_lock；E4 增量复用/发布节流状态 _icp_target_cache/
  _publish_throttle/_products_version/_tsdf_cloud_version/
  _products_force_publish（节流标志的清零属发布面自身簿记，非产物写）。
"""
from __future__ import annotations

import json
from typing import Optional

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Vector3Stamped
import numpy as np
from peach_interfaces.msg import (
    BagFitting,
    BagFittingArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
    ShapeHypothesis,
)
from peach_target_reconstruction.cloud_builder import pack_rgb_bgr
from peach_target_reconstruction.geometry_refiner import (
    STATUS_ACCEPT,
    STATUS_REJECT,
)
from peach_target_reconstruction.status_messages import (
    diagnostics_to_status_msg,
    grasp_decision_to_msg,
)
from peach_target_reconstruction.view_coverage import summarize_view_coverage
from peach_target_reconstruction.visualization import (
    build_camera_markers,
    build_mesh_marker,
    build_refined_grasp_markers,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, String


def xyzrgb_to_cloud_msg(xyz: np.ndarray, colors_bgr,
                        header: Header) -> PointCloud2:
    """
    (N, 3) 点 [m] + (N, 3) uint8 BGR → PointCloud2（xyz + 位打包 rgb 字段）.

    布局与 peach_scene_perception_node._xyzrgb_to_cloud 一致：x/y/z/rgb 各一个 FLOAT32
    （offset 0/4/8/12，point_step=16），rgb 位内容为 0xRRGGBB（RViz RGB8
    上色约定）；colors_bgr 为 None 或长度不符时 rgb 字段补零（黑色），
    空云/启动首发同样保持该字段布局。消息组装用官方
    sensor_msgs_py.point_cloud2.create_cloud（numpy 结构化数组快速路径，
    逐字节布局与旧手写 tobytes 版一致）；唯一覆写 is_dense=True——
    create_cloud 硬编码 False，而本云已剔无效深度恒 dense。
    pack_rgb_bgr 无官方等价物（RViz float32 位打包），保留。

    Args:
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常 [m]）；空给空云.
        colors_bgr: (N, 3) uint8 BGR 颜色（OpenCV 排列）；None 补零.
        header: 输出消息头（frame_id 决定点云坐标系解释）.

    Returns
    -------
        sensor_msgs/PointCloud2（is_dense=True，反投影已剔除无效深度）.

    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = int(pts.shape[0])
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = pts
    if colors_bgr is not None and len(colors_bgr) == n:
        arr[:, 3] = pack_rgb_bgr(colors_bgr)
    msg = create_cloud(header, fields, arr)
    msg.is_dense = True  # create_cloud 硬编码 False；本云恒 dense（无效已剔）
    return msg


class PublisherMixin:
    """发布面方法集（宿主契约见模块 docstring；不自带 __init__）."""

    def _publish_heartbeat(self):
        """1Hz 活性心跳：状态 + 诊断 + 抓取许可（轻量三件套，不含云/Marker）."""
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
            grasp_decision_to_msg(self._grasp_decision(), header))

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
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.params.frames.base_frame
        # on_change_only=false 回退逐次全发旧行为（不查节流器）
        throttle = (self._publish_throttle
                    if self.params.publish.on_change_only else None)
        force = self._products_force_publish
        self._products_force_publish = False
        frames = self.collector.frames
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
            markers = build_camera_markers(header, self.collector.frames)
            markers.markers.extend(build_refined_grasp_markers(
                header, self._refined, self.collector.target_id or ''))
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
        if result is None or not result.get('ok'):
            return msg
        bottom = result['bottom']
        neck = result['neck']
        center = 0.5 * (bottom + neck)
        msg.center = Point(
            x=float(center[0]), y=float(center[1]), z=float(center[2]))
        msg.axis = Vector3(
            x=float(result['axis'][0]),
            y=float(result['axis'][1]),
            z=float(result['axis'][2]))
        msg.diameter_m = float(result['diameter'])
        msg.length_m = float(result.get('span_m', 0.0))
        msg.confidence = float(result.get('inlier_ratio', 0.0))
        msg.model_kind = str(result.get('kind', ''))
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
        if result is not None and result['ok']:
            cand = BagGraspCandidate()
            cand.header = header
            cand.target_id = self.collector.target_id
            # entry = bottom − axis×standoff（几何在 refiner 内算好）；
            # 姿态未用（接近方向由 translation_direction 给出），置单位四元数
            cand.entry_pose = Pose(
                position=Point(x=float(result['entry'][0]),
                               y=float(result['entry'][1]),
                               z=float(result['entry'][2])),
                orientation=Quaternion(w=1.0))
            cand.bag_bottom = Point(x=float(result['bottom'][0]),
                                    y=float(result['bottom'][1]),
                                    z=float(result['bottom'][2]))
            cand.bag_neck = Point(x=float(result['neck'][0]),
                                  y=float(result['neck'][1]),
                                  z=float(result['neck'][2]))
            # 剪切行进方向 = refined 轴（bottom→neck 单位向量）
            cand.translation_direction = Vector3(x=float(result['axis'][0]),
                                                 y=float(result['axis'][1]),
                                                 z=float(result['axis'][2]))
            # 圆柱为袋径、球为果径（均 = 2r）
            cand.bag_diameter_upper_m = float(result['diameter'])
            cand.suggested_travel_m = float(result['span_m'])
            cand.confidence = float(result['inlier_ratio'])
            cand.status = int(result['status'])
            cand.diagnostic_flags = list(result['flags'])
            cand.strategy_id = f"reconstruction_refit_{result['kind']}"
            pose_arr.candidates.append(cand)
            axis_msg.vector = Vector3(x=float(result['axis'][0]),
                                      y=float(result['axis'][1]),
                                      z=float(result['axis'][2]))
        fit = self._refined_fitting_msg(header, result)
        if fit is not None:
            fit_arr.fittings.append(fit)
        return pose_arr, axis_msg, fit_arr

    def _refined_fitting_msg(self, header: Header,
                             result: Optional[dict]) -> Optional[BagFitting]:
        """
        由 refit 结果组 BagFitting（无效标量 -1，语义对齐感知包 _to_fitting）.

        Args:
            header: 输出头.
            result: refit 唯一缓存 _refined（成功结果或 {'ok': False,
                'reason': ...} 失败记录）；None 表示未跑（失败时由
                _refined_info() 取原因，发 REJECT 记录）.

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
        for attr in ('axis_confidence', 'axis_disagreement_deg', 'theta_err_deg',
                     'error_budget_mm', 'radial_clearance_mm', 'valid_depth_ratio',
                     'foreground_ratio', 'boundary_touch_ratio', 'bag_length_m',
                     'bag_diameter_upper_m', 'travel_m', 'cylinder_rms_m',
                     'cylinder_inlier_ratio', 'fruit_radius_m', 'sphere_rms_m',
                     'sphere_inlier_ratio', 'cavity_dip_mm'):
            setattr(m, attr, -1.0)
        m.boundary_sides_touched = -1
        m.n_points = -1
        if result is None or not result['ok']:
            info = info or {}
            m.target_kind = str(info.get('kind', ''))
            m.status = STATUS_REJECT  # 拟合失败不发 pose/axis，仅留诊断记录
            m.diagnostic_flags = ['refit_failed',
                                  str(info.get('reason', 'unknown'))]
            return m
        m.target_kind = 'fruit' if result['kind'] == 'sphere' else 'bag'
        m.n_points = int(result['n_points'])
        m.bag_diameter_upper_m = float(result['diameter'])
        m.travel_m = float(result['span_m'])
        if result['kind'] == 'cylinder':
            m.bag_length_m = float(result['span_m'])
            m.cylinder_rms_m = float(result['rmse'])
            m.cylinder_inlier_ratio = float(result['inlier_ratio'])
        else:
            m.fruit_radius_m = float(result['radius'])
            m.sphere_rms_m = float(result['rmse'])
            m.sphere_inlier_ratio = float(result['inlier_ratio'])
        m.status = int(result['status'])
        m.diagnostic_flags = list(result['flags'])
        return m

    def _grasp_decision(self) -> dict:
        """把最终精化质量归一成只读抓取许可，不发送运动指令."""
        decision = {
            'harvest_run_id': self._harvest_run_id,
            'target_id': self.collector.target_id,
            'allowed': False,
            'reason': 'reconstruction_not_ready',
        }
        if self.collector.state != 'READY':
            return decision
        result = self._refined
        if result is None or not result.get('ok'):
            decision['reason'] = 'refined_geometry_unavailable'
            return decision
        if int(result.get('status', STATUS_REJECT)) != STATUS_ACCEPT:
            decision['reason'] = 'refined_quality_requires_reobserve'
            return decision
        decision.update({
            'allowed': True,
            'reason': 'refined_geometry_accept',
            'entry': [float(v) for v in result['entry']],
            'axis': [float(v) for v in result['axis']],
            'diameter_m': float(result['diameter']),
            'rmse_m': float(result['rmse']),
            'inlier_ratio': float(result['inlier_ratio']),
        })
        return decision

    def _diagnostics(self) -> dict:
        """组装完整诊断 dict（调试明细，随 /diagnostics_debug 以 JSON 发出）."""
        c = self.collector
        cloud = c.accumulated_cloud()
        last_ratio = c.frames[-1].valid_depth_ratio if c.frames else None
        # registration 摘要不存副本，由帧栈各帧 registration 派生
        registrations = [f.registration for f in c.frames]
        coverage = summarize_view_coverage(c.frames, c.target_center)
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
            'captured_views': (coverage['view_count'] if coverage['valid']
                               else len(c.frames)),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'tf_latency_ms': self._last_tf_latency_ms,
            'valid_depth_ratio': last_ratio,
            'cloud_points': int(cloud.shape[0]),
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
            # 覆盖指标按机位聚类（同机位连帧不稀释基线），captured_views 同口径。
            'view_coverage': coverage,
            # refit 摘要（kind/center/axis/diameter/rmse/inlier_ratio/ok）；
            # 未跑为 None，失败为 {'ok': False, 'reason': ...}
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
            None（未跑/已失效）；失败记录原样拷贝（{'ok': False,
            'reason': ...}）；成功结果经 _refined_diag_dict 转 JSON 形态.

        """
        result = self._refined
        if result is None:
            return None
        if not result.get('ok'):
            return dict(result)
        return self._refined_diag_dict(result)

    @staticmethod
    def _refined_diag_dict(result: dict) -> dict:
        """
        组装 diagnostics JSON 的 refined 键（refit 成功结果，numpy→原生类型）.

        Args:
            result: refine_geometry 的 ok=True 结果.

        Returns
        -------
            JSON 可序列化 dict（kind/center/axis/diameter/rmse/
            inlier_ratio/ok 等）.

        """
        return {
            'ok': True,
            'kind': result['kind'],
            'status': int(result['status']),
            'center': [float(v) for v in result['center']],
            'axis': [float(v) for v in result['axis']],
            'bottom': [float(v) for v in result['bottom']],
            'neck': [float(v) for v in result['neck']],
            'diameter': float(result['diameter']),
            'span_m': float(result['span_m']),
            'rmse': float(result['rmse']),
            'inlier_ratio': float(result['inlier_ratio']),
            'n_points': int(result['n_points']),
            'flags': list(result['flags']),
        }
