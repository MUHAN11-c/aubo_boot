"""
session/geometry 落盘纯核（W4 拆自节点与 publish 的文件 IO）.

- ``SessionRecorder``：session/geometry 根解析与 geometry.jsonl 追加
  （节点薄壳的纯核本体；锁外写盘语义见 W2/R4）。
- ``save_session`` 及 PLY/yaml writer：自 publish.py 迁入（文件 IO 不再
  混在发布模块）。
- 参数快照经 ``peach_common.yaml_params.snapshot`` 自省（替代旧 94 行
  手抄），帧级摘要由 recorder 组装。
"""
from __future__ import annotations

from datetime import datetime
import json
import logging
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np
from peach_common.paths import safe_component
from peach_harvester.vision.common.runtime import resolve_runs_root
from peach_harvester.vision.target_reconstruction.integrate import (
    require_open3d,
    summarize_view_coverage,
)
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    RefitResult,
)
import yaml

# 纯核不能 import ROS，走 stdlib logging（与 common.runtime 同约定）
_logger = logging.getLogger(__name__)


def _xyz_list(value, fallback=(0.0, 0.0, 0.0)):
    """三维点转三个 float；ndarray 不得走 Python `or`（真值歧义会抛）."""
    if value is None:
        value = fallback
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size < 3 or not np.all(np.isfinite(arr[:3])):
        arr = np.asarray(fallback, dtype=np.float64).reshape(-1)
    return [float(v) for v in arr[:3]]


def _dump_yaml(data: dict, path) -> None:
    """
    把 dict 写入 yaml 文件（utf-8，不排序保持可读顺序）.

    Args:
        data: 可 yaml 序列化的 dict（numpy 类型须已转原生类型）.
        path: 输出路径.

    Returns
    -------
        无返回值（None）；文件写入 path.

    """
    with open(str(path), 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _write_ply_xyzrgb(path, xyz: np.ndarray,
                      colors_bgr: np.ndarray) -> None:
    """
    写 ASCII PLY（open3d 官方 write_point_cloud，xyz + uchar red/green/blue）.

    与旧手写版的差异：标量属性为 double（旧为 float）、头部多一行
    ``comment Created by Open3D``——PLY 消费者（CloudCompare/离线脚本）
    均按属性名解析，无语义差异。颜色 BGR→RGB 经 [0,1] float 往返无损。

    Args:
        path: 输出 ply 路径.
        xyz: (N, 3) 点 [m].
        colors_bgr: (N, 3) uint8 BGR（OpenCV 排列）.

    Returns
    -------
        无返回值（None）；文件写入 path；写失败抛 IOError.

    """
    o3d = require_open3d()
    pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64)))
    pcd.colors = o3d.utility.Vector3dVector(
        np.asarray(colors_bgr, dtype=np.uint8)[:, ::-1] / 255.0)  # BGR→RGB
    if not o3d.io.write_point_cloud(str(path), pcd, write_ascii=True):
        raise OSError(f'PLY 写出失败: {path}')


def _write_triangle_mesh(path, mesh_data: dict) -> None:
    """用 Open3D 官方 writer 保存顶点、三角形、法向和颜色."""
    o3d = require_open3d()
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(mesh_data['vertices'])
    mesh.triangles = o3d.utility.Vector3iVector(mesh_data['triangles'])
    if len(mesh_data.get('normals', [])) == len(mesh_data['vertices']):
        mesh.vertex_normals = o3d.utility.Vector3dVector(mesh_data['normals'])
    colors = mesh_data.get('colors_bgr')
    if colors is not None and len(colors) == len(mesh_data['vertices']):
        mesh.vertex_colors = o3d.utility.Vector3dVector(
            np.asarray(colors, dtype=np.uint8)[:, ::-1] / 255.0)
    if not o3d.io.write_triangle_mesh(
            str(path), mesh, write_ascii=True, write_vertex_normals=True,
            write_vertex_colors=True):
        raise OSError(f'网格 PLY 写出失败: {path}')


def save_session(root_dir, frames: list, metadata: dict,
                 tsdf_cloud=None, tsdf_mesh=None) -> Path:
    """
    把一次重建的全部帧写到 root_dir/session_<时间戳>/.

    Args:
        root_dir: session 根目录（不存在自动创建）.
        frames: CapturedFrame 列表（按采集顺序编号 frame_00, frame_01, ...）.
        metadata: 参数快照等元信息（写入 metadata.yaml）.
        tsdf_cloud: 可选 (xyz, colors_bgr) 元组；给出时写
            result/tsdf_cloud.ply（xyz+rgb）.
        tsdf_mesh: 可选 LocalTsdf.extract_mesh() 字典.

    Returns
    -------
        创建成功的 session 目录 Path.

    """
    root = Path(root_dir)
    # 路径守卫（W1 安全修复）：root_dir 上游含消息来源的 request_id 段
    # （session_root 已净化）；此处再拒显式上跳段，防任何未净化调用方
    # 把写盘目录逃出预期根。
    if '..' in root.parts:
        raise ValueError(f'session root 含上跳段，拒绝落盘: {root}')
    # 微秒参与目录名并禁止复用：连续 finalize 不得静默覆盖前一次原始数据。
    session_dir = root / f'session_{datetime.now():%Y%m%d_%H%M%S_%f}'
    session_dir.mkdir(parents=True, exist_ok=False)
    for i, frame in enumerate(frames):
        stem = str(session_dir / f'frame_{i:02d}')
        if not cv2.imwrite(stem + '_rgb.png', frame.rgb):
            raise OSError(f'RGB PNG 写出失败: {stem}_rgb.png')
        np.save(stem + '_depth.npy', frame.depth_mm)
        _dump_yaml({
            'stamp_sec': float(frame.stamp),
            'width': int(frame.camera_K.get('width', 0)),
            'height': int(frame.camera_K.get('height', 0)),
            'fx': float(frame.camera_K['fx']),
            'fy': float(frame.camera_K['fy']),
            'cx': float(frame.camera_K['cx']),
            'cy': float(frame.camera_K['cy']),
        }, stem + '_camera_info.yaml')
        _dump_yaml({
            'T_base_camera_used': np.asarray(
                frame.T_base_camera, dtype=np.float64).tolist(),
            'T_base_camera_fk': np.asarray(
                getattr(frame, 'T_base_camera_fk', frame.T_base_camera),
                dtype=np.float64).tolist(),
            'camera_position_base': np.asarray(
                frame.camera_position_base, dtype=np.float64).tolist(),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'registration': dict(getattr(frame, 'registration', {})),
            'diagnostic_flags': list(frame.diagnostic_flags),
        }, stem + '_T_base_camera.yaml')
    if tsdf_cloud is not None:
        xyz, colors = tsdf_cloud
        if xyz is not None and len(xyz):
            result_dir = session_dir / 'result'
            result_dir.mkdir(exist_ok=True)
            if colors is None:
                colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            _write_ply_xyzrgb(result_dir / 'tsdf_cloud.ply', xyz, colors)
    if tsdf_mesh is not None and len(tsdf_mesh.get('vertices', [])):
        result_dir = session_dir / 'result'
        result_dir.mkdir(exist_ok=True)
        _write_triangle_mesh(result_dir / 'tsdf_mesh.ply', tsdf_mesh)
    _dump_yaml(metadata, session_dir / 'metadata.yaml')
    return session_dir


def _frame_row(index: int, frame) -> dict:
    """单帧摘要行（metadata.yaml 的 frames 列表元素）."""
    return {
        'index': index,
        'stamp_sec': float(frame.stamp),
        'valid_depth_ratio': float(frame.valid_depth_ratio),
        'camera_position_base': [float(v)
                                 for v in frame.camera_position_base],
        'cloud_points': int(0 if frame.cloud_base is None
                            else frame.cloud_base.shape[0]),
        'diagnostic_flags': list(frame.diagnostic_flags),
        'T_base_camera_fk': np.asarray(
            frame.T_base_camera_fk, dtype=np.float64).tolist(),
        'T_base_camera_used': np.asarray(
            frame.T_base_camera, dtype=np.float64).tolist(),
        'registration': dict(frame.registration),
    }


class SessionRecorder:
    """geometry.jsonl 追加与 session/geometry 根解析（节点薄壳的纯核本体）."""

    def __init__(self, root_resolver: Callable[[], Path]):
        """
        注入配置根解析器（无批次回退布局；每次调用现取，随 ros2 param set 生效）.

        Args:
            root_resolver: 返回配置 session 根（resolve_runs_root 形态）.

        Returns
        -------
            无返回值（None）.

        """
        self._root_resolver = root_resolver
        # 当前批次 request_id（bind_executor_run_id 由节点随 HarvestState 回填；
        # 空串=无批次回退配置根布局）
        self._executor_run_id = ''

    def session_root(self) -> Path:
        """
        Session 根：批次在跑=runs/<request_id>/sessions（单根，R7）.

        无批次回退旧布局（配置根/工作区 runs/）。request_id 为消息来源，
        入路径前经 safe_component 净化（W1 路径穿越修复）。
        """
        if self._executor_run_id:
            return (resolve_runs_root(None) /
                    safe_component(self._executor_run_id, 'harvest') /
                    'sessions')
        return self._root_resolver()

    def geometry_root(self) -> Path:
        """
        geometry.jsonl 根：批次=runs/<request_id>/（单根批根，R7）.

        request_id 消息来源，入路径前净化（W1 路径穿越修复）。
        """
        if self._executor_run_id:
            return (resolve_runs_root(None) /
                    safe_component(self._executor_run_id, 'harvest'))
        return self._root_resolver()

    @staticmethod
    def _append_row(root: Path, row: dict) -> None:
        """追加一行 JSONL（父目录自动创建；IO 异常由调用方兜底）."""
        path = Path(root) / 'geometry.jsonl'
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')

    def geometry_row(self, result: RefitResult, fused: BagModel,
                     target_id: str) -> None:
        """追加 geometry.jsonl 融合行，供离线基线复算（节点 _log_geometry_row 本体）."""
        try:
            cut_pose = fused.cut_pose
            if cut_pose is None:
                cut_pose = result.neck
            row = {
                'target_id': target_id,
                'bag_bottom': _xyz_list(result.bottom),
                'bag_neck': _xyz_list(result.neck),
                'axis': _xyz_list(result.axis, (0.0, 0.0, 1.0)),
                'd95_m': float(result.d95_m or result.diameter or 0),
                'length_m': float(fused.length_m or result.span_m or 0),
                'sigma_position_m': float(fused.sigma_position_m or 0.02),
                'sigma_axis_deg': float(fused.sigma_axis_deg or 8.0),
                'radial_margin_m': float(fused.radial_margin_m or 0.0),
                'axial_margin_m': float(fused.axial_margin_m or 0.0),
                'occlusion_class': str(fused.occlusion_class or ''),
                'allowed': bool(fused.allowed),
                'reason': str(fused.reason or ''),
                'view_count': int(fused.view_count or 0),
                'axis_conflict_deg': float(fused.axis_conflict_deg or 0.0),
                'envelope_conditioned': bool(fused.envelope_conditioned),
                'envelope_reason': str(fused.envelope_reason or ''),
                'cut_pose': _xyz_list(cut_pose),
                'cut_to_fruit_m': float(fused.cut_to_fruit_m or 0.0),
                'cut_travel_m': float(fused.cut_travel_m or 0.0),
                'rmse_m': float(fused.rmse or 0.0),
                'inlier_ratio': float(fused.inlier_ratio or 0.0),
                'corridor_clear': bool(fused.corridor_clear),
                'flags': list(fused.flags or []),
                'fused': True,
            }
            self._append_row(self.geometry_root(), row)
        except Exception as exc:  # noqa: BLE001
            _logger.warning(f'geometry.jsonl 写入失败: {exc}')

    def view_row(self, landmarks, frame, target_id: str) -> None:
        """单视角袋关键点行，供多视角离散度基线（节点 _log_view_geometry 本体）."""
        if landmarks.bottom_center is None or landmarks.neck_center is None:
            return
        try:
            row = {
                'target_id': target_id,
                'bag_bottom': [float(v) for v in landmarks.bottom_center],
                'bag_neck': [float(v) for v in landmarks.neck_center],
                'axis': [float(v) for v in (
                    landmarks.bag_axis if landmarks.bag_axis is not None
                    else (0, 0, 1))],
                'd95_m': float(landmarks.d95_m or 0.0),
                'length_m': float(np.linalg.norm(
                    landmarks.neck_center - landmarks.bottom_center)),
                'sigma_position_m': float(landmarks.sigma_position_m),
                'sigma_axis_deg': float(landmarks.sigma_axis_deg),
                'occlusion_class': str(landmarks.occlusion_class or ''),
                'flags': list(landmarks.flags),
                'stamp_sec': float(getattr(frame, 'stamp', 0.0) or 0.0),
                'fused': False,
            }
            self._append_row(self.geometry_root(), row)
        except OSError:
            _logger.warning('geometry.jsonl 视角行写入失败')

    def session_metadata(
            self, *, collector, parameters: dict, harvest_run_id: str,
            selected_target_id: str, target_mask_cache_size: int,
            tsdf_result: Optional[dict], refined_result: Optional[dict],
            timing: dict) -> dict:
        """
        参数快照与帧级摘要（随 metadata.yaml 落盘，供离线复现）.

        W4：``parameters`` 由调用方以 ``TargetReconstructionParams.snapshot()``
        自省生成（替代旧 94 行手抄转写）；帧级摘要（frames 列表/视角覆盖/
        计数器）仍由本方法组装。
        """
        c = collector
        return {
            'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'node': 'peach_target_reconstruction_node',
            'pipeline': 'exact-time FK + bounded ICP + online TSDF + refit',
            'harvest_run_id': harvest_run_id,
            'selected_target_id': selected_target_id,
            'target_mask_cache_size': int(target_mask_cache_size),
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'captured_views': len(c.frames),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'skipped_views': c.skipped_views,
            'skip_reasons': dict(c.skip_reasons),
            'last_skip_code': c.last_skip_code,
            'view_coverage': summarize_view_coverage(
                c.frames, c.target_center),
            'parameters': parameters,
            'tsdf_result': tsdf_result,
            'refined_result': refined_result,
            # 耗时基线（与 diagnostics JSON 的 timing 子对象同契约同实例）
            'timing': timing,
            'frames': [_frame_row(i, f) for i, f in enumerate(c.frames)],
        }

    def bind_executor_run_id(self, executor_run_id: str) -> None:
        """回填当前批次 request_id（节点 _on_executor_state 同步调用）."""
        self._executor_run_id = str(executor_run_id or '')
