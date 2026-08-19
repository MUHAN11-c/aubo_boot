"""
采摘感知算法验证过程数据记录器 — record 子命令（不随 colcon 构建）.

用途：最终综合验证报告（图像/点云 + 过程数据 + 参数分析 + 方向/定位分析）
的数据基础设施。每次 record 调用把「当前验证步骤」的全部可获取产物落盘到
一个步骤目录；每件产物独立 try——话题缺失/超时只在 manifest 记
'missing'，不让整个 record 失败。

每步落盘内容:
  感知/重建话题快照    perception_initial_pose.yaml、perception_axis.yaml、
                       perception_diagnostics.yaml、perception_debug_image.png、
                       perception_masks.png、perception_single_cloud.ply、
                       reconstruction_diagnostics.json、reconstruction_status.txt、
                       reconstruction_local_cloud.ply
  相机原始帧           camera_rgb.png（bgr8）、camera_depth.npy（原始数组）、
                       camera_depth_viz.png（伪彩色 + colorbar，注明单位）
  几何渲染             cloud_render.png（累加云优先、检测云兜底的俯视/侧视
                       双联散点离屏渲染，固定种子抽稀 ≤3 万点，按第三轴着色）
  参数与 TF            params_<节点>.yaml（ros2 param dump）、tf_snapshot.yaml
  清单                 manifest.yaml（每件 saved|missing + 关键标量摘要）

运行目录约定:
  <root>/run_YYYYMMDD_HHMMSS/    run 目录（root 默认 <工作区>/validation_runs/，
                                 按脚本位置 parents[1] 推算，--root 可覆盖）
  <root>/current_run             当前活动 run 指针（纯文本；--new-run 强制新建）
  run_*/.recorder_state.yaml     run 状态（下一步骤序号）
  run_*/step_NN_<名字>/          步骤目录（序号自增，NN 从 01 起）
  run_*/index.md                 run 级索引（每次 record 追加一行摘要）

用法（需先 source /opt/ros/jazzy/setup.bash 与工作区 install/setup.bash）:
    aubo_py3.12/bin/python tools/peach_validation_recorder.py record --step perception
    aubo_py3.12/bin/python tools/peach_validation_recorder.py record \
        --step capture_2views --note '速度 0.1，第 2 视角'
    aubo_py3.12/bin/python tools/peach_validation_recorder.py record \
        --step finalized --new-run --timeout 8
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
import matplotlib

matplotlib.use('Agg')  # 无显示环境的离屏渲染（须在 pyplot 导入前设置）
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
from peach_interfaces.msg import BagFittingArray, BagGraspCandidateArray
import rclpy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

# 一次性订阅的话题表：(产物名, topic, 消息类型, 是否闩锁话题)
# 闩锁（transient_local）话题的订阅侧必须同样请求 transient_local 才能拿到
# 最后一次发布（volatile 订阅者匹配得上但收不到历史样本）
_TOPICS_YAML = [
    ('perception_initial_pose.yaml', '/peach/perception/initial_pose',
     BagGraspCandidateArray, False),
    ('perception_axis.yaml', '/peach/perception/axis', Vector3Stamped, False),
    ('perception_diagnostics.yaml', '/peach/perception/diagnostics',
     BagFittingArray, False),
]
_TOPICS_IMAGE = [
    ('perception_debug_image.png', '/peach/perception/debug_image', 'bgr8', False),
    ('perception_masks.png', '/peach/perception/masks', 'passthrough', False),
]
_TOPICS_CLOUD = [
    ('perception_single_cloud.ply', '/peach/perception/single_cloud', False),
    ('reconstruction_local_cloud.ply', '/peach/reconstruction/local_cloud', True),
]
_PARAM_NODES = [
    ('params_peach_scene_perception_node.yaml',
     '/peach_scene_perception_node'),
    ('params_peach_target_reconstruction_node.yaml',
     '/peach_target_reconstruction_node'),
]
# TF 快照：(target, source)——写 yaml 时键为 'target<-source'
_TF_PAIRS = [
    ('base_link', 'camera_color_optical_frame'),
    ('wrist3_Link', 'tcp'),
]
_STATUS_NAME = {0: 'ACCEPT', 1: 'REOBSERVE', 2: 'REJECT'}


def _default_root() -> Path:
    """默认根目录 <工作区>/validation_runs（tools/ 上一级即工作区根）."""
    return Path(__file__).resolve().parents[1] / 'validation_runs'


def _sanitize(name: str) -> str:
    """步骤名转目录安全字符串（空白/斜杠折叠为下划线）."""
    cleaned = re.sub(r'[\s/\\]+', '_', name.strip())
    return cleaned or 'unnamed'


def _resolve_run_dir(root: Path, new_run: bool) -> Path:
    """解析当前 run 目录：--new-run 或无指针/指针失效时新建."""
    root.mkdir(parents=True, exist_ok=True)
    pointer = root / 'current_run'
    if not new_run and pointer.is_file():
        candidate = root / pointer.read_text(encoding='utf-8').strip()
        if candidate.is_dir():
            return candidate
    run_dir = root / f'run_{datetime.now():%Y%m%d_%H%M%S}'
    run_dir.mkdir(parents=True, exist_ok=True)
    pointer.write_text(run_dir.name, encoding='utf-8')
    return run_dir


def _load_next_index(run_dir: Path) -> int:
    """从 run 内状态文件读下一步骤序号（缺省 1）."""
    state_file = run_dir / '.recorder_state.yaml'
    if state_file.is_file():
        with open(state_file, encoding='utf-8') as f:
            state = yaml.safe_load(f) or {}
        return int(state.get('next_step_index', 1))
    return 1


def _save_next_index(run_dir: Path, next_index: int) -> None:
    """写 run 内状态文件（更新序号；保留已有 created_iso）."""
    state_file = run_dir / '.recorder_state.yaml'
    state = {}
    if state_file.is_file():
        with open(state_file, encoding='utf-8') as f:
            state = yaml.safe_load(f) or {}
    else:
        state['created_iso'] = datetime.now().isoformat(timespec='seconds')
    state['next_step_index'] = next_index
    _dump_yaml(state, state_file)


def _dump_yaml(data, path) -> None:
    """写 yaml（utf-8、保持键序）；调用方保证数据可序列化."""
    with open(str(path), 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _to_plain(obj):
    """OrderedDict/嵌套结构 → 原生 dict/list（yaml.safe_dump 只吃原生类型）."""
    if isinstance(obj, dict):
        return {str(k): _to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_plain(v) for v in obj]
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    return obj


def _wait_one(node, topic: str, msg_type, timeout_sec: float,
              latched: bool = False):
    """
    一次性订阅：等到一条消息或超时返回 None（订阅后马上销毁）.

    latched=True 时用 transient_local QoS（对应闩锁发布者，能拿到最后一次
    发布）；否则用默认 depth=10 配置（匹配逐帧发布的 volatile 话题）。
    """
    if latched:
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
    else:
        qos = rclpy.qos.QoSProfile(depth=10)
    got = []
    sub = node.create_subscription(msg_type, topic, got.append, qos)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got[0] if got else None


def _cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N, 3) float32；x/y/z 连续 FLOAT32 布局走 numpy 快速路径."""
    offsets = {}
    for f in msg.fields:
        offsets[f.name] = (f.offset, f.datatype)
    fast = all(offsets.get(k) == (i * 4, PointField.FLOAT32)
               for i, k in enumerate(('x', 'y', 'z')))
    if fast and msg.point_step % 4 == 0:
        stride_f = msg.point_step // 4
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        xyz = arr.reshape(-1, stride_f)[:, :3]
    else:
        # 非标准布局回退 sensor_msgs_py 通用读法（慢但通用）
        from sensor_msgs_py import point_cloud2 as pc2
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                   skip_nans=True))
        xyz = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _write_ply(path, xyz: np.ndarray) -> None:
    """写 ASCII PLY（仅 xyz；不依赖 open3d）."""
    with open(str(path), 'w', encoding='utf-8') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {xyz.shape[0]}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('end_header\n')
        np.savetxt(f, xyz, fmt='%.6f')


def _save_depth_viz(path, depth: np.ndarray) -> None:
    """
    深度图伪彩色可视化（turbo + colorbar，单位按 dtype 注明）.

    Args:
        path: 输出 png 路径.
        depth: (H, W) 深度数组（uint16=原始值；float=米）.

    Returns
    -------
        无返回值（None）；图像写入 path.

    """
    # 图面文字用英文：本机 matplotlib 默认 DejaVu Sans 无 CJK 字形，中文会变方框
    if depth.dtype == np.uint16:
        # 原始值：毫米 = raw × depth_scale_unit（见对应 params dump）
        label = 'depth raw (uint16; mm = raw * depth_scale_unit)'
    else:
        label = 'depth [m]'
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(depth, cmap='turbo')
    ax.set_title(f'camera_depth (dtype={depth.dtype})')
    ax.set_xlabel('u [px]')
    ax.set_ylabel('v [px]')
    fig.colorbar(im, ax=ax, label=label)
    fig.tight_layout()
    fig.savefig(str(path), dpi=100)
    plt.close(fig)


def _save_cloud_render(path, xyz: np.ndarray, title: str,
                       max_points: int = 30000, seed: int = 0) -> None:
    """
    点云离屏渲染：俯视(X-Y) + 侧视(X-Z) 双联散点，按第三轴着色.

    固定种子抽稀到 ≤max_points。刻意不用 matplotlib 3D projection：本机
    venv 存在 matplotlib 双版本共存问题导致 Axes3D 不可用（报
    "3D projection is not available"），2D 双视图更稳且几何关系同样可读。

    Args:
        path: 输出 png 路径.
        xyz: (N, 3) 点云（base_link 系 [m]）.
        title: 图标题（带步名与时间戳）.
        max_points: 抽稀上限（默认 3 万点，防 scatter 过慢）.
        seed: 抽稀随机种子（固定保证可复现）.

    Returns
    -------
        无返回值（None）；图像写入 path.

    """
    pts = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] > max_points:
        rng = np.random.default_rng(seed)
        idx = rng.choice(pts.shape[0], size=max_points, replace=False)
        pts = pts[idx]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
    # 图面文字用英文：本机 matplotlib 默认 DejaVu Sans 无 CJK 字形
    sc1 = ax1.scatter(pts[:, 0], pts[:, 1], s=1, c=pts[:, 2], cmap='viridis')
    ax1.set_xlabel('X [m] (base_link)')
    ax1.set_ylabel('Y [m] (base_link)')
    ax1.set_title('Top view X-Y (color=Z)')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.grid(alpha=0.3)
    fig.colorbar(sc1, ax=ax1, label='Z [m]', shrink=0.8)
    sc2 = ax2.scatter(pts[:, 0], pts[:, 2], s=1, c=pts[:, 1], cmap='viridis')
    ax2.set_xlabel('X [m] (base_link)')
    ax2.set_ylabel('Z [m] (base_link)')
    ax2.set_title('Side view X-Z (color=Y)')
    ax2.set_aspect('equal', adjustable='datalim')
    ax2.grid(alpha=0.3)
    fig.colorbar(sc2, ax=ax2, label='Y [m]', shrink=0.8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(str(path), dpi=110)
    plt.close(fig)


def _dump_params(node_name: str, timeout_sec: float):
    """subprocess 调 ros2 param dump；节点不在/超时返回 None."""
    try:
        result = subprocess.run(
            ['ros2', 'param', 'dump', node_name],
            capture_output=True, text=True, timeout=timeout_sec + 10.0,
            check=False)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    return result.stdout


def _tf_snapshot(node, timeout_sec: float) -> dict:
    """查 _TF_PAIRS 的当前变换（latest）；查不到的记 missing 字符串."""
    buffer = Buffer()
    TransformListener(buffer, node)
    snapshot = {}
    for target, source in _TF_PAIRS:
        key = f'{target}<-{source}'
        # 自旋等数据（静态/动态 TF 都兼容），不用 lookup_transform 的阻塞
        # timeout（其实现等待期不 spin，动态 TF 会干等）
        deadline = time.monotonic() + timeout_sec
        tf = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buffer.can_transform(target, source, Time()):
                tf = buffer.lookup_transform(target, source, Time())
                break
        if tf is None:
            snapshot[key] = 'missing'
            continue
        t, q = tf.transform.translation, tf.transform.rotation
        snapshot[key] = {
            'translation': [float(t.x), float(t.y), float(t.z)],
            'rotation_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
        }
    return snapshot


def _summarize(initial_pose, recon_diag_json, recon_status) -> dict:
    """从关键消息提取标量摘要（manifest/index.md 共用）."""
    summary = {}
    if initial_pose is not None:
        cands = list(initial_pose.candidates)
        counts = {}
        for c in cands:
            key = _STATUS_NAME.get(c.status, str(c.status))
            counts[key] = counts.get(key, 0) + 1
        summary['candidates_total'] = len(cands)
        summary['status_counts'] = counts
        best = None
        for c in cands:
            if c.status == 0:  # ACCEPT 优先，否则第一个
                best = c
                break
        if best is None and cands:
            best = cands[0]
        if best is not None:
            p = best.entry_pose.position
            d = best.translation_direction
            summary['best_candidate'] = {
                'target_id': best.target_id,
                'position': [float(p.x), float(p.y), float(p.z)],
                'axis': [float(d.x), float(d.y), float(d.z)],
                'diameter_m': float(best.bag_diameter_upper_m),
                'status': _STATUS_NAME.get(best.status, str(best.status)),
            }
    if recon_diag_json:
        try:
            diag = json.loads(recon_diag_json)
        except json.JSONDecodeError:
            diag = None
        if diag is not None:
            summary['reconstruction_state'] = diag.get('state')
            pairs = (diag.get('overlap') or {}).get('pairs') or []
            if pairs:
                summary['overlap'] = {
                    'mean_mm': float(np.mean([p['mean_mm'] for p in pairs])),
                    'p95_mm': float(np.max([p['p95_mm'] for p in pairs])),
                }
            else:
                summary['overlap'] = None
    if recon_status:
        summary['reconstruction_status'] = recon_status
    return summary


def _summary_line(summary: dict) -> str:
    """index.md 单行摘要：候选计数 / overlap / 重建状态."""
    parts = []
    if 'candidates_total' in summary:
        c = summary.get('status_counts', {})
        parts.append(
            f"候选 {summary['candidates_total']}"
            f"（ACCEPT {c.get('ACCEPT', 0)}/REOBSERVE {c.get('REOBSERVE', 0)}"
            f"/REJECT {c.get('REJECT', 0)}）")
    if summary.get('overlap'):
        ov = summary['overlap']
        parts.append(f"overlap mean={ov['mean_mm']:.1f}mm "
                     f"p95={ov['p95_mm']:.1f}mm")
    if summary.get('reconstruction_status'):
        parts.append(f"recon={summary['reconstruction_status']}")
    return '；'.join(parts) if parts else '无在线数据'


def _append_index(run_dir: Path, step_index: int, step_name: str,
                  note: str, summary: dict) -> None:
    """run 级 index.md 追加一行（首次创建带表头）."""
    index = run_dir / 'index.md'
    if not index.exists():
        index.write_text(
            f'# 验证运行索引 — {run_dir.name}\n\n'
            '| 步骤 | 名字 | 时间 | 摘要 | 备注 |\n'
            '|---|---|---|---|---|\n', encoding='utf-8')
    line = (f'| {step_index:02d} | {step_name} | '
            f'{datetime.now():%Y-%m-%d %H:%M:%S} | '
            f'{_summary_line(summary)} | {note} |\n')
    with open(index, 'a', encoding='utf-8') as f:
        f.write(line)


def _record_all(node, step_dir: Path, timeout_sec: float, render_title: str):
    """逐项记录全部产物；返回 (artifacts 状态, errors 明细, summary 摘要)."""
    artifacts = {}
    errors = {}
    bridge = CvBridge()
    initial_pose = None
    recon_diag_json = None
    recon_status = None

    for fname, topic, msg_type, latched in _TOPICS_YAML:
        try:
            msg = _wait_one(node, topic, msg_type, timeout_sec, latched)
            if msg is None:
                artifacts[fname] = 'missing'
                continue
            _dump_yaml(_to_plain(message_to_ordereddict(msg)),
                       step_dir / fname)
            artifacts[fname] = 'saved'
            if 'initial_pose' in fname:
                initial_pose = msg
        except Exception as exc:  # noqa: BLE001
            artifacts[fname] = 'missing'
            errors[fname] = str(exc)

    for fname, topic, encoding, latched in _TOPICS_IMAGE:
        try:
            msg = _wait_one(node, topic, Image, timeout_sec, latched)
            if msg is None:
                artifacts[fname] = 'missing'
                continue
            img = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            # cv_bridge bgr8 即 OpenCV BGR 排列，cv2.imwrite 直写无色差
            cv2.imwrite(str(step_dir / fname), img)
            artifacts[fname] = 'saved'
        except Exception as exc:  # noqa: BLE001
            artifacts[fname] = 'missing'
            errors[fname] = str(exc)

    # ---- 相机原始帧（RGB + 深度原始数组/伪彩色可视化）----
    try:
        msg = _wait_one(node, '/camera/color/image_raw', Image, timeout_sec)
        if msg is None:
            artifacts['camera_rgb.png'] = 'missing'
        else:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(str(step_dir / 'camera_rgb.png'), img)
            artifacts['camera_rgb.png'] = 'saved'
    except Exception as exc:  # noqa: BLE001
        artifacts['camera_rgb.png'] = 'missing'
        errors['camera_rgb.png'] = str(exc)

    depth_arr = None
    try:
        msg = _wait_one(node, '/camera/depth/image_raw', Image, timeout_sec)
        if msg is not None:
            # passthrough 保留原始 dtype/尺度（本数据集为 uint16 毫米）
            depth_arr = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            np.save(str(step_dir / 'camera_depth.npy'), depth_arr)
            artifacts['camera_depth.npy'] = 'saved'
        else:
            artifacts['camera_depth.npy'] = 'missing'
    except Exception as exc:  # noqa: BLE001
        artifacts['camera_depth.npy'] = 'missing'
        errors['camera_depth.npy'] = str(exc)
    try:
        if depth_arr is None:
            artifacts['camera_depth_viz.png'] = 'missing'
        else:
            _save_depth_viz(step_dir / 'camera_depth_viz.png', depth_arr)
            artifacts['camera_depth_viz.png'] = 'saved'
    except Exception as exc:  # noqa: BLE001
        artifacts['camera_depth_viz.png'] = 'missing'
        errors['camera_depth_viz.png'] = str(exc)

    clouds_xyz = {}
    for fname, topic, latched in _TOPICS_CLOUD:
        try:
            msg = _wait_one(node, topic, PointCloud2, timeout_sec, latched)
            if msg is None:
                artifacts[fname] = 'missing'
                continue
            xyz = _cloud_to_xyz(msg)
            _write_ply(step_dir / fname, xyz)
            clouds_xyz[fname] = xyz
            artifacts[fname] = 'saved'
        except Exception as exc:  # noqa: BLE001
            artifacts[fname] = 'missing'
            errors[fname] = str(exc)

    # ---- 点云离屏渲染：优先重建累加云，缺失回退感知检测云 ----
    try:
        render_xyz = clouds_xyz.get('reconstruction_local_cloud.ply')
        render_from = 'reconstruction_local_cloud'
        if render_xyz is None or render_xyz.size == 0:
            render_xyz = clouds_xyz.get('perception_single_cloud.ply')
            render_from = 'perception_single_cloud'
        if render_xyz is None or render_xyz.size == 0:
            artifacts['cloud_render.png'] = 'missing'
        else:
            _save_cloud_render(step_dir / 'cloud_render.png', render_xyz,
                               f'{render_title} | {render_from}')
            artifacts['cloud_render.png'] = 'saved'
    except Exception as exc:  # noqa: BLE001
        artifacts['cloud_render.png'] = 'missing'
        errors['cloud_render.png'] = str(exc)

    try:
        # 诊断主话题已类型化（ReconstructionStatus）；完整明细 JSON 走
        # diagnostics_debug 调试话题，落盘内容不变
        msg = _wait_one(node, '/peach/reconstruction/diagnostics_debug',
                        String, timeout_sec, latched=True)
        if msg is None:
            artifacts['reconstruction_diagnostics.json'] = 'missing'
        else:
            # String 载荷本身就是 JSON；pretty 化落盘便于人读
            recon_diag_json = msg.data
            pretty = json.dumps(json.loads(msg.data), ensure_ascii=False,
                                indent=2)
            (step_dir / 'reconstruction_diagnostics.json').write_text(
                pretty, encoding='utf-8')
            artifacts['reconstruction_diagnostics.json'] = 'saved'
    except Exception as exc:  # noqa: BLE001
        artifacts['reconstruction_diagnostics.json'] = 'missing'
        errors['reconstruction_diagnostics.json'] = str(exc)

    try:
        msg = _wait_one(node, '/peach/reconstruction/status', String,
                        timeout_sec, latched=True)
        if msg is None:
            artifacts['reconstruction_status.txt'] = 'missing'
        else:
            recon_status = msg.data
            (step_dir / 'reconstruction_status.txt').write_text(
                msg.data + '\n', encoding='utf-8')
            artifacts['reconstruction_status.txt'] = 'saved'
    except Exception as exc:  # noqa: BLE001
        artifacts['reconstruction_status.txt'] = 'missing'
        errors['reconstruction_status.txt'] = str(exc)

    for fname, node_name in _PARAM_NODES:
        try:
            text = _dump_params(node_name, timeout_sec)
            if text is None:
                artifacts[fname] = 'missing'
            else:
                (step_dir / fname).write_text(text, encoding='utf-8')
                artifacts[fname] = 'saved'
        except Exception as exc:  # noqa: BLE001
            artifacts[fname] = 'missing'
            errors[fname] = str(exc)

    try:
        snapshot = _tf_snapshot(node, timeout_sec)
        _dump_yaml(snapshot, step_dir / 'tf_snapshot.yaml')
        artifacts['tf_snapshot.yaml'] = (
            'saved' if any(v != 'missing' for v in snapshot.values())
            else 'missing')
    except Exception as exc:  # noqa: BLE001
        artifacts['tf_snapshot.yaml'] = 'missing'
        errors['tf_snapshot.yaml'] = str(exc)

    summary = _summarize(initial_pose, recon_diag_json, recon_status)
    if clouds_xyz:
        # 各云点数登记进 summary（渲染/报告汇编用）
        summary['cloud_points'] = {
            fname.replace('.ply', ''): int(xyz.shape[0])
            for fname, xyz in clouds_xyz.items()
        }
    return artifacts, errors, summary


def cmd_record(args) -> int:
    """record 子命令主流程：建步骤目录 → 逐项落盘 → manifest → index.md."""
    root = Path(args.root) if args.root else _default_root()
    run_dir = _resolve_run_dir(root, args.new_run)
    step_index = _load_next_index(run_dir)
    step_name = _sanitize(args.step)
    step_dir = run_dir / f'step_{step_index:02d}_{step_name}'
    step_dir.mkdir(parents=True, exist_ok=True)
    stamp_iso = datetime.now().isoformat(timespec='milliseconds')
    render_title = f'step_{step_index:02d}_{step_name} | {stamp_iso}'

    rclpy.init(args=args.ros_args)
    node = rclpy.create_node('peach_validation_recorder')
    try:
        artifacts, errors, summary = _record_all(
            node, step_dir, args.timeout, render_title)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    manifest = {
        'step_index': step_index,
        'step_name': step_name,
        'step_dir': step_dir.name,
        'run_dir': run_dir.name,
        'timestamp_iso': stamp_iso,
        'note': args.note,
        'artifacts': artifacts,
        'errors': errors,
        'summary': summary,
    }
    _dump_yaml(manifest, step_dir / 'manifest.yaml')
    _save_next_index(run_dir, step_index + 1)
    _append_index(run_dir, step_index, step_name, args.note, summary)

    saved = sum(1 for v in artifacts.values() if v == 'saved')
    print(f'[recorder] {step_dir}：{saved}/{len(artifacts)} 件已保存；'
          f'摘要：{_summary_line(summary)}')
    missing = [k for k, v in artifacts.items() if v != 'saved']
    if missing:
        print(f'[recorder] missing: {", ".join(missing)}')
    return 0


def main(args=None):
    """CLI 入口：record 子命令 + 参数解析（ROS 参数透传给 rclpy）."""
    parser = argparse.ArgumentParser(description='采摘感知验证过程数据记录器')
    sub = parser.add_subparsers(dest='command', required=True)
    rec = sub.add_parser('record', help='记录当前验证步骤的全部产物')
    rec.add_argument('--step', required=True, help='步骤名（目录 step_NN_<名字>）')
    rec.add_argument('--note', default='', help='备注（写入 manifest 与 index.md）')
    rec.add_argument('--root', default='',
                     help='运行根目录；空 = <工作区>/validation_runs')
    rec.add_argument('--new-run', action='store_true', help='强制新建 run 目录')
    rec.add_argument('--timeout', type=float, default=5.0,
                     help='单个话题/TF 的等待超时 (s)，默认 5')
    known, ros_args = parser.parse_known_args(args=args)
    known.ros_args = ros_args
    if known.command == 'record':
        raise SystemExit(cmd_record(known))


if __name__ == '__main__':
    main()
