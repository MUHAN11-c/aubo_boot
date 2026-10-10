"""
场景障碍快照纯核（零 ROS）：点云 → 滤除链 → 体素中心.

Survey 快照式建图（2026-09-29 用户裁定：不用 octomap、障碍=场景碰撞对象、
建图/作业分离）：Survey 完成时取最近一帧点云一次性生成障碍快照写入
PlanningScene，作业期冻结，新批次重建。与停走节拍契合，也避免眼在手上
持续更新的地图抖动。

滤除链（顺序即语义；2026-09-29 真相机轮勘定后体素先行）：
  1. 体素化去重 + 工作空间裁剪——后续滤除在体素中心（~数千点）上做。
  2. 自身滤除——体素中心距机器人 collision mesh 表面 <= self_margin +
     体素半对角 剔除：半对角保证整个方块（含角点）不贴机器人表面；
     只滤点再体素化会留下切进工具网格 ~2mm 的方块，FCL 起点即接触、
     CheckStartStateCollision 全拒（09-29 实锤）。眼在手上，工具必在
     相机视野正下方，不滤会把工具点云固化成假障碍；唯一受查对是相机
     ×障碍，假障碍恰在相机近旁必死锁（09-17 同型问题的快照侧解法，
     margin 语义对齐官方 octomap padding_offset）。
  3. 目标邻域滤除——体素中心落入膨胀胶囊（袋径/2+径向余量+半对角，
     轴向两端再延轴向余量+半对角）剔除：袋邻域是套袋工艺必经区（该挡
     的挡、该过的过）；未精化目标无胶囊不滤（不摘就别撞）。
  4. 数量上限。

模块顶层零 ROS import（可 pytest）；Open3D 延迟加载（仅自身滤除与 STL
读取用，仓内 requirements 已钉版本）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np

_MAX_URDF_BYTES = 8 * 1024 * 1024
"""robot_description 上限（8 MiB，含全部 mesh 引用的 URDF 远小于此）。"""


def _parse_urdf_xml(urdf_xml: str) -> ET.Element:
    """
    安全解析 URDF（拒绝 DTD/实体声明并限长度）.

    robot_description 来自本仓 RSP（可信），仍按不可信输入防御——
    防实体扩展（Mimosa 勘定）。
    """
    if len(urdf_xml) > _MAX_URDF_BYTES:
        raise ValueError(f'URDF 超过 {_MAX_URDF_BYTES} 字节上限')
    head = urdf_xml[:2048].lower()
    if '<!doctype' in head or '<!entity' in head:
        raise ValueError('URDF 含 DTD/实体声明，拒绝解析')
    return ET.fromstring(urdf_xml)


# ---------------------------------------------------------------- 几何滤除

def point_segment_distance(
    points: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray
) -> np.ndarray:
    """(N,3) 点到线段 [seg_a, seg_b] 的距离（胶囊滤除底元）."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    seg_a = np.asarray(seg_a, dtype=np.float64).reshape(3)
    seg_b = np.asarray(seg_b, dtype=np.float64).reshape(3)
    ab = seg_b - seg_a
    length2 = float(ab @ ab)
    if length2 < 1.0e-16:
        # 退化胶囊（bottom==neck）：退化为球，距离=到球心。
        return np.linalg.norm(points - seg_a, axis=1)
    t = ((points - seg_a) @ ab) / length2
    t = np.clip(t, 0.0, 1.0)
    closest = seg_a + t[:, None] * ab[None, :]
    return np.linalg.norm(points - closest, axis=1)


@dataclass(frozen=True)
class CapsuleSpec:
    """目标袋胶囊（base 系，米）。radius=袋拟合直径/2（不含余量）."""

    bottom: Tuple[float, float, float]
    neck: Tuple[float, float, float]
    radius: float


def capsule_keep_mask(
    points: np.ndarray,
    capsules: Sequence[CapsuleSpec],
    radial_margin_m: float,
    axial_margin_m: float,
) -> np.ndarray:
    """
    True=保留。落入任一膨胀胶囊（含端球）的点剔除.

    膨胀 = 半径 + radial_margin；轴向两端各延 axial_margin。
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    keep = np.ones(points.shape[0], dtype=bool)
    for capsule in capsules:
        bottom = np.asarray(capsule.bottom, dtype=np.float64)
        neck = np.asarray(capsule.neck, dtype=np.float64)
        axis = neck - bottom
        norm = float(np.linalg.norm(axis))
        if norm > 1.0e-9:
            unit = axis / norm
            seg_a = bottom - unit * axial_margin_m
            seg_b = neck + unit * axial_margin_m
        else:
            seg_a = bottom
            seg_b = neck
        distance = point_segment_distance(points, seg_a, seg_b)
        keep &= distance > (capsule.radius + radial_margin_m)
    return keep


# ---------------------------------------------------------------- URDF/FK

def _rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF rpy（固定轴 RPY，rad）→ 3x3."""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin_to_matrix(
        xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rpy_to_matrix(rpy)
    transform[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return transform


@dataclass(frozen=True)
class UrdfJoint:
    """FK 所需最小关节描述（prismatic 未建模：AUBO E5 全 revolute/fixed）."""

    name: str
    jtype: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass(frozen=True)
class UrdfLinkMeshes:
    """link 名 → collision mesh URI 列表（package:// 原样保留）."""

    name: str
    meshes: Tuple[str, ...]


def parse_urdf(urdf_xml: str) -> Tuple[List[UrdfJoint], List[UrdfLinkMeshes]]:
    """
    解析 URDF 的 joint 链与 collision mesh 清单（visual 忽略）.

    只取 FK 与自身滤除需要的字段；非 mesh 的 collision primitive 忽略
    （本仓 URDF 碰撞几何全为 STL mesh，见 aubo_description/urdf）。
    """
    root = _parse_urdf_xml(urdf_xml)
    joints: List[UrdfJoint] = []
    for elem in root.findall('joint'):
        parent = elem.find('parent')
        child = elem.find('child')
        origin = elem.find('origin')
        axis = elem.find('axis')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] \
            if origin is not None else [0.0, 0.0, 0.0]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] \
            if origin is not None else [0.0, 0.0, 0.0]
        axis_xyz = [float(v) for v in axis.get('xyz', '1 0 0').split()] \
            if axis is not None else [1.0, 0.0, 0.0]
        joints.append(
            UrdfJoint(
                name=elem.get('name', ''),
                jtype=elem.get('type', 'fixed'),
                parent=parent.get('link', '') if parent is not None else '',
                child=child.get('link', '') if child is not None else '',
                origin=_origin_to_matrix(xyz, rpy),
                axis=np.asarray(axis_xyz, dtype=np.float64),
            ))
    links: List[UrdfLinkMeshes] = []
    for elem in root.findall('link'):
        meshes: List[str] = []
        for collision in elem.findall('collision'):
            for mesh in collision.findall('./geometry/mesh'):
                filename = mesh.get('filename', '')
                if filename:
                    meshes.append(filename)
        if meshes:
            links.append(
                UrdfLinkMeshes(
                    name=elem.get('name', ''), meshes=tuple(meshes)))
    return joints, links


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues 绕单位化 axis 旋转 angle → 4x4."""
    unit = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(unit))
    if norm < 1.0e-12:
        return np.eye(4)
    unit = unit / norm
    cross = np.array([
        [0.0, -unit[2], unit[1]],
        [unit[2], 0.0, -unit[0]],
        [-unit[1], unit[0], 0.0],
    ])
    rotation = (
        np.eye(3) + np.sin(angle) * cross +
        (1.0 - np.cos(angle)) * (cross @ cross))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


def fk_link_transforms(
    joints: Sequence[UrdfJoint], positions: Mapping[str, float]
) -> Tuple[Dict[str, np.ndarray], str]:
    """
    Root 系下各 link 的 4x4；返回 (transforms, root_link).

    positions 缺失的活动关节按 0 处理（调用方负责告警）；continuous 与
    revolute 同为绕 axis 旋转。断链时剩余 link 停留缺省（不抛）。
    """
    children = {joint.child for joint in joints}
    roots = [j.parent for j in joints if j.parent not in children]
    root_link = roots[0] if roots else 'base_link'
    transforms: Dict[str, np.ndarray] = {root_link: np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        remaining: List[UrdfJoint] = []
        for joint in pending:
            if joint.parent not in transforms:
                remaining.append(joint)
                continue
            local = joint.origin
            if joint.jtype in ('revolute', 'continuous'):
                angle = float(positions.get(joint.name, 0.0))
                local = local.copy() @ _axis_rotation(joint.axis, angle)
            transforms[joint.child] = transforms[joint.parent] @ local
            progressed = True
        pending = remaining
        if pending and not progressed:
            break
    return transforms, root_link


# ---------------------------------------------------------------- 自身滤除

MeshResolver = Callable[[str], str]
"""package 名 → share 根目录（node 侧经 ament_index 提供）。"""


def load_stl_triangles(path: str) -> np.ndarray:
    """STL → (M,3,3) 三角形。Open3D 延迟加载（纯几何测试可不触本函数）."""
    import open3d as o3d  # noqa: PLC0415（延迟加载；模块顶层零重依赖）

    mesh = o3d.io.read_triangle_mesh(path)
    # triangles 是顶点索引数组，必须 int（float 索引在 vertices[triangles]
    # 处报 arrays used as indices）；vertices 显式 float64。
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    if triangles.size == 0 or vertices.size == 0:
        return np.zeros((0, 3, 3), dtype=np.float64)
    return vertices[triangles]


def _resolve_mesh_uri(uri: str, resolver: MeshResolver) -> Optional[str]:
    """package://<pkg>/rel → share 绝对路径；其余 URI 原样返回."""
    prefix = 'package://'
    if not uri.startswith(prefix):
        return uri
    remainder = uri[len(prefix):]
    package, _, rel = remainder.partition('/')
    if not package or not rel:
        return None
    import os

    path = os.path.join(resolver(package), rel)
    return path if os.path.isfile(path) else None


def collect_self_triangles(
    links: Sequence[UrdfLinkMeshes],
    transforms: Mapping[str, np.ndarray],
    root_link: str,
    base_frame: str,
    resolver: MeshResolver,
    cache: Optional[Mapping[str, np.ndarray]] = None,
) -> np.ndarray:
    """
    各连杆 collision mesh 按 FK 变换到 base_frame 后拼接.

    cache：mesh_path → 原始三角形（node 侧进程级复用，一 mesh 只读一次）。
    base_frame 不是 root 时按 root→base 的 FK 逆统一换系。
    """
    to_base = np.eye(4)
    if base_frame != root_link and base_frame in transforms:
        to_base = np.linalg.inv(transforms[base_frame])
    chunks: List[np.ndarray] = []
    for link in links:
        transform = transforms.get(link.name)
        if transform is None:
            continue
        world = to_base @ transform
        for uri in link.meshes:
            path = _resolve_mesh_uri(uri, resolver)
            if path is None:
                continue
            triangles = (cache or {}).get(path)
            if triangles is None:
                triangles = load_stl_triangles(path)
            if triangles.shape[0] == 0:
                continue
            rotated = triangles @ world[:3, :3].T
            chunks.append(rotated + world[:3, 3][None, None, :])
    if not chunks:
        return np.zeros((0, 3, 3), dtype=np.float64)
    return np.concatenate(chunks, axis=0)


def self_keep_mask(
    points: np.ndarray, triangles: np.ndarray, margin_m: float
) -> np.ndarray:
    """
    True=保留。距机器人 mesh 表面 <= margin 的点剔除.

    用无符号距离而非 SDF：STL 碰撞网格常有开口，SDF 内外判定不可靠；
    无符号距离语义=「贴机器人表面（无论内外）都当自身点」，与官方
    octomap self-filter 的 padding 语义一致。贴臂真枝叶一并滤掉是已知
    双刃（贴臂处臂本就到不了），margin 由部署 yaml 控制。
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    if triangles.shape[0] == 0:
        return np.ones(points.shape[0], dtype=bool)
    import open3d as o3d  # noqa: PLC0415（延迟加载）

    # (M,3,3) 展平重建 TriangleMesh：RaycastingScene::add_triangles 收
    # TriangleMesh 或 (vertices, indices) 对（本仓 Open3D 版本不接受
    # (M,3,3) Tensor）；未焊接顶点不影响距离查询。
    vertices = triangles.reshape(-1, 3)
    faces = np.arange(triangles.shape[0] * 3, dtype=np.uint32).reshape(-1, 3)
    mesh = o3d.t.geometry.TriangleMesh(
        o3d.core.Tensor(vertices.astype(np.float32)),
        o3d.core.Tensor(faces))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)
    distance = scene.compute_distance(
        o3d.core.Tensor(points.astype(np.float32))).numpy()
    return distance > margin_m


# ---------------------------------------------------------------- 体素化

def voxel_centers(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """
    体素化去重 → 各占据体素中心.

    numpy 网格索引（Open3D VoxelGrid 的中心提取 API 跨版本不稳，自写
    更可测）。
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0 or voxel_size_m <= 0.0:
        return np.zeros((0, 3), dtype=np.float64)
    index = np.floor(points / voxel_size_m).astype(np.int64)
    unique = np.unique(index, axis=0)
    return (unique.astype(np.float64) + 0.5) * voxel_size_m


def keep_within_radius(centers: np.ndarray, radius_m: float) -> np.ndarray:
    """True=保留：距 base 原点 3D 距离 <= radius（工作空间裁剪）."""
    centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    return np.linalg.norm(centers, axis=1) <= radius_m


# ---------------------------------------------------------------- 快照组装

@dataclass(frozen=True)
class SnapshotParams:
    """build_snapshot 全链参数（部署 yaml 单源：config/scene_obstacles.yaml）."""

    voxel_size_m: float = 0.06
    self_filter_margin_m: float = 0.05
    capsule_radial_margin_m: float = 0.10
    capsule_axial_margin_m: float = 0.10
    workspace_radius_m: float = 1.5
    max_boxes: int = 3000


@dataclass(frozen=True)
class ObstacleSnapshot:
    """
    快照产物：体素中心（base 系）+ 统计（供日志与测试断言）.

    kept_points：滤除后存留的体素中心数（≈最终 box 数，非源点数）。
    """

    centers: np.ndarray
    voxel_size_m: float
    truncated: bool
    source_points: int
    kept_points: int


def build_snapshot(
    cloud_xyz: np.ndarray,
    capsules: Sequence[CapsuleSpec],
    self_triangles: np.ndarray,
    params: SnapshotParams,
) -> ObstacleSnapshot:
    """
    全链组装（node 与测试共用主入口）：体素→滤除→裁剪→上限.

    滤除阈值=配置余量+体素半对角（√3/2·voxel）：保证每个方块整体
    （含最坏朝向的角点）不切入机器人表面/目标走廊——否则方块角会以
    ~2mm 量级切进工具网格，FCL 判起点接触（09-29 真相机轮实锤）。
    """
    cloud_xyz = np.asarray(cloud_xyz, dtype=np.float64).reshape(-1, 3)
    source = cloud_xyz.shape[0]
    if source == 0:
        return ObstacleSnapshot(
            centers=np.zeros((0, 3)), voxel_size_m=params.voxel_size_m,
            truncated=False, source_points=0, kept_points=0)
    centers = voxel_centers(cloud_xyz, params.voxel_size_m)
    centers = centers[keep_within_radius(centers, params.workspace_radius_m)]
    half_diag_m = 0.5 * np.sqrt(3.0) * params.voxel_size_m
    keep = self_keep_mask(
        centers, self_triangles, params.self_filter_margin_m + half_diag_m)
    keep &= capsule_keep_mask(
        centers, capsules,
        params.capsule_radial_margin_m + half_diag_m,
        params.capsule_axial_margin_m + half_diag_m)
    kept_points = int(np.count_nonzero(keep))
    centers = centers[keep]
    truncated = centers.shape[0] > params.max_boxes
    if truncated:
        centers = centers[:params.max_boxes]
    return ObstacleSnapshot(
        centers=centers, voxel_size_m=params.voxel_size_m,
        truncated=truncated, source_points=source, kept_points=kept_points)
