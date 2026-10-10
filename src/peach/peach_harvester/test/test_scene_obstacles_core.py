"""
scene_obstacles 纯核单测（零 ROS）：胶囊滤除/URDF-FK/体素化/全链快照.

自身滤除走合成 mesh（Open3D RaycastingScene 真跑，不 mock 几何）。
"""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.scene_obstacles.core import (
    build_snapshot,
    capsule_keep_mask,
    CapsuleSpec,
    collect_self_triangles,
    fk_link_transforms,
    keep_within_radius,
    parse_urdf,
    point_segment_distance,
    self_keep_mask,
    SnapshotParams,
    voxel_centers,
)

import pytest

try:  # open3d 惰性可用性探测（缺库环境逐测跳过，不影响同批其他文件收集）
    import open3d as _o3d  # noqa: F401
    _HAS_OPEN3D = True
except ImportError:  # pragma: no cover（colcon 系统环境常缺 open3d）
    _HAS_OPEN3D = False

pytestmark_open3d = pytest.mark.skipif(
    not _HAS_OPEN3D, reason='open3d 仅 venv 环境可用')


# ---------------------------------------------------------------- 点到线段

class TestPointSegmentDistance:

    def test_axis_aligned_segment(self):
        points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0],
                           [2.0, 0.0, 0.0], [0.5, 0.3, 0.0]])
        distance = point_segment_distance(
            points, np.zeros(3), np.array([1.0, 0.0, 0.0]))
        assert np.allclose(distance, [0.0, 0.0, 1.0, 0.3])

    def test_degenerate_segment_falls_back_to_sphere(self):
        distance = point_segment_distance(
            np.array([[1.0, 0.0, 0.0]]), np.zeros(3), np.zeros(3))
        assert np.allclose(distance, [1.0])


# ---------------------------------------------------------------- 胶囊滤除

class TestCapsuleKeepMask:

    def test_interior_and_endcap_removed(self):
        # 袋胶囊：bottom=(0,0,0) neck=(0,0,1) 半径 0.05；余量径向 0.1 轴向 0.1
        capsule = CapsuleSpec(
            bottom=(0.0, 0.0, 0.0), neck=(0.0, 0.0, 1.0), radius=0.05)
        points = np.array([
            [0.0, 0.0, 0.5],    # 轴心：必删
            [0.10, 0.0, 0.5],   # 距轴 0.10 < 0.15（半径0.05+余量0.10）：删
            [0.16, 0.0, 0.5],   # 距轴 0.16 > 0.15：留
            [0.0, 0.0, 1.08],   # 延长段（neck+0.08<0.1）内：删
            [0.0, 0.0, 1.30],   # 端球外（延长端 1.1 + 端球 0.15）：留
            [0.0, 0.0, -0.05],  # bottom 端球内：删
            [2.0, 2.0, 2.0],    # 远点：留
        ])
        keep = capsule_keep_mask(points, [capsule], 0.10, 0.10)
        assert keep.tolist() == [False, False, True, False, True, False, True]

    def test_degenerate_capsule_filters_as_ball(self):
        capsule = CapsuleSpec(
            bottom=(0.0, 0.0, 1.0), neck=(0.0, 0.0, 1.0), radius=0.05)
        points = np.array([[0.0, 0.0, 1.05], [0.0, 0.0, 2.0]])
        keep = capsule_keep_mask(points, [capsule], 0.05, 0.10)
        assert keep.tolist() == [False, True]

    def test_multiple_capsules_union(self):
        first = CapsuleSpec(
            bottom=(0.0, 0.0, 0.0), neck=(0.0, 0.0, 1.0), radius=0.05)
        second = CapsuleSpec(
            bottom=(3.0, 0.0, 0.0), neck=(3.0, 0.0, 1.0), radius=0.05)
        points = np.array([[0.0, 0.0, 0.5], [3.0, 0.0, 0.5], [6.0, 0.0, 0.5]])
        keep = capsule_keep_mask(points, [first, second], 0.10, 0.10)
        assert keep.tolist() == [False, False, True]

    def test_empty_capsules_keeps_all(self):
        points = np.array([[0.0, 0.0, 0.0]])
        assert capsule_keep_mask(points, [], 0.1, 0.1).tolist() == [True]


# ---------------------------------------------------------------- URDF/FK

_MINI_URDF = """\
<robot name="mini">
  <link name="world"/>
  <link name="base">
    <collision><geometry>
      <mesh filename="package://fake/collision/base.stl"/>
    </geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry>
      <mesh filename="package://fake/collision/tip.stl"/>
    </geometry></collision>
  </link>
  <joint name="world_base" type="fixed">
    <parent link="world"/><child link="base"/><origin xyz="0 0 0.1"/>
  </joint>
  <joint name="tip_joint" type="revolute">
    <parent link="base"/><child link="tip"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
</robot>
"""


class TestParseUrdfAndFk:

    def test_parse_joints_and_meshes(self):
        joints, links = parse_urdf(_MINI_URDF)
        assert [j.name for j in joints] == ['world_base', 'tip_joint']
        assert joints[1].jtype == 'revolute'
        assert np.allclose(joints[1].axis, [0.0, 1.0, 0.0])
        assert {link.name for link in links} == {'base', 'tip'}
        tip = next(link for link in links if link.name == 'tip')
        assert tip.meshes == ('package://fake/collision/tip.stl',)

    def test_fk_zero_position_and_rotation(self):
        joints, _ = parse_urdf(_MINI_URDF)
        transforms, root = fk_link_transforms(joints, {'tip_joint': 0.0})
        assert root == 'world'
        assert np.allclose(transforms['base'][:3, 3], [0.0, 0.0, 0.1])
        # tip_joint=π/2 绕 +Y：轴过 joint 原点，tip 系原点不动、姿态旋转
        # （URDF 语义：子系原点=关节系原点，旋转不动原点），x̂ 指向 −Z
        transforms, _ = fk_link_transforms(joints, {'tip_joint': np.pi / 2})
        tip = transforms['tip']
        assert np.isclose(tip[0, 3], 0.5, atol=1e-9)
        assert np.allclose(
            tip[:3, :3] @ np.array([1.0, 0.0, 0.0]),
            [0.0, 0.0, -1.0], atol=1e-9)

    def test_missing_joint_treated_as_zero(self):
        joints, _ = parse_urdf(_MINI_URDF)
        transforms, _ = fk_link_transforms(joints, {})
        assert 'tip' in transforms

    def test_dtd_rejected(self):
        with pytest.raises(ValueError):
            parse_urdf('<!DOCTYPE robot [<!ENTITY a "b">]>'
                       '<robot name="x"><link name="l"/></robot>')


# ---------------------------------------------------------------- 自身滤除

def _plane_triangles(z: float = 0.0, half: float = 0.5) -> np.ndarray:
    """z=z 平面正方形两三角形（合成机器人表面）."""
    a = [-half, -half, z]
    b = [half, -half, z]
    c = [half, half, z]
    d = [-half, half, z]
    return np.array([[a, b, c], [a, c, d]], dtype=np.float64)


class TestSelfKeepMask:

    @pytestmark_open3d
    def test_surface_and_near_points_removed_far_kept(self):
        triangles = _plane_triangles()
        points = np.array([
            [0.0, 0.0, 0.0],     # 表面上：删
            [0.0, 0.0, 0.03],    # 距表面 0.03 < 0.05：删
            [0.0, 0.0, 0.20],    # 远：留
            [9.0, 9.0, 9.0],     # 平面外远点：留
        ])
        keep = self_keep_mask(points, triangles, 0.05)
        assert keep.tolist() == [False, False, True, True]

    def test_no_triangles_keeps_all(self):
        keep = self_keep_mask(np.zeros((3, 3)), np.zeros((0, 3, 3)), 0.05)
        assert keep.tolist() == [True, True, True]


class TestCollectSelfTriangles:

    def test_transform_applied_to_base_frame(self, tmp_path):
        joints, links = parse_urdf(_MINI_URDF)
        transforms, root = fk_link_transforms(joints, {'tip_joint': 0.0})
        # resolver 须返回真实目录（core 对 mesh URI 做存在性检查）
        share = tmp_path / 'fake'
        (share / 'collision').mkdir(parents=True)
        for name in ('base.stl', 'tip.stl'):
            (share / 'collision' / name).write_bytes(b'')
        resolver = lambda _pkg: str(share)  # noqa: E731
        cache = {str(share / 'collision' / 'base.stl'): _plane_triangles(0.0),
                 str(share / 'collision' / 'tip.stl'): _plane_triangles(0.0)}
        # root=world、base 高 0.1：base 系下 tip mesh（tip 系原点 x=0.5）
        # 应在 (0.5, 0, 0)——z=0 证明 to_base 已扣掉 base 的 0.1 抬升
        triangles = collect_self_triangles(
            links, transforms, root, 'base', resolver, cache=cache)
        tip_chunk = triangles[2:]
        assert tip_chunk.shape[0] == 2
        assert np.isclose(tip_chunk[:, :, 0].mean(), 0.5, atol=1e-9)
        assert np.isclose(tip_chunk[:, :, 2].mean(), 0.0, atol=1e-9)


# ---------------------------------------------------------------- 体素化

class TestVoxelAndRadius:

    def test_voxel_centers_dedupe_and_center(self):
        points = np.array([
            [0.01, 0.01, 0.01], [0.02, 0.02, 0.02],   # 同一体素
            [0.07, 0.01, 0.01],                        # 相邻体素
        ])
        centers = voxel_centers(points, 0.06)
        assert centers.shape == (2, 3)
        assert np.allclose(centers[0], [0.03, 0.03, 0.03])
        assert np.allclose(centers[1], [0.09, 0.03, 0.03])

    def test_keep_within_radius(self):
        centers = np.array([[0.1, 0.0, 0.0], [2.0, 0.0, 0.0]])
        assert keep_within_radius(centers, 1.5).tolist() == [True, False]


# ---------------------------------------------------------------- 全链

class TestBuildSnapshot:

    @pytestmark_open3d
    def test_chain_filters_self_and_capsule_then_voxelize(self):
        # 机器人=原点平面；袋胶囊在 (2,0,0)-(2,0,1)；远枝点在 (-1,0,0)
        self_triangles = _plane_triangles(half=3.0)
        capsule = CapsuleSpec(
            bottom=(2.0, 0.0, 0.0), neck=(2.0, 0.0, 1.0), radius=0.05)
        points = np.array([
            [0.0, 0.0, 0.05],    # 贴机器人表面：自身滤除删
            [2.0, 0.0, 0.5],     # 袋内：胶囊滤除删
            [-1.0, 0.0, 0.5],    # 远枝：保留成体素
            [0.5, 0.5, 0.5],     # 场景点：保留
        ])
        snapshot = build_snapshot(
            points, [capsule], self_triangles,
            SnapshotParams(voxel_size_m=0.06, self_filter_margin_m=0.05,
                           capsule_radial_margin_m=0.10,
                           capsule_axial_margin_m=0.10,
                           workspace_radius_m=1.5, max_boxes=3000))
        assert snapshot.source_points == 4
        assert snapshot.kept_points == 2
        assert not snapshot.truncated
        # 两个保留点相距 >0.06 → 两个体素；均在 1.5m 半径内
        assert snapshot.centers.shape[0] == 2

    def test_max_boxes_truncates(self):
        grid = np.random.default_rng(7).uniform(
            -0.8, 0.8, size=(5000, 3))
        snapshot = build_snapshot(
            grid, [], np.zeros((0, 3, 3)),
            SnapshotParams(voxel_size_m=0.02, self_filter_margin_m=0.0,
                           capsule_radial_margin_m=0.0,
                           capsule_axial_margin_m=0.0,
                           workspace_radius_m=1.5, max_boxes=100))
        assert snapshot.truncated
        assert snapshot.centers.shape[0] == 100

    def test_empty_cloud(self):
        snapshot = build_snapshot(
            np.zeros((0, 3)), [], np.zeros((0, 3, 3)), SnapshotParams())
        assert snapshot.centers.shape[0] == 0
        assert snapshot.source_points == 0

    def test_workspace_clip_removes_far_points(self):
        points = np.array([[10.0, 0.0, 0.0], [0.2, 0.0, 0.0]])
        snapshot = build_snapshot(
            points, [], np.zeros((0, 3, 3)),
            SnapshotParams(workspace_radius_m=1.5))
        assert snapshot.centers.shape[0] == 1
