#!/usr/bin/env python3
"""从本机 TSDF 袋网格 + 工业袋尺寸生成 Gazebo 可用的套袋桃模型。

不调用云端 image-to-3D。输出 visual mesh、碰撞盒、model.sdf。
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
SESSION = (
    ROOT / '_archive/runs/peach_sessions'
    / 'session_20260814_180443_729802' / 'result' / 'tsdf_mesh.ply'
)
OUT = ROOT / 'runs/scene_models/peach_bag'


def _nvidia_status() -> dict:
    """Agent 沙箱默认看不到 /dev/nvidia*；查驱动必须在宿主机上跑。"""
    try:
        import torch
        if torch.cuda.is_available():
            props = torch.cuda.get_device_properties(0)
            return {
                'available': True,
                'name': torch.cuda.get_device_name(0),
                'vram_gib': round(props.total_memory / (1024 ** 3), 1),
                'torch': torch.__version__,
            }
        return {'available': False, 'torch': torch.__version__, 'reason': 'cuda not available'}
    except Exception as exc:  # noqa: BLE001 — 探测失败写进 manifest
        return {'available': False, 'reason': str(exc)}


def _pca_align_z(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    verts = np.asarray(mesh.vertices)
    center = verts.mean(axis=0)
    x = verts - center
    _, _, vt = np.linalg.svd(x, full_matrices=False)
    axis = vt[0]
    if axis[2] < 0:
        axis = -axis
    z = axis / np.linalg.norm(axis)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(tmp, z)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z, x_axis)
    rot = np.column_stack((x_axis, y_axis, z))
    aligned = x @ rot
    out = o3d.geometry.TriangleMesh(mesh)
    out.vertices = o3d.utility.Vector3dVector(aligned)
    out.compute_vertex_normals()
    return out


def _write_sdf(path: Path, visual_uri: str, box: np.ndarray) -> None:
    sx, sy, sz = (float(v) for v in box)
    text = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="peach_bag">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.18</mass>
        <inertia>
          <ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.0002</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{visual_uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    path.write_text(text, encoding='utf-8')


def _write_config(path: Path, name: str) -> None:
    path.write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Bagged peach placeholder for Gazebo (visual from TSDF or primitive).</description>
</model>
""",
        encoding='utf-8')


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    tsdf_dir = OUT / 'from_tsdf'
    prim_dir = OUT / 'from_spec'
    for folder in (tsdf_dir / 'meshes', prim_dir / 'meshes'):
        folder.mkdir(parents=True, exist_ok=True)

    mesh = o3d.io.read_triangle_mesh(str(SESSION))
    if mesh.is_empty():
        raise SystemExit(f'空网格: {SESSION}')
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh = _pca_align_z(mesh)
    aabb = mesh.get_axis_aligned_bounding_box()
    extent = np.asarray(aabb.get_extent())
    visual = mesh.simplify_quadric_decimation(target_number_of_triangles=2500)
    visual.compute_vertex_normals()
    o3d.io.write_triangle_mesh(
        str(tsdf_dir / 'meshes' / 'peach_bag.obj'), visual, write_vertex_normals=True)
    o3d.io.write_triangle_mesh(
        str(tsdf_dir / 'meshes' / 'peach_bag.ply'), visual)
    _write_sdf(tsdf_dir / 'model.sdf', 'model://peach_bag/meshes/peach_bag.obj', extent)
    _write_config(tsdf_dir / 'model.config', 'peach_bag')

    # 工业桃袋平面约 150×180 mm，充果厚度按果径约 80 mm
    spec = np.array([0.15, 0.08, 0.18])
    box = o3d.geometry.TriangleMesh.create_box(spec[0], spec[1], spec[2])
    box.translate(-np.asarray(box.get_axis_aligned_bounding_box().get_center()))
    box.paint_uniform_color([0.72, 0.45, 0.18])
    box.compute_vertex_normals()
    o3d.io.write_triangle_mesh(
        str(prim_dir / 'meshes' / 'peach_bag.obj'), box, write_vertex_normals=True)
    _write_sdf(prim_dir / 'model.sdf', 'model://peach_bag/meshes/peach_bag.obj', spec)
    _write_config(prim_dir / 'model.config', 'peach_bag')

    meta = {
        'source_mesh': str(SESSION),
        'tsdf_aabb_m': extent.tolist(),
        'tsdf_triangles': int(np.asarray(visual.triangles).shape[0]),
        'spec_box_m': spec.tolist(),
        'spec_note': 'peach growing bag 150x180 mm flat, thickness 80 mm when filled (web spec)',
        'nvidia': _nvidia_status(),
        'copy_to_tomato_sim': (
            'Copy from_spec or from_tsdf to '
            'robot_workspaces/combined_ws/src/combined_robot/models/peach_bag/'
        ),
    }
    (OUT / 'manifest.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(meta, indent=2, ensure_ascii=False))
    print(f'wrote {OUT}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
