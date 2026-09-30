import os
import struct

import numpy as np
from peach2_scene import robot_model as rm
import pytest
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))


def _aubo_description() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('aubo_description')
    except (ImportError, LookupError):
        return os.path.normpath(os.path.join(HERE, '..', '..', '..', 'aubo_description'))


AUBO_DESCRIPTION = _aubo_description()

URDF = """<?xml version="1.0"?>
<robot name="r">
  <link name="base_link">
    <collision><origin xyz="0 0 0.1" rpy="0 0 0"/><geometry><box size="0.2 0.1 0.05"/></geometry>
    </collision>
    <visual><geometry><sphere radius="9"/></geometry></visual>
  </link>
  <link name="arm">
    <collision><origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
      <geometry><cylinder radius="0.03" length="0.2"/></geometry></collision>
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
    <collision><geometry><mesh filename="package://fake_pkg/tri.stl" scale="2 2 2"/></geometry>
    </collision>
  </link>
  <link name="empty"/>
</robot>
"""


def _write_binary_stl(path: str, triangles: np.ndarray, header: bytes = b'solid fake') -> None:
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b' '))
        f.write(struct.pack('<I', triangles.shape[0]))
        for tri in triangles:
            f.write(struct.pack('<3f', 0.0, 0.0, 1.0))
            f.write(struct.pack('<9f', *tri.reshape(-1)))
            f.write(struct.pack('<H', 0))


def _max_gap(samples: np.ndarray, surface: np.ndarray) -> float:
    return float(cKDTree(samples).query(surface)[0].max())


def test_parse_collision_shapes_ignores_visual_and_reads_origin():
    shapes = rm.parse_collision_shapes(URDF)
    kinds = [(s.link, s.kind) for s in shapes]
    assert kinds == [('base_link', 'box'), ('arm', 'cylinder'), ('arm', 'sphere'), ('arm', 'mesh')]
    assert np.allclose(shapes[0].origin[:3, 3], [0.0, 0.0, 0.1])
    # rpy roll 90 deg maps link +Z onto -Y.
    assert np.allclose(shapes[1].origin[:3, :3] @ [0, 0, 1], [0, -1, 0], atol=1e-9)
    assert shapes[3].scale == (2.0, 2.0, 2.0)
    assert shapes[3].filename == 'package://fake_pkg/tri.stl'


def test_dtd_and_oversize_rejected():
    with pytest.raises(ValueError):
        rm.parse_collision_shapes('<?xml version="1.0"?><!DOCTYPE r [<!ENTITY x "y">]><robot/>')
    with pytest.raises(ValueError):
        rm.parse_collision_shapes('<robot>' + ' ' * rm.MAX_URDF_BYTES + '</robot>')


def test_read_stl_binary_with_solid_header_and_ascii(tmp_path):
    tri = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 1], [1, 0, 1], [0, 1, 1]]],
                   dtype=np.float64)
    binary = str(tmp_path / 'b.stl')
    _write_binary_stl(binary, tri)
    assert np.allclose(rm.read_stl(binary), tri)
    ascii_path = str(tmp_path / 'a.stl')
    with open(ascii_path, 'w', encoding='utf-8') as f:
        f.write('solid a\n')
        for t in tri:
            f.write(' facet normal 0 0 1\n  outer loop\n')
            for v in t:
                f.write(f'   vertex {v[0]} {v[1]} {v[2]}\n')
            f.write('  endloop\n endfacet\n')
        f.write('endsolid a\n')
    assert np.allclose(rm.read_stl(ascii_path), tri)


def test_sample_triangles_gap_bounded():
    tri = np.array([[[0, 0, 0], [0.3, 0, 0], [0, 0.1, 0]]], dtype=np.float64)
    samples = rm.sample_triangles(tri, 0.01)
    rng = np.random.default_rng(0)
    ab = rng.random((4000, 2))
    ab = np.where(ab.sum(axis=1, keepdims=True) > 1.0, 1.0 - ab, ab)
    surface = ab[:, :1] * tri[0, 1] + ab[:, 1:] * tri[0, 2]
    assert _max_gap(samples, surface) <= 0.01
    assert len(np.unique(np.round(samples / 1e-6), axis=0)) == len(samples)


def test_link_samples_primitives_and_mesh(tmp_path):
    os.makedirs(tmp_path / 'fake_pkg')
    _write_binary_stl(str(tmp_path / 'fake_pkg' / 'tri.stl'),
                      np.array([[[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0]]], dtype=np.float64))
    samples, problems = rm.link_samples(
        URDF, 0.01, lambda uri: rm.resolve_mesh_uri(uri, lambda pkg: str(tmp_path / pkg)))
    assert problems == []
    assert set(samples) == {'base_link', 'arm'}
    base = samples['base_link']
    assert np.allclose(base.min(axis=0), [-0.1, -0.05, 0.075])
    assert np.allclose(base.max(axis=0), [0.1, 0.05, 0.125])
    arm = samples['arm']
    # scaled mesh reaches 0.1 m; the cylinder (rolled onto Y) reaches +-0.1 m on Y.
    assert arm[:, 0].max() == pytest.approx(0.1, abs=1e-6)
    assert arm[:, 1].min() == pytest.approx(-0.1, abs=1e-3)


def test_missing_mesh_reported_not_raised():
    samples, problems = rm.link_samples(URDF, 0.02, lambda uri: None)
    assert 'arm' in samples and len(problems) == 1 and 'tri.stl' in problems[0]


def test_resolve_mesh_uri():
    share = {'pkg': '/opt/share/pkg'}.__getitem__
    assert rm.resolve_mesh_uri('package://pkg/m/a.stl', share) == '/opt/share/pkg/m/a.stl'
    assert rm.resolve_mesh_uri('package://nope/a.stl', share) is None
    assert rm.resolve_mesh_uri('package://pkg', share) is None
    assert rm.resolve_mesh_uri('file:///abs/a.stl', share) == '/abs/a.stl'


def test_real_aubo_urdf_samples_every_collision_link():
    with open(os.path.join(AUBO_DESCRIPTION, 'urdf', 'aubo_e5.urdf'), encoding='utf-8') as f:
        urdf = f.read()
    samples, problems = rm.link_samples(
        urdf, 0.01, lambda uri: rm.resolve_mesh_uri(
            uri, lambda pkg: AUBO_DESCRIPTION if pkg == 'aubo_description' else ''))
    assert problems == []
    expect = {'base_link', 'shoulder_Link', 'upperArm_Link', 'foreArm_Link', 'wrist1_Link',
              'wrist2_Link', 'wrist3_Link'}
    assert expect <= set(samples)
    total = sum(v.shape[0] for v in samples.values())
    assert 1000 < total < 2_000_000
