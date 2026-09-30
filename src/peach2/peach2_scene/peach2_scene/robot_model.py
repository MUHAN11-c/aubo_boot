"""
Robot collision geometry from robot_description (zero ROS).

URDF <collision> shapes become surface samples in each link frame. Link poses are not computed
here: the node looks each link up in TF at the image stamp (robot_state_publisher is the single
FK source), then transforms these samples to base_link.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from typing import Callable
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

MAX_URDF_BYTES = 8 * 1024 * 1024
KIND_MESH = 'mesh'
KIND_BOX = 'box'
KIND_CYLINDER = 'cylinder'
KIND_SPHERE = 'sphere'


@dataclass(frozen=True)
class CollisionShape:
    link: str
    kind: str
    origin: np.ndarray                  # (4, 4) link <- shape
    dims: tuple[float, ...] = ()        # box (x, y, z) | cylinder (radius, length) | sphere (r,)
    filename: str = ''
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


def _safe_root(urdf_xml: str) -> ET.Element:
    # robot_description is trusted in this stack, but still parsed as untrusted input (no DTD).
    if len(urdf_xml) > MAX_URDF_BYTES:
        raise ValueError(f'URDF larger than {MAX_URDF_BYTES} bytes')
    head = urdf_xml[:4096].lower()
    if '<!doctype' in head or '<!entity' in head:
        raise ValueError('URDF contains a DTD / entity declaration')
    return ET.fromstring(urdf_xml)


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if text is None or not text.strip():
        return default
    vals = tuple(float(x) for x in text.split())
    if len(vals) != len(default):
        raise ValueError(f'expected {len(default)} values, got {text!r}')
    return vals


def _origin(elem: ET.Element | None) -> np.ndarray:
    T = np.eye(4)
    if elem is None:
        return T
    xyz = _floats(elem.get('xyz'), (0.0, 0.0, 0.0))
    rpy = _floats(elem.get('rpy'), (0.0, 0.0, 0.0))
    # URDF rpy = fixed-axis roll, pitch, yaw = extrinsic 'xyz'.
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def parse_collision_shapes(urdf_xml: str) -> list[CollisionShape]:
    """All <collision> shapes of all links (visual ignored)."""
    root = _safe_root(urdf_xml)
    shapes: list[CollisionShape] = []
    for link in root.findall('link'):
        name = link.get('name', '')
        for col in link.findall('collision'):
            geom = col.find('geometry')
            if geom is None or len(geom) == 0:
                continue
            g = geom[0]
            origin = _origin(col.find('origin'))
            if g.tag == 'mesh':
                shapes.append(CollisionShape(
                    name, KIND_MESH, origin, filename=g.get('filename', ''),
                    scale=_floats(g.get('scale'), (1.0, 1.0, 1.0))))
            elif g.tag == 'box':
                shapes.append(CollisionShape(name, KIND_BOX, origin,
                                             dims=_floats(g.get('size'), (0.0, 0.0, 0.0))))
            elif g.tag == 'cylinder':
                shapes.append(CollisionShape(name, KIND_CYLINDER, origin,
                                             dims=(float(g.get('radius', '0')),
                                                   float(g.get('length', '0')))))
            elif g.tag == 'sphere':
                shapes.append(CollisionShape(name, KIND_SPHERE, origin,
                                             dims=(float(g.get('radius', '0')),)))
    return shapes


def read_stl(path: str) -> np.ndarray:
    """STL (binary or ASCII) -> (M, 3, 3) triangles."""
    with open(path, 'rb') as f:
        data = f.read()
    # Binary files may also start with "solid": decide by the exact binary size instead.
    if len(data) >= 84:
        n = int(np.frombuffer(data, dtype='<u4', count=1, offset=80)[0])
        if len(data) == 84 + 50 * n:
            rec = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
            return np.frombuffer(data, dtype=rec, count=n, offset=84)['v'].astype(np.float64)
    verts = [line.split()[1:4] for line in data.decode('utf-8', 'replace').splitlines()
             if line.strip().startswith('vertex')]
    if len(verts) % 3:
        raise ValueError(f'malformed ASCII STL: {path}')
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def _box_triangles(size: tuple[float, ...]) -> np.ndarray:
    h = 0.5 * np.asarray(size, dtype=np.float64)
    c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return c[np.asarray(faces)]


def _cylinder_triangles(radius: float, length: float, spacing: float) -> np.ndarray:
    n = max(int(math.ceil(2.0 * math.pi * radius / spacing)), 8)
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    ring = np.column_stack([radius * np.cos(a), radius * np.sin(a), np.zeros(n)])
    lo = ring - [0.0, 0.0, 0.5 * length]
    hi = ring + [0.0, 0.0, 0.5 * length]
    j = (np.arange(n) + 1) % n
    tris = [np.stack([lo, lo[j], hi[j]], axis=1), np.stack([lo, hi[j], hi], axis=1)]
    for cap, z in ((lo, -0.5 * length), (hi, 0.5 * length)):
        centre = np.tile([0.0, 0.0, z], (n, 1))
        tris.append(np.stack([centre, cap, cap[j]], axis=1))
    return np.concatenate(tris, axis=0)


def _sphere_points(radius: float, spacing: float) -> np.ndarray:
    n = max(int(math.ceil(4.0 * math.pi * radius * radius / (spacing * spacing))), 12)
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    theta = math.pi * (1.0 + math.sqrt(5.0)) * i
    return radius * np.column_stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi),
                                     np.cos(phi)])


def sample_triangles(triangles: np.ndarray, spacing: float) -> np.ndarray:
    """
    Deterministic surface samples: each triangle gets a barycentric grid with step <= spacing.

    Every surface point is then within `spacing` of a sample (grid cells are sub-triangles whose
    edges are at most spacing long).
    """
    tri = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    if tri.shape[0] == 0:
        return np.zeros((0, 3))
    edges = np.stack([np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1),
                      np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1),
                      np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1)], axis=1).max(axis=1)
    subdiv = np.maximum(np.ceil(edges / spacing), 1).astype(np.int64)
    out = []
    for n in np.unique(subdiv):
        ij = np.array([(i, j) for i in range(n + 1) for j in range(n + 1 - i)], dtype=np.float64)
        bary = np.column_stack([1.0 - (ij[:, 0] + ij[:, 1]) / n, ij[:, 0] / n, ij[:, 1] / n])
        out.append(np.einsum('pk,tkd->tpd', bary, tri[subdiv == n]).reshape(-1, 3))
    pts = np.vstack(out)
    # Shared edges/vertices produce duplicates; dedupe on a grid well below the spacing.
    q = np.round(pts / (0.25 * spacing)).astype(np.int64)
    _, first = np.unique(q, axis=0, return_index=True)
    return pts[np.sort(first)]


def shape_samples(shape: CollisionShape, spacing: float,
                  resolve: Callable[[str], str | None]) -> np.ndarray:
    """Surface samples of one shape in its link frame. Raises OSError for an unresolved mesh."""
    if shape.kind == KIND_MESH:
        path = resolve(shape.filename)
        if path is None or not os.path.isfile(path):
            raise OSError(f'collision mesh not found: {shape.filename}')
        local = sample_triangles(read_stl(path) * np.asarray(shape.scale), spacing)
    elif shape.kind == KIND_BOX:
        local = sample_triangles(_box_triangles(shape.dims), spacing)
    elif shape.kind == KIND_CYLINDER:
        local = sample_triangles(_cylinder_triangles(shape.dims[0], shape.dims[1], spacing),
                                 spacing)
    else:
        local = _sphere_points(shape.dims[0], spacing)
    return local @ shape.origin[:3, :3].T + shape.origin[:3, 3]


def resolve_mesh_uri(uri: str, share_dir: Callable[[str], str]) -> str | None:
    """package://pkg/rel -> <share(pkg)>/rel; file:///abs -> /abs; plain paths unchanged."""
    if uri.startswith('package://'):
        pkg, _, rel = uri[len('package://'):].partition('/')
        if not pkg or not rel:
            return None
        try:
            return os.path.join(share_dir(pkg), rel)
        except (LookupError, ValueError):
            return None
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return uri


def link_samples(urdf_xml: str, spacing: float,
                 resolve: Callable[[str], str | None]) -> tuple[dict[str, np.ndarray], list[str]]:
    """
    Per-link surface samples (link frame). Returns (samples, problems).

    A shape that cannot be loaded is skipped and reported: the caller must surface it, because
    missing robot geometry turns robot points into false obstacles.
    """
    per_link: dict[str, list[np.ndarray]] = {}
    problems: list[str] = []
    for shape in parse_collision_shapes(urdf_xml):
        try:
            pts = shape_samples(shape, spacing, resolve)
        except (OSError, ValueError) as exc:
            problems.append(f'{shape.link}: {exc}')
            continue
        if pts.shape[0]:
            per_link.setdefault(shape.link, []).append(pts)
    return {k: np.vstack(v) for k, v in per_link.items()}, problems
