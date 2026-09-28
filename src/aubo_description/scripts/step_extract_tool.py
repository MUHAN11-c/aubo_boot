# 功能：STEP 总装 → 末端工具网格（法兰坐标系，米）+ 法兰帧测量报告（stdout JSON）。
"""STEP 总装 → 末端工具网格（法兰坐标系，米）+ 法兰帧测量报告（stdout JSON）。

用法（FreeCAD snap 无头；两个输出 STL 路径由环境变量直传，所在目录需预先存在）：
  STEP_PATH=... TOOL_STL=.../tool_raw.stl REF_STL=.../wrist_ref.stl \
  EE_LABEL=咬合式末端执行器 \
  freecad.cmd src/aubo_description/scripts/step_extract_tool.py > extract_report.json

约定：
- 法兰帧假设 = CAD 全局系（原点在法兰面心、+Z 沿法兰轴朝外）；脚本用
  i5-末端 法兰圆柱/平面实测校验，偏差大则人审后用 FLANGE_OVERRIDE_XYZ_RPY 覆盖。
- 工具网格 = 末端执行器子树 + 快换盘付盘（工具侧）；臂本体/主盘/相机族排除。
- 另出 wrist_ref.stl（i5-末端+主盘+相机族）供「CAD vs URDF link6+快换盘」叠对验收。
- stdout 末行是 REPORT_JSON:{}，整体可重定向保存。
"""

import json
import os

STEP_PATH = os.environ['STEP_PATH']
TOOL_STL = os.environ['TOOL_STL']
REF_STL = os.environ['REF_STL']
EE_LABEL = os.environ['EE_LABEL']
EXTRA_TOOL_LABELS = os.environ.get('EXTRA_TOOL_LABELS', 'STW-X20DYF-快换盘付盘')
WRIST_REF_LABELS = os.environ.get(
    'WRIST_REF_LABELS', 'i5-末端,STW-X20DY快换盘主盘,相机安装板,PS800-E1相机,装载滤片组件')
# 法兰帧覆盖（测量偏差可接受时人审后可平移/旋转，xyz_mm,rpy_deg）
FLANGE_OVERRIDE = os.environ.get('FLANGE_OVERRIDE_XYZ_RPY', '')

import FreeCAD as App
import Mesh

_params = App.ParamGet('User parameter:BaseApp/Preferences/Mod/Import/hSTEP')
_params.SetInt('UseLinkGroup', 0)
_params.SetBool('UseLinkGroup', False)

doc = App.newDocument('asm')
import Import  # noqa: E402  FreeCAD 顶层模块
Import.insert(STEP_PATH, 'asm')


def label_matches(label, templates):
    for tpl in templates:
        tpl = tpl.strip()
        if not tpl:
            continue
        if label == tpl or label.startswith(tpl):
            return True
    return False


def split_templates(raw):
    return [t for t in raw.split(',') if t.strip()]


def find_assembly_root():
    # 总装根 = 不被任何其他 App::Part 包含的最顶层 App::Part（孩子数最多者兜底）
    parts = [o for o in doc.Objects if o.TypeId == 'App::Part']
    contained = set()
    for part in parts:
        stack = list(getattr(part, 'Group', []) or [])
        while stack:
            child = stack.pop()
            if child.TypeId == 'App::Part':
                contained.add(child.Name)
            stack.extend(getattr(child, 'Group', []) or [])
    top = [p for p in parts if p.Name not in contained]
    if top:
        return max(top, key=lambda p: len(getattr(p, 'Group', []) or []))
    return max(parts, key=lambda p: len(getattr(p, 'Group', []) or []))


def leaves_of(root_obj):
    out = []
    stack = list(getattr(root_obj, 'Group', []) or [])
    while stack:
        obj = stack.pop()
        if hasattr(obj, 'Group'):
            stack.extend(obj.Group)
        elif obj.TypeId == 'Part::Feature':
            out.append(obj)
    return out


def shape_in_frame(obj, frame_inv):
    shp = obj.Shape.copy()
    shp.Placement = frame_inv.multiply(obj.getGlobalPlacement())
    return shp


root = find_assembly_root()
root_children = list(getattr(root, 'Group', []) or [])
child_labels = [o.Label for o in root_children]

# ---------- 法兰帧测量：i5-末端 圆柱轴 + 顶面 ----------
flange_parts = [o for o in root_children
                if label_matches(o.Label, split_templates('i5-末端'))]
flange_shapes = []
for part in flange_parts:
    for leaf in leaves_of(part):
        try:
            shp = leaf.Shape.copy()
            shp.Placement = leaf.getGlobalPlacement()
            flange_shapes.append(shp)
        except Exception as exc:
            print('FLANGE-SKIP', leaf.Label, type(exc).__name__, exc, flush=True)
print('FLANGE-DEBUG parts:', len(flange_parts),
      'leaves:', sum(len(leaves_of(p)) for p in flange_parts),
      'shapes:', len(flange_shapes), flush=True)

axis_hits, face_top = [], -1e9
type_seen = {}
for shp in flange_shapes:
    for face in shp.Faces:
        surf = face.Surface
        type_seen[surf.TypeId] = type_seen.get(surf.TypeId, 0) + 1
        is_cyl = 'Cylinder' in surf.TypeId
        is_plane = 'Plane' in surf.TypeId
        if is_cyl:
            axis = surf.Axis
            if abs(axis.z) > 0.99 and 8.0 < surf.Radius < 80.0:
                center = surf.Center
                axis_hits.append((surf.Radius, center.x, center.y, face.Area))
        elif is_plane:
            try:
                normal = face.normalAt(0, 0)
            except Exception:
                continue
            if normal.z > 0.95 and face.Area > 300.0:
                face_top = max(face_top, face.BoundBox.ZMax)

if not axis_hits:
    print('FLANGE-DEBUG faces by surface type:', type_seen, flush=True)
    print('FAIL no vertical flange cylinder found', flush=True)
    raise SystemExit(2)

wsum = sum(h[3] for h in axis_hits)
ax_x = sum(h[1] * h[3] for h in axis_hits) / wsum
ax_y = sum(h[2] * h[3] for h in axis_hits) / wsum

if FLANGE_OVERRIDE:
    vals = [float(v) for v in FLANGE_OVERRIDE.split(',')]
    flange = App.Placement(App.Vector(*vals[:3]), App.Rotation(*vals[3:6]))
else:
    flange = App.Placement(App.Vector(0, 0, 0), App.Rotation(0, 0, 0, 1))

flange_inv = flange.inverse()

# ---------- 组装工具叶子集合 ----------
tool_parts = [o for o in root_children
              if label_matches(o.Label, [EE_LABEL] + split_templates(EXTRA_TOOL_LABELS))]
if not tool_parts:
    print('FAIL EE label not found. root children:', child_labels, flush=True)
    raise SystemExit(2)
tool_leaves = []
for part in tool_parts:
    tool_leaves.extend(leaves_of(part))

ref_parts = [o for o in root_children
             if label_matches(o.Label, split_templates(WRIST_REF_LABELS))]
ref_leaves = []
for part in ref_parts:
    ref_leaves.extend(leaves_of(part))


def build_mesh(leaves, tolerance=0.15):
    triangles = []
    for leaf in leaves:
        try:
            shp = shape_in_frame(leaf, flange_inv)
            if shp.isNull():
                continue
            verts, facets = shp.tessellate(tolerance)
            triangles.extend(
                (verts[a], verts[b], verts[c]) for a, b, c in facets)
        except Exception as exc:
            print('skip', leaf.Label, exc)
    mesh = Mesh.Mesh(triangles)
    scale = App.Matrix(0.001, 0, 0, 0,
                       0, 0.001, 0, 0,
                       0, 0, 0.001, 0,
                       0, 0, 0, 1)
    mesh.transform(scale)
    return mesh


tool_mesh = build_mesh(tool_leaves)
tool_mesh.write(TOOL_STL)
bb = tool_mesh.BoundBox
ref_mesh = build_mesh(ref_leaves, tolerance=0.5)
ref_mesh.write(REF_STL)
ref_bb = ref_mesh.BoundBox

per_part = {}
for part in tool_parts:
    for leaf in leaves_of(part):
        try:
            shp = shape_in_frame(leaf, flange_inv)
            if shp.isNull():
                continue
            lb = shp.BoundBox
            try:
                vol = round(shp.Volume * 1e-9, 6)
            except Exception:
                vol = -1.0
            per_part[leaf.Label] = {
                'group': part.Label,
                'min': [round(lb.XMin * 0.001, 4), round(lb.YMin * 0.001, 4),
                        round(lb.ZMin * 0.001, 4)],
                'max': [round(lb.XMax * 0.001, 4), round(lb.YMax * 0.001, 4),
                        round(lb.ZMax * 0.001, 4)],
                'volume_m3': vol}
        except Exception:
            pass

report = {
    'step': STEP_PATH,
    'ee_label': EE_LABEL,
    'flange_measure': {
        'axis_xy_mm': [round(ax_x, 3), round(ax_y, 3)],
        'flange_face_z_mm': round(face_top, 3),
        'cylinders': [{'r': round(h[0], 2), 'x': round(h[1], 2),
                       'y': round(h[2], 2), 'area': round(h[3], 1)}
                      for h in sorted(axis_hits, key=lambda h: -h[3])[:8]],
    },
    'tool_parts': [p.Label for p in tool_parts],
    'tool_leaf_count': len(tool_leaves),
    'tool_triangles': tool_mesh.CountFacets,
    'tool_bbox_m': {'min': [round(bb.XMin, 4), round(bb.YMin, 4), round(bb.ZMin, 4)],
                    'max': [round(bb.XMax, 4), round(bb.YMax, 4), round(bb.ZMax, 4)]},
    'ref_bbox_m': {'min': [round(ref_bb.XMin, 4), round(ref_bb.YMin, 4), round(ref_bb.ZMin, 4)],
                   'max': [round(ref_bb.XMax, 4), round(ref_bb.YMax, 4), round(ref_bb.ZMax, 4)]},
    'per_part_bbox_m': per_part,
}
print('REPORT_JSON:' + json.dumps(report, ensure_ascii=False))
