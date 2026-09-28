# 功能：STEP 总装解剖（装配树+包围盒+全局位姿 → JSON/缩进树）。
"""STEP 总装解剖：装配树 + 零件包围盒 + 全局位姿 → JSON / 缩进树（末端分组人审用）。

用法（FreeCAD snap 无头；snap 的 /tmp 是私有目录，输出必须落 $HOME）：

  STEP_PATH=/path/to/asm.STEP \
  OUT_JSON=/home/mu/.../asm.json \
  OUT_TREE=/home/mu/.../asm.txt \
  freecad.cmd src/aubo_description/scripts/step_inspect.py

- STEP 零件名可能是 GBK 乱码：分组按树结构 + 包围盒，不靠 Label。
- 关闭 UseLinkGroup（App::Link 展开为经典 App::Part 树），叶子
  Part::Feature 的 getGlobalPlacement 即含装配链累积位姿。
- 单位 mm（FreeCAD 内部）。
"""

import json
import os

import FreeCAD as App

STEP_PATH = os.environ['STEP_PATH']
OUT_JSON = os.environ['OUT_JSON']
OUT_TREE = os.environ.get('OUT_TREE', '')

# 经典树导入：避免 App::Link 模板位姿与实例位姿分离（叶子全局位姿直接可用）
_params = App.ParamGet('User parameter:BaseApp/Preferences/Mod/Import/hSTEP')
_params.SetInt('UseLinkGroup', 0)
_params.SetBool('UseLinkGroup', False)

doc = App.newDocument('asm')
import Import  # noqa: E402  FreeCAD 顶层模块
Import.insert(STEP_PATH, 'asm')


def _r3(value):
    return round(float(value), 3)


def node_of(obj):
    entry = {
        'name': obj.Name,
        'label': obj.Label,
        'type': obj.TypeId,
        'children': [],
        'parent': None,
    }
    shape = getattr(obj, 'Shape', None)
    if shape is not None and not shape.isNull():
        bb = shape.BoundBox
        entry['bbox_mm'] = {
            'min': [_r3(bb.XMin), _r3(bb.YMin), _r3(bb.ZMin)],
            'max': [_r3(bb.XMax), _r3(bb.YMax), _r3(bb.ZMax)],
            'size': [_r3(bb.XLength), _r3(bb.YLength), _r3(bb.ZLength)],
        }
        try:
            entry['volume_mm3'] = _r3(shape.Volume / 1000.0) * 1000.0
        except Exception:
            entry['volume_mm3'] = -1
        try:
            entry['solids'] = len(shape.Solids)
        except Exception:
            entry['solids'] = -1
    try:
        gp = obj.getGlobalPlacement()
        matrix = gp.Matrix
        entry['global'] = {
            'base_mm': [_r3(gp.Base.x), _r3(gp.Base.y), _r3(gp.Base.z)],
            'matrix': [round(matrix[i], 6) for i in range(16)],
        }
    except Exception:
        pass
    return entry


nodes = {obj.Name: node_of(obj) for obj in doc.Objects}

for obj in doc.Objects:
    kids = list(getattr(obj, 'Group', []) or [])
    for kid in kids:
        if kid.Name in nodes:
            nodes[kid.Name]['parent'] = obj.Name
            nodes[obj.Name]['children'].append(kid.Name)

roots = [name for name, entry in nodes.items() if entry['parent'] is None]


def write_tree(path):
    lines = []

    def rec(name, depth):
        entry = nodes[name]
        bbox = entry.get('bbox_mm')
        geo = ''
        if bbox:
            size = bbox['size']
            geo = f"  bbox≈{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}mm"
            base = entry.get('global', {}).get('base_mm')
            if base:
                geo += f"  base=({base[0]:.0f},{base[1]:.0f},{base[2]:.0f})"
            geo += f"  solids={entry.get('solids')}"
        lines.append('  ' * depth + f"[{entry['type'].split('::')[-1]}] "
                     f"{entry['label']} ({entry['name']}){geo}")
        for child in entry['children']:
            rec(child, depth + 1)

    for root in roots:
        rec(root, 0)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')


with open(OUT_JSON, 'w', encoding='utf-8') as fh:
    json.dump({'step': STEP_PATH, 'objects': len(doc.Objects), 'roots': roots,
               'nodes': nodes}, fh, ensure_ascii=False, indent=1)
if OUT_TREE:
    write_tree(OUT_TREE)
print('WROTE', OUT_JSON, 'objects:', len(doc.Objects))
