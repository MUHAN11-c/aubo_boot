# 功能：Blender 无头减面：tool_raw.stl → visual / collision 双档 STL（米，法兰系不变）。
"""Blender 无头减面：tool_raw.stl → visual / collision 双档 STL（米，法兰系不变）。

用法（_tools/blender-4.5.14 约定）：
  _tools/blender-4.5.14-linux-x64/blender -b -P src/aubo_description/scripts/decimate_export.py \
    --env INPUT_STL=... VIS_STL=... COL_STL=... [VIS_TARGET=55000] [COL_TARGET=4500]
（blender 不透传环境时用 --env 包装；本仓用 `env VAR=... blender -b -P` 直接传）

流程：导入→合并→按距离焊接→去松散面→collapse 减面到目标面数→导出。
collision 档减面后做一次 merge by distance 保持流形稳定。
"""

import os

import bpy

INPUT_STL = os.environ['INPUT_STL']
VIS_STL = os.environ['VIS_STL']
COL_STL = os.environ['COL_STL']
VIS_TARGET = int(os.environ.get('VIS_TARGET', '55000'))
COL_TARGET = int(os.environ.get('COL_TARGET', '4500'))

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.stl_import(filepath=INPUT_STL)

objs = list(bpy.context.scene.objects)
if not objs:
    raise SystemExit('no mesh imported')

# 全部并成一个对象
bpy.ops.object.select_all(action='SELECT')
bpy.context.view_layer.objects.active = objs[0]
bpy.ops.object.join()
mesh_obj = bpy.context.view_layer.objects.active
me = mesh_obj.data
faces_before = len(me.polygons)
print('imported faces:', faces_before)

# 焊接近点 + 删退化/松散几何
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=1e-5)
bpy.ops.mesh.delete_loose(use_verts=True, use_edges=True, use_faces=True)
bpy.ops.mesh.dissolve_degenerate()
bpy.ops.object.mode_set(mode='OBJECT')
welded = len(me.polygons)
print('after weld:', welded)


def decimated_copy(target, name):
    """当前网格 deep copy + collapse 减面到 ~target 面数，返回新对象名。"""
    import bmesh
    copy = mesh_obj.copy()
    copy.data = me.copy()
    bpy.context.collection.objects.link(copy)
    bpy.context.view_layer.objects.active = copy
    mod = copy.modifiers.new('dec', 'DECIMATE')
    mod.decimate_type = 'COLLAPSE'
    ratio = max(0.0005, min(1.0, target / max(1, len(copy.data.polygons))))
    mod.ratio = ratio
    bpy.ops.object.modifier_apply(modifier=mod.name)
    print(name, 'faces:', len(copy.data.polygons), 'ratio:', round(ratio, 5))
    copy.name = name
    return copy


vis_obj = decimated_copy(VIS_TARGET, 'tool_visual')
col_obj = decimated_copy(COL_TARGET, 'tool_collision')

# collision 再焊接一次稳流形
bpy.context.view_layer.objects.active = col_obj
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=2e-5)
bpy.ops.object.mode_set(mode='OBJECT')
print('collision faces final:', len(col_obj.data.polygons))

bpy.ops.object.select_all(action='DESELECT')
vis_obj.select_set(True)
bpy.ops.wm.stl_export(filepath=VIS_STL, export_selected_objects=True)

bpy.ops.object.select_all(action='DESELECT')
col_obj.select_set(True)
bpy.ops.wm.stl_export(filepath=COL_STL, export_selected_objects=True)
print('WROTE', VIS_STL, COL_STL)
