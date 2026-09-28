# 功能：Blender 无头对齐验收渲染（CAD 法兰参考 vs URDF link6/快换盘/相机叠图）。
"""Blender 无头对齐验收渲染：CAD 法兰参考网格 vs URDF link6/快换盘/相机叠图。

用法：
  env REF_STL=.../wrist_ref.stl OUT_DIR=... [TOOL_STL=.../visual.stl] [TAG=bite] \
    _tools/blender-4.5.14-linux-x64/blender -b -P src/aubo_description/scripts/step_render_check.py

- URDF 侧：link6 collision STL（wrist3 系原点）+ quick_changer（z=0.0415, Rz=π）
  + camera（z=0.020），位姿照 aubo_e5.urdf.xacro 组件链。
- CAD 侧：wrist_ref.stl（i5-末端+主盘+相机族，法兰系=全局系）半透明红。
- 两者若重合 → CAD 原点即 wrist3_Link（映射恒等）成立；出 top/side/front 三图。
- 渲染管线同 blender_orchard（EEVEE_NEXT + 世界 + 太阳灯 + camera_add）。
"""

import math
import os

import bpy
from mathutils import Vector

REF_STL = os.environ['REF_STL']
OUT_DIR = os.environ['OUT_DIR']
TOOL_STL = os.environ.get('TOOL_STL', '')
TAG = os.environ.get('TAG', 'check')
PKG = '/home/mu/Desktop/aubo_e5_jazzy_ws/src/aubo_description/meshes'

bpy.ops.wm.read_factory_settings(use_empty=True)

MAT_COLORS = {
    'urdf_arm': (0.65, 0.68, 0.72, 1.0),
    'urdf_changer': (0.90, 0.60, 0.10, 1.0),
    'urdf_camera': (0.20, 0.45, 0.90, 1.0),
    'cad_ref': (0.85, 0.15, 0.10, 0.45),
    'tool': (0.30, 0.75, 0.35, 1.0),
}


def make_mat(name):
    color = MAT_COLORS[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Base Color'].default_value = (*color[:3], 1.0)
    bsdf.inputs['Alpha'].default_value = color[3]
    mat.blend_method = 'BLEND'
    return mat


def add_stl(path, mat_name, loc=(0, 0, 0), rot_z=0.0):
    bpy.ops.wm.stl_import(filepath=path)
    for obj in bpy.context.selected_objects:
        obj.location = loc
        obj.rotation_euler[2] = rot_z
        obj.data.materials.append(make_mat(mat_name))
        obj.name = mat_name + ':' + os.path.basename(path)[:28]


add_stl(os.path.join(PKG, 'collision', 'link6.stl'), 'urdf_arm')
add_stl(os.path.join(PKG, 'visual', 'tools', 'quick_changer.stl'),
        'urdf_changer', loc=(0, 0, 0.0415), rot_z=math.pi)
add_stl(os.path.join(PKG, 'visual', 'sensors', 'camera.stl'),
        'urdf_camera', loc=(0, 0, 0.020))
add_stl(REF_STL, 'cad_ref')
if TOOL_STL:
    add_stl(TOOL_STL, 'tool')

world = bpy.data.worlds.new('w')
bpy.context.scene.world = world
world.use_nodes = True
world.node_tree.nodes['Background'].inputs[0].default_value = (0.92, 0.92, 0.92, 1.0)
bpy.ops.object.light_add(type='SUN', location=(2, -2, 6))
bpy.context.active_object.data.energy = 3.0

scene = bpy.context.scene
scene.render.resolution_x = 1000
scene.render.resolution_y = 800
scene.render.engine = 'BLENDER_EEVEE_NEXT'
scene.eevee.taa_render_samples = 24


def aim_cam(eye, target):
    bpy.ops.object.camera_add(location=eye)
    cam = bpy.context.active_object
    cam.data.type = 'ORTHO'
    cam.data.ortho_scale = 0.6
    cam.rotation_euler = (
        Vector(target) - Vector(eye)).to_track_quat('-Z', 'Y').to_euler()
    return cam


VIEWS = {
    'top': ((0, 0, 0.6), (0, 0, 0)),
    'side': ((0.6, 0, 0.1), (0, 0, 0.1)),
    'front': ((0, -0.6, 0.1), (0, 0, 0.1)),
}
for view, (eye, target) in VIEWS.items():
    cam = aim_cam(eye, target)
    scene.camera = cam
    out = os.path.join(OUT_DIR, f'{TAG}_align_{view}.png')
    scene.render.filepath = out
    scene.render.image_settings.file_format = 'PNG'
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam, do_unlink=True)
    print('RENDER', out)
print('DONE')
