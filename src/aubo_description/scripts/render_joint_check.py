# Copyright 2026 aubo_e5_jazzy_ws contributors.
# 功能：连接处核查渲染：URDF 落位（link6+快换）+工具网格+TCP 三轴标记，特写复现 RViz 视角。
"""Blender 无头：复现 RViz 里快换与工具连接处 + TCP 轴，供重叠/TF 核查。

用法：
  env TOOL=<bite_shear_v1|adaptive_shear_v1|shear_v1> TCP_XYZ=0,0.047,0.16866 \
  OUT=/path/out.png blender -b -P render_joint_check.py
URDF 位姿：link6 原点；quick_changer z+0.0415 Rz180°；工具网格 identity（法兰系建模）；
TCP 轴组在 tcp_xyz、姿态 Rx(-90°)（三把剪切手同款：tcpZ=法兰+Y 开口）。
"""

import math
import os

import bpy

TOOL = os.environ['TOOL']
TCP_XYZ = [float(v) for v in os.environ['TCP_XYZ'].split(',')]
OUT = os.environ['OUT']
PKG = '/home/mu/Desktop/aubo_e5_jazzy_ws/src/aubo_description/meshes'

bpy.ops.wm.read_factory_settings(use_empty=True)


def mat(name, color, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = (*color, 1.0)
    b.inputs['Alpha'].default_value = alpha
    m.blend_method = 'BLEND'
    return m


def add(path, material, loc=(0, 0, 0), rot=(0, 0, 0), scale=1.0):
    bpy.ops.wm.stl_import(filepath=path)
    for obj in bpy.context.selected_objects:
        obj.location = loc
        obj.rotation_euler = rot
        obj.scale = (scale,) * 3
        obj.data.materials.append(material)


M_ARM = mat('arm', (0.65, 0.68, 0.72), 0.55)
M_QC = mat('qc', (0.90, 0.60, 0.10), 0.75)
M_TOOL = mat('tool', (0.30, 0.75, 0.35), 0.85)
M_AXIS = [mat('ax_r', (0.9, 0.1, 0.1)), mat('ax_g', (0.1, 0.8, 0.1)),
          mat('ax_b', (0.1, 0.2, 0.9))]
M_ORIGIN = mat('tcp0', (1.0, 1.0, 1.0))

add(os.path.join(PKG, 'collision', 'link6.stl'), M_ARM)
add(os.path.join(PKG, 'visual', 'tools', 'quick_changer.stl'), M_QC,
    loc=(0, 0, 0.0415), rot=(0, 0, math.pi))
add(os.path.join(PKG, 'visual', 'tools', f'{TOOL}.stl'), M_TOOL)

# TCP 三轴（tcp 系：X/Y/Z 各一根，Rx(-90°) 后 X=法兰X、Y=法兰-Z、Z=法兰+Y）
tcp = bpy.data.objects.new('tcp_origin', None)
tcp.location = TCP_XYZ
bpy.context.collection.objects.link(tcp)
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.008, location=TCP_XYZ)
bpy.context.selected_objects[0].data.materials.append(M_ORIGIN)
AX_LEN, AX_R = 0.10, 0.0035
for i, (direction, material) in enumerate((
        ((1, 0, 0), M_AXIS[0]), ((0, 1, 0), M_AXIS[1]), ((0, 0, 1), M_AXIS[2]))):
    bpy.ops.mesh.primitive_cylinder_add(radius=AX_R, depth=AX_LEN,
                                        location=tuple(
                                            TCP_XYZ[j] + direction[j] * AX_LEN / 2
                                            for j in range(3)))
    cyl = bpy.context.selected_objects[0]
    cyl.data.materials.append(material)
    cyl.rotation_euler = ((math.pi / 2, 0, 0) if i == 1
                          else (0, math.pi / 2, 0) if i == 2 else (0, 0, 0))

world = bpy.data.worlds.new('w')
bpy.context.scene.world = world
world.use_nodes = True
world.node_tree.nodes['Background'].inputs[0].default_value = (0.9, 0.9, 0.9, 1)
bpy.ops.object.light_add(type='SUN', location=(1, -1, 3))
bpy.context.active_object.data.energy = 3

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE_NEXT'
scene.eevee.taa_render_samples = 24
scene.render.resolution_x = 1200
scene.render.resolution_y = 900


def cam(name, eye, target, lens=50):
    bpy.ops.object.camera_add(location=eye)
    c = bpy.context.active_object
    c.name = name
    c.data.lens = lens
    c.rotation_euler = (Vector(target) - Vector(eye)).to_track_quat('-Z', 'Y').to_euler()
    return c


from mathutils import Vector  # noqa: E402

views = {
    'joint_closeup': (cam('c1', (0.28, -0.30, 0.16), (0, -0.03, 0.09), 40),),
    'side': (cam('c2', (0.45, 0, 0.14), (0, 0, 0.12), 50),),
    'top': (cam('c3', (0, 0, 0.5), (0, 0, 0.12), 50),),
}
for name, (c,) in views.items():
    scene.camera = c
    scene.render.filepath = f'{OUT}_{name}.png'
    scene.render.image_settings.file_format = 'PNG'
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(c, do_unlink=True)
    print('RENDER', name)
print('DONE')
