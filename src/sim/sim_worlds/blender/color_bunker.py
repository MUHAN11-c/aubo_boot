"""
将下载的 BUNKER STL 适配为固定工位外观；用 Blender 自带 Python 运行.

示例：blender -b --factory-startup -P blender/color_bunker.py --
      --source meshes/bunker_pro2.stl --out meshes/bunker_chassis.glb
网格顶点烘焙为 Z-up，名义包络 1.05 x 0.71 x 0.54 m。
只改显示几何，URDF 碰撞体与驱动插件独立保留。
"""

import argparse
from pathlib import Path
import sys

import bpy


def material(name, color, metallic, roughness):
    """直接使用 glTF 支持的 Principled PBR 常量，避免程序材质丢色."""
    result = bpy.data.materials.new(name)
    result.diffuse_color = (*color, 1.0)
    result.use_nodes = True
    shader = result.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1.0)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    return result


def main():
    """导入、归一化几何、按区域着色，并导出自包含 GLB."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.wm.stl_import(filepath=str(Path(args.source).resolve()),
                          forward_axis='Y', up_axis='Z')
    obj = bpy.context.object
    obj.name = 'bunker_chassis'
    vertices = obj.data.vertices
    low = [min(v.co[i] for v in vertices) for i in range(3)]
    high = [max(v.co[i] for v in vertices) for i in range(3)]
    span = [high[i] - low[i] for i in range(3)]
    # CAD 原点不是地面；将 XY 中心移到履带中心、最低点设为 z=0。
    dimensions = (1.05, 0.71, 0.54)
    for vertex in vertices:
        for axis in range(3):
            origin = low[axis] if axis == 2 else (low[axis] + high[axis]) / 2
            vertex.co[axis] = (vertex.co[axis] - origin) * dimensions[axis] / span[axis]
    obj.data.update()
    colors = [
        material('rubber_black', (0.018, 0.022, 0.027), 0.0, 0.92),
        material('chassis_graphite', (0.075, 0.095, 0.12), 0.35, 0.55),
        material('panel_orange', (0.88, 0.18, 0.025), 0.15, 0.45),
        material('wheel_metal', (0.27, 0.31, 0.36), 0.65, 0.38),
    ]
    for color in colors:
        obj.data.materials.append(color)
    counts = [0] * len(colors)
    for polygon in obj.data.polygons:
        center = polygon.center
        if abs(center.y) > 0.235 and center.z < 0.34:
            index = 0
            # 外侧轮毂区域留金属灰，履带齿与橡胶保持黑色。
            if abs(polygon.normal.y) > 0.85 and 0.07 < center.z < 0.23:
                index = 3
        elif center.z > 0.39 and abs(center.y) < 0.235:
            index = 2
        else:
            index = 1
        polygon.material_index = index
        counts[index] += 1
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(filepath=str(output), export_format='GLB',
                              use_selection=True, export_yup=False)
    print('BUNKER SOURCE BOUNDS', low, high, 'PBR FACE COUNTS', counts,
          'OUTPUT', output, flush=True)


if __name__ == '__main__':
    main()
