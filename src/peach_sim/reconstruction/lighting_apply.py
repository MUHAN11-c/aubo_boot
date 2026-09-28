"""把 lighting.py 预设应用到 Blender world/补光（bpy 依赖，建场与矩阵共用）.

build_scene.setup_render 与 run_matrix 都经此函数，保证同一预设在不同
入口下产生同一光照。
"""

import math

import bpy
import lighting
from mathutils import Vector


def apply_lighting(scene, preset: lighting.LightingPreset) -> None:
    """改 world 天空节点 + 冠下补光 SUN；要求 world 已 use_nodes."""
    if not scene.world or not scene.world.use_nodes:
        raise ValueError('scene.world must exist with use_nodes enabled')
    nodes = scene.world.node_tree.nodes
    links = scene.world.node_tree.links
    sky = next((node for node in nodes if node.type == 'TEX_SKY'), None)
    if sky is None:
        sky = nodes.new('ShaderNodeTexSky')
        sky.sky_type = 'NISHITA'
        links.new(sky.outputs[0], nodes.get('Background').inputs[0])
    sky.sun_elevation = math.radians(preset.sun_elevation_deg)
    sky.sun_rotation = math.radians(preset.sun_azimuth_deg)
    sky.sun_disc = True
    sky.sun_size = math.radians(preset.sun_size_deg)
    # Nishita exposes Air/Dust densities instead of a single turbidity;
    # map the preset turbidity onto both (dust drives the haze look).
    sky.air_density = preset.turbidity
    sky.dust_density = preset.turbidity * .8
    nodes.get('Background').inputs[1].default_value = preset.sky_strength

    fill = bpy.data.objects.get('Sun through canopy')
    if fill is None:
        lamp = bpy.data.lights.new('Sun through canopy', 'SUN')
        fill = bpy.data.objects.new('Sun through canopy', lamp)
        bpy.context.collection.objects.link(fill)
    fill.data.energy = preset.fill_energy
    fill.data.angle = math.radians(preset.fill_spread_deg)
    # Fill travels from the sun azimuth/elevation toward the canopy.
    az = math.radians(preset.sun_azimuth_deg)
    el = math.radians(preset.sun_elevation_deg)
    sun_dir = Vector((math.sin(az) * math.cos(el),
                      math.cos(az) * math.cos(el), math.sin(el)))
    fill.rotation_euler = sun_dir.to_track_quat('Z', 'Y').to_euler()
