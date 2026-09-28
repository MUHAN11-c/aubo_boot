"""矩阵渲染：已建 .blend × 光照预设 × 轨迹视角 → RGB + Depth/IndexOB/Position EXR.

前置：build_scene.py 已用当前代码重建（manifest 带光照/遮挡/世界系字段）。
本脚本只换 world 光照与相机，不重建几何；输出
`output/matrix/<lighting>/<view_id>/`。停走节拍轨迹由 viewpoints.py 生成
（survey 停靠 + 每目标近距拍照位 + 补视链）。

用法：
  _tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
    -P src/peach_sim/reconstruction/run_matrix.py -- --samples 32
"""

import argparse
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_scene import _enable_gpu  # noqa: E402
import lighting as lighting_mod  # noqa: E402
from lighting_apply import apply_lighting  # noqa: E402
from viewpoints import (alley_stops, build_trajectory,  # noqa: E402
                        supplemental_viewpoint)

PASSES = ('Depth', 'IndexOB', 'Position')
RETREAT_STEPS_M = (.15, .30, .45, .60)
"""Primary 视位若被枝叶挡住，沿 -Y（作业道方向）后退的步进档。"""


def _view_clear(depsgraph, eye, target):
    direction = Vector(target) - Vector(eye)
    dist = direction.length
    if dist < .05:
        return True
    hit, _, _, _, _, _ = bpy.context.scene.ray_cast(
        depsgraph, Vector(eye), direction.normalized(),
        distance=dist - .05)
    return not hit


def adjust_primary_views(traj):
    """冠外取景修正：primary 视位被挡则沿作业道后退，补视链随之重算.

    产线语义：臂相机从冠层外伸向挂果枝，不在冠内取景。纯核给的是意图
    视位；这里用 ray_cast 保证"相机→袋心"通路无几何，物理上可达才渲。
    """
    depsgraph = bpy.context.evaluated_depsgraph_get()
    view_by_id = {v['view_id']: v for v in traj['views']}
    adjusted = 0
    retreat = Vector((0., -1., 0.))
    for tid, chain in traj['per_target'].items():
        if chain[0] not in view_by_id:  # filtered out (--max-views / subset)
            continue
        info = traj['target_registry'][tid]
        center = Vector(info['center_world'])
        axis = Vector(info['axis_world'])
        primary = view_by_id[chain[0]]
        eye = Vector(primary['eye'])
        final = eye + retreat * RETREAT_STEPS_M[-1]
        for step in (0.,) + RETREAT_STEPS_M:
            cand = eye + retreat * step
            if _view_clear(depsgraph, cand, center):
                final = cand
                break
        if (final - eye).length > 1e-6:
            adjusted += 1
            primary['eye_adjusted_m'] = round((final - eye).length, 3)
        primary['eye'] = list(final)
        cam = final
        for vid in chain[1:]:
            nxt = Vector(supplemental_viewpoint(
                list(center), list(cam), list(axis)))
            view_by_id[vid]['eye'] = list(nxt)
            cam = nxt
    traj['primary_adjusted'] = adjusted


def matrix_camera(traj):
    name = 'Matrix trajectory camera'
    data = bpy.data.cameras.new(name)
    data.type = 'PERSP'
    intr = traj['intrinsics']
    data.lens = intr['lens_mm']
    data.sensor_width = intr['sensor_width_mm']
    data.clip_start = .015
    data.clip_end = 200
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    return obj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--blend', default=str(
        HERE / 'output/bagged_peach_orchard.blend'))
    parser.add_argument('--samples', type=int, default=32)
    parser.add_argument('--lightings', default='all',
                        help="'all' or comma-separated preset names")
    parser.add_argument('--out', default=str(HERE / 'output/matrix'))
    parser.add_argument('--max-views', type=int, default=0,
                        help='smoke test: render only the first N views')
    parser.add_argument('--subset-per-level', type=int, default=0,
                        help='evaluate a stratified subset: N tree bags per '
                             'occlusion level (plus all reference bags); '
                             '0 = every target (slow: ~1900 renders)')
    args = parser.parse_args(sys.argv[sys.argv.index(
        '--') + 1:] if '--' in sys.argv else [])

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.open_mainfile(filepath=args.blend)
    manifest = json.loads((HERE / 'output/scene_manifest.json').read_text())
    stops = alley_stops(-5.0, 4.5, 6, -0.5, 1.45, (0, 2.8, 1.35))
    traj = build_trajectory(manifest['targets'], stops)
    traj['lighting_applied'] = (sorted(lighting_mod.PRESETS)
                                if args.lightings == 'all'
                                else [s.strip() for s in args.lightings.split(',')])
    if args.subset_per_level:
        import random as _random
        rng = _random.Random(240928)
        by_level = defaultdict(list)
        for t in manifest['targets']:
            by_level[t.get('occlusion', {}).get('level', 'reference')].append(
                t['id'])
        keep = set(by_level['reference'])
        for level in ('none', 'light', 'heavy'):
            pool = sorted(by_level[level])
            keep.update(rng.sample(pool, min(args.subset_per_level,
                                             len(pool))))
        traj['views'] = [v for v in traj['views']
                         if v['kind'] == 'alley_stop'
                         or set(v.get('target_ids', ())) & keep]
        traj['per_target'] = {tid: chain for tid, chain
                              in traj['per_target'].items() if tid in keep}
        traj['target_registry'] = {tid: info for tid, info
                                   in traj['target_registry'].items()
                                   if tid in keep}
        traj['subset_per_level'] = args.subset_per_level
    if args.max_views:
        keep = {v['view_id'] for v in traj['views'][:args.max_views]}
        traj['views'] = [v for v in traj['views'] if v['view_id'] in keep]
        traj['smoke_max_views'] = args.max_views
    adjust_primary_views(traj)
    (out / 'trajectory.json').write_text(json.dumps(traj, indent=1))

    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    backend = _enable_gpu()
    scene.cycles.device = 'GPU' if backend != 'CPU' else 'CPU'
    scene.cycles.samples = args.samples
    scene.render.resolution_x = traj['intrinsics']['width']
    scene.render.resolution_y = traj['intrinsics']['height']
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    fnode = next(n for n in scene.node_tree.nodes
                 if n.type == 'OUTPUT_FILE')
    fnode.format.file_format = 'OPEN_EXR'
    fnode.format.color_depth = '32'
    fnode.format.exr_codec = 'ZIP'
    cam = matrix_camera(traj)

    total = len(traj['lighting_applied']) * len(traj['views'])
    done = 0
    t0 = time.time()
    times = []
    for lname in traj['lighting_applied']:
        apply_lighting(scene, lighting_mod.get(lname))
        for view in traj['views']:
            view_dir = out / lname / view['view_id']
            view_dir.mkdir(parents=True, exist_ok=True)
            cam.location = Vector(view['eye'])
            cam.rotation_euler = (
                Vector(view['lookat']) -
                Vector(view['eye'])).to_track_quat('-Z', 'Y').to_euler()
            scene.camera = cam
            fnode.base_path = str(view_dir)
            for slot, key in zip(fnode.file_slots, PASSES):
                slot.path = f'{key}_'
            scene.render.filepath = str(view_dir / 'rgb.png')
            stamp = time.time()
            bpy.ops.render.render(write_still=True)
            times.append({'lighting': lname, 'view': view['view_id'],
                          'seconds': round(time.time() - stamp, 2)})
            done += 1
            if done % 25 == 0:
                rate = (time.time() - t0) / done
                print(f'MATRIX {done}/{total} avg {rate:.1f}s '
                      f'eta {rate * (total - done) / 60:.0f}min', flush=True)
    (out / 'render_times.json').write_text(json.dumps(times, indent=1))
    print(f'MATRIX COMPLETE {done} renders in '
          f'{(time.time() - t0) / 60:.1f} min', flush=True)


if __name__ == '__main__':
    main()
