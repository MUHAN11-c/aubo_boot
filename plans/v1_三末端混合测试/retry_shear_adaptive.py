#!/usr/bin/env python3
"""补跑 shear / adaptive 一次完整 FULL，墙钟≤120s。mock 允许打开 imu_follow.motion.enabled。"""
from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

ROOT = Path('/home/mu/Desktop/aubo_e5_jazzy_ws')
spec = importlib.util.spec_from_file_location(
    'run_hil', ROOT / 'plans' / 'v1_三末端混合测试' / 'run_hil.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

mod.MANAGED = (
    'peach_scene_perception_node',
    'peach_target_reconstruction_node',
    'peach_supervisor',
    'peach_arm',
)


def one(profile: str, imu_motion: bool) -> dict:
    print(f'== retry {profile}', flush=True)
    log = mod.OUT / f'{profile}_retry_launch.log'
    proc, handle = mod.start_stack(profile, log)
    item = {'profile': profile, 'retry': True}
    try:
        item['lifecycle'] = mod.wait_lifecycle(90.0)
        time.sleep(8.0)
        if imu_motion:
            try:
                r = mod.bash(
                    'timeout 10 ros2 param set /imu_follow motion.enabled true',
                    14)
                item['imu_motion'] = (r.stdout or '') + (r.stderr or '')
            except Exception as exc:  # noqa: BLE001
                item['imu_motion'] = repr(exc)
        item['tool_profile_id'] = mod.param_get('peach_arm', 'tool.profile_id')
        item['color_hz'] = mod.topic_hz('/camera/color/image_raw', 5)
        grasp_log = mod.OUT / f'{profile}_retry_grasp_1757.txt'
        item['grasp'] = mod.run_one_full_grasp(profile, grasp_log)
    finally:
        item['launch_exit'] = mod.stop_launch(proc, handle)
        item['leftover'] = mod.domain_pids()
    (mod.OUT / f'{profile}_retry.json').write_text(
        __import__('json').dumps(item, ensure_ascii=False, indent=2),
        encoding='utf-8')
    return item


def main() -> int:
    mod.OUT.mkdir(parents=True, exist_ok=True)
    mod.stop_domain()
    results = [
        one('shear_v1', False),
        one('adaptive_shear_v1', True),
    ]
    leftover = mod.domain_pids()
    print(__import__('json').dumps({
        'leftover': leftover,
        'profiles': [
            {
                'id': r['profile'],
                'elapsed_s': (r.get('grasp') or {}).get('elapsed_s'),
                'tail': ((r.get('grasp') or {}).get('tail') or '')[-400:],
            }
            for r in results
        ],
    }, ensure_ascii=False, indent=2))
    return 0 if not leftover else 2


if __name__ == '__main__':
    sys.exit(main())
