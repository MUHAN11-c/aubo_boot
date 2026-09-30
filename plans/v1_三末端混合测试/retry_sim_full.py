#!/usr/bin/env python3
"""无相机 mock 补跑三末端 FULL 干跑，墙钟≤120s（相机已在第一轮证明在环）。"""
from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path

ROOT = Path('/home/mu/Desktop/aubo_e5_jazzy_ws')
spec = importlib.util.spec_from_file_location(
    'run_hil', ROOT / 'plans' / 'v1_三末端混合测试' / 'run_hil.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def one(profile: str) -> dict:
    print(f'== sim-full {profile}', flush=True)
    log = mod.OUT / f'{profile}_sim_full_launch.log'
    proc, handle = mod.start_stack(profile, log, camera_enabled=False)
    item = {'profile': profile, 'camera_enabled': False}
    try:
        item['lifecycle'] = mod.wait_lifecycle(90.0)
        time.sleep(8.0)
        if profile == 'adaptive_shear_v1':
            try:
                r = mod.bash(
                    'timeout 10 ros2 param set /imu_follow motion.enabled true',
                    14)
                item['imu_motion'] = (r.stdout or '') + (r.stderr or '')
            except Exception as exc:  # noqa: BLE001
                item['imu_motion'] = repr(exc)
        item['tool_profile_id'] = mod.param_get('peach_arm', 'tool.profile_id')
        grasp_log = mod.OUT / f'{profile}_sim_full_grasp_1757.txt'
        item['grasp'] = mod.run_one_full_grasp(profile, grasp_log)
    finally:
        item['launch_exit'] = mod.stop_launch(proc, handle)
        item['leftover'] = mod.domain_pids()
    (mod.OUT / f'{profile}_sim_full.json').write_text(
        json.dumps(item, ensure_ascii=False, indent=2), encoding='utf-8')
    return item


def main() -> int:
    mod.OUT.mkdir(parents=True, exist_ok=True)
    mod.stop_domain()
    results = [one(p) for p in mod.PROFILES]
    leftover = mod.domain_pids()
    summary = {
        'leftover': leftover,
        'profiles': [
            {
                'id': r['profile'],
                'elapsed_s': (r.get('grasp') or {}).get('elapsed_s'),
                'passed_gate': (r.get('grasp') or {}).get('passed_gate'),
                'within_2min': (r.get('grasp') or {}).get('within_2min'),
                'tail': ((r.get('grasp') or {}).get('tail') or '')[-500:],
            }
            for r in results
        ],
    }
    (mod.OUT / 'sim_full_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    gates = [p.get('passed_gate') for p in summary['profiles']]
    return 0 if leftover == [] and all(gates) else 2


if __name__ == '__main__':
    sys.exit(main())
