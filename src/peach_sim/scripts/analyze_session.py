#!/usr/bin/env python3
"""Summarize recorded RGB-D, gravity/contact and terminal-action evidence offline."""

import argparse
import json
from pathlib import Path

import numpy as np


def records(path):
    """Read an optional finite JSON-lines recording."""
    return [json.loads(line) for line in path.read_text().splitlines()] if path.is_file() else []


def analyze(run):
    """Keep measured evidence distinct from the simulator task's success flag."""
    summary = json.loads((run / 'summary.json').read_text())
    frames = records(run / 'sensors/frames.jsonl')
    events = records(run / 'sim_grasp.jsonl')
    poses = [pose for row in records(run / 'entity_poses.jsonl') for pose in row]
    contacts = [contact for row in records(run / 'contacts.jsonl') for contact in row]
    errors = [frame['cloud_z_depth_max_error_m'] for frame in frames
              if frame['cloud_z_depth_max_error_m'] is not None]
    sync = summary.get('sensor_evidence', {}).get('synchronized_frames', 0)
    report = {'action_result': summary['result'], 'process_group_clean': summary.get(
        'process_group_clean', False), 'sensor_sync_frames': sync, 'snapshots': len(frames),
        'cloud_depth_max_error_m': max(errors, default=None),
        'max_displacement_m': summary['max_displacement_m'],
        'final_velocity': summary.get('final_velocity', {}), 'loads': {}}
    for complete in (item for item in events if item['stage'] == 'harvest_confirmed'):
        key = complete['target']
        release = next(item['ros_time_ns'] for item in events
                       if item['stage'] == 'release' and item['target'] == key)
        relevant = [contact for contact in contacts if contact['stamp_ns'] >= release
                    and key + '_picked::' in (contact['collision1'] + contact['collision2'])]
        if not relevant:
            report['loads'][key] = {'error': 'no_native_basket_contact'}
            continue
        first = min(contact['stamp_ns'] for contact in relevant)
        fall = [pose for pose in poses if pose['name'] == key + '_picked'
                and release <= pose['stamp_ns'] < first]
        fit = None
        if len(fall) >= 4:
            t = [(pose['stamp_ns'] - release) / 1e9 for pose in fall]
            z = [pose['xyz'][2] for pose in fall]
            fit = float(2 * np.polyfit(t, z, 2)[0])
        force_by_time = {}
        for contact in relevant:
            if contact['stamp_ns'] > first + 1e9:
                at = contact['stamp_ns']
                force_by_time[at] = force_by_time.get(at, 0) + sum(
                    force[2] for force in contact.get('forces', []))
        settled = [item for item in events if item['stage'] == 'load_settled'
                   and item['target'] == key]
        report['loads'][key] = {
            'release_to_contact_s': (first - release) / 1e9,
            'freefall_samples': len(fall), 'freefall_acceleration_m_s2': fit,
            'native_contact_count': len(relevant),
            'max_contact_penetration_m': max(
                (depth for contact in relevant for depth in contact['depths_m']), default=0),
            'median_support_force_z_N': float(np.median(list(force_by_time.values())))
            if force_by_time else None,
            'settled_checks': len(settled),
            'last_settled': settled[-1] if settled else None,
        }
    action = summary.get('result') or {}
    image_ok = sync >= 10 and bool(errors) and max(errors) < 1e-5
    loads_ok = (action.get('succeeded', 0) == len(report['loads'])
                and all(load.get('settled_checks', 0) >= 2
                        and load.get('native_contact_count', 0) > 0
                        for load in report['loads'].values()))
    report['verification_passed'] = bool(
        image_ok and report['process_group_clean'] and summary['exit_code'] == 0
        and ((action.get('status') == 5
              and report['final_velocity'].get('linear_m_s', 1) < .01)
             or (action.get('success') and loads_ok and bool(report['loads']))))
    return report


def main():
    """Write the verification result beside the immutable input recordings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    run = args.run.resolve()
    if not run.is_relative_to(root):
        parser.error('run must be inside peach_sim')
    report = analyze(run)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    (run / 'verification.json').write_text(text)
    print(text)
    return int(not report['verification_passed'])


if __name__ == '__main__':
    raise SystemExit(main())
