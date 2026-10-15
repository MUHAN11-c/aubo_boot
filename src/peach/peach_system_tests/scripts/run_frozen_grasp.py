#!/usr/bin/env python3
"""Run a frozen-camera mock grasp and stop only this wrapper's process groups.

Default ROS domain 81. Frozen regression is headless unless --rviz is explicit.
The source bag is read only; --skip-launch preserves the external stack.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from run_live_camera_mock import (
    OwnedProcesses, RosDriver, cleanup_driver, defer_stop_signals, snapshot,
)

GRASP_S = 180.0
READY_S = 120.0
MARKERS = ('peach', 'frozen_camera', 'move_group', 'ros2_control_node',
           'robot_state_publisher', 'component_container', 'extrinsics_publisher',
           'lifecycle_manager', 'ros2 launch', 'ros2 run', 'bag record')


def _domain_of(pid):
    try:
        raw = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        return next((part.split(b'=', 1)[1].decode(errors='replace') for part in raw
                     if part.startswith(b'ROS_DOMAIN_ID=')), '0')
    except OSError:
        return None


def _pgrep(domain=None):
    """Read process inventory; expose PID/executable only, never shell arguments."""
    rows = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            pid = int(path.parent.name)
            if pid in (os.getpid(), os.getppid()):
                continue
            executable = (path.parent / 'exe').resolve().name
            if executable in ('bash', 'sh', 'dash', 'timeout', 'bwrap', 'zsh'):
                continue
            command = path.read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if any(marker in command for marker in MARKERS) and (domain is None or _domain_of(pid) == domain):
                rows.append((pid, executable))
        except (OSError, ValueError):
            pass
    return rows


def create_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='')
    parser.add_argument('--index', type=int, default=70)
    parser.add_argument('--hz', type=float, default=2.0)
    parser.add_argument('--request-id', default='')
    parser.add_argument('--timeout', type=float, default=GRASP_S)
    parser.add_argument('--ready-timeout', type=float, default=READY_S)
    parser.add_argument('--skip-launch', action='store_true')
    parser.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=False)
    return parser


def assert_mock_stack(domain):
    facts = snapshot().get('startup', {}).get('facts', {})
    if facts.get('hardware_mode') != 'mock' or str(facts.get('ros_domain_id')) != str(domain):
        raise RuntimeError('Refusing operation: expected mock stack on this ROS domain')


def cleanup_request(env, request):
    """Cancel/disable only the batch this wrapper sent; never a foreign request."""
    errors = []
    driver = None
    try:
        state = snapshot()
        facts = state.get('startup', {}).get('facts', {})
        task = state.get('task_executor', {}).get('state', {})
        if (facts.get('hardware_mode') != 'mock' or str(facts.get('ros_domain_id')) != env['ROS_DOMAIN_ID']
                or task.get('run_id') != request):
            print('cancel/disable skipped: current request/domain is not owned', flush=True)
            return []
        os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        driver = RosDriver()
        if task.get('action_active') or task.get('batch_state') not in (0, 6, 8):
            from peach_interfaces.srv import ControlTask
            try:
                reply = driver.service(ControlTask, '/peach_supervisor/control', ControlTask.Request(
                    request_id=request, expected_state_seq=task['state_seq'], command=4,
                    reason='frozen wrapper owned request cleanup'))
                if not reply.accepted:
                    errors.append('CANCEL_NOW rejected: ' + reply.message)
            except Exception as exc:
                errors.append('CANCEL_NOW: ' + str(exc))
    except Exception as exc:
        errors.append('cleanup verification: ' + str(exc))
    finally:
        if driver:
            cleanup_driver(driver, errors)
    return errors


def main(argv=None):
    args = create_parser().parse_args(argv)
    if args.bag.strip():
        bag = args.bag.strip()
    else:
        from frozen_camera_replay import discover_bag
        bag = discover_bag()
    if not bag or not (Path(bag) / 'metadata.yaml').is_file():
        print('frozen camera bag not found; set --bag or PEACH_FROZEN_BAG', file=sys.stderr)
        return 2
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '81')
    env.update(ROS_LOCALHOST_ONLY='1', PEACH_FROZEN_BAG=bag)
    domain = env['ROS_DOMAIN_ID']
    request = args.request_id or ('frozen_' + uuid.uuid4().hex[:12])
    print(f'ROS_DOMAIN_ID={domain} bag={bag} ready={args.ready_timeout:.0f}s grasp={args.timeout:.0f}s', flush=True)
    for pid, executable in _pgrep()[:8]:
        print(f'note: existing process pid={pid} executable={executable}', flush=True)
    owner = OwnedProcesses(env)
    started = time.monotonic()
    code = 1
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    def interrupt(signum, frame):
        raise KeyboardInterrupt('received signal ' + str(signum))
    try:
        for sig in previous:
            signal.signal(sig, interrupt)
        if not args.skip_launch:
            if _pgrep(domain):
                raise RuntimeError('Domain already has ROS processes; refusing to launch over an external stack')
            launch = owner.start(['ros2', 'launch', 'peach_system_tests', 'frozen_camera_grasp.launch.py',
                f'bag:={bag}', f'frame_index:={args.index}', f'hz:={args.hz}',
                'rviz:=' + str(args.rviz).lower()])
            time.sleep(3)
            if launch.poll() is not None:
                return launch.returncode or 1
        else:
            assert_mock_stack(domain)
        driver = owner.start(['ros2', 'run', 'peach_system_tests', 'drive_frozen_grasp.py',
            '--ready-timeout', str(args.ready_timeout), '--timeout', str(args.timeout),
            '--request-id', request])
        code = driver.wait(timeout=args.ready_timeout + args.timeout + 60)
    except subprocess.TimeoutExpired:
        print('drive timed out; outcome unknown, cancelling owned batch', file=sys.stderr)
        code = 2
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            with defer_stop_signals():
                errors = cleanup_request(env, request)
                for message in errors:
                    print(message, file=sys.stderr)
                if errors:
                    code = code or 1
                residuals = owner.cleanup()
                if residuals:
                    print('owned process residual PIDs: ' + str(residuals), file=sys.stderr)
                    code = code or 1
                else:
                    print('owned process groups clean; external stacks untouched', flush=True)
                print(f'wall elapsed={time.monotonic() - started:.1f}s', flush=True)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return code


if __name__ == '__main__':
    sys.exit(main())
