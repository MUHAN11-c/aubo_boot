#!/usr/bin/env python3
"""
Start the v2 frozen-camera mock stack, drive one FULL harvest, then stop.

Isolates ROS_DOMAIN_ID (default 82). Does not connect to a real robot.
Requires the frozen bag and a GPU for YOLO/SAM. Mock IO backend; no real SetIO.
Whole-run wall is 180 s.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from frozen_camera_replay import discover_bag

WALL_S = 180.0
MARKERS = (
    'ros2 launch',
    'component_container',
    'extrinsics_publisher',
    'ros2 run',
    'bag record',
    'frozen_camera',
    'move_group',
    'ros2_control_node',
    'robot_state_publisher',
    'peach2_',
    'lifecycle_manager',
)


def _domain_of(pid: int) -> str | None:
    try:
        raw = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    for item in raw:
        if item.startswith(b'ROS_DOMAIN_ID='):
            return item.split(b'=', 1)[1].decode(errors='replace')
    return ''


def _pgrep(domain: str | None = None) -> list[tuple[int, str]]:
    try:
        out = subprocess.check_output(
            ['pgrep', '-af', '|'.join(MARKERS)], text=True)
    except subprocess.CalledProcessError:
        return []
    rows = []
    self = os.getpid()
    for line in out.splitlines():
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid == self or pid == os.getppid():
            continue
        if 'pgrep' in parts[1]:
            continue
        if domain is not None:
            pid_domain = _domain_of(pid)
            if pid_domain is None:
                continue
            if pid_domain != domain:
                continue
        rows.append((pid, parts[1]))
    return rows


def _stop_pid(pid: int) -> None:
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def _stop_group(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=12)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=4)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=4)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='')
    parser.add_argument('--index', type=int, default=70)
    parser.add_argument('--hz', type=float, default=2.0)
    parser.add_argument('--request-id', default='')
    parser.add_argument('--tool-id', default='bite_shear_v1',
                        help='mounted tool (must match a frozen_tools/<id>.yaml)')
    parser.add_argument('--intent', default='', choices=('', 'FULL', 'PREGRASP_ONLY'),
                        help='override the driven RunBatch intent (default FULL)')
    parser.add_argument('--timeout', type=float, default=WALL_S,
                        help='whole-run wall clock seconds (default 180)')
    parser.add_argument(
        '--skip-launch', action='store_true',
        help='drive only; stack already running on this domain')
    args = parser.parse_args()
    bag = args.bag.strip() or discover_bag()
    if not bag or not (Path(bag) / 'metadata.yaml').is_file():
        print('frozen camera bag not found; set --bag or PEACH_FROZEN_BAG',
              file=sys.stderr)
        return 2

    env = os.environ.copy()
    env.setdefault('ROS_DOMAIN_ID', '82')
    env['ROS_LOCALHOST_ONLY'] = '1'
    # 不设 QT_QPA_PLATFORM=offscreen：会破坏 rviz2 的 GLX 窗口（可视化验证需要真渲染）
    env['PEACH_FROZEN_BAG'] = bag
    domain = env['ROS_DOMAIN_ID']
    print(f'ROS_DOMAIN_ID={domain} bag={bag} wall={args.timeout:.0f}s',
          flush=True)

    leftover = _pgrep()
    if leftover:
        print('note: other ROS processes exist (other domain, not killed):',
              flush=True)
        for pid, cmd in leftover[:8]:
            print(f'  {pid} {cmd}', flush=True)

    t0 = time.monotonic()

    def remaining() -> float:
        return args.timeout - (time.monotonic() - t0)

    launch_proc = None
    drive_proc = None
    code = 1
    try:
        if not args.skip_launch:
            launch_cmd = [
                'ros2', 'launch', 'peach2_system_tests',
                'frozen_camera_grasp.launch.py',
                f'bag:={bag}',
                f'frame_index:={args.index}',
                f'hz:={args.hz}',
                f'tool_id:={args.tool_id}',
            ]
            # spawn = 真拍照位 global_photo_pose（launch 默认 mock_initial_positions）
            launch_proc = subprocess.Popen(
                launch_cmd, env=env, start_new_session=True)
            print(f'launch pid={launch_proc.pid}', flush=True)
            time.sleep(min(3.0, max(0.0, remaining() - 20.0)))
            if launch_proc.poll() is not None:
                print(
                    f'launch exited early rc={launch_proc.returncode}',
                    file=sys.stderr)
                code = launch_proc.returncode or 1
                launch_proc = None
                return code
        harvest_s = min(100.0, max(20.0, remaining() - 20.0))  # 批跑（出生即拍照位，无 home→photo 过渡）
        ready_s = min(45.0, max(15.0, remaining() - harvest_s - 25.0))
        drive_cmd = [
            'ros2', 'run', 'peach2_system_tests', 'drive_frozen_grasp.py',
            '--ready-timeout', f'{ready_s:.1f}',
            '--timeout', f'{harvest_s:.1f}',
        ]
        if args.request_id:
            drive_cmd.extend(['--request-id', args.request_id])
        if args.intent:
            drive_cmd.extend(['--intent', args.intent])
        drive_proc = subprocess.Popen(
            args=drive_cmd, env=env, start_new_session=True)
        try:
            code = drive_proc.wait(timeout=max(5.0, remaining() - 8.0))
        except subprocess.TimeoutExpired:
            print(
                f'drive TIMEOUT wall>={args.timeout:.0f}s — inspect',
                file=sys.stderr)
            _stop_group(drive_proc)
            code = 2
            drive_proc = None
        elapsed = time.monotonic() - t0
        print(f'drive elapsed={elapsed:.1f}s exit={code}', flush=True)
        if elapsed > args.timeout + 1.0 and code == 0:
            print('full run exceeded 180s — inspect', file=sys.stderr)
            code = 2
    finally:
        if drive_proc is not None and drive_proc.poll() is None:
            _stop_group(drive_proc)
        if launch_proc is not None:
            _stop_group(launch_proc)
        time.sleep(1.0)
        leftover = _pgrep(domain)
        if leftover:
            print('stopping leftover pids on this domain:', flush=True)
            for pid, cmd in leftover:
                print(f'  TERM {pid} {cmd}', flush=True)
                _stop_pid(pid)
            time.sleep(0.5)
            leftover = _pgrep(domain)
        if leftover:
            print('still running after cleanup:', file=sys.stderr)
            for pid, cmd in leftover:
                print(f'  {pid} {cmd}', file=sys.stderr)
        else:
            print('pgrep clean (this domain)', flush=True)
        print(f'wall elapsed={time.monotonic() - t0:.1f}s', flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
