"""
Launch preflight for peach2_system (no ROS imports).

Refuses to start when a stack process already runs in the same ROS domain: a second
robot_state_publisher / extrinsics_publisher silently stacks the same TF child frame, and a
second lifecycle manager or peach node splits the graph. Matching is on argv basenames, never on
substrings of the whole command line, so editors, colcon and pytest arguments do not match.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Iterable

STACK_EXECUTABLES = frozenset({
    # peach2 (this stack; a stale instance is refused like any other)
    'peach2_manipulation_node',
    'peach2_task_node',
    'target_model_node',
    'perception_node',
    'scene_node',
    # peach v1 (must not share a graph with v2)
    'peach_harvester',
    'peach_scene_perception_node',
    'peach_target_reconstruction_node',
    'peach_arm',
    'peach_supervisor',
    'peach_observability',
    'peach_lifecycle_flag_bridge',
    'peach_autostart_client',
    'peach_scene_obstacles',
    # driver / MoveIt / TF publishers started by aubo_e5_bringup
    'lifecycle_manager',
    'component_container',
    'component_container_mt',
    'move_group',
    'robot_state_publisher',
    'ros2_control_node',
    'joint_state_publisher',
    'joint_state_publisher_gui',
    'extrinsics_publisher',
    'stereo_camera_node',
    'serial_imu_node',
    'imu_follow_node',
    'servo_node',
    'rviz2',
})

DEFAULT_DOMAIN_ID = '0'
UNKNOWN_DOMAIN = '?'


@dataclass(frozen=True)
class StackProcess:
    """One conflicting process: pid, its ROS_DOMAIN_ID and a shortened command line."""

    pid: int
    domain_id: str
    command: str


def split_cmdline(raw: bytes | str) -> list[str]:
    """/proc/<pid>/cmdline (NUL separated) -> argv without empty parts."""
    text = raw.decode('utf-8', 'replace') if isinstance(raw, bytes) else raw
    return [part for part in text.split('\x00') if part]


def matched_executable(argv: list[str],
                       executables: Iterable[str] = STACK_EXECUTABLES) -> str | None:
    """
    Stack executable this argv runs, or None.

    argv[0] covers C++ binaries; a later part counts only when it is an installed script under a
    lib/ directory (python3 /…/lib/<pkg>/<exe> --ros-args …), so a bare word in some other
    tool's arguments does not match.
    """
    names = set(executables)
    if not argv:
        return None
    first = os.path.basename(argv[0].rstrip('/'))
    if first in names:
        return first
    for part in argv[1:]:
        if part.startswith('-'):
            continue
        base = os.path.basename(part.rstrip('/'))
        if base in names and '/lib/' in part.replace('\\', '/'):
            return base
    return None


def domain_from_environ(raw: bytes | None) -> str:
    """ROS_DOMAIN_ID from /proc/<pid>/environ; unset/empty -> '0'; unreadable -> '?'."""
    if raw is None:
        return UNKNOWN_DOMAIN
    for entry in raw.split(b'\x00'):
        if entry.startswith(b'ROS_DOMAIN_ID='):
            value = entry.split(b'=', 1)[1].decode('utf-8', 'replace').strip()
            return value or DEFAULT_DOMAIN_ID
    return DEFAULT_DOMAIN_ID


def conflicts(domain_id: str, own_domain: str) -> bool:
    """Report a conflict for the same domain or an unknown one (unreadable environ)."""
    return domain_id in (own_domain, UNKNOWN_DOMAIN)


def _read(path: str) -> bytes | None:
    try:
        with open(path, 'rb') as stream:
            return stream.read()
    except OSError:
        return None


def running_stack_processes(own_domain: str | None = None, proc_root: str = '/proc',
                            exclude_pids: Iterable[int] = ()) -> list[StackProcess]:
    """Stack processes in `own_domain` (default: this process's ROS_DOMAIN_ID)."""
    if own_domain is None:
        own_domain = os.environ.get('ROS_DOMAIN_ID', '').strip() or DEFAULT_DOMAIN_ID
    skip = {os.getpid(), *exclude_pids}
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return []
    found = []
    for entry in entries:
        if not entry.isdigit() or int(entry) in skip:
            continue
        raw = _read(os.path.join(proc_root, entry, 'cmdline'))
        if not raw:
            continue
        argv = split_cmdline(raw)
        if matched_executable(argv) is None:
            continue
        domain = domain_from_environ(_read(os.path.join(proc_root, entry, 'environ')))
        if conflicts(domain, own_domain):
            found.append(StackProcess(int(entry), domain, ' '.join(argv)[:200]))
    return sorted(found, key=lambda p: p.pid)


def refusal_message(stale: list[StackProcess], limit: int = 12) -> str:
    """Operator-facing text listing the processes to stop."""
    lines = ['stack processes already running in this ROS domain; clean up first '
             '(kill -TERM, then kill -9 after 2 s):']
    lines += [f'  kill -TERM {p.pid}  # domain={p.domain_id} {p.command}' for p in stale[:limit]]
    if len(stale) > limit:
        lines.append(f'  … {len(stale) - limit} more')
    return '\n'.join(lines)
