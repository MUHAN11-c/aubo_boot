"""启动预检（零 ROS）：拒重复栈实例."""
from __future__ import annotations

import os

PREFLIGHT_PATTERNS = (
    # brain 进程 exec=peach_harvester 承载场景/重建/调度三托管节点（launch
    # 不传 name= 重映射，节点名不进 argv），必须按 exec 名匹配——否则残留
    # 旧脑时双 supervisor 静默共存（G5，2026-09-20 端到端审查）。
    'peach_harvester',
    'peach_scene_perception_node',
    'peach_target_reconstruction_node',
    'peach_arm',
    'peach_supervisor',
    'peach_observability',
    'peach_lifecycle_flag_bridge',
    'peach_autostart_client',
    'peach_lifecycle_manager',
    # nav2 可执行文件 basename；节点名 peach_lifecycle_manager 只在
    # -r __node:= 里，按 argv0 匹配必须用 lifecycle_manager，否则停栈漏杀。
    'lifecycle_manager',
    'component_container',
    'move_group',
    'robot_state_publisher',
    'ros2_control_node',
    'controller_manager',
    'joint_state_publisher',
    'joint_state_publisher_gui',
    'extrinsics_publisher',
    'serial_imu_node',
    'stereo_camera_node',
    'imu_follow_node',
    'servo_node',
    'rviz2',
)


def executable_basenames(cmdline: str):
    """返回 argv 各段的 basename；不用整串子串，避免编辑器路径误伤."""
    names = []
    for part in cmdline.split('\x00'):
        if not part:
            continue
        names.append(os.path.basename(part.rstrip('/')))
    return names


def cmdline_looks_like_stack(cmdline: str) -> bool:
    """判断 cmdline 是否为栈节点可执行文件，而不是 colcon/pytest 参数名."""
    if 'harvest_system.launch' in cmdline:
        return False
    if 'cursorsandbox' in cmdline:
        return False
    parts = [part for part in cmdline.split('\x00') if part]
    if not parts:
        return False
    names = executable_basenames(cmdline)
    if names[0] in PREFLIGHT_PATTERNS:
        return True
    for part in parts[1:]:
        base = os.path.basename(part.rstrip('/'))
        if base in PREFLIGHT_PATTERNS and '/lib/' in part.replace('\\', '/'):
            return True
    return False


def running_stack_pids(proc_root: str = '/proc'):
    """返回 (pid, cmdline) 列表：仍在跑的栈节点进程."""
    found = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            cmd_path = os.path.join(proc_root, entry, 'cmdline')
            with open(cmd_path, 'rb') as stream:
                cmdline = stream.read().decode('utf-8', 'replace')
        except OSError:
            continue
        if cmdline_looks_like_stack(cmdline):
            display = ' '.join(part for part in cmdline.split('\x00') if part)
            found.append((int(entry), display[:200]))
    return found


def runs_disk_free_gb(runs_root: str) -> float | None:
    """返回 runs 根所在盘剩余空间 [GB]；不可得给 None（不阻断）."""
    target = os.path.abspath(runs_root or 'runs')
    probe = target if os.path.isdir(target) else os.path.dirname(target)
    try:
        st = os.statvfs(probe)
    except OSError:
        return None
    return st.f_bavail * st.f_frsize / (1 << 30)


def check_disk_free(min_free_gb: float, runs_root: str = 'runs') -> str | None:
    """磁盘余量预检：低于阈值返回拒绝理由，否则 None（bag 预算安全垫）."""
    free = runs_disk_free_gb(runs_root)
    if free is None:
        return None
    if free < min_free_gb:
        return (
            f'runs 根所在盘剩余 {free:.1f}GB < {min_free_gb:.1f}GB'
            '（会话 bag 无落地空间，先清理或改 record.root_dir）')
    return None
