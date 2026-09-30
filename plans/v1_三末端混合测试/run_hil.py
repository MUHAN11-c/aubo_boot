#!/usr/bin/env python3
"""v1 三末端混合测试：真实立体相机在环 + mock 臂一次完整抓取.

门：一次 ExecuteTarget FULL 干跑墙钟 ≤ 120 s（不含栈启动）。
不动真机、不 SetIO、hardware_mode:=mock。DDS 域 71，只清本域。
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path('/home/mu/Desktop/aubo_e5_jazzy_ws')
OUT = ROOT / 'plans' / 'v1_三末端混合测试' / 'results'
DOMAIN = '71'
GRASP_LIMIT_S = 120.0
PROFILES = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')
TCP_XYZ = {
    'shear_v1': (0.0, 0.0479, 0.15107),
    'bite_shear_v1': (0.0, 0.03024, 0.1655),
    'adaptive_shear_v1': (0.0, 0.047, 0.16866),
}
PGREP_PAT = (
    'ros2 launch|component_container|extrinsics_publisher|ros2 run|'
    'bag record|collect|probe|move_group|ros2_control_node|'
    'robot_state_publisher|lifecycle_manager|peach_|imu_follow|'
    'sim_field_targets|harvest_system'
)
MANAGED = (
    'peach_scene_perception_node',
    'peach_target_reconstruction_node',
    'peach_supervisor',
    'peach_arm',
)


def env() -> dict[str, str]:
    merged = os.environ.copy()
    merged['ROS_DOMAIN_ID'] = DOMAIN
    merged['ROS_LOCALHOST_ONLY'] = '0'
    merged['QT_QPA_PLATFORM'] = 'offscreen'
    merged['AUBO_RUNS_DIR'] = str(OUT / 'runs')
    return merged


def bash(cmd: str, timeout: float) -> subprocess.CompletedProcess:
    wrapped = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {ROOT}/install/setup.bash && {cmd}'
    )
    return subprocess.run(
        ['bash', '-lc', wrapped],
        cwd=str(ROOT),
        env=env(),
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def domain_pids() -> list[tuple[int, str]]:
    proc = subprocess.run(
        ['bash', '-lc', f"pgrep -af '{PGREP_PAT}' || true"],
        text=True, capture_output=True)
    found: list[tuple[int, str]] = []
    for line in proc.stdout.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        cmd = parts[1]
        if any(x in cmd for x in (
                'run_hil.py', 'retry_shear_adaptive.py', 'retry_sim_full.py',
                'pgrep ')):
            continue
        try:
            data = Path(f'/proc/{pid}/environ').read_bytes()
        except OSError:
            continue
        items = {x.decode('utf-8', 'replace') for x in data.split(b'\0') if x}
        if f'ROS_DOMAIN_ID={DOMAIN}' in items:
            found.append((pid, cmd[:180]))
    return found


def stop_domain() -> None:
    for _round in range(2):
        for pid, _cmd in domain_pids():
            sig = signal.SIGTERM if _round == 0 else signal.SIGKILL
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(2.0 if _round == 0 else 1.0)
        if not domain_pids():
            return


def start_stack(profile: str, log_path: Path, camera_enabled: bool = True):
    imu = 'true' if profile == 'adaptive_shear_v1' else 'false'
    cam = 'true' if camera_enabled else 'false'
    cmd = (
        f'source /opt/ros/jazzy/setup.bash && source {ROOT}/install/setup.bash && '
        f'ros2 launch peach_bringup harvest_system.launch.py '
        f'hardware_mode:=mock camera_enabled:={cam} camera_frontend:=stereo '
        f'skip_reconstruction:=true tool_profile:={profile} '
        f'autostart:=false imu_enabled:={imu} moveit_enabled:=true'
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_path, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        ['bash', '-lc', cmd],
        cwd=str(ROOT),
        env=env(),
        stdout=handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return proc, handle


def wait_lifecycle(seconds: float) -> dict[str, str]:
    deadline = time.time() + seconds
    last: dict[str, str] = {}
    while time.time() < deadline:
        ok = True
        for name in MANAGED:
            try:
                proc = bash(f'ros2 lifecycle get /{name}', 8)
            except subprocess.TimeoutExpired:
                last[name] = 'timeout'
                ok = False
                continue
            text = (proc.stdout or '') + (proc.stderr or '')
            last[name] = text.strip().replace('\n', ' ')[:120]
            if 'active [3]' not in text:
                ok = False
        if ok:
            return last
        time.sleep(3.0)
    return last


def topic_hz(topic: str, seconds: int = 5) -> str:
    try:
        proc = bash(f'timeout {seconds} ros2 topic hz {topic}', seconds + 4)
    except subprocess.TimeoutExpired:
        return 'timeout'
    return ((proc.stdout or '') + (proc.stderr or '')).strip()[-350:]


def param_get(node: str, name: str) -> str:
    try:
        proc = bash(f'timeout 8 ros2 param get /{node} {name}', 12)
    except subprocess.TimeoutExpired:
        return 'timeout'
    return ((proc.stdout or '') + (proc.stderr or '')).strip()[:200]


def tf_echo(a: str, b: str) -> str:
    try:
        proc = bash(f'timeout 5 ros2 run tf2_ros tf2_echo {a} {b}', 9)
    except subprocess.TimeoutExpired:
        return 'timeout'
    return ((proc.stdout or '') + (proc.stderr or '')).strip()[-500:]


def service_list(substr: str) -> str:
    try:
        proc = bash('timeout 8 ros2 service list', 12)
    except subprocess.TimeoutExpired:
        return 'timeout'
    lines = [ln for ln in (proc.stdout or '').splitlines() if substr in ln]
    return '\n'.join(lines) if lines else '(none)'


def run_one_full_grasp(profile: str, log_path: Path) -> dict:
    """一次完整抓取：sim_field_targets FULL / case 1757 / velocity=1.0，上限 120 s."""
    cmd = (
        f'timeout -s INT -k 10 {int(GRASP_LIMIT_S)} python3 scripts/sim_field_targets.py '
        f'--mode full --case 1757 --velocity 1.0 --tool-profile {profile}'
    )
    t0 = time.time()
    try:
        proc = bash(cmd, GRASP_LIMIT_S + 15.0)
        rc = proc.returncode
        text = ((proc.stdout or '') + '\n' + (proc.stderr or '')).strip()
    except subprocess.TimeoutExpired as exc:
        rc = -9
        text = f'timeout_expired:{exc}'
    elapsed = time.time() - t0
    log_path.write_text(text[-12000:], encoding='utf-8')
    passed = (
        rc == 0 and elapsed <= GRASP_LIMIT_S and
        'FULL 干跑成功 1/1' in text)
    if 'goal 超时' in text or 'goal 被拒' in text:
        passed = False
    return {
        'elapsed_s': round(elapsed, 2),
        'limit_s': GRASP_LIMIT_S,
        'returncode': rc,
        'within_2min': elapsed <= GRASP_LIMIT_S,
        'log': str(log_path),
        'tail': text[-2500:],
        'passed_gate': passed and elapsed <= GRASP_LIMIT_S,
    }


def stop_launch(proc: subprocess.Popen, handle) -> int:
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()
        proc.wait(timeout=5)
    handle.close()
    stop_domain()
    return proc.returncode if proc.returncode is not None else -1


def one_profile(profile: str) -> dict:
    print(f'== {profile} 真相机+FULL≤{int(GRASP_LIMIT_S)}s', flush=True)
    log = OUT / f'{profile}_launch.log'
    proc, handle = start_stack(profile, log)
    item: dict = {'profile': profile, 'launch_pid': proc.pid}
    try:
        item['lifecycle'] = wait_lifecycle(120.0)
        item['tool_profile_id'] = param_get('peach_arm', 'tool.profile_id')
        item['skip_reconstruction'] = param_get(
            'peach_supervisor', 'skip_reconstruction')
        item['allow_unrefined'] = param_get(
            'peach_arm', 'quality.allow_unrefined_geometry')
        item['color_hz'] = topic_hz('/camera/color/image_raw', 5)
        item['depth_hz'] = topic_hz('/camera/depth/image_raw', 5)
        item['tf_tcp'] = tf_echo('wrist3_Link', 'tcp')
        item['tf_cam'] = tf_echo('wrist3_Link', 'camera_link')
        item['expected_tcp'] = TCP_XYZ[profile]
        item['imu_follow_srvs'] = service_list('/imu_follow/')
        item['controllers'] = (
            bash('timeout 8 ros2 control list_controllers', 12).stdout or '')[:600]
        grasp_log = OUT / f'{profile}_grasp_1757.txt'
        item['grasp'] = run_one_full_grasp(profile, grasp_log)
    finally:
        item['launch_exit'] = stop_launch(proc, handle)
        item['leftover'] = domain_pids()
    return item


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'runs').mkdir(parents=True, exist_ok=True)
    summary = {
        'started': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'domain': DOMAIN,
        'camera_ip': '169.254.10.110',
        'hardware_mode': 'mock',
        'grasp_limit_s': GRASP_LIMIT_S,
        'case': '1757',
        'mode': 'FULL dry sleeve, tool off, skip_observation',
        'note': '真相机 stereo 在环；mock 臂一次完整抓取墙钟≤120s；不开刀；不 real',
        'profiles': [],
    }
    print('pre-clean domain 71', flush=True)
    stop_domain()
    ping = subprocess.run(
        ['ping', '-c', '1', '-W', '1', '169.254.10.110'],
        capture_output=True, text=True)
    summary['camera_ping'] = ping.returncode == 0
    summary['camera_ping_out'] = (ping.stdout or ping.stderr)[:300]
    results = []
    for profile in PROFILES:
        try:
            item = one_profile(profile)
        except Exception as exc:  # noqa: BLE001
            item = {'profile': profile, 'error': repr(exc)}
            stop_domain()
        (OUT / f'{profile}.json').write_text(
            json.dumps(item, ensure_ascii=False, indent=2), encoding='utf-8')
        results.append(item)
        summary['profiles'].append({
            'id': profile,
            'elapsed_s': (item.get('grasp') or {}).get('elapsed_s'),
            'within_2min': (item.get('grasp') or {}).get('within_2min'),
            'passed_gate': (item.get('grasp') or {}).get('passed_gate'),
        })
    leftover = domain_pids()
    summary['finished'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    summary['domain_leftover'] = leftover
    summary['all_within_2min'] = all(
        (p.get('within_2min') is True) for p in summary['profiles'])
    (OUT / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    gates = [r.get('grasp', {}).get('passed_gate') for r in results]
    return 0 if leftover == [] and all(gates) else 2


if __name__ == '__main__':
    sys.exit(main())
