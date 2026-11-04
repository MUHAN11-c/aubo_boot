#!/usr/bin/env python3
"""真实 stereo 相机 + mock 臂重复测试，仅操作自己创建的进程组."""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import traceback
import urllib.request
import uuid

import yaml

DISPLAY_KEYS = ('DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS')


def owned_runtime_environment(base, output, domain):
    """DDS域之外也隔离spawner文件锁；BLAS 单线程，不钉 OpenMP（SGBM 需要）."""
    env = dict(base, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               ROS_HOME=str(output / 'ros_home'), AUBO_RUNS_DIR=str(output / 'records'))
    for key in ('OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        env.setdefault(key, '1')
    return env


@contextmanager
def defer_stop_signals():
    """Finish bounded cancel/disable/bag cleanup even after repeated stop requests."""
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rounds', type=int, default=10)
    p.add_argument('--models', type=int, default=0)
    p.add_argument('--reconstruction', action=argparse.BooleanOptionalAction, default=False,
                   help='本轮作业是否调用重建；默认关闭，节点仍部署供网页切换')
    p.add_argument('--soak-seconds', type=float, default=1800)
    p.add_argument('--domain', type=int, default=86)
    p.add_argument('--output', type=Path, default=Path('runs') / (
        'live_mock_' + time.strftime('%Y%m%d_%H%M%S')))
    p.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--task-params-file', type=Path,
                   help='标准 ROS 节点参数 YAML；mock/real 共用同一作业配置，空则使用产品默认')
    p.add_argument('--first-target', action='store_true',
                   help='每轮选择当前锁定集第一颗')
    p.add_argument('--pick-all', action='store_true',
                   help='每轮发 PICK_ALL/AUTO（多目标同场景，复现 drive_3 的完整 Build 轮），'
                        '不是单目标点名；全部目标达到当前作业完成档才成功')
    return p


def section(value, *keys):
    """Handle JSON objects explicitly null before first model arrival."""
    for key in keys:
        value = value.get(key) if isinstance(value, dict) else None
    return value if isinstance(value, dict) else {}


def ros_seconds(value):
    value = section(value)
    sec, ns = value.get('sec'), value.get('nanosec')
    return (float(sec) + float(ns) * 1e-9
            if isinstance(sec, (int, float)) and isinstance(ns, (int, float)) else None)


def build_generation(model):
    model = section(model)
    key = model_key(model)
    generated = ros_seconds(model.get('generated_at'))
    valid = ros_seconds(model.get('valid_until'))
    if not key or generated is None or valid is None or generated <= 0 or valid < generated:
        return None
    return key + (generated, valid)


def web_model_matches(state, model):
    proof = build_generation(model)
    if not proof:
        return False
    diagnostics = section(state, 'reconstruction', 'diagnostics')
    decision = section(state, 'reconstruction', 'grasp_decision')
    stamp = diagnostics.get('model_stamp')
    fields = ('target_id', 'scene_epoch', 'model_revision', 'tool_profile_id',
              'calibration_revision', 'config_revision')
    if not all(model.get(k) and decision.get(k) == model[k] for k in fields):
        return False
    if not model.get('run_id') or decision.get('harvest_run_id') != model['run_id']:
        return False
    times = ((stamp, proof[3]), (decision.get('stamp'), proof[3]),
             (decision.get('valid_until'), proof[4]))
    return all(isinstance(a, (int, float)) and abs(a - b) < 1e-6 for a, b in times)


def config_directory():
    local = Path(__file__).resolve().parents[1] / 'config'
    if local.exists():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('peach_system_tests')) / 'config'


def application_rviz_config():
    """Use the same application display profile as the product launch."""
    local = Path(__file__).resolve().parents[2] / 'peach_bringup/config/harvest_app.rviz'
    if local.is_file():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('peach_bringup')) / 'config/harvest_app.rviz'


def rviz_parameters(values):
    return {k: v for k, v in values.items()
            if v is not None and not (isinstance(v, (list, tuple)) and not v)}


def choose_target(targets, index):
    """
    按场景 priority（抓取顺序 1..N）选目标，含 memory/LOST 候选.

    旧实现按 target_id 排序后过滤 LOST/RETIRED；固定相机下只有前排目标
    被跟踪，索引会反复选同一颗（2026-10-22 实测 target_0→8→15→16）。
    场景 observations 已带 priority 与 candidate（LOST 时 anchor_from_memory），
    按 priority 选才是“抓取顺序”；LOST 的 memory 候选是否可执行由下游门决定。
    """
    targets = section(targets)
    observations = targets.get('observations', [])
    for item in observations:
        try:
            priority = int(item.get('priority') or 0)
        except (TypeError, ValueError):
            priority = 0
        if priority == index + 1 and item.get('target_id') and item.get('candidate'):
            return item['target_id']
    return None


def target_metadata(item):
    """Keep source identity and image/world anchors without renaming target IDs."""
    return {key: item.get(key) for key in ('target_id', 'priority', 'bbox')} | {
        'entry_position': section(item, 'candidate').get('entry_position'),
        'tracking_status': item.get('tracking_status')}


def model_key(model):
    model = section(model)
    if not all(model.get(key) for key in ('target_id', 'scene_epoch', 'model_revision')):
        return None
    return model['target_id'], model['scene_epoch'], model['model_revision']


def full_cycle(ledger, request, minimum_completion=6, expected_targets=None):
    rows = ledger.get('rows', [])
    targets = [row.get('target_id') for row in rows]
    if expected_targets is not None and (
            not expected_targets or set(targets) != set(expected_targets)
            or len(targets) != len(set(targets))):
        return False
    return ledger.get('request_id') == request and bool(rows) and all(
        r.get('outcome_name') == 'SUCCEEDED' and isinstance(r.get('completion_level'), int)
        and r['completion_level'] >= minimum_completion for r in rows)


def pregrasp_completed(ledger, request, expected_targets, state, minimum_completion):
    """Recognize successful pregrasp stopped for human recovery without ACK."""
    return (minimum_completion == 2
            and section(state, 'task_executor', 'state').get('recovery_required') is True
            and full_cycle(ledger, request, 2, expected_targets))


class RecoveryRequired(RuntimeError):
    """Carry the stopping snapshot so partial batch evidence is retained."""

    def __init__(self, state):
        super().__init__('Recovery required; stopping test for operator review')
        self.state = state


def require_no_recovery(state):
    """两种硬件模式同样等待人工恢复，测试驱动不得代替操作员 ACK."""
    task = section(state, 'task_executor', 'state')
    if task.get('recovery_required'):
        raise RecoveryRequired(state)


def required_completion(params, tool=False):
    """依据实际读回的产品参数决定验收档，缺参数不猜默认值."""
    for group in params.get('groups', []):
        if group.get('node') == 'peach_supervisor':
            for item in group.get('params', []):
                if item.get('name') == 'execute_pregrasp_only' and item.get('present'):
                    if not isinstance(item.get('value'), bool):
                        raise RuntimeError('Invalid execute_pregrasp_only parameter')
                    return 2 if item['value'] else (7 if tool else 6)
    raise RuntimeError('Missing effective execute_pregrasp_only parameter')


def stack_command(args, profile, web_port):
    """只装配环境差异，业务参数原样使用与 real 相同的作业配置."""
    command = ['ros2', 'launch', 'peach_bringup', 'harvest_system.launch.py',
               'hardware_mode:=mock', 'camera_enabled:=true', 'camera_frontend:=stereo',
               'extrinsics_enabled:=false', 'force_enabled:=false', 'autostart:=false',
               'use_sim_time:=false', 'rviz:=false', 'tool_profile:=bite_shear_v1',
               'reconstruction_enabled:=true',
               'skip_reconstruction:=' + ('false' if args.reconstruction else 'true'),
               'mock_initial_positions:=' + ','.join(map(str, profile['mock_initial_positions'])),
               'observability_port:=' + str(web_port)]
    command.append('rosbridge_port:=' + str(web_port + 1000))
    if args.task_params_file:
        command.append('task_params_file:=' + str(args.task_params_file.resolve()))
    return command


def statistics(values):
    data = sorted(float(v) for v in values
                  if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v))
    if not data:
        return {'count': 0, 'min': None, 'max': None, 'p95': None, 'range': None}
    pos = .95 * (len(data) - 1)
    low = int(pos)
    p95 = data[low] + (data[min(low + 1, len(data) - 1)] - data[low]) * (pos - low)
    return {'count': len(data), 'min': data[0], 'max': data[-1],
            'p95': p95, 'range': data[-1] - data[0]}


def startup_ready(state, domain):
    facts = section(state, 'startup', 'facts')
    task = section(state, 'task_executor', 'state')
    checks = {item.get('name'): item.get('status')
              for item in section(state, 'selfcheck', 'report').get('checks', [])}
    required = ('managed_lifecycle', 'controllers', 'joint_states_flow', 'move_group',
                'action_run_harvest', 'action_build_target_model')
    return (facts.get('hardware_mode') == 'mock' and str(facts.get('ros_domain_id')) == str(domain)
            and facts.get('camera_frontend') == 'stereo' and facts.get('camera_enabled') == 'true'
            and isinstance(task.get('state_seq'), int) and task['state_seq'] > 0
            and task.get('batch_state') in (0, 1, 6) and not task.get('action_active')
            and not task.get('recovery_required') and not task.get('blockers')
            and all(checks.get(name) == 'pass' for name in required))


class StreamEvidence:
    def __init__(self, maximum=32):
        self.stamps = deque(maxlen=maximum)
        self.total = 0
        self.last_received = 0

    def add(self, stamp):
        self.total += 1
        self.stamps.append(stamp)
        self.last_received = time.monotonic()

    def evidence(self, before):
        source_age = time.time() - self.stamps[-1] if self.stamps else None
        receive_age = time.monotonic() - self.last_received
        new_frames = self.total - before
        new_stamps = list(self.stamps)[-min(max(new_frames, 0), len(self.stamps)):]
        return {'first': self.stamps[0] if self.stamps else None,
                'last': self.stamps[-1] if self.stamps else None,
                'new_frames': new_frames,
                'source_age_s': source_age, 'receive_age_s': receive_age,
                'fresh_stamp': new_frames >= 2 and len(set(new_stamps)) > 1
                and receive_age <= 3 and source_age is not None and 0 <= source_age <= 3}


def _report_attempt_error(exc: Exception) -> None:
    """预期的重建/抓取失败只打一行；意外异常仍保留堆栈."""
    if isinstance(exc, RuntimeError):
        print(exc, flush=True)
        return
    traceback.print_exc()


def soak_passed(requested, elapsed, resources):
    return requested == 0 or (
        elapsed >= requested and bool(resources)
        and all(row.get('streams_fresh') and row.get('rviz_alive') for row in resources))


def validate_soak_sample(row):
    if row.get('rviz_alive') is not True:
        raise RuntimeError('RViz exited during soak')
    if row.get('streams_fresh') is not True:
        raise RuntimeError('camera/joint flow lost during soak')


def proc_env(pid):
    try:
        return dict(part.decode(errors='replace').split('=', 1)
                    for part in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                    if b'=' in part)
    except (OSError, ValueError):
        return {}


def gui_environment(base):
    env = dict(base)
    for key in list(env):
        if key.startswith(('QT_', 'CV2_')):
            env.pop(key)
    for path in Path('/proc').glob('[0-9]*/comm'):
        try:
            if path.read_text().strip() not in ('gnome-session', 'gnome-shell'):
                continue
        except OSError:
            continue
        source = proc_env(path.parent.name)
        env.update({key: source[key] for key in DISPLAY_KEYS if source.get(key)})
    env['QT_QPA_PLATFORM'] = 'xcb'
    if not env.get('DISPLAY'):
        raise RuntimeError('无法 RViz：没有可用 DISPLAY；明确使用 --no-rviz 才可无显示运行')
    return env


def domain_processes(domain, proc_root='/proc'):
    """Reject actual stack executables and launchers, excluding the ros2 run wrapper."""
    from peach_bringup.preflight import domain_from_environ, running_stack_pids
    found = {pid: {'pid': pid, 'command': command}
             for pid, command in running_stack_pids(str(proc_root), str(domain))}
    launch_files = {
        ('peach_bringup', 'harvest_system.launch.py'),
        ('peach_bringup', 'real_web.launch.py'),
        ('peach_system_tests', 'frozen_camera_grasp.launch.py'),
        ('aubo_e5_bringup', 'bringup.launch.py'),
    }
    for path in Path(proc_root).glob('[0-9]*/cmdline'):
        try:
            pid = int(path.parent.name)
            parts = [p.decode(errors='replace') for p in path.read_bytes().split(b'\0') if p]
            if pid == os.getpid():
                continue
            for index, part in enumerate(parts[:-3]):
                if (Path(part).name == 'ros2' and parts[index + 1] == 'launch'
                        and tuple(parts[index + 2:index + 4]) in launch_files):
                    try:
                        actual_domain = domain_from_environ(
                            (path.parent / 'environ').read_bytes())
                    except OSError:
                        actual_domain = '?'
                    if actual_domain in (str(domain), '?'):
                        found[pid] = {'pid': pid, 'command': ' '.join(parts)}
                    break
        except (OSError, ValueError):
            continue
    return list(found.values())


def launch_exit_evidence(path):
    """Audit explicit launch child exits, allowing only signal-confirmed SIGINT."""
    prefix = r'^\[(?:ERROR|INFO)\] \[([^\]]+)\]: '
    signaled = re.compile(prefix + r"sending signal 'SIGINT' to process\[([^\]]+)\]")
    died = re.compile(prefix + r'process has died \[pid (\d+), exit code (-?\d+),')
    clean = re.compile(prefix + r'process has finished cleanly \[pid (\d+)\]')
    interrupted, exits = set(), []
    with path.open(encoding='utf-8', errors='replace') as source:
        for line_number, line in enumerate(source, 1):
            line = re.sub(r'\x1b\[[0-9;]*m', '', line)
            match = signaled.match(line)
            if match and match[1] == match[2]:
                interrupted.add(match[1])
            match = died.match(line) or clean.match(line)
            if not match:
                continue
            code = int(match[3]) if len(match.groups()) == 3 else 0
            exits.append({'process': match[1], 'pid': int(match[2]), 'returncode': code,
                          'line': line_number,
                          'expected_shutdown': code == 0 or (
                              code == -signal.SIGINT and match[1] in interrupted)})
    return exits


class OwnedProcesses:
    def __init__(self, env, audit_path=None, on_start=None):
        self.env, self.processes, self.logs = env, [], []
        self.audit_path, self.on_start = audit_path, on_start
        self.audit_records = []
        self.cleanup_issues = []
        self.exit_records = []
        self.cleanup_events = []

    def start(self, command, log=None, env=None):
        handle = open(log, 'w', encoding='utf-8') if log else None
        if handle:
            self.logs.append(handle)
        process = subprocess.Popen(
            command, env=env or self.env, stdout=handle,
            stderr=subprocess.STDOUT if handle else None, start_new_session=True)
        self.processes.append(process)
        self.audit_records.append({'pid': process.pid, 'pgid': process.pid,
                                   'command': command, 'started_at': time.time(),
                                   'ros_domain_id': (env or self.env).get('ROS_DOMAIN_ID')})
        if self.audit_path:
            save(self.audit_path, {'owner_pid': os.getpid(), 'processes': self.audit_records})
        if self.on_start:
            self.on_start(process.pid)
        return process

    def signal_group(self, pgid, sig):
        if pgid not in [p.pid for p in self.processes]:
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass

    def residuals(self):
        groups = {p.pid for p in self.processes}
        found = []
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                text = path.read_text()
                fields = text[text.rfind(')') + 2:].split()
                if int(fields[2]) in groups and fields[0] != 'Z':
                    found.append(int(path.parent.name))
            except (OSError, ValueError, IndexError):
                pass
        return found

    def interrupt(self, process):
        """Let launch forward SIGINT once, so a second signal cannot break bag join."""
        if process not in self.processes:
            return
        if list(process.args)[:2] == ['ros2', 'launch'] and process.poll() is None:
            try:
                os.kill(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self.signal_group(process.pid, signal.SIGINT)

    def cleanup(self):
        exited_early = {p.pid for p in self.processes if p.poll() is not None}
        # Drain the visualization client while its MoveIt services still exist.
        # This orders teardown; it does not claim to repair native destructor crashes.
        rviz = [p for p in self.processes if p.args and Path(p.args[0]).name == 'rviz2']
        for p in rviz:
            self.interrupt(p)
        started = time.monotonic()
        while rviz and time.monotonic() - started < 15:
            live_groups = set()
            for pid in self.residuals():
                try:
                    live_groups.add(os.getpgid(pid))
                except ProcessLookupError:
                    pass
            if not any(p.pid in live_groups for p in rviz):
                break
            time.sleep(.2)
        if rviz:
            self.cleanup_events.append({
                'phase': 'rviz_before_stack', 'pgids': [p.pid for p in rviz],
                'elapsed_s': time.monotonic() - started,
                'all_exited': all(p.poll() is not None for p in rviz)})
        # SIGINT first; rosbag writers get a full 70 seconds to finalize metadata.
        for sig, budget in ((signal.SIGINT, 70), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            live_groups = set()
            for pid in self.residuals():
                try:
                    live_groups.add(os.getpgid(pid))
                except ProcessLookupError:
                    pass
            for p in reversed(self.processes):
                if sig == signal.SIGINT:
                    if p not in rviz:
                        self.interrupt(p)
                else:
                    if p.pid in live_groups:
                        self.cleanup_issues.append(
                            f'Forced {signal.Signals(sig).name} for owned group {p.pid}')
                    self.signal_group(p.pid, sig)
            deadline = time.monotonic() + budget
            while self.residuals() and time.monotonic() < deadline:
                time.sleep(.2)
            if not self.residuals():
                break
        for p in self.processes:
            try:
                p.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.cleanup_issues.append(f'Process did not exit: {p.pid}')
            self.exit_records.append({'pid': p.pid, 'returncode': p.poll(),
                                      'exited_before_cleanup': p.pid in exited_early})
            if p.pid in exited_early or p.poll() not in (0, -signal.SIGINT, 130):
                self.cleanup_issues.append(f'Unexpected process exit {p.pid}: {p.poll()}')
        for log in self.logs:
            log.close()
        return self.residuals()


_web_port = 8090


def snapshot():
    with urllib.request.urlopen(
            f'http://127.0.0.1:{_web_port}/api/state', timeout=5) as response:
        return json.load(response)


def choose_web_port():
    # Web 统一 8090-8098；8099/8100 第二套页面已删除。
    for port in range(8090, 8099):
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1', port)) != 0:
                return port
    raise RuntimeError('8090-8098 已占用；拒绝启动，不清理其他进程')


def save(path, data):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def checkpoint(output, report, phase):
    report['phase'] = phase
    report['updated_at'] = time.time()
    save(output / 'report.json', report)


class RosDriver:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        self.ros = rclpy
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('live_camera_mock_driver')
        self.active = []
        self.pending = []
        self.action_events = []
        self.target_proofs = {}
        self.stamps = {key: StreamEvidence() for key in ('camera', 'depth', 'joints')}
        from sensor_msgs.msg import Image, JointState
        from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
        # The local raw-image publishers are RELIABLE/depth=1. A BEST_EFFORT
        # acceptance reader can lose fragmented images under load even while
        # the source and perception keep advancing; match the image contract.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for key, typ, topic in (
                ('camera', Image, '/camera/color/image_raw'),
                ('depth', Image, '/camera/depth/image_raw'),
                ('joints', JointState, '/joint_states')):
            self.node.create_subscription(typ, topic, lambda msg, k=key: self.stamps[k].add(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
                qos_profile_sensor_data if key == 'joints' else image_qos)
        from peach_interfaces.msg import PeachTargetObservationArray
        self.node.create_subscription(
            PeachTargetObservationArray, '/peach/perception/target_observations',
            self._observe_targets, qos_profile_sensor_data)

    def _observe_targets(self, message):
        for item in message.observations:
            image = item.candidate_2d
            key = message.scene_epoch, item.target_id
            self.target_proofs[key] = {
                'target_id': item.target_id, 'priority': int(item.priority),
                'scene_epoch': message.scene_epoch,
                'entry_position': [item.candidate.entry_pose.position.x,
                                   item.candidate.entry_pose.position.y,
                                   item.candidate.entry_pose.position.z],
                'bbox': [image.bbox_x, image.bbox_y, image.bbox_w, image.bbox_h]
                if image.target_id == item.target_id and image.bbox_w > 0 else None,
                'image_stamp': image.header.stamp.sec + image.header.stamp.nanosec * 1e-9}
        self.target_proofs = {key: value for key, value in self.target_proofs.items()
                              if key[0] == message.scene_epoch}

    def wait(self, future, timeout):
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS request timeout; outcome unknown')
        return future.result()

    def service(self, typ, name, request, timeout=15):
        client = self.node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(name)
            return self.wait(client.call_async(request), timeout)
        finally:
            self.node.destroy_client(client)

    def enables(self, execution=False, grasp=False):
        from peach_interfaces.srv import SetEnables
        response = self.service(SetEnables, '/peach_supervisor/set_enables', SetEnables.Request(
            execution=execution, grasp=grasp, tool=False, reason='live_camera_mock owned test'))
        if not response.accepted:
            raise RuntimeError(response.message)

    def action(self, typ, name, goal, timeout, on_tick=None):
        from rclpy.action import ActionClient
        from rosidl_runtime_py.convert import message_to_ordereddict
        client = ActionClient(self.node, typ, name)
        if not client.wait_for_server(timeout_sec=30):
            client.destroy()
            raise TimeoutError(name)
        pending = (client, client.send_goal_async(goal), name)
        self.pending.append(pending)
        try:
            handle = self.wait(pending[1], 30)
        except BaseException:
            self.action_events.append({'action': name, 'event': 'acceptance_unknown',
                                       'request_id': getattr(goal, 'request_id', '')})
            # Keep the future/client alive for bounded late acceptance cleanup.
            raise
        self.pending.remove(pending)
        if not handle.accepted:
            client.destroy()
            return {'accepted': False}
        self.active.append((client, handle))
        try:
            future = handle.get_result_async()
            deadline, next_tick = time.monotonic() + timeout, 0
            while not future.done() and time.monotonic() < deadline:
                self.ros.spin_once(self.node, timeout_sec=.2)
                if on_tick and time.monotonic() >= next_tick:
                    on_tick()
                    next_tick = time.monotonic() + 1
            if not future.done():
                raise TimeoutError('ROS action timeout; outcome unknown')
            result = future.result()
            self.active.remove((client, handle))
            client.destroy()
            return {'accepted': True, 'status': result.status,
                    'result': message_to_ordereddict(result.result)}
        except BaseException:
            # Preserve the original interruption/failure even if cancellation is
            # unavailable; run() must reach cleanup rather than start another round.
            try:
                self.cancel()
            except Exception as exc:
                self.action_events.append({'action': name, 'error': str(exc)})
            raise

    def cancel(self):
        errors = []
        for client, future, name in list(getattr(self, 'pending', [])):
            try:
                handle = self.wait(future, 10)
                self.action_events.append({'action': name, 'event': 'late_acceptance',
                                           'accepted': bool(handle.accepted)})
                if handle.accepted:
                    self.active.append((client, handle))
                else:
                    client.destroy()
            except Exception as exc:
                errors.append(f'{name} late acceptance unresolved: {exc}')
                client.destroy()
            self.pending.remove((client, future, name))
        for client, handle in list(self.active):
            try:
                response = self.wait(handle.cancel_goal_async(), 10)
                terminal = self.wait(handle.get_result_async(), 15)
                self.action_events.append({
                    'event': 'cancel_terminal', 'return_code': response.return_code,
                    'status': terminal.status})
                if terminal.status not in (4, 5, 6):
                    raise RuntimeError('Cancel did not reach terminal action status')
            except Exception as exc:
                errors.append(str(exc))
            finally:
                client.destroy()
                self.active.remove((client, handle))
        if errors:
            self.action_events.append({'event': 'cleanup_failed', 'error': '; '.join(errors)})
            raise RuntimeError('; '.join(errors))

    def rviz_params(self, output):
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.parameter import parameter_value_to_python
        listed = self.service(ListParameters, '/move_group/list_parameters',
                              ListParameters.Request(depth=0))
        names = [n for n in listed.result.names
                 if n in ('robot_description', 'robot_description_semantic')
                 or n.startswith('robot_description_kinematics.')]
        result = self.service(GetParameters, '/move_group/get_parameters',
                              GetParameters.Request(names=names))
        params = rviz_parameters(dict(zip(names, map(parameter_value_to_python, result.values))))
        if not params.get('robot_description') or not params.get('robot_description_semantic'):
            raise RuntimeError('RViz model parameters unavailable')
        output.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))

    def spin_for(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.ros.spin_once(self.node, timeout_sec=min(.2, max(0, until - time.monotonic())))

    def close(self):
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def wait_state(driver, predicate, timeout=120):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        driver.spin_for(.5)
        try:
            last = snapshot()
            facts = section(last, 'startup', 'facts')
            if facts.get('hardware_mode') != 'mock':
                raise RuntimeError('Expected mock hardware; refusing operation')
            if predicate(last):
                return last
        except (OSError, ValueError):
            continue
    raise TimeoutError('State condition not reached; last revision=' + str(
        section(last, 'system').get('revision')))


def new_scene(driver, request, scene, index):
    from peach_interfaces.srv import BeginScene
    result = driver.service(BeginScene, '/peach_scene_perception_node/begin_scene',
                            BeginScene.Request(request_id=request, scene_key=scene))
    if not result.accepted:
        raise RuntimeError(result.message)
    state = wait_state(driver, lambda s: (
        section(s, 'perception', 'targets').get('scene_epoch') == result.scene_epoch
        and section(s, 'perception', 'targets').get('target_set_locked') is True
        and choose_target(s['perception']['targets'], index) is not None), 45)
    return result.scene_epoch, choose_target(state['perception']['targets'], index)


def frame_evidence(driver, before):
    return {key: stream.evidence(before[key]) for key, stream in driver.stamps.items()}


def model_geometry(model, decision):
    """Record execution entry/cut, after matching the decision to BuildResult."""
    if not model_key(model):
        raise ValueError('Geometry lacks a complete model identity')
    result = {'d95_m': decision.get('d95_m')}
    for key in ('entry', 'cut_pose', 'axis'):
        values = decision.get(key)
        if not isinstance(values, list) or len(values) != 3 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError('Geometry lacks finite ' + key)
        result[key] = values
    if (not isinstance(result['d95_m'], (int, float))
            or not math.isfinite(result['d95_m']) or result['d95_m'] <= 0):
        raise ValueError('Geometry lacks finite diameter')
    if sum(v*v for v in result['axis']) < 1e-12 or sum(
            (a-b)**2 for a, b in zip(result['entry'], result['cut_pose'])) < 1e-12:
        raise ValueError('Geometry has zero axis or travel')
    return result


def geometry_statistics(models):
    """Group by registered target; retain identities in raw data for scene auditing."""
    grouped = {}
    for item in models:
        target = item['identity'][0]
        grouped.setdefault(target, []).append(section(item, 'geometry'))
    result = {}
    for target, values in grouped.items():
        metrics = {'d95_m': statistics([g.get('d95_m') for g in values])}
        for key in ('entry', 'cut_pose'):
            for axis in range(3):
                metrics[f'{key}_{axis}_m'] = statistics([
                    g[key][axis] for g in values
                    if isinstance(g.get(key), list) and len(g[key]) >= 3])
        axes = [g['axis'] for g in values
                if isinstance(g.get('axis'), list) and len(g['axis']) == 3]
        angles = []
        if axes:
            first = axes[0]
            for other in axes:
                denom = math.sqrt(sum(x*x for x in first) * sum(x*x for x in other))
                if denom > 0:
                    cosine = sum(x*y for x, y in zip(first, other)) / denom
                    angles.append(math.degrees(math.acos(max(-1, min(1, cosine)))))
        metrics['axis_angle_from_first_deg'] = statistics(angles)
        result[target] = metrics
    return result


def physical_coverage(models, rounds):
    """Associate fixed-scene bags across per-round target renaming, within 6 cm."""
    anchors = {}
    for item in models:
        anchors.setdefault(item['identity'][0], section(item, 'geometry').get('entry'))
    counts = {target: 0 for target in anchors}
    for item in rounds:
        point = item.get('entry_position')
        if (not item.get('success')
                or not full_cycle(section(item, 'ledger'), item.get('request_id'))
                or not isinstance(point, list) or len(point) != 3):
            continue
        distances = sorted(
            (sum((a-b)**2 for a, b in zip(point, anchor)), target)
            for target, anchor in anchors.items() if isinstance(anchor, list) and len(anchor) == 3)
        if distances and distances[0][0] <= .06**2 and (
                len(distances) == 1 or distances[1][0] > .06**2):
            counts[distances[0][1]] += 1
    return counts


def run(args):
    if args.models > 0 and not args.reconstruction:
        raise SystemExit('--models > 0 requires explicit --reconstruction')
    if not (0 <= args.domain <= 232 and args.rounds >= 0
            and args.models >= 0 and args.soak_seconds >= 0):
        raise SystemExit('domain must be 0..232; counts/duration must be nonnegative')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'records').mkdir()
    cfg_dir = config_directory()
    profile = yaml.safe_load((cfg_dir / 'live_camera_mock.yaml').read_text())
    env = owned_runtime_environment(os.environ, output, args.domain)
    report = {'hardware_mode': 'mock', 'profile': profile, 'rounds': [], 'models': [],
              'model_attempts': [], 'resources': [], 'acks': [], 'errors': [],
              'rviz_requested': args.rviz, 'pick_all': args.pick_all}
    report['owned_process_groups'] = []
    report['runtime_environment'] = {key: env[key] for key in (
        'ROS_DOMAIN_ID', 'ROS_HOME', 'AUBO_RUNS_DIR',
        'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS') if key in env}
    if 'OMP_NUM_THREADS' in env:
        report['runtime_environment']['OMP_NUM_THREADS'] = env['OMP_NUM_THREADS']

    def audit_started(pgid):
        report['owned_process_groups'].append(pgid)
        save(output / 'report.json', report)
    owner = OwnedProcesses(env, output / 'owned_processes.json', on_start=audit_started)
    previous_runs_root = os.environ.get('AUBO_RUNS_DIR')
    driver = None
    try:
        stale = domain_processes(args.domain)
        if stale:
            report['preexisting_processes'] = stale
            raise RuntimeError('域内已有采摘进程；拒绝启动，不清理其他进程')
        global _web_port
        _web_port = choose_web_port()
        report['web_port'] = _web_port
        if _web_port != 8090:
            print(f'8090 已被他人占用，本轮监控改 {_web_port}', flush=True)
        gui = gui_environment(env) if args.rviz else None
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['AUBO_RUNS_DIR'] = env['AUBO_RUNS_DIR']
        if args.task_params_file and not args.task_params_file.is_file():
            raise RuntimeError('Task parameter file does not exist')
        if args.task_params_file:
            task_config = yaml.safe_load(args.task_params_file.read_text()) or {}
            for node in task_config.values():
                params = section(node, 'ros__parameters')
                for root_key in ('record', 'session'):
                    configured = params.get(root_key + '.root_dir') or section(
                        params, root_key).get('root_dir')
                    if configured:
                        raise RuntimeError('Task config must leave record/session root_dir '
                                           'empty; runner owns output/records')
        command = stack_command(args, profile, _web_port)
        owner.start(command, output / 'stack.log')
        # harvest_system 在 extrinsics_enabled:=false 时发布固定采集位，
        # 不要再起一套 static_transform_publisher 叠 camera_link。
        (output / 'camera_tf.log').write_text(
            'fixed camera mount published by harvest_system\n')
        driver = RosDriver()
        state = wait_state(driver, lambda s: startup_ready(s, args.domain))
        save(output / 'startup_state.json', state)
        with urllib.request.urlopen(
                f'http://127.0.0.1:{_web_port}/api/params', timeout=20) as response:
            effective_params = json.load(response)
        report['effective_params'] = effective_params
        report['required_completion_level'] = required_completion(effective_params, tool=False)
        if args.rviz:
            params = output / 'rviz_model_params.yaml'
            driver.rviz_params(params)
            rviz = owner.start(
                ['rviz2', '-d', str(application_rviz_config()),
                 '--ros-args', '--params-file', str(params)], output / 'rviz.log', gui)
            driver.spin_for(5)
            if rviz.poll() is not None:
                raise RuntimeError('RViz exited during startup; see rviz.log')
            report['rviz_started'] = True
        else:
            report['rviz_started'] = False
            report['rviz_note'] = '用户显式 --no-rviz；本轮未验证 RViz'
        from peach_interfaces.action import BuildTargetModel, RunHarvest

        def rviz_tick():
            if args.rviz and rviz.poll() is not None:
                raise RuntimeError('RViz exited before test completion')

        driver.enables(False, False)
        seen = set()
        sample_request = 'sample_scene_' + uuid.uuid4().hex[:12]
        sample_epoch = None
        sample_targets = []
        if args.models:
            sample_epoch, _ = new_scene(driver, sample_request, sample_request, 0)
            sample_targets = [item['target_id'] for item in sorted(
                section(snapshot(), 'perception', 'targets').get('observations', []),
                key=lambda item: int(item.get('priority') or 0))
                if item.get('target_id') and item.get('candidate') and item.get('priority')]
            report['model_target_order'] = sample_targets
        # Count completed model identities, never diagnostic heartbeat publications.
        for attempt in range(args.models):
            if len(report['models']) >= args.models:
                break
            request = 'model_' + uuid.uuid4().hex[:12]
            started = time.monotonic()
            item = {'request_id': request}
            try:
                epoch = sample_epoch
                target = sample_targets[attempt % len(sample_targets)] if sample_targets else None
                if not target:
                    raise RuntimeError('Fixed sampling scene has no current valid target')
                result = driver.action(
                    BuildTargetModel,
                    '/peach_target_reconstruction_node/build_target_model',
                    BuildTargetModel.Goal(
                        request_id=request, target_id=target, scene_epoch=epoch),
                    180, on_tick=rviz_tick)
                item.update(result)
                model = section(result, 'result', 'model')
                sample_key = build_generation(model)
                key = model_key(model)
                if (result.get('status') != 4
                        or section(result, 'result').get('success') is not True):
                    raise RuntimeError('BuildTargetModel failed: ' + str(
                        section(result, 'result').get('message')))
                if not sample_key or sample_key in seen or key[:2] != (target, epoch):
                    raise RuntimeError(
                        'BuildResult lacks new generated_at/valid_until/model identity proof')
                state = wait_state(driver, lambda s: web_model_matches(s, model), 10)
                seen.add(sample_key)
                evidence = model_geometry(
                    model, section(state, 'reconstruction', 'grasp_decision'))
                report['models'].append({
                    'identity': key, 'sample_identity': sample_key, 'model': model,
                    'target': driver.target_proofs.get((epoch, target)),
                    'geometry': evidence, 'elapsed_s': time.monotonic() - started})
                save(output / f'model_{attempt + 1}_state.json', state)
            except Exception as exc:
                item['error'] = str(exc)
                item['traceback'] = traceback.format_exc()
                _report_attempt_error(exc)
            item['elapsed_s'] = time.monotonic() - started
            report['model_attempts'].append(item)
            save(output / 'report.json', report)
            if item.get('error'):
                raise RuntimeError('Model sampling failed; no execution: ' + item['error'])
        if args.rounds:
            driver.enables(True, True)
        for index in range(args.rounds):
            request = 'round_' + uuid.uuid4().hex[:12]
            before = {key: value.total for key, value in driver.stamps.items()}
            started = time.monotonic()
            record = {'request_id': request, 'success': False}
            expected_targets = []

            def capture_targets(state):
                targets = section(state, 'perception', 'targets')
                if not targets.get('target_set_locked'):
                    return
                epoch = targets.get('scene_epoch')
                if args.pick_all and epoch != section(
                        state, 'task_executor', 'state').get('scene_epoch'):
                    return
                observations = targets.get('observations', [])
                record['locked_targets'] = [
                    target_metadata(item) | driver.target_proofs.get(
                        (epoch, item.get('target_id')), {}) for item in observations]
                record['locked_snapshot'] = targets
                if args.pick_all and not expected_targets:
                    expected_targets.extend(item['target_id'] for item in observations
                                            if item.get('target_id'))

            def tick():
                state = snapshot()
                task = section(state, 'task_executor', 'state')
                if task.get('run_id') == request:
                    if args.pick_all and not expected_targets:
                        capture_targets(state)
                    if task.get('batch_state') == 7:
                        require_no_recovery(state)
                rviz_tick()

            try:
                if args.pick_all:
                    # 复现 drive_3 的完整重建轮：不预调 BeginScene，交给
                    # supervisor 首巡 Survey/BeginScene；PICK_ALL 让它按锁定集
                    # 逐颗走完整 Build/精化，而不是单目标点名。
                    record['target_id'] = 'PICK_ALL'
                    goal = RunHarvest.Goal(
                        request_id=request, scene_key=request,
                        intent=RunHarvest.Goal.INTENT_PICK_ALL,
                        selection_mode=RunHarvest.Goal.AUTO)
                    result = driver.action(
                        RunHarvest, '/peach_supervisor/run_harvest', goal, 300,
                        on_tick=tick)
                else:
                    epoch, target = new_scene(
                        driver, request, request, 0 if args.first_target else index)
                    # 场景源 ID 就是 2D/3D/账本 ID；不再另造 grasp_id 重映射。
                    record['requested_slot'] = 1 if args.first_target else index + 1
                    record.update(target_id=target, scene_epoch=epoch)
                    selected_state = snapshot()
                    capture_targets(selected_state)
                    observations = section(selected_state, 'perception', 'targets').get(
                        'observations', [])
                    selected = next((o for o in observations if o.get('target_id') == target), {})
                    record.update(target_metadata(selected) | driver.target_proofs.get(
                        (epoch, target), {}))
                    record['grasp_slot'] = record.get('priority')
                    expected_targets.append(target)
                    result = driver.action(
                        RunHarvest, '/peach_supervisor/run_harvest',
                        RunHarvest.Goal(request_id=request, scene_key=request, intent=1,
                                        selection_mode=1, target_ids=[target]), 300,
                        on_tick=tick)
                record['action'] = result
                state = wait_state(driver, lambda s: (
                    section(s, 'ledger', 'live').get('request_id') == request
                    and bool(section(s, 'ledger', 'live').get('rows'))), 10)
                ledger = section(state, 'ledger', 'live')
                record['ledger'] = ledger
                record['success'] = (
                    result.get('status') == 4
                    and section(result, 'result').get('success') is True
                    and full_cycle(ledger, request, report['required_completion_level'],
                                   expected_targets))
                save(output / f'round_{index + 1}_state.json', state)
            except RecoveryRequired as exc:
                report['recovery_required'] = True
                record['recovery_required'] = True
                record['action'] = {'interrupted_for_recovery': True,
                                    'terminal_events': driver.action_events[-3:]}
                record['stopping_state'] = exc.state
                record['stop_reason'] = 'operator_recovery_required'
            except (Exception, KeyboardInterrupt) as exc:
                record['error'] = str(exc)
                record['traceback'] = traceback.format_exc()
                _report_attempt_error(exc)
            # Failure/cancel/recovery must retain the same request's final ledger too.
            try:
                state = wait_state(driver, lambda s: (
                    section(s, 'ledger', 'live').get('request_id') == request
                    and bool(section(s, 'ledger', 'live').get('rows'))), 10)
                record['ledger'] = section(state, 'ledger', 'live')
                if not record.get('locked_targets'):
                    capture_targets(state)
                record['expected_targets'] = expected_targets
                if record.get('recovery_required'):
                    record['pregrasp_completed'] = pregrasp_completed(
                        record['ledger'], request, expected_targets,
                        record['stopping_state'], report['required_completion_level'])
                    record['success'] = record['pregrasp_completed']
                save(output / f'round_{index + 1}_state.json', state)
            except Exception as exc:
                record['ledger_error'] = str(exc)
                record['success'] = False
                ledger_file = output / 'records' / request / 'ledger.json'
                if ledger_file.is_file():
                    try:
                        record['persisted_ledger'] = json.loads(ledger_file.read_text())
                    except (OSError, ValueError) as read_error:
                        record['ledger_error'] += '; disk: ' + str(read_error)
            record['elapsed_s'] = time.monotonic() - started
            record['expected_targets'] = expected_targets
            # 证据采样前让驱动相机回调追上；3 s 新鲜门不放宽（真停流仍 false）。
            driver.spin_for(0.5)
            record['streams'] = frame_evidence(driver, before)
            if not all(value['fresh_stamp'] for value in record['streams'].values()):
                record['success'] = False
            report['rounds'].append(record)
            save(output / 'report.json', report)
            if not record['success']:
                raise RuntimeError('Round failed; subsequent rounds stopped')
            if record.get('recovery_required'):
                report['recovery_required'] = True
                report['stop_reason'] = 'operator_recovery_required'
                # A normal level2 validates this request, not any remaining requests.
                break
        driver.enables(False, False)
        soak_started = time.monotonic()
        report['soak_started_monotonic'] = soak_started
        until = soak_started + (0 if report.get('recovery_required') else args.soak_seconds)
        while time.monotonic() < until:
            soak_before = {key: value.total for key, value in driver.stamps.items()}
            driver.spin_for(min(5, max(0, until - time.monotonic())))
            state = snapshot()
            report['resources'].append({
                'monotonic': time.monotonic(), 'metrics': state.get('metrics'),
                'topic_age_s': section(state, 'system').get('topic_age_s'),
                'streams_fresh': all(
                    v['fresh_stamp'] for v in frame_evidence(driver, soak_before).values()),
                'streams': frame_evidence(driver, soak_before),
                'rviz_alive': not args.rviz or rviz.poll() is None})
            report['soak_elapsed_s'] = time.monotonic() - soak_started
            checkpoint(output, report, 'soaking')
            validate_soak_sample(report['resources'][-1])
        rviz_tick()
        report['rviz_lifetime_ok'] = True
        report['soak_elapsed_s'] = time.monotonic() - soak_started
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(str(exc) or type(exc).__name__)
    finally:
        with defer_stop_signals():
            checkpoint(output, report, 'cleaning')
            if driver:
                cleanup_driver(driver, report['errors'])
            checkpoint(output, report, 'stopping_processes')
            report['owned_process_groups'] = [p.pid for p in owner.processes]
            report['residual_pids'] = owner.cleanup()
            report['process_exits'] = owner.exit_records
            report['process_cleanup_events'] = getattr(owner, 'cleanup_events', [])
            report['errors'].extend(owner.cleanup_issues)
            stack_log = output / 'stack.log'
            if stack_log.is_file():
                try:
                    report['launch_child_exits'] = launch_exit_evidence(stack_log)
                    report['errors'].extend(
                        f"Launch child {row['process']} exited {row['returncode']} "
                        f"(stack.log:{row['line']})"
                        for row in report['launch_child_exits'] if not row['expected_shutdown'])
                except OSError as exc:
                    report['errors'].append('Cannot audit stack child exits: ' + str(exc))
            report['action_events'] = driver.action_events if driver else []
            report['errors'].extend(event['error'] for event in report['action_events']
                                    if event.get('error'))
            report['record_directories'] = sorted(
                child.name for child in (output / 'records').iterdir() if child.is_dir())
            bag_seals = []
            for bag in (output / 'records').glob('session_*/bag'):
                sealed = (bag / 'metadata.yaml').is_file()
                proof = {'path': str(bag), 'metadata_present': sealed}
                if sealed:
                    try:
                        metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
                        files = section(metadata, 'rosbag2_bagfile_information').get(
                            'relative_file_paths', [])
                        proof['storage_files_present'] = bool(files) and all(
                            (bag / name).is_file() for name in files)
                        sealed = proof['storage_files_present']
                    except (OSError, ValueError, yaml.YAMLError) as exc:
                        sealed = False
                        proof['error'] = str(exc)
                bag_seals.append(proof)
                if not sealed:
                    report['errors'].append('Unsealed recording: ' + str(bag))
            report['bag_seals'] = bag_seals
            report['domain_processes_after'] = domain_processes(args.domain)
            report['model_count'] = len(report['models'])
            report['successful_rounds'] = sum(r['success'] for r in report['rounds'])
            report['unattempted_rounds'] = args.rounds - len(report['rounds'])
            if report['unattempted_rounds']:
                report['errors'].append(
                    f"Unattempted requested rounds: {report['unattempted_rounds']}")
            report['full_level6_rounds'] = sum(
                full_cycle(section(r, 'ledger'), r['request_id']) for r in report['rounds'])
            report['model_elapsed_s'] = statistics([m['elapsed_s'] for m in report['models']])
            report['geometry_statistics'] = geometry_statistics(report['models'])
            groups = {}
            for model in report['models']:
                target = model['identity'][0]
                groups[target] = groups.get(target, 0) + 1
            report['model_samples_per_target'] = groups
            if args.models >= 3 and any(count < 3 for count in groups.values()):
                report['errors'].append(
                    'Insufficient repeat samples: each observed target requires at least 3 models')
            counts = {}
            for round_record in report['rounds']:
                for row in section(round_record, 'ledger').get('rows', []):
                    target_counts = counts.setdefault(row.get('target_id', 'unknown'), {})
                    outcome = row.get('outcome_name', 'UNKNOWN')
                    target_counts[outcome] = target_counts.get(outcome, 0) + 1
            report['per_target_outcomes'] = counts
            report['physical_bag_level6_coverage'] = physical_coverage(
                report['models'], report['rounds'])
            if (report.get('required_completion_level', 6) >= 6
                    and args.rounds >= 6 and len(groups) == 2 and any(
                        count < 3 for count in report['physical_bag_level6_coverage'].values())):
                report['errors'].append('Both modeled bags require at least 3 level6 rounds each')
            report['soak_passed'] = soak_passed(
                args.soak_seconds, report.get('soak_elapsed_s', 0), report['resources'])
            if not report['soak_passed']:
                report['errors'].append('Soak incomplete or camera/joint/RViz liveness failed')
            report['passed'] = (
                report.get('rviz_lifetime_ok', False) and not report['errors']
                and not report['residual_pids'] and report['model_count'] == args.models
                and report['successful_rounds'] == args.rounds)
            checkpoint(output, report, 'finished')
            if previous_runs_root is None:
                os.environ.pop('AUBO_RUNS_DIR', None)
            else:
                os.environ['AUBO_RUNS_DIR'] = previous_runs_root
        print(json.dumps({'output': str(output), 'passed': report['passed'],
                          'errors': report['errors']}, ensure_ascii=False))
    return 0 if report['passed'] else 1


def cleanup_driver(driver, errors):
    # Keep context live for both independent operations before destroying it.
    for name, operation in (('cancel', driver.cancel),
                            ('disable', lambda: driver.enables(False, False)),
                            ('ROS shutdown', driver.close)):
        try:
            operation()
        except Exception as exc:
            errors.append(name + ': ' + str(exc))


def main(argv=None):
    args = parser().parse_args(argv)
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}

    def interrupt(signum, frame):
        raise KeyboardInterrupt('received signal ' + str(signum))
    try:
        for sig in previous:
            signal.signal(sig, interrupt)
        return run(args)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
