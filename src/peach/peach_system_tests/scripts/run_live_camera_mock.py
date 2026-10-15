#!/usr/bin/env python3
"""真实 stereo 相机 + mock 臂重复测试。仅操作自己创建的进程组。"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
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
    """DDS域之外也隔离spawner文件锁；测试默认限制数值库过量并行."""
    env = dict(base, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               ROS_HOME=str(output / 'ros_home'))
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
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
    p.add_argument('--models', type=int, default=30)
    p.add_argument('--soak-seconds', type=float, default=1800)
    p.add_argument('--domain', type=int, default=86)
    p.add_argument('--output', type=Path, default=Path('runs') / ('live_mock_' + time.strftime('%Y%m%d_%H%M%S')))
    p.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=True)
    return p



def section(value, *keys):
    """Optional JSON objects may be explicitly null before first model arrival."""
    for key in keys:
        value = value.get(key) if isinstance(value, dict) else None
    return value if isinstance(value, dict) else {}


def ros_seconds(value):
    value = section(value)
    sec, ns = value.get('sec'), value.get('nanosec')
    return float(sec) + float(ns) * 1e-9 if isinstance(sec, (int, float)) and isinstance(ns, (int, float)) else None


def build_generation(model):
    model = section(model)
    key = model_key(model)
    generated, valid = ros_seconds(model.get('generated_at')), ros_seconds(model.get('valid_until'))
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


def rviz_parameters(values):
    return {k: v for k, v in values.items() if v is not None and not (isinstance(v, (list, tuple)) and not v)}


def choose_target(targets, index):
    targets = section(targets)
    ids = sorted({t['target_id'] for t in targets.get('observations', [])
                  if t.get('target_id') and t.get('tracking_status') not in ('LOST', 'RETIRED')})
    return ids[index % len(ids)] if ids else None


def model_key(model):
    model = section(model)
    if not model.get('target_id') or not model.get('scene_epoch') or not model.get('model_revision'):
        return None
    return model['target_id'], model['scene_epoch'], model['model_revision']


def full_cycle(ledger, request):
    rows = ledger.get('rows', [])
    return ledger.get('request_id') == request and bool(rows) and all(
        r.get('outcome_name') == 'SUCCEEDED' and isinstance(r.get('completion_level'), int)
        and r['completion_level'] >= 6 for r in rows)


def can_ack(mode, request, state, terminal):
    return mode == 'mock' and terminal and state.get('run_id') == request and state.get('recovery_required') is True


def statistics(values):
    data = sorted(float(v) for v in values if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v))
    if not data:
        return {'count': 0, 'min': None, 'max': None, 'p95': None, 'range': None}
    pos = .95 * (len(data) - 1)
    low = int(pos)
    p95 = data[low] + (data[min(low + 1, len(data) - 1)] - data[low]) * (pos - low)
    return {'count': len(data), 'min': data[0], 'max': data[-1], 'p95': p95, 'range': data[-1] - data[0]}



def startup_ready(state, domain):
    facts = section(state, 'startup', 'facts')
    task = section(state, 'task_executor', 'state')
    checks = {item.get('name'): item.get('status') for item in section(state, 'selfcheck', 'report').get('checks', [])}
    required = ('managed_lifecycle', 'controllers', 'joint_states_flow', 'move_group',
                'action_run_harvest', 'action_build_target_model')
    return (facts.get('hardware_mode') == 'mock' and str(facts.get('ros_domain_id')) == str(domain)
            and facts.get('camera_frontend') == 'stereo' and facts.get('camera_enabled') == 'true'
            and isinstance(task.get('state_seq'), int) and task['state_seq'] > 0
            and task.get('batch_state') in (0, 1, 6) and not task.get('action_active')
            and not task.get('recovery_required') and not task.get('blockers')
            and all(checks.get(name) == 'pass' for name in required))


def ack_evidence(state, request, target):
    task = section(state, 'task_executor', 'state')
    arm = section(state, 'manipulation', 'status')
    if (not can_ack(section(state, 'startup', 'facts').get('hardware_mode'), request, task, True)
            or arm.get('running') is not False or not target or task.get('target_id') != target):
        return None
    ledger = section(state, 'ledger', 'live')
    if ledger.get('request_id') == request:
        for row in ledger.get('rows', []):
            if row.get('target_id') == target and row.get('outcome_name') in ('SUCCEEDED', 'FAILED', 'CANCELED', 'SKIPPED_QUALITY', 'SKIPPED_UNREACHABLE'):
                return {'source': 'ledger', 'completion_level': row.get('completion_level')}
    # Historical 20261014 supervisor flushes ledger only after ACK; canonical
    # target terminal event is already emitted. Do not fabricate a ledger row.
    for event in reversed(section(state, 'task_executor').get('events', [])):
        if event.get('request_id') != request or event.get('run_id') != request or event.get('target_id') != target:
            continue
        if event.get('code') != 'target_succeeded' or arm.get('state') != 'SUCCEEDED':
            continue
        try:
            details = json.loads(event.get('message', '{}'))
        except ValueError:
            continue
        if isinstance(details.get('completion_level'), int) and details['completion_level'] >= 6:
            return {'source': 'canonical_target_succeeded', 'completion_level': details['completion_level'],
                    'event_state_seq': event.get('state_seq')}
    return None


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
        return {'first': self.stamps[0] if self.stamps else None,
                'last': self.stamps[-1] if self.stamps else None,
                'new_frames': self.total - before,
                'fresh_stamp': self.total - before >= 2 and len(set(self.stamps)) > 1
                and time.monotonic() - self.last_received <= 3}


def soak_passed(requested, elapsed, resources):
    return requested == 0 or (elapsed >= requested and bool(resources)
        and all(row.get('streams_fresh') and row.get('rviz_alive') for row in resources))


def validate_soak_sample(row):
    if row.get('rviz_alive') is not True:
        raise RuntimeError('RViz exited during soak')
    if row.get('streams_fresh') is not True:
        raise RuntimeError('camera/joint flow lost during soak')

def proc_env(pid):
    try:
        return dict(part.decode(errors='replace').split('=', 1) for part in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0') if b'=' in part)
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


def domain_processes(domain):
    found = []
    markers = ('peach_', 'ros2 launch', 'component_container', 'move_group', 'extrinsics_publisher', 'robot_state_publisher', 'ros2_control_node')
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            pid = int(path.parent.name)
            command = path.read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if pid != os.getpid() and any(m in command for m in markers) and proc_env(pid).get('ROS_DOMAIN_ID', '0') == str(domain):
                found.append({'pid': pid, 'command': command})
        except (OSError, ValueError):
            pass
    return found


class OwnedProcesses:
    def __init__(self, env, audit_path=None, on_start=None):
        self.env, self.processes, self.logs = env, [], []
        self.audit_path, self.on_start = audit_path, on_start
        self.audit_records = []

    def start(self, command, log=None, env=None):
        handle = open(log, 'w', encoding='utf-8') if log else None
        if handle:
            self.logs.append(handle)
        process = subprocess.Popen(command, env=env or self.env, stdout=handle,
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
        # SIGINT first; rosbag writers get a full 70 seconds to finalize metadata.
        for sig, budget in ((signal.SIGINT, 70), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            for p in reversed(self.processes):
                if sig == signal.SIGINT:
                    self.interrupt(p)
                else:
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
                pass
        for log in self.logs:
            log.close()
        return self.residuals()


def snapshot():
    with urllib.request.urlopen('http://127.0.0.1:8090/api/state', timeout=5) as response:
        return json.load(response)


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
        self.stamps = {'camera': StreamEvidence(), 'joints': StreamEvidence()}
        from sensor_msgs.msg import Image, JointState
        from rclpy.qos import qos_profile_sensor_data
        for key, typ, topic in (('camera', Image, '/camera/color/image_raw'), ('joints', JointState, '/joint_states')):
            self.node.create_subscription(typ, topic, lambda msg, k=key: self.stamps[k].add(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9), qos_profile_sensor_data)

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
        handle = self.wait(client.send_goal_async(goal), 30)
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
            return {'accepted': True, 'status': result.status, 'result': message_to_ordereddict(result.result)}
        except BaseException:
            # Preserve the original interruption/failure even if cancellation is
            # unavailable; run() must reach cleanup rather than start another round.
            try:
                self.wait(handle.cancel_goal_async(), 10)
            except Exception:
                pass
            raise

    def cancel(self):
        for client, handle in self.active:
            try:
                self.wait(handle.cancel_goal_async(), 10)
            except Exception:
                pass
            client.destroy()
        self.active.clear()

    def rviz_params(self, output):
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.parameter import parameter_value_to_python
        listed = self.service(ListParameters, '/move_group/list_parameters', ListParameters.Request(depth=0))
        names = [n for n in listed.result.names if n in ('robot_description', 'robot_description_semantic') or n.startswith('robot_description_kinematics.')]
        result = self.service(GetParameters, '/move_group/get_parameters', GetParameters.Request(names=names))
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
    raise TimeoutError('State condition not reached; last revision=' + str(section(last, 'system').get('revision')))


def new_scene(driver, request, scene, index):
    from peach_interfaces.srv import BeginScene
    result = driver.service(BeginScene, '/peach_scene_perception_node/begin_scene',
                            BeginScene.Request(request_id=request, scene_key=scene))
    if not result.accepted:
        raise RuntimeError(result.message)
    state = wait_state(driver, lambda s: section(s, 'perception', 'targets').get('scene_epoch') == result.scene_epoch
                       and choose_target(s['perception']['targets'], index) is not None, 45)
    return result.scene_epoch, choose_target(state['perception']['targets'], index)


def frame_evidence(driver, before):
    return {key: stream.evidence(before[key]) for key, stream in driver.stamps.items()}


def ack_tick(driver, request, target, records):
    state = snapshot()
    proof = ack_evidence(state, request, target)
    if not proof:
        return
    # Retry only a stale-sequence rejection, taking a fresh snapshot each tick.
    previous = [item for item in records if item.get('request_id') == request]
    if any(item.get('accepted') for item in previous) or len(previous) >= 3:
        return
    from peach_interfaces.srv import ControlTask
    task = state['task_executor']['state']
    reply = driver.service(ControlTask, '/peach_supervisor/control', ControlTask.Request(
        request_id=request, command=6, expected_state_seq=task['state_seq'],
        reason='mock only; current target terminal evidence verified'))
    records.append({'request_id': request, 'state_seq': task['state_seq'],
                    'accepted': reply.accepted, 'message': reply.message, 'evidence': proof})


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
    if not isinstance(result['d95_m'], (int, float)) or not math.isfinite(result['d95_m']) or result['d95_m'] <= 0:
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
                    g[key][axis] for g in values if isinstance(g.get(key), list) and len(g[key]) >= 3])
        axes = [g['axis'] for g in values if isinstance(g.get('axis'), list) and len(g['axis']) == 3]
        angles = []
        if axes:
            first = axes[0]
            for other in axes:
                denom = math.sqrt(sum(x*x for x in first) * sum(x*x for x in other))
                if denom > 0:
                    angles.append(math.degrees(math.acos(max(-1, min(1, sum(x*y for x, y in zip(first, other))/denom)))))
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
        if not item.get('success') or not isinstance(point, list) or len(point) != 3:
            continue
        distances = sorted((sum((a-b)**2 for a, b in zip(point, anchor)), target)
                           for target, anchor in anchors.items() if isinstance(anchor, list) and len(anchor) == 3)
        if distances and distances[0][0] <= .06**2 and (
                len(distances) == 1 or distances[1][0] > .06**2):
            counts[distances[0][1]] += 1
    return counts

def run(args):
    if not (0 <= args.domain <= 232 and args.rounds >= 0 and args.models >= 0 and args.soak_seconds >= 0):
        raise SystemExit('domain must be 0..232; counts/duration must be nonnegative')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cfg_dir = config_directory()
    profile = yaml.safe_load((cfg_dir / 'live_camera_mock.yaml').read_text())
    env = owned_runtime_environment(os.environ, output, args.domain)
    report = {'hardware_mode': 'mock', 'profile': profile, 'rounds': [], 'models': [],
              'model_attempts': [], 'resources': [], 'acks': [], 'errors': [], 'rviz_requested': args.rviz}
    report['owned_process_groups'] = []
    report['runtime_environment'] = {key: env[key] for key in (
        'ROS_DOMAIN_ID', 'ROS_HOME', 'OMP_NUM_THREADS',
        'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}
    def audit_started(pgid):
        report['owned_process_groups'].append(pgid)
        save(output / 'report.json', report)
    owner = OwnedProcesses(env, output / 'owned_processes.json', on_start=audit_started)
    driver = None
    try:
        stale = domain_processes(args.domain)
        if stale:
            report['preexisting_processes'] = stale
            raise RuntimeError('域内已有采摘进程；拒绝启动，不清理其他进程')
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1', 8090)) == 0:
                raise RuntimeError('8090 已占用；拒绝启动')
        gui = gui_environment(env) if args.rviz else None
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        overlay = output / 'brain_overlay.yaml'
        overlay.write_text(yaml.safe_dump({'/peach_supervisor': {'ros__parameters': {
            'execute_pregrasp_only': False,
            'reconstruct_in_trajectory': profile['reconstruct_in_trajectory']}}}))
        command = ['ros2', 'launch', 'peach_bringup', 'harvest_system.launch.py',
                   'hardware_mode:=mock', 'camera_enabled:=true', 'camera_frontend:=stereo',
                   'extrinsics_enabled:=false', 'imu_enabled:=false', 'autostart:=false',
                   'use_sim_time:=false', 'rviz:=false', 'tool_profile:=bite_shear_v1',
                   'mock_initial_positions:=' + ','.join(map(str, profile['mock_initial_positions'])),
                   'brain_params_overlay:=' + str(overlay)]
        owner.start(command, output / 'stack.log')
        tf = profile['camera_calibration']
        xyz, quat = tf['xyz_m'], tf['quaternion_xyzw']
        tf_args = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher']
        for flag, value in zip(('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'), xyz + quat):
            tf_args += [flag, str(value)]
        tf_args += ['--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        owner.start(tf_args, output / 'camera_tf.log')
        driver = RosDriver()
        state = wait_state(driver, lambda s: startup_ready(s, args.domain))
        save(output / 'startup_state.json', state)
        if args.rviz:
            params = output / 'rviz_model_params.yaml'
            driver.rviz_params(params)
            rviz = owner.start(['rviz2', '-d', str(cfg_dir / 'live_camera_mock.rviz'),
                                '--ros-args', '--params-file', str(params)], output / 'rviz.log', gui)
            driver.spin_for(5)
            if rviz.poll() is not None:
                raise RuntimeError('RViz exited during startup; see rviz.log')
            report['rviz_started'] = True
        else:
            report['rviz_started'] = False
            report['rviz_note'] = '用户显式 --no-rviz；本轮未验证 RViz'
        from peach_interfaces.action import BuildTargetModel, RunHarvest
        from peach_interfaces.srv import ControlTask
        def rviz_tick():
            if args.rviz and rviz.poll() is not None:
                raise RuntimeError('RViz exited before test completion')

        def round_tick(request, target):
            rviz_tick()
            ack_tick(driver, request, target, report['acks'])

        driver.enables(False, False)
        seen = set()
        sample_request = 'sample_scene_' + uuid.uuid4().hex[:12]
        sample_epoch = None
        if args.models:
            sample_epoch, _ = new_scene(driver, sample_request, sample_request, 0)
        # Count completed model identities, never diagnostic heartbeat publications.
        for attempt in range(args.models * 3):
            if len(report['models']) >= args.models:
                break
            request = 'model_' + uuid.uuid4().hex[:12]
            started = time.monotonic()
            item = {'request_id': request}
            try:
                epoch = sample_epoch
                current = snapshot()
                target = choose_target(section(current, 'perception', 'targets'), len(report['models']))
                if not target:
                    raise RuntimeError('Fixed sampling scene has no current valid target')
                result = driver.action(BuildTargetModel,
                    '/peach_target_reconstruction_node/build_target_model',
                    BuildTargetModel.Goal(request_id=request, target_id=target, scene_epoch=epoch), 180, on_tick=rviz_tick)
                item.update(result)
                model = section(result, 'result', 'model')
                sample_key = build_generation(model)
                key = model_key(model)
                if result.get('status') != 4 or section(result, 'result').get('success') is not True:
                    raise RuntimeError('BuildTargetModel failed: ' + str(section(result, 'result').get('message')))
                if not sample_key or sample_key in seen or key[:2] != (target, epoch):
                    raise RuntimeError('BuildResult lacks new generated_at/valid_until/model identity proof')
                state = wait_state(driver, lambda s: web_model_matches(s, model), 10)
                seen.add(sample_key)
                evidence = model_geometry(model, section(state, 'reconstruction', 'grasp_decision'))
                report['models'].append({'identity': key, 'sample_identity': sample_key, 'model': model, 'geometry': evidence, 'elapsed_s': time.monotonic() - started})
                save(output / f'model_{attempt + 1}_state.json', state)
            except Exception as exc:
                item['error'] = str(exc)
                item['traceback'] = traceback.format_exc()
                traceback.print_exc()
            item['elapsed_s'] = time.monotonic() - started
            report['model_attempts'].append(item)
            save(output / 'report.json', report)
        driver.enables(True, True)
        for index in range(args.rounds):
            request = 'round_' + uuid.uuid4().hex[:12]
            before = {key: value.total for key, value in driver.stamps.items()}
            started = time.monotonic()
            record = {'request_id': request, 'success': False}
            try:
                epoch, target = new_scene(driver, request, request, index)
                record.update(target_id=target, scene_epoch=epoch)
                observations = section(snapshot(), 'perception', 'targets').get('observations', [])
                selected = next((o for o in observations if o.get('target_id') == target), {})
                record['entry_position'] = section(selected, 'candidate').get('entry_position')
                result = driver.action(RunHarvest, '/peach_supervisor/run_harvest',
                    RunHarvest.Goal(request_id=request, scene_key=request, intent=1,
                                    selection_mode=1, target_ids=[target], per_target_timeout_s=180., sector_timeout_s=240.), 300,
                    on_tick=lambda: round_tick(request, target))
                record['action'] = result
                state = wait_state(driver, lambda s: section(s, 'ledger', 'live').get('request_id') == request and bool(section(s, 'ledger', 'live').get('rows')), 10)
                ledger = section(state, 'ledger', 'live')
                record['ledger'] = ledger
                record['success'] = result.get('status') == 4 and section(result, 'result').get('success') is True and full_cycle(ledger, request)
                save(output / f'round_{index + 1}_state.json', state)
            except Exception as exc:
                record['error'] = str(exc)
                record['traceback'] = traceback.format_exc()
                traceback.print_exc()
            record['elapsed_s'] = time.monotonic() - started
            record['streams'] = frame_evidence(driver, before)
            if not all(value['fresh_stamp'] for value in record['streams'].values()):
                record['success'] = False
            report['rounds'].append(record)
            save(output / 'report.json', report)
        driver.enables(False, False)
        soak_started = time.monotonic()
        report['soak_started_monotonic'] = soak_started
        until = soak_started + args.soak_seconds
        while time.monotonic() < until:
            soak_before = {key: value.total for key, value in driver.stamps.items()}
            driver.spin_for(min(5, max(0, until - time.monotonic())))
            state = snapshot()
            report['resources'].append({'monotonic': time.monotonic(), 'metrics': state.get('metrics'),
                                        'topic_age_s': section(state, 'system').get('topic_age_s'),
                                        'streams_fresh': all(v['fresh_stamp'] for v in frame_evidence(driver, soak_before).values()),
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
            report['domain_processes_after'] = domain_processes(args.domain)
            report['model_count'] = len(report['models'])
            report['full_level6_rounds'] = sum(r['success'] for r in report['rounds'])
            report['model_elapsed_s'] = statistics([m['elapsed_s'] for m in report['models']])
            report['geometry_statistics'] = geometry_statistics(report['models'])
            groups = {}
            for model in report['models']:
                target = model['identity'][0]
                groups[target] = groups.get(target, 0) + 1
            report['model_samples_per_target'] = groups
            if args.models >= 3 and any(count < 3 for count in groups.values()):
                report['errors'].append('Insufficient repeat samples: each observed target requires at least 3 models')
            counts = {}
            for round_record in report['rounds']:
                for row in section(round_record, 'ledger').get('rows', []):
                    target_counts = counts.setdefault(row.get('target_id', 'unknown'), {})
                    outcome = row.get('outcome_name', 'UNKNOWN')
                    target_counts[outcome] = target_counts.get(outcome, 0) + 1
            report['per_target_outcomes'] = counts
            report['physical_bag_level6_coverage'] = physical_coverage(report['models'], report['rounds'])
            if args.rounds >= 6 and len(groups) == 2 and any(
                    count < 3 for count in report['physical_bag_level6_coverage'].values()):
                report['errors'].append('Both modeled bags require at least 3 level6 rounds each')
            report['soak_passed'] = soak_passed(args.soak_seconds, report.get('soak_elapsed_s', 0), report['resources'])
            if not report['soak_passed']:
                report['errors'].append('Soak incomplete or camera/joint/RViz liveness failed')
            report['passed'] = report.get('rviz_lifetime_ok', False) and not report['errors'] and not report['residual_pids'] and report['model_count'] == args.models and report['full_level6_rounds'] == args.rounds
            checkpoint(output, report, 'finished')
        print(json.dumps({'output': str(output), 'passed': report['passed'], 'errors': report['errors']}, ensure_ascii=False))
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
