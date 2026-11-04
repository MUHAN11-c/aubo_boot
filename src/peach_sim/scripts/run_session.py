#!/usr/bin/env python3
"""Own a finite simulation session and clean exactly its process group on exit."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from peach_interfaces.action import RunHarvest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--mission', action='store_true')
    parser.add_argument('--duration', type=float, default=480)
    parser.add_argument('--cancel-after', type=float, default=0)
    parser.add_argument('--max-targets', type=int, default=1)
    parser.add_argument('--scene', choices=['data_orchard', 'demo', 'orchard', 'field'],
                        default='data_orchard')
    parser.add_argument('--perception', choices=['ground_truth', 'vision'], default='vision')
    parser.add_argument('--tool-profile', choices=['shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'],
                        default='adaptive_shear_v1')
    options = parser.parse_args()
    root = Path(os.environ['PEACH_SIM_PROJECT_ROOT'])
    run = root / '.runtime' / ('session_' + time.strftime('%Y%m%d_%H%M%S'))
    run.mkdir(parents=True)
    os.environ['PEACH_SIM_RUN_DIRECTORY'] = str(run)
    os.environ['AUBO_RUNS_DIR'] = str(run / 'vision')
    (run / 'deployment.json').write_text(json.dumps({
        'options': vars(options), 'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
        'gz_partition': os.environ['GZ_PARTITION']}))
    rclpy.init()
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    node = Node('sim_session_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    from sensor_evidence import SensorEvidence
    evidence = SensorEvidence(node, run)
    counts = {'clock': 0, 'joint_states': 0, 'rgb_frames': 0, 'odom': 0}
    odometry, events = [], []
    final_velocity = {}

    def count(name):
        return lambda _: counts.update({name: counts[name] + 1})

    def odom(msg):
        counts['odom'] += 1
        p = msg.pose.pose.position
        odometry.append([p.x, p.y, p.z])
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        final_velocity.update(linear_m_s=(v.x*v.x + v.y*v.y + v.z*v.z)**.5,
                              angular_rad_s=(w.x*w.x + w.y*w.y + w.z*w.z)**.5)

    def event(msg):
        value = json.loads(msg.data)
        events.append(value)
        evidence.stage = value['stage']

    node.create_subscription(Clock, '/clock', count('clock'), qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', count('joint_states'),
                             qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/color/image_raw', count('rgb_frames'),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(String, '/sim_harvest/events', event, 100)
    lifecycle = [node.create_client(GetState, '/' + name + '/get_state')
                 for name in ['amcl', 'bt_navigator']]
    client = ActionClient(node, RunHarvest, '/sim_harvest/run_harvest')
    command = ['ros2', 'launch', 'sim_bringup', 'harvest.launch.py',
               f'scene:={options.scene}', f'runtime_dir:={run}',
               f'gui:={str(options.gui).lower()}', f'rviz:={str(options.rviz).lower()}',
               f'perception:={options.perception}', f'max_targets:={options.max_targets}',
               f'tool_profile:={options.tool_profile}',
               'autostart:=false']
    output = (run / 'launch.log').open('w')
    environment = os.environ.copy()
    # cv_bridge loads pip OpenCV in this probe; its private Qt paths must not
    # leak into Gazebo/RViz, which use the system Qt libraries.
    for key in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        if '/cv2/' in environment.get(key, ''):
            environment.pop(key)
    process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                               start_new_session=True, env=environment)
    print('Session:', run, flush=True)
    start = time.monotonic()
    accepted, handle, result_future, result, cancel_future = None, None, None, None, None
    requested_at = 0
    last_progress = 0
    screenshot_done = False
    topology_saved = False
    ready_futures = []
    health_started = 0.
    rc = 2
    try:
        while time.monotonic() - start < options.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if stopping.is_set() or (run / 'STOP').exists():
                raise KeyboardInterrupt
            if time.monotonic() - last_progress > 5:
                last_progress = time.monotonic()
                progress = {'counts': counts, 'mission_server': client.server_is_ready(),
                            'lifecycle_ready': [c.service_is_ready() for c in lifecycle],
                            'future_done': [f.done() for f in ready_futures],
                            'accepted': bool(handle and handle.accepted)}
                (run / 'progress.json').write_text(json.dumps(progress))
            if not topology_saved and time.monotonic() - start > 15:
                topology_saved = True

                def topology():
                    try:
                        value = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                                               text=True, timeout=5)
                        (run / 'gz-topics.txt').write_text(value.stdout)
                    except Exception as exc:
                        (run / 'gz-topics-error.txt').write_text(str(exc))
                threading.Thread(target=topology, daemon=True).start()
            if (options.gui or options.rviz) and not screenshot_done and (
                    time.monotonic() - start > 25):
                screenshot_done = True

                def screenshots():
                    try:
                        from capture_sim import capture
                        capture(run)
                    except Exception as exc:
                        (run / 'capture-error.txt').write_text(str(exc))
                threading.Thread(target=screenshots, daemon=True).start()
            if process.poll() is not None:
                raise RuntimeError('launch exited:' + str(process.returncode))
            if options.mission and accepted is None and client.server_is_ready():
                if ready_futures and time.monotonic() - health_started > 3:
                    for dependency, future in zip(lifecycle, ready_futures):
                        dependency.remove_pending_request(future)
                        future.cancel()
                    ready_futures = []
                if not ready_futures and all(c.service_is_ready() for c in lifecycle):
                    health_started = time.monotonic()
                    ready_futures = [c.call_async(GetState.Request()) for c in lifecycle]
                if ready_futures and all(f.done() for f in ready_futures):
                    active = all(f.result() and f.result().current_state.id == 3
                                 for f in ready_futures)
                    ready_futures = []
                    if active and counts['joint_states'] > 5 and counts['clock'] > 5:
                        accepted = client.send_goal_async(RunHarvest.Goal(
                            request_id='session_' + str(time.time_ns())))
            if accepted is not None and accepted.done() and handle is None:
                handle = accepted.result()
                if not handle.accepted:
                    accepted, handle = None, None
                    time.sleep(.2)
                    continue
                requested_at = time.monotonic()
                result_future = handle.get_result_async()
                print('Mission accepted', flush=True)
            if handle and options.cancel_after and cancel_future is None:
                if time.monotonic() - requested_at >= options.cancel_after:
                    cancel_future = handle.cancel_goal_async()
            if result_future is not None and result_future.done():
                response = result_future.result()
                value = response.result
                result = {'status': response.status, 'success': value.success,
                          'succeeded': value.summary.succeeded,
                          'reason': value.termination_reason}
                rc = (0 if response.status == 5 else 1) if options.cancel_after else (
                    0 if value.success else 1)
                # Retain final sensor/odometry and GUI evidence after the terminal
                # action result, before dismantling the owned process group.
                until = time.monotonic() + 3
                while time.monotonic() < until and not stopping.is_set():
                    rclpy.spin_once(node, timeout_sec=.1)
                if options.gui or options.rviz:
                    final = run / 'final'
                    final.mkdir()
                    from capture_sim import capture
                    capture(final)
                break
            if not options.mission and time.monotonic() - start > options.duration - 1:
                rc = 0 if all(counts.values()) and evidence.counts['snapshots'] else 1
    except KeyboardInterrupt:
        rc = 130
    except Exception as exc:
        result = {'exception': str(exc)}
        rc = 1
    finally:
        if rclpy.ok() and handle and handle.accepted and handle.status not in (4, 5, 6):
            future = handle.cancel_goal_async()
            deadline = time.monotonic() + 8
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            # Give dependency cancellation and final result callbacks time to drain.
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)

        def group_alive():
            try:
                os.killpg(process.pid, 0)
                return True
            except ProcessLookupError:
                return False

        for sig, grace in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
            if not group_alive():
                break
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + grace
            while group_alive() and time.monotonic() < deadline:
                process.poll()
                time.sleep(.1)
        process.wait(timeout=5)
        output.close()
        if odometry:
            origin = odometry[0]
            displacement = max(sum((a-b)**2 for a, b in zip(p, origin)) ** .5
                               for p in odometry)
        else:
            displacement = 0
        summary = {'result': result, 'counts': counts, 'max_displacement_m': displacement,
                   'sensor_evidence': evidence.counts,
                   'final_velocity': final_velocity, 'process_group_clean': not group_alive(),
                   'event_stages': [e['stage'] for e in events], 'exit_code': rc,
                   'duration_s': time.monotonic() - start}
        (run / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
