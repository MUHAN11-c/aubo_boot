#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Run an isolated, memory-bounded real-project benchmark and clean every child."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import psutil
import yaml


def stop(process, *, launch_root=False):
    """Let an owned launch forward SIGINT once; bound every owned group."""
    if process.poll() is not None:
        return

    def send_owned(signum):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            # poll -> signal is racy; a group already gone is already stopped.
            pass

    if launch_root:
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    else:
        send_owned(signal.SIGINT)
    try:
        # web.launch allows 15s for upstream's default10s publisher timers.
        # A launch forwards SIGINT to children; group INT would deliver twice.
        process.wait(timeout=18 if launch_root else 8)
    except subprocess.TimeoutExpired:
        send_owned(signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            send_owned(signal.SIGKILL)
            process.wait(timeout=3)


def main():
    """Use existing launch, project contract and SDK; do not launch robot nodes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--project', default='peach_replay')
    parser.add_argument('--projects-config', type=Path)
    parser.add_argument('--view-config', type=Path)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--seconds', type=int, default=8)
    parser.add_argument('--web-port', type=int, default=18288)
    parser.add_argument('--ws-port', type=int, default=19290)
    parser.add_argument('--max-rss-mib', type=int, default=1024)
    parser.add_argument('--write-queue-size', type=int,
                        help='Override upstream outgoing queue in messages; default uses launch')
    parser.add_argument('--service-client-cache-size', type=int,
                        help='Cache slots 0..1024; 0 uses upstream per-call clients')
    parser.add_argument('--ui-seconds', type=int, default=0,
                        help='Guard a manual/CUA UI session instead of running the Node probe')
    parser.add_argument('--multi-client-seconds', type=int, default=0)
    parser.add_argument('--separate-control', action='store_true',
                        help='In multi-client mode, use a second RosClient for read-only RPC')
    parser.add_argument('--native-churn-seconds', type=int, default=0,
                        help='Compare native client lifetimes instead of WebSocket clients')
    parser.add_argument('--client-count', type=int, default=3)
    parser.add_argument('--rpc-period-ms', type=int, default=2000,
                        help='Web query or native comparison-cycle interval in milliseconds')
    parser.add_argument('--trace-services', action='store_true',
                        help='Diagnose /rosapi/nodes with metadata and a native DDS control')
    parser.add_argument('--trace-waits', action='store_true',
                        help='Observe /rosapi/nodes wait timeouts without extra ROS entities')
    parser.add_argument('--trace-server', action='store_true',
                        help='With trace-waits, observe server metadata without control requests')
    parser.add_argument('--trace-native', action='store_true',
                        help='With trace-waits, record reviewed native send/take boundaries')
    parser.add_argument('--trace-late-client', action='store_true',
                        help='With trace-native, keep only timed-out readers 2s for diagnosis')
    parser.add_argument('--asset-packages', default='ros2_web_link')
    args = parser.parse_args()
    if args.write_queue_size is not None and not 1 <= args.write_queue_size <= 4096:
        parser.error('write-queue-size must be 1..4096')
    if (args.service_client_cache_size is not None and
            not 0 <= args.service_client_cache_size <= 1024):
        parser.error('service-client-cache-size must be 0..1024')
    if args.trace_late_client:
        if args.service_client_cache_size is None:
            args.service_client_cache_size = 0
        elif args.service_client_cache_size != 0:
            parser.error('trace-late-client requires service-client-cache-size=0')
    if not 0 <= args.domain <= 232 or not 2 <= args.seconds <= 60:
        parser.error('domain must be 0..232 and seconds 2..60')
    if args.ui_seconds and not 2 <= args.ui_seconds <= 600:
        parser.error('ui-seconds must be 0 or 2..600')
    if args.multi_client_seconds and not 30 <= args.multi_client_seconds <= 3600:
        parser.error('multi-client-seconds must be 0 or 30..3600')
    if args.separate_control and not args.multi_client_seconds:
        parser.error('separate-control requires multi-client mode')
    if args.native_churn_seconds and not 2 <= args.native_churn_seconds <= 3600:
        parser.error('native-churn-seconds must be 0 or 2..3600')
    if not 2 <= args.client_count <= 6 or sum(bool(value) for value in (
            args.ui_seconds, args.multi_client_seconds, args.native_churn_seconds)) > 1:
        parser.error('client-count must be 2..6; choose UI, multi-client or native-churn mode')
    if args.trace_services and not args.multi_client_seconds:
        parser.error('trace-services requires multi-client mode')
    if args.trace_waits and (not args.multi_client_seconds or args.trace_services):
        parser.error('trace-waits requires multi-client mode without trace-services')
    if args.trace_server and not args.trace_waits:
        parser.error('trace-server requires trace-waits')
    if args.trace_native and not args.trace_waits:
        parser.error('trace-native requires trace-waits')
    if args.trace_late_client and not args.trace_native:
        parser.error('trace-late-client requires trace-native')
    if not 100 <= args.rpc_period_ms <= 10000:
        parser.error('rpc-period-ms must be 100..10000')
    if args.max_rss_mib < 128 or any(not 1 <= p <= 65535 for p in (
            args.web_port, args.ws_port)):
        parser.error('RSS limit must be >=128MiB and ports must be 1..65535')
    for port in (args.web_port, args.ws_port):
        with socket.socket() as test:
            if test.connect_ex(('127.0.0.1', port)) == 0:
                parser.error(f'port {port} is in use; refusing to stop another process')
    root = args.evidence.resolve()
    # Claim a fresh run atomically. Reusing a previous native snapshot after a
    # forced shutdown could otherwise pass the evidence check for a new run.
    root.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('ros2_web_link'))
    projects = yaml.safe_load((share / 'config/projects.yaml').read_text())['projects']
    if args.projects_config:
        projects.update(yaml.safe_load(args.projects_config.read_text())['projects'])
    project = projects[args.project]
    (root / 'project.json').write_text(json.dumps(project, ensure_ascii=False, indent=2))
    (root / 'run-config.json').write_text(json.dumps(
        {**vars(args), 'rmwRequested': os.environ.get('RMW_IMPLEMENTATION', 'default')},
        default=str, indent=2))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(root / 'ros-logs'),
               WEB_LINK_EVIDENCE=str(root), WEB_LINK_BENCH_SECONDS=str(args.seconds),
               WEB_LINK_PROJECT_CONFIG=str(root / 'project.json'))
    bag_info = yaml.safe_load((args.bag / 'metadata.yaml').read_text())
    env.update(WEB_LINK_MULTI_SECONDS=str(args.multi_client_seconds),
               WEB_LINK_MULTI_CLIENTS=str(args.client_count),
               WEB_LINK_MULTI_QUERY_MS=str(args.rpc_period_ms),
               WEB_LINK_MULTI_SEPARATE_CONTROL=str(int(args.separate_control)),
               WEB_LINK_REPLAY_SECONDS=str(
                   bag_info['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9))
    if args.trace_services:
        env['WEB_LINK_TRACE_SERVICE'] = '/rosapi/nodes'
    else:
        env.pop('WEB_LINK_TRACE_SERVICE', None)
    if args.trace_waits:
        env['WEB_LINK_WAIT_SERVICE'] = '/rosapi/nodes'
    else:
        env.pop('WEB_LINK_WAIT_SERVICE', None)
    if args.trace_server:
        env['WEB_LINK_TRACE_SERVER_SERVICE'] = '/rosapi/nodes'
    else:
        env.pop('WEB_LINK_TRACE_SERVER_SERVICE', None)
    if args.trace_native:
        env['WEB_LINK_NATIVE_SERVICE'] = '/rosapi/nodes'
    else:
        env.pop('WEB_LINK_NATIVE_SERVICE', None)
    if args.trace_late_client:
        env['WEB_LINK_LATE_CLIENT_SERVICE'] = '/rosapi/nodes'
    else:
        env.pop('WEB_LINK_LATE_CLIENT_SERVICE', None)
    argv = ['ros2', 'launch', 'ros2_web_link', 'replay.launch.py',
            f'bag:={args.bag.resolve()}', f'project:={args.project}',
            f'asset_packages:={args.asset_packages}',
            f'web_port:={args.web_port}', f'ws_port:={args.ws_port}']
    if args.projects_config:
        argv.append(f'projects_config:={args.projects_config.resolve()}')
    if args.view_config:
        argv.append(f'config:={args.view_config.resolve()}')
    if args.write_queue_size is not None:
        # Standard rosbridge launch configuration, not a new buffering layer.
        argv.append(f'bridge_write_queue_size:={args.write_queue_size}')
    if args.service_client_cache_size is not None:
        argv.append(f'bridge_service_client_cache_size:={args.service_client_cache_size}')
    processes = []
    samples = []
    inventory = {}
    failure = None
    diagnostic_failure = None
    native_diagnostic = None
    late_diagnostic = None
    server = None
    trace = None
    dumps = 0
    termination_signal = None

    def request_stop(signum, _frame):
        # Keep Popen -> registration and cleanup atomic with respect to signals.
        # Python handlers only record intent; the main path raises at checkpoints.
        nonlocal termination_signal
        termination_signal = termination_signal or signal.Signals(signum).name

    def check_termination():
        if termination_signal is not None:
            raise InterruptedError(f'Interrupted by {termination_signal}')

    previous_handlers = {number: signal.signal(number, request_stop)
                         for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        check_termination()
        # subprocess.run owns/reaps its preflight child, including its 10s timeout.
        # A termination request is observed before any application launch follows.
        graph = subprocess.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
                               env=env, capture_output=True,
                               text=True, timeout=10, check=True)
        check_termination()
        if graph.stdout.strip():
            raise RuntimeError('DDS domain already contains nodes; choose a clean --domain')
        with (root / 'launch.log').open('w') as log:
            server = subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT,
                                      stdin=subprocess.DEVNULL,
                                      start_new_session=True)
        processes.append(server)
        for _ in range(50):
            check_termination()
            with socket.socket() as test:
                if test.connect_ex(('127.0.0.1', args.ws_port)) == 0:
                    break
            if server.poll() is not None:
                raise RuntimeError('launch exited before bridge became ready')
            time.sleep(.2)
        client = None
        check_termination()
        if args.trace_services or args.trace_server:
            collector = Path(__file__).resolve().with_name('service_trace_collector.py')
            collector_args = ['python3', str(collector), '--out', str(root),
                              '--seconds', str(args.multi_client_seconds + 60)]
            if args.trace_server:
                collector_args.append('--passive')
            with (root / 'service-trace.log').open('w') as log:
                trace = subprocess.Popen(collector_args, env=env, stdout=log,
                                         stderr=subprocess.STDOUT,
                                         start_new_session=True)
            processes.append(trace)
            for _ in range(60):
                check_termination()
                if (root / 'trace.ready').exists():
                    break
                if trace.poll() is not None:
                    raise RuntimeError((root / 'service-trace.log').read_text()[-3000:])
                time.sleep(.1)
            else:
                raise RuntimeError('Service trace did not become ready')
        check_termination()
        if args.native_churn_seconds:
            probe = Path(__file__).resolve().with_name('service_churn_probe.py')
            with (root / 'probe.log').open('w') as log:
                client = subprocess.Popen(['python3', str(probe), '--out', str(root),
                                           '--seconds', str(args.native_churn_seconds),
                                           '--period-ms', str(args.rpc_period_ms)],
                                          env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            processes.append(client)
        elif not args.ui_seconds:
            probe = Path(__file__).resolve().with_name(
                'multi_client_probe.mjs' if args.multi_client_seconds else 'transport_probe.mjs')
            with (root / 'probe.log').open('w') as log:
                client = subprocess.Popen(['node', '--max-old-space-size=512', str(probe),
                                           f'ws://127.0.0.1:{args.ws_port}'], env=env,
                                          stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            processes.append(client)
        (root / 'owned.json').write_text(json.dumps(
            {'argv': argv, 'pids': [p.pid for p in processes], 'domain': args.domain}, indent=2))
        start = time.monotonic()
        print(f'Test server ready: http://127.0.0.1:{args.web_port}', flush=True)
        deadline = args.ui_seconds or (
            (args.multi_client_seconds or args.native_churn_seconds) + 60
            if args.multi_client_seconds or args.native_churn_seconds else 2 * args.seconds + 30)
        while client is None or client.poll() is None:
            check_termination()
            memory = []
            for parent in processes:
                try:
                    proc = psutil.Process(parent.pid)
                    for child in [proc, *proc.children(recursive=True)]:
                        try:
                            if child.pid not in inventory:
                                inventory[child.pid] = {
                                    'pid': child.pid, 'ppid': child.ppid(),
                                    'createdAt': child.create_time(), 'argv': child.cmdline()}
                            memory.append({'pid': child.pid, 'name': child.name(),
                                           'rssMiB': child.memory_info().rss / 1048576})
                        except psutil.Error:
                            pass
                except psutil.Error:
                    pass
            total = sum(p['rssMiB'] for p in memory)
            available = psutil.virtual_memory().available / 1048576
            elapsed = time.monotonic() - start
            samples.append({'seconds': round(elapsed, 2), 'rssMiB': total,
                            'availableMiB': available, 'processes': memory})
            request = root / 'stack.request'
            if args.trace_services and request.exists():
                request.unlink()
                if dumps < 10:
                    pids = []
                    for item in inventory.values():
                        if (len(item['argv']) < 2 or Path(item['argv'][1]).name
                                not in ('bridge_node.py', 'rosapi_node.py')):
                            continue
                        try:
                            child = psutil.Process(item['pid'])
                            if (child.create_time() == item['createdAt']
                                    and child.cmdline()[:2] == item['argv'][:2]):
                                child.send_signal(signal.SIGUSR1)
                                pids.append(child.pid)
                        except psutil.Error:
                            pass
                    with (root / 'stack-dumps.jsonl').open('a') as log:
                        log.write(json.dumps({'wall': time.time(), 'pids': pids}) + '\n')
                    dumps += 1
            if args.ui_seconds and ((root / 'stop.request').exists() or elapsed >= deadline):
                break
            if total > args.max_rss_mib or available < 4096 or elapsed > deadline:
                raise RuntimeError(f'bounded stop: RSS={total:.1f}MiB free={available:.1f}MiB')
            if server.poll() is not None:
                raise RuntimeError('server exited during benchmark')
            if trace is not None and trace.poll() is not None:
                raise RuntimeError('Service metadata/control collector exited early')
            time.sleep(.5)
        check_termination()
        if client is not None and client.returncode:
            raise RuntimeError((root / 'probe.log').read_text()[-3000:])
        if trace is not None:
            stop(trace)
            control = json.loads((root / 'service-trace.json').read_text())
            native_bad = not args.trace_server and (
                not control['native']['completed'] or control['native']['timeouts']
                or control['native']['errors'])
            if (native_bad or not control['phaseCounts'].get('1')
                    or not control['phaseCounts'].get('2')):
                raise RuntimeError(f'Native DDS control/introspection failed: {control}')
    except Exception as error:
        failure = str(error)
        raise
    finally:
        remaining = []
        cleanup_errors = []
        try:
            for process in reversed(processes):
                try:
                    stop(process, launch_root=process is server)
                except Exception as error:
                    cleanup_errors.append(f'PID {process.pid}: {type(error).__name__}: {error}')
            groups = {p.pid for p in processes}
            for proc in psutil.process_iter(['pid', 'status']):
                try:
                    if os.getpgid(proc.pid) in groups and proc.status() != psutil.STATUS_ZOMBIE:
                        remaining.append(proc.pid)
                except (psutil.Error, ProcessLookupError):
                    pass
            if remaining:
                # A dead launcher may leave a reparented child in its owned group.
                children = []
                for pid in remaining:
                    try:
                        child = psutil.Process(pid)
                        children.append(child)
                        child.terminate()
                    except psutil.NoSuchProcess:
                        pass
                    except psutil.Error as error:
                        cleanup_errors.append(f'PID {pid}: {type(error).__name__}: {error}')
                _, alive = psutil.wait_procs(children, timeout=5)
                for child in alive:
                    try:
                        child.kill()
                    except psutil.NoSuchProcess:
                        pass
                    except psutil.Error as error:
                        cleanup_errors.append(f'PID {child.pid}: {type(error).__name__}: {error}')
                psutil.wait_procs(alive, timeout=3)
                remaining = []
                for child in children:
                    try:
                        if child.is_running() and child.status() != psutil.STATUS_ZOMBIE:
                            remaining.append(child.pid)
                    except psutil.NoSuchProcess:
                        pass
            # Optional evidence reads follow mandatory group cleanup.
            if args.trace_native:
                try:
                    from service_native_trace import read_native_trace
                    native_diagnostic = read_native_trace(root, '/rosapi/nodes')
                except Exception as error:
                    diagnostic_failure = f'Native boundary evidence incomplete: {error}'
                    failure = failure or diagnostic_failure
            if args.trace_late_client:
                try:
                    from service_late_client import read_late_client_trace
                    late_diagnostic = read_late_client_trace(root)
                except Exception as error:
                    detail = f'Post-timeout reader evidence incomplete: {error}'
                    diagnostic_failure = '; '.join(filter(None, [diagnostic_failure, detail]))
                    failure = failure or diagnostic_failure
        except Exception as error:
            cleanup_errors.append(f'{type(error).__name__}: {error}')
        finally:
            # Completion boundary: children stopped; no new operation can follow.
            # Ignore late requests while committing the final snapshot, then restore
            # callers' handlers even when serialization/write/replace fails.
            for number in previous_handlers:
                signal.signal(number, signal.SIG_IGN)
            if termination_signal is not None:
                failure = failure or f'Interrupted by {termination_signal}'
            if cleanup_errors:
                failure = failure or 'Cleanup errors: ' + '; '.join(cleanup_errors)
            try:
                report = {'processInventory': list(inventory.values()), 'samples': samples,
                          'failure': failure, 'remainingPids': remaining,
                          'terminationSignal': termination_signal, 'cleanupErrors': cleanup_errors,
                          'completionBoundary': (
                              'Owned cleanup finished before atomic report commit'),
                          'nativeDiagnostic': native_diagnostic,
                          'lateClientDiagnostic': late_diagnostic,
                          'diagnosticFailure': diagnostic_failure}
                temporary = root / 'resources.json.tmp'
                temporary.write_text(json.dumps(report, indent=2))
                temporary.replace(root / 'resources.json')
            finally:
                for number, handler in previous_handlers.items():
                    signal.signal(number, handler)
        if remaining:
            raise RuntimeError(f'owned process groups not clean: {remaining}')
        if cleanup_errors:
            raise RuntimeError('Cleanup errors: ' + '; '.join(cleanup_errors))
    if termination_signal is not None:
        raise InterruptedError(f'Interrupted by {termination_signal}')
    if diagnostic_failure:
        raise RuntimeError(diagnostic_failure)
    label = ('UI fixture stopped' if args.ui_seconds else
             'PASS: native service churn benchmark' if args.native_churn_seconds else
             'PASS: multi-client benchmark' if args.multi_client_seconds else
             'PASS: transport benchmark')
    print(f'{label}; peak RSS {max(s["rssMiB"] for s in samples):.1f}MiB; '
          f'no child remains; evidence {root}')


if __name__ == '__main__':
    main()
