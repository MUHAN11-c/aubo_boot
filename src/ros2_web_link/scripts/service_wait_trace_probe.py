#!/usr/bin/env python3
# Copyright 2026 ros2_web_link contributors
# SPDX-License-Identifier: Apache-2.0
"""Explicit native delayed-service check for opt-in wait diagnostics."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import psutil


def server():
    """Use a real Jazzy service and executor, without robot interfaces."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from service_trace import install_service_trace
    endpoint = os.environ.get('WEB_LINK_TRACE_SERVER_SERVICE', '')
    if endpoint:
        install_service_trace(endpoint, trace_clients=False)
    rclpy.init()
    node = Node('wait_fixture', namespace='/web_link_demo')

    def delayed(request, response):
        time.sleep(.4)
        response.success = True
        response.message = 'native delayed fixture'
        return response

    node.create_service(Trigger, '/web_link_demo/wait_probe', delayed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def check_native_fixture(root, late_reader=False):
    """Verify exact success CID/sequence and a separate intentional timeout."""
    from service_native_trace import read_native_trace
    stats = read_native_trace(root, '/web_link_demo/wait_probe')
    events = json.loads((root / stats['file']).read_text())['events']
    sends = [e for e in events if e['phase'] == 'sendReturn']
    assert {e['cid'] for e in sends} == {'wait-probe-success', 'wait-probe-timeout'}
    good = next(e for e in sends if e['cid'] == 'wait-probe-success')
    takes = [e for e in events if e['phase'] == 'takeReturn' and e['headerPresent'] and
             e['clientKey'] == good['clientKey'] and e['sequence'] == good['sequence']
             and e['monotonic'] >= good['monotonic']]
    assert len(takes) == 1
    # This fixture runs on the recorded local FastDDS build. Other RMWs may
    # report unsupported/zero times; the general diagnostic retains them as-is.
    assert int(takes[0]['sourceTimestampNs']) > 0
    assert int(takes[0]['receivedTimestampNs']) >= int(takes[0]['sourceTimestampNs'])
    if late_reader:
        from service_late_client import read_late_client_trace
        late = read_late_client_trace(root)
        assert len(late['records']) == 1 and late['records'][0]['cid'] == 'wait-probe-timeout'
        bad = next(e for e in sends if e['cid'] == 'wait-probe-timeout')
        delayed = [e for e in events if e['phase'] == 'takeReturn' and e['headerPresent'] and
                   e['clientKey'] == bad['clientKey'] and e['sequence'] == bad['sequence']
                   and e['monotonic'] >= bad['monotonic']]
        assert len(delayed) == 1, 'Held reader did not expose the delayed native response'
        failed_wait = next(json.loads(s)['trigger'] for path in root.glob('service-waits-*.jsonl')
                           for s in path.read_text().splitlines()
                           if json.loads(s)['trigger']['cid'] == 'wait-probe-timeout')
        assert delayed[0]['monotonic'] > failed_wait['waitReturnedAt']
        assert late['records'][0]['cleanupSubmittedAt'] >= (
            late['records'][0]['destroyRequestedAt']+1.9)


def main():
    """Bound time/RSS and always stop only the groups created here."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', action='store_true')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--domain', type=int, default=147)
    parser.add_argument('--port', type=int, default=20290)
    parser.add_argument('--server-metadata', action='store_true',
                        help='Also validate passive standard server introspection')
    parser.add_argument('--client-metadata', action='store_true',
                        help='With server-metadata, validate all four standard metadata phases')
    parser.add_argument('--native-boundary', action='store_true',
                        help='Also validate reviewed native send/take metadata at normal exit')
    parser.add_argument('--late-reader', action='store_true',
                        help='Keep a timed-out reader 2s without changing the timeout result')
    parser.add_argument('--cache-size', type=int,
                        help='Service cache slots; default 64, or 0 with --late-reader')
    args = parser.parse_args()
    if args.server:
        return server()
    if args.cache_size is None:
        args.cache_size = 0 if args.late_reader else 64
    if not 0 <= args.cache_size <= 1024 or (args.late_reader and args.cache_size):
        parser.error('cache-size must be 0..1024; late-reader requires 0')
    if args.late_reader and not args.native_boundary:
        parser.error('late-reader requires native-boundary')
    if args.client_metadata and not args.server_metadata:
        parser.error('client-metadata requires server-metadata')
    if args.out is None or not 0 <= args.domain <= 232 or not 1 <= args.port <= 65535:
        parser.error('out, domain 0..232 and port 1..65535 are required')
    root = args.out.resolve()
    root.mkdir(parents=True, exist_ok=False)
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', args.port)) == 0:
            raise RuntimeError('Refusing to use an occupied port')
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               ROS_LOG_DIR=str(root / 'ros-logs'), WEB_LINK_EVIDENCE=str(root),
               WEB_LINK_WAIT_SERVICE='/web_link_demo/wait_probe')
    env.pop('WEB_LINK_TRACE_SERVICE', None)
    env.pop('WEB_LINK_TRACE_SERVER_SERVICE', None)
    env.pop('WEB_LINK_NATIVE_SERVICE', None)
    env.pop('WEB_LINK_LATE_CLIENT_SERVICE', None)
    if args.client_metadata:
        env['WEB_LINK_TRACE_SERVICE'] = '/web_link_demo/wait_probe'
    if args.native_boundary:
        env['WEB_LINK_NATIVE_SERVICE'] = '/web_link_demo/wait_probe'
    if args.late_reader:
        env['WEB_LINK_LATE_CLIENT_SERVICE'] = '/web_link_demo/wait_probe'
    if args.server_metadata:
        env['WEB_LINK_TRACE_SERVER_SERVICE'] = '/web_link_demo/wait_probe'
    processes, files, samples = [], [], []
    failure = None
    diagnostic_failure = None
    try:
        commands = [
            ('server', ['python3', str(Path(__file__).resolve()), '--server']),
            ('bridge', ['ros2', 'launch', 'ros2_web_link', 'web.launch.py',
                        'web:=false', f'ws_port:={args.port}',
                        f'service_client_cache_size:={args.cache_size}']),
        ]
        if args.server_metadata:
            commands.append(('metadata', [
                'python3', str(Path(__file__).with_name('service_trace_collector.py')),
                '--out', str(root), '--seconds', '30', '--passive',
                '--service', '/web_link_demo/wait_probe',
                '--service-type', 'std_srvs/srv/Trigger']))
        for name, command in commands:
            log = (root / f'{name}.log').open('w')
            files.append(log)
            child_env = dict(env)
            if args.client_metadata:
                # Full client metadata belongs to the bridge; the independent
                # fixture server uses its server-only mode. Never combine both
                # mutually exclusive modes inside a single ROS process.
                if name == 'bridge':
                    child_env.pop('WEB_LINK_TRACE_SERVER_SERVICE', None)
                else:
                    child_env.pop('WEB_LINK_TRACE_SERVICE', None)
            processes.append(subprocess.Popen(command, env=child_env, stdout=log,
                                              stderr=subprocess.STDOUT, start_new_session=True))
        start = time.monotonic()
        wire = None
        while time.monotonic() - start < 40:
            if any(p.poll() is not None for p in processes[:2]):
                raise RuntimeError('Native fixture exited early')
            if args.server_metadata and processes[2].poll() is not None:
                raise RuntimeError('Passive metadata observer exited early')
            if wire is None and (not args.server_metadata or (root / 'trace.ready').exists()):
                with socket.socket() as sock:
                    if sock.connect_ex(('127.0.0.1', args.port)) == 0:
                        log = (root / 'wire.log').open('w')
                        files.append(log)
                        wire = subprocess.Popen([
                            'node', '--max-old-space-size=128',
                            str(Path(__file__).with_name('service_wait_wire_probe.mjs')),
                            f'ws://127.0.0.1:{args.port}', str(root / 'wire-result.json')],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                        processes.append(wire)
            owned = {}
            for process in processes:
                try:
                    if process.poll() is None:
                        parent = psutil.Process(process.pid)
                        for child in [parent] + parent.children(recursive=True):
                            owned[child.pid] = child
                except psutil.Error:
                    pass
            rss = 0
            for process in owned.values():
                try:
                    rss += process.memory_info().rss / 1048576
                except psutil.Error:
                    pass
            samples.append(rss)
            if rss > 1024 or psutil.virtual_memory().available < 4 * 1024**3:
                raise RuntimeError('Native diagnostic RSS/available-memory guard')
            if wire is not None and wire.poll() is not None:
                assert wire.returncode == 0, (root / 'wire.log').read_text()[-3000:]
                time.sleep(2.3 if args.late_reader else (1 if args.server_metadata else .2))
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Native diagnostic exceeded 40 seconds')
        traces = [json.loads(line) for path in root.glob('service-waits-*.jsonl')
                  for line in path.read_text().splitlines()]
        assert traces, 'Missing pre-cancellation wait timeout snapshot'
        record = traces[-1]
        assert record['trigger']['cid'] == 'wait-probe-timeout'
        state = record['trigger']['waitState']
        assert state['futureDone'] is False and state['futureCancelled'] is False
        assert state['eventSet'] is False
        assert len(record['recent']) <= 256
        assert any(c['cid'] == 'wait-probe-success' for c in record['recent'])
        assert all(c['service'] == '/web_link_demo/wait_probe' for c in record['recent'])
        if args.server_metadata:
            events = [json.loads(line) for line in (root / 'service-events.jsonl').read_text(
                ).splitlines()]
            phases = [e['phase'] for e in events if e['kind'] == 'service_event']
            assert phases.count(1) == 2 and phases.count(2) == 2, phases
            if args.client_metadata:
                assert phases.count(0) == 2 and phases.count(3) == 2, phases
            else:
                assert 0 not in phases and 3 not in phases, 'Client introspection was enabled'
            meta = json.loads((root / 'service-trace.json').read_text())
            assert meta['mode'] == 'passive-server' and meta['native']['calls'] == 0
    except BaseException as error:
        failure = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        groups = {p.pid for p in processes}
        for process in reversed(processes):
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    pass
        remaining = []
        for proc in psutil.process_iter(['pid', 'status']):
            try:
                if os.getpgid(proc.pid) in groups and proc.status() != psutil.STATUS_ZOMBIE:
                    remaining.append(proc.pid)
            except (psutil.Error, ProcessLookupError):
                pass
        if remaining:
            # A terminated leader can leave reparented children in its group.
            children = []
            for pid in remaining:
                try:
                    child = psutil.Process(pid)
                    child.terminate()
                    children.append(child)
                except psutil.Error:
                    pass
            _, alive = psutil.wait_procs(children, timeout=3)
            for child in alive:
                child.kill()
            psutil.wait_procs(alive, timeout=3)
            remaining = [p.pid for p in children
                         if p.is_running() and p.status() != psutil.STATUS_ZOMBIE]
        for log in files:
            log.close()
        if args.native_boundary:
            try:
                check_native_fixture(root, args.late_reader)
            except Exception as error:
                diagnostic_failure = f'{type(error).__name__}: {error}'
        (root / 'guard-result.json').write_text(json.dumps({
            'failure': failure or diagnostic_failure, 'diagnosticFailure': diagnostic_failure,
            'peakTreeRssMiB': max(samples, default=0),
            'remainingPids': remaining}, indent=2))
        if remaining:
            raise RuntimeError(f'Owned native diagnostic processes remain: {remaining}')
    if diagnostic_failure:
        raise RuntimeError(diagnostic_failure)
    if args.native_boundary:
        print('PASS native boundary: successful CID/sequence taken; '
              'deliberate timeout sent; no drops')
    print('PASS native success, timeout before cancellation, IDs and endpoint confinement')


if __name__ == '__main__':
    main()
