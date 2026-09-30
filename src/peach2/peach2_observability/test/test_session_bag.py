from pathlib import Path
import signal
import subprocess

from peach2_observability.session_bag import (
    BAG_RECORD_TOPICS,
    build_record_command,
    session_folder,
    SessionBagRecorder,
    stop_bag_process,
)


def test_build_record_command_includes_fixed_topics():
    cmd = build_record_command(Path('/tmp/bag_out'))
    assert cmd[:6] == ['ros2', 'bag', 'record', '-s', 'mcap', '-o']
    for topic in BAG_RECORD_TOPICS:
        assert topic in cmd


def test_session_folder_naming():
    path = session_folder(Path('/runs'), now=1_700_000_000.0)
    assert path.name.startswith('session_')


def test_stop_bag_process_sigint_then_kill():
    proc = subprocess.Popen(['sleep', '600'], start_new_session=True)
    try:
        code = stop_bag_process(proc, sigint_timeout_s=0.2, term_timeout_s=0.2)
        assert proc.poll() is not None
        assert code != 0 or proc.returncode is not None
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGKILL)


class _FakeBagProcess:
    def __init__(self):
        self.returncode = None
        self._signals = []

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self._signals.append(sig)
        if sig == signal.SIGKILL:
            self.returncode = -9
        else:
            self.returncode = 0

    def terminate(self):
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = 0
        return self.returncode


def test_session_bag_recorder_builds_command_without_starting_real_bag(tmp_path: Path):
    captured = {}

    def fake_popen(cmd, **kwargs):
        captured['cmd'] = cmd
        return _FakeBagProcess()

    rec = SessionBagRecorder(
        tmp_path,
        enabled=True,
        sigint_timeout_s=0.5,
        term_timeout_s=0.5,
        popen_factory=fake_popen,
    )
    rec.start(now=1_700_000_000.0)
    assert 'cmd' in captured
    assert captured['cmd'][0] == 'ros2'
    rec.stop()
