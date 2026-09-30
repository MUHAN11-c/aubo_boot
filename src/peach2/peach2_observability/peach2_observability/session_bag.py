"""Optional session bag via ``ros2 bag record`` subprocess (pure helpers + manager)."""
from __future__ import annotations

from pathlib import Path
import signal
import subprocess
import time


# Fixed v2 topic set for session replay (no wildcard discovery).
BAG_RECORD_TOPICS: tuple[str, ...] = (
    '/diagnostics',
    '/peach/task/state',
    '/peach/end_effector/tool_state',
    '/peach/manipulation/recovery_required',
    '/peach/target_model/models',
    '/peach/perception/observations',
    '/peach/enables',
    '/joint_states',
    '/tf',
    '/tf_static',
)


def session_folder(runs_root: Path, now: float | None = None) -> Path:
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now or time.time()))
    return Path(runs_root) / f'session_{stamp}'


def build_record_command(bag_dir: Path, topics: tuple[str, ...] | None = None) -> list[str]:
    names = list(topics or BAG_RECORD_TOPICS)
    return ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir), *names]


def stop_bag_process(
    proc: subprocess.Popen,
    *,
    sigint_timeout_s: float = 15.0,
    term_timeout_s: float = 5.0,
) -> int:
    """
    Graceful bag shutdown: SIGINT → wait → SIGTERM → wait → SIGKILL.

    Returns the process exit code (or -9 after kill).
    """
    if proc.poll() is not None:
        return int(proc.returncode or 0)
    try:
        proc.send_signal(signal.SIGINT)
    except ProcessLookupError:
        return int(proc.returncode or 0)
    try:
        proc.wait(timeout=max(0.1, float(sigint_timeout_s)))
        return int(proc.returncode or 0)
    except subprocess.TimeoutExpired:
        pass
    try:
        proc.terminate()
    except ProcessLookupError:
        return int(proc.returncode or 0)
    try:
        proc.wait(timeout=max(0.1, float(term_timeout_s)))
        return int(proc.returncode or 0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5.0)
        return int(proc.returncode if proc.returncode is not None else -9)


class SessionBagRecorder:
    """Start/stop ``ros2 bag record`` without ROS clients in this node."""

    def __init__(
        self,
        runs_root: Path,
        *,
        enabled: bool,
        sigint_timeout_s: float,
        term_timeout_s: float,
        popen_factory=subprocess.Popen,
        log_warning=lambda msg: None,
    ) -> None:
        self._runs_root = Path(runs_root)
        self._enabled = bool(enabled)
        self._sigint_timeout_s = float(sigint_timeout_s)
        self._term_timeout_s = float(term_timeout_s)
        self._popen_factory = popen_factory
        self._log_warning = log_warning
        self._proc: subprocess.Popen | None = None
        self._session_dir: Path | None = None
        self._bag_dir: Path | None = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    def info(self) -> dict:
        running = self._proc is not None and self._proc.poll() is None
        return {
            'enabled': self._enabled,
            'running': running,
            'session': str(self._session_dir) if self._session_dir else None,
            'directory': str(self._bag_dir) if self._bag_dir else None,
        }

    def start(self, now: float | None = None) -> None:
        if not self._enabled or self._proc is not None:
            return
        self._session_dir = session_folder(self._runs_root, now=now)
        self._bag_dir = self._session_dir / 'bag'
        self._bag_dir.mkdir(parents=True, exist_ok=True)
        cmd = build_record_command(self._bag_dir)
        try:
            self._proc = self._popen_factory(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
        except OSError as error:
            self._log_warning(f'session bag failed to start: {error}')
            self._proc = None
            self._bag_dir = None
            self._session_dir = None

    def stop(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        code = stop_bag_process(
            proc,
            sigint_timeout_s=self._sigint_timeout_s,
            term_timeout_s=self._term_timeout_s,
        )
        if code not in (0, -2, 130):
            self._log_warning(f'session bag exited with code {code}')
