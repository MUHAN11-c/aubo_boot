"""
五点力接触段：只保留从离开空载到回到空载的那一段.

空载发布值是 0 N。起始门高于现场空载噪声，回到门略低并要求稳住，
避免边缘抖动把一次抓取切成多段。最长时长到了仍未归零也会闭合，避免一直写。

落盘跟采摘会话走：正在写的 ``runs/session_*/force/``，不另开目录。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

# 约 15 g 才算「出现变化」；约 8 g 以下视为回到空载。
# 空载噪声远低于 5 g（doc/force_20s_consistency.md）。
ONSET_N = 0.15
RELEASE_N = 0.08
RELEASE_HOLD_S = 0.35
MAX_EPISODE_S = 180.0


def peak_level(forces) -> float:
    """五路里最大的绝对力；非有限值不参与."""
    level = 0.0
    for value in forces or ():
        if value is None or not math.isfinite(value):
            continue
        level = max(level, abs(float(value)))
    return level


@dataclass
class ClosedEpisode:
    """一段已经闭合的接触."""

    reason: str
    t0: float
    t_end: float
    last_hot_t: float
    rows: list

    @property
    def duration_s(self) -> float:
        """从第一帧变化到归零稳住的文件时长."""
        return self.t_end - self.t0

    @property
    def contact_s(self) -> float:
        """从第一帧变化到最后一帧仍高于回到门的接触时长."""
        return self.last_hot_t - self.t0


@dataclass
class ForceEpisodeTracker:
    """按帧喂入力幅值，跨过起始门才开始记，归零稳住后交出这一段."""

    onset_n: float = ONSET_N
    release_n: float = RELEASE_N
    hold_s: float = RELEASE_HOLD_S
    max_s: float = MAX_EPISODE_S
    phase: str = 'idle'
    t0: float | None = None
    last_hot_t: float | None = None
    quiet_since: float | None = None
    rows: list = field(default_factory=list)

    def update(self, t: float, level: float, row: list) -> ClosedEpisode | None:
        """喂入一帧。归零稳住或到达最长时长时返回闭合段，否则 None."""
        if self.phase == 'idle':
            if level >= self.onset_n:
                self.phase = 'recording'
                self.t0 = t
                self.last_hot_t = t
                self.quiet_since = None
                self.rows = [row]
            return None
        self.rows.append(row)
        if level >= self.release_n:
            self.last_hot_t = t
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = t
        elapsed = t - self.t0
        held = (
            self.quiet_since is not None
            and (t - self.quiet_since) >= self.hold_s)
        if not held and elapsed < self.max_s:
            return None
        closed = ClosedEpisode(
            reason='returned' if held else 'max_duration',
            t0=self.t0,
            t_end=t,
            last_hot_t=self.last_hot_t if self.last_hot_t is not None else t,
            rows=self.rows)
        self.phase = 'idle'
        self.t0 = None
        self.last_hot_t = None
        self.quiet_since = None
        self.rows = []
        return closed


def live_peach_session(runs, now: float | None = None,
                       fresh_s: float = 5.0) -> Path | None:
    """
    正在录的采摘会话目录.

    ``session_*/bag`` 里还有新写入才算这一轮。已封口且袋文件不再变的旧会话、
    以及崩溃后留下的旧开口袋，都不接收新的力记录。
    """
    root = Path(runs)
    if not root.is_dir():
        return None
    clock = time.time() if now is None else float(now)
    best: Path | None = None
    best_mtime = -1.0
    for session in root.glob('session_*'):
        bag = session / 'bag'
        if not session.is_dir() or not bag.is_dir():
            continue
        newest = _newest_mtime(bag)
        if newest < 0.0 or (clock - newest) > fresh_s:
            continue
        if newest > best_mtime:
            best = session
            best_mtime = newest
    return best


def _newest_mtime(bag: Path) -> float:
    newest = -1.0
    try:
        newest = bag.stat().st_mtime
    except OSError:
        return -1.0
    try:
        children = list(bag.iterdir())
    except OSError:
        return newest
    for child in children:
        try:
            newest = max(newest, child.stat().st_mtime)
        except OSError:
            continue
    return newest


def next_episode_index(force_dir: Path) -> int:
    """该会话 force/ 里下一号，避免网页重启覆盖已有段."""
    highest = 0
    if force_dir.is_dir():
        for path in force_dir.glob('episode_*.csv'):
            stem = path.stem.replace('episode_', '')
            if stem.isdigit():
                highest = max(highest, int(stem))
    return highest + 1
