"""
全流程阶段时序与批次账本直播（纯核，零 ROS 依赖）.

服务器侧权威时间线：HarvestState / 技能 ~/status 的每次键变化记一条
转移，段时长 = 到下一条转移的间隔（前端不再依赖页面存活期自行掐表，
刷新不丢历史）。批次账本 ``runs/<request_id>/ledger.json`` 按
mtime+size 增量重读，归一成 per-target 行（阶段名/时长 zip、outcome
命名、总计）。observability_node 只做接线；本模块可被无 ROS 上下文
的单测直接装载。
"""
from __future__ import annotations

import json
from pathlib import Path

# TargetOutcome.msg 常量（0..4）；与 IDL 语义一致，前端配色按 token 键控
OUTCOME_NAMES = {
    0: 'SUCCEEDED',
    1: 'SKIPPED_QUALITY',
    2: 'SKIPPED_UNREACHABLE',
    3: 'FAILED',
    4: 'CANCELED',
}

# ledger 行可选遥测键（supervisor outcome_to_dict 白名单 + FULL 附加）。
# W7：cut/retreat/harvest_confirmed 顶层镜像已删且白名单本就不收——不再透传。
_ROW_PASSTHROUGH = (
    'failure_code', 'build_view_count', 'build_status', 'build_duration_s',
    'timeout_source', 'completion_level', 'failure_code_n',
)


def sanitize_request_id(request_id: str) -> str | None:
    """与调度侧同口径：拒绝路径穿越/分隔符；空白返回 None."""
    text = str(request_id or '').strip()
    if not text or text in ('.', '..') or '/' in text or '\\' in text \
            or '\0' in text:
        return None
    return text


def merge_job_landmarks(traj_landmarks: dict, job: dict) -> dict:
    """作业票坐标优先，重建许可镜像补缺（入口/预抓取/轴）."""
    coords = (job or {}).get('coords') or {}
    grasp = (job or {}).get('grasp') or {}
    merged = dict(traj_landmarks)
    mapping = {
        'perception_entry': coords.get('perception_entry'),
        'perception_bottom': coords.get('perception_bottom'),
        'perception_neck': coords.get('perception_neck'),
        'reconstruction_center': coords.get('reconstruction_center'),
        'grasp_entry': coords.get('grasp_entry'),
        'grasp_pregrasp': coords.get('grasp_pregrasp'),
        'axis': grasp.get('axis') or coords.get('refined_axis'),
    }
    for key, value in mapping.items():
        if value:
            merged[key] = value
    merged['target_id'] = (
        (job or {}).get('target_id') or merged.get('target_id') or '')
    return merged


def _round_s(value: float) -> float:
    return round(float(value), 3)


class StageTracker:
    """按 key 变化记录阶段转移；段时长在下一条到达时结算."""

    def __init__(self, limit: int = 150):
        """时间线保留条数（超限截头）."""
        self._limit = max(1, int(limit))
        self._key: tuple | None = None
        self._entries: list[dict] = []

    def feed(self, key: tuple, entry: dict, now: float) -> bool:
        """键未变化视为心跳去重；变化则结算上一段并追加新转移."""
        if key == self._key:
            return False
        self._key = key
        record = dict(entry)
        record['t'] = float(now)
        if self._entries:
            previous = self._entries[-1]
            previous['dur_s'] = _round_s(record['t'] - previous['t'])
        self._entries.append(record)
        if len(self._entries) > self._limit:
            del self._entries[:len(self._entries) - self._limit]
        return True

    def export(self, now: float) -> list[dict]:
        """浅拷贝时间线；末条未闭合段补 dur_s 并标 current=True."""
        exported = [dict(item) for item in self._entries]
        if exported and 'dur_s' not in exported[-1]:
            exported[-1]['dur_s'] = _round_s(now - exported[-1]['t'])
            exported[-1]['current'] = True
        return exported

    def reset(self) -> None:
        """清空时间线与去重键（cleanup 后重配不会串场）."""
        self._key = None
        self._entries = []


class LedgerWatch:
    """``runs/<request_id>/ledger.json`` 增量直播（mtime+size 缓存）."""

    def __init__(self, runs_root):
        """runs_root 已由节点侧解析（resolve_runs_root）."""
        self._root = Path(runs_root)
        self._last_id = ''
        self._last_sig: tuple | None = None
        self._payload: dict = {}

    def refresh(self, request_id: str) -> dict | None:
        """
        轮询入口：返回归一化载荷或 None（无变化不刷镜像）.

        request_id 为空（尚无批次）→ 返回清空载荷一次；文件不存在 →
        带 error 提示的空载荷（批次刚开、尚无终局属正常态）。
        """
        safe = sanitize_request_id(request_id)
        if safe is None:
            if self._payload:
                self._payload = {}
                self._last_id = ''
                self._last_sig = None
                return {}
            return None
        path = self._root / safe / 'ledger.json'
        try:
            stat = path.stat()
            sig = (safe, stat.st_mtime_ns, stat.st_size)
        except OSError:
            if safe == self._last_id:
                return None
            self._last_id = safe
            self._last_sig = None
            self._payload = self._empty(safe, path,
                                        '账本尚未生成（等待首颗终局）')
            return self._payload
        if sig == self._last_sig:
            return None
        self._last_id = safe
        self._last_sig = sig
        try:
            document = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(document, dict):
                raise ValueError('ledger 根节点不是对象')
            rows = [self._row(item)
                    for item in document.get('outcomes') or []]
            claimed = sorted(
                str(item) for item in document.get('claimed') or [] if item)
            self._payload = {
                'request_id': safe,
                'path': str(path),
                'mtime': stat.st_mtime,
                'error': None,
                'claimed': claimed,
                'rows': rows,
                'totals': self._totals(rows, claimed),
            }
        except (OSError, ValueError, TypeError, AttributeError) as error:
            self._payload = self._empty(safe, path, f'账本解析失败: {error}')
        return self._payload

    @staticmethod
    def _empty(safe: str, path: Path, error: str) -> dict:
        return {
            'request_id': safe,
            'path': str(path),
            'error': error,
            'claimed': [],
            'rows': [],
            'totals': {},
        }

    @staticmethod
    def _row(item) -> dict:
        """单行账本 → 前端行（阶段名/时长 zip 成 stages；缺省键容错）."""
        if not isinstance(item, dict):
            item = {}
        try:
            outcome = int(item.get('outcome', 3))
        except (TypeError, ValueError):
            outcome = 3
        names = [str(name) for name in item.get('stage_names') or []]
        durations = item.get('stage_durations') or []
        stages = []
        for index, name in enumerate(names):
            try:
                dur = _round_s(durations[index])
            except (IndexError, TypeError, ValueError):
                dur = None
            stages.append({'name': name, 'dur_s': dur})
        try:
            elapsed = _round_s(item.get('elapsed_s'))
        except (TypeError, ValueError):
            elapsed = None
        row = {
            'target_id': str(item.get('target_id') or ''),
            'outcome': outcome,
            'outcome_name': OUTCOME_NAMES.get(outcome, str(outcome)),
            'reason': str(item.get('reason') or ''),
            'quality_score': item.get('quality_score'),
            'elapsed_s': elapsed,
            'stages': stages,
        }
        for key in _ROW_PASSTHROUGH:
            if item.get(key) is not None:
                row[key] = item.get(key)
        return row

    @staticmethod
    def _totals(rows: list[dict], claimed: list[str]) -> dict:
        totals = {
            name: 0 for name in ('SUCCEEDED', 'SKIPPED_QUALITY',
                                 'SKIPPED_UNREACHABLE', 'FAILED', 'CANCELED')}
        for row in rows:
            key = str(row.get('outcome_name') or '')
            if key in totals:
                totals[key] += 1
        totals['attempted'] = len(rows)
        totals['claimed'] = len(claimed)
        return totals
