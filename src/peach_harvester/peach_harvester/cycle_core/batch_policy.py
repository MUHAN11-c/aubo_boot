"""批次策略与补采清单（清洁重写轮 3c 纯核，零 ROS）。

跳过是调度参数不是失败（FFRobotics 先例）：采收率/单果时限/扇区时限超限
即跳过，跳过目标自动入 rework_list.json 人工补采清单；策略来源
RunHarvest goal（开批初值）与 SetBatchPolicy 服务（运行期改）。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class BatchPolicy:
    """批次策略值对象（0=不限语义统一）。"""

    target_harvest_ratio: float = 0.0   # 0..1；0=不限
    per_target_timeout_s: float = 0.0   # 单果时限；0=不限
    sector_timeout_s: float = 0.0       # 扇区时限；0=不限
    view_policy: int = 0                # 0=fast（重写轮默认）/1=conservative

    VIEW_FAST = 0
    VIEW_CONSERVATIVE = 1

    @classmethod
    def from_goal(cls, goal_dict: dict) -> 'BatchPolicy':
        """RunHarvest goal 字段 → 策略（缺省 0=不限/fast）。"""
        return cls(
            target_harvest_ratio=float(goal_dict.get('target_harvest_ratio', 0.0)),
            per_target_timeout_s=float(goal_dict.get('per_target_timeout_s', 0.0)),
            sector_timeout_s=float(goal_dict.get('sector_timeout_s', 0.0)),
            view_policy=int(goal_dict.get('view_policy', 0)),
        )

    def with_updates(self, **updates) -> 'BatchPolicy':
        data = dict(
            target_harvest_ratio=self.target_harvest_ratio,
            per_target_timeout_s=self.per_target_timeout_s,
            sector_timeout_s=self.sector_timeout_s,
            view_policy=self.view_policy)
        for key, value in updates.items():
            if value is not None:
                data[key] = value
        return BatchPolicy(**data)


@dataclass
class TargetDeadline:
    """单果时限簿记：dispatch 时刻起算，超限即跳过。"""

    started_s: float
    timeout_s: float

    def exceeded(self, now_s: Optional[float] = None) -> bool:
        if self.timeout_s <= 0.0:
            return False
        now = time.monotonic() if now_s is None else now_s
        return now - self.started_s >= self.timeout_s

    def remaining_s(self, now_s: Optional[float] = None) -> float:
        if self.timeout_s <= 0.0:
            return float('inf')
        now = time.monotonic() if now_s is None else now_s
        return self.timeout_s - (now - self.started_s)


def ratio_reached(harvested: int, discovered: int,
                  policy: BatchPolicy) -> bool:
    """目标采收率已达（0=不限恒 False；discovered=0 无意义恒 False）。"""
    if policy.target_harvest_ratio <= 0.0 or discovered <= 0:
        return False
    return harvested >= policy.target_harvest_ratio * discovered


# ---- 补采清单（人工补采是一等出口：失败分类学落地的载体） ----

REWORK_KINDS = (
    'timeout',            # 单果/扇区时限超限
    'ratio_satisfied',    # 采收率已达，余果不再尝试
    'unreachable',        # 不可达/护栏拒发
    'quality',            # 质量门拒绝
    'occluded',           # 遮挡
    'contact_failed',     # 接触/脱离失败
    'dropped',            # 落果未承接（预留）
    'operator_skipped',   # 人工跳过
)


@dataclass
class ReworkList:
    """跳过目标清单：批末随账本落 runs/<request_id>/rework_list.json。"""

    request_id: str
    entries: list = field(default_factory=list)

    def append(self, target_id: str, kind: str, reason: str,
               attempted: bool = True) -> None:
        if kind not in REWORK_KINDS:
            raise ValueError(f'未知补采类别 {kind!r}（可用：{REWORK_KINDS}）')
        self.entries.append({
            'target_id': target_id,
            'kind': kind,
            'reason': reason,
            'attempted': attempted,
        })

    def save(self, runs_root: Path) -> Path:
        out = Path(runs_root) / self.request_id / 'rework_list.json'
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(
            json.dumps(
                {'request_id': self.request_id, 'entries': self.entries},
                ensure_ascii=False, indent=1),
            encoding='utf-8')
        return out

    @classmethod
    def load(cls, runs_root: Path, request_id: str) -> 'ReworkList':
        path = Path(runs_root) / request_id / 'rework_list.json'
        out = cls(request_id=request_id)
        if path.exists():
            doc = json.loads(path.read_text(encoding='utf-8'))
            out.entries = list(doc.get('entries', []))
        return out

    def kinds_count(self) -> dict:
        counts: dict = {}
        for entry in self.entries:
            counts[entry['kind']] = counts.get(entry['kind'], 0) + 1
        return counts
