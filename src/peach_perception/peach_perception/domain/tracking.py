"""身份跟踪门：权威表只在精确 TF 上提交."""
from __future__ import annotations

from peach_perception.domain.evidence import may_commit_identity
from peach_perception.scene_perception.identity import TargetRegistry

__all__ = ['TargetRegistry', 'may_commit_identity', 'registry']


def registry() -> TargetRegistry:
    """默认注册表；生产节点仍自己构造并注入时钟."""
    return TargetRegistry()
