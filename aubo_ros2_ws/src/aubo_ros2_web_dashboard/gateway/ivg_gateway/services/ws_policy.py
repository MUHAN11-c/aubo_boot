"""
rosbridge 协议出站（浏览器→rosbridge）侧策略。

- parse_rosbridge_op：解析单帧 JSON 的 op 字段；非 JSON 或解析失败则 op=None（通常放行）。
- viewer：禁止 advertise / publish / call_service。
- deny_substrings：call_service 时匹配 service 名子串则拒绝。
- ClientMessageRateLimiter：滑动窗口每秒条数。
- PublishDebouncer：同一 topic 在 debounce_ms 内只保留第一条后的更新节奏。
"""

from __future__ import annotations

import json
import time
from collections import deque
from typing import Deque, Dict, List, Tuple

from ivg_gateway.models import UserRole


def parse_rosbridge_op(text: str) -> Tuple[str | None, dict | None]:
    s = text.strip()
    if not s.startswith("{"):
        return None, None
    try:
        obj = json.loads(s)
    except json.JSONDecodeError:
        return None, None
    if not isinstance(obj, dict):
        return None, None
    op = obj.get("op")
    return (str(op) if op is not None else None), obj


# rosbridge v2 常见写操作；viewer 仅允许 subscribe/unsubscribe 等读向操作
_VIEWER_BLOCKED = frozenset({"advertise", "publish", "call_service"})


def allowed_by_role(role: str, op: str | None) -> Tuple[bool, str]:
    if op is None:
        return True, ""
    if role == UserRole.viewer.value and op in _VIEWER_BLOCKED:
        return False, f"角色 viewer 禁止 rosbridge 操作: {op}"
    return True, ""


def service_denied(service_name: str | None, deny_substrings: List[str]) -> Tuple[bool, str]:
    if not service_name or not deny_substrings:
        return False, ""
    sn = service_name.lower()
    for frag in deny_substrings:
        if frag.lower() in sn:
            return True, f"服务在禁区列表中: {service_name}"
    return False, ""


def check_outbound_json(
    role: str,
    text: str,
    *,
    deny_substrings: List[str],
) -> Tuple[bool, str]:
    op, obj = parse_rosbridge_op(text)
    ok, reason = allowed_by_role(role, op)
    if not ok:
        return False, reason
    if op == "call_service" and isinstance(obj, dict):
        svc = obj.get("service")
        if isinstance(svc, str):
            denied, r2 = service_denied(svc, deny_substrings)
            if denied:
                return False, r2
    return True, ""


class ClientMessageRateLimiter:
    def __init__(self, per_second: int) -> None:
        self.per_second = max(0, int(per_second))
        self._ts: Deque[float] = deque()

    def allow(self) -> Tuple[bool, str]:
        if self.per_second <= 0:
            return True, ""
        now = time.monotonic()
        while self._ts and now - self._ts[0] > 1.0:
            self._ts.popleft()
        if len(self._ts) >= self.per_second:
            return False, "超过 WebSocket 客户端消息速率限制"
        self._ts.append(now)
        return True, ""


class PublishDebouncer:
    def __init__(self, debounce_ms: int) -> None:
        self.debounce_ms = max(0, int(debounce_ms))
        self._last: Dict[str, float] = {}

    def should_send_publish(self, topic: str) -> Tuple[bool, str]:
        if self.debounce_ms <= 0:
            return True, ""
        now = time.monotonic() * 1000
        t = self._last.get(topic)
        if t is not None and now - t < self.debounce_ms:
            return False, "publish 去抖丢弃"
        self._last[topic] = now
        return True, ""
