"""Idempotent ledger close (no ROS)."""
from __future__ import annotations


class LedgerIndex:
    """按 transaction_id 只结案一次."""

    def __init__(self):
        self._closed = {}

    def close(self, transaction_id: str, target_id: str, outcome: int) -> bool:
        """True=首次入账；False=重复/空事务丢弃."""
        key = str(transaction_id or '')
        if not key:
            return False
        if key in self._closed:
            return False
        self._closed[key] = (str(target_id), int(outcome))
        return True

    def get(self, transaction_id: str):
        return self._closed.get(str(transaction_id or ''))
