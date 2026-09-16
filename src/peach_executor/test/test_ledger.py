"""Idempotent ledger close."""
from peach_executor.domain.ledger import LedgerIndex


def test_close_once():
    ledger = LedgerIndex()
    assert ledger.close('txn-1', 't1', 0) is True
    assert ledger.close('txn-1', 't1', 0) is False
    assert ledger.close('', 't1', 0) is False
    assert ledger.get('txn-1') == ('t1', 0)
