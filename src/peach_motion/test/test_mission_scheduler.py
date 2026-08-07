"""ready_order 调度排序：置信×近距打分、非 READY 过滤."""
import unittest

import numpy as np

from peach_motion.target_registry import (
    DONE,
    NEW,
    OBSERVING,
    READY,
    ready_order,
    TargetRecord,
)


def _rec(tid, status, conf, x):
    """造档案（位置取 (x,0,0.5)，距原点越远近距分越低）."""
    return TargetRecord(target_id=tid, position=np.array([x, 0.0, 0.5]),
                        kind='bag', status=status, confidence=conf)


class ReadyOrderTest(unittest.TestCase):
    def test_only_ready_listed(self):
        """非 READY（NEW/OBSERVING/DONE/FAILED）一律不调度."""
        recs = [_rec('T001', NEW, 0.9, 0.1), _rec('T002', OBSERVING, 0.9, 0.1),
                _rec('T003', DONE, 0.9, 0.1)]
        self.assertEqual(ready_order(recs), [])

    def test_confidence_dominates_when_closer_tie(self):
        """同距时高置信优先."""
        near = _rec('T001', READY, 0.9, 0.3)
        far = _rec('T002', READY, 0.9, 0.9)
        order = ready_order([far, near])
        self.assertEqual([r.target_id for r in order], ['T001', 'T002'])

    def test_near_breaks_confidence_tie(self):
        """置信相同时近距优先；置信差 0.4 可盖过近距差."""
        conf_high_far = _rec('T001', READY, 0.9, 0.9)
        conf_low_near = _rec('T002', READY, 0.3, 0.1)
        order = ready_order([conf_low_near, conf_high_far])
        # 0.6·0.6=0.36 的置信差 > 近距差（≤0.4·(0.9−0.47)≈0.17）
        self.assertEqual(order[0].target_id, 'T001')

    def test_empty(self):
        """空列表 → 空调度队列."""
        self.assertEqual(ready_order([]), [])


if __name__ == '__main__':
    unittest.main()
