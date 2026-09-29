"""EventMeter 单测：首次必记 / 周期摘要 / 不刷屏（零 ROS）."""
import logging

from peach_common.event_meter import EventMeter


class _Capture(logging.Handler):

    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record.getMessage())


def _meter(every=3):
    logger = logging.getLogger('test_event_meter')
    logger.handlers = []
    capture = _Capture()
    logger.addHandler(capture)
    logger.setLevel(logging.INFO)
    return EventMeter(logger, '测试事件', every=every), capture


def test_first_hit_and_periodic_summary():
    meter, capture = _meter(every=3)
    meter.hit('a')
    meter.hit('b')
    meter.hit('c')
    assert len(capture.records) == 2  # 第 1 次 + 第 3 次
    assert '累计 1 次' in capture.records[0] and 'a' in capture.records[0]
    assert '累计 3 次' in capture.records[1] and 'c' in capture.records[1]
    meter.hit('d')
    assert len(capture.records) == 2
    assert meter.count == 4


def test_snapshot():
    meter, _ = _meter(every=100)
    meter.hit('原因x')
    snap = meter.snapshot()
    assert snap['count'] == 1
    assert snap['last_detail'] == '原因x'
    assert snap['name'] == '测试事件'
