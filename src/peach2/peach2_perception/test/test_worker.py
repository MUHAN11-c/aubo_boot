import threading
import time

from peach2_perception.worker import LatestSlot


def test_drop_oldest():
    slot = LatestSlot()
    assert slot.put(1) is False
    assert slot.put(2) is True
    assert slot.dropped == 1
    assert slot.get(timeout=0.1) == 2
    assert slot.get(timeout=0.01) is None


def test_close_unblocks_waiter_and_reopen():
    slot = LatestSlot()
    got = []
    t = threading.Thread(target=lambda: got.append(slot.get(timeout=5.0)))
    t.start()
    time.sleep(0.05)
    slot.close()
    t.join(timeout=1.0)
    assert not t.is_alive() and got == [None]
    assert slot.put(3) is False and slot.get(timeout=0.01) is None
    slot.reopen()
    slot.put(4)
    assert slot.get(timeout=0.1) == 4


def test_clear():
    slot = LatestSlot()
    slot.put(1)
    slot.clear()
    assert slot.get(timeout=0.01) is None
