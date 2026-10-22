"""调试图标签：序号是抓取顺序，分割和检测框都会画上."""
import numpy as np

from peach_harvester.vision.scene_perception.debug_draw import (
    draw_debug,
    grasp_order_by_id,
    orders_for_image,
    stick_labels,
)


class _Grasp:
    status = 'ACCEPT'
    bottom_px = None
    neck_px = None
    grasp_px = None
    travel_line = None


def test_locked_order_follows_plan_priority():
    """确认锁定后，图上的 id 来自抓取顺序，不看 target_id 的数字."""
    orders = grasp_order_by_id(
        True, {'target_2': 1, 'target_0': 3, 'target_1': 2})
    assert orders == {'target_2': 1, 'target_1': 2, 'target_0': 3}


def test_unlocked_has_no_grasp_id():
    """锁定前还没有确认后的抓取顺序，不编 id."""
    assert grasp_order_by_id(False, {'target_2': 1}) == {}


def test_image_keeps_previous_order_until_relock():
    """收齐窗重开、身份还在时，画面继续标上一套顺序."""
    remembered = {'target_0': 1, 'target_1': 2}
    assert orders_for_image(False, {}, remembered) == remembered
    assert orders_for_image(True, {'target_1': 1, 'target_0': 2}, remembered) == {
        'target_1': 1, 'target_0': 2,
    }


def test_label_stays_when_detection_drops():
    """当前锁定里的号，这一帧没检出也留在原位置."""
    item = {
        'tid': 'target_0', 'order': 1, 'bbox': (10, 10, 40, 40),
        'det': {'bbox': (10, 10, 40, 40)}, 'confirmed': True,
    }
    orders = {'target_0': 1, 'target_1': 2}
    drawn, memory = stick_labels([item], {}, orders)
    assert [row['tid'] for row in drawn] == ['target_0']
    drawn, memory = stick_labels([], memory, orders)
    assert drawn[0]['tid'] == 'target_0'
    assert drawn[0]['order'] == 1
    assert drawn[0]['bbox'] == (10, 10, 40, 40)


def test_new_lock_drops_ids_that_left():
    """新锁定换了名单后，旧号不再留在画面上，也不套到别的检测."""
    _, memory = stick_labels([{
        'tid': 'target_5', 'order': 1, 'bbox': (10, 10, 40, 40),
        'det': {'bbox': (10, 10, 40, 40)}, 'confirmed': True,
    }], {}, {'target_5': 1})
    drawn, memory = stick_labels([{
        'tid': 'target_1', 'order': 0, 'bbox': (12, 12, 42, 42),
        'det': {'bbox': (12, 12, 42, 42)}, 'confirmed': True,
    }], memory, {'target_1': 1})
    assert [row['tid'] for row in drawn] == ['target_1']
    assert drawn[0]['order'] == 1
    assert 'target_5' not in memory


def test_locked_image_hides_ids_outside_the_order():
    """顺序已经标上之后，过程中新冒出来的号和裸置信度不再叠到画面上."""
    fruit = {
        'tid': 'target_2', 'bbox': (80, 10, 120, 50),
        'det': {'bbox': (80, 10, 120, 50), 'conf': 0.9}, 'confirmed': True,
    }
    extra = {
        'tid': 'target_3', 'bbox': (90, 12, 130, 52),
        'det': {'bbox': (90, 12, 130, 52), 'conf': 0.44}, 'confirmed': True,
    }
    orders = {'target_0': 2, 'target_1': 1, 'target_2': 3}
    drawn, memory = stick_labels([fruit, extra], {}, orders)
    assert [row['tid'] for row in drawn] == ['target_2']
    assert drawn[0]['order'] == 3
    assert 'target_3' not in memory
    before_lock, _ = stick_labels([fruit, extra], {}, {})
    assert {row['tid'] for row in before_lock} == {'target_2', 'target_3'}


def test_draw_paints_box_and_mask_for_unconfirmed():
    """未确认目标也画出检测框和分割."""
    image = np.zeros((80, 80, 3), dtype=np.uint8)
    mask = np.zeros((80, 80), dtype=np.uint8)
    mask[50:70, 50:70] = 1
    draw_debug(
        image,
        {'bbox': (4, 4, 24, 24), 'class_id': 0, 'conf': 0.5},
        _Grasp(), mask, tid='target_2', confirmed=False, order=1)
    assert int(image[60, 60].sum()) > 0
    assert int(image[4, 4].sum()) > 0
