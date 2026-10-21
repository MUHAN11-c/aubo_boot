"""调试图标签：序号是抓取顺序，分割和检测框都会画上."""
import numpy as np

from peach_harvester.vision.scene_perception.debug_draw import (
    draw_debug,
    grasp_order_by_id,
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
