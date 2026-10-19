"""任务状态名稳定，供 RViz/日志对照."""

import math

from sim_bringup.orchard_harvest import (
    MissionState, fruit_survey_yaw, park_toward_fruit,
)


def test_mission_states_cover_the_harvest_loop():
    """阶段名与参考仓库同类：定位、巡视、规划、抓取、返回、终态."""
    assert [item.value for item in MissionState] == [
        'BOOT', 'LOCALIZE', 'STOW', 'NAVIGATE', 'SURVEY', 'PLAN_HARVEST',
        'HARVESTING', 'RETURN', 'DONE', 'STOPPED', 'FAILED',
    ]


def test_fruit_heading_faces_bags_not_the_trunk():
    """树 0 的袋在站点东侧；对准树干（-x）时光轴对着空地."""
    heading = fruit_survey_yaw(0.780, 0.077)
    assert heading > 1.0
    assert heading < 2.2


def test_park_toward_fruit_leaves_the_bag_inside_reach():
    """站点离袋心 0.78 m 时，前移到袋心只剩臂展距离."""
    shift_x, shift_y, aim_x, aim_y = park_toward_fruit(0.78, 0.0, 0.55)
    assert shift_x > 0.0
    assert abs(shift_y) < 1e-9
    assert abs(math.hypot(aim_x, aim_y) - 0.55) < 1e-6
