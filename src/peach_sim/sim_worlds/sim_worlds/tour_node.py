"""
``orchard_tour`` 巡航节点：逐停靠位发 NavigateToPose（thin 接线层）.

工位序列来自纯核 ``tour_core.stations``（config/orchard.yaml 单源）；
导航本身全走 Nav2 官方 ``nav2_simple_commander``。每站到点后停留
``dwell`` 秒模拟采摘节拍（本轮不动臂）。仿真用途，不进 harvest_system。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
import yaml

from .cli import default_share
from .params import check_params, params_from_dict
from .tour_core import stations_by_row


def _pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw: float):
    """构造 map 系目标位姿（stamp 走节点时钟，与仿真时钟一致）."""
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def main(argv: list[str] | None = None) -> int:
    share = default_share()
    parser = argparse.ArgumentParser(
        prog='orchard_tour',
        description='果园巡航：逐树停靠位发 NavigateToPose（须先起 orchard_nav）')
    parser.add_argument(
        '--params', type=Path, default=share / 'config' / 'orchard.yaml',
        help='场景参数 yaml（停靠位几何唯一事实源）')
    parser.add_argument(
        '--rows', default='',
        help='只巡航的行号（逗号分隔，如 2,3；默认全部行）')
    parser.add_argument(
        '--dwell', type=float, default=3.0,
        help='每站到点后的停留秒数（模拟采摘节拍）')
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='单站导航超时秒数')
    args = parser.parse_args(argv)

    raw = yaml.safe_load(args.params.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    problems += check_params(params)
    if problems:
        print(f'{args.params}: 场景参数不合法：' + '；'.join(problems))
        return 2
    rows = [int(item) for item in args.rows.split(',') if item.strip()] \
        if args.rows else list(range(params.rows.count))
    plan = stations_by_row(params, rows)
    print(f'巡航计划：{len(plan)} 站 / 行 {rows}，dwell={args.dwell}s')

    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', value=True)])
    log = navigator.get_logger()
    log.info('等待 Nav2 激活（amcl + bt_navigator）…')
    navigator.waitUntilNav2Active(localizer='amcl')
    log.info('Nav2 已激活，开始巡航')

    ok = 0
    for number, station in enumerate(plan, start=1):
        goal = _pose_stamped(navigator, station.x, station.y, station.yaw)
        log.info(f'[{number}/{len(plan)}] {station.tree_id} → '
                 f'({station.x:.2f}, {station.y:.2f}, yaw {station.yaw:.2f})')
        navigator.goToPose(goal)
        deadline = time.monotonic() + args.timeout
        while not navigator.isTaskComplete():
            if time.monotonic() > deadline:
                navigator.cancelTask()
                break
        if navigator.getResult() == TaskResult.SUCCEEDED:
            ok += 1
            log.info(f'到站 {station.tree_id}')
            time.sleep(args.dwell)
        else:
            log.warning(f'{station.tree_id} 未到站（{navigator.getResult()}），跳过')

    log.info(f'巡航结束：{ok}/{len(plan)} 站到点')
    rclpy.shutdown()
    return 0 if ok == len(plan) else 1


if __name__ == '__main__':
    raise SystemExit(main())
