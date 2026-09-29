#!/usr/bin/env python3
"""
操作台使能 + RunHarvest goal 发送器（对已起栈的系统测驱动）.

把 09-29 真相机轮手工 /tmp 驱动脚本收编为包内资产（测试相关统一放
peach_system_tests）。不建图、不代授权——只替代人手敲 ros2 service /
action 命令；使能语义仍=操作台运行时开关。

用法（栈已在跑；ROS_DOMAIN_ID 由环境决定）：
  ros2 run peach_system_tests drive_harvest.py \
      --request-id lab_pregrasp_20260929 --intent 0 --grasp
  ros2 run peach_system_tests drive_harvest.py \
      --request-id lab_survey_20260929 --intent 2 --skip-enables
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peach_interfaces.action import RunHarvest
from peach_interfaces.srv import SetEnables


def _send_enables(node: Node, grasp: bool, reason: str) -> bool:
    client = node.create_client(SetEnables, '/peach_supervisor/set_enables')
    if not client.wait_for_service(timeout_sec=10.0):
        print('set_enables SERVICE_UNAVAILABLE')
        return False
    request = SetEnables.Request()
    request.execution = True
    request.grasp = grasp
    request.tool = False
    request.reason = reason
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    response = future.result()
    ok = response is not None
    print(f'enables(execution=True grasp={grasp} tool=False): {"OK" if ok else "FAIL"}')
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-id', required=True, help='批次 id（不复用）')
    parser.add_argument(
        '--intent', type=int, default=0,
        help='0=FULL（execute_pregrasp_only 时止于预抓取） 2=SURVEY_ONLY')
    parser.add_argument('--scene-key', default='lab')
    parser.add_argument('--profile', default='default')
    parser.add_argument('--grasp', action='store_true', help='使能 grasp（PREVIEW/接近链需要）')
    parser.add_argument('--reason', default='peach_system_tests drive_harvest')
    parser.add_argument('--skip-enables', action='store_true', help='只发 goal 不动使能')
    parser.add_argument('--timeout', type=float, default=420.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('drive_harvest')
    try:
        if not args.skip_enables:
            if not _send_enables(node, args.grasp, args.reason):
                return 1
        action_client = ActionClient(node, RunHarvest, '/peach_supervisor/run_harvest')
        if not action_client.wait_for_server(timeout_sec=10.0):
            print('run_harvest ACTION_UNAVAILABLE')
            return 1
        goal = RunHarvest.Goal()
        goal.request_id = args.request_id
        goal.scene_key = args.scene_key
        goal.profile_id = args.profile
        goal.intent = args.intent
        print(f'goal: request_id={args.request_id} intent={args.intent}')
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            print('goal REJECTED')
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        wrapper = result_future.result()
        if wrapper is None:
            print(f'goal TIMEOUT(>={args.timeout}s) 或中断')
            return 2
        status_names = {
            0: 'STATUS_UNKNOWN', 1: 'STATUS_ACCEPTED', 2: 'STATUS_EXECUTING',
            3: 'STATUS_CANCELED', 4: 'STATUS_SUCCEEDED', 5: 'STATUS_ABORTED',
        }
        status = status_names.get(wrapper.status, str(wrapper.status))
        result = wrapper.result
        elapsed = result.elapsed.sec + result.elapsed.nanosec * 1e-9
        print(
            f'RESULT status={status} quality_score={result.quality_score} '
            f'elapsed={elapsed:.1f}s')
        return 0 if wrapper.status == 4 else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
