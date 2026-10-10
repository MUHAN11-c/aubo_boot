#!/usr/bin/env python3
"""
PlanningScene 巡检：世界障碍对象 + ACM 豁免条目（GetPlanningScene 权威）.

09-29 真相机轮排查工具收编：快照 box 数、目标邻域空洞靠 world 对象判定；
「起点碰撞但本该豁免」类问题靠 ACM 条目判定（如 tool_body_link ×
peach_scene_obstacles 应 allowed=true）。

用法：
  ros2 run peach_system_tests check_planning_scene.py
  ros2 run peach_system_tests check_planning_scene.py \
      --object peach_scene_obstacles --link tool_body_link
"""
import argparse
import sys

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPlanningScene


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--object', default='peach_scene_obstacles',
                        help='要核查 ACM 豁免的障碍对象 id')
    parser.add_argument(
        '--link', action='append', default=None,
        help='要核查的连杆（可多次；默认 tool_body_link/camera_body_link/tcp）')
    args = parser.parse_args()
    probe_links = args.link or ['tool_body_link', 'camera_body_link', 'tcp']

    rclpy.init()
    node = Node('planning_scene_check')
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    if not client.wait_for_service(timeout_sec=5.0):
        print('SERVICE_UNAVAILABLE（move_group 未起？）')
        return 1
    request = GetPlanningScene.Request()
    request.components.components = 255  # ALL
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    response = future.result()
    if response is None:
        print('CALL_TIMEOUT')
        return 1

    world = response.scene.world.collision_objects
    print(f'world objects: {len(world)}')
    for obj in world:
        primitive_count = sum(len(o.primitives) for o in [obj])
        pose_frame = (obj.primitive_poses[0].position
                      if obj.primitive_poses else None)
        print(
            f'  {obj.id}: primitives={primitive_count} '
            f"frame={obj.header.frame_id or (pose_frame and '(pose相对)')}"
            f'{" REMOVE" if obj.operation == 1 else ""}')

    acm = response.scene.allowed_collision_matrix
    names = list(acm.entry_names)
    index = {n: i for i, n in enumerate(names)}
    print(f'ACM entries: {len(names)}')
    target = args.object
    for link in probe_links:
        if link not in index:
            print(f'  {link} x {target}: LINK_NOT_IN_ACM')
            continue
        row = acm.entry_values[index[link]].enabled
        allowed = target in names and row[names.index(target)]
        print(f'  {link} x {target}: allowed={bool(allowed)}')
    if target not in index:
        print(f'  {target}: OBJECT_NOT_IN_ACM（豁免未写入！）')
    else:
        row = acm.entry_values[index[target]].enabled
        allowed = [names[c] for c, e in enumerate(row) if e]
        print(f'  {target}: allowed_links={len(allowed)}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
