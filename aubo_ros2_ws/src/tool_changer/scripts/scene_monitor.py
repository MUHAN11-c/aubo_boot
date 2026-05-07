#!/usr/bin/env python3
"""监控 PlanningScene 中 attached objects 和 world objects 的变化"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene


class SceneMonitor(Node):
    def __init__(self):
        super().__init__('scene_monitor')
        # 监听 monitored_planning_scene（move_group 发布的当前场景）
        self.sub = self.create_subscription(
            PlanningScene, '/monitored_planning_scene', self._cb, 10)
        self.get_logger().info('监听 /monitored_planning_scene ...')

    def _cb(self, scene: PlanningScene):
        attached = scene.robot_state.attached_collision_objects
        world = scene.world.collision_objects
        if attached:
            for a in attached:
                self.get_logger().info(
                    f'[ATTACHED] {a.object.id} → {a.link_name}')
        if world:
            ids = [o.id for o in world]
            self.get_logger().info(f'[WORLD] {len(world)} objects: {ids}')


def main():
    rclpy.init()
    rclpy.spin(SceneMonitor())


if __name__ == '__main__':
    main()
