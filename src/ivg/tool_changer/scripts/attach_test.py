#!/usr/bin/env python3
"""
独立脚本：直接向 /planning_scene 发 diff（与 scene_attach_worker 生产路径不同）.

生产环境工具附着请使用节点 scene_attach_worker
（/attached_collision_object + world REMOVE）或服务 /scene_attach、
话题 /tool_changer_status。附着帧为本仓 quick_changer_link。
"""
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

ATTACH_FRAME = 'quick_changer_link'


class AttachTest(Node):

    def __init__(self):
        super().__init__('attach_test')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_timer(3.0, self._run_test)

    def _run_test(self):
        if not self.client.wait_for_service(3.0):
            self.get_logger().error('无 /apply_planning_scene 服务')
            return

        # 1) 先在世界上放一个盒子
        self.get_logger().info('--- 第1步: 添加世界盒子 ---')
        scene = PlanningScene()
        scene.is_diff = True
        box = CollisionObject()
        box.id = 'test_box'
        box.header.frame_id = 'base_link'
        box.operation = CollisionObject.ADD
        box.pose.position.x = 0.3
        box.pose.position.y = 0.0
        box.pose.position.z = 0.3
        box.pose.orientation.w = 1.0
        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = [0.05, 0.05, 0.1]
        box.primitives.append(p)
        pp = Pose()
        pp.orientation.w = 1.0
        box.primitive_poses.append(pp)
        scene.world.collision_objects.append(box)
        self._call(scene)

        time.sleep(2.0)

        # 2) attach 到附着帧
        self.get_logger().info(f'--- 第2步: attach test_box -> {ATTACH_FRAME} ---')
        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.robot_state.is_diff = True
        att = AttachedCollisionObject()
        att.object.id = 'test_box'
        att.link_name = ATTACH_FRAME
        att.touch_links = [ATTACH_FRAME, 'wrist3_Link']
        att.object.operation = CollisionObject.ADD
        att.object.primitives.append(p)
        att.object.primitive_poses.append(pp)
        att.object.pose.orientation.w = 1.0
        scene2.robot_state.attached_collision_objects.append(att)
        self._call(scene2)

        time.sleep(2.0)

        # 3) detach
        self.get_logger().info('--- 第3步: detach test_box ---')
        scene3 = PlanningScene()
        scene3.is_diff = True
        scene3.robot_state.is_diff = True
        det = AttachedCollisionObject()
        det.object.id = 'test_box'
        det.object.operation = CollisionObject.REMOVE
        det.link_name = ATTACH_FRAME
        scene3.robot_state.attached_collision_objects.append(det)
        self._call(scene3)

        self.get_logger().info('--- 测试完成 ---')

    def _call(self, scene):
        self.pub.publish(scene)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result():
            self.get_logger().info(f'  结果: success={future.result().success}')
        else:
            self.get_logger().error('  超时或失败')


def main():
    rclpy.init()
    rclpy.spin(AttachTest())


if __name__ == '__main__':
    main()
