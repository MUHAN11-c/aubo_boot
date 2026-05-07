#!/usr/bin/env python3
"""最小化测试：用简单盒子测试 attach 到 kuaihuan_Link"""
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


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

        # 1) 先在世界上放一个红色盒子
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

        # 2) attach 到 kuaihuan_Link
        self.get_logger().info('--- 第2步: attach test_box → kuaihuan_Link ---')
        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.robot_state.is_diff = True
        att = AttachedCollisionObject()
        att.object.id = 'test_box'
        att.link_name = 'kuaihuan_Link'
        att.touch_links = ['kuaihuan_Link', 'wrist3_Link', 'camera_Link']
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
        det.link_name = 'kuaihuan_Link'
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
