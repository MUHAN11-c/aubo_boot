#!/usr/bin/env python3
"""
直接使用Python API发布障碍物到MoveIt规划场景
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class ObstaclePublisher(Node):
    """发布障碍物到规划场景"""
    
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # 发布碰撞对象到规划场景监控话题
        self.collision_object_publisher = self.create_publisher(
            CollisionObject,
            '/move_group/planning_scene_monitor',
            10
        )
        
        # 发布规划场景话题
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            '/move_group/publish_planning_scene',
            10
        )
        
        # 等待发布者就绪
        time.sleep(1.0)
        
        self.get_logger().info('障碍物发布节点已启动')
        
        # 发布障碍物
        self.publish_obstacle()
    
    def publish_obstacle(self):
        """发布障碍物到规划场景"""
        # 创建碰撞对象
        collision_object = CollisionObject()
        collision_object.id = "test_obstacle"
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        
        # 定义障碍物的形状和尺寸
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.2, 0.2, 0.2]  # 长、宽、高（米）
        
        # 定义障碍物的位置
        box_pose = Pose()
        box_pose.position.x = 0.3
        box_pose.position.y = 0.0
        box_pose.position.z = 1.0  # 用户指定的z坐标
        box_pose.orientation.w = 1.0
        
        # 将形状和位置添加到障碍物对象中
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        # 方法1：直接发布碰撞对象到规划场景监控话题（这是move_group监听的话题）
        self.get_logger().info(f'发布障碍物到 /move_group/planning_scene_monitor: {collision_object.id}')
        for i in range(3):  # 发布3次，确保障碍物被接收
            self.collision_object_publisher.publish(collision_object)
            time.sleep(0.2)
        
        # 等待一下，让move_group处理
        time.sleep(1.0)
        
        # 方法2：发布完整的规划场景（备用方法）
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self.get_logger().info(f'发布规划场景到 /move_group/publish_planning_scene: {collision_object.id}')
        for i in range(3):  # 发布3次，确保障碍物被接收
            self.planning_scene_publisher.publish(planning_scene)
            time.sleep(0.2)
        
        self.get_logger().info('障碍物已发布，请在RViz2中检查是否显示')
        self.get_logger().info('障碍物ID: ' + collision_object.id)
        self.get_logger().info('位置: x={}, y={}, z={}'.format(
            box_pose.position.x, box_pose.position.y, box_pose.position.z))
        self.get_logger().info('尺寸: {}x{}x{}'.format(
            box.dimensions[0], box.dimensions[1], box.dimensions[2]))


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstaclePublisher()
    
    # 保持节点运行一段时间，确保障碍物被处理
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()
    
    print("\n障碍物发布完成！")
    print("请在RViz2中检查Motion Planning插件的Scene Geometry部分，应该能看到障碍物")


if __name__ == '__main__':
    main()

