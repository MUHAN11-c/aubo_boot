#!/usr/bin/env python3
"""
MoveIt2 TCP位姿发布器
使用MoveIt2的正向运动学计算机器人末端执行器的准确位姿
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time


class MoveIt2TcpPosePublisher(Node):
    def __init__(self):
        super().__init__('moveit2_tcp_pose_publisher')
        
        # 声明参数
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'wrist3_Link')
        self.declare_parameter('publish_rate_hz', 20.0)
        
        # 获取参数
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        
        self.get_logger().info('🚀 初始化MoveIt2 TCP位姿发布器（TF版本）')
        self.get_logger().info(f'   基座坐标系: {self.base_frame}')
        self.get_logger().info(f'   末端执行器: {self.end_effector_link}')
        self.get_logger().info(f'   发布频率: {self.publish_rate} Hz')
        
        # 创建TF监听器
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info('✅ TF2监听器初始化成功')
        except Exception as e:
            self.get_logger().error(f'❌ TF2初始化失败: {e}')
            raise
        
        # 创建发布器
        self.tcp_pose_pub = self.create_publisher(
            PoseStamped,
            '/moveit2_tcp_pose',
            10
        )
        
        # 创建定时器，定期发布TCP位姿
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_tcp_pose
        )
        
        self.last_log_time = time.time()
        
        self.get_logger().info('✅ 发布到 /moveit2_tcp_pose')
        self.get_logger().info('🎯 MoveIt2 TCP位姿发布器启动完成（基于TF树）')
    
    def publish_tcp_pose(self):
        """定期发布TCP位姿（基于TF树）"""
        try:
            # 从TF树获取末端执行器相对于基座的变换
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 创建PoseStamped消息
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation.x = transform.transform.rotation.x
            pose_stamped.pose.orientation.y = transform.transform.rotation.y
            pose_stamped.pose.orientation.z = transform.transform.rotation.z
            pose_stamped.pose.orientation.w = transform.transform.rotation.w
            
            # 发布位姿
            self.tcp_pose_pub.publish(pose_stamped)
            
            # 定期打印日志（每5秒）
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:
                self.get_logger().info(
                    f'📍 TCP位姿: [{transform.transform.translation.x:.3f}, '
                    f'{transform.transform.translation.y:.3f}, '
                    f'{transform.transform.translation.z:.3f}] '
                    f'[{transform.transform.rotation.x:.3f}, '
                    f'{transform.transform.rotation.y:.3f}, '
                    f'{transform.transform.rotation.z:.3f}, '
                    f'{transform.transform.rotation.w:.3f}]'
                )
                self.last_log_time = current_time
                
        except Exception as e:
            # TF查找失败是正常的（刚启动或TF不可用），不需要每次都打印错误
            pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MoveIt2TcpPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
