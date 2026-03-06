#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定静态 TF 发布节点：根据 YAML 中的标定矩阵发布 末端(ee_frame_id) -> camera_frame 的静态变换。

标定矩阵含义：T_ee_camera 为 相机坐标系 → 机械臂末端坐标系，平移单位为毫米。
发布的是 parent=ee_frame_id, child=child_frame_id（默认 camera_frame）的静态 TF。
"""

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def load_hand_eye_tf_args(yaml_path):
    """
    从手眼标定 YAML 读取变换矩阵，返回 (x, y, z, yaw, pitch, roll)。
    矩阵含义：相机坐标系 → 机械臂末端坐标系（T_ee_camera），平移单位为毫米，返回的平移为米。
    若文件不存在或格式无效则返回 None。
    """
    if not yaml_path or not os.path.exists(yaml_path):
        return None
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        he = data.get('hand_eye_calibration', {})
        T_list = he.get('transformation_matrix', None)
        if not T_list:
            return None
        T = np.array(T_list, dtype=np.float64)
        if T.shape != (4, 4):
            return None
        # 平移：毫米 -> 米
        x = T[0, 3] / 1000.0
        y = T[1, 3] / 1000.0
        z = T[2, 3] / 1000.0
        R = T[:3, :3]
        euler = ScipyRotation.from_matrix(R).as_euler('zyx', degrees=False)
        yaw, pitch, roll = float(euler[0]), float(euler[1]), float(euler[2])
        return (x, y, z, yaw, pitch, roll)
    except Exception:
        return None


class HandEyeStaticTFNode(Node):
    """发布手眼标定静态 TF：parent=ee_frame_id -> child=child_frame_id。"""

    def __init__(self):
        super().__init__('hand_eye_static_tf_node')
        self.declare_parameter('hand_eye_yaml_path', '')
        self.declare_parameter('ee_frame_id', 'wrist3_Link')
        self.declare_parameter('child_frame_id', 'camera_frame')

        hand_eye_yaml = self.get_parameter('hand_eye_yaml_path').get_parameter_value().string_value
        self._parent_frame = self.get_parameter('ee_frame_id').get_parameter_value().string_value
        self._child_frame = self.get_parameter('child_frame_id').get_parameter_value().string_value

        tf_args = load_hand_eye_tf_args(hand_eye_yaml)
        if tf_args is not None:
            x, y, z, yaw, pitch, roll = tf_args
            self.get_logger().info(
                f'已加载手眼标定: {hand_eye_yaml} -> 发布 {self._parent_frame} -> {self._child_frame}'
            )
        else:
            x = y = z = yaw = pitch = roll = 0.0
            self.get_logger().info(
                f'未使用标定文件，发布单位变换: {self._parent_frame} -> {self._child_frame}'
            )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = ScipyRotation.from_euler('zyx', [yaw, pitch, roll]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(t)
        self.get_logger().info('手眼静态 TF 已发布一次。')


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeStaticTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
