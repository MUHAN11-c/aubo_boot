#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 抓取检测节点（点云版）.

订阅 PointCloud2 → 工作区过滤 → GraspNet 推理 → 碰撞/NMS/topK →
发布 MarkerArray（相机系）+ 动态 TF（grasp_pose_i）+ PoseArray（base 系）。

采集节奏与旧栈兼容：默认待命；调 /graspnet_capture_control (std_srvs/SetBool)
True 开始一个采集会话，累计 capture_groups_target 组有效结果后自动停止。

推理核心在 graspnet_ros2.grasp_core（无 rclpy 依赖，纯 torch 后端），
节点只负责 IO/参数/发布，与 anygrasp_with_ros 的「采集-推理-发布」分层一致。
"""

from __future__ import annotations

import os
from typing import Optional

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from graspnet_ros2.grasp_core import (
    GraspList,
    graspnet_to_ros_rotation,
    GraspNetConfig,
    GraspNetInference,
)
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _default_model_path() -> str:
    """默认权重路径：install share 优先，源码树兜底."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = os.path.join(
            get_package_share_directory('graspnet_ros2'), 'models', 'checkpoint-rs.tar'
        )
        if os.path.exists(share):
            return share
    except Exception:
        pass
    # 源码树：<pkg>/graspnet_ros2/graspnet_node.py → <pkg>/models/
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg_root, 'models', 'checkpoint-rs.tar')


def _parse_workspace(text: str) -> Optional[tuple]:
    """把 'xmin,xmax,ymin,ymax,zmin,zmax' 解析为元组；空/非法返回 None."""
    text = (text or '').strip()
    if not text:
        return None
    parts = [float(v) for v in text.split(',')]
    if len(parts) != 6:
        raise ValueError(f'workspace 应为 6 个逗号分隔数值: {text!r}')
    return tuple(parts)


def _tf_to_matrix(tf_msg) -> np.ndarray:
    """geometry_msgs/Transform → 4x4 齐次矩阵."""
    q = tf_msg.rotation
    t = tf_msg.translation
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def _matrix_to_pose(mat: np.ndarray) -> Pose:
    """4x4 齐次矩阵 → geometry_msgs/Pose."""
    quat = Rotation.from_matrix(mat[:3, :3]).as_quat(canonical=False)
    pose = Pose()
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


class GraspNetDemoPointsNode(Node):
    """点云抓取检测节点：订阅 → 推理 → 发布."""

    def __init__(self):
        super().__init__('graspnet_demo_points_node')

        # ---------- 参数 ----------
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('input_pointcloud_topic', '/camera/depth_registered/points')
        self.declare_parameter('marker_topic', 'grasp_markers')
        self.declare_parameter('frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('compute_interval_sec', 1.0)
        self.declare_parameter('capture_groups_target', 3)
        self.declare_parameter('capture_control_service', '/graspnet_capture_control')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('grasp_poses_topic', 'grasp_poses_base')
        # 推理数值参数（透传 GraspNetConfig）
        self.declare_parameter('device', 'auto')
        self.declare_parameter('num_point', 20000)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('approach_dist', 0.05)
        self.declare_parameter('max_gripper_width', 0.1)
        self.declare_parameter('max_grasps_num', 5)
        self.declare_parameter('workspace', '')  # 'xmin,xmax,ymin,ymax,zmin,zmax'，空=不过滤

        model_path = self.get_parameter('model_path').value
        workspace = _parse_workspace(self.get_parameter('workspace').value)
        config = GraspNetConfig(
            checkpoint_path=model_path,
            device=self.get_parameter('device').value,
            num_point=int(self.get_parameter('num_point').value),
            collision_thresh=float(self.get_parameter('collision_thresh').value),
            voxel_size=float(self.get_parameter('voxel_size').value),
            approach_dist=float(self.get_parameter('approach_dist').value),
            max_gripper_width=float(self.get_parameter('max_gripper_width').value),
            max_grasps=int(self.get_parameter('max_grasps_num').value),
            workspace=workspace,
        )

        self.input_topic = self.get_parameter('input_pointcloud_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.default_frame_id = self.get_parameter('frame_id').value
        self.compute_interval_sec = float(self.get_parameter('compute_interval_sec').value)
        self.capture_groups_target = max(1, int(self.get_parameter('capture_groups_target').value))
        self.capture_control_service = self.get_parameter('capture_control_service').value
        self.base_frame = self.get_parameter('base_frame').value
        self.grasp_poses_topic = self.get_parameter('grasp_poses_topic').value

        # ---------- 推理核心 ----------
        self.get_logger().info(f'加载 GraspNet 权重: {model_path}')
        self.inference = GraspNetInference(config)
        self.get_logger().info(f'推理设备: {self.inference.device}')

        # ---------- ROS 通信 ----------
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.grasp_poses_pub = self.create_publisher(PoseArray, self.grasp_poses_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, self.input_topic, self._pc_callback, 10)
        self.create_timer(self.compute_interval_sec, self._timer_callback)
        self.create_service(SetBool, self.capture_control_service, self._capture_control_callback)

        # ---------- 会话状态 ----------
        self._latest_pc_msg: Optional[PointCloud2] = None
        self._latest_pc_stamp = self.get_clock().now().to_msg()
        self.processed_grasps: Optional[GraspList] = None
        self.processed_frame_id = ''
        self.collect_enabled = False
        self.target_groups = self.capture_groups_target
        self.collected_groups = 0
        self.session_id = 0

        self.get_logger().info(
            f'GraspNet 检测节点已启动：订阅 {self.input_topic}，'
            f'MarkerArray → {self.marker_topic}，'
            f'PoseArray → {self.grasp_poses_topic}（{self.base_frame}）；'
            f'服务 {self.capture_control_service}（True=开始，'
            f'目标 {self.capture_groups_target} 组）；workspace={workspace}'
        )

    # ---------- 采集控制 ----------
    def _capture_control_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.session_id += 1
            self.collect_enabled = True
            self.target_groups = max(1, self.capture_groups_target)
            self.collected_groups = 0
            self._latest_pc_msg = None
            self.processed_grasps = None
            response.success = True
            response.message = f'已开始采集: session={self.session_id}, 目标组数={self.target_groups}'
        else:
            self.collect_enabled = False
            response.success = True
            response.message = (
                f'已停止采集: session={self.session_id}, '
                f'collected={self.collected_groups}/{self.target_groups}'
            )
        self.get_logger().info(response.message)
        return response

    # ---------- 点云订阅 ----------
    def _pc_callback(self, msg: PointCloud2):
        self._latest_pc_msg = msg

    # ---------- 定时推理 ----------
    def _timer_callback(self):
        if not self.collect_enabled or self._latest_pc_msg is None:
            return
        try:
            self._compute_grasps(self._latest_pc_msg)
            if self.processed_grasps is not None and self._publish_results(self.processed_grasps):
                self.collected_groups += 1
                self.get_logger().info(
                    f'采集进度: session={self.session_id}, '
                    f'{self.collected_groups}/{self.target_groups}'
                )
                if self.collected_groups >= self.target_groups:
                    self.collect_enabled = False
                    self.get_logger().info(
                        f'达到目标组数，自动停止采集: session={self.session_id}'
                    )
        except Exception as e:  # noqa: BLE001 - 定时循环内兜底，避免节点退出
            self.get_logger().error(f'推理/发布失败: {e}')
            import traceback

            self.get_logger().error(traceback.format_exc())

    # ---------- 数据处理 ----------
    def _pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N,3) float32（skip_nans；空云由调用方兜底）."""
        pts = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if pts.dtype.fields is not None:
            pts = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
        return np.asarray(pts, dtype=np.float32).reshape(-1, 3)

    def _compute_grasps(self, pc_msg: PointCloud2):
        self._latest_pc_stamp = pc_msg.header.stamp
        points = self._pc2_to_xyz(pc_msg)
        if points.shape[0] == 0:
            raise RuntimeError('输入点云为空（或全部为 NaN）')

        self.processed_grasps = self.inference.get_grasp(points)
        self.processed_frame_id = (pc_msg.header.frame_id or '').strip() or self.default_frame_id
        self.get_logger().info(
            f'抓取计算完成: {len(self.processed_grasps)} 个（frame={self.processed_frame_id}）'
        )

    # ---------- 发布 ----------
    def _publish_results(self, grasps: GraspList) -> bool:
        if grasps is None or len(grasps) == 0:
            self.get_logger().warn('没有可发布的抓取')
            return False
        self._publish_marker_array(grasps)
        return self._publish_pose_array(grasps)

    def _publish_marker_array(self, grasps: GraspList) -> None:
        stamp = self._latest_pc_stamp
        frame_id = self.processed_frame_id

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = stamp
        self.marker_pub.publish(MarkerArray(markers=[delete_marker]))

        marker_array = MarkerArray()
        marker_id = 0
        for i in range(len(grasps)):
            grasp = grasps[i]
            s = float(np.clip(grasp.score, 0.0, 1.0))
            rgba = (1.0 - s, s, 0.0, 1.0)
            marker_array.markers.extend(
                self._create_grasp_markers(grasp, rgba, marker_id, stamp, frame_id)
            )
            marker_id += 4
            self._publish_grasp_tf(grasp, i, frame_id)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'已发布 MarkerArray: {len(grasps)} 个抓取')

    def _publish_pose_array(self, grasps: GraspList) -> bool:
        """base_frame 下的 PoseArray；查不到 TF 时发布空数组并返回 False."""
        stamp = self._latest_pc_stamp
        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame
        pose_array.header.stamp = stamp
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.processed_frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
            t_base_camera = _tf_to_matrix(tf_msg.transform)

            for i in range(len(grasps)):
                grasp = grasps[i]
                t_camera_grasp = np.eye(4)
                t_camera_grasp[:3, :3] = graspnet_to_ros_rotation(grasp.rotation_matrix)
                t_camera_grasp[:3, 3] = grasp.translation
                pose_array.poses.append(_matrix_to_pose(t_base_camera @ t_camera_grasp))

            self.grasp_poses_pub.publish(pose_array)
            self.get_logger().info(
                f'已发布 {len(pose_array.poses)} 个抓取位姿到 {self.grasp_poses_topic}'
            )
            return len(pose_array.poses) > 0
        except Exception as e:
            self.get_logger().warn(
                f'查询 {self.base_frame} <- {self.processed_frame_id} 失败，发布空 PoseArray: {e}'
            )
            self.grasp_poses_pub.publish(pose_array)
            return False

    def _publish_grasp_tf(self, grasp, idx: int, frame_id: str) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = f'grasp_pose_{idx}'
        t.transform.translation.x = float(grasp.translation[0])
        t.transform.translation.y = float(grasp.translation[1])
        t.transform.translation.z = float(grasp.translation[2])
        quat = Rotation.from_matrix(
            graspnet_to_ros_rotation(grasp.rotation_matrix)
        ).as_quat(canonical=False)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    # ---------- Marker 几何 ----------
    def _create_grasp_markers(self, grasp, rgba, id_start: int, stamp, frame_id: str):
        """单抓取 4 圆柱（左指/右指/手腕/手掌），几何与旧栈一致."""
        pose_mat = np.eye(4, dtype=np.float32)
        pose_mat[:3, 3] = grasp.translation.astype(np.float32)
        rot = grasp.rotation_matrix.astype(np.float32)
        pose_mat[:3, :3] = rot

        w = float(grasp.width)
        d = float(grasp.depth)
        radius = 0.005
        finger_width = 0.004
        depth_base = 0.02
        tail_length = 0.04
        finger_length = d + (depth_base + finger_width)
        finger_center_x = d / 2 - (depth_base + finger_width) / 2

        # 夹爪圆柱轴基：Marker Z=approach(col0) → (X,Y,Z)=(col1,col2,col0)；
        # 手掌轴沿 width(col1) → 右手系 (X,Y,Z)=(col2,col0,col1)
        axis_approach = np.column_stack([rot[:, 1], rot[:, 2], rot[:, 0]])
        axis_width = np.column_stack([rot[:, 2], rot[:, 0], rot[:, 1]])

        segments = [
            # (局部中心, 轴基, 长度)
            (
                np.array([finger_center_x, -(w / 2 + finger_width / 2), 0]),
                axis_approach, finger_length,
            ),
            (
                np.array([finger_center_x, w / 2 + finger_width / 2, 0]),
                axis_approach, finger_length,
            ),
            (
                np.array([-(tail_length / 2 + finger_width + depth_base), 0, 0]),
                axis_approach, tail_length,
            ),
            (
                np.array([-depth_base - finger_width / 2, 0, 0]),
                axis_width, w,
            ),
        ]

        markers = []
        for offset_id, (center_local, axis, length) in enumerate(segments):
            center = (pose_mat @ np.append(center_local, 1.0).astype(np.float32))[:3]
            seg = np.eye(4, dtype=np.float32)
            seg[:3, :3] = axis
            seg[:3, 3] = center
            markers.append(
                self._create_marker(
                    Marker.CYLINDER, seg, [radius, radius, length],
                    rgba, id_start + offset_id, stamp, frame_id,
                )
            )
        return markers

    def _create_marker(self, marker_type, pose_mat, scale, color, marker_id, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'grasp'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.pose = _matrix_to_pose(pose_mat)
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetDemoPointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
