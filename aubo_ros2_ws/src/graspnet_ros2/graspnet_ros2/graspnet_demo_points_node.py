#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo(点云版)：
  - 订阅 PointCloud2（默认：/camera/depth_registered/points）
  - 将点云转换为 GraspNet 的 end_points['point_clouds'] 输入
  - 推理 → 碰撞检测 → NMS/排序
  - 发布 MarkerArray / TF（复用 graspnet_demo_node.py 的发布逻辑风格）
  - 可选发布点云到 pointcloud_topic（供 RViz/Octomap 等使用）

说明：
  - 本文件参考 graspnet_demo_node.py，但将模型输入从“文件RGB-D反投影点云”改为“直接使用 PointCloud2 点云话题”
  - GraspNet 的最小输入仅需要 end_points['point_clouds']，形状 (1, N, 3)，dtype float32，单位米
"""

import os
import sys
import time
import multiprocessing
from typing import Optional, Tuple

import numpy as np
import torch
import open3d as o3d
from typing import cast

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def _get_graspnet_baseline_root():
    """解析 graspnet-baseline 根目录：源码为 ../graspnet-baseline，install 为 share/graspnet_ros2/graspnet-baseline。"""
    this_file = os.path.abspath(__file__)
    src_root = os.path.join(os.path.dirname(this_file), '..', 'graspnet-baseline')
    src_root = os.path.abspath(src_root)
    if os.path.isdir(src_root) and os.path.isdir(os.path.join(src_root, 'models')):
        return src_root
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('graspnet_ros2')
        install_root = os.path.join(share, 'graspnet-baseline')
        if os.path.isdir(install_root) and os.path.isdir(os.path.join(install_root, 'models')):
            return install_root
    except Exception:
        pass
    return src_root


ROOT_DIR = _get_graspnet_baseline_root()
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'models'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from models.graspnet import GraspNet, pred_decode  # noqa: E402
from utils.collision_detector import ModelFreeCollisionDetector  # noqa: E402
from graspnetAPI import GraspGroup  # noqa: E402


def _vis_grasps_open3d_process(gg: GraspGroup, cloud: o3d.geometry.PointCloud):
    """在独立进程中运行 Open3D 可视化（与 graspnet_demo_node.py 一致的思路）"""
    try:
        gg = gg.nms()
        gg.sort_by_score()
        gg_vis = gg[: min(10, len(gg))]
        grippers = gg_vis.to_open3d_geometry_list()  # type: ignore[attr-defined]
        geometries = [cloud] + grippers
        o3d.visualization.draw_geometries(  # type: ignore[attr-defined]
            geometries,
            window_name='GraspNet visualization',
            width=1920,
            height=1080,
            point_show_normal=False,
        )
    except Exception as e:
        print(f'Open3D 可视化失败: {str(e)}')
        import traceback

        traceback.print_exc()


class GraspNetDemoPointsNode(Node):
    """订阅点云 → GraspNet 推理 → 发布 MarkerArray/TF"""

    def __init__(self):
        super().__init__('graspnet_demo_points_node')

        # ========== 参数 ==========
        self.declare_parameter('baseline_dir', ROOT_DIR)
        self.declare_parameter('model_path', '')

        # 输入点云（新需求）
        self.declare_parameter('input_pointcloud_topic', '/camera/depth_registered/points')

        # 推理参数（与 graspnet_demo_node.py 保持一致默认值）
        self.declare_parameter('num_point', 20000)
        self.declare_parameter('num_view', 300)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('max_grasps_num', 20)
        self.declare_parameter('gpu', 0)

        # 发布参数
        self.declare_parameter('marker_topic', 'grasp_markers')
        self.declare_parameter('pointcloud_topic', 'graspnet_pointcloud')  # 可选：发布点云（从输入点云转换/复用）
        self.declare_parameter('frame_id', 'camera_frame')  # 若输入点云 header.frame_id 为空则用它

        # 可视化/运行模式
        self.declare_parameter('use_open3d', False)
        self.declare_parameter('auto_run', False)

        # ========== 获取参数 ==========
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.input_pointcloud_topic = self.get_parameter('input_pointcloud_topic').get_parameter_value().string_value
        self.num_point = self.get_parameter('num_point').get_parameter_value().integer_value
        self.num_view = self.get_parameter('num_view').get_parameter_value().integer_value
        self.collision_thresh = self.get_parameter('collision_thresh').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.max_grasps_num = self.get_parameter('max_grasps_num').get_parameter_value().integer_value
        self.gpu = self.get_parameter('gpu').get_parameter_value().integer_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.default_frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.use_open3d = bool(self.get_parameter('use_open3d').get_parameter_value().bool_value)
        self.auto_run = bool(self.get_parameter('auto_run').get_parameter_value().bool_value)

        # ========== 设备/模型 ==========
        self.device = torch.device(f"cuda:{self.gpu}" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'使用设备: {self.device}')
        self.net = self._load_model()

        # ========== ROS 通信 ==========
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅输入点云
        self._latest_pc_msg: Optional[PointCloud2] = None
        self.create_subscription(PointCloud2, self.input_pointcloud_topic, self._pc_callback, 10)

        # 服务：手动触发一次推理（命名沿用 graspnet_demo_node.py）
        self.publish_srv = self.create_service(Trigger, 'publish_grasps', self.publish_service_callback)

        # 缓存结果（与 graspnet_demo_node.py 风格一致）
        self.processed_gg: Optional[GraspGroup] = None
        self.cloud_o3d: Optional[o3d.geometry.PointCloud] = None
        self.has_computed = False

        self.get_logger().info('GraspNet 点云版节点已启动')
        self.get_logger().info(f'订阅输入点云: {self.input_pointcloud_topic}')
        self.get_logger().info(f'发布 MarkerArray: {self.marker_topic}')
        self.get_logger().info(f'发布点云(可选): {self.pointcloud_topic}')

        self._run_count = 0
        self._auto_timer = None
        if self.auto_run:
            # 轮询等待点云到达；成功计算一次后取消
            self._auto_timer = self.create_timer(1.0, self._auto_run_once)
            self.get_logger().info('自动模式：收到点云后将自动计算并发布一次抓取')
        else:
            self.get_logger().info('手动模式：调用 /publish_grasps 服务触发推理与发布')

    def _pc_callback(self, msg: PointCloud2):
        self._latest_pc_msg = msg

    def _load_model(self):
        self.get_logger().info(f'正在加载模型: {self.model_path}')
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f'模型文件不存在: {self.model_path}')

        net = GraspNet(
            input_feature_dim=0,
            num_view=self.num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        net.to(self.device)
        checkpoint = torch.load(self.model_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])
        net.eval()
        self.get_logger().info('模型加载完成')
        return net

    def _auto_run_once(self):
        if self._run_count > 0:
            return
        if self._latest_pc_msg is None:
            self.get_logger().warn('自动模式：尚未收到点云，等待中...')
            return
        try:
            self._compute_grasps(self._latest_pc_msg)
            self.publish_marker_array(self.processed_gg)  # type: ignore[arg-type]
            self.publish_pointcloud(self.cloud_o3d, self.processed_frame_id)  # type: ignore[arg-type]
            self._run_count += 1
            if self._auto_timer is not None:
                self._auto_timer.cancel()
            self.get_logger().info('自动模式：预测和发布完成')
        except Exception as e:
            self.get_logger().error(f'自动模式预测失败: {str(e)}')
            import traceback

            self.get_logger().error(traceback.format_exc())

    def publish_service_callback(self, request, response):
        if self._latest_pc_msg is None:
            response.success = False
            response.message = f'尚未收到输入点云: {self.input_pointcloud_topic}'
            return response
        try:
            self._compute_grasps(self._latest_pc_msg)
            self.publish_marker_array(self.processed_gg)  # type: ignore[arg-type]
            self.publish_pointcloud(self.cloud_o3d, self.processed_frame_id)  # type: ignore[arg-type]
            response.success = True
            response.message = f'已发布 {len(self.processed_gg)} 个抓取'  # type: ignore[arg-type]
            return response
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'服务触发预测失败: {str(e)}')
            import traceback

            self.get_logger().error(traceback.format_exc())
            return response

    # ========= 数据处理：PointCloud2 -> end_points =========
    def _pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """PointCloud2 转 (N,3) float32。优先使用 read_points_numpy，失败则回退迭代方式。"""
        try:
            pts = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            # read_points_numpy 可能返回 structured 或 (N,3)；统一转 (N,3)
            if pts.dtype.fields is not None:
                pts = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
            pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            return pts
        except Exception:
            pts_list = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
            if len(pts_list) == 0:
                return np.zeros((0, 3), dtype=np.float32)
            return np.asarray(pts_list, dtype=np.float32).reshape(-1, 3)

    def _get_and_process_data(self, pc_msg: PointCloud2) -> Tuple[dict, o3d.geometry.PointCloud, str]:
        points = self._pc2_to_xyz(pc_msg)
        if points.shape[0] == 0:
            raise RuntimeError('输入点云为空（或全部为 NaN）')

        # 采样到固定 num_point（与 demo.py / graspnet_demo_node.py 一致：不足则重复补齐）
        np.random.seed(1)
        if len(points) >= self.num_point:
            idxs = np.random.choice(len(points), self.num_point, replace=False)
        else:
            idxs1 = np.arange(len(points))
            idxs2 = np.random.choice(len(points), self.num_point - len(points), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)

        cloud_sampled = points[idxs].astype(np.float32)
        cloud_sampled_torch = torch.from_numpy(cloud_sampled[np.newaxis]).to(self.device)
        end_points = {'point_clouds': cloud_sampled_torch}

        # Open3D 点云（用于碰撞检测/可视化/发布）
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(points.astype(np.float32))

        frame_id = (pc_msg.header.frame_id or '').strip() or self.default_frame_id
        return end_points, cloud_o3d, frame_id

    # ========= 推理与后处理 =========
    def _get_grasps(self, end_points) -> GraspGroup:
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(gg_array)

    def _collision_detection(self, gg: GraspGroup, cloud_xyz: np.ndarray) -> GraspGroup:
        if self.collision_thresh <= 0:
            return gg
        mfcdetector = ModelFreeCollisionDetector(cloud_xyz, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        collision_mask = np.asarray(collision_mask, dtype=bool)
        gg = cast(GraspGroup, gg[~collision_mask])
        self.get_logger().info(f'碰撞检测后剩余 {len(gg)} 个抓取')
        return gg

    def _compute_grasps(self, pc_msg: PointCloud2):
        end_points, cloud, frame_id = self._get_and_process_data(pc_msg)
        gg = self._get_grasps(end_points)
        gg = self._collision_detection(gg, np.asarray(cloud.points))
        processed_gg = cast(GraspGroup, gg.nms())
        processed_gg.sort_by_score()
        processed_gg = processed_gg[: self.max_grasps_num]

        self.processed_gg = cast(GraspGroup, processed_gg)
        self.cloud_o3d = cloud
        self.processed_frame_id = frame_id
        self.has_computed = True

        self.get_logger().info(f'抓取计算完成，共 {len(cast(GraspGroup, processed_gg))} 个抓取')

        if self.use_open3d:
            vis_process = multiprocessing.Process(
                target=_vis_grasps_open3d_process,
                args=(processed_gg, cloud),
                daemon=True,
            )
            vis_process.start()

    # ========= 发布：MarkerArray / TF / PointCloud2 =========
    def publish_marker_array(self, gg: GraspGroup):
        if gg is None or len(gg) == 0:
            self.get_logger().warn('没有可发布的抓取')
            return

        # 先清空
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.processed_frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(MarkerArray(markers=[delete_marker]))

        # 简单颜色映射（与 demo 节点风格类似）
        def color_map(score):
            s = float(np.clip(score, 0.0, 1.0))
            return (1.0 - s, s, 0.0, 1.0)

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        # 逐个抓取发布 4 个圆柱 marker（直接复用 graspnet_demo_node.py 的“夹爪=4段圆柱”思路）
        for i in range(len(gg)):
            grasp = gg[i]
            rgba = color_map(getattr(grasp, 'score', 0.5))  # type: ignore[attr-defined]
            markers = self._create_grasp_markers(grasp, rgba, marker_id, stamp, self.processed_frame_id)
            marker_array.markers.extend(markers)
            marker_id += 4

            # 同时发布 TF：camera_frame -> grasp_pose_i
            self._publish_grasp_tf(grasp, i, stamp, self.processed_frame_id)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'已发布 MarkerArray: {len(gg)} 个抓取')

    def _publish_grasp_tf(self, grasp, idx: int, stamp, frame_id: str):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = f'grasp_pose_{idx}'
        t.transform.translation.x = float(grasp.translation[0])
        t.transform.translation.y = float(grasp.translation[1])
        t.transform.translation.z = float(grasp.translation[2])

        # grasp.rotation_matrix -> quaternion (x,y,z,w)
        from scipy.spatial.transform import Rotation

        quat = Rotation.from_matrix(grasp.rotation_matrix).as_quat(canonical=False)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _create_grasp_markers(self, grasp, rgba, id_start: int, stamp, frame_id: str):
        """
        用 4 个圆柱体表示夹爪：
          - 左指、右指、手腕、手掌
        坐标系与 graspnet_demo_node.py 的 RViz 绘制一致（以 grasp 的 rotation/translation 为基础）。
        """
        from scipy.spatial.transform import Rotation

        pose_mat = np.eye(4, dtype=np.float32)
        pose_mat[:3, 3] = grasp.translation.astype(np.float32)
        pose_mat[:3, :3] = grasp.rotation_matrix.astype(np.float32)

        w = float(grasp.width)
        d = float(grasp.depth)
        radius = 0.005

        def mk_cyl(id_, center_xyz, rot_mat, scale_xyz):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = 'grasp'
            m.id = int(id_)
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            quat = Rotation.from_matrix(rot_mat).as_quat(canonical=False)
            m.pose.position.x = float(center_xyz[0])
            m.pose.position.y = float(center_xyz[1])
            m.pose.position.z = float(center_xyz[2])
            m.pose.orientation.x = float(quat[0])
            m.pose.orientation.y = float(quat[1])
            m.pose.orientation.z = float(quat[2])
            m.pose.orientation.w = float(quat[3])

            m.scale.x = float(scale_xyz[0])
            m.scale.y = float(scale_xyz[1])
            m.scale.z = float(scale_xyz[2])
            m.color.r = float(rgba[0])
            m.color.g = float(rgba[1])
            m.color.b = float(rgba[2])
            m.color.a = float(rgba[3])
            return m

        # grasp 坐标系下的局部点（与 graspnet_demo_node.py 类似）
        left_local = np.array([-w / 2, 0.0, -d / 2, 1.0], dtype=np.float32)
        right_local = np.array([w / 2, 0.0, -d / 2, 1.0], dtype=np.float32)
        wrist_local = np.array([0.0, 0.0, -d * 5 / 4, 1.0], dtype=np.float32)
        palm_local = np.array([0.0, 0.0, -d, 1.0], dtype=np.float32)

        left_world = (pose_mat @ left_local)[:3]
        right_world = (pose_mat @ right_local)[:3]
        wrist_world = (pose_mat @ wrist_local)[:3]
        palm_world = (pose_mat @ palm_local)[:3]

        # 圆柱默认轴沿 z；这里简单用 grasp 的旋转即可（与 demo 的“简化绘制”一致）
        rot_world = pose_mat[:3, :3]

        # 手掌需要绕 y 轴旋转 90°（让圆柱横向连接两指）
        rot_y_90 = Rotation.from_rotvec([0.0, np.pi / 2, 0.0]).as_matrix().astype(np.float32)
        palm_rot_world = rot_world @ rot_y_90

        markers = [
            mk_cyl(id_start, left_world, rot_world, (radius, radius, d)),
            mk_cyl(id_start + 1, right_world, rot_world, (radius, radius, d)),
            mk_cyl(id_start + 2, wrist_world, rot_world, (radius, radius, d / 2)),
            mk_cyl(id_start + 3, palm_world, palm_rot_world, (radius, radius, w)),
        ]
        return markers

    def publish_pointcloud(self, cloud: o3d.geometry.PointCloud, frame_id: str):
        if cloud is None:
            return
        msg = self._o3d_to_ros2_pointcloud(cloud, frame_id)
        self.pointcloud_pub.publish(msg)

    def _o3d_to_ros2_pointcloud(self, o3d_cloud: o3d.geometry.PointCloud, frame_id: str) -> PointCloud2:
        points = np.asarray(o3d_cloud.points, dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = np.zeros(len(points), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = cloud_data.dtype.itemsize
        msg.row_step = cloud_data.dtype.itemsize * len(points)
        msg.is_dense = True
        msg.data = cloud_data.tobytes()
        return msg


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

