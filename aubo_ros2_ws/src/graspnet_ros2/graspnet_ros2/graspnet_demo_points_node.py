#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 Demo(点云版)：
  - 订阅 PointCloud2（默认：/camera/depth_registered/points）
  - 将点云转换为 GraspNet 的 end_points['point_clouds'] 输入
  - 推理 → 碰撞检测 → NMS/排序
  - 发布 MarkerArray / TF（复用 graspnet_demo_node.py 的发布逻辑风格）

说明：
  - 本文件参考 graspnet_demo_node.py，但将模型输入从“文件RGB-D反投影点云”改为“直接使用 PointCloud2 点云话题”
  - GraspNet 的最小输入仅需要 end_points['point_clouds']，形状 (1, N, 3)，dtype float32，单位米
"""

# ---------- 标准库 ----------
import os
import sys
import multiprocessing
from typing import Optional, Tuple, cast

# ---------- 第三方 ----------
import numpy as np
import torch
import open3d as o3d
from scipy.spatial.transform import Rotation

# ---------- ROS2 ----------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, TransformStamped
from builtin_interfaces.msg import Duration, Time
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


# ========== graspnet-baseline 路径与依赖（须先设置 sys.path 再导入） ==========
def _get_graspnet_baseline_root() -> str:
    """解析 graspnet-baseline 根目录：源码为 ../graspnet-baseline，install 为 share/graspnet_ros2/graspnet-baseline。"""
    this_file = os.path.abspath(__file__)
    src_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '..', 'graspnet-baseline'))
    if os.path.isdir(src_root) and os.path.isdir(os.path.join(src_root, 'models')):
        return src_root
    try:
        from ament_index_python.packages import get_package_share_directory

        install_root = os.path.join(get_package_share_directory('graspnet_ros2'), 'graspnet-baseline')
        if os.path.isdir(install_root) and os.path.isdir(os.path.join(install_root, 'models')):
            return install_root
    except Exception:
        pass
    return src_root


def _setup_graspnet_baseline():
    """设置 sys.path 并导入 graspnet-baseline 依赖，返回 (ROOT_DIR, GraspNet, pred_decode, ModelFreeCollisionDetector, GraspGroup)。"""
    root = _get_graspnet_baseline_root()
    sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, 'models'))
    sys.path.insert(0, os.path.join(root, 'dataset'))
    sys.path.append(os.path.join(root, 'utils'))

    from models.graspnet import GraspNet, pred_decode  # noqa: E402
    from utils.collision_detector import ModelFreeCollisionDetector  # noqa: E402
    from graspnetAPI import GraspGroup  # noqa: E402

    return root, GraspNet, pred_decode, ModelFreeCollisionDetector, GraspGroup


ROOT_DIR, GraspNet, pred_decode, ModelFreeCollisionDetector, GraspGroup = _setup_graspnet_baseline()


# ========== 模块级：Open3D 可视化（独立进程） ==========
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


# ========== GraspNet 点云版节点 ==========
class GraspNetDemoPointsNode(Node):
    """
    订阅点云 → GraspNet 推理 → 发布 MarkerArray/TF

    类内方法顺序：初始化 → 输入(订阅) → 模型加载 → 服务入口 → 数据处理 → 推理与流水线 → 发布与辅助
    """

    def __init__(self):
        super().__init__('graspnet_demo_points_node')

        # ========== 参数 ==========
        self.declare_parameter('baseline_dir', ROOT_DIR)
        self.declare_parameter('model_path', '')

        # 输入点云（新需求）
        self.declare_parameter('input_pointcloud_topic', '/camera/depth_registered/points')

        # 推理参数使用 GraspNet 默认值，不暴露为 launch 参数
        self.num_point = 20000
        self.num_view = 300
        self.collision_thresh = 0.01
        self.voxel_size = 0.01
        self.max_grasps_num = 5
        self.gpu = 0

        # 发布参数
        self.declare_parameter('marker_topic', 'grasp_markers')
        self.declare_parameter('frame_id', 'camera_frame')  # 若输入点云 header.frame_id 为空则用它

        # 可视化
        self.declare_parameter('use_open3d', False)

        # ========== 获取参数 ==========
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.input_pointcloud_topic = self.get_parameter('input_pointcloud_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.default_frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.use_open3d = bool(self.get_parameter('use_open3d').get_parameter_value().bool_value)

        # ========== 设备/模型 ==========
        self.device = torch.device(f"cuda:{self.gpu}" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'使用设备: {self.device}')
        self.net = self._load_model()

        # ========== ROS 通信 ==========
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, self.input_pointcloud_topic, self._pc_callback, 10)
        self.publish_srv = self.create_service(Trigger, 'publish_grasps', self.publish_service_callback)

        # ========== 状态缓存（服务回调与发布使用） ==========
        self._latest_pc_msg: Optional[PointCloud2] = None
        self.processed_gg: Optional[GraspGroup] = None
        self.cloud_o3d: Optional[o3d.geometry.PointCloud] = None
        self.processed_frame_id: str = ''
        self.has_computed = False

        self._log_startup()

    # ========== 初始化辅助 ==========
    def _log_startup(self):
        """打印节点启动信息（话题、服务）。"""
        self.get_logger().info('GraspNet 点云版节点已启动')
        self.get_logger().info(f'订阅输入点云: {self.input_pointcloud_topic}')
        self.get_logger().info(f'发布 MarkerArray: {self.marker_topic}')
        self.get_logger().info('调用 /publish_grasps 服务触发推理与发布')

    # ========== 输入：点云订阅 ==========
    def _pc_callback(self, msg: PointCloud2):
        self._latest_pc_msg = msg

    # ========== 模型加载 ==========
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

    # ========== 服务入口：触发推理与发布 ==========
    def publish_service_callback(self, request, response):
        if self._latest_pc_msg is None:
            response.success = False
            response.message = f'尚未收到输入点云: {self.input_pointcloud_topic}'
            return response
        try:
            self._compute_grasps(self._latest_pc_msg)
            self.publish_marker_array(self.processed_gg)  # type: ignore[arg-type]
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

    # ========== 数据处理：PointCloud2 -> end_points ==========
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
        """PointCloud2 → 采样/转张量 → end_points、Open3D 点云、frame_id（流水线第一步）。"""
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

    # ========== 推理与后处理 ==========
    def _get_grasps(self, end_points) -> GraspGroup:
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(gg_array)

    # ========== 碰撞检测 ==========
    def _collision_detection(self, gg: GraspGroup, cloud_xyz: np.ndarray) -> GraspGroup:
        if self.collision_thresh <= 0:
            return gg
        mfcdetector = ModelFreeCollisionDetector(cloud_xyz, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        collision_mask = np.asarray(collision_mask, dtype=bool)
        gg = cast(GraspGroup, gg[~collision_mask])
        self.get_logger().info(f'碰撞检测后剩余 {len(gg)} 个抓取')
        return gg

    # ========== 流水线入口：点云 → 抓取结果 ==========
    def _compute_grasps(self, pc_msg: PointCloud2):
        """数据准备 → 网络推理 → 碰撞检测 → NMS/排序 → 写缓存，可选 Open3D 可视化。"""
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

    # ========== 发布：MarkerArray / TF ==========
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

    # ---------- TF 与 Marker 辅助 ----------
    def _publish_grasp_tf(self, grasp, idx: int, stamp, frame_id: str):
        """发布 GraspNet 抓取位姿对应的 TF（对齐 graspnet_demo_node.py 的坐标系转换）。"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = f'grasp_pose_{idx}'

        # 平移直接使用 GraspNet 输出（与 demo 节点一致，均在相机系下）
        t.transform.translation.x = float(grasp.translation[0])
        t.transform.translation.y = float(grasp.translation[1])
        t.transform.translation.z = float(grasp.translation[2])

        # 坐标系转换：GraspNet -> ROS 末端执行器标准坐标系
        # GraspNet rotation_matrix 列为:
        #   col0 = approach (手指伸出方向)
        #   col1 = width   (左右张开方向)
        #   col2 = height  (上下方向)
        # ROS 末端坐标约定:
        #   Z 轴 = approach
        #   X 轴 = width
        #   Y 轴 = height
        R_graspnet = grasp.rotation_matrix
        R_ros = np.column_stack(
            [
                R_graspnet[:, 1],  # ROS X = GraspNet width
                R_graspnet[:, 2],  # ROS Y = GraspNet height
                R_graspnet[:, 0],  # ROS Z = GraspNet approach
            ]
        )

        # ROS 四元数 (x, y, z, w)
        quat = Rotation.from_matrix(R_ros).as_quat(canonical=False)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _create_grasp_markers(self, grasp, rgba, id_start: int, stamp, frame_id: str):
        """
        为单个抓取创建可视化标记（4 个圆柱：左指、右指、手腕、手掌），
        几何和坐标系与 graspnet_demo_node.py 的 create_grasp_markers 完全对齐。
        """
        # ===== 构建抓取位姿矩阵（GraspNet 坐标系）=====
        pose_mat = np.eye(4, dtype=np.float32)
        pose_mat[:3, 3] = grasp.translation.astype(np.float32)
        pose_mat[:3, :3] = grasp.rotation_matrix.astype(np.float32)
        R = pose_mat[:3, :3]

        w = float(grasp.width)
        d = float(grasp.depth)
        radius = 0.005
        markers: list[Marker] = []

        # 与 plot_gripper_pro_max 完全一致的几何常数
        finger_width = 0.004  # 手指/手掌厚度
        depth_base = 0.02  # 夹爪基座深度
        tail_length = 0.04  # 手腕(tail)长度
        finger_length = d + (depth_base + finger_width)
        finger_center_x = d / 2 - (depth_base + finger_width) / 2

        # ===== 左指 =====
        left_point = pose_mat @ np.array(
            [finger_center_x, -w / 2 - finger_width / 2, 0, 1], dtype=np.float32
        )
        left_pose = np.eye(4, dtype=np.float32)
        # Marker Z = approach(col0)，(X,Y,Z) = (width,height,approach) = (col1,col2,col0)
        left_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
        left_pose[:3, 3] = left_point[:3]
        markers.append(
            self._create_marker(
                Marker.CYLINDER,
                left_pose,
                [radius, radius, finger_length],
                rgba,
                id_start,
                stamp,
                frame_id,
            )
        )

        # ===== 右指 =====
        right_point = pose_mat @ np.array(
            [finger_center_x, w / 2 + finger_width / 2, 0, 1], dtype=np.float32
        )
        right_pose = np.eye(4, dtype=np.float32)
        right_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
        right_pose[:3, 3] = right_point[:3]
        markers.append(
            self._create_marker(
                Marker.CYLINDER,
                right_pose,
                [radius, radius, finger_length],
                rgba,
                id_start + 1,
                stamp,
                frame_id,
            )
        )

        # ===== 手腕(tail) =====
        wrist_center_x = -(tail_length / 2 + finger_width + depth_base)
        wrist_point = pose_mat @ np.array([wrist_center_x, 0, 0, 1], dtype=np.float32)
        wrist_pose = np.eye(4, dtype=np.float32)
        wrist_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
        wrist_pose[:3, 3] = wrist_point[:3]
        markers.append(
            self._create_marker(
                Marker.CYLINDER,
                wrist_pose,
                [radius, radius, tail_length],
                rgba,
                id_start + 2,
                stamp,
                frame_id,
            )
        )

        # ===== 手掌(bottom): 轴沿 width(col1)，长度 w =====
        palm_center_x = -depth_base - finger_width / 2
        palm_point = pose_mat @ np.array([palm_center_x, 0, 0, 1], dtype=np.float32)
        palm_pose = np.eye(4, dtype=np.float32)
        # 右手系: Marker Z=col1, X=col2, Y=col0，保证 X×Y=Z
        palm_pose[:3, :3] = np.column_stack([R[:, 2], R[:, 0], R[:, 1]])
        palm_pose[:3, 3] = palm_point[:3]
        markers.append(
            self._create_marker(
                Marker.CYLINDER,
                palm_pose,
                [radius, radius, w],
                rgba,
                id_start + 3,
                stamp,
                frame_id,
            )
        )

        return markers

    def _create_marker(
        self,
        marker_type: int,
        pose_mat: np.ndarray,
        scale,
        color,
        marker_id: int,
        stamp,
        frame_id: str,
    ) -> Marker:
        """创建单个 RViz Marker（与 graspnet_demo_node.py 对齐）。"""
        # 处理尺寸
        if np.isscalar(scale):
            scale = [float(scale), float(scale), float(scale)]
        else:
            scale = [float(s) for s in scale]

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp

        marker.ns = 'grasp'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.lifetime = Duration(sec=0, nanosec=0)

        marker.pose = self._matrix_to_pose(pose_mat)

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        return marker

    def _matrix_to_pose(self, pose_mat: np.ndarray) -> Pose:
        """将 4x4 齐次变换矩阵转换为 geometry_msgs/Pose（与 graspnet_demo_node.py 对齐）。"""
        pose = Pose()

        quat = Rotation.from_matrix(pose_mat[:3, :3]).as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        pose.position.x = float(pose_mat[0, 3])
        pose.position.y = float(pose_mat[1, 3])
        pose.position.z = float(pose_mat[2, 3])

        return pose


# ========== 节点入口 ==========
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

