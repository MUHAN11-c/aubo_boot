"""recon_fusion_node — Open3D 场景融合（默认点云；可选 TSDF RGB-D）。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from aubo_scene_recon.backends import create_backend
from aubo_scene_recon.backends.tsdf_volume import TsdfBackend
from aubo_scene_recon.pc_utils import (
    cloud_to_arrays,
    make_pointcloud2,
    transform_msg_to_matrix,
)


class ReconFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('recon_fusion_node')

        self.declare_parameter('backend', 'open3d')
        self.declare_parameter('map_frame', 'base_link')
        self.declare_parameter('pointcloud_topic', '/camera/depth_registered/points')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('voxel_size', 0.005)
        self.declare_parameter('sdf_trunc', 0.04)
        self.declare_parameter('depth_scale', 4000.0)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 1.5)
        self.declare_parameter('tf_timeout_sec', 0.1)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('max_map_points', 2_000_000)
        self.declare_parameter('outlier_every_n', 5)
        self.declare_parameter('save_dir', '')
        self.declare_parameter('no_cloud_warn_sec', 5.0)

        self.backend_name = str(self.get_parameter('backend').value).strip().lower()
        self.map_frame = self.get_parameter('map_frame').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))
        save_dir = self.get_parameter('save_dir').value
        if not save_dir:
            save_dir = str(Path.cwd() / 'recon_maps')
        self.save_dir = Path(save_dir)
        self.no_cloud_warn_sec = float(self.get_parameter('no_cloud_warn_sec').value)

        voxel_size = float(self.get_parameter('voxel_size').value)
        max_pts = int(self.get_parameter('max_map_points').value)
        self.backend = create_backend(
            self.backend_name,
            voxel_size,
            max_pts,
            outlier_every_n=int(self.get_parameter('outlier_every_n').value),
            sdf_trunc=float(self.get_parameter('sdf_trunc').value),
            depth_scale=float(self.get_parameter('depth_scale').value),
            depth_max=self.max_range,
        )
        self._bridge = CvBridge()
        self._use_tsdf = isinstance(self.backend, TsdfBackend)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frames_ok = 0
        self.frames_dropped = 0
        self._last_cloud_time = self.get_clock().now()
        self._logged_frame = False
        self._last_no_cloud_warn = None
        self._tf_fallback_logged = False

        self.map_pub = self.create_publisher(PointCloud2, '/recon/map_cloud', 1)
        self.status_pub = self.create_publisher(String, '/recon/status', 1)
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_service(Trigger, '~/save', self._on_save)

        if self._use_tsdf:
            color_topic = self.get_parameter('color_topic').value
            depth_topic = self.get_parameter('depth_topic').value
            info_topic = self.get_parameter('color_info_topic').value
            self._color_sub = Subscriber(self, Image, color_topic)
            self._depth_sub = Subscriber(self, Image, depth_topic)
            self._info_sub = Subscriber(self, CameraInfo, info_topic)
            self._sync = ApproximateTimeSynchronizer(
                [self._color_sub, self._depth_sub, self._info_sub],
                queue_size=5,
                slop=0.1,
            )
            self._sync.registerCallback(self._on_rgbd)
            self.get_logger().info(
                f'TSDF 模式: {color_topic} + {depth_topic} + {info_topic}')
        else:
            topic = self.get_parameter('pointcloud_topic').value
            self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
            self.get_logger().info(f'Open3D 点云模式: {topic}')

        pub_period = float(self.get_parameter('publish_period_sec').value)
        status_period = float(self.get_parameter('status_period_sec').value)
        self.create_timer(pub_period, self._publish_map)
        self.create_timer(status_period, self._publish_status)

        self.get_logger().info(
            f'recon_fusion_node ready: backend={self.backend_name} '
            f'map_frame={self.map_frame} save_dir={self.save_dir}')

    def _lookup_T(self, frame_id: str, stamp) -> np.ndarray | None:
        """查 map←camera。

        相机用设备 HW 时间戳，常比机器人 TF（系统时间）略超前，严格按 stamp
        会报 extrapolation into the future 并丢帧，重建就会缺块/错位。
        策略：先按 stamp；失败则回退到最新 TF（慢扫时误差可接受）。
        """
        stamp_time = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                stamp_time,
                timeout=self.tf_timeout,
            )
            return transform_msg_to_matrix(tf.transform)
        except TransformException as ex_stamp:
            err = str(ex_stamp)
            # 时钟不同步 / TF 稍旧：用最新可用位姿
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    Time(),  # latest
                    timeout=self.tf_timeout,
                )
                if not getattr(self, '_tf_fallback_logged', False):
                    self.get_logger().warning(
                        '图像时间戳与机器人 TF 不同步（常见：相机 HW stamp 超前）。'
                        '已回退用最新 TF；请慢速扫。首次原因: '
                        f'{err}')
                    self._tf_fallback_logged = True
                return transform_msg_to_matrix(tf.transform)
            except TransformException as ex_latest:
                self.frames_dropped += 1
                self.get_logger().warning(
                    f'TF 失败，丢帧: {self.map_frame}←{frame_id}: '
                    f'stamp失败[{err}]; latest失败[{ex_latest}]')
                return None

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._last_cloud_time = self.get_clock().now()
        if not self._logged_frame:
            self.get_logger().info(f'首帧 cloud frame_id={msg.header.frame_id!r}')
            self._logged_frame = True

        T = self._lookup_T(msg.header.frame_id, msg.header.stamp)
        if T is None:
            return
        xyz, colors = cloud_to_arrays(msg)
        if xyz.size == 0:
            return
        self.backend.integrate(xyz, colors, T, self.min_range, self.max_range)
        self.frames_ok += 1

    def _on_rgbd(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        self._last_cloud_time = self.get_clock().now()
        if not self._logged_frame:
            self.get_logger().info(
                f'首帧 RGB-D frame_id={color_msg.header.frame_id!r}')
            self._logged_frame = True

        T = self._lookup_T(color_msg.header.frame_id, color_msg.header.stamp)
        if T is None:
            return

        try:
            color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as ex:  # noqa: BLE001 — cv_bridge 异常类型多
            self.get_logger().warning(f'图像转换失败: {ex}')
            self.frames_dropped += 1
            return

        if depth.dtype != np.uint16:
            self.get_logger().warning(
                f'深度 dtype={depth.dtype}，TSDF 需要 16UC1(mm)，丢帧')
            self.frames_dropped += 1
            return

        # 配准深度时分辨率应与彩色一致；不一致则跳过
        if depth.shape[:2] != color.shape[:2]:
            self.get_logger().warning(
                f'彩色/深度尺寸不一致 {color.shape[:2]} vs {depth.shape[:2]}，丢帧')
            self.frames_dropped += 1
            return

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=int(info_msg.width),
            height=int(info_msg.height),
            fx=float(info_msg.k[0]),
            fy=float(info_msg.k[4]),
            cx=float(info_msg.k[2]),
            cy=float(info_msg.k[5]),
        )
        assert isinstance(self.backend, TsdfBackend)
        self.backend.integrate_rgbd(color, depth, intrinsic, T, color_is_bgr=True)
        self.frames_ok += 1

    def _publish_map(self) -> None:
        xyz, colors = self.backend.get_map()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        self.map_pub.publish(make_pointcloud2(header, xyz, colors))

    def _publish_status(self) -> None:
        idle = (self.get_clock().now() - self._last_cloud_time).nanoseconds * 1e-9
        if idle > self.no_cloud_warn_sec and self.frames_ok == 0:
            now = self.get_clock().now()
            if (self._last_no_cloud_warn is None or
                    (now - self._last_no_cloud_warn).nanoseconds * 1e-9 >= 5.0):
                tip = ('等待 RGB-D' if self._use_tsdf else '等待点云')
                self.get_logger().warning(f'{tip}（确认 percipio_rgbd 已开）')
                self._last_no_cloud_warn = now
        msg = String()
        msg.data = (
            f'frames_ok={self.frames_ok} frames_dropped={self.frames_dropped} '
            f'map_points={self.backend.num_points()} backend={self.backend_name}')
        self.status_pub.publish(msg)

    def _on_reset(self, _req, resp):
        self.backend.reset()
        self.frames_ok = 0
        self.frames_dropped = 0
        resp.success = True
        resp.message = 'map cleared'
        self.get_logger().info('地图已清空')
        return resp

    def _on_save(self, _req, resp):
        if self.backend.num_points() == 0:
            resp.success = False
            resp.message = 'empty map'
            return resp
        try:
            self.save_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = self.save_dir / f'scene_{stamp}.ply'
            pcd = self.backend.get_o3d_cloud()
            ok = o3d.io.write_point_cloud(str(path), pcd, write_ascii=False)
            if not ok:
                resp.success = False
                resp.message = f'open3d write failed: {path}'
                return resp
            resp.success = True
            resp.message = str(path)
            self.get_logger().info(f'已保存 {path} ({self.backend.num_points()} points)')
        except OSError as e:
            resp.success = False
            resp.message = str(e)
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReconFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
