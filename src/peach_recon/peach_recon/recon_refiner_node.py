"""
纯三维重建节点 — Open3D TSDF 融合 + 完整点云发布.

订阅 RGB-D + CameraInfo（三路近似同步）与 TF，逐帧把对齐 RGB-D 按相机位姿
融合进 ScalableTSDFVolume；定时提取**完整** TSDF 点云发布 ~/roi_cloud；
~/reset(Trigger) 重建 volume。

本节点只做重建，与感知完全解耦：ROI 裁剪/融合/精化/最终答案全部在
peach_pose_ros2 侧完成（它订阅本节点的完整云自行适配）。
本包节点经 console_scripts + venv shebang 运行（open3d 在 venv）。
"""
from cv_bridge import CvBridge
import message_filters
import numpy as np
from peach_pose_ros2.conversions import _transform_msg_to_matrix
from peach_pose_ros2.visualization import _pack_rgb_bgr, _xyzrgb_to_cloud
from peach_recon.roi_accumulator import TsdfRecon
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# 感知主输出话题（仅用于 ROI 盒中心定位，见模块 docstring）

# 参数默认值（与 config/recon.yaml 对齐，yaml 可覆盖）
DEFAULTS = {
    # 订阅与坐标系（命名/语义同 peach_pose_ros2）
    'color_topic': '/camera/color/image_raw',
    'depth_topic': '/camera/depth/image_raw',
    'camera_info_topic': '/camera/color/camera_info',
    'camera_optical_frame': 'camera_color_optical_frame',
    'output_frame': 'base_link',
    'tf_timeout_sec': 0.5,
    'sync_slop_s': 0.05,
    'depth_scale_unit': 0.25,  # Percipio 深度比例：raw × 本值 = 毫米（回放真毫米设 1.0）
    # TSDF 重建内核
    'tsdf_voxel_length': 0.005,   # TSDF 体素边长 (m)
    'sdf_trunc': 0.02,            # 截断距离 (m)（≥4×voxel）
    'depth_max_m': 3.0,           # 融合深度上限 (m)
    # ROI 发布
    'roi_cloud_period_sec': 1.0,  # ~/roi_cloud 发布周期 (s)（降频）
}


class ReconRefinerNode(Node):
    """纯重建节点：RGB-D → TSDF 融合 → ROI 裁剪点云."""

    def __init__(self):
        """建节点：参数 → TSDF 内核 → 同步订阅/TF → 发布/服务/定时器."""
        super().__init__('recon_refiner_node')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        def _get(name):
            return self.get_parameter(name).value

        self.color_topic = str(_get('color_topic'))
        self.depth_topic = str(_get('depth_topic'))
        self.camera_info_topic = str(_get('camera_info_topic'))
        self.camera_optical_frame = str(_get('camera_optical_frame')).strip()
        self.output_frame = str(_get('output_frame')).strip()
        self.tf_timeout = Duration(seconds=float(_get('tf_timeout_sec')))
        self.sync_slop_s = float(_get('sync_slop_s'))
        self.depth_scale_unit = float(_get('depth_scale_unit'))
        self.roi_cloud_period = float(_get('roi_cloud_period_sec'))

        self.bridge = CvBridge()
        self.recon = TsdfRecon(
            tsdf_voxel_length=float(_get('tsdf_voxel_length')),
            sdf_trunc=float(_get('sdf_trunc')),
            depth_max=float(_get('depth_max_m')))

        # TF：相机位姿（输出系←相机系）喂 TSDF integrate
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # RGB-D 三路近似同步（QoS/slop 写法同 peach_pose_ros2）
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        sub_rgb = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=self.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)

        self.pub_roi_cloud = self.create_publisher(
            PointCloud2, '~/roi_cloud', 10)
        self.srv_reset = self.create_service(
            Trigger, '~/reset', self._on_reset)
        self.create_timer(self.roi_cloud_period, self._publish_roi_cloud)
        self.get_logger().info(
            f'recon_refiner_node 就绪（纯 TSDF 重建）：voxel='
            f'{self.recon.tsdf_voxel_length}m trunc={self.recon.sdf_trunc}m，'
            f'输出系 {self.output_frame or "(camera)"}')

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """
        三路同步回调：转图像 → 查相机位姿 → TSDF 融合一帧.

        Args:
            rgb_msg: 彩色图（bgr8）.
            depth_msg: 深度图（uint16，毫米）.
            info: 彩色相机内参.

        Returns
        -------
            无返回值（None）；TF 失败丢帧（不融合错误位姿）.

        """
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg,
                                              desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'图像转换失败: {exc}',
                                   throttle_duration_sec=2.0)
            return
        if depth.dtype != np.uint16:
            self.get_logger().warn('深度非 uint16，丢帧',
                                   throttle_duration_sec=2.0)
            return
        # Percipio：raw × depth_scale_unit 才是毫米（不乘则距离放大 4 倍，
        # TSDF 几何落错位置，ROI 里看不到正确重建）
        if abs(self.depth_scale_unit - 1.0) > 1e-9:
            depth = np.clip(
                np.round(depth.astype(np.float32) * self.depth_scale_unit),
                0, 65535).astype(np.uint16)
        K = {
            'fx': float(info.k[0]), 'fy': float(info.k[4]),
            'cx': float(info.k[2]), 'cy': float(info.k[5]),
            'width': int(depth.shape[1]), 'height': int(depth.shape[0]),
        }
        cam_frame = (self.camera_optical_frame
                     or depth_msg.header.frame_id
                     or rgb_msg.header.frame_id)
        out_frame = self.output_frame or cam_frame
        if out_frame != cam_frame:
            try:
                # 相机位姿：输出系←相机系（world←cam，integrate 内部取逆）
                tf = self.tf_buffer.lookup_transform(
                    out_frame, cam_frame, Time.from_msg(rgb_msg.header.stamp),
                    timeout=self.tf_timeout)
                T = _transform_msg_to_matrix(tf.transform, self.get_logger())
            except TransformException as ex:
                self.get_logger().warn(f'TF 失败，本帧不融合: {ex}',
                                       throttle_duration_sec=2.0)
                return
        else:
            T = np.eye(4)
        self.recon.integrate_rgbd(rgb, depth, K, T)

    def _publish_roi_cloud(self):
        """定时：提取 TSDF 全部点云 → 发布 ~/roi_cloud（完整重建，不做 ROI 裁剪）."""
        xyz, rgb = self.recon.extract_cloud()
        if xyz.shape[0] == 0:
            return
        # 累积点数心跳（用户可直接在终端看"云在变厚"）
        self.get_logger().info(f'TSDF 累积 {xyz.shape[0]} 点',
                               throttle_duration_sec=5.0)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.output_frame or 'camera_color_optical_frame'
        # 颜色 [0,1] → uint8 → PointCloud2 位打包 rgb
        packed = _pack_rgb_bgr(np.round(rgb * 255.0).astype(np.uint8))
        self.pub_roi_cloud.publish(_xyzrgb_to_cloud(header, xyz, packed))

    def _on_reset(self, request, response):
        """
        ~/reset：重建 TSDF volume（清空全部融合）.

        Args:
            request: std_srvs/Trigger 请求（空）.
            response: std_srvs/Trigger 响应.

        Returns
        -------
            response（success=true）.

        """
        self.recon.reset()
        response.success = True
        response.message = '已重建 TSDF volume'
        self.get_logger().info('~/reset：已重建 TSDF volume')
        return response


def main(args=None):
    """
    入口：rclpy 初始化 → ReconRefinerNode spin → KeyboardInterrupt 干净收尾.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = ReconRefinerNode()
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
