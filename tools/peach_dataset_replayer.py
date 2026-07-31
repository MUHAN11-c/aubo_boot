"""
离线数据集回放工具：把数据集（rgb/ + depth/ 成对 PNG）发布为 RGB-D + CameraInfo.

独立测试工具（不随 colcon 构建，不依赖 peach_pose_ros2 包内模块）；无真相机时
可用它驱动 peach_pose_node 做全链路冒烟。默认内参为本机 Percipio
（与 color_camera_info.yaml 一致）；Azure 包请显式传 --fx 等。

用法示例（需先 source /opt/ros/jazzy/setup.bash 与工作区 install/setup.bash）:
    aubo_py3.12/bin/python tools/peach_dataset_replayer.py --limit 3 --loop
    aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <数据集根> --rate 0.5
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

# ── 本机 Percipio 彩色内参（640×480，棋盘标定）──
# 权威源: src/percipio_camera/config/color_camera_info.yaml — 改标定后请同步此处
K_PERCIPIO = {
    'fx': 466.174635,
    'fy': 465.556589,
    'cx': 326.071333,
    'cy': 244.789156,
    'width': 640,
    'height': 480,
}


def _camera_info(width: int, height: int, K: dict, frame_id: str, stamp) -> CameraInfo:
    """由 pinhole K 填 CameraInfo（无畸变；P 与 K 一致）."""
    msg = CameraInfo()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return msg


class DatasetReplayer(Node):
    """按定时器循环发布 rgb/ + depth/ 成对 PNG，话题名与真相机一致."""

    def __init__(self, dataset_dir: Path, rate_hz: float, limit: int,
                 loop: bool, frame_id: str, K: dict):
        super().__init__('peach_pose_dataset_replayer')
        self.dataset_dir = dataset_dir
        self.limit = limit
        self.loop = loop
        self.frame_id = frame_id
        self.K = K
        rgb_dir = dataset_dir / 'rgb'
        depth_dir = dataset_dir / 'depth'
        names = sorted(p.stem for p in rgb_dir.glob('*.png'))
        # 仅保留 rgb/depth 同名成对的帧
        self.frames = [
            (rgb_dir / f'{n}.png', depth_dir / f'{n}.png')
            for n in names
            if (depth_dir / f'{n}.png').is_file()
        ]
        if self.limit > 0:
            self.frames = self.frames[: self.limit]
        if not self.frames:
            raise RuntimeError(f'No rgb/depth pairs under {dataset_dir}')
        self.idx = 0
        self.pub_rgb = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_info = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', 10)
        period = 1.0 / max(rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Replaying {len(self.frames)} frames from {dataset_dir} @ {rate_hz} Hz')

    def _tick(self):
        """发布下一帧；三话题同 stamp，便于 ApproximateTimeSynchronizer."""
        if self.idx >= len(self.frames):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info('Replay finished')
                self.timer.cancel()
                return
        rgb_path, depth_path = self.frames[self.idx]
        self.idx += 1
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            self.get_logger().warn(f'Skip unreadable {rgb_path.name}')
            return
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        if rgb.shape[:2] != depth.shape[:2]:
            # 深度用最近邻，避免插值污染毫米值
            depth = cv2.resize(
                depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        stamp = self.get_clock().now().to_msg()
        h, w = rgb.shape[:2]

        rgb_msg = Image()
        rgb_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        rgb_msg.height, rgb_msg.width = h, w
        rgb_msg.encoding = 'bgr8'
        rgb_msg.is_bigendian = 0
        rgb_msg.step = w * 3
        rgb_msg.data = rgb.tobytes()

        depth_msg = Image()
        depth_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        depth_msg.height, depth_msg.width = h, w
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = 0
        depth_msg.step = w * 2
        depth_msg.data = depth.tobytes()

        info = _camera_info(w, h, self.K, self.frame_id, stamp)
        # Azure 历史包默认按 1280×720 标定；分辨率不同时按比例缩放 K
        if w != 1280 or h != 720:
            sx, sy = w / 1280.0, h / 720.0
            scaled = {
                'fx': self.K['fx'] * sx, 'fy': self.K['fy'] * sy,
                'cx': self.K['cx'] * sx, 'cy': self.K['cy'] * sy,
            }
            info = _camera_info(w, h, scaled, self.frame_id, stamp)

        self.pub_rgb.publish(rgb_msg)
        self.pub_depth.publish(depth_msg)
        self.pub_info.publish(info)
        self.get_logger().info(f'Published {rgb_path.name} ({self.idx}/{len(self.frames)})')


def main(args=None):
    """CLI：解析数据集路径与内参，spin 回放节点."""
    parser = argparse.ArgumentParser(description='PeachPose 数据集回放工具')
    parser.add_argument(
        '--dataset', type=str, default='',
        help='含 rgb/ 与 depth/ 的数据集根目录；空 → 工作区 '
             'src/peach_pose_ros2/data/dataset')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--limit', type=int, default=3)
    parser.add_argument('--loop', action='store_true')
    parser.add_argument(
        '--frame-id', type=str, default='camera_color_optical_frame')
    # 默认本机 Percipio 棋盘内参；Azure 离线包再改 CLI
    parser.add_argument('--fx', type=float, default=K_PERCIPIO['fx'])
    parser.add_argument('--fy', type=float, default=K_PERCIPIO['fy'])
    parser.add_argument('--cx', type=float, default=K_PERCIPIO['cx'])
    parser.add_argument('--cy', type=float, default=K_PERCIPIO['cy'])
    known, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    if known.dataset:
        dataset = Path(known.dataset)
    else:
        # 默认取包内 data/dataset 软链（tools/ 上一级即工作区根）
        dataset = (Path(__file__).resolve().parents[1]
                   / 'src' / 'peach_pose_ros2' / 'data' / 'dataset')
        if not dataset.is_dir():
            raise RuntimeError(
                f'推断默认数据集目录失败: {dataset} 不存在。'
                '请显式传 --dataset <数据集根目录>（含 rgb/ 与 depth/ 子目录）。')
    K = {'fx': known.fx, 'fy': known.fy, 'cx': known.cx, 'cy': known.cy}
    node = DatasetReplayer(
        dataset, known.rate, known.limit, known.loop, known.frame_id, K)
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
