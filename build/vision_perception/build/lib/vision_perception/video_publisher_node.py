#!/usr/bin/env python3
"""
视频发布节点: 读取视频文件，按指定帧率逐帧发布到 ROS2 图像话题。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        self.declare_parameter('video_path', '')
        self.declare_parameter('output_topic', '/camera/color/image_raw')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('resize_height', 480)

        video_path = self.get_parameter('video_path').value
        output_topic = self.get_parameter('output_topic').value
        self.fps = self.get_parameter('fps').value
        self.loop = self.get_parameter('loop').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value

        if not video_path:
            self.get_logger().fatal('video_path 参数未设置')
            raise RuntimeError('video_path 参数未设置')

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'无法打开视频: {video_path}')
            raise RuntimeError(f'无法打开视频: {video_path}')

        real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        total_frames = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.get_logger().info(
            f'视频加载成功: {video_path} '
            f'(原始 {real_fps:.1f}fps, {total_frames:.0f}帧, '
            f'发布 {self.fps}fps)'
        )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.period = 1.0 / self.fps
        self.timer = self.create_timer(self.period, self.publish_frame)

        self.get_logger().info(f'开始发布到话题: {output_topic}')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.get_logger().info('视频播放完毕，重新循环')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    return
            else:
                self.get_logger().info('视频播放完毕')
                self.timer.cancel()
                return

        w = self.resize_width
        h = self.resize_height
        if w > 0 or h > 0:
            if w == 0:
                orig_h, orig_w = frame.shape[:2]
                w = int(orig_w * h / orig_h)
            if h == 0:
                orig_h, orig_w = frame.shape[:2]
                h = int(orig_h * w / orig_w)
            frame = cv2.resize(frame, (w, h))

        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
