#!/usr/bin/env python3
"""
YOLO26 检测节点: 检测拉花缸(COCO bowl/bottle/cup)和咖啡杯，实时显示+发布
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json


class YoloTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_track_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/vision/yolo_track/image')
        self.declare_parameter('detection_topic', '/vision/yolo_track/detections')
        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('display_result', True)

        self.target_classes = {
            41: 'cup',
            45: 'bowl',    # 拉花缸
            39: 'bottle',
            44: 'spoon',
        }

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        self.conf = self.get_parameter('conf_threshold').value

        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO26 加载: {model_path} ({device})')

        self.bridge = CvBridge()

        input_topic = self.get_parameter('input_topic').value
        self.sub = self.create_subscription(Image, input_topic, self.image_cb, 10)
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('output_image_topic').value, 10)
        self.det_pub = self.create_publisher(
            String, self.get_parameter('detection_topic').value, 10)

        self.get_logger().info(
            f'启动, 订阅: {input_topic}, 目标: {list(self.target_classes.values())}')

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(
            frame, conf=self.conf,
            device=self.get_parameter('device').value,
            verbose=False, classes=list(self.target_classes.keys()))

        detections = []
        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()
            detections.append({
                'class': cls_name,
                'confidence': round(conf, 3),
                'box': [round(v) for v in xyxy],
            })

        annotated = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self.image_pub.publish(out_msg)

        self.det_pub.publish(String(data=json.dumps(detections, ensure_ascii=False)))

        if self.get_parameter('display_result').value:
            cv2.imshow('YOLO26 Latte Detection', annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
