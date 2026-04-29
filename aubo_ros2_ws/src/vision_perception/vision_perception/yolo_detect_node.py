#!/usr/bin/env python3
"""
YOLO26 标准检测 + 实时显示节点
订阅图像话题，用 yolo26n.pt (COCO 80类) 推理，OpenCV 显示结果
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('classes', [0, 39, 40, 41, 44, 45])  # person,bottle,wine glass,cup,spoon,bowl

        input_topic = self.get_parameter('input_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf_threshold').value
        self.device = self.get_parameter('device').value
        self.classes = self.get_parameter('classes').value if self.get_parameter('classes').value else None

        self.bridge = CvBridge()

        self.model = YOLO(model_path)
        self.get_logger().info(f'模型加载: {model_path} ({self.device}), 共 {len(self.model.names)} 类')

        self.sub = self.create_subscription(Image, input_topic, self.callback, 10)
        self.get_logger().info(f'订阅: {input_topic}, 显示类: {[self.model.names.get(c,str(c)) for c in (self.classes or [])]}')

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, conf=self.conf, device=self.device, verbose=False, classes=self.classes)

        # 打印检测详情
        for box in results[0].boxes:
            cls = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()
            self.get_logger().info(f'{cls} conf={conf:.2f} @ [{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]', throttle_duration_sec=1.0)

        annotated = results[0].plot()
        cv2.imshow('YOLO26n Detection', annotated)
        if cv2.waitKey(1) == 27:
            raise SystemExit


def main():
    rclpy.init()
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
