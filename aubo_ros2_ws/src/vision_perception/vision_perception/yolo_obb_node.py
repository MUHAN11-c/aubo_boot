#!/usr/bin/env python3
"""
YOLO26 OBB 节点: 订阅图像话题，使用 Ultralytics YOLO26 OBB 模型进行旋转框检测，
发布标注图像和检测结果 (JSON)。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
import json


class YoloOBBNode(Node):
    def __init__(self):
        super().__init__('yolo_obb_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/vision/yolo_obb/image')
        self.declare_parameter('detection_topic', '/vision/yolo_obb/detections')
        self.declare_parameter('marker_topic', '/vision/yolo_obb/markers')
        self.declare_parameter('model_path', 'yolo26n-obb.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.7)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('publish_markers', True)

        input_topic = self.get_parameter('input_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, 10)

        self.image_publisher = self.create_publisher(
            Image, output_image_topic, 10)

        self.detection_publisher = self.create_publisher(
            String, detection_topic, 10)

        self.marker_publisher = self.create_publisher(
            MarkerArray, marker_topic, 10)

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value

        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info(
                f'YOLO26 OBB 模型加载成功: {model_path} (device={device})'
            )
        except Exception as e:
            self.get_logger().error(f'YOLO OBB 模型加载失败: {e}')
            raise

        self.device = device
        self.get_logger().info(
            f'YOLO OBB 节点已启动, 订阅: {input_topic}'
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        try:
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f'YOLO 推理失败: {e}')
            return

        annotated = cv_image.copy()
        detections = []
        marker_array = MarkerArray()
        marker_id = 0

        for result in results:
            if result.obb is None:
                continue

            boxes = result.obb.xyxyxyxy
            confs = result.obb.conf
            classes = result.obb.cls

            if boxes is None or len(boxes) == 0:
                continue

            boxes_np = boxes.cpu().numpy() if hasattr(boxes, 'cpu') else np.array(boxes)
            confs_np = confs.cpu().numpy() if hasattr(confs, 'cpu') else np.array(confs)
            classes_np = classes.cpu().numpy() if hasattr(classes, 'cpu') else np.array(classes)

            names = result.names if hasattr(result, 'names') else {}

            for i in range(len(boxes_np)):
                poly = boxes_np[i]
                conf = float(confs_np[i])
                cls_id = int(classes_np[i])
                cls_name = names.get(cls_id, f'class_{cls_id}')

                pts = poly.reshape(-1, 2).astype(np.int32)
                cv2.polylines(annotated, [pts], isClosed=True,
                              color=(0, 255, 0), thickness=2)
                label = f'{cls_name} {conf:.2f}'
                cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
                cv2.putText(annotated, label, (cx - 20, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                detections.append({
                    'class': cls_name,
                    'class_id': cls_id,
                    'confidence': conf,
                    'points': pts.tolist(),
                })

                if self.get_parameter('publish_markers').value:
                    m = Marker()
                    m.header = msg.header
                    m.ns = 'obb'
                    m.id = marker_id
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 0.0
                    m.color.a = 0.8
                    m.scale.x = 2.0
                    pts_loop = np.vstack([pts, pts[0]])
                    for p in pts_loop:
                        pt = Point()
                        pt.x = float(p[0])
                        pt.y = float(p[1])
                        pt.z = 0.0
                        m.points.append(pt)
                    marker_array.markers.append(m)
                    marker_id += 1

        out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self.image_publisher.publish(out_msg)

        det_msg = String()
        det_msg.data = json.dumps(detections, ensure_ascii=False)
        self.detection_publisher.publish(det_msg)

        if self.get_parameter('publish_markers').value and marker_array.markers:
            self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloOBBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
