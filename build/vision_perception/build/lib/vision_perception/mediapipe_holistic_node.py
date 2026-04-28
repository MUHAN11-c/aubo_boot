#!/usr/bin/env python3
"""
MediaPipe Holistic 节点 (mp.tasks API).
- 订阅图像，检测人体姿态+面部+手部关键点
- 发布标注图像、MarkerArray
- 发布手臂/手腕/手掌数据，用于机械臂末端映射
"""
import json
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core import base_options as mp_base_options

POSE_CONNECTIONS = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10), (11, 12), (11, 13), (13, 15), (15, 17), (15, 19),
    (15, 21), (17, 19), (12, 14), (14, 16), (16, 18), (16, 20),
    (16, 22), (18, 20), (11, 23), (12, 24), (23, 24), (23, 25),
    (24, 26), (25, 27), (26, 28), (27, 29), (28, 30), (29, 31),
    (30, 32), (27, 31), (28, 32),
])

HAND_CONNECTIONS = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
])

FACE_CONTOURS = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8),
    (8, 9), (9, 10), (10, 11), (11, 12), (12, 13), (13, 14), (14, 15),
    (15, 16), (17, 18), (18, 19), (19, 20), (20, 21), (22, 23),
    (23, 24), (24, 25), (25, 26), (27, 28), (28, 29), (29, 30),
    (30, 31), (32, 33), (33, 34), (34, 35), (35, 36),
])

POSE_COLOR = (0, 255, 0)
FACE_COLOR = (255, 0, 255)
LEFT_HAND_COLOR = (255, 0, 0)
RIGHT_HAND_COLOR = (0, 0, 255)

# MediaPipe Pose 关键点索引
P_SHOULDER_L = 11
P_SHOULDER_R = 12
P_ELBOW_L = 13
P_ELBOW_R = 14
P_WRIST_L = 15
P_WRIST_R = 16

# MediaPipe Hand 关键点索引
H_WRIST = 0
H_INDEX_MCP = 5
H_MIDDLE_MCP = 9
H_PINKY_MCP = 17


def _landmark_to_tuple(lm):
    return (lm.x, lm.y, lm.z)


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_norm(v):
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if mag < 1e-9:
        return (0.0, 0.0, 0.0)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _angle_between(a, b):
    """两个向量夹角 (弧度)"""
    dot = _vec_dot(a, b)
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot)


def _compute_arm_data(pose_lms, hand_lms, img_w, img_h):
    """
    从 MediaPipe 关键点计算手臂/手腕/手掌数据.
    返回 dict 或 None.
    pose_lms: List[NormalizedLandmark] (33 个)
    hand_lms: List[NormalizedLandmark] (21 个), 可为 None
    """
    if pose_lms is None:
        return None

    p = [_landmark_to_tuple(lm) for lm in pose_lms]
    if len(p) < 33:
        return None

    def _arm_side(shoulder_idx, elbow_idx, wrist_idx, hand_lms, side_label):
        """计算单侧手臂数据"""
        shoulder = p[shoulder_idx]
        elbow = p[elbow_idx]
        wrist = p[wrist_idx]

        upper = _vec_sub(elbow, shoulder)
        forearm = _vec_sub(wrist, elbow)

        upper_n = _vec_norm(upper)
        forearm_n = _vec_norm(forearm)
        elbow_angle = _angle_between(
            (-upper_n[0], -upper_n[1], -upper_n[2]), forearm_n)

        shoulder_angle = _angle_between(
            upper_n, (1.0, 0.0, 0.0))

        data = {
            'side': side_label,
            'shoulder': {'x': shoulder[0], 'y': shoulder[1], 'z': shoulder[2]},
            'elbow': {'x': elbow[0], 'y': elbow[1], 'z': elbow[2]},
            'wrist': {'x': wrist[0], 'y': wrist[1], 'z': wrist[2]},
            'wrist_px': {'x': wrist[0] * img_w, 'y': wrist[1] * img_h,
                         'z': wrist[2] * img_w},
            'upper_arm_vec': {'x': upper_n[0], 'y': upper_n[1], 'z': upper_n[2]},
            'forearm_vec': {'x': forearm_n[0], 'y': forearm_n[1], 'z': forearm_n[2]},
            'elbow_angle_rad': elbow_angle,
            'elbow_angle_deg': math.degrees(elbow_angle),
            'shoulder_angle_rad': shoulder_angle,
            'shoulder_angle_deg': math.degrees(shoulder_angle),
            'palm_normal': None,
            'palm_direction': None,
        }

        if hand_lms is not None and len(hand_lms) == 21:
            h = [_landmark_to_tuple(lm) for lm in hand_lms]
            # 手掌方向: 手腕 → 中指根部
            palm_dir = _vec_norm(_vec_sub(h[H_MIDDLE_MCP], h[H_WRIST]))
            # 手掌法向: (手腕→食指根部) × (手腕→小指根部)
            wrist_to_index = _vec_sub(h[H_INDEX_MCP], h[H_WRIST])
            wrist_to_pinky = _vec_sub(h[H_PINKY_MCP], h[H_WRIST])
            palm_n = _vec_norm(_vec_cross(wrist_to_index, wrist_to_pinky))

            data['palm_direction'] = {'x': palm_dir[0], 'y': palm_dir[1],
                                      'z': palm_dir[2]}
            data['palm_normal'] = {'x': palm_n[0], 'y': palm_n[1],
                                   'z': palm_n[2]}

            if np is not None:
                rx, ry, rz = _palm_euler(palm_dir, palm_n)
                data['palm_euler_rad'] = {'roll': rx, 'pitch': ry, 'yaw': rz}
                data['palm_euler_deg'] = {'roll': math.degrees(rx),
                                          'pitch': math.degrees(ry),
                                          'yaw': math.degrees(rz)}

        return data

    result = {}
    if pose_lms[P_WRIST_R] is not None:
        result['right_arm'] = _arm_side(P_SHOULDER_R, P_ELBOW_R, P_WRIST_R,
                                        hand_lms, 'right')
    if pose_lms[P_WRIST_L] is not None:
        result['left_arm'] = _arm_side(P_SHOULDER_L, P_ELBOW_L, P_WRIST_L,
                                       hand_lms, 'left')
    return result if result else None


def _palm_euler(palm_dir, palm_normal):
    """
    从手掌方向向量和法向量计算欧拉角 (roll, pitch, yaw).
    Camera frame: x→右, y→下, z→前 (MediaPipe 归一化坐标:
    x→右, y→下, z→前)
    返回 (roll, pitch, yaw) 弧度.
    """
    nx, ny, nz = palm_normal
    dx, dy, dz = palm_dir

    # Roll: 手掌绕 palm_dir 的旋转, 由法向量在 yz 平面偏移决定
    roll = math.atan2(ny, nz)
    # Pitch: 抬腕/压腕, 指尖向上/下
    pitch = -math.asin(max(-1.0, min(1.0, dy)))
    # Yaw: 手指方向在 xz 平面的投影
    yaw = math.atan2(dx, dz)

    return roll, pitch, yaw


def _draw_landmarks(img, landmarks, connections, color, point_radius=2, line_thickness=2):
    h, w = img.shape[:2]
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for x, y in pts:
        cv2.circle(img, (x, y), point_radius, color, -1)
    for i, j in connections:
        if i < len(pts) and j < len(pts):
            cv2.line(img, pts[i], pts[j], color, line_thickness)


class MediaPipeHolisticNode(Node):
    def __init__(self):
        super().__init__('mediapipe_holistic_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/vision/holistic/image')
        self.declare_parameter('marker_topic', '/vision/holistic/markers')
        self.declare_parameter('arm_data_topic', '/vision/holistic/arm_data')
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('enable_face', True)
        self.declare_parameter('enable_hands', True)
        self.declare_parameter('enable_pose', True)
        self.declare_parameter('model_asset_path', '')
        self.declare_parameter('publish_arm_data', True)
        self.declare_parameter('arm_view_topic', '/vision/holistic/arm_view')

        model_asset_path = self.get_parameter('model_asset_path').value
        if not model_asset_path:
            model_asset_path = os.path.expanduser(
                '~/.cache/mediapipe/holistic_landmarker.task')

        input_topic = self.get_parameter('input_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        arm_data_topic = self.get_parameter('arm_data_topic').value
        arm_view_topic = self.get_parameter('arm_view_topic').value

        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, input_topic,
                                            self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.arm_pub = self.create_publisher(String, arm_data_topic, 10)
        self.arm_view_pub = self.create_publisher(Image, arm_view_topic, 10)

        min_det = self.get_parameter('min_detection_confidence').value
        min_trk = self.get_parameter('min_tracking_confidence').value

        base_opts = mp_base_options.BaseOptions(
            model_asset_path=model_asset_path)
        options = vision.HolisticLandmarkerOptions(
            base_options=base_opts,
            running_mode=vision.RunningMode.IMAGE,
            min_face_detection_confidence=min_det,
            min_face_landmarks_confidence=min_trk,
            min_pose_detection_confidence=min_det,
            min_pose_landmarks_confidence=min_trk,
            min_hand_landmarks_confidence=min_det,
            output_face_blendshapes=False,
            output_segmentation_mask=False,
        )
        self.landmarker = vision.HolisticLandmarker.create_from_options(options)
        self.get_logger().info(
            f'MediaPipe Holistic 节点已启动, 订阅: {input_topic}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        h, w = cv_image.shape[:2]
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.landmarker.detect(mp_img)

        annotated = cv_image.copy()

        enable_face = self.get_parameter('enable_face').value
        enable_hands = self.get_parameter('enable_hands').value
        enable_pose = self.get_parameter('enable_pose').value

        markers = MarkerArray()
        marker_id = 0

        if enable_pose and result.pose_landmarks:
            lm_list = result.pose_landmarks
            _draw_landmarks(annotated, lm_list, POSE_CONNECTIONS, POSE_COLOR)
            for lm in lm_list:
                m = Marker()
                m.header = msg.header
                m.ns = 'pose'
                m.id = marker_id; marker_id += 1
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose.position.x = lm.x * w
                m.pose.position.y = lm.y * h
                m.pose.position.z = lm.z * w
                m.scale.x = m.scale.y = m.scale.z = 3.0
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
                m.color.a = 0.8
                markers.markers.append(m)

        if enable_face and result.face_landmarks:
            lm_list = result.face_landmarks
            _draw_landmarks(annotated, lm_list, FACE_CONTOURS, FACE_COLOR,
                            point_radius=1, line_thickness=1)
            for i, lm in enumerate(lm_list):
                if i % 5 != 0:
                    continue
                m = Marker()
                m.header = msg.header
                m.ns = 'face'
                m.id = marker_id; marker_id += 1
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose.position.x = lm.x * w
                m.pose.position.y = lm.y * h
                m.pose.position.z = lm.z * w
                m.scale.x = m.scale.y = m.scale.z = 1.5
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0
                m.color.a = 0.6
                markers.markers.append(m)

        hand_for_arm = None
        if enable_hands:
            if result.left_hand_landmarks:
                lm_list = result.left_hand_landmarks
                _draw_landmarks(annotated, lm_list, HAND_CONNECTIONS,
                                LEFT_HAND_COLOR)
                for lm in lm_list:
                    m = Marker()
                    m.header = msg.header
                    m.ns = 'left_hand'
                    m.id = marker_id; marker_id += 1
                    m.type = Marker.SPHERE; m.action = Marker.ADD
                    m.pose.position.x = lm.x * w
                    m.pose.position.y = lm.y * h
                    m.pose.position.z = lm.z * w
                    m.scale.x = m.scale.y = m.scale.z = 2.5
                    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0
                    m.color.a = 0.8
                    markers.markers.append(m)
                if hand_for_arm is None:
                    hand_for_arm = lm_list

            if result.right_hand_landmarks:
                lm_list = result.right_hand_landmarks
                _draw_landmarks(annotated, lm_list, HAND_CONNECTIONS,
                                RIGHT_HAND_COLOR)
                for lm in lm_list:
                    m = Marker()
                    m.header = msg.header
                    m.ns = 'right_hand'
                    m.id = marker_id; marker_id += 1
                    m.type = Marker.SPHERE; m.action = Marker.ADD
                    m.pose.position.x = lm.x * w
                    m.pose.position.y = lm.y * h
                    m.pose.position.z = lm.z * w
                    m.scale.x = m.scale.y = m.scale.z = 2.5
                    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
                    m.color.a = 0.8
                    markers.markers.append(m)
                if hand_for_arm is None:
                    hand_for_arm = lm_list

        out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        self.image_pub.publish(out_msg)

        if self.get_parameter('publish_markers').value:
            self.marker_pub.publish(markers)

        if self.get_parameter('publish_arm_data').value and result.pose_landmarks:
            arm_data = _compute_arm_data(result.pose_landmarks, hand_for_arm,
                                         w, h)
            if arm_data:
                arm_msg = String()
                arm_msg.data = json.dumps(arm_data, ensure_ascii=False)
                self.arm_pub.publish(arm_msg)

                arm_view = self._build_arm_view(cv_image, result.pose_landmarks,
                                                hand_for_arm, arm_data, w, h)
                if arm_view is not None:
                    av_msg = self.bridge.cv2_to_imgmsg(arm_view, 'bgr8')
                    av_msg.header.stamp = msg.header.stamp
                    av_msg.header.frame_id = msg.header.frame_id
                    self.arm_view_pub.publish(av_msg)

    def _build_arm_view(self, img, pose_lms, hand_lms, arm_data, w, h):
        """裁切手腕区域并叠加角度文字"""
        side = 'right_arm'
        if side not in arm_data:
            side = 'left_arm'
        if side not in arm_data:
            return None

        ad = arm_data[side]
        wx = int(ad['wrist_px']['x'])
        wy = int(ad['wrist_px']['y'])

        margin = 200
        x1 = max(0, wx - margin)
        y1 = max(0, wy - margin)
        x2 = min(w, wx + margin)
        y2 = min(h, wy + margin)

        crop = img[y1:y2, x1:x2].copy()

        if hand_lms and len(hand_lms) == 21:
            _draw_landmarks(crop, hand_lms, HAND_CONNECTIONS,
                            RIGHT_HAND_COLOR if side == 'right_arm'
                            else LEFT_HAND_COLOR,
                            point_radius=3, line_thickness=2)
            # 调整关键点坐标到裁切区域
            for lm in hand_lms:
                cx = int(lm.x * w) - x1
                cy = int(lm.y * h) - y1
                cv2.circle(crop, (cx, cy), 3,
                           (0, 255, 255) if side == 'right_arm'
                           else (255, 255, 0), -1)

        lines = [
            f"Side: {side}",
            f"Wrist px: ({wx},{wy})",
            f"Elbow angle: {ad['elbow_angle_deg']:.1f} deg",
            f"Shoulder angle: {ad['shoulder_angle_deg']:.1f} deg",
        ]
        if ad.get('palm_euler_deg'):
            eu = ad['palm_euler_deg']
            lines.append(
                f"Palm RPY: ({eu['roll']:.1f}, {eu['pitch']:.1f}, {eu['yaw']:.1f}) deg")

        for i, line in enumerate(lines):
            y_pos = 25 + i * 22
            cv2.putText(crop, line, (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0), 1, cv2.LINE_AA)

        return crop


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHolisticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
