"""ROS2 节点实现：相机、估姿服务客户端；由 RosBridgeManager 在后台线程 spin."""

from __future__ import annotations

import base64
import json
import math
import threading
import time
import traceback

import cv2
import numpy as np
from cv_bridge import CvBridge
from ivg_interfaces.srv import EstimatePose, EstimatePose2D, ListTemplates, StandardizeTemplate, UpdateParams
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import String

from ...params_manager import ParamsManager
from ...path_resolver import WebPaths
from ..runtime_support import quaternion_to_euler_rpy

try:
    from visual_pose_estimation_python.preprocessor import Preprocessor
    from visual_pose_estimation_python.feature_extractor import FeatureExtractor
    from visual_pose_estimation_python.config import ConfigReader

    ALGORITHM_AVAILABLE = True
except ImportError:
    ALGORITHM_AVAILABLE = False
    Preprocessor = None
    FeatureExtractor = None
    ConfigReader = None


class ROS2Node(Node):
    """ROS2 bridge node used by the FastAPI web service."""

    def __init__(self, paths: WebPaths):
        super().__init__("algorithm_http_server_node")
        self._paths = paths
        self.bridge = CvBridge()

        self.trigger_pub = self.create_publisher(String, "/camera/soft_trigger", 10)
        self.estimate_pose_client = self.create_client(EstimatePose, "/estimate_pose")
        self.estimate_pose_2d_client = self.create_client(EstimatePose2D, "/estimate_pose_2d")
        self.list_templates_client = self.create_client(ListTemplates, "/list_templates")
        self.standardize_template_client = self.create_client(StandardizeTemplate, "/standardize_template")
        self.update_params_client = self.create_client(UpdateParams, "/update_params")

        self.depth_image_sub = None
        self.color_image_sub = None

        self.latest_depth_image = None
        self.latest_color_image = None
        self.depth_image_received = False
        self.color_image_received = False
        self.debug_last_capture_error = None
        self.image_lock = threading.Lock()

        self.preprocessor = None
        self.feature_extractor = None
        self.params_manager = ParamsManager(config_path=paths.configs_dir / "debug_thresholds.json")
        self.config_reader = None

        if ALGORITHM_AVAILABLE:
            try:
                self.preprocessor = Preprocessor()
                self.feature_extractor = FeatureExtractor()
                self.config_reader = ConfigReader()
                self.get_logger().info("Algorithm modules initialized")
            except Exception as exc:
                self.get_logger().warning(f"Algorithm modules init failed: {exc}")

        self.get_logger().info("ROS2 bridge node initialized")

    def depth_image_callback(self, msg):
        try:
            with self.image_lock:
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.depth_image_received = True
        except Exception as exc:
            self.get_logger().error(f"Depth image conversion failed: {exc}")

    def color_image_callback(self, msg):
        try:
            with self.image_lock:
                self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.color_image_received = True
        except Exception as exc:
            self.get_logger().error(f"Color image conversion failed: {exc}")

    def _cleanup_temp_subscriptions(self, depth_sub, color_sub, reason: str = ""):
        try:
            if depth_sub is not None:
                self.destroy_subscription(depth_sub)
        except Exception as exc:
            self.get_logger().warning(f"销毁 depth_sub 失败: {exc}")
        try:
            if color_sub is not None:
                self.destroy_subscription(color_sub)
        except Exception as exc:
            self.get_logger().warning(f"销毁 color_sub 失败: {exc}")
        if reason:
            self.get_logger().info(f"临时订阅已清理: {reason}")

    def _spin_future(self, future, timeout_sec=None):
        """
        等待 ROS2 Future 完成（Event 模式，无需阻塞 spin）.

        依赖后台 RosBridgeManager 的 spin 线程处理服务响应。
        FastAPI 线程仅阻塞在 threading.Event.wait() 上，不干扰 ROS2 调度。
        """
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(timeout=timeout_sec):
            self.get_logger().warning(
                f"ROS2 service call timed out after {timeout_sec}s"
            )
            future.cancel()
            return False
        return True

    def capture_image(self, camera_id="", timeout=10.0):
        del camera_id
        depth_sub = None
        color_sub = None
        try:
            with self.image_lock:
                self.depth_image_received = False
                self.color_image_received = False
                self.latest_depth_image = None
                self.latest_color_image = None

            depth_sub = self.create_subscription(
                SensorImage,
                "/camera/depth/image_raw",
                self.depth_image_callback,
                10,
            )
            color_sub = self.create_subscription(
                SensorImage,
                "/camera/color/image_raw",
                self.color_image_callback,
                10,
            )
            time.sleep(0.2)

            msg = String()
            msg.data = "SoftTrigger"
            self.trigger_pub.publish(msg)

            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.image_lock:
                    if self.depth_image_received and self.color_image_received:
                        if self.latest_depth_image is not None and self.latest_color_image is not None:
                            depth_copy = self.latest_depth_image.copy()
                            color_copy = self.latest_color_image.copy()
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "image captured")
                            return depth_copy, color_copy, None
                time.sleep(0.05)

            with self.image_lock:
                missing = []
                if not self.depth_image_received or self.latest_depth_image is None:
                    missing.append("深度图")
                if not self.color_image_received or self.latest_color_image is None:
                    missing.append("彩色图")
                error_msg = (
                    f"图像接收超时，缺少: {', '.join(missing)}。"
                    "请确认 Percipio 已由 bringup 启动（话题 /camera/{color,depth}/image_raw）。"
                )

            self._cleanup_temp_subscriptions(depth_sub, color_sub, "image timeout")
            return None, None, error_msg
        except Exception as exc:
            self._cleanup_temp_subscriptions(depth_sub, color_sub, "exception")
            self.get_logger().error(f"相机采集异常: {exc}")
            self.get_logger().error(traceback.format_exc())
            return None, None, f"相机采集异常: {exc}"

    def notify_params_updated(self, params=None):
        """
        通知 ROS 端 /update_params 服务同步参数.

        Args:
            params: 可选的参数字典，直接传递给 ROS 节点（避免跨进程单例不可见）
        """
        try:
            if self.update_params_client.wait_for_service(timeout_sec=0.5):
                request = UpdateParams.Request()
                request.section = "all"
                request.params_json = json.dumps(params) if params else ""
                future = self.update_params_client.call_async(request)
                self._spin_future(future, timeout_sec=1.0)
        except Exception:
            pass

    def estimate_pose(self, depth_image_base64, color_image_base64, object_id, timeout=30.0):
        try:
            service_available = False
            for _attempt in range(3):
                if self.estimate_pose_client.wait_for_service(timeout_sec=3.0):
                    service_available = True
                    break
                time.sleep(0.5)
            if not service_available:
                return (
                    None,
                    "姿态估计服务未运行，请先启动 visual_pose_estimation_python"
                    "（ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py）",
                )

            try:
                depth_data = base64.b64decode(depth_image_base64)
                depth_array = np.frombuffer(depth_data, np.uint8)
                cv_depth = cv2.imdecode(depth_array, cv2.IMREAD_ANYDEPTH)
                if cv_depth is None:
                    return None, "深度图解码失败"
            except Exception as exc:
                return None, f"深度图解码异常: {exc}"

            try:
                color_data = base64.b64decode(color_image_base64)
                color_array = np.frombuffer(color_data, np.uint8)
                cv_color = cv2.imdecode(color_array, cv2.IMREAD_COLOR)
                if cv_color is None:
                    return None, "彩色图解码失败"
            except Exception as exc:
                return None, f"彩色图解码异常: {exc}"

            try:
                ros_depth_image = self.bridge.cv2_to_imgmsg(cv_depth, encoding="passthrough")
                ros_color_image = self.bridge.cv2_to_imgmsg(cv_color, encoding="bgr8")
            except Exception as exc:
                return None, f"图像转换失败: {exc}"

            request = EstimatePose.Request()
            request.image = ros_depth_image
            request.color_image = ros_color_image
            request.object_id = object_id
            future = self.estimate_pose_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "姿态估计服务调用超时"

            response = future.result()
            if response is None:
                return None, "姿态估计服务调用失败"

            result = {
                "success": True,
                "success_num": response.success_num,
                "confidence": list(response.confidence) if response.confidence else [],
                "matched_pose_ids": list(response.matched_pose_ids) if hasattr(response, "matched_pose_ids") else [],
                "positions": [],
                "grab_positions": [],
                "preparation_positions": [],
                "preplace_positions": [],
                "place_positions": [],
                "pose_images": [],
                "vis_image": "",
                "processing_time_sec": float(response.processing_time_sec)
                if hasattr(response, "processing_time_sec")
                else None,
            }

            for pos in response.position:
                result["positions"].append({"x": float(pos.x), "y": float(pos.y), "z": float(pos.z)})

            def convert_cartesian_position(cart_pos):
                if hasattr(cart_pos, "euler_orientation_rpy_rad"):
                    euler_rad = [float(value) for value in cart_pos.euler_orientation_rpy_rad]
                else:
                    euler_rad = quaternion_to_euler_rpy(
                        cart_pos.orientation.x,
                        cart_pos.orientation.y,
                        cart_pos.orientation.z,
                        cart_pos.orientation.w,
                    )

                if hasattr(cart_pos, "euler_orientation_rpy_deg"):
                    euler_deg = [float(value) for value in cart_pos.euler_orientation_rpy_deg]
                else:
                    euler_deg = [math.degrees(value) for value in euler_rad]

                return {
                    "position": {
                        "x": float(cart_pos.position.x),
                        "y": float(cart_pos.position.y),
                        "z": float(cart_pos.position.z),
                    },
                    "orientation": {
                        "x": float(cart_pos.orientation.x),
                        "y": float(cart_pos.orientation.y),
                        "z": float(cart_pos.orientation.z),
                        "w": float(cart_pos.orientation.w),
                    },
                    "euler_orientation_rpy_rad": euler_rad,
                    "euler_orientation_rpy_deg": euler_deg,
                    "joint_position_rad": [float(value) for value in cart_pos.joint_position_rad]
                    if hasattr(cart_pos, "joint_position_rad")
                    else [0.0] * 6,
                    "joint_position_deg": [float(value) for value in cart_pos.joint_position_deg]
                    if hasattr(cart_pos, "joint_position_deg")
                    else [0.0] * 6,
                }

            for grab_pos in response.grab_position:
                result["grab_positions"].append(convert_cartesian_position(grab_pos))
            for prep_pos in response.preparation_position:
                result["preparation_positions"].append(convert_cartesian_position(prep_pos))
            for preplace_pos in response.preplace_position:
                result["preplace_positions"].append(convert_cartesian_position(preplace_pos))
            for place_pos in response.place_position:
                result["place_positions"].append(convert_cartesian_position(place_pos))

            for index, pose_img in enumerate(response.pose_image):
                if index == 0 and pose_img.data:
                    try:
                        vis_cv_image = self.bridge.imgmsg_to_cv2(pose_img, desired_encoding="bgr8")
                        _, vis_buffer = cv2.imencode(".jpg", vis_cv_image)
                        result["vis_image"] = "data:image/jpeg;base64," + base64.b64encode(vis_buffer).decode("utf-8")
                    except Exception:
                        pass

                if pose_img.data:
                    try:
                        pose_cv_image = self.bridge.imgmsg_to_cv2(pose_img, desired_encoding="bgr8")
                        _, pose_buffer = cv2.imencode(".jpg", pose_cv_image)
                        result["pose_images"].append(
                            "data:image/jpeg;base64," + base64.b64encode(pose_buffer).decode("utf-8")
                        )
                    except Exception:
                        result["pose_images"].append("")
                else:
                    result["pose_images"].append("")

            return result, None
        except Exception as exc:
            self.get_logger().error(f"姿态估计服务异常: {exc}\n{traceback.format_exc()}")
            return None, f"姿态估计服务异常: {exc}"

    def estimate_pose_2d(self, color_image_base64, object_id, timeout=30.0):
        try:
            service_available = False
            for _attempt in range(3):
                if self.estimate_pose_2d_client.wait_for_service(timeout_sec=3.0):
                    service_available = True
                    break
                time.sleep(0.5)
            if not service_available:
                return None, "2D姿态估计服务未运行"

            try:
                color_data = base64.b64decode(color_image_base64)
                color_array = np.frombuffer(color_data, np.uint8)
                cv_color = cv2.imdecode(color_array, cv2.IMREAD_COLOR)
                if cv_color is None:
                    return None, "彩色图解码失败"
            except Exception as exc:
                return None, f"彩色图解码异常: {exc}"

            try:
                ros_color_image = self.bridge.cv2_to_imgmsg(cv_color, encoding="bgr8")
            except Exception as exc:
                return None, f"图像转换失败: {exc}"

            request = EstimatePose2D.Request()
            request.image = ros_color_image
            request.object_id = str(object_id)
            future = self.estimate_pose_2d_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "2D姿态估计服务调用超时"

            response = future.result()
            if response is None:
                return None, "2D姿态估计服务调用失败"

            result = {
                "success": True,
                "success_num": int(response.success_num),
                "center_x": list(response.center_x) if response.center_x else [],
                "center_y": list(response.center_y) if response.center_y else [],
                "rotation_angle": list(response.rotation_angle) if response.rotation_angle else [],
                "confidence": list(response.confidence) if response.confidence else [],
                "message": str(response.message) if response.message else "",
            }

            if response.vis_image and response.vis_image.data:
                try:
                    vis_cv = self.bridge.imgmsg_to_cv2(response.vis_image, desired_encoding="bgr8")
                    _, vis_buffer = cv2.imencode(".jpg", vis_cv)
                    result["vis_image"] = "data:image/jpeg;base64," + base64.b64encode(vis_buffer).decode("utf-8")
                except Exception:
                    result["vis_image"] = ""

            return result, None
        except Exception as exc:
            self.get_logger().error(f"2D姿态估计服务异常: {exc}\n{traceback.format_exc()}")
            return None, f"2D姿态估计服务异常: {exc}"

    def list_templates(self, workpiece_id="", timeout=10.0):
        try:
            if not self.list_templates_client.wait_for_service(timeout_sec=5.0):
                return None, "列出模板服务未运行，请先启动 visual_pose_estimation_python 节点"

            request = ListTemplates.Request()
            request.workpiece_id = workpiece_id or ""
            future = self.list_templates_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "列出模板服务调用超时"

            response = future.result()
            if response is None:
                return None, "列出模板服务调用失败"
            if not response.success:
                return None, response.error_message or "列出模板失败"
            return {
                "success": True,
                "template_ids": list(response.template_ids),
                "workpiece_ids": list(response.workpiece_ids),
            }, None
        except Exception as exc:
            self.get_logger().error(f"列出模板服务异常: {exc}\n{traceback.format_exc()}")
            return None, f"列出模板服务异常: {exc}"

    def standardize_template(self, workpiece_id, timeout=120.0):
        try:
            if not self.standardize_template_client.wait_for_service(timeout_sec=5.0):
                return None, "模板标准化服务未运行，请先启动 visual_pose_estimation_python 节点"

            request = StandardizeTemplate.Request()
            request.workpiece_id = str(workpiece_id)
            future = self.standardize_template_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "模板标准化服务调用超时"

            response = future.result()
            if response is None:
                return None, "模板标准化服务调用失败"
            return {
                "success": bool(response.success),
                "processed_count": int(response.processed_count),
                "skipped_count": int(response.skipped_count),
                "processed_pose_ids": list(response.processed_pose_ids),
                "skipped_pose_ids": list(response.skipped_pose_ids),
                "error_message": str(response.error_message) if response.error_message else "",
            }, None
        except Exception as exc:
            self.get_logger().error(f"模板标准化服务异常: {exc}\n{traceback.format_exc()}")
            return None, f"模板标准化服务异常: {exc}"
