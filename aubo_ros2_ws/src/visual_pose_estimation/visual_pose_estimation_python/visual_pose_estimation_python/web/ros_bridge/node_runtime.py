"""ROS2 节点实现：相机、服务客户端、与算法包交互；由 RosBridgeManager 在后台线程 spin。"""

from __future__ import annotations

import base64
import math
import threading
import time
import traceback

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from demo_interface.srv import ExecuteGraspPose
from tool_changer_interface.srv import RunGripperSwap
from interface.srv import EstimatePose, EstimatePose2D, ListTemplates, StandardizeTemplate, UpdateParams
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from std_srvs.srv import SetBool

from ...params_manager import ParamsManager
from ...path_resolver import WebPaths, resolve_web_paths
from ..runtime_support import quaternion_to_euler_rpy

try:
    from visual_pose_estimation_python.preprocessor import Preprocessor
    from visual_pose_estimation_python.feature_extractor import FeatureExtractor
    from visual_pose_estimation_python.config_reader import ConfigReader

    ALGORITHM_AVAILABLE = True
except ImportError:
    ALGORITHM_AVAILABLE = False
    Preprocessor = None
    FeatureExtractor = None
    ConfigReader = None

try:
    from percipio_camera_interface.msg import ImageData
    from percipio_camera_interface.srv import SoftwareTrigger

    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    ImageData = None
    SoftwareTrigger = None

try:
    from demo_interface.msg import RobotStatus
    from demo_interface.srv import MoveToPose, SetRobotIO

    ROBOT_AVAILABLE = True
except ImportError:
    ROBOT_AVAILABLE = False
    RobotStatus = None
    MoveToPose = None
    SetRobotIO = None


class ROS2Node(Node):
    """ROS2 bridge node used by the FastAPI web service."""

    def __init__(self, paths: WebPaths):
        super().__init__("algorithm_http_server_node")
        self._paths = paths
        self.bridge = CvBridge()

        self.trigger_client = self.create_client(SoftwareTrigger, "/software_trigger") if CAMERA_AVAILABLE else None
        self.estimate_pose_client = self.create_client(EstimatePose, "/estimate_pose")
        self.estimate_pose_2d_client = self.create_client(EstimatePose2D, "/estimate_pose_2d")
        self.list_templates_client = self.create_client(ListTemplates, "/list_templates")
        self.move_to_pose_client = self.create_client(MoveToPose, "/move_to_pose") if ROBOT_AVAILABLE else None
        self.standardize_template_client = self.create_client(StandardizeTemplate, "/standardize_template")
        self.update_params_client = self.create_client(UpdateParams, "/update_params")
        self.execute_single_grasp_client = self.create_client(ExecuteGraspPose, "/execute_single_grasp")
        self.loop_grasp_control_client = self.create_client(SetBool, "/loop_grasp_control")
        self.publish_grasps_loop_control_client = self.create_client(SetBool, "/publish_grasps_worker_loop_control")
        self.run_gripper_swap_client = self.create_client(RunGripperSwap, "/run_gripper_swap")
        self.write_plc_register_client = None
        self.set_robot_io_client = self.create_client(SetRobotIO, "/aubo_driver/set_io") if ROBOT_AVAILABLE else None

        self.robot_status_sub = (
            self.create_subscription(RobotStatus, "/aubo_driver/robot_status", self.robot_status_callback, 10)
            if ROBOT_AVAILABLE
            else None
        )
        self.image_data_sub = (
            self.create_subscription(ImageData, "/image_data", self.image_data_callback, 10) if CAMERA_AVAILABLE else None
        )
        self.depth_image_sub = None
        self.color_image_sub = None

        self.latest_robot_status = None
        self.latest_image_data = None
        self.image_received = False
        self.latest_depth_image = None
        self.latest_color_image = None
        self.depth_image_received = False
        self.color_image_received = False
        self.debug_last_capture_error = None
        self.robot_status_lock = threading.Lock()
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

    def robot_status_callback(self, msg):
        with self.robot_status_lock:
            self.latest_robot_status = msg

    def image_data_callback(self, msg):
        with self.image_lock:
            if hasattr(msg, "camera_id") and msg.camera_id:
                self.latest_image_data = msg
                self.image_received = True

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

    def get_robot_status(self, timeout=5.0):
        del timeout
        with self.robot_status_lock:
            if self.latest_robot_status is None:
                return None

            orientation = self.latest_robot_status.cartesian_position.orientation
            euler_rad = (
                list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_rad)
                if hasattr(self.latest_robot_status.cartesian_position, "euler_orientation_rpy_rad")
                else quaternion_to_euler_rpy(orientation.x, orientation.y, orientation.z, orientation.w)
            )
            euler_deg = (
                list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_deg)
                if hasattr(self.latest_robot_status.cartesian_position, "euler_orientation_rpy_deg")
                else [math.degrees(value) for value in quaternion_to_euler_rpy(orientation.x, orientation.y, orientation.z, orientation.w)]
            )

            return {
                "header": {
                    "stamp": {
                        "sec": self.latest_robot_status.header.stamp.sec,
                        "nanosec": self.latest_robot_status.header.stamp.nanosec,
                    },
                    "frame_id": self.latest_robot_status.header.frame_id,
                },
                "is_online": self.latest_robot_status.is_online,
                "enable": self.latest_robot_status.enable,
                "in_motion": self.latest_robot_status.in_motion,
                "joint_position_deg": list(self.latest_robot_status.joint_position_deg),
                "joint_position_rad": list(self.latest_robot_status.joint_position_rad),
                "cartesian_position": {
                    "position": {
                        "x": self.latest_robot_status.cartesian_position.position.x,
                        "y": self.latest_robot_status.cartesian_position.position.y,
                        "z": self.latest_robot_status.cartesian_position.position.z,
                    },
                    "orientation": {
                        "x": orientation.x,
                        "y": orientation.y,
                        "z": orientation.z,
                        "w": orientation.w,
                    },
                    "euler_orientation_rpy_rad": euler_rad,
                    "euler_orientation_rpy_deg": euler_deg,
                },
            }

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
        """等待 ROS2 Future 完成（Event 模式，无需阻塞 spin）。

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

    def capture_image(self, camera_id="207000152740", timeout=10.0):
        if not CAMERA_AVAILABLE or self.trigger_client is None:
            return None, None, "相机接口不可用"

        depth_sub = None
        color_sub = None
        try:
            if not self.trigger_client.wait_for_service(timeout_sec=5.0):
                return (
                    None,
                    None,
                    "相机服务未运行，请先启动camera_control_node节点（ros2 launch percipio_camera_interface camera_control.launch.py）",
                )

            with self.image_lock:
                self.image_received = False
                self.latest_image_data = None
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

            request = SoftwareTrigger.Request()
            request.camera_id = camera_id
            future = self.trigger_client.call_async(request)
            self._spin_future(future, timeout_sec=10.0)
            if not future.done():
                self._cleanup_temp_subscriptions(depth_sub, color_sub, "service timeout")
                return None, None, "相机服务调用超时"

            try:
                response = future.result()
                if response is None:
                    self._cleanup_temp_subscriptions(depth_sub, color_sub, "service failed")
                    return None, None, "相机服务调用失败"
                if not response.success:
                    if "Expected: camera" in response.message and camera_id and camera_id != "camera":
                        request.camera_id = "camera"
                        future = self.trigger_client.call_async(request)
                        self._spin_future(future, timeout_sec=10.0)
                        if not future.done():
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "retry timeout")
                            return None, None, "相机服务调用超时（重试）"
                        response = future.result()
                        if response is None:
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "retry failed")
                            return None, None, "相机服务调用失败（重试）"
                        if not response.success:
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "trigger failed")
                            return None, None, f"相机触发失败: {response.message}"
                    else:
                        self._cleanup_temp_subscriptions(depth_sub, color_sub, "trigger failed")
                        return None, None, f"相机触发失败: {response.message}"
            except Exception as exc:
                self._cleanup_temp_subscriptions(depth_sub, color_sub, "response failed")
                return None, None, f"获取服务响应失败: {exc}"

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
                error_msg = f"图像接收超时，缺少: {', '.join(missing)}"

            self._cleanup_temp_subscriptions(depth_sub, color_sub, "image timeout")
            return None, None, error_msg
        except Exception as exc:
            self._cleanup_temp_subscriptions(depth_sub, color_sub, "exception")
            self.get_logger().error(f"相机触发异常: {exc}")
            self.get_logger().error(traceback.format_exc())
            return None, None, f"相机触发异常: {exc}"

    def notify_params_updated(self):
        try:
            if self.update_params_client.wait_for_service(timeout_sec=0.5):
                request = UpdateParams.Request()
                request.section = "all"
                request.params_json = ""
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
                    "姿态估计服务未运行或未就绪，请检查visual_pose_estimation节点是否正常运行（ros2 launch visual_pose_estimation visual_pose_estimation.launch.py）",
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

    def move_to_pose(self, target_pose, use_joints=False, velocity_factor=0.08, acceleration_factor=0.05, timeout=180.0):
        try:
            if not ROBOT_AVAILABLE or self.move_to_pose_client is None:
                return None, "机器人接口不可用"
            if not self.move_to_pose_client.wait_for_service(timeout_sec=5.0):
                return None, "移动机器人位姿服务不可用"

            request = MoveToPose.Request()
            if isinstance(target_pose, (list, tuple)) and len(target_pose) >= 6:
                request.target_pose.position.x = float(target_pose[0])
                request.target_pose.position.y = float(target_pose[1])
                request.target_pose.position.z = float(target_pose[2])
                request.target_pose.orientation.x = float(target_pose[3])
                request.target_pose.orientation.y = float(target_pose[4])
                request.target_pose.orientation.z = float(target_pose[5])
                if len(target_pose) >= 7:
                    request.target_pose.orientation.w = float(target_pose[6])
                else:
                    qx, qy, qz = float(target_pose[3]), float(target_pose[4]), float(target_pose[5])
                    w_squared = 1.0 - (qx * qx + qy * qy + qz * qz)
                    request.target_pose.orientation.w = float(math.sqrt(w_squared)) if w_squared >= 0.0 else 0.0
            elif isinstance(target_pose, dict) and "position" in target_pose and "orientation" in target_pose:
                request.target_pose.position.x = float(target_pose["position"]["x"])
                request.target_pose.position.y = float(target_pose["position"]["y"])
                request.target_pose.position.z = float(target_pose["position"]["z"])
                request.target_pose.orientation.x = float(target_pose["orientation"]["x"])
                request.target_pose.orientation.y = float(target_pose["orientation"]["y"])
                request.target_pose.orientation.z = float(target_pose["orientation"]["z"])
                request.target_pose.orientation.w = float(target_pose["orientation"]["w"])
            else:
                return None, f"目标位姿格式错误，应为数组或包含position和orientation的字典，实际收到: {type(target_pose)}"

            request.use_joints = bool(use_joints)
            request.velocity_factor = float(velocity_factor)
            request.acceleration_factor = float(acceleration_factor)
            future = self.move_to_pose_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "移动机器人位姿服务调用超时"

            response = future.result()
            if response is None:
                return None, "移动机器人位姿服务调用失败"
            return {
                "success": bool(response.success),
                "error_code": int(response.error_code),
                "message": str(response.message),
            }, None
        except Exception as exc:
            self.get_logger().error(f"移动机器人位姿服务调用异常: {exc}\n{traceback.format_exc()}")
            return None, f"移动机器人位姿服务调用异常: {exc}"

    def set_robot_io(self, io_type, io_index, value, timeout=10.0):
        try:
            if not ROBOT_AVAILABLE or self.set_robot_io_client is None:
                return None, "机器人接口不可用"
            if not self.set_robot_io_client.wait_for_service(timeout_sec=5.0):
                return None, "设置机器人IO服务不可用"

            request = SetRobotIO.Request()
            request.io_type = str(io_type)
            request.io_index = int(io_index)
            request.value = float(value)
            future = self.set_robot_io_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                return None, "设置机器人IO服务调用超时"

            response = future.result()
            if response is None:
                return None, "设置机器人IO服务调用失败"
            return {
                "success": bool(response.success),
                "error_code": int(response.error_code),
                "message": str(response.message),
            }, None
        except Exception as exc:
            self.get_logger().error(f"设置机器人IO服务调用异常: {exc}\n{traceback.format_exc()}")
            return None, f"设置机器人IO服务调用异常: {exc}"

    def standardize_template(self, workpiece_id, timeout=120.0):
        try:
            if not self.standardize_template_client.wait_for_service(timeout_sec=5.0):
                return None, "模板标准化服务未运行，请先启动visual_pose_estimation节点"

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

    def write_plc_register(self, address, value):
        del address, value
        return None, "PLC写入功能暂未实现：WritePLCRegister服务未定义"

    def call_execute_single_grasp(self, object_id, use_visual_estimation, timeout=300.0):
        try:
            if not self.execute_single_grasp_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("单次抓取服务未运行，请先启动 execute_grasp_pose_worker 节点")
                return None

            request = ExecuteGraspPose.Request()
            request.object_id = str(object_id)
            request.use_visual_estimation = bool(use_visual_estimation)
            future = self.execute_single_grasp_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                self.get_logger().error("单次抓取服务调用超时")
                return None
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"单次抓取服务异常: {exc}\n{traceback.format_exc()}")
            return None

    def call_loop_grasp_control(self, start, timeout=10.0):
        try:
            if not self.loop_grasp_control_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("循环抓取控制服务未运行，请先启动 execute_grasp_pose_worker 节点")
                return None

            request = SetBool.Request()
            request.data = bool(start)
            future = self.loop_grasp_control_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                self.get_logger().error("循环抓取控制服务调用超时")
                return None
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"循环抓取控制服务异常: {exc}\n{traceback.format_exc()}")
            return None

    def call_publish_grasps_loop_control(self, start, timeout=10.0):
        try:
            if not self.publish_grasps_loop_control_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("publish_grasps 循环控制服务未运行，请先启动 publish_grasps_client_worker 节点")
                return None

            request = SetBool.Request()
            request.data = bool(start)
            future = self.publish_grasps_loop_control_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                self.get_logger().error("publish_grasps 循环控制服务调用超时")
                return None
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"publish_grasps 循环控制服务异常: {exc}\n{traceback.format_exc()}")
            return None

    def call_run_gripper_swap(self, direction, timeout=None):
        try:
            if not self.run_gripper_swap_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("夹爪切换服务未运行，请先启动 gripper_swap_worker 节点")
                return None

            request = RunGripperSwap.Request()
            request.direction = str(direction)
            future = self.run_gripper_swap_client.call_async(request)
            self._spin_future(future, timeout_sec=timeout)
            if not future.done():
                self.get_logger().error("夹爪切换服务调用超时")
                return None
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"夹爪切换服务异常: {exc}\n{traceback.format_exc()}")
            return None
