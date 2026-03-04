#!/usr/bin/env python3

import http.server
import socketserver
import threading
import time
import sys
import os
import json
import base64
import csv
import yaml
import datetime
import math
import io
from pathlib import Path
from typing import Optional, Tuple
from PIL import Image

# ROS2 imports
import rclpy
from rclpy.node import Node
from interface.srv import EstimatePose, ListTemplates, StandardizeTemplate, UpdateParams

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2
import numpy as np

# 导入参数管理（统一路径/参数源）
from params_manager import ParamsManager

# 导入算法模块（用于Debug处理）
try:
    from visual_pose_estimation_python.preprocessor import Preprocessor
    from visual_pose_estimation_python.feature_extractor import FeatureExtractor
    from visual_pose_estimation_python.config_reader import ConfigReader
    ALGORITHM_AVAILABLE = True
except ImportError as e:
    print(f"警告: 无法导入算法模块: {e}")
    ALGORITHM_AVAILABLE = False

# 可选导入 - rembg：当前 Python 能导入则进程内使用，否则回退到子进程（conda 环境）
REMBG_AVAILABLE = False
RemBGProcessor = None
try:
    import onnxruntime  # noqa: F401
    from visual_pose_estimation_python.rembg_processor import RemBGProcessor as _RemBGProcessorClass
    RemBGProcessor = _RemBGProcessorClass
    REMBG_AVAILABLE = True
    print(f"✓ RemBG 进程内可用 (Python: {sys.executable})，将直接使用 GPU/CPU")
except ImportError as e:
    print(f"RemBG 进程内不可用: {e}")
    print(f"  Python: {sys.executable}，将回退到子进程（conda 环境）")
    RemBGProcessor = None
    REMBG_AVAILABLE = False

# Subprocess 版本的 RemBG 处理器（当无法直接导入时使用）
class SubprocessRemBGProcessor:
    """通过 subprocess 调用 conda 环境中的 rembg 脚本"""
    
    def __init__(self, model: str = "u2net", prefer_cuda: bool = True):
        self.model = model
        self.prefer_cuda = prefer_cuda
        self._conda_python = "/home/mu/miniconda3/envs/ros2_env/bin/python"
        self._script_path = Path(__file__).parent / "rembg_subprocess.py"
        self._providers = None
    
    @property
    def providers(self):
        return self._providers
    
    def process_roi(self, color_bgr: np.ndarray, bbox: Tuple[int, int, int, int]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """通过 subprocess 处理 ROI"""
        import subprocess
        import tempfile
        import os

        x0, y0, x1, y1 = bbox

        # 准备临时文件
        temp_dir = tempfile.gettempdir()
        input_file = None
        bbox_file = None
        
        try:
            # 保存图像到临时文件
            img_pil = Image.fromarray(color_bgr[:, :, ::-1])  # BGR to RGB
            input_file = tempfile.NamedTemporaryFile(mode='wb', suffix='.png', delete=False, dir=temp_dir)
            img_pil.save(input_file, format="PNG")
            input_file.close()
            input_path = input_file.name

            # 保存 bbox JSON 到临时文件
            bbox_data = {"x0": x0, "y0": y0, "x1": x1, "y1": y1}
            bbox_file = tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, dir=temp_dir)
            json.dump(bbox_data, bbox_file)
            bbox_file.close()
            bbox_path = bbox_file.name


            # 调用 subprocess（使用临时文件路径）
            result = subprocess.run(
                [self._conda_python, str(self._script_path), input_path, bbox_path, self.model],
                capture_output=True,
                text=True,
                timeout=30,
            )


            if result.returncode != 0:
                print(f"RemBG subprocess 错误: {result.stderr}", file=sys.stderr)
                return None, None

            output = json.loads(result.stdout)
            if not output.get("success"):
                print(f"RemBG subprocess 处理失败: {output.get('error')}", file=sys.stderr)
                return None, None

            self._providers = output.get("providers", ["CPUExecutionProvider"])

            # 解码结果
            mask_data = base64.b64decode(output["mask_b64"])
            mask_img = Image.open(io.BytesIO(mask_data))
            mask = np.array(mask_img.convert("L"))

            cutout_data = base64.b64decode(output["cutout_b64"])
            cutout_img = Image.open(io.BytesIO(cutout_data))
            cutout_rgb = np.array(cutout_img.convert("RGB"))
            cutout_bgr = cutout_rgb[:, :, ::-1]  # RGB to BGR


            return mask, cutout_bgr

        except subprocess.TimeoutExpired as e:
            print("RemBG subprocess 超时", file=sys.stderr)
            return None, None
        except Exception as e:
            import traceback
            tb = traceback.format_exc()
            print(f"RemBG subprocess 异常: {e}", file=sys.stderr)
            return None, None
        finally:
            # 清理临时文件
            if input_file and os.path.exists(input_path):
                try:
                    os.unlink(input_path)
                except Exception:
                    pass
            if bbox_file and os.path.exists(bbox_path):
                try:
                    os.unlink(bbox_path)
                except Exception:
                    pass

# 检查是否可以使用 subprocess 方式
SUBPROCESS_REMBG_AVAILABLE = False
if not REMBG_AVAILABLE:
    conda_python = "/home/mu/miniconda3/envs/ros2_env/bin/python"
    script_path = Path(__file__).parent / "rembg_subprocess.py"
    if Path(conda_python).exists() and script_path.exists():
        try:
            # 测试 conda Python 是否可以运行脚本
            import subprocess
            test_result = subprocess.run(
                [conda_python, "-c", "import rembg; import onnxruntime; print('OK')"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if test_result.returncode == 0:
                SUBPROCESS_REMBG_AVAILABLE = True
                REMBG_AVAILABLE = True  # 标记为可用，但使用 subprocess 方式
                print(f"✓ RemBG 将通过 subprocess 使用 conda 环境: {conda_python}")
        except Exception as e:
            print(f"警告: 无法验证 conda rembg 环境: {e}")

# 可选导入 - 相机接口
try:
    from percipio_camera_interface.srv import SoftwareTrigger
    from percipio_camera_interface.msg import ImageData
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False

# 可选导入 - 机器人接口  
try:
    from demo_interface.srv import MoveToPose, SetRobotIO
    from demo_interface.msg import RobotStatus
    ROBOT_AVAILABLE = True
except ImportError:
    ROBOT_AVAILABLE = False

def quaternion_to_euler_rpy(x, y, z, w):
    """
    将四元数转换为欧拉角 (Roll-Pitch-Yaw, RPY)
    参数: 四元数 (x, y, z, w)
    返回: (roll, pitch, yaw) 单位：弧度，顺序为 [roll, pitch, yaw]
    """
    # Roll (绕X轴)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (绕Y轴)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (绕Z轴)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

# 拍照姿态固定旋转：z 垂直地面无倾斜，精确四元数 (-0.707, -0.707, 0, 0)，姿态角 (-180, 0, 90) deg
CAMERA_POSE_FIXED_ORIENTATION = {
    "orientation": {"x": -0.7071067811865476, "y": -0.7071067811865476, "z": 0.0, "w": 0.0},
    "euler_orientation_rpy_rad": [-3.141592653589793, 0.0, 1.5707963267948966],
    "euler_orientation_rpy_deg": [-180.0, 0.0, 90.0],
}

_APP_CONFIG_CACHE = None

_REMBG_PROCESSOR = None


def _get_rembg_processor():
    global _REMBG_PROCESSOR
    if _REMBG_PROCESSOR is None and (REMBG_AVAILABLE or SUBPROCESS_REMBG_AVAILABLE):
        if RemBGProcessor is not None:
            # 直接导入方式
            _REMBG_PROCESSOR = RemBGProcessor(prefer_cuda=True)
        elif SUBPROCESS_REMBG_AVAILABLE:
            # subprocess 方式
            _REMBG_PROCESSOR = SubprocessRemBGProcessor(prefer_cuda=True)
    return _REMBG_PROCESSOR

def _load_app_config():
    """加载 web_ui/configs/app_config.json（默认路径：web_ui/configs，可配置 template_root、camera_pose_fixed_orientation）"""
    global _APP_CONFIG_CACHE
    if _APP_CONFIG_CACHE is not None:
        return _APP_CONFIG_CACHE
    # 默认配置目录：web_ui/configs（本文件在 web_ui/scripts）
    config_dir = Path(__file__).resolve().parent.parent / "configs"
    config_path = config_dir / "app_config.json"
    if config_path.exists():
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                _APP_CONFIG_CACHE = json.load(f)
        except Exception:
            _APP_CONFIG_CACHE = {}
    else:
        _APP_CONFIG_CACHE = {}
    return _APP_CONFIG_CACHE

def get_templates_dir():
    """获取模板根目录（唯一路径）：优先从 web_ui/configs/app_config.json 的 template_root 读取"""
    config = _load_app_config()
    template_root = config.get("template_root", "").strip()
    if template_root and Path(template_root).exists():
        return template_root
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent.parent.parent.parent
    fallback = repo_root / "templates"
    if fallback.exists():
        return str(fallback)
    return "/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/templates"


def _normalize_pose_rotation(robot_status):
    """规范化姿态中的旋转字段，与 camera_pose.json 一致：orientation (四元数) + euler_orientation_rpy_rad + euler_orientation_rpy_deg"""
    if not robot_status or 'cartesian_position' not in robot_status:
        return robot_status
    cp = robot_status['cartesian_position']
    pos = cp.get('position', {})
    ori = cp.get('orientation', {})
    ox = float(ori.get('x', 0.0))
    oy = float(ori.get('y', 0.0))
    oz = float(ori.get('z', 0.0))
    ow = float(ori.get('w', 1.0))
    euler_rad = cp.get('euler_orientation_rpy_rad')
    euler_deg = cp.get('euler_orientation_rpy_deg')
    if not isinstance(euler_rad, (list, tuple)) or len(euler_rad) != 3:
        euler_rad = quaternion_to_euler_rpy(ox, oy, oz, ow)
    else:
        euler_rad = [float(v) for v in euler_rad]
    if not isinstance(euler_deg, (list, tuple)) or len(euler_deg) != 3:
        euler_deg = [math.degrees(v) for v in euler_rad]
    else:
        euler_deg = [float(v) for v in euler_deg]
    # 按 camera_pose.json (34-49) 顺序：position, orientation, euler_orientation_rpy_rad, euler_orientation_rpy_deg
    robot_status['cartesian_position'] = {
        'position': pos,
        'orientation': {'x': ox, 'y': oy, 'z': oz, 'w': ow},
        'euler_orientation_rpy_rad': euler_rad,
        'euler_orientation_rpy_deg': euler_deg,
    }
    return robot_status


class ROS2Node(Node):
    """ROS2节点，用于与ROS2系统通信"""
    def __init__(self):
        super().__init__('algorithm_http_server_node')
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/software_trigger')
        self.estimate_pose_client = self.create_client(EstimatePose, '/estimate_pose')
        self.list_templates_client = self.create_client(ListTemplates, '/list_templates')
        self.move_to_pose_client = self.create_client(MoveToPose, '/move_to_pose')
        self.standardize_template_client = self.create_client(StandardizeTemplate, '/standardize_template')
        self.update_params_client = self.create_client(UpdateParams, '/update_params')
        # WritePLCRegister 服务暂未定义，相关功能已禁用
        # self.write_plc_register_client = self.create_client(WritePLCRegister, '/write_plc_register')
        self.write_plc_register_client = None
        # 创建SetRobotIO服务客户端
        if ROBOT_AVAILABLE:
            self.set_robot_io_client = self.create_client(SetRobotIO, '/demo_driver/set_io')
        else:
            self.set_robot_io_client = None
        
        # 创建订阅者
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        
        self.image_data_sub = self.create_subscription(
            ImageData,
            '/image_data',
            self.image_data_callback,
            10
        )
        
        # 不再持续订阅深度图和彩色图话题，改为在触发拍照时临时订阅
        # 移除初始化时的持续订阅，改为在 capture_image 时临时创建订阅
        self.depth_image_sub = None
        self.color_image_sub = None
        
        # 状态变量
        self.latest_robot_status = None
        self.latest_image_data = None
        self.image_received = False
        self.latest_depth_image = None
        self.latest_color_image = None
        self.depth_image_received = False
        self.color_image_received = False
        self.robot_status_lock = threading.Lock()
        self.image_lock = threading.Lock()
        
        # 执行器标志
        self.executor_running = False
        self.executor_lock = threading.Lock()
        
        # 初始化算法模块（用于Debug处理）
        self.preprocessor = None
        self.feature_extractor = None
        self.params_manager = ParamsManager()  # 用于保存参数
        self.config_reader = None  # 用于读取阈值参数（统一使用config_reader）
        
        if ALGORITHM_AVAILABLE:
            try:
                self.preprocessor = Preprocessor()
                self.feature_extractor = FeatureExtractor()
                # 初始化ConfigReader用于统一读取阈值参数
                self.config_reader = ConfigReader()
                self.get_logger().info('算法模块已初始化')
            except Exception as e:
                self.get_logger().warning(f'算法模块初始化失败: {e}')
        
        self.get_logger().info('ROS2节点初始化完成')
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        with self.robot_status_lock:
            self.latest_robot_status = msg
    
    def image_data_callback(self, msg):
        """图像数据回调"""
        with self.image_lock:
            # 只保存匹配相机ID的图像
            if hasattr(msg, 'camera_id') and msg.camera_id:
                self.latest_image_data = msg
                self.image_received = True
    
    def depth_image_callback(self, msg):
        """深度图回调"""
        try:
            with self.image_lock:
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.depth_image_received = True
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
    
    def color_image_callback(self, msg):
        """彩色图回调"""
        try:
            with self.image_lock:
                self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.color_image_received = True
        except Exception as e:
            self.get_logger().error(f'彩色图转换失败: {e}')
    
    def get_robot_status(self, timeout=5.0):
        """获取最新的机器人状态"""
        with self.robot_status_lock:
            if self.latest_robot_status is None:
                return None
            
            # 转换为字典格式
            status_dict = {
                "header": {
                    "stamp": {
                        "sec": self.latest_robot_status.header.stamp.sec,
                        "nanosec": self.latest_robot_status.header.stamp.nanosec
                    },
                    "frame_id": self.latest_robot_status.header.frame_id
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
                        "z": self.latest_robot_status.cartesian_position.position.z
                    },
                    "orientation": {
                        "x": self.latest_robot_status.cartesian_position.orientation.x,
                        "y": self.latest_robot_status.cartesian_position.orientation.y,
                        "z": self.latest_robot_status.cartesian_position.orientation.z,
                        "w": self.latest_robot_status.cartesian_position.orientation.w
                    },
                    "euler_orientation_rpy_rad": (
                        list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_rad) 
                        if hasattr(self.latest_robot_status.cartesian_position, 'euler_orientation_rpy_rad') 
                        else quaternion_to_euler_rpy(
                            self.latest_robot_status.cartesian_position.orientation.x,
                            self.latest_robot_status.cartesian_position.orientation.y,
                            self.latest_robot_status.cartesian_position.orientation.z,
                            self.latest_robot_status.cartesian_position.orientation.w
                        )
                    ),
                    "euler_orientation_rpy_deg": (
                        list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_deg) 
                        if hasattr(self.latest_robot_status.cartesian_position, 'euler_orientation_rpy_deg') 
                        else [math.degrees(v) for v in quaternion_to_euler_rpy(
                            self.latest_robot_status.cartesian_position.orientation.x,
                            self.latest_robot_status.cartesian_position.orientation.y,
                            self.latest_robot_status.cartesian_position.orientation.z,
                            self.latest_robot_status.cartesian_position.orientation.w
                        )]
                    )
                }
            }
            return status_dict
    
    def _cleanup_temp_subscriptions(self, depth_sub, color_sub, reason=""):
        """清理临时订阅的辅助函数
        
        Args:
            depth_sub: 深度图订阅对象
            color_sub: 彩色图订阅对象
            reason: 清理原因（用于日志）
        """
        try:
            if depth_sub:
                self.destroy_subscription(depth_sub)
            if color_sub:
                self.destroy_subscription(color_sub)
            if reason:
                self.get_logger().info(f'✓ 已清理临时订阅 - {reason}')
        except Exception as e:
            self.get_logger().warning(f'清理订阅时出错: {e}')
    
    def capture_image(self, camera_id='207000152740', timeout=10.0):
        """触发相机拍照并获取深度图和彩色图
        
        注意：此方法会临时创建订阅获取图像，获取后立即销毁订阅，不持续接收视频流
        
        Returns:
            (depth_image, color_image, error_msg): 深度图、彩色图和错误信息的元组
            如果成功，返回 (depth_ndarray, color_ndarray, None)
            如果失败，返回 (None, None, error_message)
        """
        depth_sub = None
        color_sub = None
        
        try:
            # 等待服务可用（增加超时时间，因为服务可能需要时间启动）
            if not self.trigger_client.wait_for_service(timeout_sec=5.0):
                return None, None, "相机服务未运行，请先启动camera_control_node节点（ros2 launch percipio_camera_interface camera_control.launch.py）"
            
            # 临时创建订阅（不持续订阅视频流）
            self.get_logger().info('📸 临时创建图像订阅，准备触发拍照...')
            
            # 重置图像接收标志
            with self.image_lock:
                self.image_received = False
                self.latest_image_data = None
                self.depth_image_received = False
                self.color_image_received = False
                self.latest_depth_image = None
                self.latest_color_image = None
            
            # 临时创建深度图和彩色图订阅
            depth_sub = self.create_subscription(
                SensorImage,
                '/camera/depth/image_raw',
                self.depth_image_callback,
                10
            )
            
            color_sub = self.create_subscription(
                SensorImage,
                '/camera/color/image_raw',
                self.color_image_callback,
                10
            )
            
            # 等待一小段时间让订阅建立（ROS2订阅需要时间建立连接）
            time.sleep(0.2)
            
            # 调用服务触发拍照
            # camera_control_node 期望的相机ID可能是 "camera"（camera_name）或序列号（camera_id_）
            # 如果camera_id_为空，则只接受 "camera"
            # 先尝试使用传入的camera_id，如果失败且是序列号格式，再尝试使用 "camera"
            request = SoftwareTrigger.Request()
            request.camera_id = camera_id
            
            future = self.trigger_client.call_async(request)
            
            # 等待服务响应 - 使用spin_until_future_complete
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                self._cleanup_temp_subscriptions(depth_sub, color_sub, "服务调用超时")
                return None, None, "相机服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    self._cleanup_temp_subscriptions(depth_sub, color_sub, "服务调用失败")
                    return None, None, "相机服务调用失败"
                if not response.success:
                    # 如果失败且错误信息包含 "Expected: camera"，且传入的是序列号，尝试使用 "camera"
                    if "Expected: camera" in response.message and camera_id and camera_id != "camera":
                        self.get_logger().info(f'相机ID不匹配，尝试使用 "camera" 替代序列号 {camera_id}')
                        request.camera_id = "camera"
                        future = self.trigger_client.call_async(request)
                        with self.executor_lock:
                            self.executor_running = True
                        try:
                            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                        finally:
                            with self.executor_lock:
                                self.executor_running = False
                        
                        if not future.done():
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "服务调用超时（重试）")
                            return None, None, "相机服务调用超时（重试）"
                        
                        response = future.result()
                        if response is None:
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "服务调用失败（重试）")
                            return None, None, "相机服务调用失败（重试）"
                        if not response.success:
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "相机触发失败")
                            return None, None, f"相机触发失败: {response.message}"
                    else:
                        self._cleanup_temp_subscriptions(depth_sub, color_sub, "相机触发失败")
                        return None, None, f"相机触发失败: {response.message}"
            except Exception as e:
                self._cleanup_temp_subscriptions(depth_sub, color_sub, "获取服务响应失败")
                return None, None, f"获取服务响应失败: {str(e)}"
            
            # 等待深度图和彩色图数据 - 后台线程会处理消息，这里只等待
            self.get_logger().info('等待深度图和彩色图...')
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.image_lock:
                    # 检查是否同时接收到深度图和彩色图
                    if self.depth_image_received and self.color_image_received:
                        if self.latest_depth_image is not None and self.latest_color_image is not None:
                            self.get_logger().info(f'✓ 成功接收图像 - 深度图 {self.latest_depth_image.shape}, 彩色图 {self.latest_color_image.shape}')
                            # 复制图像以便返回
                            depth_copy = self.latest_depth_image.copy()
                            color_copy = self.latest_color_image.copy()
                            
                            # 销毁临时订阅（不再持续接收视频流）
                            self._cleanup_temp_subscriptions(depth_sub, color_sub, "成功获取图像，节省带宽")
                            
                            return depth_copy, color_copy, None
                
                # 短暂休眠，等待后台线程处理消息
                time.sleep(0.05)
            
            # 超时，检查哪些图像未接收到
            with self.image_lock:
                missing = []
                if not self.depth_image_received or self.latest_depth_image is None:
                    missing.append('深度图')
                if not self.color_image_received or self.latest_color_image is None:
                    missing.append('彩色图')
                error_msg = f"图像接收超时，缺少: {', '.join(missing)}"
                self.get_logger().error(error_msg)
            
            # 销毁临时订阅
            self._cleanup_temp_subscriptions(depth_sub, color_sub, "图像接收超时")
            
            return None, None, error_msg
            
        except Exception as e:
            # 确保销毁临时订阅
            self._cleanup_temp_subscriptions(depth_sub, color_sub, "异常")
            
            self.get_logger().error(f'相机触发异常: {str(e)}')
            import traceback
            traceback.print_exc()
            return None, None, f"相机触发异常: {str(e)}"
    
    def notify_params_updated(self):
        """通知ROS2节点重新加载配置文件"""
        try:
            if self.update_params_client.wait_for_service(timeout_sec=0.5):
                request = UpdateParams.Request()
                request.section = 'all'  # 重新加载所有参数
                request.params_json = ''  # 不再传递参数，直接读文件
                future = self.update_params_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception:
            pass
    
    def estimate_pose(self, depth_image_base64, color_image_base64, object_id, timeout=30.0):
        """调用姿态估计服务
        
        Args:
            depth_image_base64: 深度图的base64编码
            color_image_base64: 彩色图的base64编码
            object_id: 工件ID
            timeout: 超时时间（秒）
        
        Returns:
            (result_dict, error_msg): 结果字典和错误信息的元组
        """
        try:
            self.get_logger().info(f'开始姿态估计，工件ID: {object_id}')
            
            # 等待服务可用（增加超时时间，因为服务可能需要时间启动）
            # 服务可能已注册但需要更多时间准备，尝试多次等待
            service_available = False
            for attempt in range(3):
                if self.estimate_pose_client.wait_for_service(timeout_sec=3.0):
                    service_available = True
                    break
                self.get_logger().warn(f'等待姿态估计服务可用，尝试 {attempt + 1}/3...')
                time.sleep(0.5)
            
            if not service_available:
                return None, "姿态估计服务未运行或未就绪，请检查visual_pose_estimation节点是否正常运行（ros2 launch visual_pose_estimation visual_pose_estimation.launch.py）"
            
            # 解码深度图
            try:
                depth_data = base64.b64decode(depth_image_base64)
                np_arr = np.frombuffer(depth_data, np.uint8)
                cv_depth = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
                if cv_depth is None:
                    return None, "深度图解码失败"
                self.get_logger().info(f'深度图解码成功: {cv_depth.shape}, dtype={cv_depth.dtype}')
            except Exception as e:
                return None, f"深度图解码异常: {str(e)}"
            
            # 解码彩色图
            try:
                color_data = base64.b64decode(color_image_base64)
                np_arr = np.frombuffer(color_data, np.uint8)
                cv_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_color is None:
                    return None, "彩色图解码失败"
                self.get_logger().info(f'彩色图解码成功: {cv_color.shape}')
            except Exception as e:
                return None, f"彩色图解码异常: {str(e)}"
            
            # 转换为ROS2图像消息
            try:
                ros_depth_image = self.bridge.cv2_to_imgmsg(cv_depth, encoding='passthrough')
                ros_color_image = self.bridge.cv2_to_imgmsg(cv_color, encoding='bgr8')
            except Exception as e:
                return None, f"图像转换失败: {str(e)}"
            
            # 创建请求
            request = EstimatePose.Request()
            request.image = ros_depth_image  # 深度图
            request.color_image = ros_color_image  # 彩色图
            request.object_id = object_id
            
            # 调用服务
            future = self.estimate_pose_client.call_async(request)
            
            # 等待服务响应 - 使用spin_until_future_complete
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                return None, "姿态估计服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "姿态估计服务调用失败"
                
                self.get_logger().info(f'姿态估计服务响应: success_num={response.success_num}')
                
                # 转换响应为字典格式
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
                    "processing_time_sec": float(response.processing_time_sec) if hasattr(response, 'processing_time_sec') else None
                }
                
                # 转换位置信息
                for pos in response.position:
                    result["positions"].append({
                        "x": float(pos.x),
                        "y": float(pos.y),
                        "z": float(pos.z)
                    })
                
                # 辅助函数：转换CartesianPosition
                def convert_cartesian_position(cart_pos):
                    # 计算欧拉角（如果属性不存在，从四元数转换）
                    if hasattr(cart_pos, 'euler_orientation_rpy_rad'):
                        euler_rad = [float(v) for v in cart_pos.euler_orientation_rpy_rad]
                    else:
                        euler_rad = quaternion_to_euler_rpy(
                            cart_pos.orientation.x,
                            cart_pos.orientation.y,
                            cart_pos.orientation.z,
                            cart_pos.orientation.w
                        )
                    
                    if hasattr(cart_pos, 'euler_orientation_rpy_deg'):
                        euler_deg = [float(v) for v in cart_pos.euler_orientation_rpy_deg]
                    else:
                        euler_deg = [math.degrees(v) for v in euler_rad]
                    
                    pos_dict = {
                        "position": {
                            "x": float(cart_pos.position.x),
                            "y": float(cart_pos.position.y),
                            "z": float(cart_pos.position.z)
                        },
                        "orientation": {
                            "x": float(cart_pos.orientation.x),
                            "y": float(cart_pos.orientation.y),
                            "z": float(cart_pos.orientation.z),
                            "w": float(cart_pos.orientation.w)
                        },
                        "euler_orientation_rpy_rad": euler_rad,
                        "euler_orientation_rpy_deg": euler_deg,
                        "joint_position_rad": [float(v) for v in cart_pos.joint_position_rad] if hasattr(cart_pos, 'joint_position_rad') else [0.0] * 6,
                        "joint_position_deg": [float(v) for v in cart_pos.joint_position_deg] if hasattr(cart_pos, 'joint_position_deg') else [0.0] * 6
                    }
                    return pos_dict
                
                # 转换抓取位置
                for grab_pos in response.grab_position:
                    result["grab_positions"].append(convert_cartesian_position(grab_pos))
                
                # 转换准备位置
                for prep_pos in response.preparation_position:
                    result["preparation_positions"].append(convert_cartesian_position(prep_pos))
                
                # 转换预放置位置
                for preplace_pos in response.preplace_position:
                    result["preplace_positions"].append(convert_cartesian_position(preplace_pos))
                
                # 转换放置位置
                for place_pos in response.place_position:
                    result["place_positions"].append(convert_cartesian_position(place_pos))
                
                # 转换姿态图像数组（服务响应中没有vis_image字段，只有pose_image数组）
                # 如果pose_image数组不为空，使用第一个图像作为可视化图像
                for idx, pose_img in enumerate(response.pose_image):
                    if idx == 0 and pose_img.data:
                        # 使用第一个姿态图像作为可视化图像
                        try:
                            vis_cv_image = self.bridge.imgmsg_to_cv2(pose_img, desired_encoding='bgr8')
                            _, vis_buffer = cv2.imencode('.jpg', vis_cv_image)
                            result["vis_image"] = "data:image/jpeg;base64," + base64.b64encode(vis_buffer).decode('utf-8')
                        except Exception as e:
                            self.get_logger().warn(f'可视化图像转换失败: {str(e)}')
                    
                    # 转换所有姿态图像
                    if pose_img.data:
                        try:
                            pose_cv_image = self.bridge.imgmsg_to_cv2(pose_img, desired_encoding='bgr8')
                            _, pose_buffer = cv2.imencode('.jpg', pose_cv_image)
                            result["pose_images"].append("data:image/jpeg;base64," + base64.b64encode(pose_buffer).decode('utf-8'))
                        except Exception as e:
                            self.get_logger().warn(f'姿态图像转换失败: {str(e)}')
                            result["pose_images"].append("")
                    else:
                        result["pose_images"].append("")
                
                self.get_logger().info(f'姿态估计完成，成功数量: {result["success_num"]}')
                return result, None
                
            except Exception as e:
                import traceback
                error_detail = traceback.format_exc()
                self.get_logger().error(f'处理服务响应失败: {str(e)}\n{error_detail}')
                return None, f"处理服务响应失败: {str(e)}"
                
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            self.get_logger().error(f'姿态估计服务异常: {str(e)}\n{error_detail}')
            return None, f"姿态估计服务异常: {str(e)}"
    
    def list_templates(self, workpiece_id="", timeout=10.0):
        """调用 ROS2 /list_templates 服务获取工件列表（符合 ROS2 接口）"""
        try:
            self.get_logger().info(f'调用 /list_templates 服务，workpiece_id: {workpiece_id if workpiece_id else "全部"}')
            
            if not self.list_templates_client.wait_for_service(timeout_sec=5.0):
                return None, "列出模板服务未运行，请先启动 visual_pose_estimation_python 节点"
            
            request = ListTemplates.Request()
            request.workpiece_id = workpiece_id or ""
            
            future = self.list_templates_client.call_async(request)
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                return None, "列出模板服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "列出模板服务调用失败"
                if response.success:
                    result = {
                        "success": True,
                        "template_ids": list(response.template_ids),
                        "workpiece_ids": list(response.workpiece_ids),
                    }
                    self.get_logger().info(f'列出模板成功，工件数: {len(result["workpiece_ids"])}')
                    return result, None
                else:
                    return None, response.error_message or "列出模板失败"
            except Exception as e:
                return None, f"处理服务响应失败: {str(e)}"
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            self.get_logger().error(f'列出模板服务异常: {str(e)}\n{error_detail}')
            return None, f"列出模板服务异常: {str(e)}"
    
    def move_to_pose(self, target_pose, use_joints=False, velocity_factor=0.08, acceleration_factor=0.05, timeout=30.0):
        """移动机器人到目标位姿（使用 /move_to_pose 服务）"""
        try:
            if not self.move_to_pose_client.wait_for_service(timeout_sec=5.0):
                return None, "移动机器人位姿服务不可用"
            
            request = MoveToPose.Request()
            
            # 解析位姿数据（支持数组格式和字典格式）
            if isinstance(target_pose, (list, tuple)) and len(target_pose) == 6:
                # 数组格式：[x, y, z, qx, qy, qz, qw] 或 [x, y, z, roll, pitch, yaw]
                # 假设是 [x, y, z, qx, qy, qz, qw] 格式（四元数）
                request.target_pose.position.x = float(target_pose[0])
                request.target_pose.position.y = float(target_pose[1])
                request.target_pose.position.z = float(target_pose[2])
                request.target_pose.orientation.x = float(target_pose[3])
                request.target_pose.orientation.y = float(target_pose[4])
                request.target_pose.orientation.z = float(target_pose[5])
                # 如果没有提供 qw，假设为 1.0（归一化）
                if len(target_pose) >= 7:
                    request.target_pose.orientation.w = float(target_pose[6])
                else:
                    # 计算归一化的 w（处理负数情况，避免复数）
                    qx, qy, qz = float(target_pose[3]), float(target_pose[4]), float(target_pose[5])
                    q_squared_sum = qx*qx + qy*qy + qz*qz
                    w_squared = 1.0 - q_squared_sum
                    if w_squared < 0.0:
                        # 如果计算出的 w² 为负数，说明四元数未归一化，使用 0.0
                        qw = 0.0
                    else:
                        qw = math.sqrt(w_squared)
                    request.target_pose.orientation.w = float(qw)
            elif isinstance(target_pose, dict) and 'position' in target_pose and 'orientation' in target_pose:
                # 字典格式：包含 position 和 orientation
                request.target_pose.position.x = float(target_pose['position']['x'])
                request.target_pose.position.y = float(target_pose['position']['y'])
                request.target_pose.position.z = float(target_pose['position']['z'])
                request.target_pose.orientation.x = float(target_pose['orientation']['x'])
                request.target_pose.orientation.y = float(target_pose['orientation']['y'])
                request.target_pose.orientation.z = float(target_pose['orientation']['z'])
                request.target_pose.orientation.w = float(target_pose['orientation']['w'])
            else:
                return None, f"目标位姿格式错误，应为包含6或7个元素的数组，或包含position和orientation的字典，实际收到: {type(target_pose)}"
            
            request.use_joints = bool(use_joints)
            request.velocity_factor = float(velocity_factor)
            request.acceleration_factor = float(acceleration_factor)
            
            future = self.move_to_pose_client.call_async(request)
            
            # 等待服务响应 - 使用spin_until_future_complete
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                return None, "移动机器人位姿服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "移动机器人位姿服务调用失败"
                
                result = {
                    "success": bool(response.success),
                    "error_code": int(response.error_code),
                    "message": str(response.message)
                }
                
                if response.success:
                    self.get_logger().info(f'移动机器人位姿成功: {response.message}')
                else:
                    self.get_logger().warn(f'移动机器人位姿失败: {response.message}')
                
                return result, None
                
            except Exception as e:
                import traceback
                self.get_logger().error(f'移动机器人位姿服务响应处理异常: {str(e)}\n{traceback.format_exc()}')
                return None, f"移动机器人位姿服务响应处理异常: {str(e)}"
                
        except Exception as e:
            import traceback
            self.get_logger().error(f'移动机器人位姿服务调用异常: {str(e)}\n{traceback.format_exc()}')
            return None, f"移动机器人位姿服务调用异常: {str(e)}"
    
    def set_robot_io(self, io_type, io_index, value, timeout=10.0):
        """设置机器人IO（使用 /demo_driver/set_io 服务）"""
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
            
            # 等待服务响应
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                return None, "设置机器人IO服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "设置机器人IO服务调用失败"
                
                result = {
                    "success": bool(response.success),
                    "error_code": int(response.error_code),
                    "message": str(response.message)
                }
                
                if response.success:
                    self.get_logger().info(f'设置机器人IO成功: {response.message}')
                else:
                    self.get_logger().warn(f'设置机器人IO失败: {response.message}')
                
                return result, None
                
            except Exception as e:
                import traceback
                self.get_logger().error(f'设置机器人IO服务响应处理异常: {str(e)}\n{traceback.format_exc()}')
                return None, f"设置机器人IO服务响应处理异常: {str(e)}"
                
        except Exception as e:
            import traceback
            self.get_logger().error(f'设置机器人IO服务调用异常: {str(e)}\n{traceback.format_exc()}')
            return None, f"设置机器人IO服务调用异常: {str(e)}"
    
    def standardize_template(self, workpiece_id, timeout=120.0):
        """调用模板标准化服务"""
        try:
            if not self.standardize_template_client.wait_for_service(timeout_sec=5.0):
                return None, "模板标准化服务未运行，请先启动visual_pose_estimation节点"
            
            request = StandardizeTemplate.Request()
            request.workpiece_id = str(workpiece_id)
            
            future = self.standardize_template_client.call_async(request)
            
            # 等待服务响应 - 使用spin_until_future_complete
            # 模板标准化可能需要较长时间，使用传入的timeout参数
            with self.executor_lock:
                self.executor_running = True
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            finally:
                with self.executor_lock:
                    self.executor_running = False
            
            if not future.done():
                return None, "模板标准化服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "模板标准化服务调用失败"
                
                # 构建模板标准化结果字典
                result = {
                    "success": bool(response.success),  # 是否成功
                    "processed_count": int(response.processed_count),  # 成功处理的姿态数量
                    "skipped_count": int(response.skipped_count),  # 跳过的姿态数量
                    "processed_pose_ids": list(response.processed_pose_ids),  # 成功处理的姿态ID列表
                    "skipped_pose_ids": list(response.skipped_pose_ids),  # 跳过的姿态ID列表
                    "error_message": str(response.error_message) if response.error_message else ""  # 错误信息（如果有）
                }
                
                if response.success:
                    self.get_logger().info(f'模板标准化成功: 处理 {response.processed_count} 个姿态, 跳过 {response.skipped_count} 个姿态')
                else:
                    self.get_logger().warn(f'模板标准化失败: {response.error_message}')
                
                return result, None
                    
            except Exception as e:
                return None, f"处理服务响应失败: {str(e)}"
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            self.get_logger().error(f'模板标准化服务异常: {str(e)}\n{error_detail}')
            return None, f"模板标准化服务异常: {str(e)}"
    
    def write_plc_register(self, address, value):
        """写入PLC寄存器（功能已禁用，WritePLCRegister服务未定义）"""
        return None, "PLC写入功能暂未实现：WritePLCRegister服务未定义"


# 全局ROS2节点实例
ros2_node = None

class AlgorithmHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        """添加CORS支持"""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()
    
    def do_OPTIONS(self):
        """处理CORS预检请求"""
        self.send_response(200)
        self.end_headers()
    
    def do_GET(self):
        if self.path.startswith('/api/get_template_image'):
            self.handle_get_template_image()
        elif self.path == '/' or self.path == '/index.html':
            # 直接提供我们的Web UI页面
            try:
                with open('index.html', 'r', encoding='utf-8') as f:
                    html_content = f.read()
                
                self.send_response(200)
                self.send_header('Content-type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html_content.encode('utf-8'))
            except FileNotFoundError:
                self.send_error(404, "File not found")
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            status = {
                "status": "online",
                "port": 8088,
                "service": "visual_pose_estimation_web_ui",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            self.wfile.write(json.dumps(status).encode('utf-8'))
        else:
            # 对于其他资源文件（如logo.png），尝试直接提供
            try:
                file_path = self.path[1:]  # 移除开头的 '/'
                if os.path.exists(file_path):
                    with open(file_path, 'rb') as f:
                        content = f.read()
                    
                    # 根据文件扩展名设置Content-Type
                    if file_path.endswith('.png'):
                        content_type = 'image/png'
                    elif file_path.endswith('.jpg') or file_path.endswith('.jpeg'):
                        content_type = 'image/jpeg'
                    elif file_path.endswith('.css'):
                        content_type = 'text/css'
                    elif file_path.endswith('.js'):
                        content_type = 'application/javascript'
                    else:
                        content_type = 'application/octet-stream'
                    
                    self.send_response(200)
                    self.send_header('Content-type', content_type)
                    self.end_headers()
                    self.wfile.write(content)
                else:
                    self.send_error(404, "File not found")
            except Exception as e:
                self.send_error(500, f"Server error: {str(e)}")

    def do_POST(self):
        # 图像捕获相关
        if self.path == '/api/capture_image':
            self.handle_capture_image()  # 捕获当前图像
        elif self.path == '/api/capture_template_image':
            self.handle_capture_template_image()  # 捕获模板图像
        
        # 机器人控制相关
        elif self.path == '/api/get_robot_status':
            self.handle_get_robot_status()  # 获取机器人状态
        elif self.path == '/api/set_robot_pose':
            self.handle_set_robot_pose()  # 设置机器人姿态
        elif self.path == '/api/set_robot_io':
            self.handle_set_robot_io()  # 设置机器人IO（夹爪控制）
        elif self.path == '/api/execute_pose_sequence':
            self.handle_execute_pose_sequence()  # 执行姿态序列
        
        # 模板管理相关
        elif self.path == '/api/save_template_pose':
            self.handle_save_template_pose()  # 保存模板姿态
        elif self.path == '/api/list_templates':
            self.handle_list_templates()  # 列出所有模板
        elif self.path == '/api/list_workpiece_ids':
            self.handle_list_workpiece_ids()  # 列出所有工件ID
        elif self.path == '/api/read_template_pose':
            self.handle_read_template_pose()  # 读取模板姿态
        elif self.path == '/api/standardize_template':
            self.handle_standardize_template()  # 标准化模板
        
        # 姿态估计相关
        elif self.path == '/api/estimate_pose':
            self.handle_estimate_pose()  # 估计工件姿态
        
        # 调试功能相关
        elif self.path == '/api/save_debug_features':
            self.handle_save_debug_features()  # 保存调试特征
        elif self.path == '/api/debug/get_images':
            self.handle_debug_get_images()  # 获取调试图像
        elif self.path == '/api/debug/update_params':
            self.handle_debug_update_params()  # 更新调试参数
        elif self.path == '/api/debug/get_params':
            self.handle_debug_get_params()  # 获取调试参数
        elif self.path == '/api/debug/save_thresholds':
            self.handle_debug_save_thresholds()  # 保存调试阈值
        elif self.path == '/api/debug/capture':
            self.handle_debug_capture()  # 调试捕获
        
        # PLC通信相关
        elif self.path == '/api/write_plc_register':
            self.handle_write_plc_register()  # 写入PLC寄存器
        
        # 系统控制相关
        elif self.path == '/exit':
            try:
                # 读取请求数据
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                
                if data.get('action') == 'exit_service':
                    # 发送响应
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"status": "success", "message": "服务正在退出..."}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    
                    # 延迟退出服务器
                    def delayed_exit():
                        time.sleep(1)
                        print("\n🛑 收到退出请求，正在关闭服务器...")
                        os._exit(0)
                    
                    # 在后台线程中执行退出
                    exit_thread = threading.Thread(target=delayed_exit)
                    exit_thread.daemon = True
                    exit_thread.start()
                else:
                    self.send_error(400, "Invalid action")
            except Exception as e:
                self.send_error(500, f"Server error: {str(e)}")
        else:
            self.send_error(404, "Not Found")
    
    def handle_capture_image(self):
        """处理图像采集请求（工作流程用，不需要保存）"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化，请重启服务器"}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            content_length = int(self.headers.get('Content-Length', 0))
            camera_id = '207000152740'  # 默认相机ID
            
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                camera_id = data.get('camera_id', '207000152740')
            
            # 触发相机拍照（获取深度图和彩色图）
            depth_image, color_image, error_msg = ros2_node.capture_image(camera_id)
            
            if depth_image is None or color_image is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": error_msg}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 将深度图编码为PNG格式的base64（保留深度信息）
            _, depth_buffer = cv2.imencode('.png', depth_image)
            depth_base64 = base64.b64encode(depth_buffer).decode('utf-8')
            
            # 将彩色图编码为JPEG格式的base64
            _, color_buffer = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            color_base64 = base64.b64encode(color_buffer).decode('utf-8')
            
            response_json = json.dumps({
                "success": True,
                "depth_image_base64": f"data:image/png;base64,{depth_base64}",
                "color_image_base64": f"data:image/jpeg;base64,{color_base64}"
            }, ensure_ascii=False)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理图像采集请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
    def handle_capture_template_image(self):
        """处理图像采集请求（模板设置用，需要保存）"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化，请重启服务器"}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": "请求体为空"}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            pose_id = data.get('pose_id', '')
            camera_id = data.get('camera_id', 'DA3234363')
            
            if not workpiece_id or not pose_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": "工件ID和姿态ID不能为空"}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 触发相机拍照（获取深度图和彩色图）
            depth_image, color_image, error_msg = ros2_node.capture_image(camera_id)
            
            if depth_image is None or color_image is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": error_msg}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 保存图像
            template_dir = Path(get_templates_dir()) / workpiece_id / f"pose_{pose_id}"
            template_dir.mkdir(parents=True, exist_ok=True)
            
            # 保存彩色图
            color_image_path = template_dir / "original_image.jpg"
            ok_color = cv2.imwrite(
                str(color_image_path),
                color_image,
                [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
            
            # 保存深度图
            depth_image_path = template_dir / "depth_image.png"
            ok_depth = cv2.imwrite(str(depth_image_path), depth_image)

            if not ok_color or not ok_depth:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                missing = []
                if not ok_color:
                    missing.append("彩色图保存失败")
                if not ok_depth:
                    missing.append("深度图保存失败")
                response = {"success": False, "error": "，".join(missing)}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 将彩色图编码为JPEG格式的base64用于返回
            _, buffer = cv2.imencode('.jpg', color_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            response_json = json.dumps({
                "success": True,
                "image_path": str(color_image_path),
                "depth_image_path": str(depth_image_path),
                "image_base64": f"data:image/jpeg;base64,{image_base64}"
            }, ensure_ascii=False)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理模板图像采集请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
    def handle_get_robot_status(self):
        """获取机器人状态"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化，请重启服务器"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            robot_status = ros2_node.get_robot_status()
            
            if robot_status is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "未收到机器人状态数据"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": True, "robot_status": robot_status}
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_save_template_pose(self):
        """保存模板姿态"""
        try:
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            pose_id = data.get('pose_id', '')
            pose_type = data.get('pose_type', '')  # camera_pose, preparation_position, grab_position
            robot_status = data.get('robot_status', None)
            
            if not workpiece_id or not pose_id or not pose_type:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "工件ID、姿态ID和姿态类型不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            if robot_status is None:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "机器人状态数据不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 确定文件名
            filename_map = {
                'camera_pose': 'camera_pose.json',
                'preparation_position': 'preparation_position.json',
                'grab_position': 'grab_position.json',
                'preplace_position': 'preplace_position.json',
                'place_position': 'place_position.json'
            }
            
            if pose_type not in filename_map:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"无效的姿态类型: {pose_type}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 规范化旋转字段，与 camera_pose.json 一致：orientation (四元数) + euler_orientation_rpy_rad + euler_orientation_rpy_deg
            robot_status = _normalize_pose_rotation(robot_status)
            # 使用固定旋转（z 垂直地面无倾斜），精确四元数 (-0.707, -0.707, 0, 0)，姿态角 (-180, 0, 90) deg
            # 原先仅对拍照姿态 camera_pose 生效；现在对所有保存的姿态类型统一使用该固定旋转，
            # 以保证准备姿态、抓取姿态、预放置、放置等在旋转上与拍照姿态一致（位置仍取机器人当前实际位置）。
            fixed = _load_app_config().get("camera_pose_fixed_orientation") or CAMERA_POSE_FIXED_ORIENTATION
            if fixed and "cartesian_position" in robot_status:
                cp = robot_status["cartesian_position"]
                cp["orientation"] = dict(fixed.get("orientation", CAMERA_POSE_FIXED_ORIENTATION["orientation"]))
                cp["euler_orientation_rpy_rad"] = list(fixed.get("euler_orientation_rpy_rad", CAMERA_POSE_FIXED_ORIENTATION["euler_orientation_rpy_rad"]))
                cp["euler_orientation_rpy_deg"] = list(fixed.get("euler_orientation_rpy_deg", CAMERA_POSE_FIXED_ORIENTATION["euler_orientation_rpy_deg"]))
            
            # 保存文件（使用配置中的唯一模板根路径）
            template_dir = Path(get_templates_dir()) / workpiece_id / f"pose_{pose_id}"
            template_dir.mkdir(parents=True, exist_ok=True)
            
            json_path = template_dir / filename_map[pose_type]
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(robot_status, f, indent=2, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "file_path": str(json_path),
                "pose_type": pose_type
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_estimate_pose(self):
        """处理姿态估计请求"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": "ROS2节点未初始化"
                }
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取请求数据
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            # 获取参数
            depth_image = data.get('depth_image', '')
            color_image = data.get('color_image', '')
            object_id = data.get('object_id', '')
            
            # 验证必需参数
            if not depth_image or not color_image:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": "缺少输入图像（需要depth_image和color_image）"
                }
                self.wfile.write(json.dumps(response).encode('utf-8'))
                self.wfile.flush()
                return
            
            if not object_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": "缺少工件ID"
                }
                self.wfile.write(json.dumps(response).encode('utf-8'))
                self.wfile.flush()
                return
            
            # 处理base64图像（移除data URL前缀如果存在）
            if ',' in depth_image:
                depth_image = depth_image.split(',')[1]
            if ',' in color_image:
                color_image = color_image.split(',')[1]
            
            # 调用ROS2服务
            print(f"[INFO] 调用姿态估计服务，工件ID: {object_id}")
            try:
                result, error = ros2_node.estimate_pose(depth_image, color_image, object_id)
                
                if error:
                    print(f"[ERROR] 姿态估计失败: {error}")
                    self.send_response(500)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {
                        "success": False,
                        "error": error
                    }
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    self.wfile.flush()
                else:
                    print(f"[INFO] 姿态估计成功，检测到 {result.get('success_num', 0)} 个目标")
                    
                    # 打印所有返回内容
                    print(f"\n{'='*80}")
                    print(f"[INFO] 姿态估计返回内容详情:")
                    print(f"{'='*80}")
                    print(f"success: {result.get('success', False)}")
                    print(f"success_num: {result.get('success_num', 0)}")
                    print(f"processing_time_sec: {result.get('processing_time_sec', None)}")
                    
                    # 打印置信度
                    confidence = result.get('confidence', [])
                    print(f"confidence: {confidence} (共 {len(confidence)} 个)")
                    
                    # 打印位置信息
                    positions = result.get('positions', [])
                    print(f"positions: {positions} (共 {len(positions)} 个)")
                    
                    # 打印抓取位置
                    grab_positions = result.get('grab_positions', [])
                    print(f"grab_positions: 共 {len(grab_positions)} 个")
                    for i, grab_pos in enumerate(grab_positions):
                        print(f"  [{i}] position: ({grab_pos.get('position', {}).get('x', 0):.3f}, "
                              f"{grab_pos.get('position', {}).get('y', 0):.3f}, "
                              f"{grab_pos.get('position', {}).get('z', 0):.3f})")
                        print(f"      orientation (quat): ({grab_pos.get('orientation', {}).get('x', 0):.3f}, "
                              f"{grab_pos.get('orientation', {}).get('y', 0):.3f}, "
                              f"{grab_pos.get('orientation', {}).get('z', 0):.3f}, "
                              f"{grab_pos.get('orientation', {}).get('w', 0):.3f})")
                        if 'euler_orientation_rpy_deg' in grab_pos:
                            euler = grab_pos['euler_orientation_rpy_deg']
                            print(f"      euler_orientation_rpy_deg: ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})")
                        if 'joint_position_deg' in grab_pos:
                            joints = grab_pos['joint_position_deg']
                            print(f"      joint_position_deg: [{', '.join(f'{j:.3f}' for j in joints)}]")
                    
                    # 打印准备位置
                    prep_positions = result.get('preparation_positions', [])
                    print(f"preparation_positions: 共 {len(prep_positions)} 个")
                    for i, prep_pos in enumerate(prep_positions):
                        print(f"  [{i}] position: ({prep_pos.get('position', {}).get('x', 0):.3f}, "
                              f"{prep_pos.get('position', {}).get('y', 0):.3f}, "
                              f"{prep_pos.get('position', {}).get('z', 0):.3f})")
                        if 'euler_orientation_rpy_deg' in prep_pos:
                            euler = prep_pos['euler_orientation_rpy_deg']
                            print(f"      euler_orientation_rpy_deg: ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})")
                    
                    # 打印预放置位置
                    preplace_positions = result.get('preplace_positions', [])
                    print(f"preplace_positions: 共 {len(preplace_positions)} 个")
                    for i, preplace_pos in enumerate(preplace_positions):
                        print(f"  [{i}] position: ({preplace_pos.get('position', {}).get('x', 0):.3f}, "
                              f"{preplace_pos.get('position', {}).get('y', 0):.3f}, "
                              f"{preplace_pos.get('position', {}).get('z', 0):.3f})")
                        print(f"      orientation (quat): ({preplace_pos.get('orientation', {}).get('x', 0):.3f}, "
                              f"{preplace_pos.get('orientation', {}).get('y', 0):.3f}, "
                              f"{preplace_pos.get('orientation', {}).get('z', 0):.3f}, "
                              f"{preplace_pos.get('orientation', {}).get('w', 0):.3f})")
                        if 'euler_orientation_rpy_deg' in preplace_pos:
                            euler = preplace_pos['euler_orientation_rpy_deg']
                            print(f"      euler_orientation_rpy_deg: ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})")
                        if 'joint_position_deg' in preplace_pos:
                            joints = preplace_pos['joint_position_deg']
                            print(f"      joint_position_deg: [{', '.join(f'{j:.3f}' for j in joints)}]")
                    
                    # 打印放置位置
                    place_positions = result.get('place_positions', [])
                    print(f"place_positions: 共 {len(place_positions)} 个")
                    for i, place_pos in enumerate(place_positions):
                        print(f"  [{i}] position: ({place_pos.get('position', {}).get('x', 0):.3f}, "
                              f"{place_pos.get('position', {}).get('y', 0):.3f}, "
                              f"{place_pos.get('position', {}).get('z', 0):.3f})")
                        print(f"      orientation (quat): ({place_pos.get('orientation', {}).get('x', 0):.3f}, "
                              f"{place_pos.get('orientation', {}).get('y', 0):.3f}, "
                              f"{place_pos.get('orientation', {}).get('z', 0):.3f}, "
                              f"{place_pos.get('orientation', {}).get('w', 0):.3f})")
                        if 'euler_orientation_rpy_deg' in place_pos:
                            euler = place_pos['euler_orientation_rpy_deg']
                            print(f"      euler_orientation_rpy_deg: ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})")
                        if 'joint_position_deg' in place_pos:
                            joints = place_pos['joint_position_deg']
                            print(f"      joint_position_deg: [{', '.join(f'{j:.3f}' for j in joints)}]")
                    
                    # 打印图像信息（只打印长度，不打印完整base64）
                    pose_images = result.get('pose_images', [])
                    print(f"pose_images: 共 {len(pose_images)} 个")
                    for i, img in enumerate(pose_images):
                        if isinstance(img, str):
                            print(f"  [{i}] base64长度: {len(img)} 字符")
                        else:
                            print(f"  [{i}] 类型: {type(img)}")
                    
                    vis_image = result.get('vis_image', '')
                    if vis_image:
                        print(f"vis_image: base64长度 {len(vis_image)} 字符")
                    else:
                        print(f"vis_image: 无")
                    
                    print(f"{'='*80}\n")
                    
                    try:
                        response_json = json.dumps(result, ensure_ascii=False)
                        print(f"[INFO] 响应数据大小: {len(response_json)} 字节")
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json; charset=utf-8')
                        self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
                        self.end_headers()
                        self.wfile.write(response_json.encode('utf-8'))
                        self.wfile.flush()  # 确保数据已发送
                    except Exception as json_error:
                        import traceback
                        error_detail = traceback.format_exc()
                        print(f"[ERROR] JSON序列化失败: {str(json_error)}\n{error_detail}")
                        self.send_response(500)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        error_response = {
                            "success": False,
                            "error": f"JSON序列化失败: {str(json_error)}"
                        }
                        self.wfile.write(json.dumps(error_response).encode('utf-8'))
                        self.wfile.flush()
            except Exception as e:
                import traceback
                error_detail = traceback.format_exc()
                print(f"[ERROR] 姿态估计调用异常: {str(e)}\n{error_detail}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": f"姿态估计调用异常: {str(e)}"
                }
                self.wfile.write(json.dumps(response).encode('utf-8'))
                self.wfile.flush()
                
        except json.JSONDecodeError as e:
            self.send_response(400)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": False,
                "error": f"JSON解析失败: {str(e)}"
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            self.wfile.flush()
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] 姿态估计处理异常: {str(e)}\n{error_detail}")
            try:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": f"服务器错误: {str(e)}"
                }
                self.wfile.write(json.dumps(response).encode('utf-8'))
                self.wfile.flush()
            except:
                # 如果响应已经发送，忽略错误
                pass
    
    def handle_list_templates(self):
        """处理列出模板请求：先调用 ROS2 /list_templates 服务获取工件列表，再按模板根目录扫描各工件的 pose 与 image.jpg"""
        try:
            workpiece_id_filter = ""
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                workpiece_id_filter = data.get('workpiece_id', '')
            
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": "ROS2 节点未初始化"}, ensure_ascii=False).encode('utf-8'))
                self.wfile.flush()
                return
            
            # 符合 ROS2：通过 /list_templates 服务获取工件 ID 列表
            result, error = ros2_node.list_templates(workpiece_id=workpiece_id_filter, timeout=10.0)
            if error is not None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": error}, ensure_ascii=False).encode('utf-8'))
                self.wfile.flush()
                return
            
            workpiece_ids = result.get("workpiece_ids", [])
            templates_root = Path(get_templates_dir())
            if not templates_root.exists() or not templates_root.is_dir():
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": f"模板根目录不存在: {templates_root}"}, ensure_ascii=False).encode('utf-8'))
                self.wfile.flush()
                return
            
            template_list = []
            for current_workpiece_id in workpiece_ids:
                workpiece_entry = templates_root / current_workpiece_id
                if not workpiece_entry.is_dir():
                    continue
                try:
                    for pose_entry in workpiece_entry.iterdir():
                        if not pose_entry.is_dir() or not pose_entry.name.startswith('pose_'):
                            continue
                        pose_id = pose_entry.name[5:]
                        image_path = pose_entry / "image.jpg"
                        if not image_path.is_file():
                            continue
                        try:
                            with open(image_path, 'rb') as f:
                                image_base64 = base64.b64encode(f.read()).decode('utf-8')
                            template_list.append({
                                "template_id": f"{current_workpiece_id}_{pose_id}",
                                "workpiece_id": current_workpiece_id,
                                "pose_id": pose_id,
                                "image_path": str(image_path),
                                "image_base64": "data:image/jpeg;base64," + image_base64
                            })
                        except Exception as e:
                            print(f"[WARN] 无法读取模板图像 {image_path}: {e}")
                except Exception as e:
                    print(f"[WARN] 遍历工件目录 {current_workpiece_id} 时出错: {e}")
            
            response = {"success": True, "templates": template_list, "count": len(template_list)}
            response_json = json.dumps(response, ensure_ascii=False)
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理列出模板请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "error": str(e)}, ensure_ascii=False).encode('utf-8'))
            self.wfile.flush()
    
    def handle_get_template_image(self):
        """处理获取模板图像请求"""
        try:
            from urllib.parse import urlparse, parse_qs
            parsed_url = urlparse(self.path)
            query_params = parse_qs(parsed_url.query)
            
            workpiece_id = query_params.get('workpiece_id', [None])[0]
            pose_id = query_params.get('pose_id', [None])[0]
            image_name = query_params.get('image_name', ['gripper_visualization.jpg'])[0]  # 默认读取gripper_visualization.jpg
            
            if not workpiece_id or not pose_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "缺少workpiece_id或pose_id参数"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 根据image_name参数读取对应的图像文件
            image_path = Path(get_templates_dir()) / workpiece_id / f"pose_{pose_id}" / image_name
            
            if not image_path.exists():
                self.send_response(404)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "图像文件不存在"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 读取图像文件
            with open(image_path, 'rb') as f:
                image_data = f.read()
            
            # 确定Content-Type
            content_type = 'image/jpeg'
            if image_path.suffix.lower() == '.png':
                content_type = 'image/png'
            
            self.send_response(200)
            self.send_header('Content-type', content_type)
            self.send_header('Content-Length', str(len(image_data)))
            self.end_headers()
            self.wfile.write(image_data)
            self.wfile.flush()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理获取模板图像请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {"error": str(e)}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
    def handle_standardize_template(self):
        """处理模板标准化请求"""
        try:
            if ros2_node is None:
                print("[ERROR] ROS2节点未初始化（standardize_template）")
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {
                    "success": False,
                    "error_message": "ROS2节点未初始化，请重启服务器"
                }
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {
                    "success": False,
                    "error_message": "请求体为空"
                }
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            
            if not workpiece_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {
                    "success": False,
                    "error_message": "工件ID不能为空"
                }
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            print(f"[INFO] 调用模板标准化服务，工件ID: {workpiece_id}")
            
            result, error = ros2_node.standardize_template(workpiece_id)
            
            if error:
                print(f"[ERROR] 模板标准化失败: {error}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {
                    "success": False,
                    "error_message": error
                }
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                self.wfile.flush()
                return
            
            # 构建响应
            response_json = json.dumps(result, ensure_ascii=False)
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON解析错误: {str(e)}")
            self.send_response(400)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {
                "success": False,
                "error_message": f"JSON解析错误: {str(e)}"
            }
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理模板标准化请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {
                "success": False,
                "error_message": f"服务器错误: {str(e)}"
            }
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
    def handle_write_plc_register(self):
        """处理PLC寄存器写入请求"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "请求体为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            address = data.get('address', '')
            value = data.get('value', None)
            
            if not address or value is None:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "address和value不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 确保value是整数类型
            try:
                value = int(value)
            except (ValueError, TypeError):
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "value必须是整数"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            result, error = ros2_node.write_plc_register(address, value)
            
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error}
                self.wfile.write(json.dumps(response).encode('utf-8'))
            else:
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode('utf-8'))
                
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_read_template_pose(self):
        """处理读取模板姿态文件请求"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "请求体为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            pose_id = data.get('pose_id', '')
            pose_type = data.get('pose_type', '')
            
            if not workpiece_id or not pose_id or not pose_type:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "workpiece_id, pose_id, pose_type不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 映射姿态类型到文件名
            filename_map = {
                'camera_pose': 'camera_pose.json',
                'preparation_position': 'preparation_position.json',
                'grab_position': 'grab_position.json',
                'preplace_position': 'preplace_position.json',
                'place_position': 'place_position.json'
            }
            
            if pose_type not in filename_map:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"不支持的姿态类型: {pose_type}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取模板文件
            # pose_id 可能已经是 "pose_1" 格式，也可能只是 "1"，需要统一处理
            if pose_id.startswith('pose_'):
                pose_dir_name = pose_id
            else:
                pose_dir_name = f"pose_{pose_id}"
            template_dir = Path(get_templates_dir()) / workpiece_id / pose_dir_name
            json_path = template_dir / filename_map[pose_type]
            
            if not json_path.exists():
                self.send_response(404)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"模板文件不存在: {json_path}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            with open(json_path, 'r', encoding='utf-8') as f:
                pose_data = json.load(f)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": True, "pose_data": pose_data}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_execute_pose_sequence(self):
        """处理执行姿态序列请求"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取请求数据
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "请求体为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            folder_name = data.get('folder_name', '')
            if not folder_name:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "folder_name不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 构建文件夹路径
            config_dir = Path(__file__).parent / "configs" / "pose_list" / folder_name
            if not config_dir.exists() or not config_dir.is_dir():
                self.send_response(404)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"文件夹不存在: {folder_name}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取所有JSON文件并按文件名排序
            json_files = sorted([f for f in config_dir.glob("*.json")])
            if not json_files:
                self.send_response(404)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"文件夹中没有找到JSON文件: {folder_name}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取所有姿态文件
            pose_files = []
            for json_file in json_files:
                try:
                    with open(json_file, 'r', encoding='utf-8') as f:
                        pose_data = json.load(f)
                        # 提取关节角度（度）
                        if 'joint_position_deg' in pose_data and len(pose_data['joint_position_deg']) == 6:
                            pose_files.append({
                                'filename': json_file.name,
                                'joint_position_deg': pose_data['joint_position_deg']
                            })
                        else:
                            print(f"[WARN] 文件 {json_file.name} 格式不正确，跳过")
                except Exception as e:
                    print(f"[WARN] 读取文件 {json_file.name} 失败: {str(e)}")
                    continue
            
            if not pose_files:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "没有有效的姿态文件"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 获取当前机器人状态
            robot_status = ros2_node.get_robot_status()
            if robot_status is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "无法获取机器人状态"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            current_joints = robot_status.get('joint_position_deg', [])
            if len(current_joints) != 6:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "机器人状态数据格式错误"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 获取最后一个点位
            last_pose = pose_files[-1]['joint_position_deg']
            
            # 比较当前关节坐标与最后一个点位（允许0.1度的误差）
            tolerance = 0.1
            is_at_last_pose = all(abs(current_joints[i] - last_pose[i]) < tolerance for i in range(6))
            
            if is_at_last_pose:
                # 已经在最后一个点位，跳过执行
                response_json = json.dumps({
                    "success": True,
                    "skipped": True,
                    "message": "当前已在最后一个点位，跳过执行",
                    "current_joints": current_joints,
                    "last_pose": last_pose,
                    "total_files": len(pose_files)
                }, ensure_ascii=False)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
                self.end_headers()
                self.wfile.write(response_json.encode('utf-8'))
                self.wfile.flush()
                return
            
            # 执行所有点位（在后台线程中执行，避免阻塞HTTP响应）
            def execute_poses():
                try:
                    executed_count = 0
                    failed_count = 0
                    
                    for i, pose_file in enumerate(pose_files):
                        target_pose = pose_file['joint_position_deg']
                        filename = pose_file['filename']
                        
                        print(f"[INFO] 执行点位 {i+1}/{len(pose_files)}: {filename}")
                        print(f"[INFO] 目标关节角度: {target_pose}")
                        
                        # 调用机器人运动服务
                        result, error = ros2_node.move_to_pose(
                            target_pose,
                            use_joints=True,
                            velocity_factor=0.08,
                            acceleration_factor=0.05,
                            timeout=60.0
                        )
                        
                        if error:
                            print(f"[ERROR] 执行点位 {filename} 失败: {error}")
                            failed_count += 1
                        else:
                            if result and result.get('success'):
                                executed_count += 1
                                print(f"[INFO] 点位 {filename} 执行成功")
                            else:
                                print(f"[ERROR] 点位 {filename} 执行失败: {result.get('message', '未知错误')}")
                                failed_count += 1
                        
                        # 等待机器人运动完成（通过in_motion状态）
                        max_wait_time = 60.0
                        wait_start = time.time()
                        while time.time() - wait_start < max_wait_time:
                            status = ros2_node.get_robot_status()
                            if status and not status.get('in_motion', True):
                                break
                            time.sleep(0.5)
                        else:
                            print(f"[WARN] 等待点位 {filename} 运动完成超时")
                    
                    print(f"[INFO] 姿态序列执行完成: 成功 {executed_count}/{len(pose_files)}, 失败 {failed_count}")
                    
                except Exception as e:
                    import traceback
                    print(f"[ERROR] 执行姿态序列异常: {str(e)}\n{traceback.format_exc()}")
            
            # 在后台线程中执行
            import threading
            execution_thread = threading.Thread(target=execute_poses, daemon=True)
            execution_thread.start()
            
            # 立即返回响应
            response_json = json.dumps({
                "success": True,
                "skipped": False,
                "message": "姿态序列执行已开始",
                "total_files": len(pose_files),
                "current_joints": current_joints,
                "last_pose": last_pose
            }, ensure_ascii=False)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理执行姿态序列请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_list_workpiece_ids(self):
        """处理获取工件ID列表请求"""
        try:
            templates_dir = Path(get_templates_dir())
            workpiece_ids = []
            
            if templates_dir.exists() and templates_dir.is_dir():
                # 获取所有子文件夹名称作为工件ID
                for item in templates_dir.iterdir():
                    if item.is_dir() and not item.name.startswith('.'):
                        workpiece_ids.append(item.name)
                
                workpiece_ids.sort()
            
            response_json = json.dumps({
                "success": True,
                "workpiece_ids": workpiece_ids,
                "count": len(workpiece_ids)
            }, ensure_ascii=False)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理获取工件ID列表请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
    def handle_set_robot_pose(self):
        """处理移动机器人位姿请求（使用 /move_to_pose 服务）"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                print("[ERROR] ROS2节点未初始化（move_to_pose）")
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "ROS2节点未初始化"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 读取请求数据
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "请求体为空"}, ensure_ascii=False).encode('utf-8'))
                return
            
            request_data = json.loads(self.rfile.read(content_length).decode('utf-8'))
            
            # 验证必需字段
            if 'target_pose' not in request_data:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "缺少target_pose字段"}, ensure_ascii=False).encode('utf-8'))
                return
            
            target_pose = request_data.get('target_pose', [])
            use_joints = request_data.get('use_joints', False)
            velocity_factor = request_data.get('velocity_factor', 0.08)
            acceleration_factor = request_data.get('acceleration_factor', 0.05)
            
            # z值不再限制，使用原始值
            
            # 支持数组格式 [x, y, z, qx, qy, qz, qw] 或 [x, y, z, qx, qy, qz]
            if isinstance(target_pose, (list, tuple)):
                if len(target_pose) < 6:
                    self.send_response(400)
                    self.send_header('Content-type', 'application/json; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "target_pose必须包含至少6个元素 [x, y, z, qx, qy, qz] 或 7个元素 [x, y, z, qx, qy, qz, qw]"}, ensure_ascii=False).encode('utf-8'))
                    return
            elif isinstance(target_pose, dict):
                # 字典格式：包含 position 和 orientation
                if 'position' not in target_pose or 'orientation' not in target_pose:
                    self.send_response(400)
                    self.send_header('Content-type', 'application/json; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "target_pose字典必须包含position和orientation字段"}, ensure_ascii=False).encode('utf-8'))
                    return
            else:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "target_pose格式错误，应为数组或字典"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 便于与 C++ 日志、前端对照：记下本次请求位姿（支持 list 与 dict）
            if isinstance(target_pose, dict) and 'position' in target_pose:
                pos = target_pose['position']
                pose_str = f"({float(pos.get('x', 0)):.3f}, {float(pos.get('y', 0)):.3f}, {float(pos.get('z', 0)):.3f})"
            elif isinstance(target_pose, (list, tuple)) and len(target_pose) >= 3:
                pose_str = f"({float(target_pose[0]):.3f}, {float(target_pose[1]):.3f}, {float(target_pose[2]):.3f})"
            else:
                pose_str = "(?, ?, ?)"
            # 调用ROS2服务
            result, error = ros2_node.move_to_pose(target_pose, use_joints, velocity_factor, acceleration_factor)
            print(f"[move_to_pose 桥接] RECV pose={pose_str} success={result.get('success') if result else None} error_code={result.get('error_code') if result else None}")
            # 诊断：记录桥接层收到的 C++ 返回值，便于与 C++ 日志、前端显示对照
            if error:
                print(f"[move_to_pose 桥接] pose={pose_str} ROS 返回 error: {error}, 将返回 HTTP 500")
            elif result is not None:
                print(f"[move_to_pose 桥接] pose={pose_str} ROS 返回 success={result.get('success')}, error_code={result.get('error_code')}, 将返回 HTTP 200")

            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": error}, ensure_ascii=False).encode('utf-8'))
                return

            response_json = json.dumps(result, ensure_ascii=False)
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON解析错误: {str(e)}")
            self.send_response(400)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"JSON解析错误: {str(e)}"}, ensure_ascii=False).encode('utf-8'))
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理移动机器人位姿请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"处理请求异常: {str(e)}"}, ensure_ascii=False).encode('utf-8'))
    
    def handle_set_robot_io(self):
        """处理设置机器人IO请求（用于夹爪控制）"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                print("[ERROR] ROS2节点未初始化（set_robot_io）")
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": "ROS2节点未初始化"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 读取请求数据
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": "请求体为空"}, ensure_ascii=False).encode('utf-8'))
                return
            
            request_data = json.loads(self.rfile.read(content_length).decode('utf-8'))
            
            # 验证必需字段
            if 'io_type' not in request_data or 'io_index' not in request_data or 'value' not in request_data:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": "缺少必需字段: io_type, io_index, value"}, ensure_ascii=False).encode('utf-8'))
                return
            
            io_type = request_data.get('io_type')
            io_index = request_data.get('io_index')
            value = request_data.get('value')
            
            # 调用ROS2服务
            result, error = ros2_node.set_robot_io(io_type, io_index, value)
            
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": error}, ensure_ascii=False).encode('utf-8'))
                return
            
            response_json = json.dumps(result, ensure_ascii=False)
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(response_json.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(response_json.encode('utf-8'))
            self.wfile.flush()
            
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON解析错误: {str(e)}")
            self.send_response(400)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "error": f"JSON解析错误: {str(e)}"}, ensure_ascii=False).encode('utf-8'))
        except Exception as e:
            import traceback
            print(f"[ERROR] 处理设置机器人IO请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "error": f"处理请求异常: {str(e)}"}, ensure_ascii=False).encode('utf-8'))
    
    def handle_save_debug_features(self):
        """保存调试特征结果到CSV"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "请求体为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            session_id = data.get('session_id', '')
            features = data.get('features', [])
            
            if not session_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "会话ID为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 创建CSV文件
            debug_dir = Path(f"debug/{session_id}")
            debug_dir.mkdir(parents=True, exist_ok=True)
            
            csv_path = debug_dir / "features.csv"
            
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入表头
                writer.writerow(['ID', 'ub', 'vb', 'rb', 'us', 'vs', 'rs', 'theta(rad)', 'theta(deg)', 'area'])
                # 写入数据
                for feature in features:
                    writer.writerow([
                        feature.get('id', ''),
                        feature.get('ub', 0),
                        feature.get('vb', 0),
                        feature.get('rb', 0),
                        feature.get('us', 0),
                        feature.get('vs', 0),
                        feature.get('rs', 0),
                        feature.get('theta', 0),
                        feature.get('theta_deg', 0),
                        feature.get('area', 0)
                    ])
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "file_path": str(csv_path),
                "feature_count": len(features)
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    # ========== Debug API 处理函数 ==========
    
    def handle_debug_capture(self):
        """处理Debug采集图像请求"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
            else:
                data = {}
            
            camera_id = data.get('camera_id', '207000152740')
            
            # 清空之前的图像
            with ros2_node.image_lock:
                ros2_node.latest_depth_image = None
                ros2_node.latest_color_image = None
                ros2_node.depth_image_received = False
                ros2_node.color_image_received = False
            
            # 触发相机采集
            depth_img, color_img, error_msg = ros2_node.capture_image(camera_id=camera_id, timeout=10.0)
            
            if error_msg:
                # 记录最近一次采集错误，供 get_images 返回更明确的提示
                try:
                    ros2_node.debug_last_capture_error = error_msg
                except Exception:
                    pass
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error_msg}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return

            # 成功则清空最近一次错误
            try:
                ros2_node.debug_last_capture_error = None
            except Exception:
                pass
            
            # 保存到实例变量供后续使用
            if not hasattr(self, 'debug_latest_depth'):
                self.debug_latest_depth = None
            if not hasattr(self, 'debug_latest_color'):
                self.debug_latest_color = None
            
            self.debug_latest_depth = depth_img
            self.debug_latest_color = color_img
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": True, "message": "图像采集成功"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_debug_get_images(self):
        """获取Debug图像（深度图、彩色图、二值化图、预处理图）"""
        try:
            # 读取请求体（即使为空也要读取）
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
            
            # 检查是否有最新图像
            with ros2_node.image_lock:
                if ros2_node.latest_depth_image is None or ros2_node.latest_color_image is None:
                    last_err = getattr(ros2_node, "debug_last_capture_error", None)
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {
                        "success": True,
                        "has_images": False,
                        "message": last_err if last_err else "等待图像..."
                    }
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    return
                
                depth_image = ros2_node.latest_depth_image.copy()
                color_image = ros2_node.latest_color_image.copy()
            
            # 加载参数（统一使用config_reader获取阈值参数）
            params = ros2_node.config_reader.load_debug_thresholds()
            
            # 应用参数到算法模块（确保连通域/轮廓参数生效）
            if ros2_node.preprocessor:
                ros2_node.preprocessor.set_parameters({
                    "binary_threshold_min": params.get("binary_threshold_min"),
                    "binary_threshold_max": params.get("binary_threshold_max"),
                    "component_min_area": params.get("component_min_area"),
                    "component_max_area": params.get("component_max_area"),
                    "component_min_aspect_ratio": params.get("component_min_aspect_ratio"),
                    "component_max_aspect_ratio": params.get("component_max_aspect_ratio"),
                    "component_min_width": params.get("component_min_width"),
                    "component_min_height": params.get("component_min_height"),
                    "component_max_count": params.get("component_max_count"),
                    "enable_zero_interp": params.get("enable_zero_interp"),
                    "enable_smooth_edges": params.get("enable_smooth_edges", True),
                    "smooth_edges_blur_sigma": params.get("smooth_edges_blur_sigma", 0),
                })
            if ros2_node.feature_extractor:
                ros2_node.feature_extractor.set_parameters({
                    "component_min_area": params.get("component_min_area"),
                    "component_max_area": params.get("component_max_area"),
                    "component_min_aspect_ratio": params.get("component_min_aspect_ratio"),
                    "component_max_aspect_ratio": params.get("component_max_aspect_ratio"),
                    "component_min_width": params.get("component_min_width"),
                    "component_min_height": params.get("component_min_height"),
                })
            
            # 如果尺寸不匹配，调整彩色图像尺寸
            if depth_image.shape[:2] != color_image.shape[:2]:
                print(f"[INFO] 调整彩色图尺寸: {color_image.shape[:2]} -> {depth_image.shape[:2]}")
                color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]))
            
            # ========================================
            # 使用算法模块处理（Debug参数直接应用）
            # ========================================
            
            binary_threshold_min = int(params.get('binary_threshold_min', 0))
            binary_threshold_max = int(params.get('binary_threshold_max', 65535))
            
            print(f"[HTTP桥接] 调用算法模块，阈值: [{binary_threshold_min}, {binary_threshold_max}]")
            
            # 使用本地算法模块（HTTP桥接中的ROS2Node实例）
            if ros2_node.preprocessor is None:
                print("[ERROR] 算法模块未初始化")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "算法模块未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 使用算法模块处理
            components, preprocessed_color = ros2_node.preprocessor.preprocess(
                depth_image, color_image, binary_threshold_min, binary_threshold_max
            )
            features_objs = ros2_node.feature_extractor.extract_features(components, preprocessed_color)
            
            # 转换特征为字典
            features = []
            for feat in features_objs:
                features.append({
                    'workpiece_center': list(feat.workpiece_center),
                    'workpiece_radius': float(feat.workpiece_radius),
                    'valve_center': list(feat.valve_center),
                    'valve_radius': float(feat.valve_radius),
                    'angle_deg': float(feat.standardized_angle_deg)
                })

            use_rembg = bool(params.get("use_rembg", False)) and params.get("use_rembg") != 0
            if use_rembg and REMBG_AVAILABLE and preprocessed_color is not None:
                try:
                    processor = _get_rembg_processor()
                    if processor is not None:
                        bbox = None
                        if features:
                            wp_center = features[0].get("workpiece_center")
                            wp_radius = features[0].get("workpiece_radius")
                            if wp_center and wp_radius:
                                cx, cy = wp_center
                                radius = wp_radius
                                bbox = (
                                    int(round(cx - radius)),
                                    int(round(cy - radius)),
                                    int(round(radius * 2)),
                                    int(round(radius * 2)),
                                )
                        if bbox is None and components:
                            ys, xs = np.where(components[0] > 0)
                            if ys.size and xs.size:
                                x0 = int(xs.min())
                                x1 = int(xs.max())
                                y0 = int(ys.min())
                                y1 = int(ys.max())
                                bbox = (x0, y0, x1 - x0 + 1, y1 - y0 + 1)
                        if bbox is not None:
                            rembg_mask, rembg_cutout = processor.process_roi(color_image, bbox)
                            if rembg_cutout is not None:
                                preprocessed_color = rembg_cutout
                                # 同时替换 components[0] 中的掩模
                                if rembg_mask is not None and len(components) > 0:
                                    components[0] = rembg_mask
                                print(
                                    f"[HTTP桥接] Rembg启用，预处理图像和掩模切换为Rembg输出 (providers={processor.providers})"
                                )
                            else:
                        else:
                except Exception as e:
                    import traceback
                    tb = traceback.format_exc()
                    print(f"[HTTP桥接] Rembg处理失败: {e}")
                    print(f"[HTTP桥接] Traceback: {tb}")
            else:
            
            # 使用可视化器生成调试图像
            from visual_pose_estimation_python.debug_visualizer import DebugVisualizer
            visualizer = DebugVisualizer()
            debug_panel = visualizer.create_debug_panel(
                depth_image, color_image, components, features,
                binary_threshold_min, binary_threshold_max, preprocessed_color
            )
            
            # 从debug_panel提取各图像
            depth_display = debug_panel.get('depth_display')
            color_display = debug_panel.get('color_display')
            binary_display = debug_panel.get('binary_display')
            preprocessed_display = debug_panel.get('preprocessed_display')
            
            # 统计信息
            stats = {
                'component_count': len(components),
                'feature_count': len(features),
                'algorithm_source': 'algorithm_module'
            }
            
            print(f"[HTTP桥接] 算法处理完成: {len(components)} 个连通域, {len(features)} 个特征")
            
            # 转换图像为base64
            def encode_image(img, tag: str):
                if img is None:
                    return None
                try:
                    _, buffer = cv2.imencode('.jpg', img)
                    return "data:image/jpeg;base64," + base64.b64encode(buffer).decode('utf-8')
                except Exception as e:
                    print(f"[ERROR] encode_image {tag} failed: {e}")
                    return None
            
            response = {
                "success": True,
                "has_images": True,
                "depth_image": encode_image(depth_display, "depth_display"),
                "color_image": encode_image(color_display, "color_display"),
                "binary_image": encode_image(binary_display, "binary_display"),
                "preprocessed_image": encode_image(preprocessed_display, "preprocessed"),
                "stats": stats,
                "features": features,
                "algorithm_info": {
                    "source": "algorithm_module",
                    "params_applied": "Debug参数已直接应用到算法",
                    "note": "此预览使用与 /estimate_pose 完全相同的算法流程"
                }
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_debug_update_params(self):
        try:
            data = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))).decode())
            # 参数名映射（前端字段 -> 参数文件字段，使用下划线格式）
            raw_key = data.get('param_name')
            value = data.get('param_value')
            
            # 将前端发送的连字符格式转换为下划线格式（如 'min-depth' -> 'min_depth'）
            normalized_key = raw_key.replace('-', '_') if raw_key else raw_key
            
            # 前端字段 -> 参数文件字段（下划线格式）
            key_map = {
                'min_depth': 'binary_threshold_min',
                'max_depth': 'binary_threshold_max',
                'contour_min_area': 'component_min_area',
                'contour_max_area': 'component_max_area',
                'min_aspect': 'component_min_aspect_ratio',
                'max_aspect': 'component_max_aspect_ratio',
                'min_width': 'component_min_width',
                'min_height': 'component_min_height',
                'max_count': 'component_max_count',
                'enable_smooth_edges': 'enable_smooth_edges',
                'smooth_edges_blur_sigma': 'smooth_edges_blur_sigma',
                'use_rembg': 'use_rembg',
            }
            target_key = key_map.get(normalized_key, normalized_key)
            
            # 保存到共享配置文件
            ros2_node.params_manager.update(target_key, value)
            
            # 通知ROS2节点重新加载文件
            ros2_node.notify_params_updated()
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True}).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "error": str(e)}).encode())
    
    def handle_debug_get_params(self):
        try:
            if int(self.headers.get('Content-Length', 0)) > 0:
                self.rfile.read(int(self.headers['Content-Length']))
            
            # 统一使用config_reader获取阈值参数
            params = ros2_node.config_reader.load_debug_thresholds()
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True, "params": params}).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": False, "error": str(e)}).encode())
    
    def handle_debug_save_thresholds(self):
        """保存Debug阈值到共享配置文件（统一保存接口）
        
        确保当前所有参数都保存到配置文件
        """
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                self.rfile.read(content_length)
            
            # 统一保存：确保缓存中的参数都保存到文件
            success = ros2_node.params_manager.save()
            
            if not success:
                raise Exception("保存配置文件失败")
            
            # 获取保存的参数（统一使用config_reader重新加载，确保读取的是最新保存的值）
            params = ros2_node.config_reader.load_debug_thresholds()
            
            # 通知ROS2节点重新加载
            ros2_node.notify_params_updated()
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "message": f"阈值已保存到配置文件: {ros2_node.params_manager.config_path} 并同步到算法模块"
            }
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
    
def start_server():
    global ros2_node
    
    # 诊断信息：显示 Python 环境和模块可用性
    print(f"\n{'='*60}")
    print(f"🔍 环境诊断信息")
    print(f"{'='*60}")
    print(f"Python 可执行文件: {sys.executable}")
    print(f"Python 版本: {sys.version.split()[0]}")
    print(f"PYTHONPATH: {os.environ.get('PYTHONPATH', '未设置')}")
    print(f"算法模块可用: {ALGORITHM_AVAILABLE}")
    print(f"RemBG 模块可用: {REMBG_AVAILABLE}")
    if REMBG_AVAILABLE:
        if RemBGProcessor is not None:
            print(f"RemBG 模式: 直接导入")
            try:
                import onnxruntime as ort
                providers = ort.get_available_providers()
                print(f"ONNX Runtime Providers: {providers}")
            except Exception as e:
                print(f"ONNX Runtime 检查失败: {e}")
        elif SUBPROCESS_REMBG_AVAILABLE:
            conda_python = "/home/mu/miniconda3/envs/ros2_env/bin/python"
            print(f"RemBG 模式: subprocess (conda 环境)")
            print(f"Conda Python: {conda_python}")
            # 测试 conda 环境
            try:
                import subprocess
                test_result = subprocess.run(
                    [conda_python, "-c", "import onnxruntime; print(','.join(onnxruntime.get_available_providers()))"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if test_result.returncode == 0:
                    providers = test_result.stdout.strip().split(',')
                    print(f"ONNX Runtime Providers (conda): {providers}")
                else:
                    print(f"Conda 环境检查失败: {test_result.stderr}")
            except Exception as e:
                print(f"Conda 环境检查异常: {e}")
    print(f"{'='*60}\n")
    
    # 初始化ROS2
    if not rclpy.ok():
        rclpy.init()
    
    # 创建ROS2节点
    ros2_node = ROS2Node()
    
    # 在后台线程中运行spin，持续处理ROS2消息
    # 使用锁确保不会与capture_image中的spin冲突
    def ros2_spin():
        try:
            while rclpy.ok():
                with ros2_node.executor_lock:
                    if not ros2_node.executor_running:
                        rclpy.spin_once(ros2_node, timeout_sec=0.1)
                    else:
                        # 如果正在执行，等待一下
                        time.sleep(0.05)
        except Exception as e:
            print(f"ROS2执行器错误: {e}")
    
    ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
    ros2_thread.start()
    
    PORT = 8088
    # 设置SO_REUSEADDR选项，允许端口重用
    class ReusableTCPServer(socketserver.TCPServer):
        allow_reuse_address = True
    
    with ReusableTCPServer(("", PORT), AlgorithmHandler) as httpd:
        print(f"🎯 视觉姿态估计算法Web UI服务器启动成功!")
        print(f"🌐 服务地址: http://localhost:{PORT}/")
        print(f"📱 Web界面: http://localhost:{PORT}/index.html")
        print(f"🐍 Python 环境: {sys.executable}")
        print(f"📦 RemBG 可用: {REMBG_AVAILABLE}")
        if REMBG_AVAILABLE:
            print(f"✓ RemBG GPU 功能已启用")
        else:
            print(f"⚠ RemBG 不可用，请检查 Python 环境是否包含 rembg 和 onnxruntime")
        print(f"⚡ 按 Ctrl+C 停止服务器")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 服务器已停止")
            if rclpy.ok():
                ros2_node.destroy_node()
                rclpy.shutdown()
            sys.exit(0)

if __name__ == "__main__":
    start_server()
