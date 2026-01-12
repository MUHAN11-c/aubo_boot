#!/usr/bin/env python3
"""
HTTP桥接服务器
将Web UI的HTTP请求转换为ROS2服务调用
"""

import http.server
import socketserver
import threading
import json
import base64
import time
import sys
import os
import csv
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from interface.srv import SoftwareTrigger, EstimatePose, ProcessDebugStep, ListTemplates, VisualizeGraspPose, StandardizeTemplate, WritePLCRegister, SetRobotPose
from interface.msg import RobotStatus, ImageData
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class ROS2BridgeNode(Node):
    """ROS2桥接节点，用于与ROS2系统通信"""
    def __init__(self):
        super().__init__('http_bridge_node')
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/camera_0/software_trigger')
        self.estimate_pose_client = self.create_client(EstimatePose, '/estimate_pose')
        self.debug_step_client = self.create_client(ProcessDebugStep, '/process_debug_step')
        self.list_templates_client = self.create_client(ListTemplates, '/list_templates')
        self.visualize_grasp_pose_client = self.create_client(VisualizeGraspPose, '/visualize_grasp_pose')
        self.standardize_template_client = self.create_client(StandardizeTemplate, '/standardize_template')
        self.write_plc_register_client = self.create_client(WritePLCRegister, '/write_plc_register')
        self.set_robot_pose_client = self.create_client(SetRobotPose, '/set_robot_pose')
        
        # 创建订阅者
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        
        self.image_data_sub = self.create_subscription(
            ImageData,
            '/camera_0/image_data',
            self.image_data_callback,
            10
        )
        
        # 状态变量
        self.latest_robot_status = None
        self.latest_image_data = None
        self.image_received = False
        self.robot_status_lock = threading.Lock()
        self.image_lock = threading.Lock()
        
        self.get_logger().info('ROS2桥接节点初始化完成')
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        with self.robot_status_lock:
            self.latest_robot_status = msg
    
    def image_data_callback(self, msg):
        """图像数据回调"""
        with self.image_lock:
            self.latest_image_data = msg
            self.image_received = True
    
    def get_robot_status(self):
        """获取最新的机器人状态"""
        # 执行一次spin_once以确保接收到最新的机器人状态消息
        rclpy.spin_once(self, timeout_sec=0.1)
        
        with self.robot_status_lock:
            if self.latest_robot_status is None:
                return None
            
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
                    "euler_orientation_rpy_deg": list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_deg),
                    "euler_orientation_rpy_rad": list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_rad)
                }
            }
            return status_dict
    
    def capture_image(self, camera_id='DA3234363', timeout=10.0):
        """触发相机拍照并获取图像"""
        try:
            if not self.trigger_client.wait_for_service(timeout_sec=2.0):
                return None, "相机服务未运行"
            
            with self.image_lock:
                self.image_received = False
                self.latest_image_data = None
            
            request = SoftwareTrigger.Request()
            request.camera_id = camera_id
            
            future = self.trigger_client.call_async(request)
            
            # 等待服务响应，期间需要执行spin_once以处理回调
            start_time = time.time()
            while time.time() - start_time < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "相机服务调用超时"
            
            try:
                response = future.result()
                if response is None or not response.success:
                    return None, f"相机触发失败: {response.message if response else 'Unknown'}"
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 执行ROS2 spin_once以处理图像订阅回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
                with self.image_lock:
                    if self.image_received and self.latest_image_data is not None:
                        try:
                            img_msg = self.latest_image_data.image
                            encoding = img_msg.encoding.lower().strip()
                            
                            is_compressed = False
                            if 'jpeg' in encoding or 'jpg' in encoding or encoding == 'jipeg':
                                is_compressed = True
                                np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if cv_image is None:
                                    return None, "JPEG图像解码失败"
                            elif 'png' in encoding:
                                is_compressed = True
                                np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if cv_image is None:
                                    return None, "PNG图像解码失败"
                            
                            if not is_compressed:
                                try:
                                    cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                                except Exception as bridge_error:
                                    self.get_logger().warn(f'cv_bridge转换失败: {bridge_error}, 尝试作为JPEG解码')
                                    np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                    if cv_image is None:
                                        return None, f"图像解码失败: {bridge_error}"
                            
                            if cv_image is None or cv_image.size == 0:
                                return None, "图像解码后为空"
                            
                            return cv_image, None
                        except Exception as e:
                            return None, f"图像转换失败: {str(e)}"
                
                time.sleep(0.05)
            
            return None, "图像接收超时"
            
        except Exception as e:
            return None, f"相机触发异常: {str(e)}"
    
    def estimate_pose(self, image_msg, object_id=""):
        """调用姿态估计服务"""
        try:
            if not self.estimate_pose_client.wait_for_service(timeout_sec=2.0):
                return None, "姿态估计服务未运行"
            
            request = EstimatePose.Request()
            request.image = image_msg
            request.object_id = object_id
            
            future = self.estimate_pose_client.call_async(request)
            
            # 在等待期间执行ROS2 spin_once以处理服务响应
            start_time = time.time()
            while time.time() - start_time < 30.0:  # 姿态估计可能需要更长时间
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "姿态估计服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "姿态估计服务调用失败"
                
                return response, None
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"姿态估计服务异常: {str(e)}"
    
    def list_templates(self, templates_dir=""):
        """调用列出模板服务"""
        try:
            if not self.list_templates_client.wait_for_service(timeout_sec=2.0):
                return None, "列出模板服务未运行"
            
            request = ListTemplates.Request()
            request.templates_dir = templates_dir
            
            future = self.list_templates_client.call_async(request)
            
            # 在等待期间执行ROS2 spin_once以处理服务响应
            start_time = time.time()
            while time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "列出模板服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "列出模板服务调用失败"
                
                if response.success:
                    return {
                        "template_paths": list(response.template_paths),
                        "template_ids": list(response.template_ids),
                        "workpiece_ids": list(response.workpiece_ids),
                        "pose_ids": list(response.pose_ids)
                    }, None
                else:
                    return None, response.error_message
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"调用列出模板服务异常: {str(e)}"
    
    def visualize_grasp_pose(self, template_image_path, workpiece_id, pose_id):
        """调用可视化抓取姿态服务"""
        try:
            if not self.visualize_grasp_pose_client.wait_for_service(timeout_sec=2.0):
                return None, "可视化抓取姿态服务未运行"
            
            request = VisualizeGraspPose.Request()
            request.template_image_path = template_image_path
            request.workpiece_id = workpiece_id
            request.pose_id = pose_id
            
            future = self.visualize_grasp_pose_client.call_async(request)
            
            # 在等待期间执行ROS2 spin_once以处理服务响应
            start_time = time.time()
            while time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "可视化抓取姿态服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "可视化抓取姿态服务调用失败"
                
                if response.success:
                    # 转换ROS2图像消息为OpenCV图像
                    cv_image = self.bridge.imgmsg_to_cv2(response.visualized_image, desired_encoding='bgr8')
                    # 编码为base64
                    _, buffer = cv2.imencode('.jpg', cv_image)
                    image_base64 = base64.b64encode(buffer).decode('utf-8')
                    return {
                        "image_base64": f"data:image/jpeg;base64,{image_base64}"
                    }, None
                else:
                    return None, response.error_message
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"调用可视化抓取姿态服务异常: {str(e)}"
    
    def standardize_template(self, workpiece_id):
        """调用模板标准化服务"""
        try:
            if not self.standardize_template_client.wait_for_service(timeout_sec=2.0):
                return None, "模板标准化服务未运行"
            
            request = StandardizeTemplate.Request()
            request.workpiece_id = workpiece_id
            
            future = self.standardize_template_client.call_async(request)
            
            # 模板标准化可能需要较长时间，设置更长的超时时间
            start_time = time.time()
            while time.time() - start_time < 120.0:  # 2分钟超时
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "模板标准化服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "模板标准化服务调用失败"
                
                if response.success:
                    return {
                        "success": True,
                        "processed_count": response.processed_count,
                        "skipped_count": response.skipped_count,
                        "processed_pose_ids": list(response.processed_pose_ids),
                        "skipped_pose_ids": list(response.skipped_pose_ids),
                        "error_message": ""
                    }, None
                else:
                    return None, response.error_message
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"调用模板标准化服务异常: {str(e)}"
    
    def write_plc_register(self, address, value):
        """写入PLC寄存器"""
        try:
            if not self.write_plc_register_client.wait_for_service(timeout_sec=2.0):
                return None, "PLC写入服务未运行"
            
            request = WritePLCRegister.Request()
            request.address = address
            request.value = value
            
            future = self.write_plc_register_client.call_async(request)
            
            # 等待服务响应
            start_time = time.time()
            while time.time() - start_time < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "PLC写入服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "PLC写入服务调用失败"
                
                if response.success:
                    return {"success": True, "error_code": response.error_code}, None
                else:
                    return None, f"PLC写入失败，错误代码: {response.error_code}"
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"PLC写入异常: {str(e)}"
    
    def set_robot_pose(self, target_pose, use_joints, is_radian, velocity=30.0):
        """调用机器人位姿设置服务"""
        try:
            if not self.set_robot_pose_client.wait_for_service(timeout_sec=2.0):
                return None, "机器人位姿设置服务未运行"
            
            request = SetRobotPose.Request()
            request.target_pose = target_pose
            request.use_joints = use_joints
            request.is_radian = is_radian
            request.velocity = float(velocity)
            
            future = self.set_robot_pose_client.call_async(request)
            
            start_time = time.time()
            while time.time() - start_time < 30.0:  # 机器人运动可能需要较长时间
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "机器人位姿设置服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "机器人位姿设置服务调用失败"
                
                return {
                    "success": response.success,
                    "error_code": response.error_code,
                    "message": response.message
                }, None
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"调用机器人位姿设置服务异常: {str(e)}"
    
    def read_template_pose_file(self, workpiece_id, pose_id, pose_type):
        """读取模板姿态文件
        
        Args:
            workpiece_id: 工件ID
            pose_id: 姿态ID
            pose_type: 姿态类型 ('camera_pose', 'preplace_position', 'place_position')
        
        Returns:
            (pose_data, error): 姿态数据字典或错误信息
        """
        try:
            # 映射姿态类型到文件名
            filename_map = {
                'camera_pose': 'camera_pose.json',
                'preplace_position': 'preplace_position.json',
                'place_position': 'place_position.json'
            }
            
            if pose_type not in filename_map:
                return None, f"不支持的姿态类型: {pose_type}"
            
            # 读取模板文件
            template_dir = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}"
            json_path = template_dir / filename_map[pose_type]
            
            if not json_path.exists():
                return None, f"模板文件不存在: {json_path}"
            
            with open(json_path, 'r', encoding='utf-8') as f:
                pose_data = json.load(f)
            
            return pose_data, None
            
        except Exception as e:
            return None, f"读取模板文件失败: {str(e)}"
    
    def process_debug_step(self, image_msg, step_id, step_index, session_id, is_redo):
        """调用调试步骤处理服务"""
        try:
            if not self.debug_step_client.wait_for_service(timeout_sec=2.0):
                return None, "调试步骤处理服务未运行"
            
            request = ProcessDebugStep.Request()
            request.input_image = image_msg
            request.step_id = step_id
            request.step_index = step_index
            request.session_id = session_id
            request.is_redo = is_redo
            
            future = self.debug_step_client.call_async(request)
            
            # 在等待期间执行ROS2 spin_once以处理服务响应
            start_time = time.time()
            while time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    break
            
            if not future.done():
                return None, "调试步骤处理服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "调试步骤处理服务调用失败"
                
                return response, None
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
        except Exception as e:
            return None, f"调试步骤处理服务异常: {str(e)}"


# 全局ROS2节点实例
ros2_node = None

class HTTPBridgeHandler(http.server.SimpleHTTPRequestHandler):
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
        if self.path == '/' or self.path == '/index.html':
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
                "service": "visual_pose_estimation_http_bridge",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            self.wfile.write(json.dumps(status).encode('utf-8'))
        else:
            try:
                file_path = self.path[1:]
                if os.path.exists(file_path):
                    with open(file_path, 'rb') as f:
                        content = f.read()
                    
                    content_type = 'application/octet-stream'
                    if file_path.endswith('.png'):
                        content_type = 'image/png'
                    elif file_path.endswith('.jpg') or file_path.endswith('.jpeg'):
                        content_type = 'image/jpeg'
                    elif file_path.endswith('.css'):
                        content_type = 'text/css'
                    elif file_path.endswith('.js'):
                        content_type = 'application/javascript'
                    
                    self.send_response(200)
                    self.send_header('Content-type', content_type)
                    self.end_headers()
                    self.wfile.write(content)
                else:
                    self.send_error(404, "File not found")
            except Exception as e:
                self.send_error(500, f"Server error: {str(e)}")
    
    def do_POST(self):
        if self.path == '/api/capture_image':
            self.handle_capture_image()
        elif self.path == '/api/capture_template_image':
            self.handle_capture_template_image()
        elif self.path == '/api/get_robot_status':
            self.handle_get_robot_status()
        elif self.path == '/api/estimate_pose':
            self.handle_estimate_pose()
        elif self.path == '/api/process_debug_step':
            self.handle_process_debug_step()
        elif self.path == '/api/save_debug_features':
            self.handle_save_debug_features()
        elif self.path == '/api/save_template_pose':
            self.handle_save_template_pose()
        elif self.path == '/api/list_templates':
            self.handle_list_templates()
        elif self.path == '/api/visualize_grasp_pose':
            self.handle_visualize_grasp_pose()
        elif self.path == '/api/standardize_template':
            self.handle_standardize_template()
        elif self.path == '/api/write_plc_register':
            self.handle_write_plc_register()
        elif self.path == '/api/set_robot_pose':
            self.handle_set_robot_pose()
        elif self.path == '/api/read_template_pose':
            self.handle_read_template_pose()
        elif self.path == '/exit':
            self.handle_exit()
        else:
            self.send_error(404, "API endpoint not found")
    
    def handle_capture_template_image(self):
        """处理图像采集请求"""
        try:
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            # 获取请求参数
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
            camera_id = data.get('camera_id', 'DA3234363')
            
            if not workpiece_id or not pose_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "工件ID和姿态ID不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            cv_image, error = ros2_node.capture_image(camera_id)
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 保存图像到对应的姿态文件夹
            template_dir = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}"
            template_dir.mkdir(parents=True, exist_ok=True)
            
            image_path = template_dir / "original_image.jpg"
            success = cv2.imwrite(str(image_path), cv_image)
            
            if not success:
                self.get_logger().warn(f"保存图像失败: {image_path}")
            
            # 编码图像为base64
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "image_path": str(image_path),
                "image": f"data:image/jpeg;base64,{image_base64}",
                "image_base64": f"data:image/jpeg;base64,{image_base64}"
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f"处理图像采集请求异常: {str(e)}")
            self.send_error(500, f"处理请求异常: {str(e)}")
    
    def handle_capture_image(self):
        """处理图像采集请求（触发相机拍照）"""
        try:
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            # 获取请求参数
            content_length = int(self.headers.get('Content-Length', 0))
            camera_id = 'DA3234363'
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                camera_id = data.get('camera_id', 'DA3234363')
            
            # 调用 SoftwareTrigger 服务触发相机拍照
            cv_image, error = ros2_node.capture_image(camera_id)
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 编码图像为base64（JPEG格式）
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "image": f"data:image/jpeg;base64,{image_base64}",
                "image_base64": f"data:image/jpeg;base64,{image_base64}"
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_error(500, f"处理请求异常: {str(e)}")
    
    def handle_get_robot_status(self):
        """获取机器人状态"""
        try:
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            status = ros2_node.get_robot_status()
            if status is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "无法获取机器人状态"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": True, "robot_status": status}
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_error(500, f"处理请求异常: {str(e)}")
    
    def handle_estimate_pose(self):
        """处理姿态估计请求"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "请求体为空")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            input_image = data.get('input_image', '')
            if not input_image:
                self.send_error(400, "输入图像为空")
                return
            
            # 解码base64图像
            if ',' in input_image:
                image_base64 = input_image.split(',')[1]
            else:
                image_base64 = input_image
            
            image_bytes = base64.b64decode(image_base64)
            np_arr = np.frombuffer(image_bytes, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None or cv_image.size == 0:
                self.send_error(400, "图像解码失败")
                return
            
            # 获取工件ID
            object_id = data.get('object_id', '')
            if not object_id:
                self.send_error(400, "工件ID不能为空")
                return
            
            # 转换为ROS2图像消息
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # 调用ROS2服务
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            response_ros, error = ros2_node.estimate_pose(image_msg, object_id)
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response_json = {"success": False, "error": error}
                self.wfile.write(json.dumps(response_json).encode('utf-8'))
                return
            
            # 转换响应为JSON
            # 转换可视化图像
            vis_image_base64 = ""
            if response_ros.vis_image.data:
                try:
                    cv_vis_image = bridge.imgmsg_to_cv2(response_ros.vis_image, desired_encoding='bgr8')
                    _, vis_buffer = cv2.imencode('.jpg', cv_vis_image)
                    vis_image_base64 = base64.b64encode(vis_buffer).decode('utf-8')
                    vis_image_base64 = f"data:image/jpeg;base64,{vis_image_base64}"
                except Exception as e:
                    self.get_logger().warn(f'可视化图像转换失败: {e}')
            
            # 转换位姿图像数组
            pose_images_base64 = []
            for pose_img_msg in response_ros.pose_image:
                try:
                    cv_pose_image = bridge.imgmsg_to_cv2(pose_img_msg, desired_encoding='bgr8')
                    _, pose_buffer = cv2.imencode('.jpg', cv_pose_image)
                    pose_image_base64 = base64.b64encode(pose_buffer).decode('utf-8')
                    pose_images_base64.append(f"data:image/jpeg;base64,{pose_image_base64}")
                except Exception as e:
                    self.get_logger().warn(f'位姿图像转换失败: {e}')
            
            # 转换位置和姿态
            positions = []
            for pos in response_ros.position:
                positions.append({"x": pos.x, "y": pos.y, "z": pos.z})
            
            preparation_positions = []
            for prep_pos in response_ros.preparation_position:
                preparation_positions.append({
                    "position": {"x": prep_pos.position.x, "y": prep_pos.position.y, "z": prep_pos.position.z},
                    "orientation": {"x": prep_pos.orientation.x, "y": prep_pos.orientation.y, 
                                  "z": prep_pos.orientation.z, "w": prep_pos.orientation.w},
                    "euler_orientation_rpy_deg": list(prep_pos.euler_orientation_rpy_deg) if hasattr(prep_pos, 'euler_orientation_rpy_deg') else [0.0, 0.0, 0.0],
                    "euler_orientation_rpy_rad": list(prep_pos.euler_orientation_rpy_rad) if hasattr(prep_pos, 'euler_orientation_rpy_rad') else [0.0, 0.0, 0.0]
                })
            
            grab_positions = []
            for grab_pos in response_ros.grab_position:
                grab_positions.append({
                    "position": {"x": grab_pos.position.x, "y": grab_pos.position.y, "z": grab_pos.position.z},
                    "orientation": {"x": grab_pos.orientation.x, "y": grab_pos.orientation.y, 
                                  "z": grab_pos.orientation.z, "w": grab_pos.orientation.w},
                    "euler_orientation_rpy_deg": list(grab_pos.euler_orientation_rpy_deg) if hasattr(grab_pos, 'euler_orientation_rpy_deg') else [0.0, 0.0, 0.0],
                    "euler_orientation_rpy_rad": list(grab_pos.euler_orientation_rpy_rad) if hasattr(grab_pos, 'euler_orientation_rpy_rad') else [0.0, 0.0, 0.0]
                })
            
            result = {
                "success": True,
                "success_num": response_ros.success_num,
                "confidence": list(response_ros.confidence),
                "vis_image": vis_image_base64,
                "pose_images": pose_images_base64,
                "positions": positions,
                "preparation_positions": preparation_positions,
                "grab_positions": grab_positions
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
            
        except Exception as e:
            self.send_error(500, f"处理请求异常: {str(e)}")
    
    def handle_process_debug_step(self):
        """处理调试步骤请求"""
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
            
            step_id = data.get('step_id', '')
            step_index = data.get('step_index', 0)
            session_id = data.get('session_id', 'default')
            is_redo = data.get('is_redo', False)
            input_image = data.get('input_image', '')
            
            if not input_image:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "输入图像为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 解码base64图像
            try:
                if ',' in input_image:
                    image_base64 = input_image.split(',')[1]
                else:
                    image_base64 = input_image
                
                image_bytes = base64.b64decode(image_base64)
                np_arr = np.frombuffer(image_bytes, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_image is None or cv_image.size == 0:
                    self.send_response(400)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"success": False, "error": "图像解码失败"}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    return
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"图像解码异常: {str(e)}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 调用ROS2服务进行调试步骤处理
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 转换为ROS2图像消息
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # 调用调试步骤处理服务
            response_ros, error = ros2_node.process_debug_step(
                image_msg, step_id, step_index, session_id, is_redo
            )
            
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"调试步骤处理失败: {error}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            if not response_ros.success:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": response_ros.error_message}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 转换结果图像为base64
            try:
                result_cv_image = bridge.imgmsg_to_cv2(response_ros.result_image, desired_encoding='bgr8')
                _, buffer = cv2.imencode('.jpg', result_cv_image)
                result_base64 = base64.b64encode(buffer).decode('utf-8')
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"结果图像转换失败: {str(e)}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 解析特征数据
            features = []
            try:
                import json as json_lib
                for feature_json_str in response_ros.feature_data:
                    feature_data = json_lib.loads(feature_json_str)
                    features.append(feature_data)
            except Exception as e:
                # 如果解析失败，继续返回结果，只是没有特征数据
                pass
            
            # 解析元数据
            metadata = {}
            try:
                import json as json_lib
                if response_ros.metadata_json:
                    metadata = json_lib.loads(response_ros.metadata_json)
            except Exception as e:
                metadata = {"error": "metadata_parse_error", "message": str(e)}
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "result_image": f"data:image/jpeg;base64,{result_base64}",
                "features": features,
                "metadata": metadata,
                "message": f"步骤 {step_id} 处理完成"
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_save_debug_features(self):
        """保存调试特征结果"""
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
            
            session_id = data.get('session_id', 'default')
            features = data.get('features', [])
            
            # 保存到CSV文件
            debug_dir = Path(f"debug/{session_id}")
            debug_dir.mkdir(parents=True, exist_ok=True)
            
            csv_path = debug_dir / "features.csv"
            if features:
                with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(['id', 'ub', 'vb', 'rb', 'us', 'vs', 'rs', 'theta', 'theta_deg', 'area'])
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
                "features_count": len(features)
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"保存失败: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_list_templates(self):
        """列出模板"""
        try:
            # 读取请求体（可选，用于指定模板目录）
            templates_dir = ""
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                templates_dir = data.get('templates_dir', '')
            
            # 调用ROS2服务
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            result, error = ros2_node.list_templates(templates_dir)
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 读取图像文件并转换为base64
            template_list = []
            for i, template_path in enumerate(result['template_paths']):
                try:
                    # 读取图像
                    cv_image = cv2.imread(template_path)
                    if cv_image is not None:
                        # 转换为base64
                        _, buffer = cv2.imencode('.jpg', cv_image)
                        image_base64 = base64.b64encode(buffer).decode('utf-8')
                        
                        template_list.append({
                            "template_id": result['template_ids'][i],
                            "workpiece_id": result['workpiece_ids'][i],
                            "pose_id": result['pose_ids'][i],
                            "image_path": template_path,
                            "image_base64": f"data:image/jpeg;base64,{image_base64}"
                        })
                except Exception as e:
                    # 如果读取失败，跳过该模板
                    print(f"读取模板图像失败: {template_path}, 错误: {str(e)}")
                    continue
            
            # 提取唯一的工件ID列表（从模板列表和文件夹结构）
            workpiece_ids_from_templates = list(set([t['workpiece_id'] for t in template_list if t.get('workpiece_id')]))
            
            # 如果模板目录存在，也从文件夹结构中读取工件ID（包含空文件夹）
            workpiece_ids_from_folders = []
            try:
                from pathlib import Path
                # 使用工作空间根目录下的templates路径
                template_root = Path('/home/nvidia/RVG_ws/templates')
                
                if template_root.exists():
                    for item in template_root.iterdir():
                        if item.is_dir() and not item.name.startswith('.'):
                            workpiece_ids_from_folders.append(item.name)
            except Exception as e:
                print(f"读取模板文件夹失败: {str(e)}")
            
            # 合并并去重
            all_workpiece_ids = list(set(workpiece_ids_from_templates + workpiece_ids_from_folders))
            all_workpiece_ids.sort()  # 排序以确保一致性
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "templates": template_list,
                "count": len(template_list),
                "workpiece_ids": all_workpiece_ids  # 添加工件ID列表
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_visualize_grasp_pose(self):
        """可视化抓取姿态"""
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
            
            template_image_path = data.get('template_image_path', '')
            workpiece_id = data.get('workpiece_id', '')
            pose_id = data.get('pose_id', '')
            
            if not template_image_path or not workpiece_id or not pose_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "template_image_path、workpiece_id和pose_id不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 调用ROS2服务
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            result, error = ros2_node.visualize_grasp_pose(template_image_path, workpiece_id, pose_id)
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": error}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "image_base64": result["image_base64"]
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_standardize_template(self):
        """模板标准化"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error_message": "请求体为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            
            if not workpiece_id:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error_message": "工件ID不能为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            if ros2_node is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error_message": "ROS2节点未初始化"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            result, error = ros2_node.standardize_template(workpiece_id)
            
            if error:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error_message": error}
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
            response = {"success": False, "error_message": f"模板标准化异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_save_template_pose(self):
        """保存模板姿态"""
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
            
            # 保存文件到工作空间根目录下的templates文件夹
            template_dir = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}"
            template_dir.mkdir(parents=True, exist_ok=True)
            
            json_path = template_dir / filename_map[pose_type]
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(robot_status, f, indent=2, ensure_ascii=False)
            
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
            response = {"success": False, "error": f"保存失败: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
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
    
    def handle_set_robot_pose(self):
        """处理机器人位姿设置请求"""
        try:
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "请求体为空")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            target_pose = data.get('target_pose', [])
            use_joints = data.get('use_joints', False)
            is_radian = data.get('is_radian', False)
            velocity = data.get('velocity', 30.0)
            
            if len(target_pose) != 6:
                self.send_error(400, "target_pose必须包含6个元素")
                return
            
            result, error = ros2_node.set_robot_pose(target_pose, use_joints, is_radian, velocity)
            
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
            if ros2_node is None:
                self.send_error(500, "ROS2节点未初始化")
                return
            
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "请求体为空")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            workpiece_id = data.get('workpiece_id', '')
            pose_id = data.get('pose_id', '')
            pose_type = data.get('pose_type', '')
            
            if not workpiece_id or not pose_id or not pose_type:
                self.send_error(400, "workpiece_id, pose_id, pose_type不能为空")
                return
            
            pose_data, error = ros2_node.read_template_pose_file(workpiece_id, pose_id, pose_type)
            
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
                response = {"success": True, "pose_data": pose_data}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"处理请求异常: {str(e)}"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_exit(self):
        """处理退出请求"""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = b''
            if content_length > 0:
                post_data = self.rfile.read(content_length)
            
            if post_data:
                data = json.loads(post_data.decode('utf-8'))
                if data.get('action') == 'exit_service':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"status": "success", "message": "服务正在退出..."}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    
                    # 延迟退出
                    def delayed_exit():
                        time.sleep(1)
                        print("\n🛑 收到退出请求，正在关闭服务器...")
                        os._exit(0)
                    
                    exit_thread = threading.Thread(target=delayed_exit, daemon=True)
                    exit_thread.start()
                    return
            
            self.send_response(400)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"status": "error", "message": "Invalid action"}
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"status": "error", "message": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))


def run_ros2_executor():
    """在后台线程中运行ROS2执行器"""
    global ros2_node
    if ros2_node is not None:
        rclpy.spin(ros2_node)


def main():
    global ros2_node
    
    # 初始化ROS2
    rclpy.init()
    ros2_node = ROS2BridgeNode()
    
    # 在后台线程中运行ROS2执行器
    executor_thread = threading.Thread(target=run_ros2_executor, daemon=True)
    executor_thread.start()
    
    # 启动HTTP服务器
    port = 8088
    handler = HTTPBridgeHandler
    
    # 创建自定义的 TCPServer，支持端口重用
    class ReusableTCPServer(socketserver.TCPServer):
        allow_reuse_address = True
    
    try:
        httpd = ReusableTCPServer(("", port), handler)
        print(f"HTTP桥接服务器启动在端口 {port}")
        print(f"访问 http://localhost:{port}/index.html 使用Web UI")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n正在关闭服务器...")
            httpd.shutdown()
            httpd.server_close()
            rclpy.shutdown()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print(f"错误: 端口 {port} 已被占用")
            print("正在尝试清理端口...")
            import subprocess
            try:
                # 尝试找到并终止占用端口的进程
                result = subprocess.run(['lsof', '-ti', f':{port}'], 
                                      capture_output=True, text=True)
                if result.stdout.strip():
                    pid = result.stdout.strip().split('\n')[0]
                    print(f"找到占用端口的进程 PID: {pid}，正在终止...")
                    subprocess.run(['kill', '-9', pid], check=False)
                    import time
                    time.sleep(2)  # 等待端口释放
                    # 重新尝试启动
                    httpd = ReusableTCPServer(("", port), handler)
                    print(f"HTTP桥接服务器启动在端口 {port}")
                    print(f"访问 http://localhost:{port}/index.html 使用Web UI")
                    try:
                        httpd.serve_forever()
                    except KeyboardInterrupt:
                        print("\n正在关闭服务器...")
                        httpd.shutdown()
                        httpd.server_close()
                        rclpy.shutdown()
                else:
                    print(f"无法找到占用端口 {port} 的进程")
                    print("请手动检查并终止占用端口的进程")
                    sys.exit(1)
            except Exception as cleanup_error:
                print(f"清理端口时出错: {cleanup_error}")
                print("请手动运行以下命令清理端口:")
                print(f"  lsof -ti :{port} | xargs kill -9")
                sys.exit(1)
        else:
            raise


if __name__ == '__main__':
    main()

