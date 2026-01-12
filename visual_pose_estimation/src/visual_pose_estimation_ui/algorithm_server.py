#!/usr/bin/env python3

import http.server
import socketserver
import threading
import time
import sys
import os
import json
import signal
import base64
import csv
import yaml
import datetime
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from interface.srv import SoftwareTrigger, EstimatePose, ListTemplates, SetRobotPose, StandardizeTemplate, WritePLCRegister
from interface.msg import RobotStatus, ImageData
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ROS2Node(Node):
    """ROS2节点，用于与ROS2系统通信"""
    def __init__(self):
        super().__init__('algorithm_http_server_node')
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/camera_0/software_trigger')
        self.estimate_pose_client = self.create_client(EstimatePose, '/estimate_pose')
        self.list_templates_client = self.create_client(ListTemplates, '/list_templates')
        self.set_robot_pose_client = self.create_client(SetRobotPose, '/set_robot_pose')
        self.standardize_template_client = self.create_client(StandardizeTemplate, '/standardize_template')
        self.write_plc_register_client = self.create_client(WritePLCRegister, '/write_plc_register')
        
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
        
        # 执行器标志
        self.executor_running = False
        self.executor_lock = threading.Lock()
        
        self.get_logger().info('ROS2节点初始化完成')
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        with self.robot_status_lock:
            self.latest_robot_status = msg
    
    def image_data_callback(self, msg):
        """图像数据回调"""
        with self.image_lock:
            self.latest_image_data = msg
            self.image_received = True
    
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
                    "euler_orientation_rpy_deg": list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_deg),
                    "euler_orientation_rpy_rad": list(self.latest_robot_status.cartesian_position.euler_orientation_rpy_rad)
                }
            }
            return status_dict
    
    def capture_image(self, camera_id='DA3234363', timeout=10.0):
        """触发相机拍照并获取图像"""
        try:
            # 等待服务可用
            if not self.trigger_client.wait_for_service(timeout_sec=2.0):
                return None, "相机服务未运行，请先启动vision_acquisition节点"
            
            # 重置图像接收标志
            with self.image_lock:
                self.image_received = False
                self.latest_image_data = None
            
            # 调用服务
            request = SoftwareTrigger.Request()
            request.camera_id = camera_id
            
            future = self.trigger_client.call_async(request)
            
            # 等待服务响应 - 需要spin executor来处理响应
            start_time = time.time()
            while time.time() - start_time < 5.0:
                # 确保executor处理消息
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
            if not future.done():
                return None, "相机服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "相机服务调用失败"
                if not response.success:
                    return None, f"相机触发失败: {response.message}"
            except Exception as e:
                return None, f"获取服务响应失败: {str(e)}"
            
            # 等待图像数据 - 后台线程会处理消息，这里只等待
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.image_lock:
                    if self.image_received and self.latest_image_data is not None:
                        # 转换图像
                        try:
                            # 从ImageData消息中提取图像
                            img_msg = self.latest_image_data.image
                            
                            # 检查编码类型
                            encoding = img_msg.encoding.lower().strip()
                            self.get_logger().info(f'图像编码: {encoding}, 尺寸: {img_msg.width}x{img_msg.height}, 数据大小: {len(img_msg.data)} bytes')
                            
                            # 判断是否为压缩格式（JPEG/PNG等）
                            is_compressed = False
                            if 'jpeg' in encoding or 'jpg' in encoding or encoding == 'jipeg':
                                # JPEG编码（包括拼写错误的jipeg），使用cv2.imdecode直接解码
                                is_compressed = True
                                np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if cv_image is None:
                                    return None, "JPEG图像解码失败"
                            elif 'png' in encoding:
                                # PNG编码，使用cv2.imdecode
                                is_compressed = True
                                np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if cv_image is None:
                                    return None, "PNG图像解码失败"
                            
                            if not is_compressed:
                                # 其他编码（如bgr8, rgb8等），使用cv_bridge
                                try:
                                    cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                                except Exception as bridge_error:
                                    # 如果cv_bridge失败，尝试作为JPEG解码（可能是编码字段错误）
                                    self.get_logger().warn(f'cv_bridge转换失败: {bridge_error}, 尝试作为JPEG解码')
                                    np_arr = np.frombuffer(bytes(img_msg.data), np.uint8)
                                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                    if cv_image is None:
                                        return None, f"图像解码失败 (编码: {encoding}): {bridge_error}"
                            
                            if cv_image is None or cv_image.size == 0:
                                return None, "图像解码后为空"
                            
                            return cv_image, None
                        except Exception as e:
                            import traceback
                            error_detail = traceback.format_exc()
                            self.get_logger().error(f'图像转换异常: {str(e)}\n{error_detail}')
                            return None, f"图像转换失败: {str(e)}"
                
                # 短暂休眠，等待后台线程处理消息
                time.sleep(0.05)
            
            return None, "图像接收超时"
            
        except Exception as e:
            return None, f"相机触发异常: {str(e)}"
    
    def estimate_pose(self, image_base64, object_id, timeout=30.0):
        """调用姿态估计服务"""
        try:
            self.get_logger().info(f'开始姿态估计，工件ID: {object_id}')
            
            # 等待服务可用
            if not self.estimate_pose_client.wait_for_service(timeout_sec=2.0):
                return None, "姿态估计服务未运行，请先启动visual_pose_estimation节点"
            
            # 解码base64图像
            try:
                image_data = base64.b64decode(image_base64)
                np_arr = np.frombuffer(image_data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    return None, "图像解码失败"
            except Exception as e:
                return None, f"图像解码异常: {str(e)}"
            
            # 转换为ROS2图像消息
            try:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            except Exception as e:
                return None, f"图像转换失败: {str(e)}"
            
            # 创建请求
            request = EstimatePose.Request()
            request.image = ros_image
            request.object_id = object_id
            
            # 调用服务
            future = self.estimate_pose_client.call_async(request)
            
            # 等待服务响应，需要spin executor来处理响应
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 确保executor处理消息
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
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
                        "euler_orientation_rpy_rad": [float(v) for v in cart_pos.euler_orientation_rpy_rad],
                        "euler_orientation_rpy_deg": [float(v) for v in cart_pos.euler_orientation_rpy_deg],
                        "joint_position_rad": [float(v) for v in cart_pos.joint_position_rad],
                        "joint_position_deg": [float(v) for v in cart_pos.joint_position_deg]
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
                
                # 转换可视化图像
                if response.vis_image.data:
                    try:
                        vis_cv_image = self.bridge.imgmsg_to_cv2(response.vis_image, desired_encoding='bgr8')
                        _, vis_buffer = cv2.imencode('.jpg', vis_cv_image)
                        result["vis_image"] = "data:image/jpeg;base64," + base64.b64encode(vis_buffer).decode('utf-8')
                    except Exception as e:
                        self.get_logger().warn(f'可视化图像转换失败: {str(e)}')
                
                # 转换姿态图像数组
                for pose_img in response.pose_image:
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
    
    def list_templates(self, templates_dir="", timeout=10.0):
        """调用列出模板服务"""
        try:
            self.get_logger().info(f'开始列出模板，目录: {templates_dir if templates_dir else "默认"}')
            
            # 等待服务可用
            if not self.list_templates_client.wait_for_service(timeout_sec=2.0):
                return None, "列出模板服务未运行，请先启动visual_pose_estimation节点"
            
            # 创建请求
            request = ListTemplates.Request()
            request.templates_dir = templates_dir
            
            # 调用服务
            future = self.list_templates_client.call_async(request)
            
            # 等待服务响应，需要spin executor来处理响应
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 确保executor处理消息
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
            if not future.done():
                return None, "列出模板服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "列出模板服务调用失败"
                
                if response.success:
                    result = {
                        "success": True,
                        "template_paths": list(response.template_paths),
                        "template_ids": list(response.template_ids),
                        "workpiece_ids": list(response.workpiece_ids),
                        "pose_ids": list(response.pose_ids)
                    }
                    self.get_logger().info(f'列出模板成功，找到 {len(result["template_ids"])} 个模板')
                    return result, None
                else:
                    return None, response.error_message if response.error_message else "列出模板失败"
                    
            except Exception as e:
                return None, f"处理服务响应失败: {str(e)}"
                
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            self.get_logger().error(f'列出模板服务异常: {str(e)}\n{error_detail}')
            return None, f"列出模板服务异常: {str(e)}"
    
    def set_robot_pose(self, target_pose, use_joints=False, is_radian=False, velocity=50.0, timeout=30.0):
        """设置机器人位姿"""
        try:
            if not self.set_robot_pose_client.wait_for_service(timeout_sec=5.0):
                return None, "设置机器人位姿服务不可用"
            
            request = SetRobotPose.Request()
            request.target_pose = [float(v) for v in target_pose]
            request.use_joints = bool(use_joints)
            request.is_radian = bool(is_radian)
            request.velocity = float(velocity)
            
            future = self.set_robot_pose_client.call_async(request)
            
            # 等待服务响应
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
            if not future.done():
                return None, "设置机器人位姿服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "设置机器人位姿服务调用失败"
                
                result = {
                    "success": bool(response.success),
                    "error_code": int(response.error_code),
                    "message": str(response.message)
                }
                
                if response.success:
                    self.get_logger().info(f'设置机器人位姿成功: {response.message}')
                else:
                    self.get_logger().warn(f'设置机器人位姿失败: {response.message}')
                
                return result, None
                
            except Exception as e:
                import traceback
                self.get_logger().error(f'设置机器人位姿服务响应处理异常: {str(e)}\n{traceback.format_exc()}')
                return None, f"设置机器人位姿服务响应处理异常: {str(e)}"
                
        except Exception as e:
            import traceback
            self.get_logger().error(f'设置机器人位姿服务调用异常: {str(e)}\n{traceback.format_exc()}')
            return None, f"设置机器人位姿服务调用异常: {str(e)}"
    
    def standardize_template(self, workpiece_id, timeout=120.0):
        """调用模板标准化服务"""
        try:
            if not self.standardize_template_client.wait_for_service(timeout_sec=5.0):
                return None, "模板标准化服务未运行，请先启动visual_pose_estimation节点"
            
            request = StandardizeTemplate.Request()
            request.workpiece_id = str(workpiece_id)
            
            future = self.standardize_template_client.call_async(request)
            
            # 模板标准化可能需要较长时间，设置更长的超时时间
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
            if not future.done():
                return None, "模板标准化服务调用超时"
            
            try:
                response = future.result()
                if response is None:
                    return None, "模板标准化服务调用失败"
                
                result = {
                    "success": bool(response.success),
                    "processed_count": int(response.processed_count),
                    "skipped_count": int(response.skipped_count),
                    "processed_pose_ids": list(response.processed_pose_ids),
                    "skipped_pose_ids": list(response.skipped_pose_ids),
                    "error_message": str(response.error_message) if response.error_message else ""
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
                with self.executor_lock:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if future.done():
                    break
                time.sleep(0.05)
            
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
        if self.path == '/api/capture_image':
            self.handle_capture_image()
        elif self.path == '/api/capture_template_image':
            self.handle_capture_template_image()
        elif self.path == '/api/get_robot_status':
            self.handle_get_robot_status()
        elif self.path == '/api/save_template_pose':
            self.handle_save_template_pose()
        elif self.path == '/api/process_debug_step':
            self.handle_process_debug_step()
        elif self.path == '/api/save_debug_features':
            self.handle_save_debug_features()
        elif self.path == '/api/estimate_pose':
            self.handle_estimate_pose()
        elif self.path == '/api/list_templates':
            self.handle_list_templates()
        elif self.path == '/api/list_workpiece_ids':
            self.handle_list_workpiece_ids()
        elif self.path == '/api/set_robot_pose':
            self.handle_set_robot_pose()
        elif self.path == '/api/standardize_template':
            self.handle_standardize_template()
        elif self.path == '/api/write_plc_register':
            self.handle_write_plc_register()
        elif self.path == '/api/read_template_pose':
            self.handle_read_template_pose()
        elif self.path == '/api/execute_pose_sequence':
            self.handle_execute_pose_sequence()
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
            camera_id = 'DA3234363'  # 默认相机ID
            
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                camera_id = data.get('camera_id', 'DA3234363')
            
            # 触发相机拍照
            cv_image, error_msg = ros2_node.capture_image(camera_id)
            
            if cv_image is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": error_msg}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 将图像编码为JPEG格式的base64用于返回
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            response_json = json.dumps({
                "success": True,
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
            
            # 触发相机拍照
            cv_image, error_msg = ros2_node.capture_image(camera_id)
            
            if cv_image is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {"success": False, "error": error_msg}
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                return
            
            # 保存图像
            template_dir = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}"
            template_dir.mkdir(parents=True, exist_ok=True)
            
            image_path = template_dir / "original_image.jpg"
            cv2.imwrite(str(image_path), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            # 将图像编码为JPEG格式的base64用于返回
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            response_json = json.dumps({
                "success": True,
                "image_path": str(image_path),
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
            
            # 保存文件
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
            response = {"success": False, "error": str(e)}
            self.wfile.write(json.dumps(response).encode('utf-8'))
    
    def handle_process_debug_step(self):
        """处理调试步骤"""
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
            step_index = data.get('step_index', 0)
            step_id = data.get('step_id', '')
            input_image = data.get('input_image', '')
            is_redo = data.get('is_redo', False)
            
            if not input_image:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "输入图像为空"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 解码base64图像
            try:
                # 移除data:image/jpeg;base64,前缀
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
            
            # 创建调试输出目录
            debug_dir = Path(f"debug/{session_id}")
            debug_dir.mkdir(parents=True, exist_ok=True)
            
            # 加载配置文件（如果是重做，重新加载以获取最新参数）
            config_path = "configs/default.yaml"
            if is_redo:
                # 重新加载配置文件
                config_params = self.load_config_params(config_path)
            else:
                # 首次处理，加载配置
                config_params = self.load_config_params(config_path)
            
            # ========== 步骤0: 读取并保存原始图像副本（所有步骤都使用这个副本） ==========
            # 保存原始图像（如果还没保存）
            original_path = debug_dir / "step_00_original.jpg"
            if step_index == 0:
                if cv_image is not None and len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    cv2.imwrite(str(original_path), cv_image)
            
            # 读取原始图像并创建副本（作为所有步骤的原始图像参考）
            original_rgb_copy = None
            original_image_path = debug_dir / "step_00_original.jpg"
            
            if original_image_path.exists():
                original_rgb_from_file = cv2.imread(str(original_image_path), cv2.IMREAD_COLOR)
                if original_rgb_from_file is not None and len(original_rgb_from_file.shape) == 3 and original_rgb_from_file.shape[2] == 3:
                    original_rgb_copy = original_rgb_from_file.copy()
                elif cv_image is not None and len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    original_rgb_copy = cv_image.copy()
            elif cv_image is not None and len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                original_rgb_copy = cv_image.copy()
                cv2.imwrite(str(original_image_path), cv_image)
            
            if original_rgb_copy is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "无法获取有效的原始图像"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 处理不同步骤
            result_image = None
            features = []
            metadata = {}
            
            if step_id == 'original':
                result_image = cv_image
            elif step_id == 'remove_green':
                result_image = self.process_remove_green(cv_image, debug_dir, step_index, config_params)
            elif step_id == 'big_circle':
                # big_circle 步骤需要原始RGB图像和前一步的结果（去除绿色背景）
                prev_result = self.load_prev_step_result(debug_dir, step_index - 1, original_rgb_copy)
                if prev_result is None:
                    # 尝试直接加载 remove_green 步骤的结果
                    remove_green_path = debug_dir / "step_01_remove_green.jpg"
                    if remove_green_path.exists():
                        prev_result = cv2.imread(str(remove_green_path), cv2.IMREAD_COLOR)
                        if prev_result is not None and len(prev_result.shape) == 3:
                            prev_result = cv2.cvtColor(prev_result, cv2.COLOR_BGR2GRAY)
                
                result_image, step_features = self.process_big_circle(original_rgb_copy, prev_result, debug_dir, step_index, config_params)
                features.extend(step_features)
            elif step_id == 'head_only':
                # head_only 步骤需要去除绿色背景后的二值图像（step_01_remove_green.jpg）
                # 直接加载 remove_green 步骤的结果
                remove_green_path = debug_dir / "step_01_remove_green.jpg"
                prev_result = None
                if remove_green_path.exists():
                    prev_result = cv2.imread(str(remove_green_path), cv2.IMREAD_COLOR)
                    if prev_result is not None:
                        if len(prev_result.shape) == 3:
                            prev_result = cv2.cvtColor(prev_result, cv2.COLOR_BGR2GRAY)
                        print(f"[DEBUG] head_only: Loaded remove_green image, shape={prev_result.shape if prev_result is not None else None}")
                    else:
                        print(f"[ERROR] head_only: Failed to read remove_green image from {remove_green_path}")
                else:
                    print(f"[ERROR] head_only: remove_green image not found at {remove_green_path}")
                
                if prev_result is None:
                    self.send_response(500)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"success": False, "error": "无法加载去除绿色背景后的二值图像"}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    return
                
                result_image = self.process_head_only(prev_result, debug_dir, step_index, config_params)
            elif step_id == 'valve_circle':
                # valve_circle 步骤需要 head_only 步骤的结果（只保留头部的二值图像）
                # 需要原始RGB图像用于可视化
                # 还需要 big_circle 步骤的特征数据（用于在左侧原图上绘制大圆）
                head_only_path = debug_dir / f"step_{step_index - 1:02d}_head_only.jpg"
                prev_result = None
                if head_only_path.exists():
                    prev_result = cv2.imread(str(head_only_path), cv2.IMREAD_COLOR)
                    if prev_result is not None and len(prev_result.shape) == 3:
                        prev_result = cv2.cvtColor(prev_result, cv2.COLOR_BGR2GRAY)
                # 如果不存在，尝试加载前一步的结果
                if prev_result is None:
                    prev_result = self.load_prev_step_result(debug_dir, step_index - 1, cv_image)
                
                if prev_result is None:
                    self.send_response(500)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"success": False, "error": "无法加载 head_only 步骤的结果"}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    return
                
                # 加载 big_circle 步骤的特征数据（用于在左侧原图上绘制大圆）
                big_circle_features = []
                big_circle_step_index = 2  # big_circle 是第3步（索引为2）
                big_circle_path = debug_dir / f"step_{big_circle_step_index:02d}_big_circle.jpg"
                if big_circle_path.exists():
                    # 尝试从保存的 metadata 文件中加载特征数据
                    import json as json_lib
                    metadata_path = debug_dir / "metadata.json"
                    if metadata_path.exists():
                        try:
                            with open(metadata_path, 'r', encoding='utf-8') as f:
                                metadata = json_lib.load(f)
                                if 'big_circle_features' in metadata:
                                    big_circle_features = metadata['big_circle_features']
                        except Exception as e:
                            print(f"[WARN] 无法加载 big_circle 特征数据: {e}")
                
                result_image, step_features = self.process_valve_circle(original_rgb_copy, prev_result, debug_dir, step_index, config_params, big_circle_features)
                features.extend(step_features)
            elif step_id == 'crop_and_rotate':
                # crop_and_rotate 步骤需要原始RGB图像、大圆和小圆信息
                # 加载大圆和小圆的特征数据
                big_circle_features = []
                valve_circle_features = []
                metadata_path = debug_dir / "metadata.json"
                if metadata_path.exists():
                    try:
                        with open(metadata_path, 'r', encoding='utf-8') as f:
                            metadata = json.load(f)
                            if 'big_circle_features' in metadata:
                                big_circle_features = metadata['big_circle_features']
                            if 'valve_circle_features' in metadata:
                                valve_circle_features = metadata['valve_circle_features']
                    except Exception as e:
                        print(f"[WARN] 无法加载特征数据: {e}")
                
                if not big_circle_features or not valve_circle_features:
                    self.send_response(500)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = {"success": False, "error": "无法加载大圆或小圆的特征数据"}
                    self.wfile.write(json.dumps(response).encode('utf-8'))
                    return
                
                # 加载去除绿色背景后的掩膜
                remove_green_path = debug_dir / "step_01_remove_green.jpg"
                remove_green_mask = None
                if remove_green_path.exists():
                    remove_green_mask = cv2.imread(str(remove_green_path), cv2.IMREAD_GRAYSCALE)
                
                result_image = self.process_crop_and_rotate(
                    original_rgb_copy, 
                    big_circle_features[0], 
                    valve_circle_features[0],
                    remove_green_mask,
                    debug_dir, 
                    step_index, 
                    config_params
                )
            elif step_id == 'binarize':
                # 需要原始RGB图像和去除绿色背景后的掩码
                # 使用原始RGB图像副本
                remove_green_path = debug_dir / "step_01_remove_green.jpg"
                prev_result = None
                if remove_green_path.exists():
                    prev_result = cv2.imread(str(remove_green_path), cv2.IMREAD_COLOR)
                    if prev_result is not None and len(prev_result.shape) == 3:
                        prev_result = cv2.cvtColor(prev_result, cv2.COLOR_BGR2GRAY)
                # 如果不存在，尝试加载前一步的结果
                if prev_result is None:
                    prev_result = self.load_prev_step_result(debug_dir, step_index - 1, cv_image)
                # 使用原始RGB图像
                result_image = self.process_binarize(original_rgb_copy, prev_result, debug_dir, step_index)
            elif step_id == 'morphology':
                prev_result = self.load_prev_step_result(debug_dir, step_index - 1, cv_image)
                result_image = self.process_morphology(prev_result, debug_dir, step_index, config_params)
            elif step_id == 'components':
                prev_result = self.load_prev_step_result(debug_dir, step_index - 1, cv_image)
                result_image = self.process_components(prev_result, debug_dir, step_index, config_params)
            elif step_id == 'features':
                prev_result = self.load_prev_step_result(debug_dir, step_index - 1, cv_image)
                result_image, features = self.process_features(cv_image, prev_result, debug_dir, step_index, config_params)
            else:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": f"未知的步骤ID: {step_id}"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            if result_image is None:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {"success": False, "error": "步骤处理失败"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                return
            
            # 保存结果图像
            result_path = debug_dir / f"step_{step_index:02d}_{step_id}.jpg"
            cv2.imwrite(str(result_path), result_image)
            
            # 编码结果图像为base64
            _, buffer = cv2.imencode('.jpg', result_image)
            result_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # 构建消息，包含特征数据信息
            message = f"步骤 {step_index + 1} 处理完成，结果已保存到 {result_path}"
            if step_id == 'big_circle' and features:
                # 为 big_circle 步骤添加特征数据到消息
                for i, feat in enumerate(features):
                    message += f"\n圆 {i+1}: 圆心=({feat['center_x']:.1f}, {feat['center_y']:.1f}), 半径={feat['radius']:.1f}, 面积={feat['area']:.2f}"
                metadata['big_circle_count'] = len(features)
                metadata['big_circle_features'] = features
            elif step_id == 'valve_circle' and features:
                # 为 valve_circle 步骤添加特征数据到消息
                for i, feat in enumerate(features):
                    message += f"\n阀体外接圆 {i+1}: 圆心=({feat['center_x']:.1f}, {feat['center_y']:.1f}), 半径={feat['radius']:.1f}, 面积={feat['area']:.2f}"
                metadata['valve_circle_count'] = len(features)
                metadata['valve_circle_features'] = features
                
                # valve_circle 步骤完成后，保存所有几何特征数据
                self.save_geometric_features(debug_dir, metadata)
            
            # 保存特征数据到 metadata（如果存在）
            if features:
                metadata['features'] = features
                metadata['feature_count'] = len(features)
            
            # 保存 metadata 到文件（用于后续步骤读取）
            metadata_path = debug_dir / "metadata.json"
            try:
                # 如果文件已存在，先读取现有数据
                existing_metadata = {}
                if metadata_path.exists():
                    with open(metadata_path, 'r', encoding='utf-8') as f:
                        existing_metadata = json.load(f)
                # 合并新的 metadata
                existing_metadata.update(metadata)
                # 保存到文件
                with open(metadata_path, 'w', encoding='utf-8') as f:
                    json.dump(existing_metadata, f, indent=2, ensure_ascii=False)
            except Exception as e:
                print(f"[WARN] 保存 metadata 到文件失败: {e}")
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {
                "success": True,
                "result_image": f"data:image/jpeg;base64,{result_base64}",
                "result_path": str(result_path),
                "features": features,
                "metadata": metadata,
                "message": message
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] 步骤处理异常: {error_detail}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            response = {"success": False, "error": f"步骤处理异常: {str(e)}", "detail": error_detail}
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
            input_image = data.get('input_image', '')
            object_id = data.get('object_id', '')
            
            if not input_image:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    "success": False,
                    "error": "缺少输入图像"
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
            if ',' in input_image:
                input_image = input_image.split(',')[1]
            
            # 调用ROS2服务
            print(f"[INFO] 调用姿态估计服务，工件ID: {object_id}")
            try:
                result, error = ros2_node.estimate_pose(input_image, object_id)
                
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
        """处理列出模板请求（直接扫描文件系统，查找image.jpg）"""
        try:
            # 读取请求数据
            templates_dir = "/home/nvidia/RVG_ws/templates"
            workpiece_id = ""
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                # 如果templates_dir为空字符串，使用默认值
                requested_dir = data.get('templates_dir', '')
                if requested_dir and requested_dir.strip():
                    templates_dir = requested_dir
                workpiece_id = data.get('workpiece_id', '')
            
            # 检查模板目录是否存在
            templates_path = Path(templates_dir)
            if not templates_path.exists() or not templates_path.is_dir():
                self.send_response(500)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                response = {
                    "success": False,
                    "error": f"模板目录不存在: {templates_dir}"
                }
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
                self.wfile.flush()
                return
            
            print(f"[INFO] 扫描模板目录: {templates_dir}, 工件ID过滤: {workpiece_id if workpiece_id else '无'}")
            
            # 直接扫描文件系统查找模板（查找image.jpg）
            template_list = []
            
            # 遍历所有工件ID目录
            try:
                for workpiece_entry in templates_path.iterdir():
                    if not workpiece_entry.is_dir():
                        continue
                    
                    current_workpiece_id = workpiece_entry.name
                    
                    # 跳过隐藏文件和特殊文件
                    if current_workpiece_id.startswith('.'):
                        continue
                    
                    # 如果指定了workpiece_id，进行过滤
                    if workpiece_id and current_workpiece_id != workpiece_id:
                        print(f"[DEBUG] 跳过工件ID: {current_workpiece_id} (不匹配过滤条件: {workpiece_id})")
                        continue
                    
                    print(f"[DEBUG] 检查工件ID目录: {current_workpiece_id}")
                    
                    # 遍历该工件ID下的所有pose目录
                    try:
                        for pose_entry in workpiece_entry.iterdir():
                            if not pose_entry.is_dir():
                                continue
                            
                            pose_dir_name = pose_entry.name
                            
                            # 检查是否是pose_开头的目录
                            if not pose_dir_name.startswith('pose_'):
                                continue
                            
                            # 提取姿态ID（pose_后面的部分）
                            pose_id = pose_dir_name[5:]  # 跳过"pose_"前缀
                            
                            # 查找image.jpg文件
                            image_path = pose_entry / "image.jpg"
                            
                            print(f"[DEBUG] 检查路径: {image_path}, 存在: {image_path.exists()}, 是文件: {image_path.is_file() if image_path.exists() else False}")
                            
                            if image_path.exists() and image_path.is_file():
                                try:
                                    # 读取图像文件并转换为base64
                                    with open(image_path, 'rb') as f:
                                        image_bytes = f.read()
                                        image_base64 = base64.b64encode(image_bytes).decode('utf-8')
                                    
                                    template_list.append({
                                        "template_id": f"{current_workpiece_id}_{pose_id}",
                                        "workpiece_id": current_workpiece_id,
                                        "pose_id": pose_id,
                                        "image_path": str(image_path),
                                        "image_base64": "data:image/jpeg;base64," + image_base64
                                    })
                                    
                                    print(f"[DEBUG] 找到模板: 工件ID={current_workpiece_id}, 姿态ID={pose_id}, 路径={image_path}")
                                except Exception as e:
                                    print(f"[WARN] 无法读取模板图像 {image_path}: {str(e)}")
                                    continue
                            else:
                                print(f"[DEBUG] 图像文件不存在或不是文件: {image_path}")
                    except Exception as e:
                        print(f"[WARN] 遍历工件ID目录 {current_workpiece_id} 时出错: {str(e)}")
                        continue
            except Exception as e:
                print(f"[ERROR] 遍历模板目录时出错: {str(e)}")
                import traceback
                traceback.print_exc()
            
            # 构建响应
            response = {
                "success": True,
                "templates": template_list,
                "count": len(template_list)
            }
            
            print(f"[INFO] 列出模板成功，找到 {len(template_list)} 个模板")
            
            # 发送响应
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
            response = {
                "success": False,
                "error": f"服务器错误: {str(e)}"
            }
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode('utf-8'))
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
            image_path = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}" / image_name
            
            # 如果指定的图像不存在，尝试其他图像（向后兼容）
            if not image_path.exists():
                if image_name == 'gripper_visualization.jpg':
                    # 如果gripper_visualization.jpg不存在，尝试image.jpg
                    image_path = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}" / "image.jpg"
                if not image_path.exists():
                    # 如果image.jpg不存在，尝试standardized_image.jpg
                    image_path = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}" / "standardized_image.jpg"
                if not image_path.exists():
                    # 如果都不存在，尝试original_image.jpg
                    image_path = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}" / "original_image.jpg"
            
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
            template_dir = Path("/home/nvidia/RVG_ws/templates") / workpiece_id / f"pose_{pose_id}"
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
                        result, error = ros2_node.set_robot_pose(
                            target_pose,
                            use_joints=True,
                            is_radian=False,
                            velocity=30.0,
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
            templates_dir = Path("/home/nvidia/RVG_ws/templates")
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
        """处理设置机器人位姿请求"""
        try:
            # 检查ROS2节点是否已初始化
            if ros2_node is None:
                print("[ERROR] ROS2节点未初始化（set_robot_pose）")
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
            is_radian = request_data.get('is_radian', False)
            velocity = request_data.get('velocity', 50.0)
            
            if len(target_pose) != 6:
                self.send_response(400)
                self.send_header('Content-type', 'application/json; charset=utf-8')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "target_pose必须包含6个元素"}, ensure_ascii=False).encode('utf-8'))
                return
            
            # 调用ROS2服务
            result, error = ros2_node.set_robot_pose(target_pose, use_joints, is_radian, velocity)
            
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
            print(f"[ERROR] 处理设置机器人位姿请求异常: {str(e)}\n{traceback.format_exc()}")
            self.send_response(500)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"处理请求异常: {str(e)}"}, ensure_ascii=False).encode('utf-8'))
    
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
    
    def save_geometric_features(self, debug_dir, metadata):
        """保存几何特征数据到文件（JSON和CSV格式）
        保存的数据包括：
        - 整体连通域面积
        - 大圆圆心、大圆半径
        - 小圆圆心、小圆半径
        """
        try:
            # 提取特征数据
            big_circle_features = metadata.get('big_circle_features', [])
            valve_circle_features = metadata.get('valve_circle_features', [])
            
            # 计算整体连通域面积（从 remove_green 步骤的结果图像）
            total_area = 0.0
            remove_green_path = debug_dir / "step_01_remove_green.jpg"
            if remove_green_path.exists():
                remove_green_img = cv2.imread(str(remove_green_path), cv2.IMREAD_GRAYSCALE)
                if remove_green_img is not None:
                    # 计算非零像素的数量（连通域面积）
                    total_area = float(np.count_nonzero(remove_green_img))
            
            # 准备几何特征数据
            geometric_features = {
                'timestamp': datetime.datetime.now().isoformat(),
                'total_area': total_area,  # 整体连通域面积（像素²）
                'big_circle': None,  # 大圆信息
                'valve_circle': None  # 小圆（阀体圆）信息
            }
            
            # 提取大圆信息（取第一个，如果有多个）
            if big_circle_features and len(big_circle_features) > 0:
                big_circle = big_circle_features[0]
                geometric_features['big_circle'] = {
                    'center_x': big_circle.get('center_x', 0.0),
                    'center_y': big_circle.get('center_y', 0.0),
                    'radius': big_circle.get('radius', 0.0),
                    'area': big_circle.get('area', 0.0)
                }
            
            # 提取小圆信息（取第一个，如果有多个）
            if valve_circle_features and len(valve_circle_features) > 0:
                valve_circle = valve_circle_features[0]
                geometric_features['valve_circle'] = {
                    'center_x': valve_circle.get('center_x', 0.0),
                    'center_y': valve_circle.get('center_y', 0.0),
                    'radius': valve_circle.get('radius', 0.0),
                    'area': valve_circle.get('area', 0.0)
                }
            
            # 保存为JSON格式
            json_path = debug_dir / "geometric_features.json"
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(geometric_features, f, indent=2, ensure_ascii=False)
            print(f"[INFO] 几何特征数据已保存到: {json_path}")
            
            # 保存为CSV格式（便于Excel等工具查看）
            csv_path = debug_dir / "geometric_features.csv"
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 写入表头
                writer.writerow([
                    '特征名称', '数值', '单位', '说明'
                ])
                # 写入数据
                writer.writerow(['整体连通域面积', f"{total_area:.2f}", '像素²', '去除绿色背景后的连通域总面积'])
                
                if geometric_features['big_circle']:
                    bc = geometric_features['big_circle']
                    writer.writerow(['大圆圆心X', f"{bc['center_x']:.2f}", '像素', '大圆圆心X坐标'])
                    writer.writerow(['大圆圆心Y', f"{bc['center_y']:.2f}", '像素', '大圆圆心Y坐标'])
                    writer.writerow(['大圆半径', f"{bc['radius']:.2f}", '像素', '大圆半径'])
                    writer.writerow(['大圆面积', f"{bc['area']:.2f}", '像素²', '大圆对应的轮廓面积'])
                
                if geometric_features['valve_circle']:
                    vc = geometric_features['valve_circle']
                    writer.writerow(['小圆圆心X', f"{vc['center_x']:.2f}", '像素', '小圆（阀体圆）圆心X坐标'])
                    writer.writerow(['小圆圆心Y', f"{vc['center_y']:.2f}", '像素', '小圆（阀体圆）圆心Y坐标'])
                    writer.writerow(['小圆半径', f"{vc['radius']:.2f}", '像素', '小圆（阀体圆）半径'])
                    writer.writerow(['小圆面积', f"{vc['area']:.2f}", '像素²', '小圆对应的轮廓面积'])
            
            print(f"[INFO] 几何特征数据已保存到: {csv_path}")
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] 保存几何特征数据失败: {error_detail}")
    
    def load_config_params(self, config_path):
        """加载配置文件参数"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            return config
        except Exception as e:
            print(f"加载配置文件失败: {e}，使用默认参数")
            return {}
    
    def load_prev_step_result(self, debug_dir, prev_step_index, original_image):
        """加载前一步的结果"""
        if prev_step_index < 0:
            print(f"[DEBUG] load_prev_step_result: prev_step_index={prev_step_index} < 0, returning None")
            return None
        
        # 尝试加载前一步的结果图像
        prev_step_ids = ['original', 'remove_green', 'big_circle', 'head_only', 'valve_circle', 'crop_and_rotate', 'binarize', 'morphology', 'components']
        print(f"[DEBUG] load_prev_step_result: prev_step_index={prev_step_index}, total_steps={len(prev_step_ids)}")
        
        if prev_step_index < len(prev_step_ids):
            prev_step_id = prev_step_ids[prev_step_index]
            prev_result_path = debug_dir / f"step_{prev_step_index:02d}_{prev_step_id}.jpg"
            print(f"[DEBUG] Trying to load: {prev_result_path}, exists={prev_result_path.exists()}")
            
            if prev_result_path.exists():
                # 根据步骤类型决定读取方式
                if prev_step_id == 'original':
                    result = cv2.imread(str(prev_result_path), cv2.IMREAD_COLOR)
                    print(f"[DEBUG] Loaded original image: {result is not None}")
                    return result
                elif prev_step_id == 'remove_green':
                    # remove_green 保存的是 BGR 图像，但我们需要灰度图
                    result = cv2.imread(str(prev_result_path), cv2.IMREAD_COLOR)
                    if result is not None and len(result.shape) == 3:
                        # 转换为灰度图
                        result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                    print(f"[DEBUG] Loaded remove_green image: {result is not None}, shape={result.shape if result is not None else None}")
                    return result
                elif prev_step_id == 'big_circle':
                    # big_circle 输出的是拼接的彩色图像，需要提取左侧或右侧
                    # 但为了后续处理，我们使用原始的二值图像
                    # 这里返回灰度图，实际上应该从 remove_green 步骤获取
                    # 为了简化，我们直接读取 remove_green 的结果
                    remove_green_path = debug_dir / f"step_{prev_step_index-1:02d}_remove_green.jpg"
                    print(f"[DEBUG] big_circle step: trying to load remove_green from {remove_green_path}")
                    if remove_green_path.exists():
                        result = cv2.imread(str(remove_green_path), cv2.IMREAD_COLOR)
                        if result is not None and len(result.shape) == 3:
                            result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                        print(f"[DEBUG] Loaded remove_green for big_circle: {result is not None}, shape={result.shape if result is not None else None}")
                        return result
                    result = cv2.imread(str(prev_result_path), cv2.IMREAD_COLOR)
                    if result is not None and len(result.shape) == 3:
                        result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                    print(f"[DEBUG] Loaded big_circle image directly: {result is not None}, shape={result.shape if result is not None else None}")
                    return result
                else:
                    result = cv2.imread(str(prev_result_path), cv2.IMREAD_GRAYSCALE)
                    print(f"[DEBUG] Loaded {prev_step_id} image: {result is not None}")
                    return result
            else:
                print(f"[WARN] load_prev_step_result: File not found: {prev_result_path}")
                # 如果找不到指定步骤的文件，尝试查找 remove_green（如果 prev_step_index >= 1）
                if prev_step_index >= 1:
                    remove_green_path = debug_dir / "step_01_remove_green.jpg"
                    print(f"[DEBUG] Fallback: trying to load remove_green from {remove_green_path}")
                    if remove_green_path.exists():
                        result = cv2.imread(str(remove_green_path), cv2.IMREAD_COLOR)
                        if result is not None and len(result.shape) == 3:
                            result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                        print(f"[DEBUG] Fallback loaded remove_green: {result is not None}, shape={result.shape if result is not None else None}")
                        return result
        else:
            print(f"[WARN] load_prev_step_result: prev_step_index {prev_step_index} out of range")
        
        return None
    
    def process_remove_green(self, image, debug_dir, step_index, config_params):
        """去除绿色背景 - 使用配置文件参数"""
        # 获取背景去除参数
        bg_params = config_params.get('preprocess', {}).get('background', {})
        
        # 转换到多个颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_channels = cv2.split(hsv)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        
        # 边缘采样区域比例
        border_ratio = bg_params.get('border_ratio', 0.15)
        margin = max(10, int(border_ratio * min(image.shape[0], image.shape[1])))
        margin = min(margin, min(image.shape[0], image.shape[1]) // 3)
        
        # 创建边缘掩码
        border_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        if margin > 0:
            border_mask[:margin, :] = 255
            border_mask[-margin:, :] = 255
            border_mask[:, :margin] = 255
            border_mask[:, -margin:] = 255
        
        if np.count_nonzero(border_mask) < image.shape[0] * image.shape[1] * 0.1:
            border_mask.fill(255)
        
        # 计算边缘区域的统计信息
        mean_hsv, std_hsv = cv2.meanStdDev(hsv, mask=border_mask)
        mean_lab, std_lab = cv2.meanStdDev(lab, mask=border_mask)
        
        mean_hsv = mean_hsv.flatten()
        std_hsv = std_hsv.flatten()
        mean_lab = mean_lab.flatten()
        std_lab = std_lab.flatten()
        
        # 使用直方图分析找到主要背景色（如果启用）
        use_histogram = bg_params.get('use_histogram', True)
        if use_histogram:
            border_h = hsv_channels[0][border_mask > 0]
            if len(border_h) > 0:
                hist, bins = np.histogram(border_h, bins=180, range=(0, 180))
                peak_h = np.argmax(hist)
                mean_hsv[0] = peak_h
                
                # 使用中位数而不是平均值（更鲁棒）
                border_s = hsv_channels[1][border_mask > 0]
                border_v = hsv_channels[2][border_mask > 0]
                if len(border_s) > 0:
                    mean_hsv[1] = np.median(border_s)
                if len(border_v) > 0:
                    mean_hsv[2] = np.median(border_v)
        
        # 计算自适应阈值
        hue_margin = bg_params.get('hue_margin', 20.0)
        hue_std_mul = bg_params.get('hue_std_mul', 2.5)
        sat_margin = bg_params.get('sat_margin', 40.0)
        sat_std_mul = bg_params.get('sat_std_mul', 2.5)
        val_margin = bg_params.get('val_margin', 50.0)
        val_std_mul = bg_params.get('val_std_mul', 2.5)
        
        hue_threshold = max(15.0, min(60.0, hue_margin + hue_std_mul * std_hsv[0]))
        sat_threshold = max(25.0, min(120.0, sat_margin + sat_std_mul * std_hsv[1]))
        val_threshold = max(30.0, min(140.0, val_margin + val_std_mul * std_hsv[2]))
        
        # HSV空间匹配
        hue_diff = np.abs(hsv_channels[0].astype(np.float32) - mean_hsv[0])
        hue_wrap = np.abs(hue_diff - 180)
        hue_diff = np.minimum(hue_diff, hue_wrap)
        
        sat_diff = np.abs(hsv_channels[1].astype(np.float32) - mean_hsv[1])
        val_diff = np.abs(hsv_channels[2].astype(np.float32) - mean_hsv[2])
        
        hsv_mask = (hue_diff < hue_threshold) & (sat_diff < sat_threshold) & (val_diff < val_threshold)
        
        # Lab空间匹配
        lab_channels = cv2.split(lab)
        lab_distance = np.zeros(image.shape[:2], dtype=np.float32)
        for i in range(3):
            diff = np.abs(lab_channels[i].astype(np.float32) - mean_lab[i])
            denom = max(2.0, float(std_lab[i]))
            lab_distance += diff / denom
        lab_distance /= 3.0
        
        lab_threshold = max(2.0, min(8.0, bg_params.get('lab_threshold', 4.0)))
        lab_mask = lab_distance < lab_threshold
        
        # 融合HSV和Lab结果
        bg_mask = hsv_mask.astype(np.uint8) * 255
        bg_mask = cv2.bitwise_and(bg_mask, (lab_mask.astype(np.uint8) * 255))
        
        # 经典绿色范围检测（作为补充）
        enable_classic = bg_params.get('enable_classic_hsv', True)
        if enable_classic:
            lower_green = np.array([25, 25, 25])
            upper_green = np.array([95, 255, 255])
            classic_green_mask = cv2.inRange(hsv, lower_green, upper_green)
            bg_mask = cv2.bitwise_or(bg_mask, classic_green_mask)
        
        # 只在检测到的背景区域与边缘区域重叠时，才标记边缘为背景
        # 这样可以避免工件延伸到边缘时被误删
        # 计算边缘区域中检测为背景的比例
        border_bg_ratio_threshold = bg_params.get('border_bg_ratio_threshold', 0.7)
        border_bg_count = np.count_nonzero(bg_mask & border_mask)
        border_total = np.count_nonzero(border_mask)
        if border_total > 0:
            border_bg_ratio = border_bg_count / border_total
            # 如果边缘区域中超过阈值比例被检测为背景，才强制标记整个边缘为背景
            if border_bg_ratio > border_bg_ratio_threshold:
                bg_mask = cv2.bitwise_or(bg_mask, border_mask)
        
        # 形态学清理背景掩码
        cleanup_kernel_size = bg_params.get('cleanup_kernel', 9)
        cleanup_dilate_iter = bg_params.get('cleanup_dilate_iterations', 1)
        if cleanup_kernel_size % 2 == 0:
            cleanup_kernel_size += 1
        if cleanup_kernel_size >= 3:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cleanup_kernel_size, cleanup_kernel_size))
            bg_mask = cv2.dilate(bg_mask, kernel, iterations=cleanup_dilate_iter)
            bg_mask = cv2.erode(bg_mask, kernel, iterations=1)
        
        # 获取前景掩码
        non_green = cv2.bitwise_not(bg_mask)
        
        # 去除前景中的小噪声（连通域分析）
        min_noise_area = bg_params.get('min_noise_area', 100)
        if min_noise_area > 0:
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(non_green, connectivity=8)
            cleaned_fg = np.zeros_like(non_green)
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                if area >= min_noise_area:
                    cleaned_fg[labels == i] = 255
            non_green = cleaned_fg
        
        # 前景形态学处理
        erode_before_dilate = bg_params.get('erode_before_dilate', True)
        if erode_before_dilate:
            fg_erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            non_green = cv2.erode(non_green, fg_erode_kernel, iterations=1)
        
        fg_close_kernel_size = bg_params.get('foreground_close_kernel', 11)
        fg_close_iterations = bg_params.get('foreground_close_iterations', 2)
        if fg_close_kernel_size % 2 == 0:
            fg_close_kernel_size += 1
        if fg_close_kernel_size >= 3:
            fg_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (fg_close_kernel_size, fg_close_kernel_size))
            non_green = cv2.morphologyEx(non_green, cv2.MORPH_CLOSE, fg_kernel, iterations=fg_close_iterations)
        
        # 中值滤波去除小噪声
        median_ksize = bg_params.get('median_ksize', 7)
        if median_ksize % 2 == 0:
            median_ksize += 1
        if median_ksize >= 3:
            non_green = cv2.medianBlur(non_green, median_ksize)
        
        # 转换为3通道用于显示
        result = cv2.cvtColor(non_green, cv2.COLOR_GRAY2BGR)
        return result
    
    def process_big_circle(self, original_image, prev_result, debug_dir, step_index, config_params):
        """查找大圆（整体外接圆）
        
        Args:
            original_image: 原始RGB图像的副本（在主函数中已创建，这里是只读参考）
            prev_result: 前一步的处理结果（去除绿色背景后的二值图像）
            debug_dir: 调试输出目录
            step_index: 步骤索引
            config_params: 配置参数
        """
        if prev_result is None:
            print(f"[ERROR] process_big_circle: prev_result is None for step_index={step_index}")
            return None, []
        
        if original_image is None or original_image.size == 0:
            print(f"[ERROR] process_big_circle: original_image is None or empty")
            return None, []
        
        # 验证传入的原始图像是RGB彩色图像
        if len(original_image.shape) != 3 or original_image.shape[2] != 3:
            return None, []
        
        try:
            # ========== 步骤1: 处理二值图像（独立变量） ==========
            # prev_result 是去除绿色背景后的结果（可能是BGR或灰度）
            if len(prev_result.shape) == 3:
                binary_mask = cv2.cvtColor(prev_result, cv2.COLOR_BGR2GRAY)
            else:
                binary_mask = prev_result.copy()
            
            if binary_mask is None or binary_mask.size == 0:
                return None, []
            
            # ========== 步骤2: 获取配置参数 ==========
            big_circle_params = config_params.get('preprocess', {}).get('big_circle', {})
            combine_contours = big_circle_params.get('combine_contours', True)
            min_area = big_circle_params.get('min_area', 100)
            min_radius = big_circle_params.get('min_radius', 10)
            max_radius = big_circle_params.get('max_radius', 5000)
            
            # 可视化参数
            circle_color = big_circle_params.get('circle_color', [0, 255, 0])
            center_color = big_circle_params.get('center_color', [0, 255, 0])
            circle_thickness = big_circle_params.get('circle_thickness', 2)
            center_radius = big_circle_params.get('center_radius', 5)
            font_scale = big_circle_params.get('font_scale', 0.6)
            font_thickness = big_circle_params.get('font_thickness', 1)
            
            # 转换为BGR格式（如果是列表格式）
            if isinstance(circle_color, list):
                circle_color = tuple(circle_color)
            if isinstance(center_color, list):
                center_color = tuple(center_color)
            
            # ========== 步骤3: 查找轮廓 ==========
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 过滤轮廓
            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= min_area:
                    valid_contours.append(contour)
            
            # ========== 步骤4: 准备可视化图像（从原始图像副本再创建可视化副本） ==========
            # 左边：从原始RGB图像副本创建可视化副本（用于绘制，不修改original_image）
            original_vis = original_image.copy()
            
            # 右边：二值图像的BGR副本（用于绘制）
            binary_bgr = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
            binary_vis = binary_bgr.copy()
            
            if not valid_contours:
                # 没有找到有效轮廓，返回原图和二值图的拼接
                print(f"[INFO] 找大圆: 未找到有效轮廓（最小面积: {min_area}）")
                result = np.hstack([original_vis, binary_vis])
                return result, []
            
            # 辅助函数：从三个点计算圆的圆心和半径
            def circle_from_three_points(p1, p2, p3):
                """计算通过三个点的圆的圆心和半径"""
                x1, y1 = float(p1[0]), float(p1[1])
                x2, y2 = float(p2[0]), float(p2[1])
                x3, y3 = float(p3[0]), float(p3[1])
                
                # 计算三个点之间的距离
                d12 = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                d23 = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
                d31 = np.sqrt((x1 - x3)**2 + (y1 - y3)**2)
                
                # 检查三点是否共线
                if abs((y2 - y1) * (x3 - x1) - (y3 - y1) * (x2 - x1)) < 1e-10:
                    # 三点共线，返回中点作为圆心，最大距离的一半作为半径
                    center_x = (x1 + x2 + x3) / 3.0
                    center_y = (y1 + y2 + y3) / 3.0
                    radius = max(d12, d23, d31) / 2.0
                    return (center_x, center_y), radius
                
                # 使用垂直平分线方法计算圆心
                # 计算中点
                mid12_x, mid12_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                mid23_x, mid23_y = (x2 + x3) / 2.0, (y2 + y3) / 2.0
                
                # 计算垂直平分线的方向向量
                # P1P2的垂直向量
                v12_x, v12_y = -(y2 - y1), (x2 - x1)
                # P2P3的垂直向量
                v23_x, v23_y = -(y3 - y2), (x3 - x2)
                
                # 求解两条垂直平分线的交点（圆心）
                # 直线1: mid12 + t * v12
                # 直线2: mid23 + s * v23
                # 求解: mid12 + t * v12 = mid23 + s * v23
                
                # 构建线性方程组
                A = np.array([[v12_x, -v23_x],
                              [v12_y, -v23_y]])
                b = np.array([mid23_x - mid12_x,
                              mid23_y - mid12_y])
                
                try:
                    t, s = np.linalg.solve(A, b)
                    center_x = mid12_x + t * v12_x
                    center_y = mid12_y + t * v12_y
                    radius = np.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
                    return (center_x, center_y), radius
                except np.linalg.LinAlgError:
                    # 如果求解失败，使用最小外接圆作为备选
                    points = np.array([[x1, y1], [x2, y2], [x3, y3]])
                    (center_x, center_y), radius = cv2.minEnclosingCircle(points)
                    return (center_x, center_y), radius
            
            # 辅助函数：从轮廓中选择三个最远的点（优化版本，使用凸包加速）
            def select_three_farthest_points(contour):
                """从轮廓中选择三个最远的点（形成最大三角形）
                优化：先计算凸包，然后从凸包中选择，大大减少计算量
                """
                if len(contour) < 3:
                    return None
                
                # 将轮廓点展平
                points = contour.reshape(-1, 2).astype(np.float32)
                
                # 如果点数太多，先计算凸包来减少点数
                if len(points) > 50:
                    hull = cv2.convexHull(points.reshape(-1, 1, 2))
                    hull_points = hull.reshape(-1, 2)
                    if len(hull_points) >= 3:
                        points = hull_points
                
                # 如果点数仍然较多，进行采样（每N个点取一个）
                if len(points) > 100:
                    step = len(points) // 100
                    points = points[::step]
                
                # 使用向量化计算找到最远的两个点（优化：使用平方距离避免开方）
                n = len(points)
                if n < 3:
                    return None
                
                # 计算所有点对之间的平方距离
                points_expanded = points[:, np.newaxis, :]  # (n, 1, 2)
                diff = points_expanded - points  # (n, n, 2)
                sq_distances = np.sum(diff * diff, axis=2)  # (n, n)
                
                # 找到最大距离的两个点
                max_idx = np.unravel_index(np.argmax(sq_distances), sq_distances.shape)
                p1_idx, p2_idx = max_idx[0], max_idx[1]
                p1, p2 = points[p1_idx], points[p2_idx]
                
                # 找到距离这两个点最远的第三个点（使用平方距离）
                sq_dist1 = np.sum((points - p1) ** 2, axis=1)
                sq_dist2 = np.sum((points - p2) ** 2, axis=1)
                sum_sq_dist = sq_dist1 + sq_dist2
                sum_sq_dist[p1_idx] = 0  # 排除已选的点
                sum_sq_dist[p2_idx] = 0
                p3_idx = np.argmax(sum_sq_dist)
                
                return points[p1_idx], points[p2_idx], points[p3_idx]
            
            # 存储所有检测到的圆
            circles_info = []
            
            if combine_contours and len(valid_contours) > 1:
                # 合并所有轮廓的点（优化：直接使用凸包，避免合并所有点）
                # 先计算每个轮廓的凸包，然后合并凸包点
                hull_points_list = []
                for contour in valid_contours:
                    if len(contour) >= 3:
                        hull = cv2.convexHull(contour)
                        hull_points_list.append(hull.reshape(-1, 2))
                
                if hull_points_list:
                    # 合并所有凸包点
                    all_hull_points = np.vstack(hull_points_list).astype(np.float32)
                    
                    # 如果点数仍然较多，再次计算整体凸包
                    if len(all_hull_points) > 100:
                        all_hull_points = cv2.convexHull(all_hull_points.reshape(-1, 1, 2)).reshape(-1, 2)
                    
                    # 从凸包中选择三个最远的点
                    if len(all_hull_points) >= 3:
                        three_points = select_three_farthest_points(all_hull_points.reshape(-1, 1, 2))
                        if three_points is not None:
                            (center_x, center_y), radius = circle_from_three_points(three_points[0], three_points[1], three_points[2])
                            center = (int(center_x), int(center_y))
                            radius = int(radius)
                            
                            # 验证半径范围
                            if min_radius <= radius <= max_radius:
                                circles_info.append({
                                    'center': center,
                                    'radius': radius,
                                    'area': sum(cv2.contourArea(c) for c in valid_contours)
                                })
            else:
                # 为每个轮廓单独计算最小外接圆（使用三个点）
                for contour in valid_contours:
                    if len(contour) < 3:
                        continue
                    
                    # 从轮廓中选择三个最远的点
                    three_points = select_three_farthest_points(contour)
                    if three_points is None:
                        continue
                    
                    # 使用三个点计算圆
                    (center_x, center_y), radius = circle_from_three_points(three_points[0], three_points[1], three_points[2])
                    center = (int(center_x), int(center_y))
                    radius = int(radius)
                    
                    # 验证半径范围
                    if min_radius <= radius <= max_radius:
                        circles_info.append({
                            'center': center,
                            'radius': radius,
                            'area': cv2.contourArea(contour)
                        })
            
            # 准备特征数据列表
            features = []
            
            # 在原图和二值图上绘制圆和中心点
            for i, circle_info in enumerate(circles_info):
                center = circle_info['center']
                radius = circle_info['radius']
                area = circle_info['area']
                
                # 打印日志（简化）
                print(f"[INFO] 圆 {i+1}: 圆心=({center[0]}, {center[1]}), 半径={radius}")
                
                # 保存特征数据
                feature_data = {
                    'type': 'big_circle',
                    'center_x': float(center[0]),
                    'center_y': float(center[1]),
                    'radius': float(radius),
                    'area': float(area)
                }
                features.append(feature_data)
                
                # 在原图上绘制
                cv2.circle(original_vis, center, radius, circle_color, circle_thickness)
                cv2.circle(original_vis, center, center_radius, center_color, -1)
                
                # 在二值图上绘制
                cv2.circle(binary_vis, center, radius, circle_color, circle_thickness)
                cv2.circle(binary_vis, center, center_radius, center_color, -1)
                
                # 添加坐标文字
                label_text = f"({center[0]},{center[1]}) r={radius}"
                # 计算文字位置（在圆的上方）
                text_y = max(center[1] - radius - 10, 20)
                text_x = center[0] - 50
                
                # 在原图上添加文字
                cv2.putText(original_vis, label_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, circle_color, font_thickness)
                
                # 在二值图上添加文字
                cv2.putText(binary_vis, label_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, circle_color, font_thickness)
            
            # 打印汇总信息
            if features:
                print(f"[INFO] 找大圆 - 共检测到 {len(features)} 个大圆")
            else:
                print(f"[WARN] 找大圆 - 未检测到符合条件的大圆（半径范围: {min_radius}-{max_radius}）")
            
            # 确保两张图像高度一致（用于拼接）
            h1, w1 = original_vis.shape[:2]
            h2, w2 = binary_vis.shape[:2]
            
            if h1 != h2:
                # 调整高度使之一致
                target_h = max(h1, h2)
                if h1 < target_h:
                    pad = np.zeros((target_h - h1, w1, 3), dtype=np.uint8)
                    original_vis = np.vstack([original_vis, pad])
                if h2 < target_h:
                    pad = np.zeros((target_h - h2, w2, 3), dtype=np.uint8)
                    binary_vis = np.vstack([binary_vis, pad])
            
            # 左右拼接：左边原图，右边二值图
            result = np.hstack([original_vis, binary_vis])
            
            return result, features
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_big_circle 处理异常: {error_detail}")
            raise
    
    def process_head_only(self, binary_image, debug_dir, step_index, config_params):
        """保留头部，去除尾巴
        通过腐蚀去除细尾巴，然后膨胀恢复头部区域
        """
        if binary_image is None:
            print(f"[ERROR] process_head_only: binary_image is None")
            return None
        
        if binary_image.size == 0:
            print(f"[ERROR] process_head_only: binary_image is empty")
            return None
        
        try:
            # 获取配置参数
            head_only_params = config_params.get('preprocess', {}).get('head_only', {})
            erode_kernel_size = head_only_params.get('erode_kernel', 15)
            erode_iterations = head_only_params.get('erode_iterations', 3)
            dilate_kernel_size = head_only_params.get('dilate_kernel', 15)
            dilate_iterations = head_only_params.get('dilate_iterations', 3)
            min_area = head_only_params.get('min_area', 500)
            
            # 转换为灰度图
            if len(binary_image.shape) == 3:
                binary = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
            else:
                binary = binary_image.copy()
            
            # 确保数据类型为 uint8
            if binary.dtype != np.uint8:
                binary = binary.astype(np.uint8)
            
            # 验证图像尺寸
            h, w = binary.shape[:2]
            if h == 0 or w == 0:
                print(f"[ERROR] process_head_only: Invalid image size: {binary.shape}")
                return None
            
            # 确保图像尺寸足够大
            if h < 3 or w < 3:
                print(f"[ERROR] process_head_only: Image too small: {binary.shape}")
                return None
            
            # 确保核大小不超过图像尺寸，并且为奇数
            max_kernel_size = min(h, w) - 2
            if erode_kernel_size > max_kernel_size:
                erode_kernel_size = max_kernel_size if max_kernel_size > 0 else 3
                print(f"[WARN] process_head_only: erode_kernel_size adjusted to {erode_kernel_size}")
            if erode_kernel_size % 2 == 0:
                erode_kernel_size = max(3, erode_kernel_size - 1)
            
            if dilate_kernel_size > max_kernel_size:
                dilate_kernel_size = max_kernel_size if max_kernel_size > 0 else 3
                print(f"[WARN] process_head_only: dilate_kernel_size adjusted to {dilate_kernel_size}")
            if dilate_kernel_size % 2 == 0:
                dilate_kernel_size = max(3, dilate_kernel_size - 1)
            
            # 确保核大小至少为3
            erode_kernel_size = max(3, erode_kernel_size)
            dilate_kernel_size = max(3, dilate_kernel_size)
            
            # 确保迭代次数为正数
            erode_iterations = max(1, int(erode_iterations))
            dilate_iterations = max(1, int(dilate_iterations))
            
            print(f"[DEBUG] process_head_only: Input shape={binary.shape}, dtype={binary.dtype}")
            print(f"[DEBUG] process_head_only: erode_kernel={erode_kernel_size}, iterations={erode_iterations}")
            print(f"[DEBUG] process_head_only: dilate_kernel={dilate_kernel_size}, iterations={dilate_iterations}")
            
            # 步骤1: 腐蚀操作，去除细尾巴
            erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(erode_kernel_size), int(erode_kernel_size)))
            eroded = cv2.erode(binary, erode_kernel, iterations=erode_iterations)
            
            # 步骤2: 过滤小的连通域，只保留头部（面积较大的连通域）
            # 确保 eroded 是 uint8 类型
            if eroded.dtype != np.uint8:
                eroded = eroded.astype(np.uint8)
            
            contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            head_mask = np.zeros_like(eroded, dtype=np.uint8)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= min_area:
                    cv2.drawContours(head_mask, [contour], -1, 255, -1)
            
            # 步骤3: 膨胀操作，恢复头部区域
            dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(dilate_kernel_size), int(dilate_kernel_size)))
            dilated = cv2.dilate(head_mask, dilate_kernel, iterations=dilate_iterations)
            
            # 步骤4: 转换为3通道用于显示
            result = cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR)
            
            # 打印处理信息
            num_contours = len([c for c in contours if cv2.contourArea(c) >= min_area])
            print(f"[INFO] 保留头部 - 腐蚀核大小: {erode_kernel_size}, 迭代次数: {erode_iterations}")
            print(f"[INFO] 保留头部 - 膨胀核大小: {dilate_kernel_size}, 迭代次数: {dilate_iterations}")
            print(f"[INFO] 保留头部 - 保留了 {num_contours} 个连通域（面积 >= {min_area}）")
            
            return result
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_head_only 处理异常: {error_detail}")
            raise
    
    def process_valve_circle(self, original_image, head_only_binary, debug_dir, step_index, config_params, big_circle_features=None):
        """找阀体外接圆圆心
        从head_only步骤的结果中选择面积最大的连通域，进行膨胀，然后计算最小外接圆
        可视化：左边是原图（包含大圆和阀体圆的标记），右边是连通域图像
        """
        if head_only_binary is None:
            print(f"[ERROR] process_valve_circle: head_only_binary is None")
            return None, []
        
        if original_image is None or original_image.size == 0:
            print(f"[ERROR] process_valve_circle: original_image is None or empty")
            return None, []
        
        if len(original_image.shape) != 3 or original_image.shape[2] != 3:
            print(f"[ERROR] process_valve_circle: original_image is not 3-channel BGR")
            return None, []
        
        try:
            # 获取配置参数
            valve_circle_params = config_params.get('preprocess', {}).get('valve_circle', {})
            dilate_kernel_size = valve_circle_params.get('dilate_kernel', 15)
            dilate_iterations = valve_circle_params.get('dilate_iterations', 3)
            min_area = valve_circle_params.get('min_area', 500)
            
            # 可视化参数
            circle_color = valve_circle_params.get('circle_color', [0, 0, 255])
            center_color = valve_circle_params.get('center_color', [0, 0, 255])
            circle_thickness = valve_circle_params.get('circle_thickness', 2)
            center_radius = valve_circle_params.get('center_radius', 5)
            font_scale = valve_circle_params.get('font_scale', 0.6)
            font_thickness = valve_circle_params.get('font_thickness', 1)
            
            # 转换为BGR格式（如果是列表格式）
            if isinstance(circle_color, list):
                circle_color = tuple(circle_color)
            if isinstance(center_color, list):
                center_color = tuple(center_color)
            
            # 转换为灰度图
            if len(head_only_binary.shape) == 3:
                binary = cv2.cvtColor(head_only_binary, cv2.COLOR_BGR2GRAY)
            else:
                binary = head_only_binary.copy()
            
            # 确保数据类型为 uint8
            if binary.dtype != np.uint8:
                binary = binary.astype(np.uint8)
            
            # 验证图像尺寸
            h, w = binary.shape[:2]
            if h == 0 or w == 0:
                print(f"[ERROR] process_valve_circle: Invalid image size: {binary.shape}")
                return None, []
            
            # 步骤1: 查找连通域，选择面积最大的那个
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                print(f"[WARN] process_valve_circle: No contours found")
                # 返回原始图像
                return original_image.copy(), []
            
            # 过滤并找到面积最大的连通域
            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= min_area:
                    valid_contours.append((contour, area))
            
            if not valid_contours:
                print(f"[WARN] process_valve_circle: No valid contours (area >= {min_area})")
                return original_image.copy(), []
            
            # 选择面积最大的连通域
            largest_contour, largest_area = max(valid_contours, key=lambda x: x[1])
            print(f"[INFO] 找阀体外接圆 - 选择了面积最大的连通域: 面积={largest_area:.2f}")
            
            # 步骤2: 创建只包含最大连通域的掩码
            valve_mask = np.zeros_like(binary, dtype=np.uint8)
            cv2.drawContours(valve_mask, [largest_contour], -1, 255, -1)
            
            # 步骤3: 对选中的连通域进行膨胀
            if dilate_kernel_size % 2 == 0:
                dilate_kernel_size += 1
            # 确保核大小不超过图像尺寸
            max_kernel_size = min(h, w) - 2
            if dilate_kernel_size > max_kernel_size:
                dilate_kernel_size = max_kernel_size if max_kernel_size > 0 else 3
            dilate_kernel_size = max(3, dilate_kernel_size)
            
            dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(dilate_kernel_size), int(dilate_kernel_size)))
            dilated_mask = cv2.dilate(valve_mask, dilate_kernel, iterations=dilate_iterations)
            
            # 步骤4: 从膨胀后的掩码中提取轮廓，计算最小外接圆
            dilated_contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not dilated_contours:
                print(f"[WARN] process_valve_circle: No contours found after dilation")
                # 使用原始轮廓计算外接圆
                (center_x, center_y), radius = cv2.minEnclosingCircle(largest_contour)
            else:
                # 使用膨胀后的最大轮廓计算外接圆
                dilated_contour = max(dilated_contours, key=cv2.contourArea)
                (center_x, center_y), radius = cv2.minEnclosingCircle(dilated_contour)
            center = (int(center_x), int(center_y))
            radius = int(radius)
            
            # 步骤5: 准备可视化图像
            # 左边：原始RGB图像（用于绘制大圆和阀体圆）
            original_vis = original_image.copy()
            
            # 获取 big_circle 的可视化参数（用于绘制大圆）
            big_circle_params = config_params.get('preprocess', {}).get('big_circle', {})
            big_circle_color = big_circle_params.get('circle_color', [0, 255, 0])
            big_center_color = big_circle_params.get('center_color', [0, 255, 0])
            big_circle_thickness = big_circle_params.get('circle_thickness', 2)
            big_center_radius = big_circle_params.get('center_radius', 5)
            big_font_scale = big_circle_params.get('font_scale', 0.6)
            big_font_thickness = big_circle_params.get('font_thickness', 1)
            
            # 转换为BGR格式（如果是列表格式）
            if isinstance(big_circle_color, list):
                big_circle_color = tuple(big_circle_color)
            if isinstance(big_center_color, list):
                big_center_color = tuple(big_center_color)
            
            # 在左侧原图上绘制大圆（如果存在）
            if big_circle_features:
                for i, feat in enumerate(big_circle_features):
                    big_center = (int(feat['center_x']), int(feat['center_y']))
                    big_radius = int(feat['radius'])
                    cv2.circle(original_vis, big_center, big_radius, big_circle_color, big_circle_thickness)
                    cv2.circle(original_vis, big_center, big_center_radius, big_center_color, -1)
                    # 添加坐标文字
                    big_label_text = f"大圆({big_center[0]},{big_center[1]}) r={big_radius}"
                    big_text_y = max(big_center[1] - big_radius - 10, 20)
                    big_text_x = big_center[0] - 50
                    cv2.putText(original_vis, big_label_text, (big_text_x, big_text_y),
                               cv2.FONT_HERSHEY_SIMPLEX, big_font_scale, big_circle_color, big_font_thickness)
            
            # 在左侧原图上绘制阀体圆（红色）
            cv2.circle(original_vis, center, radius, circle_color, circle_thickness)
            cv2.circle(original_vis, center, center_radius, center_color, -1)
            
            # 添加阀体圆坐标文字
            label_text = f"阀体({center[0]},{center[1]}) r={radius}"
            text_y = max(center[1] - radius - 10, 20)
            text_x = center[0] - 50
            cv2.putText(original_vis, label_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, circle_color, font_thickness)
            
            # 右边：连通域图像（显示膨胀后的掩码）
            # 将膨胀后的掩码转换为3通道BGR图像
            dilated_bgr = cv2.cvtColor(dilated_mask, cv2.COLOR_GRAY2BGR)
            
            # 在连通域图像上绘制阀体圆（红色）
            cv2.circle(dilated_bgr, center, radius, circle_color, circle_thickness)
            cv2.circle(dilated_bgr, center, center_radius, center_color, -1)
            cv2.putText(dilated_bgr, label_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, circle_color, font_thickness)
            
            # 添加标签文字
            cv2.putText(original_vis, "ORIGINAL RGB", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(dilated_bgr, "VALVE MASK", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # 确保两张图像高度一致（用于拼接）
            h1, w1 = original_vis.shape[:2]
            h2, w2 = dilated_bgr.shape[:2]
            
            if h1 != h2:
                # 调整高度使之一致
                target_h = max(h1, h2)
                if h1 < target_h:
                    pad = np.zeros((target_h - h1, w1, 3), dtype=np.uint8)
                    original_vis = np.vstack([original_vis, pad])
                if h2 < target_h:
                    pad = np.zeros((target_h - h2, w2, 3), dtype=np.uint8)
                    dilated_bgr = np.vstack([dilated_bgr, pad])
            
            # 左右拼接：左边原图（包含大圆和阀体圆），右边连通域图像
            result_image = np.hstack([original_vis, dilated_bgr])
            
            # 步骤6: 保存特征数据
            features = [{
                'type': 'valve_circle',
                'center_x': float(center[0]),
                'center_y': float(center[1]),
                'radius': float(radius),
                'area': float(largest_area)
            }]
            
            print(f"[INFO] 找阀体外接圆 - 圆心=({center[0]}, {center[1]}), 半径={radius}, 面积={largest_area:.2f}")
            
            return result_image, features
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_valve_circle 处理异常: {error_detail}")
            raise
    
    def process_crop_region(self, original_image, big_circle_feature, remove_green_mask, debug_dir, step_index, config_params):
        """裁剪区域并去除背景
        以大圆圆心作为矩形的中心，大圆的直径作为矩形的边长，从原始图像中裁剪出空间区域
        通过连通域掩膜去除非工件区域（背景用白色占位）
        """
        if original_image is None or original_image.size == 0:
            print(f"[ERROR] process_crop_region: original_image is None or empty")
            return None
        
        if len(original_image.shape) != 3 or original_image.shape[2] != 3:
            print(f"[ERROR] process_crop_region: original_image is not 3-channel BGR")
            return None
        
        if big_circle_feature is None:
            print(f"[ERROR] process_crop_region: big_circle_feature is None")
            return None
        
        try:
            # 获取配置参数
            crop_params = config_params.get('preprocess', {}).get('crop_region', {})
            background_color = crop_params.get('background_color', [255, 255, 255])
            if isinstance(background_color, list):
                background_color = tuple(background_color)
            
            # 获取大圆信息
            big_center_x = float(big_circle_feature.get('center_x', 0))
            big_center_y = float(big_circle_feature.get('center_y', 0))
            big_radius = float(big_circle_feature.get('radius', 0))
            
            # 计算裁剪区域（以大圆圆心为中心，边长为大圆直径的正方形）
            side_length = int(big_radius * 2)  # 大圆直径
            half_side = side_length // 2
            
            # 计算裁剪区域的左上角坐标
            x1 = int(big_center_x - half_side)
            y1 = int(big_center_y - half_side)
            x2 = x1 + side_length
            y2 = y1 + side_length
            
            # 获取图像尺寸
            img_h, img_w = original_image.shape[:2]
            
            # 确保裁剪区域在图像范围内
            crop_x1 = max(0, x1)
            crop_y1 = max(0, y1)
            crop_x2 = min(img_w, x2)
            crop_y2 = min(img_h, y2)
            
            # 计算实际裁剪尺寸
            crop_w = crop_x2 - crop_x1
            crop_h = crop_y2 - crop_y1
            
            # 从原始图像中裁剪
            cropped_image = original_image[crop_y1:crop_y2, crop_x1:crop_x2].copy()
            
            # 如果裁剪区域超出图像边界，需要创建更大的画布
            if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
                # 创建目标大小的白色画布
                result_image = np.full((side_length, side_length, 3), background_color, dtype=np.uint8)
                
                # 计算在画布中的偏移
                offset_x = max(0, -x1)
                offset_y = max(0, -y1)
                
                # 将裁剪的图像放置到画布上
                result_image[offset_y:offset_y+crop_h, offset_x:offset_x+crop_w] = cropped_image
            else:
                result_image = cropped_image.copy()
            
            # 如果有掩膜，使用掩膜去除背景
            if remove_green_mask is not None:
                # 从掩膜中裁剪相同的区域
                if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
                    # 创建目标大小的掩膜画布（全0，表示背景）
                    cropped_mask = np.zeros((side_length, side_length), dtype=np.uint8)
                    
                    # 计算在画布中的偏移
                    offset_x = max(0, -x1)
                    offset_y = max(0, -y1)
                    
                    # 裁剪掩膜
                    mask_crop_x1 = max(0, x1)
                    mask_crop_y1 = max(0, y1)
                    mask_crop_x2 = min(img_w, x2)
                    mask_crop_y2 = min(img_h, y2)
                    mask_crop_w = mask_crop_x2 - mask_crop_x1
                    mask_crop_h = mask_crop_y2 - mask_crop_y1
                    
                    mask_cropped = remove_green_mask[mask_crop_y1:mask_crop_y2, mask_crop_x1:mask_crop_x2]
                    cropped_mask[offset_y:offset_y+mask_crop_h, offset_x:offset_x+mask_crop_w] = mask_cropped
                else:
                    cropped_mask = remove_green_mask[crop_y1:crop_y2, crop_x1:crop_x2].copy()
                
                # 确保掩膜和图像尺寸一致
                if cropped_mask.shape[:2] != result_image.shape[:2]:
                    cropped_mask = cv2.resize(cropped_mask, (result_image.shape[1], result_image.shape[0]))
                
                # 将掩膜转换为3通道
                mask_3ch = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
                
                # 使用掩膜去除背景，背景用白色填充
                # 掩膜中非零区域保留原图，零区域用白色填充
                result_image = np.where(mask_3ch > 0, result_image, background_color).astype(np.uint8)
            
            # 可视化：显示裁剪区域在原图中的位置
            # 在原图上绘制裁剪区域
            vis_image = original_image.copy()
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 255), 2)  # 黄色矩形
            cv2.circle(vis_image, (int(big_center_x), int(big_center_y)), int(big_radius), (0, 255, 0), 2)  # 绿色大圆
            cv2.circle(vis_image, (int(big_center_x), int(big_center_y)), 5, (0, 255, 0), -1)  # 绿色圆心
            
            # 添加标签
            cv2.putText(vis_image, "CROP REGION", (x1, max(y1 - 10, 20)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # 左右拼接：左边显示原图和裁剪区域标记，右边显示裁剪后的结果
            # 调整两张图像的高度使其一致
            h1, w1 = vis_image.shape[:2]
            h2, w2 = result_image.shape[:2]
            
            if h1 != h2:
                target_h = max(h1, h2)
                if h1 < target_h:
                    pad = np.full((target_h - h1, w1, 3), (0, 0, 0), dtype=np.uint8)
                    vis_image = np.vstack([vis_image, pad])
                if h2 < target_h:
                    pad = np.full((target_h - h2, w2, 3), background_color, dtype=np.uint8)
                    result_image = np.vstack([result_image, pad])
            
            # 添加标签文字
            cv2.putText(vis_image, "ORIGINAL", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(result_image, "CROPPED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            
            # 左右拼接（用于可视化）
            final_result = np.hstack([vis_image, result_image])
            
            # 单独保存工件区域图像（右侧的裁剪后图像，用于后续步骤）
            cropped_workpiece_path = debug_dir / f"step_{step_index:02d}_crop_region_workpiece.jpg"
            cv2.imwrite(str(cropped_workpiece_path), result_image)
            print(f"[INFO] 工件区域图像已保存到: {cropped_workpiece_path}")
            
            print(f"[INFO] 裁剪区域 - 中心=({big_center_x:.1f}, {big_center_y:.1f}), 边长={side_length}, 裁剪区域=({x1}, {y1}) to ({x2}, {y2})")
            
            return final_result
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_crop_region 处理异常: {error_detail}")
            raise
    
    def process_crop_and_rotate(self, original_image, big_circle_feature, valve_circle_feature, remove_green_mask, debug_dir, step_index, config_params):
        """裁剪区域并旋转至标准位置
        1. 先通过外接圆区域及工件掩膜提取工件区域像素并保存（正方形图片）
        2. 再旋转图像（记录旋转角度），使得阀体圆心在图像的左上角，并且两个圆心的连线与竖直方向呈45度
        """
        if original_image is None or original_image.size == 0:
            print(f"[ERROR] process_crop_and_rotate: original_image is None or empty")
            return None
        
        if len(original_image.shape) != 3 or original_image.shape[2] != 3:
            print(f"[ERROR] process_crop_and_rotate: original_image is not 3-channel BGR")
            return None
        
        if big_circle_feature is None or valve_circle_feature is None:
            print(f"[ERROR] process_crop_and_rotate: circle features are None")
            return None
        
        try:
            import math
            
            # 获取配置参数
            crop_params = config_params.get('preprocess', {}).get('crop_and_rotate', {})
            background_color = crop_params.get('background_color', [255, 255, 255])
            target_angle_deg = crop_params.get('target_angle', 45.0)
            valve_center_margin = crop_params.get('valve_center_margin', 50)
            target_angle_rad = math.radians(target_angle_deg)
            
            if isinstance(background_color, list):
                background_color = tuple(background_color)
            
            # ========== 步骤1: 裁剪正方形区域 ==========
            # 获取大圆信息
            big_center_x = float(big_circle_feature.get('center_x', 0))
            big_center_y = float(big_circle_feature.get('center_y', 0))
            big_radius = float(big_circle_feature.get('radius', 0))
            
            # 计算裁剪区域（以大圆圆心为中心，边长为大圆直径的正方形）
            side_length = int(big_radius * 2)  # 大圆直径
            half_side = side_length // 2
            
            # 计算裁剪区域的左上角坐标
            x1 = int(big_center_x - half_side)
            y1 = int(big_center_y - half_side)
            x2 = x1 + side_length
            y2 = y1 + side_length
            
            # 获取图像尺寸
            img_h, img_w = original_image.shape[:2]
            
            # 确保裁剪区域在图像范围内
            crop_x1 = max(0, x1)
            crop_y1 = max(0, y1)
            crop_x2 = min(img_w, x2)
            crop_y2 = min(img_h, y2)
            
            # 计算实际裁剪尺寸
            crop_w = crop_x2 - crop_x1
            crop_h = crop_y2 - crop_y1
            
            # 从原始图像中裁剪
            cropped_image = original_image[crop_y1:crop_y2, crop_x1:crop_x2].copy()
            
            # 如果裁剪区域超出图像边界，需要创建更大的画布
            if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
                # 创建目标大小的白色画布
                result_image = np.full((side_length, side_length, 3), background_color, dtype=np.uint8)
                
                # 计算在画布中的偏移
                offset_x = max(0, -x1)
                offset_y = max(0, -y1)
                
                # 将裁剪的图像放置到画布上
                result_image[offset_y:offset_y+crop_h, offset_x:offset_x+crop_w] = cropped_image
            else:
                result_image = cropped_image.copy()
            
            # 使用掩膜去除背景
            if remove_green_mask is not None:
                # 从掩膜中裁剪相同的区域
                if x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h:
                    # 创建目标大小的掩膜画布（全0，表示背景）
                    cropped_mask = np.zeros((side_length, side_length), dtype=np.uint8)
                    
                    # 计算在画布中的偏移
                    offset_x = max(0, -x1)
                    offset_y = max(0, -y1)
                    
                    # 裁剪掩膜
                    mask_crop_x1 = max(0, x1)
                    mask_crop_y1 = max(0, y1)
                    mask_crop_x2 = min(img_w, x2)
                    mask_crop_y2 = min(img_h, y2)
                    mask_crop_w = mask_crop_x2 - mask_crop_x1
                    mask_crop_h = mask_crop_y2 - mask_crop_y1
                    
                    mask_cropped = remove_green_mask[mask_crop_y1:mask_crop_y2, mask_crop_x1:mask_crop_x2]
                    cropped_mask[offset_y:offset_y+mask_crop_h, offset_x:offset_x+mask_crop_w] = mask_cropped
                else:
                    cropped_mask = remove_green_mask[crop_y1:crop_y2, crop_x1:crop_x2].copy()
                
                # 确保掩膜和图像尺寸一致
                if cropped_mask.shape[:2] != result_image.shape[:2]:
                    cropped_mask = cv2.resize(cropped_mask, (result_image.shape[1], result_image.shape[0]))
                
                # 将掩膜转换为3通道
                mask_3ch = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
                
                # 使用掩膜去除背景，背景用白色填充
                result_image = np.where(mask_3ch > 0, result_image, background_color).astype(np.uint8)
            
            # 保存裁剪后的工件区域图像（正方形）
            cropped_workpiece_path = debug_dir / f"step_{step_index:02d}_crop_and_rotate_workpiece.jpg"
            cv2.imwrite(str(cropped_workpiece_path), result_image)
            print(f"[INFO] 工件区域图像已保存到: {cropped_workpiece_path}")
            
            # ========== 步骤2: 旋转图像 ==========
            # 将圆心坐标转换为裁剪后图像的坐标系
            crop_offset_x = x1
            crop_offset_y = y1
            big_center_x_crop = big_center_x - crop_offset_x
            big_center_y_crop = big_center_y - crop_offset_y
            valve_center_x_orig = float(valve_circle_feature.get('center_x', 0))
            valve_center_y_orig = float(valve_circle_feature.get('center_y', 0))
            valve_center_x_crop = valve_center_x_orig - crop_offset_x
            valve_center_y_crop = valve_center_y_orig - crop_offset_y
            
            # 计算两个圆心之间的向量（从大圆圆心指向小圆圆心）
            dx = valve_center_x_crop - big_center_x_crop
            dy = valve_center_y_crop - big_center_y_crop
            
            # 计算当前角度（与竖直方向的夹角）
            current_angle_rad = math.atan2(dx, dy)
            
            # 计算需要旋转的角度（使得最终角度为45度）
            rotation_angle_rad = target_angle_rad - current_angle_rad
            rotation_angle_deg = math.degrees(rotation_angle_rad)
            
            # 获取图像中心作为旋转中心
            h, w = result_image.shape[:2]
            center_x = w // 2
            center_y = h // 2
            
            # 计算旋转矩阵
            rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), rotation_angle_deg, 1.0)
            
            # 计算旋转后的圆心位置
            big_center_rotated = np.dot(rotation_matrix, np.array([big_center_x_crop, big_center_y_crop, 1]))
            valve_center_rotated = np.dot(rotation_matrix, np.array([valve_center_x_crop, valve_center_y_crop, 1]))
            big_center_rotated_x = float(big_center_rotated[0])
            big_center_rotated_y = float(big_center_rotated[1])
            valve_center_rotated_x = float(valve_center_rotated[0])
            valve_center_rotated_y = float(valve_center_rotated[1])
            
            # 计算旋转后的图像尺寸（确保不裁剪）
            cos_angle = abs(rotation_matrix[0, 0])
            sin_angle = abs(rotation_matrix[0, 1])
            new_w = int((h * sin_angle) + (w * cos_angle))
            new_h = int((h * cos_angle) + (w * sin_angle))
            
            # 调整旋转矩阵的平移部分，使图像居中
            rotation_matrix[0, 2] += (new_w / 2) - center_x
            rotation_matrix[1, 2] += (new_h / 2) - center_y
            
            # 执行旋转
            rotated_image = cv2.warpAffine(
                result_image, 
                rotation_matrix, 
                (new_w, new_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=background_color
            )
            
            # 重新计算旋转后的圆心位置（考虑新的图像尺寸）
            big_center_rotated = np.dot(rotation_matrix, np.array([big_center_x_crop, big_center_y_crop, 1]))
            valve_center_rotated = np.dot(rotation_matrix, np.array([valve_center_x_crop, valve_center_y_crop, 1]))
            big_center_rotated_x = float(big_center_rotated[0])
            big_center_rotated_y = float(big_center_rotated[1])
            valve_center_rotated_x = float(valve_center_rotated[0])
            valve_center_rotated_y = float(valve_center_rotated[1])
            
            # 计算平移，使得阀体圆心在左上角（考虑边距）
            # 目标位置：阀体圆心应该在 (valve_center_margin, valve_center_margin)
            translate_x = valve_center_margin - valve_center_rotated_x
            translate_y = valve_center_margin - valve_center_rotated_y
            
            # 计算最终图像尺寸
            # 需要考虑平移后图像的范围
            # 旋转后图像的范围是 [0, new_w] x [0, new_h]
            # 平移后图像的范围是 [translate_x, translate_x + new_w] x [translate_y, translate_y + new_h]
            # 最终图像需要包含这个范围，并且阀体圆心在 (valve_center_margin, valve_center_margin)
            
            # 计算图像的最小和最大坐标
            img_min_x = min(0, translate_x)
            img_min_y = min(0, translate_y)
            img_max_x = max(new_w, new_w + translate_x)
            img_max_y = max(new_h, new_h + translate_y)
            
            # 如果平移量为负，需要扩展图像（在左侧或上方添加空间）
            # 调整平移量，使得图像从 (0, 0) 开始
            offset_x = -img_min_x if img_min_x < 0 else 0
            offset_y = -img_min_y if img_min_y < 0 else 0
            
            # 调整后的平移量
            adjusted_translate_x = translate_x + offset_x
            adjusted_translate_y = translate_y + offset_y
            
            # 计算最终图像尺寸（包含边距）
            final_w = int(img_max_x - img_min_x + valve_center_margin)
            final_h = int(img_max_y - img_min_y + valve_center_margin)
            
            # 创建平移矩阵
            translate_matrix = np.float32([[1, 0, adjusted_translate_x], [0, 1, adjusted_translate_y]])
            
            # 执行平移
            final_image = cv2.warpAffine(
                rotated_image,
                translate_matrix,
                (final_w, final_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=background_color
            )
            
            # 计算最终圆心位置（考虑调整后的平移）
            big_center_final_x = big_center_rotated_x + adjusted_translate_x
            big_center_final_y = big_center_rotated_y + adjusted_translate_y
            valve_center_final_x = valve_center_rotated_x + adjusted_translate_x
            valve_center_final_y = valve_center_rotated_y + adjusted_translate_y
            
            # ========== 可视化 ==========
            # 左边：裁剪后的工件区域图像，标记大圆、小圆和连线
            vis_before = result_image.copy()
            
            big_radius = float(big_circle_feature.get('radius', 0))
            valve_radius = float(valve_circle_feature.get('radius', 0))
            
            # 绘制大圆和圆心
            cv2.circle(vis_before, (int(big_center_x_crop), int(big_center_y_crop)), int(big_radius), (0, 255, 0), 2)
            cv2.circle(vis_before, (int(big_center_x_crop), int(big_center_y_crop)), 5, (0, 255, 0), -1)
            
            # 绘制小圆和圆心
            cv2.circle(vis_before, (int(valve_center_x_crop), int(valve_center_y_crop)), int(valve_radius), (0, 0, 255), 2)
            cv2.circle(vis_before, (int(valve_center_x_crop), int(valve_center_y_crop)), 5, (0, 0, 255), -1)
            
            # 绘制连线
            cv2.line(vis_before, 
                    (int(big_center_x_crop), int(big_center_y_crop)), 
                    (int(valve_center_x_crop), int(valve_center_y_crop)), 
                    (255, 255, 0), 2)
            
            # 右边：旋转后的图像，标记大圆、小圆和连线
            vis_after = final_image.copy()
            
            # 绘制大圆和圆心
            cv2.circle(vis_after, (int(big_center_final_x), int(big_center_final_y)), int(big_radius), (0, 255, 0), 2)
            cv2.circle(vis_after, (int(big_center_final_x), int(big_center_final_y)), 5, (0, 255, 0), -1)
            
            # 绘制小圆和圆心
            cv2.circle(vis_after, (int(valve_center_final_x), int(valve_center_final_y)), int(valve_radius), (0, 0, 255), 2)
            cv2.circle(vis_after, (int(valve_center_final_x), int(valve_center_final_y)), 5, (0, 0, 255), -1)
            
            # 绘制连线
            cv2.line(vis_after, 
                    (int(big_center_final_x), int(big_center_final_y)), 
                    (int(valve_center_final_x), int(valve_center_final_y)), 
                    (255, 255, 0), 2)
            
            # 添加标签
            cv2.putText(vis_before, "CROPPED WORKPIECE", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_after, "ROTATED & TRANSLATED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 调整两张图像的高度使其一致
            h1, w1 = vis_before.shape[:2]
            h2, w2 = vis_after.shape[:2]
            
            if h1 != h2:
                target_h = max(h1, h2)
                if h1 < target_h:
                    pad = np.full((target_h - h1, w1, 3), background_color, dtype=np.uint8)
                    vis_before = np.vstack([vis_before, pad])
                if h2 < target_h:
                    pad = np.full((target_h - h2, w2, 3), background_color, dtype=np.uint8)
                    vis_after = np.vstack([vis_after, pad])
            
            # 左右拼接
            final_result = np.hstack([vis_before, vis_after])
            
            # 输出和保存参数
            print(f"[INFO] 裁剪并旋转 - 旋转角度: {rotation_angle_deg:.2f}度, 平移: ({adjusted_translate_x:.2f}, {adjusted_translate_y:.2f})")
            print(f"[INFO] 最终参数（新图像坐标系）:")
            print(f"  大圆圆心: ({big_center_final_x:.2f}, {big_center_final_y:.2f})")
            print(f"  大圆半径: {big_radius:.2f}")
            print(f"  小圆圆心: ({valve_center_final_x:.2f}, {valve_center_final_y:.2f})")
            print(f"  小圆半径: {valve_radius:.2f}")
            print(f"  连线角度: {target_angle_deg:.2f}度（与竖直方向）")
            
            # 保存旋转后的参数到文件
            rotated_params = {
                'big_circle': {
                    'center_x': big_center_final_x,
                    'center_y': big_center_final_y,
                    'radius': big_radius
                },
                'valve_circle': {
                    'center_x': valve_center_final_x,
                    'center_y': valve_center_final_y,
                    'radius': valve_radius
                },
                'rotation_angle': rotation_angle_deg,
                'translation': {
                    'x': adjusted_translate_x,
                    'y': adjusted_translate_y
                },
                'target_angle': target_angle_deg,
                'valve_center_margin': valve_center_margin,
                'image_size': {
                    'width': final_w,
                    'height': final_h
                }
            }
            
            # 保存到JSON文件
            rotated_params_path = debug_dir / "rotated_params.json"
            with open(rotated_params_path, 'w', encoding='utf-8') as f:
                json.dump(rotated_params, f, indent=2, ensure_ascii=False)
            print(f"[INFO] 旋转后的参数已保存到: {rotated_params_path}")
            
            # 保存最终的标准位置图像
            final_workpiece_path = debug_dir / f"step_{step_index:02d}_crop_and_rotate_final.jpg"
            cv2.imwrite(str(final_workpiece_path), final_image)
            print(f"[INFO] 最终标准位置图像已保存到: {final_workpiece_path}")
            
            return final_result
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_crop_and_rotate 处理异常: {error_detail}")
            raise
    
    def process_rotate_to_45deg(self, cropped_image, big_circle_feature, valve_circle_feature, crop_offset_x, crop_offset_y, debug_dir, step_index, config_params):
        """旋转图像使阀体圆心与整体圆心连线与竖直方向呈45度
        在裁剪后的图像上进行旋转，使得模板具有通用性
        """
        if cropped_image is None or cropped_image.size == 0:
            print(f"[ERROR] process_rotate_to_45deg: cropped_image is None or empty")
            return None
        
        if len(cropped_image.shape) != 3 or cropped_image.shape[2] != 3:
            print(f"[ERROR] process_rotate_to_45deg: cropped_image is not 3-channel BGR")
            return None
        
        if big_circle_feature is None or valve_circle_feature is None:
            print(f"[ERROR] process_rotate_to_45deg: circle features are None")
            return None
        
        try:
            import math
            
            # 获取配置参数
            rotate_params = config_params.get('preprocess', {}).get('rotate_to_45deg', {})
            target_angle_deg = rotate_params.get('target_angle', 45.0)
            target_angle_rad = math.radians(target_angle_deg)
            
            # 获取大圆和小圆的圆心坐标（原始图像坐标系）
            big_center_x_orig = float(big_circle_feature.get('center_x', 0))
            big_center_y_orig = float(big_circle_feature.get('center_y', 0))
            valve_center_x_orig = float(valve_circle_feature.get('center_x', 0))
            valve_center_y_orig = float(valve_circle_feature.get('center_y', 0))
            
            # 转换为裁剪后图像的坐标系
            big_center_x = big_center_x_orig - crop_offset_x
            big_center_y = big_center_y_orig - crop_offset_y
            valve_center_x = valve_center_x_orig - crop_offset_x
            valve_center_y = valve_center_y_orig - crop_offset_y
            
            # 计算两个圆心之间的向量（从大圆圆心指向小圆圆心）
            dx = valve_center_x - big_center_x
            dy = valve_center_y - big_center_y
            
            # 计算当前角度（与竖直方向的夹角）
            # 注意：图像坐标系中，y轴向下，所以竖直方向是(0, 1)
            # 使用atan2计算角度，然后转换为与竖直方向的夹角
            current_angle_rad = math.atan2(dx, dy)  # atan2(dx, dy) 给出与y轴（竖直方向）的夹角
            
            # 计算需要旋转的角度（使得最终角度为45度）
            # 需要旋转的角度 = 目标角度 - 当前角度
            rotation_angle_rad = target_angle_rad - current_angle_rad
            rotation_angle_deg = math.degrees(rotation_angle_rad)
            
            # 获取图像中心作为旋转中心
            h, w = cropped_image.shape[:2]
            center_x = w // 2
            center_y = h // 2
            
            # 计算旋转矩阵
            rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), rotation_angle_deg, 1.0)
            
            # 计算旋转后的图像尺寸（确保不裁剪）
            cos_angle = abs(rotation_matrix[0, 0])
            sin_angle = abs(rotation_matrix[0, 1])
            new_w = int((h * sin_angle) + (w * cos_angle))
            new_h = int((h * cos_angle) + (w * sin_angle))
            
            # 调整旋转矩阵的平移部分，使图像居中
            rotation_matrix[0, 2] += (new_w / 2) - center_x
            rotation_matrix[1, 2] += (new_h / 2) - center_y
            
            # 执行旋转
            rotated_image = cv2.warpAffine(
                cropped_image, 
                rotation_matrix, 
                (new_w, new_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(255, 255, 255)  # 白色背景
            )
            
            # 可视化：左边显示裁剪后的原图，右边显示旋转后的图像
            # 左边：在裁剪后的原图上绘制大圆圆心、小圆圆心和连线
            vis_before = cropped_image.copy()
            
            # 获取大圆和小圆的半径（用于绘制圆）
            big_radius = float(big_circle_feature.get('radius', 0))
            valve_radius = float(valve_circle_feature.get('radius', 0))
            
            # 绘制大圆和圆心
            cv2.circle(vis_before, (int(big_center_x), int(big_center_y)), int(big_radius), (0, 255, 0), 2)  # 绿色大圆
            cv2.circle(vis_before, (int(big_center_x), int(big_center_y)), 5, (0, 255, 0), -1)  # 绿色大圆圆心
            
            # 绘制小圆和圆心
            cv2.circle(vis_before, (int(valve_center_x), int(valve_center_y)), int(valve_radius), (0, 0, 255), 2)  # 红色小圆
            cv2.circle(vis_before, (int(valve_center_x), int(valve_center_y)), 5, (0, 0, 255), -1)  # 红色小圆圆心
            
            # 绘制连线
            cv2.line(vis_before, 
                    (int(big_center_x), int(big_center_y)), 
                    (int(valve_center_x), int(valve_center_y)), 
                    (255, 255, 0), 2)  # 青色连线
            
            # 计算旋转后的圆心位置（在新图像坐标系下）
            big_center_rotated = np.dot(rotation_matrix, np.array([big_center_x, big_center_y, 1]))
            valve_center_rotated = np.dot(rotation_matrix, np.array([valve_center_x, valve_center_y, 1]))
            big_center_rotated_x = float(big_center_rotated[0])
            big_center_rotated_y = float(big_center_rotated[1])
            valve_center_rotated_x = float(valve_center_rotated[0])
            valve_center_rotated_y = float(valve_center_rotated[1])
            
            # 右边：在旋转后的图像上绘制大圆圆心、小圆圆心和连线
            vis_after = rotated_image.copy()
            
            # 绘制大圆和圆心
            cv2.circle(vis_after, (int(big_center_rotated_x), int(big_center_rotated_y)), int(big_radius), (0, 255, 0), 2)  # 绿色大圆
            cv2.circle(vis_after, (int(big_center_rotated_x), int(big_center_rotated_y)), 5, (0, 255, 0), -1)  # 绿色大圆圆心
            
            # 绘制小圆和圆心
            cv2.circle(vis_after, (int(valve_center_rotated_x), int(valve_center_rotated_y)), int(valve_radius), (0, 0, 255), 2)  # 红色小圆
            cv2.circle(vis_after, (int(valve_center_rotated_x), int(valve_center_rotated_y)), 5, (0, 0, 255), -1)  # 红色小圆圆心
            
            # 绘制连线
            cv2.line(vis_after, 
                    (int(big_center_rotated_x), int(big_center_rotated_y)), 
                    (int(valve_center_rotated_x), int(valve_center_rotated_y)), 
                    (255, 255, 0), 2)  # 青色连线
            
            # 添加标签
            cv2.putText(vis_before, "CROPPED ORIGINAL", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_after, "ROTATED (45deg)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 输出新图像坐标系下的参数
            print(f"[INFO] 旋转后的参数（新图像坐标系）:")
            print(f"  大圆圆心: ({big_center_rotated_x:.2f}, {big_center_rotated_y:.2f})")
            print(f"  大圆半径: {big_radius:.2f}")
            print(f"  小圆圆心: ({valve_center_rotated_x:.2f}, {valve_center_rotated_y:.2f})")
            print(f"  小圆半径: {valve_radius:.2f}")
            print(f"  连线角度: {target_angle_deg:.2f}度（与竖直方向）")
            
            # 保存旋转后的参数到文件
            rotated_params = {
                'big_circle': {
                    'center_x': big_center_rotated_x,
                    'center_y': big_center_rotated_y,
                    'radius': big_radius
                },
                'valve_circle': {
                    'center_x': valve_center_rotated_x,
                    'center_y': valve_center_rotated_y,
                    'radius': valve_radius
                },
                'rotation_angle': rotation_angle_deg,
                'target_angle': target_angle_deg,
                'image_size': {
                    'width': new_w,
                    'height': new_h
                }
            }
            
            # 保存到JSON文件
            rotated_params_path = debug_dir / "rotated_params.json"
            with open(rotated_params_path, 'w', encoding='utf-8') as f:
                json.dump(rotated_params, f, indent=2, ensure_ascii=False)
            print(f"[INFO] 旋转后的参数已保存到: {rotated_params_path}")
            
            # 调整两张图像的高度使其一致
            h1, w1 = vis_before.shape[:2]
            h2, w2 = vis_after.shape[:2]
            
            if h1 != h2:
                target_h = max(h1, h2)
                if h1 < target_h:
                    pad = np.full((target_h - h1, w1, 3), (255, 255, 255), dtype=np.uint8)
                    vis_before = np.vstack([vis_before, pad])
                if h2 < target_h:
                    pad = np.full((target_h - h2, w2, 3), (255, 255, 255), dtype=np.uint8)
                    vis_after = np.vstack([vis_after, pad])
            
            # 左右拼接
            final_result = np.hstack([vis_before, vis_after])
            
            print(f"[INFO] 旋转至45度 - 当前角度: {math.degrees(current_angle_rad):.2f}度, 旋转角度: {rotation_angle_deg:.2f}度, 目标角度: {target_angle_deg:.2f}度")
            
            return final_result
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_rotate_to_45deg 处理异常: {error_detail}")
            raise
    
    def process_binarize(self, original_image, non_green_mask, debug_dir, step_index):
        """二值化处理"""
        if original_image is None or original_image.size == 0:
            print(f"[ERROR] process_binarize: original_image is None or empty")
            return None
        
        try:
            if non_green_mask is None:
                # 如果前一步结果不存在，先去除绿色背景
                if len(original_image.shape) != 3 or original_image.shape[2] != 3:
                    print(f"[ERROR] process_binarize: original_image is not 3-channel BGR")
                    return None
                hsv = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
                lower_green = np.array([35, 40, 40])
                upper_green = np.array([85, 255, 255])
                green_mask = cv2.inRange(hsv, lower_green, upper_green)
                non_green_mask = cv2.bitwise_not(green_mask)
            else:
                # 确保是灰度图
                if len(non_green_mask.shape) == 3:
                    non_green_mask = cv2.cvtColor(non_green_mask, cv2.COLOR_BGR2GRAY)
                elif len(non_green_mask.shape) != 2:
                    print(f"[ERROR] process_binarize: non_green_mask has invalid shape: {non_green_mask.shape}")
                    return None
        
            # 确保 original_image 是3通道BGR图像
            if len(original_image.shape) == 3 and original_image.shape[2] == 3:
                gray = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
            elif len(original_image.shape) == 2:
                gray = original_image.copy()
            else:
                print(f"[ERROR] process_binarize: original_image has invalid shape: {original_image.shape}")
                return None
            
            # 确保数据类型为 uint8
            if gray.dtype != np.uint8:
                gray = gray.astype(np.uint8)
            if non_green_mask.dtype != np.uint8:
                non_green_mask = non_green_mask.astype(np.uint8)
            
            # 验证图像尺寸
            if gray.shape != non_green_mask.shape:
                print(f"[ERROR] process_binarize: Shape mismatch: gray={gray.shape}, mask={non_green_mask.shape}")
                return None
            
            # 验证图像尺寸是否足够大（adaptiveThreshold 需要块大小至少为3）
            h, w = gray.shape[:2]
            if h < 3 or w < 3:
                print(f"[ERROR] process_binarize: Image too small: {gray.shape}")
                return None
            
            # adaptiveThreshold 的块大小必须是奇数且大于1
            block_size = 31
            if min(h, w) < block_size:
                block_size = min(h, w) - 1 if min(h, w) % 2 == 0 else min(h, w)
                block_size = max(3, block_size)
                print(f"[WARN] process_binarize: block_size adjusted to {block_size}")
            
            adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                                        cv2.THRESH_BINARY, block_size, 5)
            binary = cv2.bitwise_and(adaptive_thresh, adaptive_thresh, mask=non_green_mask)
            # 转换为3通道用于显示
            result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            return result
            
        except Exception as e:
            import traceback
            error_detail = traceback.format_exc()
            print(f"[ERROR] process_binarize 处理异常: {error_detail}")
            raise
    
    def process_morphology(self, binary_image, debug_dir, step_index, config_params):
        """形态学处理"""
        if binary_image is None:
            return None
        
        # 转换为灰度图
        if len(binary_image.shape) == 3:
            binary = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
        else:
            binary = binary_image
        
        # 开运算
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open, iterations=1)
        
        # 闭运算
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        
        # 转换为3通道用于显示
        result = cv2.cvtColor(closed, cv2.COLOR_GRAY2BGR)
        return result
    
    def process_components(self, morph_image, debug_dir, step_index, config_params):
        """提取连通域"""
        if morph_image is None:
            return None
        
        # 转换为灰度图
        if len(morph_image.shape) == 3:
            binary = cv2.cvtColor(morph_image, cv2.COLOR_BGR2GRAY)
        else:
            binary = morph_image
        
        min_area = config_params.get('preprocess', {}).get('min_area', 2000)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 创建结果图像
        result = np.zeros_like(binary)
        
        # 过滤并绘制轮廓
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                cv2.drawContours(result, [contour], -1, 255, -1)
        
        # 转换为3通道用于显示
        result_bgr = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
        return result_bgr
    
    def process_features(self, original_image, components_image, debug_dir, step_index, config_params):
        """特征提取"""
        if components_image is None:
            return original_image, []
        
        # 转换为灰度图
        if len(components_image.shape) == 3:
            mask = cv2.cvtColor(components_image, cv2.COLOR_BGR2GRAY)
        else:
            mask = components_image
        
        features = []
        result_image = original_image.copy()
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = config_params.get('preprocess', {}).get('min_area', 2000)
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            # 创建单个区域的掩码
            region_mask = np.zeros_like(mask)
            cv2.drawContours(region_mask, [contour], -1, 255, -1)
            
            # 提取大圆
            big_center, big_radius = self.extract_big_circle(region_mask, config_params)
            
            # 提取小圆
            small_center, small_radius = self.extract_small_circle(region_mask, config_params)
            
            if big_center is not None and small_center is not None:
                # 计算角度
                theta = np.arctan2(small_center[1] - big_center[1], 
                                  small_center[0] - big_center[0])
                
                # 保存特征
                feature = {
                    'ub': float(big_center[0]),
                    'vb': float(big_center[1]),
                    'rb': float(big_radius),
                    'us': float(small_center[0]),
                    'vs': float(small_center[1]),
                    'rs': float(small_radius),
                    'theta': float(theta),
                    'theta_deg': float(np.degrees(theta)),
                    'area': float(area)
                }
                features.append(feature)
                
                # 可视化
                cv2.circle(result_image, (int(big_center[0]), int(big_center[1])), 
                          int(big_radius), (0, 255, 0), 2)
                cv2.circle(result_image, (int(small_center[0]), int(small_center[1])), 
                          int(small_radius), (0, 0, 255), 2)
                cv2.circle(result_image, (int(big_center[0]), int(big_center[1])), 
                          3, (0, 255, 0), -1)
                cv2.circle(result_image, (int(small_center[0]), int(small_center[1])), 
                          3, (0, 0, 255), -1)
        
        return result_image, features
    
    def extract_big_circle(self, mask, config_params):
        """提取大圆"""
        big_circle_params = config_params.get('preprocess', {}).get('big_circle', {})
        min_area = big_circle_params.get('min_area', 100)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if big_circle_params.get('combine_contours', True) and len(contours) > 1:
            # 合并所有轮廓
            combined = np.zeros_like(mask)
            for contour in contours:
                if cv2.contourArea(contour) >= min_area:
                    cv2.drawContours(combined, [contour], -1, 255, -1)
            contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # 找到最大的轮廓
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None, 0
        
        # 拟合最小外接圆
        (x, y), radius = cv2.minEnclosingCircle(largest)
        return (x, y), radius
    
    def extract_small_circle(self, mask, config_params):
        """提取小圆"""
        small_circle_params = config_params.get('preprocess', {}).get('small_circle', {})
        erode_kernel = small_circle_params.get('erode_kernel', 11)
        erode_iterations = small_circle_params.get('erode_iterations', 5)
        dilate_kernel = small_circle_params.get('dilate_kernel', 9)
        dilate_iterations = small_circle_params.get('dilate_iterations', 1)
        keep_largest = small_circle_params.get('largest_cc', True)
        
        # 腐蚀
        kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_kernel, erode_kernel))
        eroded = cv2.erode(mask, kernel_erode, iterations=erode_iterations)
        
        # 保留最大连通域
        if keep_largest:
            contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                largest_mask = np.zeros_like(eroded)
                cv2.drawContours(largest_mask, [largest], -1, 255, -1)
                eroded = largest_mask
        
        # 膨胀
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_kernel, dilate_kernel))
        dilated = cv2.dilate(eroded, kernel_dilate, iterations=dilate_iterations)
        
        # 提取圆
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0
        
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)
        return (x, y), radius

def start_server():
    global ros2_node
    
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
    with socketserver.TCPServer(("", PORT), AlgorithmHandler) as httpd:
        print(f"🎯 视觉姿态估计算法Web UI服务器启动成功!")
        print(f"🌐 服务地址: http://localhost:{PORT}/")
        print(f"📱 Web界面: http://localhost:{PORT}/index.html")
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
