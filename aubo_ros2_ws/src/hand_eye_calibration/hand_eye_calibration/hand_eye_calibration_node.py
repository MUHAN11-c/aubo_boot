#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
单目相机手眼标定工具 - 主节点
提供基于Web UI的交互式标定界面
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from flask import Flask, render_template, jsonify, request, Response
from flask_cors import CORS
import threading
import os
import json
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time
from demo_interface.msg import RobotStatus
from percipio_camera_interface.msg import CameraStatus, ImageData
from percipio_camera_interface.srv import SoftwareTrigger
import base64
from ament_index_python.packages import get_package_share_directory
from .camera_calibration_utils import CameraCalibrationUtils
from .custom_hand_eye_calibration import CustomHandEyeCalibration
from .opencv_hand_eye_calibration import OpenCVHandEyeCalibration, _convert_to_json_serializable
import logging
from typing import Dict


class HandEyeCalibrationNode(Node):
    """手眼标定节点"""
    
    def __init__(self):
        super().__init__('hand_eye_calibration_node')
        
        # 初始化参数
        self.declare_parameter('web_host', 'localhost')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('robot_status_topic', '/robot_status')
        
        self.web_host = self.get_parameter('web_host').value
        self.web_port = self.get_parameter('web_port').value
        
        # 数据存储
        self.current_image = None
        self.current_image_raw = None  # 保存原始BGR图像用于处理
        self._image_is_undistorted = False  # 标记当前图像是否已去畸变
        self.current_robot_pose = None
        self.current_robot_status = None  # 完整的机器人状态（包括is_online等）
        self.current_robot_status_timestamp = None  # 机器人状态时间戳（原始类型）
        self.current_camera_status = None  # 相机状态
        self.current_point_cloud = None  # 当前点云数据（保留兼容性）
        self.current_depth_image = None  # 当前深度图（对齐到彩色图）
        self.current_camera_info = None  # 当前相机内参（从 CameraInfo 话题获取）
        self.bridge = CvBridge()
        
        # 线程锁保护共享数据
        import threading
        self.robot_status_lock = threading.Lock()
        
        # 相机标定工具
        self.calib_utils = CameraCalibrationUtils()
        
        # 角点检测结果
        self.detected_corners = None
        self.corners_3d = None
        self.pattern_size = (11, 8)  # 默认棋盘格大小（12x9格子 = 11x8个内角点 = 88个角点）
        
        # 标定数据
        self.camera_calibration_data = {
            'camera_matrix': None,
            'dist_coeffs': None,
            'reprojection_error': None
        }
        
        self.hand_eye_calibration_data = {
            'transformation_matrix': None,
            'captured_poses': [],
            'calibration_error': None
        }
        
        # ROS2订阅 - 订阅图像数据
        # 使用与发布者匹配的QoS设置
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_subscription = self.create_subscription(
            ImageData,
            '/image_data',
            self.image_callback,
            image_qos
        )
        
        self.get_logger().info('📡 已订阅 /image_data 话题，等待图像数据...')
        
        # 订阅点云数据（用于深度误差检查）
        point_cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # 深度图话题名称（可通过参数配置）
        # 默认使用 depth/image_raw（对齐到彩色图的深度图）
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        depth_image_topic = self.get_parameter('depth_image_topic').value
        
        # 深度缩放因子已写死为 0.00025，与 depth_z_reader 完全一致
        # 参考 depth_z_reader 的实现方式：depth_raw * 0.00025 = 深度（米）
        # 适用于 scale_unit=0.25 的相机（如 PS800-E1）
        self.get_logger().info(f'📏 深度缩放因子: {self.calib_utils.DEPTH_SCALE} (与 depth_z_reader 一致)')
        self.get_logger().info('   深度处理: depth_raw * 0.00025 * 1000 = 深度（毫米）')
        
        self.depth_image_subscription = self.create_subscription(
            Image,
            depth_image_topic,  # percipio相机发布的深度图话题（已对齐到彩色图）
            self.depth_image_callback,
            image_qos
        )
        self.get_logger().info(f'📡 已订阅 {depth_image_topic} 话题，等待深度图数据...')
        
        # 软触发客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/software_trigger')
        
        # 机器人运动控制服务客户端
        from demo_interface.srv import SetRobotPose
        self.set_robot_pose_client = self.create_client(SetRobotPose, '/set_robot_pose')
        
        # 移动到目标位姿服务客户端（用于自动标定）
        from demo_interface.srv import MoveToPose
        self.move_to_pose_client = self.create_client(MoveToPose, '/move_to_pose')
        
        # 订阅机器人状态话题（从 /demo_robot_status 获取机械臂当前位姿）
        self.robot_status_subscription = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        self.get_logger().info('✅ 订阅机器人状态话题: /demo_robot_status')
        
        # 订阅相机状态
        self.camera_status_subscription = self.create_subscription(
            CameraStatus,
            '/camera_status',
            self.camera_status_callback,
            10
        )
        
        # 完全从文件加载相机内参（不使用 CameraInfo 话题）
        import yaml
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('hand_eye_calibration')
            ost_path = os.path.join(package_share, 'config', 'calibrationdata', 'ost.yaml')
            
            if os.path.exists(ost_path):
                with open(ost_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                
                # 提取相机内参矩阵
                if 'camera_matrix' in calib_data:
                    camera_matrix_data = calib_data['camera_matrix']['data']
                    self.calib_utils.camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
                
                # 提取畸变系数
                if 'distortion_coefficients' in calib_data:
                    dist_data = calib_data['distortion_coefficients']['data']
                    self.calib_utils.dist_coeffs = np.array(dist_data, dtype=np.float64)
                    self.get_logger().info(f'✅ 从文件加载畸变系数: {self.calib_utils.dist_coeffs[:5]}')
                
                # 提取图像尺寸
                if 'image_width' in calib_data and 'image_height' in calib_data:
                    self.calib_utils.image_size = (calib_data['image_width'], calib_data['image_height'])
                
                # 同时更新到 camera_calibration_data
                self.camera_calibration_data['camera_matrix'] = self.calib_utils.camera_matrix
                self.camera_calibration_data['dist_coeffs'] = self.calib_utils.dist_coeffs
                self.camera_calibration_data['image_size'] = self.calib_utils.image_size
                
                self.get_logger().info(f'✅ 从文件加载相机内参: fx={self.calib_utils.camera_matrix[0,0]:.2f}, fy={self.calib_utils.camera_matrix[1,1]:.2f}, cx={self.calib_utils.camera_matrix[0,2]:.2f}, cy={self.calib_utils.camera_matrix[1,2]:.2f}')
                self.get_logger().info(f'   图像尺寸: {self.calib_utils.image_size}')
            else:
                self.get_logger().error(f'❌ 相机标定文件不存在: {ost_path}')
                self.get_logger().error('   请先进行相机内参标定或提供 ost.yaml 文件')
        except Exception as e:
            self.get_logger().error(f'❌ 加载相机内参失败: {str(e)}')
        
        # Flask应用
        self.app = Flask(__name__,
                        template_folder=self._get_template_folder(),
                        static_folder=self._get_static_folder())
        CORS(self.app)
        
        # 禁用Flask的HTTP请求日志
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)  # 只显示错误，不显示HTTP请求
        
        # 注册路由
        self._register_routes()
        
        # 启动Web服务器
        self.web_thread = threading.Thread(
            target=self._run_web_server,
            daemon=True
        )
        self.web_thread.start()
        
        self.get_logger().info(f'手眼标定Web界面已启动: http://{self.web_host}:{self.web_port}')
    
    def _get_template_folder(self):
        """获取模板文件夹路径"""
        try:
            package_share = get_package_share_directory('hand_eye_calibration')
            return os.path.join(package_share, 'web', 'templates')
        except:
            return os.path.join(os.path.dirname(__file__), '..', 'web', 'templates')
    
    def _get_static_folder(self):
        """获取静态文件夹路径"""
        try:
            package_share = get_package_share_directory('hand_eye_calibration')
            return os.path.join(package_share, 'web', 'static')
        except:
            return os.path.join(os.path.dirname(__file__), '..', 'web', 'static')
    
    def _get_config_dir(self):
        """获取config目录路径（优先使用源码目录）"""
        # 从当前文件路径解析出workspace路径，然后指向源码目录
        current_file = os.path.abspath(__file__)
        file_path_parts = current_file.split(os.sep)
        
        # 如果路径中包含 'install'，说明是从install目录运行的
        if 'install' in file_path_parts:
            workspace_idx = file_path_parts.index('install')
            workspace_path = os.sep.join(file_path_parts[:workspace_idx])
            config_dir = os.path.join(workspace_path, 'src', 'hand_eye_calibration', 'config')
        # 如果路径中包含 'src'，直接从源码目录计算
        elif 'src' in file_path_parts:
            workspace_idx = file_path_parts.index('src')
            workspace_path = os.sep.join(file_path_parts[:workspace_idx])
            config_dir = os.path.join(workspace_path, 'src', 'hand_eye_calibration', 'config')
        else:
            # 回退方案：使用相对路径（假设在源码目录运行）
            config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
            config_dir = os.path.abspath(config_dir)
        
        return os.path.abspath(config_dir)
    
    def _register_routes(self):
        """注册Flask路由"""
        
        @self.app.route('/')
        def index():
            """主页"""
            return render_template('index.html')
        
        @self.app.route('/test')
        def test():
            """测试诊断页面"""
            return render_template('test.html')
        
        @self.app.route('/test2')
        def test2():
            """简单测试页面"""
            return render_template('test2.html')
        
        @self.app.route('/debug')
        def debug():
            """调试页面"""
            return render_template('debug.html')
        
        @self.app.route('/api/current_image')
        def get_current_image():
            """获取当前相机图像"""
            if self.current_image is not None:
                try:
                    # 使用较高质量的JPEG编码
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                    _, buffer = cv2.imencode('.jpg', self.current_image, encode_param)
                    img_base64 = base64.b64encode(buffer).decode('utf-8')
                    
                    # 记录图像信息（节流输出）
                    img_size = self.current_image.shape
                    self.get_logger().info(f'🖼️  发送图像到Web前端 - 尺寸: {img_size[1]}x{img_size[0]}, JPEG大小: {len(buffer)/1024:.1f}KB', 
                                          throttle_duration_sec=5.0)
                    
                    return jsonify({
                        'success': True,
                        'image': f'data:image/jpeg;base64,{img_base64}',
                        'width': img_size[1],
                        'height': img_size[0]
                    })
                except Exception as e:
                    self.get_logger().error(f'编码图像失败: {str(e)}')
                    return jsonify({'success': False, 'message': f'编码失败: {str(e)}'})
            
            # 不要频繁打印"无图像数据"
            return jsonify({'success': False, 'message': '无图像数据'})
        
        @self.app.route('/api/robot_pose')
        def get_robot_pose():
            """获取当前机器人位姿"""
            if self.current_robot_pose is not None:
                return jsonify({
                    'success': True,
                    'pose': {
                        'position': {
                            'x': self.current_robot_pose.position.x,
                            'y': self.current_robot_pose.position.y,
                            'z': self.current_robot_pose.position.z
                        },
                        'orientation': {
                            'x': self.current_robot_pose.orientation.x,
                            'y': self.current_robot_pose.orientation.y,
                            'z': self.current_robot_pose.orientation.z,
                            'w': self.current_robot_pose.orientation.w
                        }
                    }
                })
            return jsonify({'success': False, 'message': '无机器人位姿数据'})
        
        @self.app.route('/api/robot_status')
        def get_robot_status():
            """获取完整的机器人状态（包括在线状态、运动状态、位姿等）"""
            try:
                # 主动处理ROS2消息队列，确保获取最新数据
                import rclpy
                # 执行多次spin_once，超时时间设置得很短以避免阻塞API响应
                for i in range(10):
                    rclpy.spin_once(self, timeout_sec=0.005)
                
                if self.current_robot_status is not None:
                    try:
                        return jsonify({
                            'success': True,
                            'is_online': self.current_robot_status.is_online,
                            'enable': self.current_robot_status.enable,
                            'in_motion': self.current_robot_status.in_motion,
                            'joint_position_rad': list(self.current_robot_status.joint_position_rad),
                            'joint_position_deg': list(self.current_robot_status.joint_position_deg),
                            'cartesian_position': {
                                'position': {
                                    'x': self.current_robot_status.cartesian_position.position.x,
                                    'y': self.current_robot_status.cartesian_position.position.y,
                                    'z': self.current_robot_status.cartesian_position.position.z
                                },
                                'orientation': {
                                    'x': self.current_robot_status.cartesian_position.orientation.x,
                                    'y': self.current_robot_status.cartesian_position.orientation.y,
                                    'z': self.current_robot_status.cartesian_position.orientation.z,
                                    'w': self.current_robot_status.cartesian_position.orientation.w
                                }
                            }
                        })
                    except Exception as e:
                        self.get_logger().error(f'创建API响应时出错: {str(e)}')
                        import traceback
                        self.get_logger().error(traceback.format_exc())
                        return jsonify({'success': False, 'message': f'处理机器人状态数据时出错: {str(e)}'}), 500
                
                return jsonify({'success': False, 'message': '无机器人状态数据'})
            except Exception as e:
                self.get_logger().error(f'/api/robot_status 未处理异常: {str(e)}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                return jsonify({'success': False, 'message': f'服务器错误: {str(e)}'}), 500
        
        @self.app.route('/api/camera_status')
        def get_camera_status():
            """获取相机状态"""
            if self.current_camera_status is not None:
                return jsonify({
                    'success': True,
                    'camera_id': self.current_camera_status.camera_id,
                    'is_connected': self.current_camera_status.is_connected,
                    'trigger_mode': self.current_camera_status.trigger_mode,
                    'exposure_time': self.current_camera_status.exposure_time,
                    'gain': self.current_camera_status.gain
                })
            return jsonify({'success': False, 'message': '无相机状态数据'})
        
        @self.app.route('/api/camera_info')
        def get_camera_info():
            """获取相机内参（从 ROS2 CameraInfo 话题）"""
            try:
                # 优先使用从 ROS2 话题获取的内参
                if self.current_camera_info is not None:
                    msg = self.current_camera_info
                    camera_matrix = np.array([
                        [msg.k[0], msg.k[1], msg.k[2]],
                        [msg.k[3], msg.k[4], msg.k[5]],
                        [msg.k[6], msg.k[7], msg.k[8]]
                    ], dtype=np.float64)
                    
                    return jsonify({
                        'success': True,
                        'source': 'ros2_topic',
                        'camera_intrinsic': {
                            'fx': float(camera_matrix[0, 0]),
                            'fy': float(camera_matrix[1, 1]),
                            'cx': float(camera_matrix[0, 2]),
                            'cy': float(camera_matrix[1, 2]),
                            'camera_matrix': camera_matrix.tolist(),
                            'dist_coeffs': list(msg.d),
                            'image_size': [msg.width, msg.height]
                        }
                    })
                
                # 如果没有从话题获取，尝试使用已加载的参数
                if self.calib_utils.camera_matrix is not None:
                    return jsonify({
                        'success': True,
                        'source': 'loaded_file',
                        'camera_intrinsic': {
                            'fx': float(self.calib_utils.camera_matrix[0, 0]),
                            'fy': float(self.calib_utils.camera_matrix[1, 1]),
                            'cx': float(self.calib_utils.camera_matrix[0, 2]),
                            'cy': float(self.calib_utils.camera_matrix[1, 2]),
                            'camera_matrix': self.calib_utils.camera_matrix.tolist(),
                            'dist_coeffs': self.calib_utils.dist_coeffs.tolist() if self.calib_utils.dist_coeffs is not None else [],
                            'image_size': []
                        }
                    })
                
                return jsonify({
                    'success': False,
                    'message': '相机内参不可用，请加载标定文件或等待 CameraInfo 话题数据'
                })
            except Exception as e:
                self.get_logger().error(f'获取相机内参失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/load_params', methods=['POST'])
        def load_camera_params():
            """加载相机参数 - 从上传的文件或默认路径"""
            try:
                # 检查是否有上传的文件
                if 'file' in request.files:
                    file = request.files['file']
                    if file.filename == '':
                        return jsonify({'success': False, 'error': '未选择文件'})
                    
                    # 保存临时文件
                    import tempfile
                    temp_dir = tempfile.gettempdir()
                    temp_path = os.path.join(temp_dir, 'camera_params_uploaded.xml')
                    file.save(temp_path)
                    config_path = temp_path
                    self.get_logger().info(f'📁 加载上传的相机参数文件: {file.filename}')
                else:
                    # 使用默认路径
                    config_path = os.path.join(
                        get_package_share_directory('hand_eye_calibration'),
                        'config', 'cameraParams.xml'
                    )
                    
                    # 如果找不到，尝试源码路径
                    if not os.path.exists(config_path):
                        config_path = os.path.join(
                            os.path.dirname(__file__), '..', 'config', 'cameraParams.xml'
                        )
                    
                    if os.path.exists(config_path):
                        self.get_logger().info(f'📁 加载默认相机参数文件: {config_path}')
                    else:
                        return jsonify({'success': False, 'error': '未找到默认标定文件，请上传标定文件'})
                
                result = self.calib_utils.load_camera_params(config_path)
                
                # 如果加载成功，保存numpy格式的原始数据到字典
                if result.get('success'):
                    # 从calib_utils获取numpy数组（不是result中的字典）
                    self.camera_calibration_data['camera_matrix'] = self.calib_utils.camera_matrix
                    self.camera_calibration_data['dist_coeffs'] = self.calib_utils.dist_coeffs
                    self.get_logger().info('✅ 相机参数已保存到内存，可以使用手动选点功能')
                    self.get_logger().info(f'   相机矩阵类型: {type(self.calib_utils.camera_matrix)}')
                    self.get_logger().info(f'   畸变系数类型: {type(self.calib_utils.dist_coeffs)}')
                
                return jsonify(result)
            except Exception as e:
                self.get_logger().error(f'❌ 加载相机参数失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/capture', methods=['POST'])
        def capture_image():
            """触发相机拍照"""
            try:
                if not self.trigger_client.wait_for_service(timeout_sec=1.0):
                    return jsonify({'success': False, 'error': '相机服务未运行，请先启动vision_acquisition节点'})
                
                # 获取相机ID
                camera_id = '207000152740'  # 默认相机ID
                if self.current_camera_status is not None:
                    camera_id = self.current_camera_status.camera_id
                
                # 调用软触发服务，传递camera_id
                request_msg = SoftwareTrigger.Request()
                request_msg.camera_id = camera_id
                
                # 同步调用，等待响应
                future = self.trigger_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info('='*60)
                        self.get_logger().info(f'📷 软触发成功')
                        self.get_logger().info(f'   相机ID: {camera_id}')
                        self.get_logger().info(f'   响应消息: {response.message}')
                        self.get_logger().info(f'   等待/image_data话题发布...')
                        self.get_logger().info('='*60)
                        return jsonify({
                            'success': True, 
                            'message': f'拍照成功，相机ID: {camera_id}'
                        })
                    else:
                        self.get_logger().error(f'❌ 软触发失败: {response.message}')
                        return jsonify({
                            'success': False, 
                            'error': f'拍照失败: {response.message}'
                        })
                else:
                    self.get_logger().error('❌ 软触发服务调用超时（5秒）')
                    return jsonify({'success': False, 'error': '服务调用超时'})
                    
            except Exception as e:
                self.get_logger().error(f'触发拍照失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/upload_image', methods=['POST'])
        def upload_image():
            """上传本地图像"""
            try:
                self.get_logger().info('='*60)
                self.get_logger().info('📁 接收到本地图像上传请求')
                
                if 'image' not in request.files:
                    self.get_logger().error('❌ 请求中未找到图像文件')
                    return jsonify({'success': False, 'error': '未找到图像文件'})
                
                file = request.files['image']
                if file.filename == '':
                    self.get_logger().error('❌ 未选择文件')
                    return jsonify({'success': False, 'error': '未选择文件'})
                
                self.get_logger().info(f'   文件名: {file.filename}')
                
                # 读取图像
                file_bytes = file.read()
                self.get_logger().info(f'   文件大小: {len(file_bytes)} bytes ({len(file_bytes)/1024:.1f} KB)')
                
                file_bytes_np = np.frombuffer(file_bytes, np.uint8)
                image = cv2.imdecode(file_bytes_np, cv2.IMREAD_COLOR)
                
                if image is None:
                    self.get_logger().error('❌ 无法解析图像，可能格式不支持')
                    return jsonify({'success': False, 'error': '无法解析图像，请使用JPG/PNG/BMP格式'})
                
                self.get_logger().info(f'✅ 图像解码成功')
                self.get_logger().info(f'   图像尺寸: {image.shape[1]} x {image.shape[0]}')
                self.get_logger().info(f'   通道数: {image.shape[2] if len(image.shape) > 2 else 1}')
                
                # 保存当前图像
                self.current_image = image
                self.current_image_raw = image.copy()
                self._image_is_undistorted = False  # 新图像未去畸变
                
                # 转换为base64返回
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, buffer = cv2.imencode('.jpg', image, encode_param)
                img_base64 = base64.b64encode(buffer).decode('utf-8')
                
                self.get_logger().info(f'   Base64编码大小: {len(img_base64)} chars ({len(buffer)/1024:.1f} KB)')
                self.get_logger().info('='*60)
                
                return jsonify({
                    'success': True,
                    'image': f'data:image/jpeg;base64,{img_base64}',
                    'width': image.shape[1],
                    'height': image.shape[0]
                })
            except Exception as e:
                self.get_logger().error(f'❌ 上传图像失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/extract_corners', methods=['POST'])
        def extract_corners():
            """提取棋盘格角点"""
            try:
                data = request.get_json()
                square_size = data.get('square_size', 20.0)  # mm（12x9格子，每格20mm）
                
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '请先加载图像'})
                
                if self.calib_utils.camera_matrix is None:
                    return jsonify({'success': False, 'error': '请先加载相机参数'})
                
                # 转换为灰度图进行角点检测（提高检测精度和速度）
                if len(self.current_image_raw.shape) == 3:
                    gray_image = cv2.cvtColor(self.current_image_raw, cv2.COLOR_BGR2GRAY)
                else:
                    gray_image = self.current_image_raw
                
                # 检测角点
                corners = self.calib_utils.detect_checkerboard_corners(
                    gray_image, self.pattern_size
                )
                
                if corners is None:
                    return jsonify({'success': False, 'error': '未检测到棋盘格角点，请确保：1.图像清晰 2.棋盘格完整可见 3.尝试调整光照'})
                
                self.detected_corners = corners
                
                # 如果检测到不同尺寸，更新pattern_size
                if self.calib_utils.detected_pattern_size is not None:
                    self.pattern_size = self.calib_utils.detected_pattern_size
                    self.get_logger().info(f'检测到棋盘格尺寸: {self.pattern_size}')
                
                # 估计Z坐标
                z_camera = self.calib_utils.estimate_checkerboard_z(
                    corners, square_size, self.pattern_size
                )
                
                # 转换到相机坐标系
                corners_2d = corners.reshape(-1, 2)
                points_3d = self.calib_utils.pixel_to_camera_coords(corners_2d, z_camera)
                self.corners_3d = points_3d
                
                # 计算相邻距离
                distances = self.calib_utils.calculate_adjacent_distances(
                    points_3d, self.pattern_size
                )
                
                # 绘制角点（将灰度图转换为BGR以便绘制彩色角点）
                gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                img_with_corners = self.calib_utils.draw_corners(
                    gray_bgr, corners, self.pattern_size
                )
                
                # 转换为base64（使用高质量JPEG）
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, buffer = cv2.imencode('.jpg', img_with_corners, encode_param)
                img_base64 = base64.b64encode(buffer).decode('utf-8')
                
                # 准备返回数据
                corners_data = []
                for i, (corner, point_3d, dist) in enumerate(zip(corners_2d, points_3d, distances)):
                    corners_data.append({
                        'index': i,
                        'pixel_u': float(corner[0]),
                        'pixel_v': float(corner[1]),
                        'camera_x': float(point_3d[0]),
                        'camera_y': float(point_3d[1]),
                        'camera_z': float(point_3d[2]),
                        'adjacent_distance': float(dist)
                    })
                
                # 深度误差检查（如果深度图可用）- 仅用于验证，不用于计算
                depth_error_result = None
                if self.current_depth_image is not None:
                    try:
                        depth_error_result = self.calib_utils.evaluate_depth_error(
                            corners, self.current_depth_image, self.pattern_size, square_size
                        )
                        if depth_error_result['success']:
                            self.get_logger().info(f'深度误差检查完成: 平均误差 {depth_error_result["depth_statistics"]["mean_error"]:.2f} mm')
                            self.get_logger().info('📊 深度数据仅用于验证，计算使用估计z值')
                    except Exception as e:
                        self.get_logger().warn(f'深度误差检查失败: {str(e)}')
                
                result = {
                    'success': True,
                    'corners_count': len(corners),
                    'corners_data': corners_data,
                    'image_with_corners': f'data:image/jpeg;base64,{img_base64}'
                }
                
                # 如果深度误差检查成功，添加到结果中
                if depth_error_result and depth_error_result.get('success'):
                    result['depth_error'] = depth_error_result
                
                return jsonify(result)
            except Exception as e:
                self.get_logger().error(f'提取角点失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/get_board_pose', methods=['POST'])
        def get_board_pose():
            """获取当前图像中标定板的姿态（用于姿态法标定）
            返回标定板坐标系到相机坐标系的变换（位置+四元数）
            """
            try:
                data = request.get_json()
                square_size = data.get('square_size', 20.0)  # mm（12x9格子，每格20mm）
                
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '请先加载图像'})
                
                if self.calib_utils.camera_matrix is None:
                    return jsonify({'success': False, 'error': '请先加载相机参数'})
                
                # 转换为灰度图进行角点检测（提高检测精度和速度）
                if len(self.current_image_raw.shape) == 3:
                    gray_image = cv2.cvtColor(self.current_image_raw, cv2.COLOR_BGR2GRAY)
                else:
                    gray_image = self.current_image_raw
                
                # 检测角点（先使用配置的尺寸）
                pattern_w, pattern_h = self.pattern_size
                corners = self.calib_utils.detect_checkerboard_corners(
                    gray_image, self.pattern_size
                )
                
                # 如果使用配置尺寸检测失败，尝试推断实际尺寸并重新检测
                actual_pattern_size = None
                if corners is None:
                    self.get_logger().warn(f'⚠️ 使用配置尺寸{pattern_w}x{pattern_h}未检测到角点，尝试其他尺寸...')
                    # 尝试常见的棋盘格尺寸（优先使用12x9格子的11x8角点）
                    common_sizes = [(11, 8), (8, 11), (9, 6), (6, 9), (8, 6), (6, 8), (7, 5), (5, 7)]
                    for test_size in common_sizes:
                        test_corners = self.calib_utils.detect_checkerboard_corners(
                            gray_image, test_size
                        )
                        if test_corners is not None and len(test_corners) == test_size[0] * test_size[1]:
                            corners = test_corners
                            actual_pattern_size = test_size
                            self.get_logger().info(f'✅ 使用尺寸{test_size[0]}x{test_size[1]}成功检测到角点')
                            break
                
                if corners is None:
                    error_msg = f'未检测到棋盘格角点（配置：{pattern_w}x{pattern_h}）'
                    self.get_logger().warn(f'⚠️ {error_msg}')
                    self.get_logger().warn(f'   建议：1) 确认棋盘格在相机视野中')
                    self.get_logger().warn(f'        2) 调整光照和距离')
                    self.get_logger().warn(f'        3) 确认棋盘格尺寸配置正确')
                    return jsonify({'success': False, 'error': error_msg})
                
                # 确定实际使用的尺寸
                if actual_pattern_size is None:
                    detected_corners = len(corners)
                    expected_corners = pattern_w * pattern_h
                    if detected_corners == expected_corners:
                        actual_pattern_size = (pattern_w, pattern_h)
                    else:
                        # 角点数量不匹配，尝试推断
                        self.get_logger().warn(f'⚠️ 角点数量不匹配：检测到{detected_corners}个，期望{expected_corners}个')
                        possible_sizes = []
                        for w in range(2, 20):
                            for h in range(2, 20):
                                if w * h == detected_corners:
                                    possible_sizes.append((w, h))
                        if possible_sizes:
                            possible_sizes.sort(key=lambda x: abs(x[0] - pattern_w) + abs(x[1] - pattern_h))
                            actual_pattern_size = possible_sizes[0]
                            self.get_logger().info(f'✅ 推断棋盘格尺寸为：{actual_pattern_size[0]}x{actual_pattern_size[1]}')
                            # 使用推断的尺寸重新检测以确保角点顺序正确（使用灰度图）
                            corners = self.calib_utils.detect_checkerboard_corners(
                                gray_image, actual_pattern_size
                            )
                            if corners is None:
                                error_msg = f'使用推断尺寸{actual_pattern_size[0]}x{actual_pattern_size[1]}重新检测失败'
                                return jsonify({'success': False, 'error': error_msg})
                        else:
                            error_msg = f'无法推断棋盘格尺寸：检测到{detected_corners}个角点'
                            return jsonify({'success': False, 'error': error_msg})
                
                # 使用实际检测到的尺寸构建3D对象点
                actual_w, actual_h = actual_pattern_size
                self.get_logger().info(f'📐 使用棋盘格尺寸：{actual_w}x{actual_h}，角点数：{len(corners)}')
                
                # 先使用所有角点进行初始solvePnP（用于后续深度滤波）
                obj_points_all = []
                for j in range(actual_h):
                    for i in range(actual_w):
                        obj_points_all.append([i * square_size, j * square_size, 0.0])
                obj_points_all = np.array(obj_points_all, dtype=np.float32)
                
                corners_2d_all = corners.reshape(-1, 1, 2).astype(np.float32)
                
                # 如果图像已去畸变，不使用畸变系数（避免双重校正）
                dist_coeffs_to_use = None if self._image_is_undistorted else self.calib_utils.dist_coeffs
                
                self.get_logger().info(f'🔍 solvePnP配置: 图像已去畸变={self._image_is_undistorted}, 使用畸变系数={dist_coeffs_to_use is not None}')
                
                # 初始solvePnP（使用所有角点）
                success, rvec, tvec = cv2.solvePnP(
                    obj_points_all,
                    corners_2d_all,
                    self.calib_utils.camera_matrix,
                    dist_coeffs_to_use
                )
                
                if not success:
                    return jsonify({'success': False, 'error': 'solvePnP求解失败'})
                
                # 将旋转向量转换为旋转矩阵
                R_board2cam, _ = cv2.Rodrigues(rvec)
                
                # 计算每个角点的3D坐标并进行深度滤波（见下方代码）
                # 然后使用滤波后的角点重新计算棋盘格位姿（欠定义方法）
                
                # （已清理遗留调试插桩）
                
                if not success:
                    return jsonify({'success': False, 'error': 'solvePnP求解失败'})
                
                # 计算重投影误差（用于评估solvePnP质量）
                # 重要：必须使用与solvePnP相同的畸变系数！
                projected_points, _ = cv2.projectPoints(
                    obj_points_all,
                    rvec,
                    tvec,
                    self.calib_utils.camera_matrix,
                    dist_coeffs_to_use  # 修复：使用与solvePnP相同的畸变系数
                )
                projected_points = projected_points.reshape(-1, 2)
                corners_reshaped = corners.reshape(-1, 2)
                reprojection_errors = np.linalg.norm(corners_reshaped - projected_points, axis=1)
                mean_reprojection_error = np.mean(reprojection_errors)
                max_reprojection_error = np.max(reprojection_errors)
                # （已清理遗留调试插桩）
                
                # 记录重投影误差（用于诊断）
                if mean_reprojection_error > 10.0:  # 误差超过10像素，严重问题
                    self.get_logger().error(f'❌ 重投影误差严重异常：平均 {mean_reprojection_error:.3f} 像素，最大 {max_reprojection_error:.3f} 像素')
                    self.get_logger().error(f'   正常情况下重投影误差应小于1像素！')
                    self.get_logger().error(f'   可能原因：')
                    self.get_logger().error(f'     1. 相机内参不准确（请重新标定相机内参）')
                    self.get_logger().error(f'     2. 棋盘格尺寸设置错误（当前检测到{len(corners)}个角点）')
                    self.get_logger().error(f'     3. 畸变校正参数不正确')
                    self.get_logger().error(f'     4. 标定板角点检测不准确')
                    self.get_logger().error(f'   建议：不要使用此姿态数据进行手眼标定！')
                elif mean_reprojection_error > 2.0:  # 误差超过2像素
                    self.get_logger().warning(f'⚠️ 重投影误差较大：平均 {mean_reprojection_error:.3f} 像素，最大 {max_reprojection_error:.3f} 像素')
                    self.get_logger().warning(f'   正常情况下应小于1像素，建议检查相机标定参数')
                elif mean_reprojection_error > 1.0:  # 误差超过1像素
                    self.get_logger().warning(f'⚠️ 重投影误差略高：平均 {mean_reprojection_error:.3f} 像素，最大 {max_reprojection_error:.3f} 像素')
                else:
                    self.get_logger().info(f'✅ 重投影误差良好：平均 {mean_reprojection_error:.3f} 像素，最大 {max_reprojection_error:.3f} 像素')
                
                # 将旋转向量转换为旋转矩阵
                R_board2cam, _ = cv2.Rodrigues(rvec)
                
                # 旋转矩阵转四元数（使用安全的转换函数）
                quat = self._rotation_matrix_to_quaternion(R_board2cam)  # [x, y, z, w]
                
                # tvec是标定板坐标系原点到相机坐标系原点的平移向量（在相机坐标系下）
                position = {
                    'x': float(tvec[0, 0]),
                    'y': float(tvec[1, 0]),
                    'z': float(tvec[2, 0])
                }
                
                # 保存旋转向量和平移向量（用于OpenCV calibrateHandEye，参考hand_eye_calibrate.py）
                rvec_data = {
                    'x': float(rvec[0, 0]),
                    'y': float(rvec[1, 0]),
                    'z': float(rvec[2, 0])
                }
                tvec_data = {
                    'x': float(tvec[0, 0]),
                    'y': float(tvec[1, 0]),
                    'z': float(tvec[2, 0])
                }
                # （已清理遗留调试插桩）
                
                # （已清理遗留调试插桩：depth comparison）
                import json,time;
                depth_z_values = [];
                depth_z_details = [];
                if self.current_depth_image is not None:
                    corners_2d_flat = corners.reshape(-1, 2);
                    for i in range(min(10, len(corners_2d_flat))):
                        px_u, px_v = int(corners_2d_flat[i,0]), int(corners_2d_flat[i,1]);
                        depth_z = self.calib_utils.get_depth_from_depth_image(self.current_depth_image, px_u, px_v, 3);
                        if depth_z is not None:
                            depth_z_values.append(float(depth_z));
                            depth_z_details.append({"corner_idx": i, "pixel": [int(px_u), int(px_v)], "depth_mm": float(depth_z)});
                
                # 深度数据质量检查（优化版：过滤离群点后计算）
                depth_quality_ok = True;
                depth_warning_msg = "";
                std_depth_z = 0.0;
                avg_depth_z = None;
                min_depth_z = None;
                max_depth_z = None;
                
                if len(depth_z_values) > 0:
                    # 原始统计
                    avg_depth_z_raw = float(np.mean(depth_z_values));
                    std_depth_z_raw = float(np.std(depth_z_values)) if len(depth_z_values) > 1 else 0.0;
                    min_depth_z = float(np.min(depth_z_values));
                    max_depth_z = float(np.max(depth_z_values));
                    
                    # 使用IQR方法过滤离群点（更鲁棒）
                    if len(depth_z_values) >= 3:
                        depth_z_array = np.array(depth_z_values);
                        q1 = np.percentile(depth_z_array, 25);
                        q3 = np.percentile(depth_z_array, 75);
                        iqr = q3 - q1;
                        # 使用1.5倍IQR作为离群点阈值（标准统计方法）
                        lower_bound = q1 - 1.5 * iqr;
                        upper_bound = q3 + 1.5 * iqr;
                        # 过滤离群点
                        filtered_depth = depth_z_array[(depth_z_array >= lower_bound) & (depth_z_array <= upper_bound)];
                        
                        if len(filtered_depth) >= 3:
                            # 使用过滤后的数据计算
                            avg_depth_z = float(np.mean(filtered_depth));
                            std_depth_z = float(np.std(filtered_depth));
                            outliers_removed = len(depth_z_values) - len(filtered_depth);
                            if outliers_removed > 0:
                                self.get_logger().info(f'📊 深度数据过滤：移除{outliers_removed}个离群点，保留{len(filtered_depth)}个有效值');
                        else:
                            # 过滤后数据太少，使用原始数据
                            avg_depth_z = avg_depth_z_raw;
                            std_depth_z = std_depth_z_raw;
                    else:
                        # 数据点太少，直接使用原始数据
                        avg_depth_z = avg_depth_z_raw;
                        std_depth_z = std_depth_z_raw;
                    
                    # 质量评估（仅用于警告，不影响标定计算）
                    if std_depth_z > 50.0:
                        depth_quality_ok = False;
                        depth_warning_msg = f"深度标准差过大({std_depth_z:.1f}mm)，深度相机数据异常";
                        self.get_logger().warning(f'⚠️ 深度数据质量警告: {depth_warning_msg}');
                        self.get_logger().warning(f'   深度范围: {min_depth_z:.1f} - {max_depth_z:.1f} mm (跨度{max_depth_z - min_depth_z:.1f}mm)');
                        self.get_logger().warning(f'   注意: 此警告不影响标定计算，仅供参考');
                    elif std_depth_z > 10.0:
                        self.get_logger().warning(f'⚠️ 深度数据质量一般: 标准差{std_depth_z:.1f}mm（建议<10mm）');
                    else:
                        self.get_logger().info(f'✅ 深度数据质量优秀: 标准差{std_depth_z:.1f}mm');
                
                # （已清理遗留调试插桩）
                
                corner_center_px = corners_reshaped.mean(axis=0).tolist();
                depth_data = {
                    "estimated_z_mm": float(tvec[2,0]),
                    "depth_z_mm": avg_depth_z,
                    "depth_z_std": std_depth_z,
                    "depth_z_min": min_depth_z,
                    "depth_z_max": max_depth_z,
                    "depth_z_samples": len(depth_z_values),
                    "depth_error_mm": float(tvec[2,0] - avg_depth_z) if avg_depth_z is not None else None,
                    "depth_error_pct": (float(tvec[2,0] - avg_depth_z) / avg_depth_z * 100.0) if avg_depth_z is not None and avg_depth_z > 0 else None,
                    "corner_center_px": [float(corner_center_px[0]), float(corner_center_px[1])],
                    "image_center_px": [320.0, 240.0],
                    "corner_offset_from_center": float(np.linalg.norm(np.array(corner_center_px) - np.array([320.0, 240.0]))),
                    "depth_details": depth_z_details[:5]
                };
                # （已清理遗留调试插桩）
                
                orientation = {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                }
                
                # 绘制角点和坐标轴到图像上（参考extract_corners的实现）
                # 将灰度图转换为BGR以便绘制彩色线条
                gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                # 首先使用draw_corners绘制角点（包含角点序号）
                img_with_features = self.calib_utils.draw_corners(
                    gray_bgr, corners, actual_pattern_size
                )
                
                # 绘制坐标轴（标定板坐标系）
                axis_length = 50.0  # mm
                axis_points_3d = np.array([
                    [0, 0, 0],           # 原点
                    [axis_length, 0, 0], # X轴
                    [0, axis_length, 0], # Y轴
                    [0, 0, -axis_length]  # Z轴（相机坐标系中Z向前）
                ], dtype=np.float32)
                
                # 投影3D点到2D图像
                axis_points_2d, _ = cv2.projectPoints(
                    axis_points_3d,
                    rvec,
                    tvec,
                    self.calib_utils.camera_matrix,
                    self.calib_utils.dist_coeffs
                )
                
                axis_points_2d = axis_points_2d.reshape(-1, 2).astype(int)
                
                # 绘制坐标轴
                origin = tuple(axis_points_2d[0])
                cv2.line(img_with_features, origin, tuple(axis_points_2d[1]), (0, 0, 255), 3)  # X轴：红色
                cv2.line(img_with_features, origin, tuple(axis_points_2d[2]), (0, 255, 0), 3)  # Y轴：绿色
                cv2.line(img_with_features, origin, tuple(axis_points_2d[3]), (255, 0, 0), 3)  # Z轴：蓝色
                
                # 编码图像为base64（与extract_corners保持一致，使用高质量JPEG）
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, buffer = cv2.imencode('.jpg', img_with_features, encode_param)
                img_base64 = base64.b64encode(buffer).decode('utf-8')
                
                # 计算每个角点在相机坐标系下的3D坐标，并进行深度滤波
                corners_3d = []
                corners_3d_filtered = []  # 滤波后的角点（用于手眼标定）
                corners_3d_filtered_indices = []  # 滤波后角点的原始索引
                
                if corners is not None:
                    corners_flat = corners.reshape(-1, 2)
                    
                    # 深度数据有效性检查阈值
                    DEPTH_VALID_MIN_MM = 100.0  # 最小有效深度（毫米）
                    DEPTH_VALID_MAX_MM = 2000.0  # 最大有效深度（毫米）
                    DEPTH_ERROR_THRESHOLD_PCT = 30.0  # 深度误差阈值（百分比）
                    
                    # 获取每个角点的实际深度值（用于滤波）
                    corner_depths = []
                    corner_depth_valid = []
                    
                    for j in range(actual_h):
                        for i in range(actual_w):
                            idx = j * actual_w + i
                            if idx < len(corners):
                                # 角点在标定板坐标系下的坐标
                                obj_point = np.array([i * square_size, j * square_size, 0.0], dtype=np.float32)
                                # 转换为相机坐标系
                                obj_point_cam = R_board2cam @ obj_point.reshape(3, 1) + tvec
                                
                                corner_3d = {
                                    'index': idx,  # 角点编号
                                    'x': float(obj_point_cam[0, 0]),
                                    'y': float(obj_point_cam[1, 0]),
                                    'z': float(obj_point_cam[2, 0]),
                                    'board_i': i,  # 标定板坐标系下的i坐标
                                    'board_j': j,  # 标定板坐标系下的j坐标
                                    'pixel_u': float(corners_flat[idx, 0]),
                                    'pixel_v': float(corners_flat[idx, 1])
                                }
                                
                                # 获取该角点的实际深度值（用于滤波）
                                actual_depth = None
                                if self.current_depth_image is not None:
                                    px_u = int(round(corner_3d['pixel_u']))
                                    px_v = int(round(corner_3d['pixel_v']))
                                    actual_depth = self.calib_utils.get_depth_from_depth_image(
                                        self.current_depth_image, px_u, px_v, search_radius=3
                                    )
                                
                                corner_3d['actual_depth_mm'] = float(actual_depth) if actual_depth is not None else None
                                corner_3d['estimated_depth_mm'] = float(obj_point_cam[2, 0])  # solvePnP估计的深度
                                
                                # 深度数据有效性检查
                                is_valid = True
                                validation_reason = ""
                                
                                if actual_depth is None:
                                    # 深度数据缺失（距离太近或超出范围）
                                    is_valid = False
                                    validation_reason = "深度数据缺失"
                                elif actual_depth < DEPTH_VALID_MIN_MM or actual_depth > DEPTH_VALID_MAX_MM:
                                    # 深度值超出合理范围
                                    is_valid = False
                                    validation_reason = f"深度值异常({actual_depth:.1f}mm)"
                                elif actual_depth > 0:
                                    # 检查深度误差（实际深度 vs 估计深度）
                                    depth_error_pct = abs(actual_depth - corner_3d['estimated_depth_mm']) / actual_depth * 100.0
                                    if depth_error_pct > DEPTH_ERROR_THRESHOLD_PCT:
                                        is_valid = False
                                        validation_reason = f"深度误差过大({depth_error_pct:.1f}%)"
                                
                                corner_3d['depth_valid'] = is_valid
                                corner_3d['validation_reason'] = validation_reason if not is_valid else ""
                                
                                corners_3d.append(corner_3d)
                                
                                # 只保留有效的角点用于手眼标定
                                if is_valid:
                                    corners_3d_filtered.append(corner_3d)
                                    corners_3d_filtered_indices.append(idx)
                                
                                corner_depths.append(actual_depth if actual_depth is not None else corner_3d['estimated_depth_mm'])
                                corner_depth_valid.append(is_valid)
                    
                    # 统计滤波结果
                    total_corners = len(corners_3d)
                    valid_corners = len(corners_3d_filtered)
                    filtered_corners = total_corners - valid_corners
                    
                    if filtered_corners > 0:
                        self.get_logger().warning(f'⚠️ 角点深度滤波：共{total_corners}个角点，过滤{filtered_corners}个异常角点，保留{valid_corners}个有效角点')
                        # 记录被过滤的角点信息
                        for corner in corners_3d:
                            if not corner['depth_valid']:
                                self.get_logger().info(f'   角点#{corner["index"]} (board[{corner["board_i"]},{corner["board_j"]}]): {corner["validation_reason"]}')
                    else:
                        self.get_logger().info(f'✅ 所有{total_corners}个角点深度数据有效')
                    
                    # 检查有效角点数量是否足够（至少需要4个角点才能进行solvePnP）
                    if valid_corners < 4:
                        self.get_logger().error(f'❌ 有效角点数量不足（{valid_corners}个），至少需要4个角点才能计算棋盘格位姿')
                        return jsonify({
                            'success': False,
                            'error': f'有效角点数量不足（{valid_corners}个），至少需要4个角点。可能原因：距离太近导致深度数据缺失或误差过大'
                        })
                    elif valid_corners < total_corners * 0.5:
                        self.get_logger().warning(f'⚠️ 有效角点比例较低（{valid_corners}/{total_corners}），可能影响标定精度')
                else:
                    # 如果没有角点数据，初始化空列表（向后兼容）
                    corners_3d = []
                    corners_3d_filtered = []
                    corners_3d_filtered_indices = []
                    total_corners = 0
                    valid_corners = 0
                    filtered_corners = 0
                
                # 准备角点列表（像素坐标）
                corners_list = []
                if corners is not None:
                    # 确保corners是(N, 2)形状的数组
                    corners_flat = corners.reshape(-1, 2)
                    for idx in range(len(corners_flat)):
                        corner = corners_flat[idx]
                        corners_list.append({
                            'index': idx,
                            'u': float(corner[0]),
                            'v': float(corner[1])
                        })
                
                # 如果没有执行深度检查，则默认为True（向后兼容）
                if 'depth_quality_ok' not in locals():
                    depth_quality_ok = True
                    depth_warning_msg = ""
                    std_depth_z = 0.0
                
                # 计算深度误差（如果深度数据可用）
                depth_error_mm = None
                depth_error_pct = None
                if avg_depth_z is not None and avg_depth_z > 0:
                    estimated_z = float(tvec[2, 0])  # solvePnP估计的z值
                    depth_error_mm = float(estimated_z - avg_depth_z)
                    depth_error_pct = float(depth_error_mm / avg_depth_z * 100.0)
                
                return jsonify({
                    'success': True,
                    'position': position,
                    'orientation': orientation,
                    'translation_mm': {
                        'x': position['x'],
                        'y': position['y'],
                        'z': position['z']
                    },
                    'rvec': rvec_data,  # 旋转向量（用于OpenCV calibrateHandEye，参考hand_eye_calibrate.py）
                    'tvec': tvec_data,  # 平移向量（用于OpenCV calibrateHandEye，参考hand_eye_calibrate.py）
                    'image_with_corners': f'data:image/jpeg;base64,{img_base64}',
                    'corners_count': len(corners) if corners is not None else 0,
                    'corners': corners_list,  # 角点像素坐标列表
                    'corners_3d': corners_3d,  # 角点相机坐标列表（包含所有角点，带深度验证信息）
                    'corners_3d_filtered': corners_3d_filtered,  # 滤波后的角点（用于手眼标定）
                    'corners_3d_filtered_indices': corners_3d_filtered_indices,  # 滤波后角点的原始索引
                    'filtering_stats': {
                        'total_corners': total_corners,
                        'valid_corners': valid_corners,
                        'filtered_corners': filtered_corners,
                        'valid_ratio': float(valid_corners / total_corners) if total_corners > 0 else 0.0
                    },
                    'reprojection_error': float(mean_reprojection_error),  # 平均重投影误差（像素）
                    'max_reprojection_error': float(max_reprojection_error),  # 最大重投影误差（像素）
                    'estimated_z_mm': float(tvec[2, 0]),  # solvePnP估计的z值（毫米）
                    'depth_avg_mm': float(avg_depth_z) if avg_depth_z is not None else None,  # 深度相机平均深度（毫米）
                    'depth_std_mm': float(std_depth_z) if std_depth_z > 0 else None,  # 深度标准差（毫米）
                    'depth_error_mm': depth_error_mm,  # 深度误差（估计z - 深度z，毫米）
                    'depth_error_pct': depth_error_pct,  # 深度误差百分比
                    'depth_quality_ok': depth_quality_ok,  # 深度数据质量标志
                    'depth_quality_warning': depth_warning_msg  # 深度质量警告信息
                })
                
            except Exception as e:
                self.get_logger().error(f'获取标定板姿态失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/pixel_to_camera', methods=['POST'])
        def pixel_to_camera():
            """像素坐标转相机坐标（手动选点）"""
            try:
                data = request.get_json()
                pixel_x = data.get('pixel_x')
                pixel_y = data.get('pixel_y')
                
                if pixel_x is None or pixel_y is None:
                    return jsonify({'success': False, 'error': '缺少像素坐标'})
                
                # 从字典中获取相机参数
                camera_matrix = self.camera_calibration_data.get('camera_matrix')
                dist_coeffs = self.camera_calibration_data.get('dist_coeffs')
                
                if camera_matrix is None or dist_coeffs is None:
                    return jsonify({'success': False, 'error': '相机参数未加载，请先点击"加载相机参数"按钮'})
                
                # 转换为numpy数组（如果还不是的话）
                if not isinstance(camera_matrix, np.ndarray):
                    camera_matrix = np.array(camera_matrix, dtype=np.float64)
                if not isinstance(dist_coeffs, np.ndarray):
                    dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
                
                self.get_logger().info(f'像素坐标转换: ({pixel_x}, {pixel_y})')
                self.get_logger().info(f'相机矩阵类型: {type(camera_matrix)}, 形状: {camera_matrix.shape}')
                
                # 将像素坐标转换为相机坐标（假设Z=0平面）
                # 使用反投影，假设物体在某个固定深度（如300mm）
                # 这里简化处理：先去畸变，再通过相机内参反投影
                
                pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
                undistorted = cv2.undistortPoints(pixel_point, camera_matrix, dist_coeffs, P=camera_matrix)
                
                # 假设深度为300mm（可以根据棋盘格平面深度估算）
                depth = 300.0  # mm
                
                # 从归一化坐标转回相机坐标
                u_norm = undistorted[0][0][0]
                v_norm = undistorted[0][0][1]
                
                fx = camera_matrix[0, 0]
                fy = camera_matrix[1, 1]
                cx = camera_matrix[0, 2]
                cy = camera_matrix[1, 2]
                
                camera_x = (u_norm - cx) * depth / fx
                camera_y = (v_norm - cy) * depth / fy
                camera_z = depth
                
                self.get_logger().info(f'相机坐标: X={camera_x:.2f}, Y={camera_y:.2f}, Z={camera_z:.2f} mm')
                
                return jsonify({
                    'success': True,
                    'camera_x': float(camera_x),
                    'camera_y': float(camera_y),
                    'camera_z': float(camera_z)
                })
                
            except Exception as e:
                self.get_logger().error(f'像素坐标转换失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/calculate_result', methods=['POST'])
        def calculate_result():
            """计算结果"""
            try:
                data = request.get_json()
                expected_size = data.get('expected_size', 20.0)  # mm（12x9格子，每格20mm）
                
                if self.corners_3d is None:
                    return jsonify({'success': False, 'error': '请先提取角点'})
                
                if self.detected_corners is None:
                    return jsonify({'success': False, 'error': '请先提取角点'})
                
                # 计算最大距离和误差
                result = self.calib_utils.calculate_max_distance_error(
                    self.corners_3d, self.pattern_size, expected_size
                )
                
                # 深度误差检查（如果深度图数据可用）- 仅用于验证，不用于计算
                if self.current_depth_image is not None:
                    try:
                        depth_error_result = self.calib_utils.evaluate_depth_error(
                            self.detected_corners,
                            self.current_depth_image,
                            self.pattern_size,
                            expected_size
                        )
                        if depth_error_result.get('success'):
                            result['depth_error'] = depth_error_result
                            self.get_logger().info(f'深度误差检查完成: 平均误差 {depth_error_result["depth_statistics"]["mean_error"]:.2f} mm')
                            self.get_logger().info('📊 深度数据仅用于验证，计算使用估计z值')
                    except Exception as e:
                        self.get_logger().warn(f'深度误差检查失败: {str(e)}')
                
                # 保存标定精度验证结果到文件
                try:
                    import time
                    accuracy_result = {
                        'timestamp': time.time(),
                        'timestamp_readable': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'checkerboard_size': f"{self.pattern_size[0]+1}x{self.pattern_size[1]+1}",
                        'square_size_mm': expected_size,
                        'corner_count': len(self.corners_3d) if self.corners_3d is not None else 0,
                        'adjacent_distances': result.get('adjacent_distances', {}),
                        'diagonal_distance': result.get('diagonal_distance', {}),
                        'depth_error': result.get('depth_error', {}) if 'depth_error' in result else None,
                        'camera_intrinsics': {
                            'fx': float(self.calib_utils.camera_matrix[0, 0]),
                            'fy': float(self.calib_utils.camera_matrix[1, 1]),
                            'cx': float(self.calib_utils.camera_matrix[0, 2]),
                            'cy': float(self.calib_utils.camera_matrix[1, 2])
                        } if self.calib_utils.camera_matrix is not None else None
                    }
                    
                    # 保存到config目录
                    config_dir = os.path.join(
                        self.get_package_share_directory('hand_eye_calibration'),
                        'config'
                    )
                    accuracy_file = os.path.join(config_dir, 'camera_calibration_accuracy.json')
                    
                    with open(accuracy_file, 'w', encoding='utf-8') as f:
                        accuracy_result_clean = _convert_to_json_serializable(accuracy_result)
                        json.dump(accuracy_result_clean, f, indent=2, ensure_ascii=False)
                    
                    self.get_logger().info(f'✅ 相机标定精度结果已保存: {accuracy_file}')
                except Exception as e:
                    self.get_logger().warn(f'⚠️ 保存相机标定精度结果失败: {str(e)}')
                
                return jsonify(result)
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/camera/undistort_image', methods=['POST'])
        def undistort_image():
            """对当前图像进行去畸变处理"""
            try:
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '没有可用的图像'})
                
                if self.calib_utils.camera_matrix is None or self.calib_utils.dist_coeffs is None:
                    return jsonify({'success': False, 'error': '请先加载相机标定参数'})
                
                self.get_logger().info('='*60)
                self.get_logger().info('🔧 开始去畸变处理')
                self.get_logger().info(f'   原始图像尺寸: {self.current_image_raw.shape}')
                
                # 使用OpenCV进行去畸变
                h, w = self.current_image_raw.shape[:2]
                
                # 获取最优新相机矩阵 (alpha=0: 裁剪黑边, alpha=1: 保留所有像素)
                # 使用 alpha=0 获得更紧凑的结果，避免黑边
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    self.calib_utils.camera_matrix,
                    self.calib_utils.dist_coeffs,
                    (w, h),
                    0,  # alpha=0 裁剪黑边，获得最大有效区域
                    (w, h)
                )
                
                # 去畸变
                undistorted = cv2.undistort(
                    self.current_image_raw,
                    self.calib_utils.camera_matrix,
                    self.calib_utils.dist_coeffs,
                    None,
                    new_camera_matrix
                )
                
                # 裁剪ROI区域（移除黑边）
                x, y, w_roi, h_roi = roi
                if x > 0 or y > 0 or w_roi < w or h_roi < h:
                    if w_roi > 0 and h_roi > 0:
                        undistorted = undistorted[y:y+h_roi, x:x+w_roi]
                        self.get_logger().info(f'   ROI裁剪: ({x}, {y}, {w_roi}, {h_roi})')
                        self.get_logger().info(f'   裁剪前尺寸: {w}x{h}, 裁剪后尺寸: {w_roi}x{h_roi}')
                
                # 更新当前图像
                self.current_image = undistorted
                self.current_image_raw = undistorted.copy()
                self._image_is_undistorted = True  # 标记图像已去畸变
                
                # 转换为base64 - 使用较低质量以减小数据量
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                success, buffer = cv2.imencode('.jpg', undistorted, encode_param)
                
                if not success:
                    self.get_logger().error('❌ JPEG编码失败')
                    return jsonify({'success': False, 'error': 'JPEG编码失败'})
                
                img_base64 = base64.b64encode(buffer).decode('utf-8')
                
                self.get_logger().info(f'✅ 去畸变完成')
                self.get_logger().info(f'   处理后尺寸: {undistorted.shape}')
                self.get_logger().info(f'   JPEG大小: {len(buffer)/1024:.1f} KB')
                self.get_logger().info(f'   Base64长度: {len(img_base64)} 字符 ({len(img_base64)/1024:.1f} KB)')
                self.get_logger().info('='*60)
                
                return jsonify({
                    'success': True,
                    'image': f'data:image/jpeg;base64,{img_base64}',
                    'width': undistorted.shape[1],
                    'height': undistorted.shape[0],
                    'message': '图像去畸变成功'
                })
                
            except Exception as e:
                self.get_logger().error(f'❌ 去畸变失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/hand_eye/capture_pose', methods=['POST'])
        def capture_pose():
            """捕获当前位姿用于手眼标定"""
            # TODO: 实现位姿捕获功能
            return jsonify({'success': True, 'message': '功能待实现'})
        
        def _perform_eye_in_hand_pose_based_calibration(data):
            """执行Eye-in-Hand标定 - 姿态法（Pose-based method，类似Tsai方法）
            
            姿态法原理：通过多组机器人运动，求解AX=XB问题
            - A: 机器人基座坐标系下的运动（从姿态1到姿态2）
            - X: 相机到末端执行器的变换（待求）
            - B: 相机坐标系下的运动（通过观察标定板计算）
            
            流程：
            1. 使用Tsai方法或SVD方法求解初始估计
            2. 使用非线性优化进一步优化结果
            
            支持两种方法：
            - 'custom': Custom模式（XY+Z约束方法，参考hand_eye_calibration.py）
            - 'opencv': OpenCV模式（calibrateHandEye方法，TSAI算法）
            
            参数:
                data: 包含以下字段的字典:
                    - poses_list: 位姿列表（必需），每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                    - method: 'custom' 或 'opencv'（可选，默认'custom'）
            """
            try:
                # 自动手眼标定采集的是单个位姿列表，不需要配对成运动组
                method = data.get('method', 'custom')  # 'custom' 或 'opencv'
                
                # 验证方法参数
                if method not in ['custom', 'opencv']:
                    return jsonify({
                        'success': False,
                        'error': f'不支持的方法: {method}，支持的方法: custom, opencv'
                    })
                
                # 自动手眼标定使用位姿列表格式
                poses_list = data.get('poses_list', [])
                
                # 验证数据格式和数量
                if len(poses_list) == 0:
                    return jsonify({
                        'success': False,
                        'error': '缺少数据：需要poses_list（位姿列表），至少需要3个位姿'
                    })
                
                # 位姿列表格式：每个元素包含{'robot_pose': {...}, 'board_pose': {...}}
                self.get_logger().info(f'📊 开始Eye-in-Hand姿态法标定（位姿列表格式）')
                self.get_logger().info(f'   标定方法: {method}')
                self.get_logger().info(f'   位姿数量: {len(poses_list)}')
                input_data = poses_list  # 直接使用位姿列表
                
                # 验证数据数量
                if len(poses_list) < 3:
                    return jsonify({
                        'success': False, 
                        'error': f'位姿数据不足，至少需要3个位姿，当前只有{len(poses_list)}个'
                    })
                
                # 检查相机标定参数
                if self.calib_utils.camera_matrix is None or self.calib_utils.dist_coeffs is None:
                    return jsonify({
                        'success': False, 
                        'error': '请先加载相机标定参数'
                    })
                
                # 根据方法选择执行标定
                # （已清理遗留调试插桩）
                if method == 'opencv':
                    # 使用OpenCV calibrateHandEye方法
                    # OpenCV模式使用独立的模块（opencv_hand_eye_calibration.py）
                    # OpenCV模式直接处理位姿列表
                    # 获取OpenCV算法选择参数（如果提供）
                    opencv_algorithm = data.get('opencv_algorithm', 'TSAI')
                    
                    # 获取算法名称用于日志
                    algorithm_names = {
                        'TSAI': 'TSAI',
                        'PARK': 'PARK',
                        'HORAUD': 'HORAUD',
                        'ANDREFF': 'ANDREFF',
                        'DANIILIDIS': 'DANIILIDIS'
                    }
                    algorithm_name = algorithm_names.get(opencv_algorithm, opencv_algorithm)
                    self.get_logger().info(f'   [OpenCV模式] 开始标定计算（{algorithm_name}方法）...')
                    self.get_logger().info(f'   使用位姿列表格式：{len(poses_list)} 个位姿')
                    
                    try:
                        # 使用OpenCV模式模块进行完整标定流程
                        # prepare_data方法直接处理位姿列表
                        opencv_calib = OpenCVHandEyeCalibration(logger=self.get_logger())
                        result = opencv_calib.calibrate(input_data, opencv_algorithm=opencv_algorithm)
                        
                        T_camera2gripper = result['T_camera2gripper']
                        translation_errors = result['translation_errors']
                        rotation_errors = result['rotation_errors']
                        error_statistics = result['error_statistics']
                        T_gripper_list = result['T_gripper_list']
                        T_board_list = result['T_board_list']
                        
                        # 提取误差统计信息
                        mean_translation_error = error_statistics['mean_translation_error']
                        mean_rotation_error = error_statistics['mean_rotation_error_rad']
                        max_translation_error = error_statistics['max_translation_error']
                        min_translation_error = error_statistics['min_translation_error']
                        std_translation_error = error_statistics['std_translation_error']
                        max_rotation_error = error_statistics['max_rotation_error_rad']
                        min_rotation_error = error_statistics['min_rotation_error_rad']
                        
                        self.get_logger().info(f'✅ [OpenCV模式] 标定完成（{algorithm_name}方法）')
                        self.get_logger().info(f'   平移误差统计:')
                        self.get_logger().info(f'     RMS误差: {mean_translation_error:.3f} mm')
                        self.get_logger().info(f'     最大误差: {max_translation_error:.3f} mm')
                        self.get_logger().info(f'     最小误差: {min_translation_error:.3f} mm')
                        self.get_logger().info(f'     标准差: {std_translation_error:.3f} mm')
                        self.get_logger().info(f'   旋转误差统计:')
                        self.get_logger().info(f'     RMS误差: {mean_rotation_error:.6f} rad ({np.degrees(mean_rotation_error):.3f}°)')
                        self.get_logger().info(f'     最大误差: {max_rotation_error:.6f} rad ({np.degrees(max_rotation_error):.3f}°)')
                        self.get_logger().info(f'     最小误差: {min_rotation_error:.6f} rad ({np.degrees(min_rotation_error):.3f}°)')
                        
                        # 显示每个姿态对的误差
                        if len(translation_errors) > 0:
                            self.get_logger().info(f'   各姿态对误差（AX=XB约束验证）:')
                            for i, (t_err, r_err) in enumerate(zip(translation_errors, rotation_errors)):
                                self.get_logger().info(f'     姿态对 #{i+1}: 平移误差 {t_err:.3f} mm, 旋转误差 {np.degrees(r_err):.3f}°')
                        
                        # 从T_camera2gripper提取R和t
                        # 注意：T_camera2gripper 从 opencv_calib.calibrate() 返回时，平移向量单位已经是毫米
                        R_cam2gripper = T_camera2gripper[:3, :3]
                        t_cam2gripper_mm = T_camera2gripper[:3, 3].flatten()  # 单位：毫米
                        t_cam2gripper_m = t_cam2gripper_mm / 1000.0  # 转换为米
                        t_cam2gripper = t_cam2gripper_mm  # 保持向后兼容，用于后续代码
                        
                        # 显示相机内参
                        if self.calib_utils.camera_matrix is not None:
                            camera_matrix = self.calib_utils.camera_matrix
                            self.get_logger().info(f'   📷 使用的相机内参:')
                            self.get_logger().info(f'     焦距: fx={camera_matrix[0,0]:.6f}, fy={camera_matrix[1,1]:.6f}')
                            self.get_logger().info(f'     主点: cx={camera_matrix[0,2]:.6f}, cy={camera_matrix[1,2]:.6f}')
                            if self.calib_utils.dist_coeffs is not None:
                                dist_coeffs_str = ', '.join([f'{d:.6f}' for d in self.calib_utils.dist_coeffs.flatten()[:5]])
                                self.get_logger().info(f'     畸变系数: [{dist_coeffs_str}]')
                            if self.calib_utils.image_size is not None:
                                self.get_logger().info(f'     图像尺寸: {self.calib_utils.image_size[0]}×{self.calib_utils.image_size[1]}')
                        
                        # 显示标定结果
                        self.get_logger().info(f'   📐 标定结果 (Camera→Gripper):')
                        self.get_logger().info(f'     旋转矩阵 R_cam2gripper:')
                        for i in range(3):
                            self.get_logger().info(f'       [{R_cam2gripper[i,0]:8.6f}, {R_cam2gripper[i,1]:8.6f}, {R_cam2gripper[i,2]:8.6f}]')
                        self.get_logger().info(f'     平移向量 t_cam2gripper:')
                        self.get_logger().info(f'        (米) [{t_cam2gripper_m[0]:.6f}, {t_cam2gripper_m[1]:.6f}, {t_cam2gripper_m[2]:.6f}]')
                        self.get_logger().info(f'        (毫米) [{t_cam2gripper_mm[0]:.3f}, {t_cam2gripper_mm[1]:.3f}, {t_cam2gripper_mm[2]:.3f}]')
                        
                        # 保存标定结果
                        self.hand_eye_calibration_data['transformation_matrix'] = T_camera2gripper.tolist()
                        self.hand_eye_calibration_data['calibration_error'] = {
                            'mean': float(mean_translation_error),
                            'max': float(max_translation_error),
                            'min': float(min_translation_error),
                            'std': float(std_translation_error),
                            'mean_rotation_error_rad': float(mean_rotation_error),
                            'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
                            'errors': [float(e) for e in translation_errors],
                            'rotation_errors_rad': [float(e) for e in rotation_errors],
                            'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
                        }
                        
                        # 保存标定数据到文件（用于验证）
                        try:
                            from datetime import datetime
                            import json as json_module
                            
                            # 在根目录创建验证数据文件夹
                            validation_dir = os.path.join('/home/mu/IVG', 'hand_eye_calibration_validation')
                            os.makedirs(validation_dir, exist_ok=True)
                            
                            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                            
                            # 保存完整的标定数据
                            saved_calibration_data = {
                                'timestamp': timestamp,
                                'timestamp_readable': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                'method': f'OpenCV calibrateHandEye ({algorithm_name})',
                                'calibration_type': 'Eye-in-Hand',
                                'solver_method': 'TSAI',
                                'num_pose_pairs': len(translation_errors),  # 姿态对数量
                                'num_poses': len(T_gripper_list),
                                
                                # 标定结果
                                'T_camera2gripper': T_camera2gripper.tolist(),
                                'R_cam2gripper': R_cam2gripper.tolist(),
                                't_cam2gripper': t_cam2gripper.tolist(),
                                
                                # 输入数据：所有姿态
                                'gripper_poses': [],
                                'board_poses': [],
                                
                                # 位姿数据（已用于标定）
                                'poses_list': [],
                                
                                # 误差统计
                                'error_statistics': {
                                    'translation': {
                                        'rms': float(mean_translation_error),
                                        'max': float(max_translation_error),
                                        'min': float(min_translation_error),
                                        'std': float(std_translation_error),
                                        'errors': [float(e) for e in translation_errors]
                                    },
                                    'rotation': {
                                        'rms_rad': float(mean_rotation_error),
                                        'rms_deg': float(np.degrees(mean_rotation_error)),
                                        'max_rad': float(max_rotation_error),
                                        'max_deg': float(np.degrees(max_rotation_error)),
                                        'min_rad': float(min_rotation_error),
                                        'min_deg': float(np.degrees(min_rotation_error)),
                                        'errors_rad': [float(e) for e in rotation_errors],
                                        'errors_deg': [float(np.degrees(e)) for e in rotation_errors]
                                    }
                                },
                                
                                # 数据质量诊断
                                'pose_pair_quality': [],
                                
                                # 姿态对数据
                                'pose_pairs': []
                            }
                            
                            # 保存所有姿态数据
                            for i in range(len(T_gripper_list)):
                                T_gripper = T_gripper_list[i]
                                T_board = T_board_list[i]
                                
                                saved_calibration_data['gripper_poses'].append({
                                    'index': i,
                                    'R': T_gripper[:3, :3].tolist(),
                                    't': T_gripper[:3, 3].tolist(),
                                    'T': T_gripper.tolist()
                                })
                                
                                saved_calibration_data['board_poses'].append({
                                    'index': i,
                                    'R': T_board[:3, :3].tolist(),
                                    't': T_board[:3, 3].tolist(),
                                    'T': T_board.tolist()
                                })
                            
                            # 保存姿态对数据和质量诊断
                            # 基于相邻姿态对（i, i+1）构建姿态对
                            num_poses = len(T_gripper_list)
                            num_saved_motions = num_poses - 1  # 相邻姿态对的数量
                            
                            for i in range(num_saved_motions):
                                pose1_idx = i
                                pose2_idx = i + 1
                                
                                if pose2_idx >= len(T_gripper_list):
                                    continue
                                
                                T_gripper1 = T_gripper_list[pose1_idx]
                                T_gripper2 = T_gripper_list[pose2_idx]
                                T_board1 = T_board_list[pose1_idx]
                                T_board2 = T_board_list[pose2_idx]
                                
                                # 计算运动幅度
                                T_gripper_motion = np.linalg.inv(T_gripper1) @ T_gripper2
                                robot_translation = float(np.linalg.norm(T_gripper_motion[:3, 3]))
                                robot_rotation_angle = float(np.arccos(np.clip((np.trace(T_gripper_motion[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                                
                                T_board_motion = T_board1 @ np.linalg.inv(T_board2)
                                board_translation = float(np.linalg.norm(T_board_motion[:3, 3]))
                                board_rotation_angle = float(np.arccos(np.clip((np.trace(T_board_motion[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                                
                                saved_calibration_data['pose_pairs'].append({
                                    'index': i,
                                    'pose1_idx': pose1_idx,
                                    'pose2_idx': pose2_idx,
                                    'T_gripper1': T_gripper1.tolist(),
                                    'T_gripper2': T_gripper2.tolist(),
                                    'T_board1': T_board1.tolist(),
                                    'T_board2': T_board2.tolist()
                                })
                                
                                saved_calibration_data['pose_pair_quality'].append({
                                    'pose_pair_index': i + 1,
                                    'robot_translation_mm': robot_translation,
                                    'robot_rotation_deg': robot_rotation_angle,
                                    'board_translation_mm': board_translation,
                                    'board_rotation_deg': board_rotation_angle,
                                    'translation_error_mm': float(translation_errors[i]) if i < len(translation_errors) else 0.0,
                                    'rotation_error_deg': float(np.degrees(rotation_errors[i])) if i < len(rotation_errors) else 0.0
                                })
                            
                            # 保存到JSON文件
                            data_file = os.path.join(validation_dir, f'calibration_data_{timestamp}.json')
                            with open(data_file, 'w', encoding='utf-8') as f:
                                saved_calibration_data_clean = _convert_to_json_serializable(saved_calibration_data)
                                json_module.dump(saved_calibration_data_clean, f, indent=2, ensure_ascii=False)
                            
                            self.get_logger().info(f'✅ 标定数据已保存到: {data_file}')
                            
                        except Exception as e:
                            self.get_logger().warn(f'⚠️ 保存标定数据失败: {str(e)}')
                            import traceback
                            self.get_logger().warn(traceback.format_exc())
                        
                        # 构建响应数据，确保所有NumPy类型都转换为Python原生类型
                        response_data = {
                            'success': True,
                            'method': f'OpenCV calibrateHandEye ({algorithm_name})',
                            'opencv_algorithm': opencv_algorithm,  # 添加算法信息
                            'calibration_type': 'Eye-in-Hand',
                            'transformation_matrix': T_camera2gripper.tolist(),
                            'rotation_matrix': R_cam2gripper.tolist(),
                            'translation_vector': t_cam2gripper.tolist(),  # 单位：毫米
                            'units': {
                                'translation': 'mm',
                                'rotation': 'rad',
                                'error': 'mm'
                            },
                            'evaluation': {
                                'mean_error': float(mean_translation_error),
                                'max_error': float(max_translation_error),
                                'min_error': float(min_translation_error),
                                'std_error': float(std_translation_error),
                                'mean_rotation_error_rad': float(mean_rotation_error),
                                'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
                                'max_rotation_error_rad': float(max_rotation_error),
                                'max_rotation_error_deg': float(np.degrees(max_rotation_error)),
                                'min_rotation_error_rad': float(min_rotation_error),
                                'min_rotation_error_deg': float(np.degrees(min_rotation_error)),
                                'data_count': len(translation_errors),  # 姿态对数量
                                'errors_per_pose_pair': [float(e) for e in translation_errors],  # 每个姿态对的平移误差（AX=XB约束验证）
                                'rotation_errors_per_pose_pair_rad': [float(e) for e in rotation_errors],
                                'rotation_errors_per_pose_pair_deg': [float(np.degrees(e)) for e in rotation_errors],
                                'rotation_errors_per_pose_rad': [float(e) for e in rotation_errors],
                                'rotation_errors_per_pose_deg': [float(np.degrees(e)) for e in rotation_errors]
                            },
                            'message': f'OpenCV calibrateHandEye标定成功（{algorithm_name}方法），平移RMS误差: {mean_translation_error:.3f} mm, 旋转RMS误差: {np.degrees(mean_rotation_error):.3f}°'
                        }
                        # 确保所有NumPy类型都被转换为Python原生类型
                        response_data_clean = _convert_to_json_serializable(response_data)
                        return jsonify(response_data_clean)
                        
                    except Exception as e:
                        self.get_logger().error(f'❌ OpenCV calibrateHandEye标定失败（{algorithm_name}方法）: {str(e)}')
                        import traceback
                        self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                        return jsonify({
                            'success': False, 
                            'error': f'OpenCV标定失败: {str(e)}'
                        })
                else:
                    # 使用Custom模式（完全参考hand_eye_calibration.py实现，XY+Z约束方法）
                    # Custom模式使用独立的模块（custom_hand_eye_calibration.py）
                    # Custom模式直接处理位姿列表
                    self.get_logger().info(f'   [Custom模式] 开始标定计算（XY+Z约束方法，参考hand_eye_calibration.py）...')
                    self.get_logger().info(f'   使用位姿列表格式：{len(poses_list)} 个位姿')
                    
                    try:
                        # 使用Custom模式模块进行完整标定流程
                        custom_calib = CustomHandEyeCalibration(logger=self.get_logger())
                        result = custom_calib.calibrate(input_data)
                        
                        T_camera2gripper = result['T_camera2gripper']
                        fixed_z = result['fixed_z']
                        errors = result['errors']
                        error_statistics = result['error_statistics']
                        shot_pose = result['shot_pose']
                        pick_poses = result['pick_poses']
                        camera_points_shot = result['camera_points_shot']
                        
                        # 提取误差统计信息
                        mean_error = error_statistics['mean_error']
                        max_error = error_statistics['max_error']
                        min_error = error_statistics['min_error']
                        std_error = error_statistics['std_error']
                        
                        self.get_logger().info('✅ Custom模式标定完成（XY+Z约束方法）')
                        self.get_logger().info(f'   平均误差: {mean_error:.3f} mm')
                        self.get_logger().info(f'   最大误差: {max_error:.3f} mm')
                        self.get_logger().info(f'   最小误差: {min_error:.3f} mm')
                        self.get_logger().info(f'   标准差: {std_error:.3f} mm')
                        self.get_logger().info(f'   固定Z值（棋盘格桌面高度）: {fixed_z:.6f} mm')
                        
                        # 详细诊断：每个数据点的误差
                        self.get_logger().info(f'   详细诊断信息:')
                        for i in range(len(errors)):
                            P_camera_shot = camera_points_shot[i]
                            T_gripper_pick2base = pick_poses[i]
                            P_base_actual = T_gripper_pick2base[:3, 3]
                            
                            P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
                            P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
                            P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
                            P_base_computed = P_base_computed_homogeneous[:3]
                            
                            self.get_logger().info(f'     数据点 #{i+1}:')
                            self.get_logger().info(f'       误差: {errors[i]:.3f} mm')
                            self.get_logger().info(f'       相机坐标: [{P_camera_shot[0]:.2f}, {P_camera_shot[1]:.2f}, {P_camera_shot[2]:.2f}] mm')
                            self.get_logger().info(f'       实际位置: [{P_base_actual[0]:.2f}, {P_base_actual[1]:.2f}, {P_base_actual[2]:.2f}] mm')
                            self.get_logger().info(f'       计算位置: [{P_base_computed[0]:.2f}, {P_base_computed[1]:.2f}, {P_base_computed[2]:.2f}] mm')
                        
                        # 数据质量检查
                        if len(pick_poses) < 5:
                            self.get_logger().warning(f'   ⚠️ 数据点较少（{len(pick_poses)}个），建议至少5-10个以提高精度')
                        
                        # 检查误差分布
                        if std_error > mean_error * 0.5:
                            self.get_logger().warning(f'   ⚠️ 误差标准差较大（{std_error:.3f} mm），可能存在异常数据点')
                        
                        # 误差过大时的改进建议
                        if mean_error > 10.0:
                            self.get_logger().warning(f'   ⚠️ 标定误差较大（{mean_error:.3f} mm），可能原因:')
                            self.get_logger().warning(f'     - 检查相机标定参数是否准确')
                            self.get_logger().warning(f'     - 检查标定板位姿检测是否准确（solvePnP质量）')
                            self.get_logger().warning(f'     - 检查机器人位姿数据是否准确')
                            self.get_logger().warning(f'     - 确保标定板在相机视野内且清晰可见')
                            self.get_logger().warning(f'     - 增加数据点数量（当前{len(pick_poses)}个），建议5-10个')
                            self.get_logger().warning(f'     - 确保数据点分布在不同位置和角度')
                        
                        # 诊断：计算运动幅度（用于诊断，但不再用于误差计算）
                        # 直接从pick_poses计算相邻位姿之间的运动，而不是从motion_groups
                        motion_amplitudes = []
                        poses_for_diagnosis = []  # 用于存储诊断用的位姿数据
                        
                        # 如果input_data是位姿列表，直接使用；如果是运动组，转换为位姿列表
                        if len(input_data) > 0 and 'robot_pose' in input_data[0]:
                            # 位姿列表格式
                            poses_for_diagnosis = input_data
                        elif len(input_data) > 0 and 'pose1' in input_data[0]:
                            # 运动组格式，转换为位姿列表（用于诊断）
                            for motion in input_data:
                                if 'pose1' in motion and 'board_pose1' in motion:
                                    poses_for_diagnosis.append({
                                        'robot_pose': motion['pose1'],
                                        'board_pose': motion['board_pose1']
                                    })
                                if 'pose2' in motion and 'board_pose2' in motion:
                                    poses_for_diagnosis.append({
                                        'robot_pose': motion['pose2'],
                                        'board_pose': motion['board_pose2']
                                    })
                        
                        # 计算相邻位姿之间的运动幅度（用于诊断）
                        for i in range(len(pick_poses) - 1):
                            if i + 1 >= len(pick_poses):
                                continue
                            
                            T_gripper1 = pick_poses[i]
                            T_gripper2 = pick_poses[i + 1]
                            
                            # 计算机器人运动（相邻位姿之间的运动）
                            T_gripper1_inv = np.linalg.inv(T_gripper1)
                            A = T_gripper1_inv @ T_gripper2
                            
                            Ra = A[:3, :3]
                            ta = A[:3, 3]
                            
                            # 旋转角度（从旋转矩阵提取，使用trace方法更可靠）
                            try:
                                # 方法1：使用trace计算旋转角度（更可靠）
                                trace = np.trace(Ra)
                                # trace(R) = 1 + 2*cos(θ)，所以 cos(θ) = (trace(R) - 1) / 2
                                cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
                                angle_a = np.arccos(cos_angle)
                                angle_a_deg = np.degrees(angle_a)
                                
                                # 如果角度很小，尝试使用as_rotvec方法验证
                                if angle_a_deg < 0.01:
                                    try:
                                        r_a = R.from_matrix(Ra)
                                        angle_a_vec = np.linalg.norm(r_a.as_rotvec())
                                        angle_a_deg_vec = np.degrees(angle_a_vec)
                                        # 如果两种方法差异很大，使用as_rotvec的结果
                                        if abs(angle_a_deg - angle_a_deg_vec) > 0.1:
                                            angle_a_deg = angle_a_deg_vec
                                    except:
                                        pass
                            except Exception as e:
                                # 如果trace方法失败，尝试as_rotvec
                                try:
                                    r_a = R.from_matrix(Ra)
                                    angle_a = np.linalg.norm(r_a.as_rotvec())
                                    angle_a_deg = np.degrees(angle_a)
                                except:
                                    angle_a_deg = 0.0
                            
                            trans_a = np.linalg.norm(ta)
                            
                            # 获取重投影误差（如果可用，从poses_for_diagnosis中获取）
                            reproj_error1 = None
                            reproj_error2 = None
                            if i < len(poses_for_diagnosis):
                                reproj_error1 = poses_for_diagnosis[i].get('board_pose', {}).get('reprojection_error')
                            if i + 1 < len(poses_for_diagnosis):
                                reproj_error2 = poses_for_diagnosis[i + 1].get('board_pose', {}).get('reprojection_error')
                            
                            reproj_info = ""
                            if reproj_error1 is not None or reproj_error2 is not None:
                                reproj_errors = [e for e in [reproj_error1, reproj_error2] if e is not None]
                                if len(reproj_errors) > 0:
                                    avg_reproj = np.mean(reproj_errors)
                                    max_reproj = np.max(reproj_errors)
                                    reproj_info = f", 重投影误差: 平均 {avg_reproj:.3f} 像素, 最大 {max_reproj:.3f} 像素"
                                    if avg_reproj > 1.0:
                                        reproj_info = f", ⚠️ 重投影误差较大: 平均 {avg_reproj:.3f} 像素, 最大 {max_reproj:.3f} 像素"
                            
                            motion_amplitudes.append({
                                'data_point': i+1,
                                'robot_rot_deg': angle_a_deg,
                                'robot_trans_mm': trans_a,
                                'error': errors[i] if i < len(errors) else 0.0
                            })
                            
                            if i < len(errors):
                                self.get_logger().info(f'     数据点 #{i+1}: 误差 {errors[i]:.3f} mm{reproj_info}')
                                self.get_logger().info(f'       与上一个数据点的机器人运动: 旋转 {angle_a_deg:.2f}°, 平移 {trans_a:.2f} mm')
                        
                        # 检查运动幅度是否足够（用于诊断）
                        if len(motion_amplitudes) > 0:
                            avg_robot_rot = np.mean([m['robot_rot_deg'] for m in motion_amplitudes])
                            avg_robot_trans = np.mean([m['robot_trans_mm'] for m in motion_amplitudes])
                            
                            self.get_logger().info(f'   运动幅度统计（相邻位姿之间）:')
                            self.get_logger().info(f'     平均机器人旋转: {avg_robot_rot:.2f}°')
                            self.get_logger().info(f'     平均机器人平移: {avg_robot_trans:.2f} mm')
                        
                        # 检查重投影误差（最关键的质量指标）
                        high_reproj_error_count = 0
                        max_reproj_in_all_poses = 0.0
                        avg_reproj_all_poses = 0.0
                        reproj_count = 0
                        
                        for pose_data in poses_for_diagnosis:
                            reproj_error = pose_data.get('board_pose', {}).get('reprojection_error')
                            if reproj_error is not None:
                                avg_reproj_all_poses += reproj_error
                                max_reproj_in_all_poses = max(max_reproj_in_all_poses, reproj_error)
                                reproj_count += 1
                                if reproj_error > 2.0:
                                    high_reproj_error_count += 1
                        
                        if reproj_count > 0:
                            avg_reproj_all_poses /= reproj_count
                            self.get_logger().info(f'   重投影误差统计:')
                            self.get_logger().info(f'     平均: {avg_reproj_all_poses:.3f} 像素')
                            self.get_logger().info(f'     最大: {max_reproj_in_all_poses:.3f} 像素')
                            if avg_reproj_all_poses > 1.0:
                                self.get_logger().warning(f'     ⚠️ 重投影误差较大，建议重新标定相机内参')
                        
                        self.get_logger().info('='*60)
                        
                        # 保存标定结果
                        self.hand_eye_calibration_data['transformation_matrix'] = T_camera2gripper.tolist()
                        self.hand_eye_calibration_data['calibration_error'] = {
                            'mean': float(mean_error),
                            'max': float(max_error),
                            'min': float(min_error),
                            'std': float(std_error),
                            'errors': [float(e) for e in errors]
                        }
                        
                        R_cam2gripper = T_camera2gripper[:3, :3]
                        t_cam2gripper = T_camera2gripper[:3, 3]
                        
                        # 保存标定数据到文件（用于验证）
                        try:
                            from datetime import datetime
                            
                            # 在根目录创建验证数据文件夹
                            validation_dir = os.path.join('/home/mu/IVG', 'hand_eye_calibration_validation')
                            os.makedirs(validation_dir, exist_ok=True)
                            
                            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                            
                            # 保存完整的标定数据
                            saved_calibration_data = {
                                'timestamp': timestamp,
                                'timestamp_readable': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                                'method': 'Custom XY+Z Constraint (参考hand_eye_calibration.py)',
                                'calibration_type': 'Eye-in-Hand',
                                'num_data_points': len(pick_poses),
                                
                                # 标定结果
                                'T_camera2gripper': T_camera2gripper.tolist(),
                                'R_cam2gripper': R_cam2gripper.tolist(),
                                't_cam2gripper': t_cam2gripper.tolist(),
                                
                                # 输入数据：shot_pose, pick_poses, camera_points_shot
                                'shot_pose': shot_pose.tolist(),
                                'pick_poses': [T.tolist() for T in pick_poses],
                                'camera_points_shot': [p.tolist() for p in camera_points_shot],
                                'fixed_z': float(fixed_z),
                                
                                # 误差统计（参考hand_eye_calibration.py）
                                'error_statistics': {
                                    'mean': float(mean_error),
                                    'max': float(max_error),
                                    'min': float(min_error),
                                    'std': float(std_error),
                                    'errors': [float(e) for e in errors]
                                },
                                
                                # 数据质量诊断
                                'motion_quality': motion_amplitudes
                            }
                            
                            # 保存原始姿态数据（用于验证）
                            for i in range(len(pick_poses)):
                                saved_calibration_data.setdefault('data_points', []).append({
                                    'index': i,
                                    'pick_pose': pick_poses[i].tolist(),
                                    'camera_point': camera_points_shot[i].tolist()
                                })
                            
                            # 保存到JSON文件
                            data_file = os.path.join(validation_dir, f'calibration_data_{timestamp}.json')
                            with open(data_file, 'w', encoding='utf-8') as f:
                                saved_calibration_data_clean = _convert_to_json_serializable(saved_calibration_data)
                                json.dump(saved_calibration_data_clean, f, indent=2, ensure_ascii=False)
                            
                            self.get_logger().info(f'✅ 标定数据已保存到: {data_file}')
                            self.get_logger().info(f'   可用于验证脚本: /home/mu/IVG/hand_eye_calibration_validation/verify_calibration.py')
                            
                        except Exception as e:
                            self.get_logger().warn(f'⚠️ 保存标定数据失败: {str(e)}')
                            import traceback
                            self.get_logger().warn(traceback.format_exc())
                        
                        # 构建响应数据，确保所有NumPy类型都转换为Python原生类型
                        response_data = {
                            'success': True,
                            'method': 'Custom XY+Z Constraint (参考hand_eye_calibration.py)',
                            'calibration_type': 'Eye-in-Hand',
                            'transformation_matrix': T_camera2gripper.tolist(),
                            'rotation_matrix': R_cam2gripper.tolist(),
                            'translation_vector': t_cam2gripper.tolist(),  # 单位：毫米
                            'fixed_z': float(fixed_z),  # 棋盘格桌面高度
                            'units': {
                                'translation': 'mm',
                                'rotation': 'rad',
                                'error': 'mm'
                            },
                            'evaluation': {
                                'mean_error': float(mean_error),
                                'max_error': float(max_error),
                                'min_error': float(min_error),
                                'std_error': float(std_error),
                                'data_count': len(pick_poses),
                                'errors_per_pose': [float(e) for e in errors]  # 每个数据点的误差
                            },
                            'message': f'Custom模式标定成功（XY+Z约束方法），平均误差: {mean_error:.3f} mm'
                        }
                        # 确保所有NumPy类型都被转换为Python原生类型
                        response_data_clean = _convert_to_json_serializable(response_data)
                        return jsonify(response_data_clean)
                        
                    except ValueError as e:
                        # 数据验证错误
                        error_msg = str(e)
                        self.get_logger().error(f'❌ [Custom模式] 数据验证失败: {error_msg}')
                        return jsonify({
                            'success': False,
                            'error': f'[Custom模式] {error_msg}',
                            'error_type': 'validation_error'
                        })
                    except Exception as e:
                        # 其他错误
                        self.get_logger().error(f'❌ [Custom模式] 标定失败: {str(e)}')
                        import traceback
                        self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                        return jsonify({
                            'success': False, 
                            'error': f'[Custom模式] 标定失败: {str(e)}',
                            'error_type': 'calibration_error'
                        })
                    
            except Exception as e:
                self.get_logger().error(f'❌ Eye-in-Hand姿态法标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({
                    'success': False, 
                    'error': f'标定流程失败: {str(e)}',
                    'error_type': 'system_error'
                })
        
        
        @self.app.route('/api/hand_eye/calibrate', methods=['POST'])
        def perform_hand_eye_calibration():
            """
            执行手眼标定 - 支持Eye-to-Hand和Eye-in-Hand两种方式
            
            请求格式 (JSON):
            {
                "calibration_type": "eye-in-hand" | "eye-to-hand",  // 标定类型
                "calibration_method": "pose-based" | "corner-based",  // 仅eye-in-hand有效
                "method": "custom" | "opencv",  // 仅pose-based有效，算法选择
                "motion_groups": [  // 运动组数据（必需，仅pose-based）
                    {
                        "pose1": {...},  // 机器人位姿1
                        "pose2": {...},  // 机器人位姿2
                        "board_pose1": {...},  // 标定板位姿1
                        "board_pose2": {...}   // 标定板位姿2
                    },
                    ...
                ],
                "pose_data": [...]  // 其他格式的数据（可选）
            }
            
            响应格式 (JSON):
            成功:
            {
                "success": true,
                "method": "OpenCV calibrateHandEye" | "Custom XY+Z Constraint",
                "calibration_type": "Eye-in-Hand",
                "transformation_matrix": [[...], ...],  // 4x4变换矩阵
                "rotation_matrix": [[...], ...],  // 3x3旋转矩阵
                "translation_vector": [x, y, z],  // 平移向量（单位：mm）
                "units": {
                    "translation": "mm",
                    "rotation": "rad",
                    "error": "mm"
                },
                "evaluation": {
                    "mean_error": float,
                    "max_error": float,
                    "min_error": float,
                    "std_error": float,
                    "data_count": int,
                    "errors_per_motion_group": [...],
                    ...
                },
                "message": "..."
            }
            
            失败:
            {
                "success": false,
                "error": "错误信息",
                "error_type": "validation_error" | "calibration_error" | "system_error"
            }
            """
            try:
                data = request.get_json()
                
                # 验证请求数据
                if data is None:
                    return jsonify({
                        'success': False,
                        'error': '请求数据为空，请提供JSON格式的数据',
                        'error_type': 'invalid_request'
                    })
                
                calibration_type = data.get('calibration_type', 'eye-to-hand')  # 默认Eye-to-Hand
                pose_data = data.get('pose_data', [])
                
                self.get_logger().info('='*60)
                self.get_logger().info(f'📡 收到手眼标定请求')
                self.get_logger().info(f'   标定类型: {calibration_type}')
                
                if calibration_type == 'eye-in-hand':
                    # Eye-in-Hand标定
                    calibration_method = data.get('calibration_method', 'pose-based')  # 默认姿态法
                    self.get_logger().info(f'   标定方法: {calibration_method}')
                    
                    if calibration_method == 'pose-based':
                        # 姿态法标定（使用独立模块）
                        method = data.get('method', 'custom')  # 'custom' 或 'opencv'
                        self.get_logger().info(f'   算法模式: {method}')
                        return _perform_eye_in_hand_pose_based_calibration(data)
                    else:
                        # 角点法标定（原有逻辑）
                        return _perform_eye_in_hand_calibration(data)
                else:
                    # Eye-to-Hand标定（原有逻辑）
                    return _perform_eye_to_hand_calibration(data)
                
            except KeyError as e:
                self.get_logger().error(f'❌ 请求数据缺少必需字段: {str(e)}')
                return jsonify({
                    'success': False,
                    'error': f'请求数据缺少必需字段: {str(e)}',
                    'error_type': 'missing_field'
                })
            except Exception as e:
                self.get_logger().error(f'❌ 手眼标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({
                    'success': False, 
                    'error': f'标定失败: {str(e)}',
                    'error_type': 'system_error'
                })
        
        def _perform_eye_to_hand_calibration(data):
            """执行Eye-to-Hand标定"""
            try:
                pose_data = data.get('pose_data', [])
                camera_height = data.get('camera_height', 840.0)  # 相机高度，默认840mm
                
                self.get_logger().info(f'🤖 开始Eye-to-Hand手眼标定')
                self.get_logger().info(f'   标定场景: 相机固定，机器人末端点选角点')
                self.get_logger().info(f'   相机高度: {camera_height} mm')
                self.get_logger().info(f'   数据组数: {len(pose_data)}')
                
                if len(pose_data) < 3:
                    return jsonify({
                        'success': False, 
                        'error': f'数据不足，至少需要3组位姿数据，当前只有{len(pose_data)}组'
                    })
                
                # 检查相机标定参数
                if self.calib_utils.camera_matrix is None or self.calib_utils.dist_coeffs is None:
                    return jsonify({
                        'success': False, 
                        'error': '请先加载相机标定参数'
                    })
                
                # Eye-to-Hand标定：点云配准方法
                # 准备两组对应的3D点集
                points_camera = []  # 角点在相机坐标系的3D坐标
                points_robot = []   # 机器人末端在机器人基座坐标系的3D坐标
                
                for idx, pose in enumerate(pose_data):
                    try:
                        # Eye-to-Hand场景：
                        # 机器人末端移动到角点位置
                        # 这个位置在机器人坐标系的坐标（已知）
                        p_robot = np.array([
                            pose['robot_pos_x'],  # mm
                            pose['robot_pos_y'],  # mm
                            pose['robot_pos_z']   # mm
                        ])
                        
                        # 同一个角点在相机坐标系的坐标
                        # 从图像检测得到XY，Z使用相机高度
                        p_camera = np.array([
                            pose['camera_x'],     # mm
                            pose['camera_y'],     # mm
                            camera_height         # mm（相机到标定板的高度）
                        ])
                        
                        points_robot.append(p_robot)
                        points_camera.append(p_camera)
                        
                        self.get_logger().info(f'   点对 #{idx+1}: 相机({p_camera[0]:.2f}, {p_camera[1]:.2f}, {p_camera[2]:.2f})mm → 机器人({p_robot[0]:.2f}, {p_robot[1]:.2f}, {p_robot[2]:.2f})mm')
                        
                    except Exception as e:
                        self.get_logger().warning(f'   数据点 #{idx+1} 处理失败: {str(e)}')
                        continue
                
                if len(points_robot) < 3:
                    return jsonify({
                        'success': False, 
                        'error': f'有效数据不足，至少需要3组，当前只有{len(points_robot)}组'
                    })
                
                # 转换为numpy数组
                points_camera = np.array(points_camera, dtype=np.float64)  # Nx3
                points_robot = np.array(points_robot, dtype=np.float64)    # Nx3
                
                # 数据质量分析
                self.get_logger().info(f'   数据质量分析...')
                self.get_logger().info(f'   相机坐标范围:')
                self.get_logger().info(f'     X: {points_camera[:, 0].min():.2f} ~ {points_camera[:, 0].max():.2f} mm (变化 {points_camera[:, 0].max() - points_camera[:, 0].min():.2f} mm)')
                self.get_logger().info(f'     Y: {points_camera[:, 1].min():.2f} ~ {points_camera[:, 1].max():.2f} mm (变化 {points_camera[:, 1].max() - points_camera[:, 1].min():.2f} mm)')
                self.get_logger().info(f'     Z: {points_camera[:, 2].min():.2f} ~ {points_camera[:, 2].max():.2f} mm (所有点Z={camera_height}mm)')
                self.get_logger().info(f'   机器人坐标范围:')
                self.get_logger().info(f'     X: {points_robot[:, 0].min():.2f} ~ {points_robot[:, 0].max():.2f} mm (变化 {points_robot[:, 0].max() - points_robot[:, 0].min():.2f} mm)')
                self.get_logger().info(f'     Y: {points_robot[:, 1].min():.2f} ~ {points_robot[:, 1].max():.2f} mm (变化 {points_robot[:, 1].max() - points_robot[:, 1].min():.2f} mm)')
                self.get_logger().info(f'     Z: {points_robot[:, 2].min():.2f} ~ {points_robot[:, 2].max():.2f} mm (变化 {points_robot[:, 2].max() - points_robot[:, 2].min():.2f} mm)')
                
                # 计算数据分布质量
                cam_range_x = points_camera[:, 0].max() - points_camera[:, 0].min()
                cam_range_y = points_camera[:, 1].max() - points_camera[:, 1].min()
                robot_range_x = points_robot[:, 0].max() - points_robot[:, 0].min()
                robot_range_y = points_robot[:, 1].max() - points_robot[:, 1].min()
                robot_range_z = points_robot[:, 2].max() - points_robot[:, 2].min()
                
                # 数据分布警告
                if cam_range_x < 50 or cam_range_y < 50:
                    self.get_logger().warning(f'   ⚠️  相机视野覆盖范围较小，建议数据点分布更广')
                if robot_range_x < 50 or robot_range_y < 50:
                    self.get_logger().warning(f'   ⚠️  机器人移动范围较小，建议增加XY方向的移动距离')
                
                # Eye-to-Hand标定：使用点云配准求解刚体变换
                # 求解：P_robot = R × P_camera + t
                self.get_logger().info(f'   开始Eye-to-Hand标定计算（点云配准方法）...')
                self.get_logger().info(f'   使用 {len(points_robot)} 组点对')
                
                try:
                    # 使用SVD方法求解刚体变换（相机→机器人）
                    R_cam2robot, t_cam2robot = self._solve_rigid_transform(points_camera, points_robot)
                    
                    # 构建4x4变换矩阵
                    T_cam2robot = np.eye(4)
                    T_cam2robot[:3, :3] = R_cam2robot
                    T_cam2robot[:3, 3] = t_cam2robot.flatten()
                    
                    self.get_logger().info(f'✅ 刚体变换求解完成')
                    self.get_logger().info(f'   旋转矩阵行列式: {np.linalg.det(R_cam2robot):.6f} (应该≈1.0)')
                    
                    # 计算配准误差（点对点距离）
                    errors = []
                    for i in range(len(points_camera)):
                        # 使用变换矩阵将相机坐标转换到机器人坐标
                        p_cam_homogeneous = np.append(points_camera[i], 1.0)  # 齐次坐标
                        p_robot_predicted = (T_cam2robot @ p_cam_homogeneous)[:3]
                        
                        # 实际的机器人坐标
                        p_robot_actual = points_robot[i]
                        
                        # 计算误差（欧氏距离）
                        error = np.linalg.norm(p_robot_predicted - p_robot_actual)
                        errors.append(error)  # 单位：mm
                    
                    mean_error = np.mean(errors)
                    max_error = np.max(errors)
                    min_error = np.min(errors)
                    std_error = np.std(errors)
                    
                    self.get_logger().info('✅ Eye-to-Hand标定完成')
                    self.get_logger().info(f'   平均误差: {mean_error:.3f} mm')
                    self.get_logger().info(f'   最大误差: {max_error:.3f} mm')
                    self.get_logger().info(f'   最小误差: {min_error:.3f} mm')
                    self.get_logger().info(f'   标准差: {std_error:.3f} mm')
                    self.get_logger().info('='*60)
                    
                    # 保存标定结果
                    self.hand_eye_calibration_data['transformation_matrix'] = T_cam2robot.tolist()
                    self.hand_eye_calibration_data['calibration_error'] = {
                        'mean': float(mean_error),
                        'max': float(max_error),
                        'min': float(min_error),
                        'std': float(std_error),
                        'errors': [float(e) for e in errors]
                    }
                    
                    return jsonify({
                        'success': True,
                        'method': 'SVD Point Cloud Registration',  # 使用的标定方法
                        'calibration_type': 'Eye-to-Hand',  # 标定类型
                        'camera_height': float(camera_height),  # 使用的相机高度设置
                        'transformation_matrix': T_cam2robot.tolist(),  # 4x4矩阵，相机→机器人基座
                        'rotation_matrix': R_cam2robot.tolist(),  # 3x3旋转矩阵
                        'translation_vector': t_cam2robot.flatten().tolist(),  # 平移向量，单位：mm
                        'units': {
                            'translation': 'mm',  # 平移单位：毫米
                            'rotation': 'rad',    # 旋转单位：弧度
                            'error': 'mm'         # 误差单位：毫米
                        },
                        'data_quality': {
                            'position_range_camera': {
                                'x': float(cam_range_x),
                                'y': float(cam_range_y)
                            },
                            'position_range_robot': {
                                'x': float(robot_range_x),
                                'y': float(robot_range_y),
                                'z': float(robot_range_z)
                            }
                        },
                        'evaluation': {
                            'mean_error': float(mean_error),      # 单位：mm
                            'max_error': float(max_error),        # 单位：mm
                            'min_error': float(min_error),        # 单位：mm
                            'std_error': float(std_error),        # 单位：mm
                            'data_count': len(points_robot),
                            'errors_per_pose': [float(e) for e in errors]  # 单位：mm
                        },
                        'message': f'Eye-to-Hand标定成功，平均误差: {mean_error:.3f} mm'
                    })
                    
                except Exception as e:
                    self.get_logger().error(f'❌ 标定计算失败: {str(e)}')
                    import traceback
                    self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                    return jsonify({
                        'success': False, 
                        'error': f'标定计算失败: {str(e)}'
                    })
                    
            except Exception as e:
                self.get_logger().error(f'❌ Eye-to-Hand标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        def _perform_eye_in_hand_calibration(data):
            """执行Eye-in-Hand标定"""
            try:
                pose_data = data.get('pose_data', [])  # 点选姿态数据
                shot_pose_data = data.get('shot_pose', None)  # 拍照姿态数据
                
                self.get_logger().info(f'🤖 开始Eye-in-Hand手眼标定')
                self.get_logger().info(f'   标定场景: 相机安装在机器人末端')
                self.get_logger().info(f'   点选姿态组数: {len(pose_data)}')
                
                if len(pose_data) < 3:
                    return jsonify({
                        'success': False, 
                        'error': f'数据不足，至少需要3组点选姿态数据，当前只有{len(pose_data)}组'
                    })
                
                if shot_pose_data is None:
                    return jsonify({
                        'success': False,
                        'error': '缺少拍照姿态数据，请先提取角点以保存拍照姿态'
                    })
                
                # 检查相机标定参数
                if self.calib_utils.camera_matrix is None or self.calib_utils.dist_coeffs is None:
                    return jsonify({
                        'success': False, 
                        'error': '请先加载相机标定参数'
                    })
                
                # 构建拍照姿态的变换矩阵（末端执行器到基座）
                shot_pose = self._pose_to_transform_matrix(
                    {'x': shot_pose_data['robot_pos_x'], 'y': shot_pose_data['robot_pos_y'], 'z': shot_pose_data['robot_pos_z']},
                    {'x': shot_pose_data['robot_ori_x'], 'y': shot_pose_data['robot_ori_y'], 'z': shot_pose_data['robot_ori_z'], 'w': shot_pose_data['robot_ori_w']}
                )
                
                # 构建点选姿态的变换矩阵列表
                pick_poses = []
                camera_points_shot = []  # 拍照时角点在相机坐标系下的坐标
                
                # 从拍照姿态数据中获取角点坐标（需要找到对应的角点）
                shot_corners = {corner['index']: {
                    'camera_x': corner['camera_x'], 
                    'camera_y': corner['camera_y'],
                    'camera_z': corner.get('camera_z', 550.0)  # 使用拍照时的Z坐标
                } for corner in shot_pose_data.get('corners', [])}
                
                for idx, pose in enumerate(pose_data):
                    try:
                        corner_index = pose.get('corner_index')
                        
                        # 查找拍照时对应的角点坐标
                        if corner_index is None or corner_index not in shot_corners:
                            self.get_logger().warning(f'   数据点 #{idx+1} 缺少对应的拍照角点数据（角点索引: {corner_index}），已跳过')
                            continue
                        
                        shot_corner = shot_corners[corner_index]
                        
                        # 构建点选姿态的变换矩阵
                        pick_pose = self._pose_to_transform_matrix(
                            {'x': pose['robot_pos_x'], 'y': pose['robot_pos_y'], 'z': pose['robot_pos_z']},
                            {'x': pose['robot_ori_x'], 'y': pose['robot_ori_y'], 'z': pose['robot_ori_z'], 'w': pose['robot_ori_w']}
                        )
                        pick_poses.append(pick_pose)
                        
                        # 拍照时角点在相机坐标系下的坐标（使用拍照时的camera_z）
                        camera_points_shot.append(np.array([
                            shot_corner['camera_x'],
                            shot_corner['camera_y'],
                            shot_corner['camera_z']
                        ]))
                        
                        self.get_logger().info(f'   点对 #{len(pick_poses)}: 角点#{corner_index}, 相机({shot_corner["camera_x"]:.2f}, {shot_corner["camera_y"]:.2f}, {shot_corner["camera_z"]:.2f})mm')
                        
                    except Exception as e:
                        self.get_logger().warning(f'   数据点 #{idx+1} 处理失败: {str(e)}')
                        continue
                
                if len(pick_poses) < 3:
                    return jsonify({
                        'success': False, 
                        'error': f'有效数据不足，至少需要3组，当前只有{len(pick_poses)}组'
                    })
                
                # 执行Eye-in-Hand标定
                self.get_logger().info(f'   开始Eye-in-Hand标定计算（优化方法）...')
                self.get_logger().info(f'   使用 {len(pick_poses)} 组点对')
                
                try:
                    T_camera2gripper, fixed_z = _solve_eye_in_hand(shot_pose, pick_poses, camera_points_shot)
                    
                    # 计算误差
                    errors = []
                    for i in range(len(pick_poses)):
                        P_camera_shot = camera_points_shot[i]
                        T_gripper_pick2base = pick_poses[i]
                        
                        # 计算角点在基座坐标系下的位置
                        P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
                        P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
                        P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
                        P_base_computed = P_base_computed_homogeneous[:3]
                        
                        # 实际位置（点选姿态的末端执行器位置）
                        P_base_actual = T_gripper_pick2base[:3, 3]
                        
                        # 计算误差
                        error = np.linalg.norm(P_base_computed - P_base_actual)
                        errors.append(error)
                    
                    mean_error = np.mean(errors)
                    max_error = np.max(errors)
                    min_error = np.min(errors)
                    std_error = np.std(errors)
                    
                    self.get_logger().info('✅ Eye-in-Hand标定完成')
                    self.get_logger().info(f'   平均误差: {mean_error:.3f} mm')
                    self.get_logger().info(f'   最大误差: {max_error:.3f} mm')
                    self.get_logger().info(f'   最小误差: {min_error:.3f} mm')
                    self.get_logger().info(f'   标准差: {std_error:.3f} mm')
                    self.get_logger().info('='*60)
                    
                    # 保存标定结果
                    self.hand_eye_calibration_data['transformation_matrix'] = T_camera2gripper.tolist()
                    self.hand_eye_calibration_data['calibration_error'] = {
                        'mean': float(mean_error),
                        'max': float(max_error),
                        'min': float(min_error),
                        'std': float(std_error),
                        'errors': [float(e) for e in errors]
                    }
                    
                    R_cam2gripper = T_camera2gripper[:3, :3]
                    t_cam2gripper = T_camera2gripper[:3, 3]
                    
                    return jsonify({
                        'success': True,
                        'method': 'Optimization with XY+Z Constraint',
                        'calibration_type': 'Eye-in-Hand',
                        'transformation_matrix': T_camera2gripper.tolist(),  # 4x4矩阵，相机→末端执行器
                        'rotation_matrix': R_cam2gripper.tolist(),  # 3x3旋转矩阵
                        'translation_vector': t_cam2gripper.tolist(),  # 平移向量，单位：mm
                        'units': {
                            'translation': 'mm',
                            'rotation': 'rad',
                            'error': 'mm'
                        },
                        'evaluation': {
                            'mean_error': float(mean_error),
                            'max_error': float(max_error),
                            'min_error': float(min_error),
                            'std_error': float(std_error),
                            'data_count': len(pick_poses),
                            'errors_per_pose': [float(e) for e in errors]
                        },
                        'message': f'Eye-in-Hand标定成功，平均误差: {mean_error:.3f} mm'
                    })
                    
                except Exception as e:
                    self.get_logger().error(f'❌ Eye-in-Hand标定计算失败: {str(e)}')
                    import traceback
                    self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                    return jsonify({
                        'success': False, 
                        'error': f'标定计算失败: {str(e)}'
                    })
                    
            except Exception as e:
                self.get_logger().error(f'❌ Eye-in-Hand标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        def _pose_to_transform_matrix(position, orientation_quat):
            """将位置和四元数组合成4x4齐次变换矩阵"""
            T = np.eye(4)
            T[:3, :3] = self._quaternion_to_rotation_matrix([
                orientation_quat['x'],
                orientation_quat['y'],
                orientation_quat['z'],
                orientation_quat['w']
            ])
            T[:3, 3] = [position['x'], position['y'], position['z']]
            return T
        
        def _solve_eye_in_hand(shot_pose, pick_poses, camera_points_shot, fixed_z=None):
            """
            Eye-in-Hand手眼标定核心算法
            基于hand_eye_calibration.py中的实现
            """
            n = len(pick_poses)
            
            # 辅助函数：兼容不同版本的scipy Rotation.as_matrix()
            def rotation_to_matrix(r):
                """将Rotation对象转换为矩阵（兼容不同版本scipy）"""
                if hasattr(r, 'as_matrix'):
                    return r.as_matrix()
                elif hasattr(r, 'as_dcm'):
                    return r.as_dcm()
                else:
                    # 手动转换（使用Rodrigues公式）
                    rotvec = r.as_rotvec()
                    angle = np.linalg.norm(rotvec)
                    if angle < 1e-6:
                        return np.eye(3)
                    axis = rotvec / angle
                    K = np.array([[0, -axis[2], axis[1]],
                                  [axis[2], 0, -axis[0]],
                                  [-axis[1], axis[0], 0]])
                    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
            
            # 计算Z值（棋盘格桌面高度）
            pick_z_values = [T_pick[2, 3] for T_pick in pick_poses]
            if fixed_z is None:
                fixed_z = np.mean(pick_z_values)
                self.get_logger().info(f'   平均Z值（棋盘格桌面高度）: {fixed_z:.6f} mm')
            
            # 优化目标函数
            def residuals(params):
                """计算残差"""
                axis_angle = params[:3]
                translation = params[3:6]
                
                # 构建旋转矩阵
                angle = np.linalg.norm(axis_angle)
                if angle < 1e-6:
                    R_mat = np.eye(3)
                else:
                    axis = axis_angle / angle
                    r = R.from_rotvec(axis * angle)
                    R_mat = rotation_to_matrix(r)
                
                # 构建变换矩阵 T_camera2gripper
                T_camera2gripper = np.eye(4)
                T_camera2gripper[:3, :3] = R_mat
                T_camera2gripper[:3, 3] = translation
                
                residuals_list = []
                
                for i in range(n):
                    T_gripper_pick2base = pick_poses[i]
                    P_camera_shot = camera_points_shot[i]
                    
                    # 从拍照姿态和手眼标定矩阵计算角点在基座坐标系下的位置
                    P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
                    P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
                    P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
                    P_base_computed = P_base_computed_homogeneous[:3]
                    
                    # 从点选姿态得到角点在基座坐标系下的实际位置
                    P_base_actual = T_gripper_pick2base[:3, 3]
                    
                    # XY+Z约束
                    residual = P_base_computed - P_base_actual
                    residuals_list.extend(residual)
                
                return np.array(residuals_list)
            
            # 初始估计：使用SVD方法
            T_gripper_shot2base_inv = np.linalg.inv(shot_pose)
            
            points_gripper = []
            points_camera = []
            
            for i in range(n):
                T_gripper_pick2base = pick_poses[i]
                P_camera_shot = camera_points_shot[i]
                
                # 从点选姿态得到角点在基座坐标系的位置
                P_base = T_gripper_pick2base[:3, 3]
                
                # 转换到末端执行器坐标系（拍照时的末端执行器坐标系）
                P_base_homogeneous = np.append(P_base, 1.0)
                P_gripper_homogeneous = T_gripper_shot2base_inv @ P_base_homogeneous
                P_gripper = P_gripper_homogeneous[:3]
                
                points_gripper.append(P_gripper)
                points_camera.append(P_camera_shot)
            
            points_gripper = np.array(points_gripper)
            points_camera = np.array(points_camera)
            
            # SVD方法求解初始估计
            centroid_gripper = np.mean(points_gripper, axis=0)
            centroid_camera = np.mean(points_camera, axis=0)
            
            points_gripper_centered = points_gripper - centroid_gripper
            points_camera_centered = points_camera - centroid_camera
            
            H = points_camera_centered.T @ points_gripper_centered
            
            U, S, Vt = np.linalg.svd(H)
            
            R_init = Vt.T @ U.T
            
            if np.linalg.det(R_init) < 0:
                Vt[-1, :] *= -1
                R_init = Vt.T @ U.T
            
            t_init = centroid_gripper - R_init @ centroid_camera
            
            # 使用安全的转换函数
            quat_init = self._rotation_matrix_to_quaternion(R_init)
            # 四元数转轴角
            from scipy.spatial.transform import Rotation as R_scipy
            try:
                if hasattr(R_scipy, 'from_quat'):
                    r_init = R_scipy.from_quat(quat_init)
                    axis_angle_init = r_init.as_rotvec()
                else:
                    # 手动转换四元数到轴角
                    w = quat_init[3]
                    if abs(w) >= 1.0:
                        axis_angle_init = np.array([0.0, 0.0, 0.0])
                    else:
                        angle = 2 * np.arccos(abs(w))
                        if w < 0:
                            axis = -quat_init[:3] / np.sin(angle / 2)
                        else:
                            axis = quat_init[:3] / np.sin(angle / 2)
                        axis_angle_init = axis * angle
            except Exception as e:
                self.get_logger().error(f'四元数转轴角失败: {e}')
                # 使用Rodrigues公式的逆变换
                import cv2
                axis_angle_init, _ = cv2.Rodrigues(R_init)
                axis_angle_init = axis_angle_init.flatten()
            
            initial_params = np.concatenate([axis_angle_init, t_init])
            
            self.get_logger().info(f'   初始估计完成，开始优化求解...')
            
            # 优化求解
            result = least_squares(residuals, initial_params, method='lm',
                                  max_nfev=10000, ftol=1e-8, xtol=1e-8, verbose=0)
            
            # 提取结果
            axis_angle = result.x[:3]
            translation = result.x[3:6]
            
            # 构建旋转矩阵
            angle = np.linalg.norm(axis_angle)
            if angle < 1e-6:
                R_camera2gripper = np.eye(3)
            else:
                axis = axis_angle / angle
                r = R.from_rotvec(axis * angle)
                R_camera2gripper = rotation_to_matrix(r)
            
            # 构建变换矩阵
            T_camera2gripper = np.eye(4)
            T_camera2gripper[:3, :3] = R_camera2gripper
            T_camera2gripper[:3, 3] = translation
            
            mean_residual = np.mean(np.abs(result.fun))
            self.get_logger().info(f'   优化完成，平均残差: {mean_residual:.6f} mm')
            
            return T_camera2gripper, fixed_z
        
        # _convert_to_json_serializable 已从 opencv_hand_eye_calibration 模块导入，不再需要本地定义
        
        def _prepare_opencv_data(motion_groups):
            """
            OpenCV模式：准备数据用于calibrateHandEye
            从motion_groups中提取唯一姿态，并转换为OpenCV需要的格式
            
            参数:
                motion_groups: 运动组列表，每个运动组包含pose1, pose2, board_pose1, board_pose2
            
            返回:
                R_gripper2base: (N, 3, 3) numpy数组，机器人旋转矩阵列表
                t_gripper2base: 列表，每个元素(3,1)数组，机器人平移向量（单位：米）
                rvecs: 列表，每个元素(3,1)数组，标定板旋转向量（单位：弧度）
                tvecs: 列表，每个元素(3,1)数组，标定板平移向量（单位：米）
                T_gripper_list: 列表，机器人变换矩阵（单位：毫米，用于误差计算）
                T_board_list: 列表，标定板变换矩阵（单位：毫米，用于误差计算）
            """
            import json
            import time
            
            self.get_logger().info(f'   [OpenCV模式] 开始数据准备：{len(motion_groups)} 个运动组')
            
            # 提取所有唯一姿态数据用于OpenCV（去重处理）
            # 因为运动组可能是相邻姿态对（0-1, 1-2, 2-3, ...），会有重复姿态
            # 需要保持顺序：按照运动组的顺序依次提取，但只保留唯一姿态
            T_gripper_list = []  # 保持顺序的唯一姿态列表（单位：毫米）
            T_board_list = []    # 保持顺序的唯一标定板姿态列表（单位：毫米）
            board_rvec_tvec_map = {}  # 保存每个T_board对应的原始rvec/tvec数据
            seen_poses = set()   # 用于快速检查是否已存在
            
            # 姿态去重精度：0.1mm（可配置）
            pose_key_epsilon = 0.1  # mm
            
            def pose_key(T):
                """生成姿态的唯一标识（用于去重）"""
                # 使用变换矩阵的扁平化数组，四舍五入到0.1mm精度
                # 如果两个姿态的差异<0.1mm，认为是重复姿态
                return tuple(np.round(T.flatten() / pose_key_epsilon) * pose_key_epsilon)
            
            # 统计信息
            processed_groups = 0
            skipped_groups = 0
            duplicate_poses = 0
            
            for idx, motion in enumerate(motion_groups):
                try:
                    pose1_data = motion.get('pose1')
                    pose2_data = motion.get('pose2')
                    board_pose1_data = motion.get('board_pose1')
                    board_pose2_data = motion.get('board_pose2')
                    
                    if not all([pose1_data, pose2_data, board_pose1_data, board_pose2_data]):
                        skipped_groups += 1
                        self.get_logger().warning(f'   [OpenCV模式] 运动组 #{idx+1} 数据不完整，已跳过')
                        continue
                    
                    log_entry = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-full-pipeline',
                        'hypothesisId': 'FULL1',
                        'location': 'hand_eye_calibration_node.py:_prepare_opencv_data',
                        'message': f'[数据采集] 运动组#{idx+1}原始数据',
                        'data': {
                            'motion_group_idx': idx + 1,
                            'robot_pose1_original_m': {
                                'x': pose1_data['robot_pos_x'],
                                'y': pose1_data['robot_pos_y'],
                                'z': pose1_data['robot_pos_z'],
                                'qx': pose1_data['robot_ori_x'],
                                'qy': pose1_data['robot_ori_y'],
                                'qz': pose1_data['robot_ori_z'],
                                'qw': pose1_data['robot_ori_w']
                            },
                            'robot_pose2_original_m': {
                                'x': pose2_data['robot_pos_x'],
                                'y': pose2_data['robot_pos_y'],
                                'z': pose2_data['robot_pos_z'],
                                'qx': pose2_data['robot_ori_x'],
                                'qy': pose2_data['robot_ori_y'],
                                'qz': pose2_data['robot_ori_z'],
                                'qw': pose2_data['robot_ori_w']
                            },
                            'board_pose1_original_mm': {
                                'x': board_pose1_data['position']['x'],
                                'y': board_pose1_data['position']['y'],
                                'z': board_pose1_data['position']['z'],
                                'qx': board_pose1_data['orientation']['x'],
                                'qy': board_pose1_data['orientation']['y'],
                                'qz': board_pose1_data['orientation']['z'],
                                'qw': board_pose1_data['orientation']['w'],
                                'has_rvec': 'rvec' in board_pose1_data,
                                'has_tvec': 'tvec' in board_pose1_data
                            },
                            'board_pose2_original_mm': {
                                'x': board_pose2_data['position']['x'],
                                'y': board_pose2_data['position']['y'],
                                'z': board_pose2_data['position']['z'],
                                'qx': board_pose2_data['orientation']['x'],
                                'qy': board_pose2_data['orientation']['y'],
                                'qz': board_pose2_data['orientation']['z'],
                                'qw': board_pose2_data['orientation']['w'],
                                'has_rvec': 'rvec' in board_pose2_data,
                                'has_tvec': 'tvec' in board_pose2_data
                            }
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    # 记录原始rvec/tvec（如果存在）
                    if 'rvec' in board_pose1_data and 'tvec' in board_pose1_data:
                        log_entry['data']['board_pose1_rvec'] = board_pose1_data['rvec']
                        log_entry['data']['board_pose1_tvec'] = board_pose1_data['tvec']
                    if 'rvec' in board_pose2_data and 'tvec' in board_pose2_data:
                        log_entry['data']['board_pose2_rvec'] = board_pose2_data['rvec']
                        log_entry['data']['board_pose2_tvec'] = board_pose2_data['tvec']
                    
                    # 构建变换矩阵（将机器人位姿从米转换为毫米）
                    T_gripper1 = _pose_to_transform_matrix(
                        {'x': pose1_data['robot_pos_x'] * 1000.0, 'y': pose1_data['robot_pos_y'] * 1000.0, 'z': pose1_data['robot_pos_z'] * 1000.0},
                        {'x': pose1_data['robot_ori_x'], 'y': pose1_data['robot_ori_y'], 'z': pose1_data['robot_ori_z'], 'w': pose1_data['robot_ori_w']}
                    )
                    T_gripper2 = _pose_to_transform_matrix(
                        {'x': pose2_data['robot_pos_x'] * 1000.0, 'y': pose2_data['robot_pos_y'] * 1000.0, 'z': pose2_data['robot_pos_z'] * 1000.0},
                        {'x': pose2_data['robot_ori_x'], 'y': pose2_data['robot_ori_y'], 'z': pose2_data['robot_ori_z'], 'w': pose2_data['robot_ori_w']}
                    )
                    
                    # 标定板位姿（已经是毫米单位）
                    T_board1 = _pose_to_transform_matrix(
                        {'x': board_pose1_data['position']['x'], 'y': board_pose1_data['position']['y'], 'z': board_pose1_data['position']['z']},
                        {'x': board_pose1_data['orientation']['x'], 'y': board_pose1_data['orientation']['y'], 
                         'z': board_pose1_data['orientation']['z'], 'w': board_pose1_data['orientation']['w']}
                    )
                    T_board2 = _pose_to_transform_matrix(
                        {'x': board_pose2_data['position']['x'], 'y': board_pose2_data['position']['y'], 'z': board_pose2_data['position']['z']},
                        {'x': board_pose2_data['orientation']['x'], 'y': board_pose2_data['orientation']['y'], 
                         'z': board_pose2_data['orientation']['z'], 'w': board_pose2_data['orientation']['w']}
                    )
                    
                    # 验证变换矩阵的有效性
                    if idx == 0:  # 只记录第一个运动组
                        det_R_gripper1 = float(np.linalg.det(T_gripper1[:3, :3]))
                        det_R_board1 = float(np.linalg.det(T_board1[:3, :3]))
                        robot_pos_norm = float(np.linalg.norm(T_gripper1[:3, 3]))
                        board_pos_norm = float(np.linalg.norm(T_board1[:3, 3]))
                        board_z1 = float(T_board1[2, 3])  # Z方向值（mm）
                        
                        if abs(det_R_gripper1 - 1.0) > 0.01:
                            self.get_logger().warning(f'   [OpenCV模式] 机器人旋转矩阵行列式异常: {det_R_gripper1:.6f}（应为1.0）')
                        if abs(det_R_board1 - 1.0) > 0.01:
                            self.get_logger().warning(f'   [OpenCV模式] 标定板旋转矩阵行列式异常: {det_R_board1:.6f}（应为1.0）')
                        
                        # 检查标定板Z值是否异常
                        if board_z1 > 1500.0:  # 如果Z>1.5米（1500mm），异常
                            self.get_logger().error(f'   [OpenCV模式] ❌ 严重问题：第一个运动组的标定板Z方向异常大 {board_z1:.1f}mm！')
                            self.get_logger().error(f'      正常情况下，标定板到相机的距离应该在300-800mm范围内')
                            self.get_logger().error(f'      这可能表明：solvePnP的单位有问题，或者标定板离相机过远')
                        elif board_z1 > 1000.0:  # 1-1.5米，警告
                            self.get_logger().warning(f'   [OpenCV模式] ⚠️ 第一个运动组的标定板Z方向较大 {board_z1:.1f}mm，建议检查')
                        elif board_z1 < 100.0:  # 如果Z<100mm，也可能有问题
                            self.get_logger().warning(f'   [OpenCV模式] ⚠️ 第一个运动组的标定板Z方向过小 {board_z1:.1f}mm，可能检测不准确')
                    
                    # 保存姿态（基于机器人位姿+标定板位姿的组合去重，确保数据匹配）
                    # 重要：OpenCV calibrateHandEye需要机器人位姿和标定板位姿一一对应
                    # 如果只基于机器人位姿去重，会导致数据不匹配
                    key1_gripper = pose_key(T_gripper1)
                    key1_board = pose_key(T_board1)
                    key1_combined = (key1_gripper, key1_board)  # 组合键
                    
                    key2_gripper = pose_key(T_gripper2)
                    key2_board = pose_key(T_board2)
                    key2_combined = (key2_gripper, key2_board)  # 组合键
                    
                    added_pose1 = False
                    added_pose2 = False
                    
                    # 检查组合键（机器人位姿+标定板位姿）
                    if key1_combined not in seen_poses:
                        seen_poses.add(key1_combined)
                        pose_idx_1 = len(T_gripper_list)  # 记录添加的索引
                        T_gripper_list.append(T_gripper1)
                        T_board_list.append(T_board1)
                        # 保存对应的原始board_pose数据，用于后续提取rvec/tvec
                        board_key1 = pose_key(T_board1)  # 使用pose_key生成一致的键
                        if 'rvec' in board_pose1_data and 'tvec' in board_pose1_data:
                            board_rvec_tvec_map[board_key1] = {
                                'rvec': board_pose1_data['rvec'],
                                'tvec': board_pose1_data['tvec']
                            }
                        added_pose1 = True
                    else:
                        duplicate_poses += 1
                        self.get_logger().debug(f'   [OpenCV模式] 运动组 #{idx+1} 位姿1重复（机器人+标定板组合）')
                    
                    if key2_combined not in seen_poses:
                        seen_poses.add(key2_combined)
                        pose_idx_2 = len(T_gripper_list)  # 记录添加的索引
                        T_gripper_list.append(T_gripper2)
                        T_board_list.append(T_board2)
                        # 保存对应的原始board_pose数据，用于后续提取rvec/tvec
                        board_key2 = pose_key(T_board2)  # 使用pose_key生成一致的键
                        if 'rvec' in board_pose2_data and 'tvec' in board_pose2_data:
                            board_rvec_tvec_map[board_key2] = {
                                'rvec': board_pose2_data['rvec'],
                                'tvec': board_pose2_data['tvec']
                            }
                        added_pose2 = True
                    else:
                        duplicate_poses += 1
                        self.get_logger().debug(f'   [OpenCV模式] 运动组 #{idx+1} 位姿2重复（机器人+标定板组合）')
                    
                    processed_groups += 1
                    
                except Exception as e:
                    skipped_groups += 1
                    self.get_logger().warning(f'   [OpenCV模式] 运动组 #{idx+1} 处理失败: {str(e)}')
                    continue
            
            self.get_logger().info(f'   [OpenCV模式] 数据准备完成：处理{processed_groups}组，跳过{skipped_groups}组，去重后{len(T_gripper_list)}个唯一姿态对（去除{duplicate_poses}个重复姿态对）')
            self.get_logger().info(f'   [OpenCV模式] 去重策略：基于（机器人位姿+标定板位姿）组合，确保数据匹配')
            
            if len(T_gripper_list) < 4:
                raise ValueError(f'有效姿态数据不足，至少需要4个姿态，当前只有{len(T_gripper_list)}个')
            
            # 保存原始列表长度（用于日志）
            T_gripper_list_orig_count = len(T_gripper_list)
            
            # 数据质量过滤：过滤掉运动幅度过小的姿态对
            # 运动幅度过小的数据会导致OpenCV算法不稳定，返回异常大的Z值
            # 阈值：机器人平移 >= 50mm 且 机器人旋转 >= 5°
            # 注意：这些阈值比之前的更严格（从10mm/1°提高到50mm/5°），以确保数据质量足够高
            MIN_ROBOT_TRANSLATION_MM = 50.0  # 最小机器人平移（毫米）
            MIN_ROBOT_ROTATION_DEG = 5.0     # 最小机器人旋转（度）
            
            # 迭代过滤：持续过滤直到所有剩余姿态对都满足阈值要求
            T_gripper_filtered = T_gripper_list.copy()
            T_board_filtered = T_board_list.copy()
            filtered_pairs_count = 0
            iteration = 0
            max_iterations = 10  # 防止无限循环
            
            while iteration < max_iterations:
                iteration += 1
                pose_keep_mask = [True] * len(T_gripper_filtered)
                iteration_filtered_count = 0
                
                # 检查当前过滤后列表中的相邻姿态对
                for i in range(len(T_gripper_filtered) - 1):
                    T_gripper1 = T_gripper_filtered[i]
                    T_gripper2 = T_gripper_filtered[i + 1]
                    
                    # 计算机器人运动：A = inv(T_gripper1) @ T_gripper2
                    T_gripper1_inv = np.linalg.inv(T_gripper1)
                    A = T_gripper1_inv @ T_gripper2
                    
                    robot_translation = float(np.linalg.norm(A[:3, 3]))
                    robot_rotation = float(np.arccos(np.clip((np.trace(A[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    
                    # 检查运动幅度是否满足要求
                    if robot_translation < MIN_ROBOT_TRANSLATION_MM or robot_rotation < MIN_ROBOT_ROTATION_DEG:
                        iteration_filtered_count += 1
                        filtered_pairs_count += 1
                        # 过滤掉第二个姿态（这样不会影响后续的姿态对计算）
                        pose_keep_mask[i + 1] = False
                        self.get_logger().warning(f'   [OpenCV模式] ⚠️ 迭代#{iteration}：姿态对#{i+1}-#{i+2}运动幅度过小，已过滤第二个姿态：平移={robot_translation:.2f}mm, 旋转={robot_rotation:.2f}°')
                
                # 如果没有过滤任何姿态对，说明所有姿态对都满足要求，可以退出循环
                if iteration_filtered_count == 0:
                    break
                
                # 应用过滤：只保留标记为True的姿态
                T_gripper_filtered = [T for i, T in enumerate(T_gripper_filtered) if pose_keep_mask[i]]
                T_board_filtered = [T for i, T in enumerate(T_board_filtered) if pose_keep_mask[i]]
                
                # 如果过滤后姿态数量不足，停止过滤
                if len(T_gripper_filtered) < 4:
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 迭代#{iteration}后姿态数量不足（{len(T_gripper_filtered)}个），停止过滤')
                    break
            
            if iteration >= max_iterations:
                self.get_logger().warning(f'   [OpenCV模式] ⚠️ 达到最大迭代次数（{max_iterations}），停止过滤')
            
            filtered_poses_count = T_gripper_list_orig_count - len(T_gripper_filtered)
            
            # 验证过滤后的数据：检查剩余姿态之间的运动幅度
            filtered_motion_amplitudes = []
            if len(T_gripper_filtered) > 1:
                for i in range(len(T_gripper_filtered) - 1):
                    T_gripper1_f = T_gripper_filtered[i]
                    T_gripper2_f = T_gripper_filtered[i + 1]
                    T_gripper1_inv_f = np.linalg.inv(T_gripper1_f)
                    A_f = T_gripper1_inv_f @ T_gripper2_f
                    robot_translation_f = float(np.linalg.norm(A_f[:3, 3]))
                    robot_rotation_f = float(np.arccos(np.clip((np.trace(A_f[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    filtered_motion_amplitudes.append({
                        'pair_idx': f'{i+1}-{i+2}',
                        'translation_mm': robot_translation_f,
                        'rotation_deg': robot_rotation_f,
                        'meets_criteria': robot_translation_f >= MIN_ROBOT_TRANSLATION_MM and robot_rotation_f >= MIN_ROBOT_ROTATION_DEG
                    })
            
            # 更新列表
            filtered_poses_count = T_gripper_list_orig_count - len(T_gripper_filtered)
            T_gripper_list = T_gripper_filtered
            T_board_list = T_board_filtered
            
            if len(T_gripper_list) < 4:
                raise ValueError(f'数据过滤后有效姿态不足，至少需要4个姿态，当前只有{len(T_gripper_list)}个（已过滤{filtered_pairs_count}个运动幅度过小的姿态对，移除了{filtered_poses_count}个姿态）')
            
            if filtered_pairs_count > 0:
                self.get_logger().warning(f'   [OpenCV模式] ⚠️ 数据过滤完成：保留{len(T_gripper_list)}个姿态（过滤{filtered_pairs_count}个运动幅度过小的姿态对，移除了{filtered_poses_count}个姿态）')
            else:
                self.get_logger().info(f'   [OpenCV模式] 数据过滤完成：保留{len(T_gripper_list)}个姿态（无需过滤）')
            
            # 转换为OpenCV需要的格式
            R_gripper2base_list = []
            t_gripper2base_list = []
            rvecs_list = []  # 旋转向量列表（用于OpenCV calibrateHandEye）
            tvecs_list = []  # 平移向量列表（用于OpenCV calibrateHandEye）
            
            # 从motion_groups中提取rvec和tvec（如果存在）
            # 如果不存在，则从T_board转换回rvec
            rvec_found_count = 0
            rvec_converted_count = 0
            tvec_found_count = 0
            tvec_converted_count = 0
            
            for idx, (T_gripper, T_board) in enumerate(zip(T_gripper_list, T_board_list)):
                R_gripper2base_list.append(T_gripper[:3, :3])
                t_gripper2base_list.append(T_gripper[:3, 3])
                
                # 尝试从保存的原始数据中获取rvec和tvec（优先使用）
                rvec_found = False
                tvec_found = False
                rvec_source = None
                tvec_source = None
                
                # 首先检查board_rvec_tvec_map中是否保存了原始rvec/tvec（在去重时保存的）
                board_key = pose_key(T_board)  # 使用pose_key生成一致的键
                if board_key in board_rvec_tvec_map:
                    rvec_tvec_data = board_rvec_tvec_map[board_key]
                    rvec_data = rvec_tvec_data['rvec']
                    tvec_data = rvec_tvec_data['tvec']
                    rvec = np.array([[rvec_data['x']], [rvec_data['y']], [rvec_data['z']]], dtype=np.float64)
                    tvec = np.array([[tvec_data['x']], [tvec_data['y']], [tvec_data['z']]], dtype=np.float64)
                    rvecs_list.append(rvec)
                    tvecs_list.append(tvec)
                    rvec_found = True
                    tvec_found = True
                    rvec_source = 'original_data'
                    tvec_source = 'original_data'
                    rvec_found_count += 1
                    tvec_found_count += 1
                else:
                    # 如果没有保存的原始数据，从motion_groups中查找匹配的board_pose
                    for motion in motion_groups:
                        board_pose1 = motion.get('board_pose1', {})
                        board_pose2 = motion.get('board_pose2', {})
                        
                        # 检查board_pose1是否匹配
                        if 'rvec' in board_pose1 and 'tvec' in board_pose1:
                            # 从四元数和位置重建T矩阵进行比较
                            T_check = _pose_to_transform_matrix(
                                {'x': board_pose1['position']['x'], 'y': board_pose1['position']['y'], 'z': board_pose1['position']['z']},
                                {'x': board_pose1['orientation']['x'], 'y': board_pose1['orientation']['y'], 
                                 'z': board_pose1['orientation']['z'], 'w': board_pose1['orientation']['w']}
                            )
                            if np.allclose(T_check, T_board, atol=1e-3):
                                rvec = np.array([[board_pose1['rvec']['x']], [board_pose1['rvec']['y']], [board_pose1['rvec']['z']]], dtype=np.float64)
                                tvec = np.array([[board_pose1['tvec']['x']], [board_pose1['tvec']['y']], [board_pose1['tvec']['z']]], dtype=np.float64)
                                rvecs_list.append(rvec)
                                tvecs_list.append(tvec)
                                rvec_found = True
                                tvec_found = True
                                rvec_found_count += 1
                                tvec_found_count += 1
                                break
                        
                        # 检查board_pose2是否匹配
                        if not rvec_found and 'rvec' in board_pose2 and 'tvec' in board_pose2:
                            T_check = _pose_to_transform_matrix(
                                {'x': board_pose2['position']['x'], 'y': board_pose2['position']['y'], 'z': board_pose2['position']['z']},
                                {'x': board_pose2['orientation']['x'], 'y': board_pose2['orientation']['y'], 
                                 'z': board_pose2['orientation']['z'], 'w': board_pose2['orientation']['w']}
                            )
                            if np.allclose(T_check, T_board, atol=1e-3):
                                rvec = np.array([[board_pose2['rvec']['x']], [board_pose2['rvec']['y']], [board_pose2['rvec']['z']]], dtype=np.float64)
                                tvec = np.array([[board_pose2['tvec']['x']], [board_pose2['tvec']['y']], [board_pose2['tvec']['z']]], dtype=np.float64)
                            rvecs_list.append(rvec)
                            tvecs_list.append(tvec)
                            rvec_found = True
                            tvec_found = True
                            rvec_source = 'motion_groups_match'
                            tvec_source = 'motion_groups_match'
                            rvec_found_count += 1
                            tvec_found_count += 1
                            break
                
                # 如果没有找到rvec/tvec，从旋转矩阵转换回rvec（兼容旧数据）
                if not rvec_found:
                    rvec_source = 'converted_from_matrix'
                    R_board = T_board[:3, :3]
                    rvec, _ = cv2.Rodrigues(R_board)
                    rvecs_list.append(rvec)
                    rvec_converted_count += 1
                    
                    # 检查转换精度
                    R_reconstructed, _ = cv2.Rodrigues(rvec)
                    rvec_recon_error = np.linalg.norm(R_reconstructed - R_board, 'fro')
                    
                
                if not tvec_found:
                    tvec = T_board[:3, 3].reshape(3, 1)
                    tvecs_list.append(tvec)
                    tvec_source = 'converted_from_matrix'
                    tvec_converted_count += 1
            
            self.get_logger().info(f'   [OpenCV模式] rvec/tvec提取：从原始数据找到{rvec_found_count}个rvec，转换{rvec_converted_count}个；找到{tvec_found_count}个tvec，转换{tvec_converted_count}个')
            
            # 验证数据一致性
            if len(R_gripper2base_list) != len(rvecs_list):
                raise ValueError(f'数据不一致: R_gripper2base({len(R_gripper2base_list)}) != rvecs({len(rvecs_list)})')
            if len(t_gripper2base_list) != len(tvecs_list):
                raise ValueError(f'数据不一致: t_gripper2base({len(t_gripper2base_list)}) != tvecs({len(tvecs_list)})')
            
            # 单位转换：毫米 → 米（OpenCV要求）
            tvecs_list_z_mm = [float(t[2, 0]) for t in tvecs_list]
            tvecs_list_norms_mm = [float(np.linalg.norm(t)) for t in tvecs_list]
            
            R_gripper2base = np.array(R_gripper2base_list, dtype=np.float64)
            t_gripper2base = [t.reshape(3, 1) / 1000.0 for t in t_gripper2base_list]  # 毫米 → 米
            rvecs = rvecs_list  # 弧度，无需转换
            tvecs = [t / 1000.0 for t in tvecs_list]  # 毫米 → 米
            
            # 验证单位转换后的数据合理性
            if len(t_gripper2base) > 0:
                sample_t_norm = float(np.linalg.norm(t_gripper2base[0]))
                if sample_t_norm > 10.0:  # 如果>10m，可能是单位错误
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 机器人位姿平移异常大 {sample_t_norm:.3f}m，请检查单位')
                
                # 记录所有机器人位姿的范围（用于诊断）
                robot_t_norms = [float(np.linalg.norm(t)) for t in t_gripper2base]
                robot_t_min = float(np.min(robot_t_norms))
                robot_t_max = float(np.max(robot_t_norms))
                robot_t_mean = float(np.mean(robot_t_norms))
                robot_t_std = float(np.std(robot_t_norms))
                
                self.get_logger().info(f'   [OpenCV模式] 机器人位姿平移范围（米）: 最小={robot_t_min:.3f}, 最大={robot_t_max:.3f}, 平均={robot_t_mean:.3f}, 标准差={robot_t_std:.3f}')
            
            if len(tvecs) > 0:
                sample_tvec_norm = float(np.linalg.norm(tvecs[0]))
                if sample_tvec_norm > 10.0 or sample_tvec_norm < 0.01:  # 0.01m - 10m范围
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 标定板位姿平移异常 {sample_tvec_norm:.3f}m，请检查单位')
                
                # 记录所有标定板位姿的范围（用于诊断）
                board_tvec_norms = [float(np.linalg.norm(t)) for t in tvecs]
                board_tvec_z_values = [float(t[2, 0]) for t in tvecs]  # Z方向值（米）
                board_tvec_min = float(np.min(board_tvec_norms))
                board_tvec_max = float(np.max(board_tvec_norms))
                board_tvec_mean = float(np.mean(board_tvec_norms))
                board_tvec_std = float(np.std(board_tvec_norms))
                board_z_min = float(np.min(board_tvec_z_values))
                board_z_max = float(np.max(board_tvec_z_values))
                board_z_mean = float(np.mean(board_tvec_z_values))
                
                self.get_logger().info(f'   [OpenCV模式] 标定板位姿平移范围（米）: 最小={board_tvec_min:.3f}, 最大={board_tvec_max:.3f}, 平均={board_tvec_mean:.3f}, 标准差={board_tvec_std:.3f}')
                self.get_logger().info(f'   [OpenCV模式] 标定板位姿Z方向范围（米）: 最小={board_z_min:.3f}, 最大={board_z_max:.3f}, 平均={board_z_mean:.3f}')
                
                # 检查Z值是否异常大（正常范围应该在0.3-0.8米，即300-800mm）
                if board_z_mean > 2.0:  # 如果平均Z值>2米（2000mm），可能是单位错误或数据异常
                    self.get_logger().error(f'   [OpenCV模式] ❌ 严重问题：标定板Z方向平均距离异常大 {board_z_mean:.3f}m（{board_z_mean*1000:.1f}mm）！')
                    self.get_logger().error(f'      正常情况下，标定板到相机的距离应该在0.3-0.8m（300-800mm）范围内')
                    self.get_logger().error(f'      可能原因：')
                    self.get_logger().error(f'        1. solvePnP的tvec单位不正确（检查square_size单位）')
                    self.get_logger().error(f'        2. 标定板离相机过远')
                    self.get_logger().error(f'        3. 标定板位姿检测不准确')
                elif board_z_mean > 1.5:  # 1.5-2.0米，警告
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 标定板Z方向平均距离较大 {board_z_mean:.3f}m（{board_z_mean*1000:.1f}mm），建议检查')
            
            self.get_logger().info(f'   [OpenCV模式] 单位转换完成：机器人位姿({len(t_gripper2base)}个)和标定板位姿({len(tvecs)}个)已转换为米')
            
            return R_gripper2base, t_gripper2base, rvecs, tvecs, T_gripper_list, T_board_list
        
        def _solve_opencv_hand_eye(R_gripper2base, t_gripper2base, rvecs, tvecs):
            """
            OpenCV模式：调用calibrateHandEye算法计算手眼标定
            
            参数:
                R_gripper2base: (N, 3, 3) numpy数组，机器人旋转矩阵列表
                t_gripper2base: 列表，每个元素(3,1)数组，机器人平移向量（单位：米）
                rvecs: 列表，每个元素(3,1)数组，标定板旋转向量（单位：弧度）
                tvecs: 列表，每个元素(3,1)数组，标定板平移向量（单位：米）
            
            返回:
                T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
            """
            import json
            import time
            
            self.get_logger().info(f'   [OpenCV模式] 开始算法计算（TSAI方法）')
            self.get_logger().info(f'   输入数据：{len(R_gripper2base)} 个姿态')
            
            # 验证输入数据格式
            if len(R_gripper2base) != len(t_gripper2base):
                raise ValueError(f'输入数据不一致: R_gripper2base({len(R_gripper2base)}) != t_gripper2base({len(t_gripper2base)})')
            if len(rvecs) != len(tvecs):
                raise ValueError(f'输入数据不一致: rvecs({len(rvecs)}) != tvecs({len(tvecs)})')
            if len(R_gripper2base) != len(rvecs):
                raise ValueError(f'输入数据不一致: R_gripper2base({len(R_gripper2base)}) != rvecs({len(rvecs)})')
            
            # 验证数据格式
            for i, (R, t, rvec, tvec) in enumerate(zip(R_gripper2base, t_gripper2base, rvecs, tvecs)):
                if R.shape != (3, 3):
                    raise ValueError(f'姿态#{i+1}: R形状错误 {R.shape}，应为(3,3)')
                if t.shape != (3, 1):
                    raise ValueError(f'姿态#{i+1}: t形状错误 {t.shape}，应为(3,1)')
                if rvec.shape != (3, 1):
                    raise ValueError(f'姿态#{i+1}: rvec形状错误 {rvec.shape}，应为(3,1)')
                if tvec.shape != (3, 1):
                    raise ValueError(f'姿态#{i+1}: tvec形状错误 {tvec.shape}，应为(3,1)')
            
            # 检查旋转矩阵的行列式（应该接近1.0）
            det_errors = []
            for i, R in enumerate(R_gripper2base):
                det = np.linalg.det(R)
                if abs(det - 1.0) > 0.01:
                    det_errors.append((i+1, det))
            
            if det_errors:
                self.get_logger().warning(f'   [OpenCV模式] ⚠️ 发现{len(det_errors)}个旋转矩阵行列式异常')
                for idx, det in det_errors[:5]:  # 只显示前5个
                    self.get_logger().warning(f'     姿态#{idx}: det={det:.6f}')
            
            # 调用OpenCV calibrateHandEye
            solver_method = cv2.CALIB_HAND_EYE_DANIILIDIS
            
            try:
                # 数据质量检查：计算相邻姿态之间的运动幅度
                # 如果运动幅度过小，可能导致算法不稳定
                motion_amplitudes_check = []
                for i in range(len(R_gripper2base) - 1):
                    # 计算机器人运动
                    R1 = R_gripper2base[i]
                    t1 = t_gripper2base[i]
                    R2 = R_gripper2base[i + 1]
                    t2 = t_gripper2base[i + 1]
                    
                    # A = inv(T1) @ T2
                    T1 = np.eye(4)
                    T1[:3, :3] = R1
                    T1[:3, 3] = t1.flatten()
                    T2 = np.eye(4)
                    T2[:3, :3] = R2
                    T2[:3, 3] = t2.flatten()
                    T1_inv = np.linalg.inv(T1)
                    A = T1_inv @ T2
                    
                    robot_translation = float(np.linalg.norm(A[:3, 3]))
                    robot_rotation = float(np.arccos(np.clip((np.trace(A[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    
                    # 计算标定板运动
                    rvec1 = rvecs[i]
                    tvec1 = tvecs[i]
                    rvec2 = rvecs[i + 1]
                    tvec2 = tvecs[i + 1]
                    
                    R_board1, _ = cv2.Rodrigues(rvec1)
                    R_board2, _ = cv2.Rodrigues(rvec2)
                    T_board1 = np.eye(4)
                    T_board1[:3, :3] = R_board1
                    T_board1[:3, 3] = tvec1.flatten()
                    T_board2 = np.eye(4)
                    T_board2[:3, :3] = R_board2
                    T_board2[:3, 3] = tvec2.flatten()
                    T_board2_inv = np.linalg.inv(T_board2)
                    B = T_board1 @ T_board2_inv
                    
                    board_translation = float(np.linalg.norm(B[:3, 3]))
                    board_rotation = float(np.arccos(np.clip((np.trace(B[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    
                    motion_amplitudes_check.append({
                        'motion_pair_idx': i + 1,
                        'robot_translation_m': robot_translation,
                        'robot_rotation_deg': robot_rotation,
                        'board_translation_m': board_translation,
                        'board_rotation_deg': board_rotation
                    })
                
                # 检查是否有运动幅度过小的情况
                small_motions = [m for m in motion_amplitudes_check if m['robot_translation_m'] < 0.01 or m['robot_rotation_deg'] < 1.0]
                if small_motions:
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 发现{len(small_motions)}个运动幅度过小的运动对，可能导致算法不稳定')
                    for m in small_motions[:5]:
                        self.get_logger().warning(f'     运动对#{m["motion_pair_idx"]}: 平移={m["robot_translation_m"]*1000:.2f}mm, 旋转={m["robot_rotation_deg"]:.2f}°')
                
                """
                # 详细记录所有输入数据，特别是tvecs的Z值，用于分析Z值误差的来源
                all_robot_t = [t.flatten().tolist() for t in t_gripper2base]
                all_robot_t_norms = [float(np.linalg.norm(t)) for t in t_gripper2base]
                all_tvecs = [t.flatten().tolist() for t in tvecs]
                all_tvec_norms = [float(np.linalg.norm(t)) for t in tvecs]
                all_tvec_z = [float(t[2, 0]) for t in tvecs]
                
                # 检查机器人位姿和标定板位姿的对应关系
                # 计算每个姿态对中，机器人位姿和标定板位姿的Z值差异
                pose_pairs_analysis = []
                for i in range(len(t_gripper2base)):
                    robot_t = t_gripper2base[i]
                    tvec = tvecs[i]
                    robot_z = float(robot_t[2, 0])
                    board_z = float(tvec[2, 0])
                    z_diff = abs(robot_z - board_z)
                    pose_pairs_analysis.append({
                        'pose_idx': i + 1,
                        'robot_z_m': robot_z,
                        'robot_z_mm': robot_z * 1000.0,
                        'board_z_m': board_z,
                        'board_z_mm': board_z * 1000.0,
                        'z_diff_m': z_diff,
                        'z_diff_mm': z_diff * 1000.0,
                        'z_diff_ratio': z_diff / board_z if board_z > 1e-6 else 9999.0
                    })
                
                # 计算输入数据的统计信息
                robot_t_stats = {
                    'min_norm': float(np.min(all_robot_t_norms)),
                    'max_norm': float(np.max(all_robot_t_norms)),
                    'mean_norm': float(np.mean(all_robot_t_norms)),
                    'std_norm': float(np.std(all_robot_t_norms)),
                    'all_norms': all_robot_t_norms
                }
                tvec_stats = {
                    'min_norm': float(np.min(all_tvec_norms)),
                    'max_norm': float(np.max(all_tvec_norms)),
                    'mean_norm': float(np.mean(all_tvec_norms)),
                    'std_norm': float(np.std(all_tvec_norms)),
                    'all_norms': all_tvec_norms
                }
                tvec_z_stats = {
                    'min': float(np.min(all_tvec_z)),
                    'max': float(np.max(all_tvec_z)),
                    'mean': float(np.mean(all_tvec_z)),
                    'std': float(np.std(all_tvec_z)),
                    'median': float(np.median(all_tvec_z)),
                    'all_values': all_tvec_z
                }
                
                log_entry_z_tracking_before = {
                    'sessionId': 'debug-session',
                    'runId': 'opencv-full-pipeline',
                    'hypothesisId': 'Z_TRACK1',
                    'location': 'hand_eye_calibration_node.py:_solve_opencv_hand_eye',
                    'message': '[Z值追踪] OpenCV调用前的输入数据详细分析',
                    'data': {
                        'num_poses': len(R_gripper2base),
                        'robot_t_statistics_m': robot_t_stats,
                        'tvec_statistics_m': tvec_stats,
                        'tvec_z_statistics_m': tvec_z_stats,
                        'tvec_z_statistics_mm': {
                            'min': tvec_z_stats['min'] * 1000.0,
                            'max': tvec_z_stats['max'] * 1000.0,
                            'mean': tvec_z_stats['mean'] * 1000.0,
                            'std': tvec_z_stats['std'] * 1000.0,
                            'median': tvec_z_stats['median'] * 1000.0,
                            'all_values_mm': [z * 1000.0 for z in all_tvec_z]
                        },
                        'pose_pairs_analysis': pose_pairs_analysis,
                        'robot_board_z_correlation': {
                            'robot_z_mean_m': float(np.mean([p['robot_z_m'] for p in pose_pairs_analysis])),
                            'robot_z_mean_mm': float(np.mean([p['robot_z_mm'] for p in pose_pairs_analysis])),
                            'board_z_mean_m': float(np.mean([p['board_z_m'] for p in pose_pairs_analysis])),
                            'board_z_mean_mm': float(np.mean([p['board_z_mm'] for p in pose_pairs_analysis])),
                            'z_diff_mean_m': float(np.mean([p['z_diff_m'] for p in pose_pairs_analysis])),
                            'z_diff_mean_mm': float(np.mean([p['z_diff_mm'] for p in pose_pairs_analysis])),
                            'z_diff_max_mm': float(np.max([p['z_diff_mm'] for p in pose_pairs_analysis])),
                            'z_diff_min_mm': float(np.min([p['z_diff_mm'] for p in pose_pairs_analysis]))
                        },
                        'expected_tvec_z_range_m': [0.3, 0.8],  # 正常范围300-800mm
                        'expected_tvec_z_range_mm': [300.0, 800.0],
                        'sample_robot_t_m': all_robot_t[0] if len(all_robot_t) > 0 else None,
                        'sample_tvec_m': all_tvecs[0] if len(all_tvecs) > 0 else None,
                        'sample_tvec_z_m': all_tvec_z[0] if len(all_tvec_z) > 0 else None,
                        'sample_tvec_z_mm': all_tvec_z[0] * 1000.0 if len(all_tvec_z) > 0 else None
                    },
                    'timestamp': int(time.time() * 1000)
                }
                log_entry_clean = _convert_to_json_serializable(log_entry_z_tracking_before)
                # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                """
                
                R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,  # 单位：米
                    rvecs, tvecs,                     # 单位：米
                    method=solver_method
                )
                t_cam2gripper_flat = t_cam2gripper.flatten()
                t_cam2gripper_norm_m = float(np.linalg.norm(t_cam2gripper))
                t_cam2gripper_norm_mm = t_cam2gripper_norm_m * 1000.0
            except Exception as e:
                raise
            
            # 检查返回结果的合理性
            if t_cam2gripper_norm_mm > 2000.0:  # 如果>2米（2000mm），异常
                self.get_logger().error(f'   [OpenCV模式] ❌ 严重问题：OpenCV返回的平移向量异常大 {t_cam2gripper_norm_mm:.1f}mm！')
                self.get_logger().error(f'      正常范围应该在100-1000mm')
                self.get_logger().error(f'      Z方向: {t_cam2gripper_flat[2]*1000:.1f}mm')
                self.get_logger().error(f'      可能原因：输入数据有问题（标定板位姿或机器人位姿不正确）')
            elif t_cam2gripper_norm_mm > 1000.0:  # 1-2米，警告
                self.get_logger().warning(f'   [OpenCV模式] ⚠️ OpenCV返回的平移向量较大 {t_cam2gripper_norm_mm:.1f}mm，建议检查输入数据')
            
            if abs(t_cam2gripper_flat[2]) * 1000.0 > 1500.0:  # Z方向>1.5米
                self.get_logger().error(f'   [OpenCV模式] ❌ 严重问题：Z方向平移异常大 {t_cam2gripper_flat[2]*1000:.1f}mm！')
                self.get_logger().error(f'      正常情况下，相机到末端执行器的Z方向距离应该在100-800mm范围内')
                self.get_logger().error(f'      这可能表明标定板位姿的Z值有问题，或者数据不匹配')
            
            # 验证返回结果
            det_R = np.linalg.det(R_cam2gripper)
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warning(f'   [OpenCV模式] ⚠️ 返回的旋转矩阵行列式异常: {det_R:.6f}（应为1.0）')
            
            # 构建4x4变换矩阵（单位：毫米）
            T_camera2gripper = np.eye(4)
            T_camera2gripper[:3, :3] = R_cam2gripper
            T_camera2gripper[:3, 3] = t_cam2gripper.flatten() * 1000.0  # 米 → 毫米
            
            self.get_logger().info(f'   [OpenCV模式] 算法计算完成：t_cam2gripper={np.linalg.norm(T_camera2gripper[:3, 3]):.1f}mm, det(R)={det_R:.6f}')
            
            return T_camera2gripper
        
        def _calculate_opencv_errors(T_gripper_list, T_board_list, T_camera2gripper):
            """
            OpenCV模式：计算标定误差（验证AX=XB约束）
            
            参数:
                T_gripper_list: 列表，机器人变换矩阵（单位：毫米）
                T_board_list: 列表，标定板变换矩阵（单位：毫米）
                T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
            
            返回:
                translation_errors: 列表，每个运动组的平移误差（单位：毫米）
                rotation_errors: 列表，每个运动组的旋转误差（单位：弧度）
                error_statistics: 字典，包含RMS、最大、最小、标准差等统计信息
            """
            import json
            import time
            
            # #region agent log - 误差计算开始
            log_entry = {
                'sessionId': 'debug-session',
                'runId': 'opencv-errors',
                'hypothesisId': 'OP14',
                'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                'message': 'OpenCV模式：开始误差计算',
                'data': {
                    'num_poses': len(T_gripper_list),
                    'expected_motion_groups': len(T_gripper_list) - 1
                },
                'timestamp': int(time.time() * 1000)
            }
            # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry) + '\n')
            # #endregion
            
            self.get_logger().info(f'   [OpenCV模式] 开始误差计算：验证AX=XB约束')
            
            translation_errors = []
            rotation_errors = []
            
            # 验证输入数据一致性
            if len(T_gripper_list) != len(T_board_list):
                raise ValueError(f'输入数据不一致: T_gripper_list({len(T_gripper_list)}) != T_board_list({len(T_board_list)})')
            
            # 从去重后的姿态列表构建运动组（相邻姿态对）
            A_list_for_error = []  # 机器人运动矩阵列表
            B_list_for_error = []  # 标定板运动矩阵列表
            
            num_motions = len(T_gripper_list) - 1  # 相邻姿态对的数量
            
            self.get_logger().info(f'   [OpenCV模式] 构建{num_motions}个运动组用于误差验证')
            
            # 统计运动幅度（用于数据质量诊断）
            motion_amplitudes = []
            
            for i in range(num_motions):
                pose1_idx = i
                pose2_idx = i + 1
                
                if pose2_idx >= len(T_gripper_list):
                    continue
                
                try:
                    # 计算机器人运动：A = inv(T_gripper1) @ T_gripper2
                    T_gripper1 = T_gripper_list[pose1_idx]
                    T_gripper2 = T_gripper_list[pose2_idx]
                    T_gripper1_inv = np.linalg.inv(T_gripper1)
                    A = T_gripper1_inv @ T_gripper2
                    A_list_for_error.append(A)
                    
                    robot_translation = float(np.linalg.norm(A[:3, 3]))
                    robot_rotation_angle = float(np.arccos(np.clip((np.trace(A[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    
                    # 计算标定板运动：B = T_board1 @ inv(T_board2)
                    T_board1 = T_board_list[pose1_idx]
                    T_board2 = T_board_list[pose2_idx]
                    T_board2_inv = np.linalg.inv(T_board2)
                    B = T_board1 @ T_board2_inv
                    B_list_for_error.append(B)
                    
                    board_translation = float(np.linalg.norm(B[:3, 3]))
                    board_rotation_angle = float(np.arccos(np.clip((np.trace(B[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                    
                    motion_amplitudes.append({
                        'motion_group_idx': i + 1,
                        'robot_translation_mm': robot_translation,
                        'robot_rotation_deg': robot_rotation_angle,
                        'board_translation_mm': board_translation,
                        'board_rotation_deg': board_rotation_angle
                    })
                    
                    # #region agent log - 运动组幅度
                    if i < 5:  # 记录前5个运动组
                        log_entry = {
                            'sessionId': 'debug-session',
                            'runId': 'opencv-errors',
                            'hypothesisId': 'OP15',
                            'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                            'message': f'OpenCV模式：运动组#{i+1}运动幅度',
                            'data': {
                                'motion_group_idx': i + 1,
                                'robot_translation_mm': robot_translation,
                                'robot_rotation_deg': robot_rotation_angle,
                                'board_translation_mm': board_translation,
                                'board_rotation_deg': board_rotation_angle
                            },
                            'timestamp': int(time.time() * 1000)
                        }
                        # 确保所有NumPy类型都被转换为Python原生类型
                        log_entry_clean = _convert_to_json_serializable(log_entry)
                        # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                    # #endregion
                    
                    self.get_logger().info(f'     运动组 #{i+1}: 机器人平移 {robot_translation:.1f}mm, 旋转 {robot_rotation_angle:.1f}° | 标定板平移 {board_translation:.1f}mm, 旋转 {board_rotation_angle:.1f}°')
                    
                except Exception as e:
                    self.get_logger().warning(f'   [OpenCV模式] 运动组 #{i+1} 构建失败: {str(e)}')
                    # #region agent log - 运动组构建失败
                    log_entry = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-errors',
                        'hypothesisId': 'OP16',
                        'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                        'message': f'OpenCV模式：运动组#{i+1}构建失败',
                        'data': {
                            'motion_group_idx': i + 1,
                            'error': str(e)
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry) + '\n')
                    # #endregion
                    continue
            
            if len(A_list_for_error) == 0:
                raise ValueError('无法构建运动组，至少需要2个姿态')
            
            # 记录运动幅度的统计信息（用于诊断数据质量）
            if len(motion_amplitudes) > 0:
                robot_translations = [m['robot_translation_mm'] for m in motion_amplitudes]
                robot_rotations = [m['robot_rotation_deg'] for m in motion_amplitudes]
                board_translations = [m['board_translation_mm'] for m in motion_amplitudes]
                board_rotations = [m['board_rotation_deg'] for m in motion_amplitudes]
                
                self.get_logger().info(f'   [OpenCV模式] 运动幅度统计:')
                self.get_logger().info(f'     机器人平移: 平均={np.mean(robot_translations):.1f}mm, 最大={np.max(robot_translations):.1f}mm, 最小={np.min(robot_translations):.1f}mm')
                self.get_logger().info(f'     机器人旋转: 平均={np.mean(robot_rotations):.1f}°, 最大={np.max(robot_rotations):.1f}°, 最小={np.min(robot_rotations):.1f}°')
                self.get_logger().info(f'     标定板平移: 平均={np.mean(board_translations):.1f}mm, 最大={np.max(board_translations):.1f}mm, 最小={np.min(board_translations):.1f}mm')
                self.get_logger().info(f'     标定板旋转: 平均={np.mean(board_rotations):.1f}°, 最大={np.max(board_rotations):.1f}°, 最小={np.min(board_rotations):.1f}°')
                
                # 检查运动幅度是否足够
                if np.mean(robot_rotations) < 10.0:
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 机器人平均旋转幅度过小 {np.mean(robot_rotations):.1f}°，建议至少15-30°')
                if np.mean(robot_translations) < 50.0:
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 机器人平均平移幅度过小 {np.mean(robot_translations):.1f}mm，建议至少100-200mm')
                if np.mean(board_rotations) < 5.0:
                    self.get_logger().warning(f'   [OpenCV模式] ⚠️ 标定板平均旋转幅度过小 {np.mean(board_rotations):.1f}°，检查标定板检测精度')
                
                # #region agent log - 运动幅度统计
                log_entry = {
                    'sessionId': 'debug-session',
                    'runId': 'opencv-errors',
                    'hypothesisId': 'OP17A',
                    'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                    'message': 'OpenCV模式：运动幅度统计',
                    'data': {
                        'num_motion_groups': len(motion_amplitudes),
                        'robot_translation_mm': {
                            'mean': float(np.mean(robot_translations)),
                            'max': float(np.max(robot_translations)),
                            'min': float(np.min(robot_translations)),
                            'std': float(np.std(robot_translations))
                        },
                        'robot_rotation_deg': {
                            'mean': float(np.mean(robot_rotations)),
                            'max': float(np.max(robot_rotations)),
                            'min': float(np.min(robot_rotations)),
                            'std': float(np.std(robot_rotations))
                        },
                        'board_translation_mm': {
                            'mean': float(np.mean(board_translations)),
                            'max': float(np.max(board_translations)),
                            'min': float(np.min(board_translations)),
                            'std': float(np.std(board_translations))
                        },
                        'board_rotation_deg': {
                            'mean': float(np.mean(board_rotations)),
                            'max': float(np.max(board_rotations)),
                            'min': float(np.min(board_rotations)),
                            'std': float(np.std(board_rotations))
                        }
                    },
                    'timestamp': int(time.time() * 1000)
                }
                log_entry_clean = _convert_to_json_serializable(log_entry)
                # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                # #endregion
            
            # #region agent log - 误差计算阶段：开始计算
            log_entry_error_start = {
                'sessionId': 'debug-session',
                'runId': 'opencv-full-pipeline',
                'hypothesisId': 'FULL13',
                'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                'message': '[误差计算] 开始计算误差',
                'data': {
                    'num_motion_groups': len(A_list_for_error),
                    'T_camera2gripper_mm': T_camera2gripper.tolist(),
                    'T_camera2gripper_translation_mm': T_camera2gripper[:3, 3].tolist(),
                    'T_camera2gripper_z_mm': float(T_camera2gripper[2, 3])
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_error_start)
            # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
            # #endregion
            
            # 直接验证AX=XB约束：计算 A @ X - X @ B 的误差
            for i in range(len(A_list_for_error)):
                try:
                    A = A_list_for_error[i]
                    B = B_list_for_error[i]
                    
                    # #region agent log - 误差计算阶段：每个运动组数据
                    log_entry_motion_group = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-full-pipeline',
                        'hypothesisId': 'FULL14',
                        'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                        'message': f'[误差计算] 运动组#{i+1}矩阵A和B',
                        'data': {
                            'motion_group_idx': i + 1,
                            'A_translation_mm': A[:3, 3].tolist(),
                            'A_translation_norm_mm': float(np.linalg.norm(A[:3, 3])),
                            'A_rotation_deg': float(np.arccos(np.clip((np.trace(A[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi),
                            'B_translation_mm': B[:3, 3].tolist(),
                            'B_translation_norm_mm': float(np.linalg.norm(B[:3, 3])),
                            'B_rotation_deg': float(np.arccos(np.clip((np.trace(B[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / np.pi)
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    log_entry_clean = _convert_to_json_serializable(log_entry_motion_group)
                    # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                    # #endregion
                    
                    # 计算误差矩阵：E = A @ X - X @ B
                    # 理想情况下，E应该为零矩阵
                    AX = A @ T_camera2gripper
                    XB = T_camera2gripper @ B
                    error_matrix = AX - XB
                    
                    # #region agent log - 误差计算阶段：中间计算结果
                    log_entry_intermediate = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-full-pipeline',
                        'hypothesisId': 'FULL15',
                        'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                        'message': f'[误差计算] 运动组#{i+1}中间计算结果',
                        'data': {
                            'motion_group_idx': i + 1,
                            'AX_translation_mm': AX[:3, 3].tolist(),
                            'AX_translation_norm_mm': float(np.linalg.norm(AX[:3, 3])),
                            'XB_translation_mm': XB[:3, 3].tolist(),
                            'XB_translation_norm_mm': float(np.linalg.norm(XB[:3, 3])),
                            'error_matrix_translation_mm': error_matrix[:3, 3].tolist(),
                            'error_matrix_translation_norm_mm': float(np.linalg.norm(error_matrix[:3, 3]))
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    log_entry_clean = _convert_to_json_serializable(log_entry_intermediate)
                    # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                    # #endregion
                    
                    # 平移误差：误差矩阵的平移部分的范数
                    translation_error = float(np.linalg.norm(error_matrix[:3, 3]))
                    translation_errors.append(translation_error)
                    
                    # 旋转误差：计算A@X和X@B的旋转部分的角度差
                    R_AX = (A @ T_camera2gripper)[:3, :3]
                    R_XB = (T_camera2gripper @ B)[:3, :3]
                    R_diff = R_AX @ R_XB.T
                    trace = np.trace(R_diff)
                    cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
                    rotation_error_angle = float(np.arccos(cos_angle))
                    rotation_errors.append(rotation_error_angle)
                    
                    # #region agent log - 误差计算阶段：每个运动组误差
                    log_entry_error = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-full-pipeline',
                        'hypothesisId': 'FULL16',
                        'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                        'message': f'[误差计算] 运动组#{i+1}最终误差',
                        'data': {
                            'motion_group_idx': i + 1,
                            'translation_error_mm': translation_error,
                            'rotation_error_rad': rotation_error_angle,
                            'rotation_error_deg': float(np.degrees(rotation_error_angle)),
                            'error_matrix_frobenius': float(np.linalg.norm(error_matrix, 'fro')),
                            'R_AX_det': float(np.linalg.det(R_AX)),
                            'R_XB_det': float(np.linalg.det(R_XB)),
                            'R_diff_trace': float(trace)
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    log_entry_clean = _convert_to_json_serializable(log_entry_error)
                    # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                    # #endregion
                    
                    # #region agent log - 每个运动组误差
                    if i < 5:  # 记录前5个运动组的详细误差
                        log_entry = {
                            'sessionId': 'debug-session',
                            'runId': 'opencv-errors',
                            'hypothesisId': 'OP17',
                            'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                            'message': f'OpenCV模式：运动组#{i+1}误差',
                            'data': {
                                'motion_group_idx': i + 1,
                                'translation_error_mm': translation_error,
                                'rotation_error_rad': rotation_error_angle,
                                'rotation_error_deg': float(np.degrees(rotation_error_angle)),
                                'error_matrix_frobenius': float(np.linalg.norm(error_matrix, 'fro'))
                            },
                            'timestamp': int(time.time() * 1000)
                        }
                        # 确保所有NumPy类型都被转换为Python原生类型
                        log_entry_clean = _convert_to_json_serializable(log_entry)
                        # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
                    # #endregion
                    
                except Exception as e:
                    self.get_logger().warning(f'   [OpenCV模式] 运动组 #{i+1} 误差计算失败: {str(e)}')
                    # #region agent log - 误差计算失败
                    log_entry = {
                        'sessionId': 'debug-session',
                        'runId': 'opencv-errors',
                        'hypothesisId': 'OP18',
                        'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                        'message': f'OpenCV模式：运动组#{i+1}误差计算失败',
                        'data': {
                            'motion_group_idx': i + 1,
                            'error': str(e)
                        },
                        'timestamp': int(time.time() * 1000)
                    }
                    # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry) + '\n')
                    # #endregion
                    continue
            
            if len(translation_errors) == 0:
                raise ValueError('无法计算误差，至少需要1个有效的运动组')
            
            # #region agent log - 误差计算阶段：所有误差列表
            log_entry_all_errors = {
                'sessionId': 'debug-session',
                'runId': 'opencv-full-pipeline',
                'hypothesisId': 'FULL17',
                'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                'message': '[误差计算] 所有运动组误差列表',
                'data': {
                    'num_motion_groups': len(translation_errors),
                    'translation_errors_mm': [float(e) for e in translation_errors],
                    'rotation_errors_rad': [float(e) for e in rotation_errors],
                    'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_all_errors)
            # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
            # #endregion
            
            # 计算RMS误差
            mean_translation_error = float(np.sqrt(np.mean([e**2 for e in translation_errors])))
            mean_rotation_error = float(np.sqrt(np.mean([e**2 for e in rotation_errors])))
            max_translation_error = float(np.max(translation_errors))
            min_translation_error = float(np.min(translation_errors))
            std_translation_error = float(np.std(translation_errors))
            max_rotation_error = float(np.max(rotation_errors))
            min_rotation_error = float(np.min(rotation_errors))
            
            # #region agent log - 误差计算阶段：误差统计（最终结果）
            log_entry_statistics = {
                'sessionId': 'debug-session',
                'runId': 'opencv-full-pipeline',
                'hypothesisId': 'FULL18',
                'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                'message': '[误差计算] 误差统计（最终结果）',
                'data': {
                    'num_motion_groups': len(translation_errors),
                    'translation_error_statistics': {
                        'rms_mm': mean_translation_error,
                        'max_mm': max_translation_error,
                        'min_mm': min_translation_error,
                        'std_mm': std_translation_error,
                        'all_errors_mm': [float(e) for e in translation_errors]
                    },
                    'rotation_error_statistics': {
                        'rms_rad': mean_rotation_error,
                        'rms_deg': float(np.degrees(mean_rotation_error)),
                        'max_rad': max_rotation_error,
                        'max_deg': float(np.degrees(max_rotation_error)),
                        'min_rad': min_rotation_error,
                        'min_deg': float(np.degrees(min_rotation_error)),
                        'all_errors_rad': [float(e) for e in rotation_errors],
                        'all_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
                    },
                    'T_camera2gripper_mm': {
                        'translation': T_camera2gripper[:3, 3].tolist(),
                        'translation_norm': float(np.linalg.norm(T_camera2gripper[:3, 3])),
                        'translation_z': float(T_camera2gripper[2, 3]),
                        'rotation_det': float(np.linalg.det(T_camera2gripper[:3, :3])),
                        'full_matrix': T_camera2gripper.tolist()
                    }
                },
                'timestamp': int(time.time() * 1000)
            }
            log_entry_clean = _convert_to_json_serializable(log_entry_statistics)
            # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry_clean) + '\n')
            # #endregion
            
            error_statistics = {
                'mean_translation_error': mean_translation_error,
                'max_translation_error': max_translation_error,
                'min_translation_error': min_translation_error,
                'std_translation_error': std_translation_error,
                'mean_rotation_error_rad': mean_rotation_error,
                'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
                'max_rotation_error_rad': max_rotation_error,
                'max_rotation_error_deg': float(np.degrees(max_rotation_error)),
                'min_rotation_error_rad': min_rotation_error,
                'min_rotation_error_deg': float(np.degrees(min_rotation_error)),
                'translation_errors': translation_errors,
                'rotation_errors_rad': rotation_errors,
                'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
            }
            
            # #region agent log - 误差统计
            log_entry = {
                'sessionId': 'debug-session',
                'runId': 'opencv-errors',
                'hypothesisId': 'OP19',
                'location': 'hand_eye_calibration_node.py:_calculate_opencv_errors',
                'message': 'OpenCV模式：误差统计',
                'data': {
                    'num_motion_groups': len(translation_errors),
                    'mean_translation_error_mm': mean_translation_error,
                    'max_translation_error_mm': max_translation_error,
                    'std_translation_error_mm': std_translation_error,
                    'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
                    'max_rotation_error_deg': float(np.degrees(max_rotation_error))
                },
                'timestamp': int(time.time() * 1000)
            }
            # open('/home/mu/IVG/aubo_ros2_ws/.cursor/debug.log', 'a').write(json.dumps(log_entry) + '\n')
            # #endregion
            
            self.get_logger().info(f'   [OpenCV模式] 误差计算完成：平移RMS={mean_translation_error:.3f}mm，旋转RMS={np.degrees(mean_rotation_error):.3f}°')
            
            return translation_errors, rotation_errors, error_statistics
        
        def _prepare_custom_data(motion_groups):
            """
            Custom模式：准备数据用于XY+Z约束方法
            从motion_groups中提取数据，转换为XY+Z约束方法需要的格式
            
            注意：机器人拍照位姿（pose1）就是机器人点选位姿（pose2），使用所有采集的角点进行后续计算
            
            参数:
                motion_groups: 运动组列表，每个运动组包含pose1, pose2, board_pose1, board_pose2
            
            返回:
                shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
                pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
                camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
            """
            self.get_logger().info(f'   [Custom模式] 开始数据准备：{len(motion_groups)} 个运动组')
            self.get_logger().info(f'   [Custom模式] 使用所有采集的角点，pose1=pose2（拍照位姿=点选位姿）')
            
            if len(motion_groups) == 0:
                raise ValueError('[Custom模式] 没有运动组数据')
            
            # 提取shot_pose（第一个motion_group的pose1，作为固定的拍照姿态）
            first_motion = motion_groups[0]
            pose1_data = first_motion.get('pose1')
            if not pose1_data:
                raise ValueError('[Custom模式] 第一个运动组缺少pose1数据')
            
            shot_pose = _pose_to_transform_matrix(
                {'x': pose1_data['robot_pos_x'] * 1000.0, 'y': pose1_data['robot_pos_y'] * 1000.0, 'z': pose1_data['robot_pos_z'] * 1000.0},
                {'x': pose1_data['robot_ori_x'], 'y': pose1_data['robot_ori_y'], 'z': pose1_data['robot_ori_z'], 'w': pose1_data['robot_ori_w']}
            )
            
            # 提取pick_poses和camera_points_shot
            # 使用所有采集的角点，每个角点对应一个数据点
            pick_poses = []
            camera_points_shot = []
            
            total_corners_count = 0
            skipped_groups = 0
            
            for idx, motion in enumerate(motion_groups):
                # pose1就是pose2（拍照位姿=点选位姿）
                pose1_data = motion.get('pose1')
                board_pose1_data = motion.get('board_pose1')
                
                if not pose1_data or not board_pose1_data:
                    self.get_logger().warning(f'   [Custom模式] 运动组 #{idx+1} 数据不完整，已跳过')
                    skipped_groups += 1
                    continue
                
                # 构建点选姿态（使用pose1，因为pose1=pose2）
                pick_pose = _pose_to_transform_matrix(
                    {'x': pose1_data['robot_pos_x'] * 1000.0, 'y': pose1_data['robot_pos_y'] * 1000.0, 'z': pose1_data['robot_pos_z'] * 1000.0},
                    {'x': pose1_data['robot_ori_x'], 'y': pose1_data['robot_ori_y'], 'z': pose1_data['robot_ori_z'], 'w': pose1_data['robot_ori_w']}
                )
                
                # 提取所有滤波后的角点数据
                corners_3d_filtered = board_pose1_data.get('corners_3d_filtered', [])
                
                if corners_3d_filtered and len(corners_3d_filtered) > 0:
                    # 使用所有滤波后的角点
                    valid_corners_in_group = 0
                    for corner in corners_3d_filtered:
                        if not corner.get('depth_valid', True):
                            # 跳过深度数据无效的角点
                            continue
                        
                        camera_point = np.array([
                            corner['x'],
                            corner['y'],
                            corner['z']
                        ])
                        
                        # 每个角点对应一个数据点，使用相同的点选姿态（pose1）
                        pick_poses.append(pick_pose)
                        camera_points_shot.append(camera_point)
                        valid_corners_in_group += 1
                        total_corners_count += 1
                    
                    self.get_logger().info(f'   [Custom模式] 运动组 #{idx+1}: 使用{valid_corners_in_group}个有效角点')
                else:
                    # 如果没有滤波后的角点，使用board_pose1的position（向后兼容，只使用一个点）
                    camera_point = np.array([
                        board_pose1_data['position']['x'],
                        board_pose1_data['position']['y'],
                        board_pose1_data['position']['z']
                    ])
                    pick_poses.append(pick_pose)
                    camera_points_shot.append(camera_point)
                    total_corners_count += 1
                    self.get_logger().info(f'   [Custom模式] 运动组 #{idx+1}: 使用board_pose1.position（未找到滤波后的角点）')
            
            self.get_logger().info(f'   [Custom模式] 数据准备完成：shot_pose(1个), pick_poses({len(pick_poses)}个), camera_points_shot({len(camera_points_shot)}个)')
            self.get_logger().info(f'   [Custom模式] 共使用{total_corners_count}个角点数据点，跳过{skipped_groups}个无效运动组')
            
            if len(pick_poses) == 0:
                raise ValueError('[Custom模式] 没有有效的角点数据')
            
            return shot_pose, pick_poses, camera_points_shot
        
        def _solve_custom_hand_eye(shot_pose, pick_poses, camera_points_shot):
            """
            Custom模式：调用XY+Z约束方法计算手眼标定
            
            参数:
                shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
                pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
                camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
            
            返回:
                T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
                fixed_z: 固定Z值（棋盘格桌面高度，单位：毫米）
            """
            self.get_logger().info(f'   [Custom模式] 开始算法计算（XY+Z约束方法）')
            self.get_logger().info(f'   输入数据：{len(pick_poses)} 个数据点')
            
            # 调用XY+Z约束方法（完全参考hand_eye_calibration.py）
            T_camera2gripper, fixed_z = _solve_eye_in_hand(shot_pose, pick_poses, camera_points_shot)
            
            self.get_logger().info(f'   [Custom模式] 算法计算完成，固定Z值: {fixed_z:.6f} mm')
            
            return T_camera2gripper, fixed_z
        
        def _calculate_custom_errors(shot_pose, pick_poses, camera_points_shot, T_camera2gripper):
            """
            Custom模式：计算标定误差（XY+Z约束的误差）
            
            参数:
                shot_pose: 4x4变换矩阵，拍照姿态（机器人基座坐标系，单位：毫米）
                pick_poses: 列表，每个元素是4x4变换矩阵，点选姿态（机器人基座坐标系，单位：毫米）
                camera_points_shot: 列表，每个元素是3x1数组，角点在相机坐标系下的坐标（单位：毫米）
                T_camera2gripper: 4x4变换矩阵，相机→末端执行器（单位：毫米）
            
            返回:
                errors: 列表，每个数据点的误差（单位：毫米）
                error_statistics: 字典，包含RMS、最大、最小、标准差等统计信息
            """
            self.get_logger().info(f'   [Custom模式] 开始误差计算：验证XY+Z约束')
            
            errors = []
            error_details = []
            
            for i in range(len(pick_poses)):
                P_camera_shot = camera_points_shot[i]
                T_gripper_pick2base = pick_poses[i]
                
                # 计算角点在基座坐标系下的位置
                P_camera_shot_homogeneous = np.append(P_camera_shot, 1.0)
                P_gripper_homogeneous = T_camera2gripper @ P_camera_shot_homogeneous
                P_base_computed_homogeneous = shot_pose @ P_gripper_homogeneous
                P_base_computed = P_base_computed_homogeneous[:3]
                
                # 实际位置（点选姿态的末端执行器位置）
                P_base_actual = T_gripper_pick2base[:3, 3]
                
                # 计算误差
                error = np.linalg.norm(P_base_computed - P_base_actual)
                errors.append(error)
                
                error_details.append({
                    'data_point_idx': i + 1,
                    'error_mm': float(error),
                    'computed_position': P_base_computed.tolist(),
                    'actual_position': P_base_actual.tolist(),
                    'camera_point': P_camera_shot.tolist()
                })
            
            # 误差统计
            mean_error = float(np.mean(errors))
            max_error = float(np.max(errors))
            min_error = float(np.min(errors))
            std_error = float(np.std(errors))
            
            error_statistics = {
                'mean_error': mean_error,
                'max_error': max_error,
                'min_error': min_error,
                'std_error': std_error,
                'errors': [float(e) for e in errors],
                'error_details': error_details
            }
            
            self.get_logger().info(f'   [Custom模式] 误差计算完成：平均 {mean_error:.3f} mm, 最大 {max_error:.3f} mm, 最小 {min_error:.3f} mm, 标准差 {std_error:.3f} mm')
            
            return errors, error_statistics
        
        def _solve_pose_based_hand_eye(A_list, B_list):
            """
            求解AX=XB问题（姿态法手眼标定）
            使用类似Tsai的方法，基于SVD求解，然后非线性优化
            
            参数:
                A_list: 机器人基座坐标系下的运动列表 (N个4x4矩阵)
                B_list: 相机坐标系下的运动列表 (N个4x4矩阵)
            
            返回:
                T_camera2gripper: 4x4变换矩阵，相机→末端执行器
            """
            n = len(A_list)
            if n < 2:
                raise ValueError("至少需要2组运动数据")
            
            # 辅助函数：兼容不同版本的scipy Rotation.as_matrix()
            def rotation_to_matrix(r):
                """将Rotation对象转换为矩阵（兼容不同版本scipy）"""
                if hasattr(r, 'as_matrix'):
                    return r.as_matrix()
                elif hasattr(r, 'as_dcm'):
                    return r.as_dcm()
                else:
                    # 手动转换（使用Rodrigues公式）
                    rotvec = r.as_rotvec()
                    angle = np.linalg.norm(rotvec)
                    if angle < 1e-6:
                        return np.eye(3)
                    axis = rotvec / angle
                    K = np.array([[0, -axis[2], axis[1]],
                                  [axis[2], 0, -axis[0]],
                                  [-axis[1], axis[0], 0]])
                    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
            
            # 提取旋转和平移
            Ra_list = [A[:3, :3] for A in A_list]
            ta_list = [A[:3, 3] for A in A_list]
            Rb_list = [B[:3, :3] for B in B_list]
            tb_list = [B[:3, 3] for B in B_list]
            
            # 步骤1：求解旋转部分（先使用SVD估计初始值，然后非线性优化）
            # 使用Kronecker积方法构建线性系统求解初始旋转估计
            # 从约束 Ra @ Rx = Rx @ Rb，可以构建线性系统求解Rx
            
            self.get_logger().info(f'   步骤1: 求解旋转部分（SVD初始估计）...')
            
            # 方法1：使用Kronecker积构建线性系统（适用于多组数据）
            # 对于 Ra @ Rx = Rx @ Rb，可以写成：
            # (Ra ⊗ I - I ⊗ Rb^T) @ vec(Rx) = 0
            # 其中 vec(Rx) 是Rx的列向量堆叠
            
            try:
                # 构建线性系统（优化：使用列表推导式加速）
                # 对于每个约束 Ra_i @ Rx = Rx @ Rb_i
                # 使用Kronecker积: (Ra_i ⊗ I - I ⊗ Rb_i^T) @ vec(Rx) = 0
                A_kron = np.vstack([
                    np.kron(Ra_list[i], np.eye(3)) - np.kron(np.eye(3), Rb_list[i].T)
                    for i in range(n)
                ])  # (9n x 9) 矩阵
                
                # SVD分解，最小奇异值对应的右奇异向量就是vec(Rx)的估计
                # 使用full_matrices=False加速计算（只计算需要的奇异向量）
                U, S, Vt = np.linalg.svd(A_kron, full_matrices=False)
                Rx_vec = Vt[-1, :].reshape(3, 3)  # 取最后一个右奇异向量
                
                # 投影到SO(3)（正交化，优化：使用full_matrices=False）
                U_proj, _, Vt_proj = np.linalg.svd(Rx_vec, full_matrices=False)
                Rx_init = U_proj @ Vt_proj.T
                
                # 确保行列式为+1（右手坐标系）
                if np.linalg.det(Rx_init) < 0:
                    U_proj[:, -1] *= -1
                    Rx_init = U_proj @ Vt_proj.T
                
                # 转换为轴角表示作为初始值
                import cv2
                axis_angle_init, _ = cv2.Rodrigues(Rx_init)
                initial_rx = axis_angle_init.flatten()
                
                angle_init = np.linalg.norm(initial_rx)
                self.get_logger().info(f'   SVD初始估计完成，旋转角度: {np.degrees(angle_init):.3f} deg')
                
            except Exception as e:
                self.get_logger().warning(f'   SVD初始估计失败: {e}，使用零初始值')
                initial_rx = np.array([0.0, 0.0, 0.0])
            
            # 非线性优化进一步优化旋转
            def rotation_residual(rx_params):
                """旋转残差函数（优化：使用numpy数组操作）"""
                angle = np.linalg.norm(rx_params)
                if angle < 1e-6:
                    Rx = np.eye(3)
                else:
                    axis = rx_params / angle
                    r = R.from_rotvec(axis * angle)
                    Rx = rotation_to_matrix(r)
                
                # 使用列表推导式加速计算
                residual = np.concatenate([
                    (Ra_list[i] @ Rx - Rx @ Rb_list[i]).flatten()
                    for i in range(n)
                ])
                return residual
            
            self.get_logger().info(f'   步骤1b: 非线性优化旋转...')
            result_rx = least_squares(rotation_residual, initial_rx, method='lm', 
                                     max_nfev=200, ftol=1e-6, xtol=1e-6, verbose=0)
            
            # 提取旋转矩阵
            rx_params = result_rx.x
            angle = np.linalg.norm(rx_params)
            if angle < 1e-6:
                Rx = np.eye(3)
            else:
                axis = rx_params / angle
                r = R.from_rotvec(axis * angle)
                Rx = rotation_to_matrix(r)
            
            self.get_logger().info(f'   初始旋转求解完成，旋转角度: {np.degrees(angle):.3f} deg')
            
            # 步骤2：求解平移部分（线性系统）
            # 从 Ra @ tx + ta = Rx @ tb + tx 推导：
            # (I - Ra) @ tx = ta - Rx @ tb
            # 优化：使用列表推导式加速矩阵构建
            A_mat = np.vstack([np.eye(3) - Ra_list[i] for i in range(n)])
            b_vec = np.hstack([ta_list[i] - Rx @ tb_list[i] for i in range(n)])
            
            tx, residuals, rank, s = np.linalg.lstsq(A_mat, b_vec, rcond=None)
            
            # 检查线性系统求解的质量
            tx_magnitude = np.linalg.norm(tx)
            self.get_logger().info(f'   步骤2: 平移部分求解完成')
            if rank < 3:
                self.get_logger().warning(f'   ⚠️ 线性系统秩不足（rank={rank}），可能数据质量差')
            
            # 步骤3：非线性优化（进一步优化结果）
            def full_residual(params):
                """完整约束的残差函数（优化：使用numpy数组操作）"""
                axis_angle = params[:3]
                translation = params[3:6]
                
                angle = np.linalg.norm(axis_angle)
                if angle < 1e-6:
                    R_mat = np.eye(3)
                else:
                    axis = axis_angle / angle
                    r = R.from_rotvec(axis * angle)
                    R_mat = rotation_to_matrix(r)
                
                T = np.eye(4)
                T[:3, :3] = R_mat
                T[:3, 3] = translation
                
                # 使用列表推导式加速计算
                residual = np.concatenate([
                    (A_list[i] @ T - T @ B_list[i]).flatten()
                    for i in range(n)
                ])
                
                return residual
            
            # 使用安全的转换函数
            quat_init = self._rotation_matrix_to_quaternion(Rx)
            # 四元数转轴角
            from scipy.spatial.transform import Rotation as R_scipy
            try:
                if hasattr(R_scipy, 'from_quat'):
                    r_init = R_scipy.from_quat(quat_init)
                    axis_angle_init = r_init.as_rotvec()
                else:
                    # 手动转换四元数到轴角
                    w = quat_init[3]
                    if abs(w) >= 1.0:
                        axis_angle_init = np.array([0.0, 0.0, 0.0])
                    else:
                        angle = 2 * np.arccos(abs(w))
                        if w < 0:
                            axis = -quat_init[:3] / np.sin(angle / 2)
                        else:
                            axis = quat_init[:3] / np.sin(angle / 2)
                        axis_angle_init = axis * angle
            except Exception as e:
                self.get_logger().error(f'四元数转轴角失败: {e}')
                # 使用Rodrigues公式的逆变换
                import cv2
                axis_angle_init, _ = cv2.Rodrigues(Rx)
                axis_angle_init = axis_angle_init.flatten()
            initial_params = np.concatenate([axis_angle_init, tx])
            
            self.get_logger().info(f'   步骤3: 非线性优化（进一步优化）...')
            result_full = least_squares(full_residual, initial_params, method='lm',
                                       max_nfev=500, ftol=1e-6, xtol=1e-6, verbose=0)
            
            # 检查优化是否成功
            if not result_full.success:
                self.get_logger().warning(f'   优化未成功收敛: {result_full.message}')
            
            # 提取优化后的结果
            axis_angle_opt = result_full.x[:3]
            translation_opt = result_full.x[3:6]
            
            angle_opt = np.linalg.norm(axis_angle_opt)
            if angle_opt < 1e-6:
                R_opt = np.eye(3)
            else:
                axis_opt = axis_angle_opt / angle_opt
                r_opt = R.from_rotvec(axis_opt * angle_opt)
                R_opt = rotation_to_matrix(r_opt)
            
            # 检查平移向量是否过大（可能表示数据质量差）
            translation_magnitude = np.linalg.norm(translation_opt)
            tx_magnitude_step2 = np.linalg.norm(tx)
            
            # 计算步骤2和步骤3的残差（优化：使用numpy数组操作）
            T_step2 = np.eye(4)
            T_step2[:3, :3] = Rx
            T_step2[:3, 3] = tx
            # 使用列表推导式加速计算
            residual_step2 = np.concatenate([
                (A_list[i] @ T_step2 - T_step2 @ B_list[i]).flatten()
                for i in range(n)
            ])
            mean_residual_step2 = np.mean(np.abs(residual_step2))
            
            mean_residual_step3 = np.mean(np.abs(result_full.fun))
            
            # 智能选择：如果步骤3的平移向量过大，且残差改善不明显，使用步骤2的结果
            # 相机到末端执行器的距离应该在合理范围内（10-1000mm）
            REASONABLE_TRANSLATION_MAX = 1000.0  # 1米，相机到末端执行器的最大合理距离
            REASONABLE_TRANSLATION_MIN = 10.0     # 1厘米，相机到末端执行器的最小合理距离
            
            use_step2_result = False
            translation_reasonable = (REASONABLE_TRANSLATION_MIN <= translation_magnitude <= REASONABLE_TRANSLATION_MAX)
            tx_step2_reasonable = (REASONABLE_TRANSLATION_MIN <= tx_magnitude_step2 <= REASONABLE_TRANSLATION_MAX)
            
            if not translation_reasonable:
                self.get_logger().warning(f'   平移向量模长超出合理范围: {translation_magnitude:.3f} mm（合理范围: {REASONABLE_TRANSLATION_MIN:.0f}-{REASONABLE_TRANSLATION_MAX:.0f} mm）')
                # 如果步骤2的平移向量在合理范围内，优先使用步骤2的结果
                if tx_step2_reasonable:
                    self.get_logger().warning(f'   使用步骤2的结果（平移向量在合理范围内）')
                    use_step2_result = True
                else:
                    self.get_logger().warning(f'   步骤2平移向量也不合理，使用步骤2的结果')
                    use_step2_result = True
            elif translation_magnitude > 500.0:  # 500mm = 0.5m，虽然合理但较大
                # 如果步骤3的残差没有显著改善，且平移向量较大，使用步骤2的结果
                if mean_residual_step3 > mean_residual_step2 * 0.9:  # 改善小于10%
                    self.get_logger().info(f'   步骤3残差改善不明显，使用步骤2的结果')
                    use_step2_result = True
                else:
                    improvement = (mean_residual_step2 - mean_residual_step3) / mean_residual_step2 * 100
                    self.get_logger().info(f'   步骤3残差改善: {improvement:.1f}%，使用步骤3结果')
            
            if use_step2_result:
                # 使用步骤2的结果（线性解）
                T_camera2gripper = T_step2
                mean_residual = mean_residual_step2
            else:
                # 使用步骤3的结果（优化解）
                T_camera2gripper = np.eye(4)
                T_camera2gripper[:3, :3] = R_opt
                T_camera2gripper[:3, 3] = translation_opt
                mean_residual = mean_residual_step3
            
            self.get_logger().info(f'   优化完成，平均残差: {mean_residual:.6f} mm')
            
            return T_camera2gripper
        
        @self.app.route('/api/hand_eye/clear_collect_data', methods=['POST'])
        def clear_collect_data():
            """清空collect_data目录中的图像和位姿文件"""
            try:
                collect_data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
                
                if not os.path.exists(collect_data_dir):
                    os.makedirs(collect_data_dir, exist_ok=True)
                    self.get_logger().info(f'✅ 创建目录: {collect_data_dir}')
                    return jsonify({'success': True, 'message': '目录已创建'})
                
                # 删除所有图像文件
                deleted_images = 0
                for filename in os.listdir(collect_data_dir):
                    if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                        file_path = os.path.join(collect_data_dir, filename)
                        try:
                            os.remove(file_path)
                            deleted_images += 1
                        except Exception as e:
                            self.get_logger().warn(f'删除图像文件失败 {filename}: {str(e)}')
                
                # 删除poses.txt文件
                poses_file = os.path.join(collect_data_dir, 'poses.txt')
                if os.path.exists(poses_file):
                    try:
                        os.remove(poses_file)
                        self.get_logger().info(f'✅ 已删除 poses.txt')
                    except Exception as e:
                        self.get_logger().warn(f'删除poses.txt失败: {str(e)}')
                
                self.get_logger().info(f'✅ 清空collect_data目录完成，删除了{deleted_images}个图像文件')
                return jsonify({
                    'success': True,
                    'message': f'已清空collect_data目录，删除了{deleted_images}个图像文件'
                })
            except Exception as e:
                self.get_logger().error(f'清空collect_data目录失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/hand_eye/save_image_and_pose', methods=['POST'])
        def save_image_and_pose():
            """保存图像和位姿到collect_data目录，每个姿态保存到独立的子文件夹"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({'success': False, 'error': '请求数据为空'})
                
                pose_index = data.get('pose_index', 0)
                robot_pose = data.get('robot_pose')
                square_size = data.get('square_size', 20.0)  # 默认20mm（与其他函数一致）
                
                if robot_pose is None:
                    return jsonify({'success': False, 'error': '机器人位姿数据缺失'})
                
                collect_data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
                pose_dir = os.path.join(collect_data_dir, f'pose_{pose_index}')
                os.makedirs(pose_dir, exist_ok=True)
                
                # 检查图像数据
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '当前没有图像数据'})
                
                if self.calib_utils.camera_matrix is None:
                    return jsonify({'success': False, 'error': '请先加载相机参数'})
                
                self.get_logger().info(f'='*60)
                self.get_logger().info(f'💾 开始保存姿态 #{pose_index} 的数据到 {pose_dir}')
                
                # 1. 保存机器人第六轴姿态信息（JSON文件）
                robot_pose_data = {
                    'pose_index': pose_index,
                    'timestamp': time.time()
                }
                
                # 优先使用current_robot_status（包含完整信息）
                if self.current_robot_status is not None:
                    robot_pose_data['joint_position'] = {
                        'rad': list(self.current_robot_status.joint_position_rad),
                        'deg': list(self.current_robot_status.joint_position_deg)
                    }
                    robot_pose_data['cartesian_position'] = {
                        'position': {
                            'x': float(self.current_robot_status.cartesian_position.position.x),
                            'y': float(self.current_robot_status.cartesian_position.position.y),
                            'z': float(self.current_robot_status.cartesian_position.position.z)
                        },
                        'orientation': {
                            'x': float(self.current_robot_status.cartesian_position.orientation.x),
                            'y': float(self.current_robot_status.cartesian_position.orientation.y),
                            'z': float(self.current_robot_status.cartesian_position.orientation.z),
                            'w': float(self.current_robot_status.cartesian_position.orientation.w)
                        }
                    }
                else:
                    # 使用传入的robot_pose参数
                    robot_pose_data['cartesian_position'] = {
                        'position': {
                            'x': float(robot_pose['position']['x']),
                            'y': float(robot_pose['position']['y']),
                            'z': float(robot_pose['position']['z'])
                        },
                        'orientation': robot_pose['orientation']
                    }
                    robot_pose_data['joint_position'] = None  # 关节坐标不可用
                
                robot_pose_file = os.path.join(pose_dir, 'robot_pose.json')
                with open(robot_pose_file, 'w', encoding='utf-8') as f:
                    json.dump(robot_pose_data, f, indent=2, ensure_ascii=False)
                self.get_logger().info(f'✅ 已保存机器人姿态信息到: {robot_pose_file}')
                
                # 2. 转换为灰度图并保存（使用灰度图进行标定）
                if len(self.current_image_raw.shape) == 3:
                    gray_image = cv2.cvtColor(self.current_image_raw, cv2.COLOR_BGR2GRAY)
                else:
                    gray_image = self.current_image_raw
                
                gray_image_file = os.path.join(pose_dir, 'image.jpg')
                cv2.imwrite(gray_image_file, gray_image)
                self.get_logger().info(f'✅ 已保存灰度图像到: {gray_image_file}')
                
                # 同时保存RGB图像作为备份（可选）
                rgb_image_file = os.path.join(pose_dir, 'rgb.jpg')
                cv2.imwrite(rgb_image_file, self.current_image_raw)
                self.get_logger().info(f'✅ 已保存RGB图像备份到: {rgb_image_file}')
                
                # 3. 保存depth图像（如果存在）
                depth_image_file = os.path.join(pose_dir, 'depth.png')
                if self.current_depth_image is not None:
                    # 深度图保存为16位PNG（保持原始精度）
                    cv2.imwrite(depth_image_file, self.current_depth_image)
                    self.get_logger().info(f'✅ 已保存depth图像到: {depth_image_file}')
                else:
                    self.get_logger().warn(f'⚠️ 深度图像不可用，跳过保存 depth.png')
                
                # 4. 检测角点并绘制坐标轴，保存可视化图
                # 使用灰度图进行角点检测（提高检测精度和速度）
                pattern_w, pattern_h = self.pattern_size
                corners = self.calib_utils.detect_checkerboard_corners(
                    gray_image, self.pattern_size
                )
                
                # 如果使用配置尺寸检测失败，尝试推断实际尺寸
                actual_pattern_size = None
                if corners is None:
                    self.get_logger().warn(f'⚠️ 使用配置尺寸{pattern_w}x{pattern_h}未检测到角点，尝试其他尺寸...')
                    common_sizes = [(11, 8), (8, 11), (9, 6), (6, 9), (8, 6), (6, 8), (7, 5), (5, 7)]
                    for test_size in common_sizes:
                        test_corners = self.calib_utils.detect_checkerboard_corners(
                            gray_image, test_size
                        )
                        if test_corners is not None and len(test_corners) == test_size[0] * test_size[1]:
                            corners = test_corners
                            actual_pattern_size = test_size
                            break
                
                if corners is None:
                    return jsonify({'success': False, 'error': '未检测到棋盘格角点，请确保棋盘格完整可见'})
                
                # 确定实际使用的尺寸
                if actual_pattern_size is None:
                    detected_corners = len(corners)
                    expected_corners = pattern_w * pattern_h
                    if detected_corners == expected_corners:
                        actual_pattern_size = (pattern_w, pattern_h)
                    else:
                        # 角点数量不匹配，尝试推断
                        possible_sizes = []
                        for w in range(2, 20):
                            for h in range(2, 20):
                                if w * h == detected_corners:
                                    possible_sizes.append((w, h))
                        if possible_sizes:
                            possible_sizes.sort(key=lambda x: abs(x[0] - pattern_w) + abs(x[1] - pattern_h))
                            actual_pattern_size = possible_sizes[0]
                            corners = self.calib_utils.detect_checkerboard_corners(
                                gray_image, actual_pattern_size
                            )
                            if corners is None:
                                return jsonify({'success': False, 'error': f'使用推断尺寸{actual_pattern_size[0]}x{actual_pattern_size[1]}重新检测失败'})
                        else:
                            return jsonify({'success': False, 'error': f'无法推断棋盘格尺寸：检测到{detected_corners}个角点'})
                
                actual_w, actual_h = actual_pattern_size
                self.get_logger().info(f'📐 检测到棋盘格尺寸：{actual_w}x{actual_h}，角点数：{len(corners)}')
                
                # 使用solvePnP计算棋盘格姿态
                obj_points_all = []
                for j in range(actual_h):
                    for i in range(actual_w):
                        obj_points_all.append([i * square_size, j * square_size, 0.0])
                obj_points_all = np.array(obj_points_all, dtype=np.float32)
                corners_2d_all = corners.reshape(-1, 1, 2).astype(np.float32)
                
                dist_coeffs_to_use = None if self._image_is_undistorted else self.calib_utils.dist_coeffs
                success, rvec, tvec = cv2.solvePnP(
                    obj_points_all,
                    corners_2d_all,
                    self.calib_utils.camera_matrix,
                    dist_coeffs_to_use
                )
                
                if not success:
                    return jsonify({'success': False, 'error': 'solvePnP求解失败'})
                
                R_board2cam, _ = cv2.Rodrigues(rvec)
                
                # 绘制角点和坐标轴（将灰度图转换为BGR以便绘制彩色线条）
                gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                img_with_features = self.calib_utils.draw_corners(
                    gray_bgr, corners, actual_pattern_size
                )
                
                # 绘制坐标轴（标定板坐标系）
                axis_length = 50.0  # mm
                axis_points_3d = np.array([
                    [0, 0, 0],           # 原点
                    [axis_length, 0, 0], # X轴
                    [0, axis_length, 0], # Y轴
                    [0, 0, -axis_length]  # Z轴
                ], dtype=np.float32)
                
                axis_points_2d, _ = cv2.projectPoints(
                    axis_points_3d,
                    rvec,
                    tvec,
                    self.calib_utils.camera_matrix,
                    self.calib_utils.dist_coeffs
                )
                axis_points_2d = axis_points_2d.reshape(-1, 2).astype(int)
                
                origin = tuple(axis_points_2d[0])
                cv2.line(img_with_features, origin, tuple(axis_points_2d[1]), (0, 0, 255), 3)  # X轴：红色
                cv2.line(img_with_features, origin, tuple(axis_points_2d[2]), (0, 255, 0), 3)  # Y轴：绿色
                cv2.line(img_with_features, origin, tuple(axis_points_2d[3]), (255, 0, 0), 3)  # Z轴：蓝色
                
                corners_visualization_file = os.path.join(pose_dir, 'corners_visualization.jpg')
                cv2.imwrite(corners_visualization_file, img_with_features)
                self.get_logger().info(f'✅ 已保存角点可视化图到: {corners_visualization_file}')
                
                # 5. 保存角点信息表（CSV）
                corners_flat = corners.reshape(-1, 2)
                corners_csv_file = os.path.join(pose_dir, 'corners.csv')
                import csv
                with open(corners_csv_file, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(['角点序号', 'RGB像素坐标U', 'RGB像素坐标V', 'Depth像素坐标U', 'Depth像素坐标V', 'Depth值(mm)'])
                    
                    for idx in range(len(corners_flat)):
                        rgb_u = float(corners_flat[idx, 0])
                        rgb_v = float(corners_flat[idx, 1])
                        depth_u = rgb_u  # 深度图已对齐到彩色图
                        depth_v = rgb_v
                        
                        # 获取深度值
                        depth_value = None
                        if self.current_depth_image is not None:
                            px_u = int(round(rgb_u))
                            px_v = int(round(rgb_v))
                            depth_value = self.calib_utils.get_depth_from_depth_image(
                                self.current_depth_image, px_u, px_v, search_radius=3
                            )
                        
                        writer.writerow([
                            idx,
                            f'{rgb_u:.2f}',
                            f'{rgb_v:.2f}',
                            f'{depth_u:.2f}',
                            f'{depth_v:.2f}',
                            f'{depth_value:.2f}' if depth_value is not None else 'N/A'
                        ])
                self.get_logger().info(f'✅ 已保存角点信息表到: {corners_csv_file}')
                
                # 6. 保存棋盘格在相机坐标系下的姿态（JSON文件）
                board_pose_data = {
                    'pose_index': pose_index,
                    'pattern_size': {
                        'width': actual_w,
                        'height': actual_h
                    },
                    'square_size_mm': square_size,
                    'rvec': {
                        'x': float(rvec[0, 0]),
                        'y': float(rvec[1, 0]),
                        'z': float(rvec[2, 0])
                    },
                    'tvec': {
                        'x': float(tvec[0, 0]),
                        'y': float(tvec[1, 0]),
                        'z': float(tvec[2, 0])
                    },
                    'rotation_matrix': R_board2cam.tolist(),
                    'transformation_matrix': np.eye(4).tolist()
                }
                
                # 构建变换矩阵
                T_board2cam = np.eye(4)
                T_board2cam[:3, :3] = R_board2cam
                T_board2cam[:3, 3] = tvec.flatten()
                board_pose_data['transformation_matrix'] = T_board2cam.tolist()
                
                # 转换为四元数
                quat = self._rotation_matrix_to_quaternion(R_board2cam)
                board_pose_data['orientation'] = {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                }
                
                board_pose_file = os.path.join(pose_dir, 'board_pose.json')
                with open(board_pose_file, 'w', encoding='utf-8') as f:
                    json.dump(board_pose_data, f, indent=2, ensure_ascii=False)
                self.get_logger().info(f'✅ 已保存棋盘格姿态信息到: {board_pose_file}')
                
                # 兼容性：同时保存到poses.txt（追加模式）
                poses_file = os.path.join(collect_data_dir, 'poses.txt')
                quat_robot = [
                    robot_pose_data['cartesian_position']['orientation']['w'],
                    robot_pose_data['cartesian_position']['orientation']['x'],
                    robot_pose_data['cartesian_position']['orientation']['y'],
                    robot_pose_data['cartesian_position']['orientation']['z']
                ]
                rotation = R.from_quat(quat_robot)
                euler = rotation.as_euler('xyz', degrees=False)
                pose_str = f"{robot_pose_data['cartesian_position']['position']['x']},{robot_pose_data['cartesian_position']['position']['y']},{robot_pose_data['cartesian_position']['position']['z']},{euler[0]},{euler[1]},{euler[2]}\n"
                with open(poses_file, 'a', encoding='utf-8') as f:
                    f.write(pose_str)
                
                self.get_logger().info(f'✅ 已保存位姿 #{pose_index} 到: {poses_file}（兼容性）')
                self.get_logger().info(f'='*60)
                self.get_logger().info(f'✅ 姿态 #{pose_index} 的所有数据已成功保存到 {pose_dir}')
                self.get_logger().info(f'='*60)
                
                return jsonify({
                    'success': True,
                    'message': f'已保存姿态 #{pose_index} 的所有数据',
                    'pose_dir': pose_dir,
                    'files': {
                        'robot_pose': robot_pose_file,
                        'rgb_image': rgb_image_file,
                        'depth_image': depth_image_file if self.current_depth_image is not None else None,
                        'corners_visualization': corners_visualization_file,
                        'corners_csv': corners_csv_file,
                        'board_pose': board_pose_file
                    }
                })
            except Exception as e:
                self.get_logger().error(f'保存图像和位姿失败: {str(e)}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/hand_eye/save_calibration', methods=['POST'])
        def save_hand_eye_calibration():
            """生成手眼标定结果（XML或YAML格式）并保存到服务器"""
            try:
                from datetime import datetime
                import yaml
                data = request.get_json()
                
                # 获取文件格式（默认为yaml）
                file_format = data.get('format', 'yaml').lower()
                if file_format not in ['xml', 'yaml']:
                    file_format = 'yaml'
                
                # 检查是否有标定结果
                if self.hand_eye_calibration_data.get('transformation_matrix') is None:
                    return jsonify({
                        'success': False,
                        'error': '没有可保存的标定结果，请先完成标定'
                    })
                
                self.get_logger().info('='*60)
                self.get_logger().info(f'📥 生成手眼标定结果{file_format.upper()}文件')
                
                # 合并前端数据和实际标定结果数据
                # 优先使用 self.hand_eye_calibration_data 中的实际标定结果
                calibration_data = data.copy() if data else {}
                
                # 从实际标定结果中获取数据
                if self.hand_eye_calibration_data.get('transformation_matrix'):
                    # 转换列表为numpy数组
                    T_matrix_list = self.hand_eye_calibration_data['transformation_matrix']
                    calibration_data['transformation_matrix'] = np.array(T_matrix_list).tolist()
                
                # 获取评估数据
                if self.hand_eye_calibration_data.get('calibration_error'):
                    error_data = self.hand_eye_calibration_data['calibration_error']
                    calibration_data['evaluation'] = {
                        'mean_error': error_data.get('mean', 0.0),
                        'max_error': error_data.get('max', 0.0),
                        'min_error': error_data.get('min', 0.0),
                        'std_error': error_data.get('std', 0.0),
                        'data_count': len(self.hand_eye_calibration_data.get('captured_poses', []))
                    }
                
                # 构建文件内容
                if file_format == 'yaml':
                    file_content = self._build_calibration_yaml(calibration_data)
                    file_extension = 'yaml'
                else:
                    file_content = self._build_calibration_xml(calibration_data)
                    file_extension = 'xml'
                
                # 保存到服务器
                try:
                    # 获取源码目录下的config/calibration_results文件夹
                    config_dir = self._get_config_dir()
                    calibration_results_dir = os.path.join(config_dir, 'calibration_results')
                    os.makedirs(calibration_results_dir, exist_ok=True)
                    
                    # 生成文件名（带时间戳）
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    filename = f'hand_eye_calibration_{timestamp}.{file_extension}'
                    filepath = os.path.join(calibration_results_dir, filename)
                    
                    # 保存文件
                    with open(filepath, 'w', encoding='utf-8') as f:
                        f.write(file_content)
                    
                    self.get_logger().info(f'✅ 标定结果已保存到服务器: {filepath}')
                    
                    result = {
                        'success': True,
                        'filename': filename,
                        'filepath': filepath,
                        'format': file_format,
                        'message': f'标定结果已保存: {filename}'
                    }
                    
                    # 根据格式返回相应内容
                    if file_format == 'yaml':
                        result['yaml_content'] = file_content
                    else:
                        result['xml_content'] = file_content
                    
                    return jsonify(result)
                except Exception as save_error:
                    # 如果保存到服务器失败，仍然返回内容供下载
                    self.get_logger().warn(f'⚠️ 保存到服务器失败: {str(save_error)}')
                    self.get_logger().info(f'   回退到仅返回内容供下载')
                    
                    result = {
                        'success': True,
                        'format': file_format,
                        'message': f'标定结果{file_format.upper()}已生成（未保存到服务器）'
                    }
                    
                    if file_format == 'yaml':
                        result['yaml_content'] = file_content
                    else:
                        result['xml_content'] = file_content
                    
                    return jsonify(result)
                
            except Exception as e:
                self.get_logger().error(f'❌ 生成标定结果失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/hand_eye/load_calibration_params', methods=['POST'])
        def load_hand_eye_calibration_params():
            """加载手眼标定参数文件"""
            try:
                # 检查是否有上传的文件
                if 'file' in request.files:
                    file = request.files['file']
                    if file.filename == '':
                        return jsonify({'success': False, 'error': '未选择文件'})
                    
                    # 保存临时文件
                    import tempfile
                    temp_dir = tempfile.gettempdir()
                    temp_path = os.path.join(temp_dir, 'hand_eye_calibration_uploaded.xml')
                    file.save(temp_path)
                    config_path = temp_path
                    self.get_logger().info(f'📁 加载上传的手眼标定参数文件: {file.filename}')
                else:
                    # 使用默认路径
                    config_path = os.path.join(
                        get_package_share_directory('hand_eye_calibration'),
                        'config', 'hand_eye_calibration.xml'
                    )
                    
                    # 如果找不到，尝试源码路径
                    if not os.path.exists(config_path):
                        config_path = os.path.join(
                            os.path.dirname(__file__), '..', 'config', 'hand_eye_calibration.xml'
                        )
                    
                    if os.path.exists(config_path):
                        self.get_logger().info(f'📁 加载默认手眼标定参数文件: {config_path}')
                    else:
                        return jsonify({'success': False, 'error': '未找到默认手眼标定文件，请上传标定文件'})
                
                result = self._load_hand_eye_calibration_xml(config_path)
                return jsonify(result)
            except Exception as e:
                self.get_logger().error(f'❌ 加载手眼标定参数失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/hand_eye/verify', methods=['POST'])
        def verify_hand_eye_calibration():
            """手眼标定精度验证"""
            # TODO: 实现手眼标定验证功能
            return jsonify({'success': True, 'message': '功能待实现'})
        
        @self.app.route('/api/poses/save', methods=['POST'])
        def save_poses_file():
            """保存位姿文件到服务器"""
            try:
                from datetime import datetime
                data = request.get_json()
                if not data:
                    return jsonify({'success': False, 'error': '缺少数据'})
                
                # 检查数据格式（支持recordedPoses字段的v4.0格式）
                if 'recordedPoses' not in data:
                    return jsonify({'success': False, 'error': '缺少recordedPoses字段'})
                
                # 获取源码目录下的config/poses文件夹
                config_dir = self._get_config_dir()
                poses_dir = os.path.join(config_dir, 'poses')
                os.makedirs(poses_dir, exist_ok=True)
                
                # 生成文件名（带时间戳）
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f'auto_hand_eye_poses_{timestamp}.json'
                filepath = os.path.join(poses_dir, filename)
                
                # 保存文件
                with open(filepath, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                
                self.get_logger().info(f'✅ 位姿文件已保存: {filepath}')
                return jsonify({
                    'success': True,
                    'filename': filename,
                    'filepath': filepath,
                    'message': f'位姿文件已保存: {filename}'
                })
            except Exception as e:
                self.get_logger().error(f'❌ 保存位姿文件失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/poses/list', methods=['GET'])
        def list_poses_files():
            """列出服务器上的位姿文件"""
            try:
                from datetime import datetime
                # 获取源码目录下的config/poses文件夹
                config_dir = self._get_config_dir()
                poses_dir = os.path.join(config_dir, 'poses')
                os.makedirs(poses_dir, exist_ok=True)
                
                # 列出所有JSON文件
                files = []
                if os.path.exists(poses_dir):
                    for filename in os.listdir(poses_dir):
                        if filename.endswith('.json') and filename.startswith('auto_hand_eye_poses_'):
                            filepath = os.path.join(poses_dir, filename)
                            try:
                                stat = os.stat(filepath)
                                files.append({
                                    'filename': filename,
                                    'size': stat.st_size,
                                    'modified': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
                                })
                            except:
                                pass
                
                # 按修改时间倒序排列（最新的在前）
                files.sort(key=lambda x: x['modified'], reverse=True)
                
                return jsonify({
                    'success': True,
                    'files': files,
                    'count': len(files)
                })
            except Exception as e:
                self.get_logger().error(f'❌ 列出位姿文件失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/poses/load', methods=['POST'])
        def load_poses_file():
            """从服务器加载位姿文件"""
            try:
                data = request.get_json()
                filename = data.get('filename')
                
                if not filename:
                    return jsonify({'success': False, 'error': '缺少文件名'})
                
                # 获取源码目录下的config/poses文件夹
                config_dir = self._get_config_dir()
                poses_dir = os.path.join(config_dir, 'poses')
                filepath = os.path.join(poses_dir, filename)
                
                # 安全检查：确保文件在poses目录内
                if not os.path.abspath(filepath).startswith(os.path.abspath(poses_dir)):
                    return jsonify({'success': False, 'error': '无效的文件路径'})
                
                if not os.path.exists(filepath):
                    return jsonify({'success': False, 'error': f'文件不存在: {filename}'})
                
                # 读取文件
                with open(filepath, 'r', encoding='utf-8') as f:
                    poses_data = json.load(f)
                
                self.get_logger().info(f'✅ 位姿文件已加载: {filename}')
                return jsonify({
                    'success': True,
                    'filename': filename,
                    'data': poses_data
                })
            except Exception as e:
                self.get_logger().error(f'❌ 加载位姿文件失败: {str(e)}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/robot/set_pose', methods=['POST'])
        def set_robot_pose():
            """设置机器人位姿 - 调用/set_robot_pose服务"""
            try:
                data = request.get_json()
                target_pose = data.get('target_pose')
                use_joints = data.get('use_joints', False)
                is_radian = data.get('is_radian', True)  # 默认使用弧度
                velocity = float(data.get('velocity', 50.0))
                
                if not target_pose:
                    return jsonify({'success': False, 'error': '缺少目标位姿数据'})
                
                # 创建服务请求
                from demo_interface.srv import SetRobotPose
                
                request_msg = SetRobotPose.Request()
                
                # 检查输入格式：如果是数组格式 [x, y, z, a, b, c]，直接使用
                if isinstance(target_pose, list) and len(target_pose) == 6:
                    request_msg.target_pose = [float(x) for x in target_pose]
                    pos_x, pos_y, pos_z = target_pose[0], target_pose[1], target_pose[2]
                    roll, pitch, yaw = target_pose[3], target_pose[4], target_pose[5]
                # 如果是字典格式（包含position和orientation），需要转换
                elif isinstance(target_pose, dict) and 'position' in target_pose and 'orientation' in target_pose:
                    # 提取位置
                    pos_x = float(target_pose['position']['x'])
                    pos_y = float(target_pose['position']['y'])
                    pos_z = float(target_pose['position']['z'])
                    
                    # 提取四元数并转换为欧拉角
                    qx = float(target_pose['orientation']['x'])
                    qy = float(target_pose['orientation']['y'])
                    qz = float(target_pose['orientation']['z'])
                    qw = float(target_pose['orientation']['w'])
                    
                    roll, pitch, yaw = self._quaternion_to_euler(qx, qy, qz, qw)
                    
                    # 构造 [x, y, z, roll, pitch, yaw] 数组
                    request_msg.target_pose = [pos_x, pos_y, pos_z, roll, pitch, yaw]
                else:
                    return jsonify({'success': False, 'error': '目标位姿格式错误，应为数组[x,y,z,a,b,c]或包含position和orientation的字典'})
                
                request_msg.use_joints = use_joints
                request_msg.is_radian = is_radian
                request_msg.velocity = velocity
                
                # 打印详细的调用参数
                self.get_logger().info('=' * 80)
                self.get_logger().info('🤖 机器人运动控制服务调用参数详情:')
                self.get_logger().info(f'   目标位置: X={pos_x:.3f}mm, Y={pos_y:.3f}mm, Z={pos_z:.3f}mm')
                self.get_logger().info(f'   目标姿态(欧拉角): Roll={roll:.6f}rad, Pitch={pitch:.6f}rad, Yaw={yaw:.6f}rad')
                self.get_logger().info(f'   运动模式: {"关节空间" if use_joints else "笛卡尔空间"}')
                self.get_logger().info(f'   角度单位: {"弧度" if is_radian else "度"}')
                self.get_logger().info(f'   运动速度: {velocity:.1f}%')
                self.get_logger().info(f'   服务名称: /set_robot_pose')
                self.get_logger().info('=' * 80)
                
                # 调用服务
                if not self.set_robot_pose_client.wait_for_service(timeout_sec=5.0):
                    return jsonify({'success': False, 'error': '机器人运动控制服务不可用'})
                
                future = self.set_robot_pose_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f'✅ 机器人运动指令发送成功: 位置({pos_x:.1f}, {pos_y:.1f}, {pos_z:.1f})')
                        return jsonify({
                            'success': True, 
                            'message': '机器人运动指令发送成功',
                            'error_code': response.error_code,
                            'response_message': response.message
                        })
                    else:
                        self.get_logger().error(f'❌ 机器人运动指令失败: {response.message}')
                        return jsonify({
                            'success': False, 
                            'error': f'机器人运动指令失败: {response.message}',
                            'error_code': response.error_code
                        })
                else:
                    return jsonify({'success': False, 'error': '服务调用超时'})
                    
            except Exception as e:
                self.get_logger().error(f'❌ 设置机器人位姿失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/robot/move_to_pose', methods=['POST'])
        def move_to_pose():
            """移动机器人到目标位姿 - 调用/move_to_pose服务"""
            try:
                data = request.get_json()
                target_pose = data.get('target_pose')
                use_joints = data.get('use_joints', False)
                velocity_factor = float(data.get('velocity_factor', 0.5))
                acceleration_factor = float(data.get('acceleration_factor', 0.5))
                
                if not target_pose:
                    return jsonify({'success': False, 'error': '缺少目标位姿数据'})
                
                # 创建服务请求
                from demo_interface.srv import MoveToPose
                from geometry_msgs.msg import Pose, Point, Quaternion
                
                request_msg = MoveToPose.Request()
                
                # 解析位姿数据（支持字典格式：position和orientation）
                if isinstance(target_pose, dict) and 'position' in target_pose and 'orientation' in target_pose:
                    # 提取位置
                    request_msg.target_pose.position.x = float(target_pose['position']['x'])
                    request_msg.target_pose.position.y = float(target_pose['position']['y'])
                    request_msg.target_pose.position.z = float(target_pose['position']['z'])
                    
                    # 提取四元数
                    request_msg.target_pose.orientation.x = float(target_pose['orientation']['x'])
                    request_msg.target_pose.orientation.y = float(target_pose['orientation']['y'])
                    request_msg.target_pose.orientation.z = float(target_pose['orientation']['z'])
                    request_msg.target_pose.orientation.w = float(target_pose['orientation']['w'])
                else:
                    return jsonify({'success': False, 'error': '目标位姿格式错误，应为包含position和orientation的字典'})
                
                request_msg.use_joints = use_joints
                request_msg.velocity_factor = velocity_factor
                request_msg.acceleration_factor = acceleration_factor
                
                # 打印调用参数
                self.get_logger().info('=' * 80)
                self.get_logger().info('🤖 机器人移动到目标位姿服务调用:')
                self.get_logger().info(f'   目标位置: X={request_msg.target_pose.position.x:.3f}m, Y={request_msg.target_pose.position.y:.3f}m, Z={request_msg.target_pose.position.z:.3f}m')
                self.get_logger().info(f'   目标姿态(四元数): x={request_msg.target_pose.orientation.x:.6f}, y={request_msg.target_pose.orientation.y:.6f}, z={request_msg.target_pose.orientation.z:.6f}, w={request_msg.target_pose.orientation.w:.6f}')
                self.get_logger().info(f'   运动模式: {"关节空间" if use_joints else "笛卡尔空间"}')
                self.get_logger().info(f'   速度因子: {velocity_factor:.2f}')
                self.get_logger().info(f'   加速度因子: {acceleration_factor:.2f}')
                self.get_logger().info(f'   服务名称: /move_to_pose')
                self.get_logger().info('=' * 80)
                
                # 调用服务
                if not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error('❌ 机器人运动控制服务不可用（等待10秒）')
                    return jsonify({'success': False, 'error': '机器人运动控制服务不可用（等待10秒超时）'})
                
                
                future = self.move_to_pose_client.call_async(request_msg)
                
                # 使用轮询方式检查服务是否完成，而不是等待固定超时时间
                # 根据日志，机器人实际执行时间只有2-7秒，但spin_until_future_complete会等待30秒
                # 增加超时时间以应对机器人移动较慢的情况
                max_wait_time = 30.0  # 最大等待30秒（从10秒增加到30秒）
                check_interval = 0.1  # 每100ms检查一次
                elapsed_time = 0.0
                last_log_time = -5.0  # 记录上次输出日志的时间
                log_interval = 5.0  # 每5秒输出一次进度信息
                
                while elapsed_time < max_wait_time:
                    # 短暂spin以处理服务响应
                    rclpy.spin_once(self, timeout_sec=check_interval)
                    
                    if future.done():
                        # 服务已完成，跳出循环
                        break
                    
                    elapsed_time += check_interval
                    
                    # 每5秒输出一次进度信息（避免频繁日志输出）
                    if elapsed_time - last_log_time >= log_interval:
                        self.get_logger().info(f'⏳ 等待机器人移动服务响应... ({elapsed_time:.1f}/{max_wait_time:.1f}秒)')
                        last_log_time = elapsed_time
                
                
                # 检查服务是否完成
                if future.done():
                    response = future.result()
                    if response and response.success:
                        self.get_logger().info(f'✅ 机器人移动服务返回成功: 位置({request_msg.target_pose.position.x:.3f}, {request_msg.target_pose.position.y:.3f}, {request_msg.target_pose.position.z:.3f})')
                        
                        # 等待机器人到位（基于 in_motion 状态）
                        is_settled = self._wait_for_robot_settled(
                            request_msg.target_pose,
                            max_wait_time=15.0  # 最大等待15秒
                        )
                        
                        if is_settled:
                            self.get_logger().info(f'✅ 机器人已到位，可以采集图像')
                        else:
                            self.get_logger().warning(f'⚠️ 等待机器人到位超时，继续执行')
                        
                        return jsonify({
                            'success': True, 
                            'message': response.message,
                            'error_code': response.error_code
                        })
                    else:
                        self.get_logger().error(f'❌ 机器人移动失败: {response.message}')
                        return jsonify({
                            'success': False, 
                            'error': f'机器人移动失败: {response.message}',
                            'error_code': response.error_code
                        })
                else:
                    self.get_logger().error(f'❌ 机器人移动服务调用超时（等待{max_wait_time}秒）')
                    return jsonify({
                        'success': False, 
                        'error': f'服务调用超时（等待{max_wait_time}秒），机器人可能移动较慢或服务无响应'
                    })
                    
            except Exception as e:
                self.get_logger().error(f'❌ 移动机器人位姿失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
    
    def _wait_for_robot_settled(self, target_pose, position_tolerance=0.001, 
                                 orientation_tolerance=0.01, max_wait_time=5.0, 
                                 stable_duration=0.3):
        """
        等待机器人到位（使用 robot_status.in_motion 判断）
        
        Args:
            target_pose: 目标位姿 (geometry_msgs/Pose) - 保留参数以兼容现有调用
            position_tolerance: 位置容差 (米) - 已废弃，保留以兼容
            orientation_tolerance: 姿态容差 (四元数距离) - 已废弃，保留以兼容
            max_wait_time: 最大等待时间 (秒)
            stable_duration: 稳定持续时间 (秒) - 已废弃，固定为3秒
            
        Returns:
            bool: 是否成功到位
        """
        import time
        
        start_time = time.time()
        motion_stopped_time = None  # 记录运动停止的时间
        
        self.get_logger().info('⏳ 等待机器人到位 (监控 in_motion 状态)...')
        
        while (time.time() - start_time) < max_wait_time:
            # 短暂spin以更新机器人状态
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # 检查机器人状态是否可用
            if self.current_robot_status is None:
                time.sleep(0.05)
                continue
            
            # 检查机器人是否仍在运动
            if self.current_robot_status.in_motion:
                # 机器人正在运动，重置停止时间
                motion_stopped_time = None
                time.sleep(0.05)
                continue
            else:
                # 机器人停止运动 (in_motion == False)
                if motion_stopped_time is None:
                    # 首次检测到停止，记录时间
                    motion_stopped_time = time.time()
                    self.get_logger().info('✅ 机器人已停止运动 (in_motion=False)，等待3秒稳定...')
                
                # 检查是否已经稳定3秒
                elapsed_since_stop = time.time() - motion_stopped_time
                if elapsed_since_stop >= 3.0:
                    self.get_logger().info(f'✅ 机器人已到位并稳定3秒，总耗时 {time.time() - start_time:.2f}秒')
                    return True
            
            time.sleep(0.05)
        
        # 超时
        self.get_logger().warning(f'⚠️ 等待机器人到位超时 ({max_wait_time}秒)')
        return False
    
    def _run_web_server(self):
        """运行Web服务器"""
        self.app.run(
            host=self.web_host,
            port=self.web_port,
            debug=False,
            threaded=True
        )
    
    def image_callback(self, msg):
        """图像回调函数 - 处理ImageData消息"""
        try:
            # 从ImageData消息中提取Image
            image_msg = msg.image
            
            # 检查编码类型
            if image_msg.encoding == 'jpeg' or image_msg.encoding == 'jpg':
                # JPEG编码，直接解码
                np_arr = np.frombuffer(bytes(image_msg.data), np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 其他编码，使用cv_bridge
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
            if image is not None:
                self.current_image = image
                self.current_image_raw = image.copy()  # 保存副本用于处理
                self._image_is_undistorted = False  # 新图像未去畸变
            else:
                self.get_logger().warn('⚠️ 图像转换结果为None')
        except Exception as e:
            self.get_logger().error(f'❌ 图像转换失败: {str(e)}')
            self.get_logger().error(f'   异常类型: {type(e).__name__}')
            import traceback
            self.get_logger().error(f'   堆栈信息: {traceback.format_exc()}')
    
    def robot_status_callback(self, msg):
        """机器人状态回调函数"""
        # 使用线程锁保护写入操作
        with self.robot_status_lock:
            # 手动深拷贝RobotStatus消息，避免引用共享
            new_status = RobotStatus()
            
            # 拷贝header
            new_status.header = Header()
            new_status.header.stamp = Time()
            new_status.header.stamp.sec = msg.header.stamp.sec
            new_status.header.stamp.nanosec = msg.header.stamp.nanosec
            new_status.header.frame_id = msg.header.frame_id
            
            # 拷贝基本字段
            new_status.is_online = msg.is_online
            new_status.enable = msg.enable
            new_status.in_motion = msg.in_motion
            new_status.joint_position_rad = list(msg.joint_position_rad)
            new_status.joint_position_deg = list(msg.joint_position_deg)
            
            # 拷贝cartesian_position（直接从 /demo_robot_status 获取）
            new_status.cartesian_position = Pose()
            new_status.cartesian_position.position = Point()
            new_status.cartesian_position.position.x = msg.cartesian_position.position.x
            new_status.cartesian_position.position.y = msg.cartesian_position.position.y
            new_status.cartesian_position.position.z = msg.cartesian_position.position.z
            new_status.cartesian_position.orientation = Quaternion()
            new_status.cartesian_position.orientation.x = msg.cartesian_position.orientation.x
            new_status.cartesian_position.orientation.y = msg.cartesian_position.orientation.y
            new_status.cartesian_position.orientation.z = msg.cartesian_position.orientation.z
            new_status.cartesian_position.orientation.w = msg.cartesian_position.orientation.w
            
            # 保存深拷贝后的对象
            self.current_robot_status = new_status
            self.current_robot_pose = new_status.cartesian_position  # 保持兼容性
            
            # 同时存储timestamp为原始类型
            self.current_robot_status_timestamp = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1000000)
    
    def camera_status_callback(self, msg):
        """相机状态回调函数"""
        self.current_camera_status = msg
    
    # camera_info_callback 已完全删除，改为在初始化时从文件加载相机内参
    
    def depth_image_callback(self, msg):
        """深度图回调函数 - 保存当前深度图用于深度误差检查"""
        try:
            # 将ROS图像消息转换为OpenCV格式（16位深度图）
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_depth_image = cv_image
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {str(e)}')
    
    def _rotation_matrix_to_quaternion(self, R):
        """
        将旋转矩阵转换为四元数（兼容不同版本的scipy）
        返回: [x, y, z, w]
        """
        from scipy.spatial.transform import Rotation as R_scipy
        try:
            # 尝试使用from_matrix（新版本scipy）
            if hasattr(R_scipy, 'from_matrix'):
                r = R_scipy.from_matrix(R)
                return r.as_quat()  # [x, y, z, w]
            else:
                # 使用from_dcm（旧版本scipy）
                r = R_scipy.from_dcm(R)
                return r.as_quat()
        except (AttributeError, TypeError):
            # 如果scipy方法都失败，使用手动转换（Shepperd's method）
            trace = np.trace(R)
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w = 0.25 * s
                x = (R[2, 1] - R[1, 2]) / s
                y = (R[0, 2] - R[2, 0]) / s
                z = (R[1, 0] - R[0, 1]) / s
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    w = (R[2, 1] - R[1, 2]) / s
                    x = 0.25 * s
                    y = (R[0, 1] + R[1, 0]) / s
                    z = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    w = (R[0, 2] - R[2, 0]) / s
                    x = (R[0, 1] + R[1, 0]) / s
                    y = 0.25 * s
                    z = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    w = (R[1, 0] - R[0, 1]) / s
                    x = (R[0, 2] + R[2, 0]) / s
                    y = (R[1, 2] + R[2, 1]) / s
                    z = 0.25 * s
            return np.array([x, y, z, w])
    
    def _quaternion_to_euler(self, qx, qy, qz, qw):
        """
        将四元数转换为欧拉角 (Roll-Pitch-Yaw)
        quat: [x, y, z, w]
        返回: (roll, pitch, yaw) 单位：弧度
        """
        import math
        
        # Roll (绕X轴)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (绕Y轴)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (绕Z轴)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _quaternion_to_rotation_matrix(self, quat):
        """
        将四元数转换为旋转矩阵
        quat: [x, y, z, w]
        """
        x, y, z, w = quat
        
        # 归一化四元数
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 构建旋转矩阵
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def _solve_rigid_transform(self, points_source, points_target):
        """
        使用SVD方法求解刚体变换（3D点云配准）
        points_source: Nx3 源点云（相机坐标系）
        points_target: Nx3 目标点云（机器人坐标系）
        返回: R (3x3旋转矩阵), t (3x1平移向量)
        使得: points_target = R @ points_source^T + t
        """
        # 计算质心
        centroid_source = np.mean(points_source, axis=0)
        centroid_target = np.mean(points_target, axis=0)
        
        # 去质心
        source_centered = points_source - centroid_source
        target_centered = points_target - centroid_target
        
        # 计算协方差矩阵
        H = source_centered.T @ target_centered
        
        # SVD分解
        U, S, Vt = np.linalg.svd(H)
        
        # 计算旋转矩阵
        R = Vt.T @ U.T
        
        # 处理反射情况（保证R的行列式为+1）
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 计算平移向量
        t = centroid_target - R @ centroid_source
        
        return R, t.reshape(3, 1)
    
    def _build_calibration_xml(self, calibration_data):
        """
        构建包含相机参数和手眼标定结果的XML内容
        """
        from datetime import datetime
        
        # 获取当前时间
        now = datetime.now()
        timestamp = now.strftime('%Y-%m-%d %H:%M:%S')
        
        # 获取相机参数
        camera_matrix = self.calib_utils.camera_matrix
        dist_coeffs = self.calib_utils.dist_coeffs
        image_size = self.calib_utils.image_size
        
        # 获取手眼标定结果
        T_matrix = np.array(calibration_data.get('transformation_matrix', np.eye(4)))
        R_matrix = T_matrix[:3, :3]
        t_vector = T_matrix[:3, 3]
        
        # 获取评估数据
        evaluation = calibration_data.get('evaluation', {})
        method = calibration_data.get('method', 'SVD')
        calib_type = calibration_data.get('calibration_type', 'Eye-to-Hand')
        camera_height = calibration_data.get('camera_height', 550.0)
        
        # 构建XML
        xml_lines = [
            '<?xml version="1.0" encoding="utf-8"?>',
            '<opencv_storage>',
            '   <!-- ========================================== -->',
            '   <!-- 相机标定参数 (Camera Calibration Parameters) -->',
            '   <!-- ========================================== -->',
            ''
        ]
        
        # 添加图像尺寸
        if image_size:
            xml_lines.extend([
                '   <ImageSize type_id="opencv-matrix">',
                '      <rows>1</rows>',
                '      <cols>2</cols>',
                '      <dt>d</dt>',
                f'      <data>{image_size[0]} {image_size[1]}</data>',
                '   </ImageSize>'
            ])
        
        # 添加相机矩阵
        if camera_matrix is not None:
            cm_data = ' '.join([f'{val:.16f}' for val in camera_matrix.flatten()])
            xml_lines.extend([
                '   <CameraMatrix type_id="opencv-matrix">',
                '      <rows>3</rows>',
                '      <cols>3</cols>',
                '      <dt>d</dt>',
                f'      <data>{cm_data}</data>',
                '   </CameraMatrix>',
                '   <FocalLength type_id="opencv-matrix">',
                '      <rows>1</rows>',
                '      <cols>2</cols>',
                '      <dt>d</dt>',
                f'      <data>{camera_matrix[0,0]:.16f} {camera_matrix[1,1]:.16f}</data>',
                '   </FocalLength>',
                '   <PrincipalPoint type_id="opencv-matrix">',
                '      <rows>1</rows>',
                '      <cols>2</cols>',
                '      <dt>d</dt>',
                f'      <data>{camera_matrix[0,2]:.16f} {camera_matrix[1,2]:.16f}</data>',
                '   </PrincipalPoint>'
            ])
        
        # 添加畸变系数
        if dist_coeffs is not None:
            dc_data = ' '.join([f'{val:.16f}' for val in dist_coeffs.flatten()])
            xml_lines.extend([
                '   <DistortionCoefficients type_id="opencv-matrix">',
                '      <rows>1</rows>',
                f'      <cols>{len(dist_coeffs)}</cols>',
                '      <dt>d</dt>',
                f'      <data>{dc_data}</data>',
                '   </DistortionCoefficients>'
            ])
        
        # 添加手眼标定部分
        xml_lines.extend([
            '',
            '   <!-- ========================================== -->',
            '   <!-- 手眼标定结果 (Hand-Eye Calibration Results) -->',
            '   <!-- ========================================== -->',
            '',
            '   <!-- 标定元数据 -->',
            f'   <CalibrationType>{calib_type}</CalibrationType>',
            f'   <CalibrationMethod>{method}</CalibrationMethod>',
            f'   <CalibrationDate>{timestamp}</CalibrationDate>',
            f'   <CameraHeight unit="mm">{camera_height:.3f}</CameraHeight>',
            '',
            '   <!-- 变换矩阵: 相机坐标系 → 机器人基座坐标系 -->',
            '   <TransformationMatrix type_id="opencv-matrix">',
            '      <rows>4</rows>',
            '      <cols>4</cols>',
            '      <dt>d</dt>',
            f'      <data>{" ".join([f"{val:.16f}" for val in T_matrix.flatten()])}</data>',
            '   </TransformationMatrix>',
            '',
            '   <!-- 旋转矩阵 (3x3) -->',
            '   <RotationMatrix type_id="opencv-matrix">',
            '      <rows>3</rows>',
            '      <cols>3</cols>',
            '      <dt>d</dt>',
            f'      <data>{" ".join([f"{val:.16f}" for val in R_matrix.flatten()])}</data>',
            '   </RotationMatrix>',
            '',
            '   <!-- 平移向量 (单位: mm) -->',
            '   <TranslationVector type_id="opencv-matrix" unit="mm">',
            '      <rows>3</rows>',
            '      <cols>1</cols>',
            '      <dt>d</dt>',
            f'      <data>{" ".join([f"{val:.16f}" for val in t_vector])}</data>',
            '   </TranslationVector>',
            '',
            '   <!-- 标定精度评估 -->',
            f'   <MeanCalibrationError unit="mm">{evaluation.get("mean_error", 0.0):.6f}</MeanCalibrationError>',
            f'   <MaxCalibrationError unit="mm">{evaluation.get("max_error", 0.0):.6f}</MaxCalibrationError>',
            f'   <MinCalibrationError unit="mm">{evaluation.get("min_error", 0.0):.6f}</MinCalibrationError>',
            f'   <StdCalibrationError unit="mm">{evaluation.get("std_error", 0.0):.6f}</StdCalibrationError>',
            f'   <DataPointCount>{evaluation.get("data_count", 0)}</DataPointCount>',
            '',
            '   <!-- 使用说明 -->',
            '   <UsageNotes>',
            '      此文件包含完整的相机内参和手眼标定结果。',
            '      变换矩阵将相机坐标系的点转换到机器人基座坐标系。',
            f'      标定场景: {calib_type}',
            f'      标定方法: {method}',
            f'      平均误差: {evaluation.get("mean_error", 0.0):.3f} mm',
            '      单位说明: 平移向量单位为毫米(mm)，旋转矩阵无单位',
            '   </UsageNotes>',
            '</opencv_storage>'
        ])
        
        return '\n'.join(xml_lines)
    
    def _build_calibration_yaml(self, calibration_data):
        """
        构建包含相机参数和手眼标定结果的YAML内容（参考ost.yaml格式）
        """
        from datetime import datetime
        import yaml
        
        # 获取当前时间
        now = datetime.now()
        timestamp = now.strftime('%Y-%m-%d %H:%M:%S')
        
        # 获取相机参数
        camera_matrix = self.calib_utils.camera_matrix
        dist_coeffs = self.calib_utils.dist_coeffs
        image_size = self.calib_utils.image_size
        
        # 获取手眼标定结果
        T_matrix_data = calibration_data.get('transformation_matrix')
        if T_matrix_data is None:
            # 如果没有提供，使用单位矩阵（但会记录警告）
            self.get_logger().warning('⚠️ 未找到标定结果数据，使用单位矩阵（可能数据未正确传递）')
            T_matrix = np.eye(4)
        else:
            T_matrix = np.array(T_matrix_data)
            # 验证矩阵维度
            if T_matrix.shape != (4, 4):
                self.get_logger().error(f'❌ 变换矩阵维度错误: {T_matrix.shape}，期望 (4, 4)')
                T_matrix = np.eye(4)
        
        R_matrix = T_matrix[:3, :3]
        t_vector = T_matrix[:3, 3]
        
        # 数据验证：检查是否是单位矩阵（可能是默认值）
        is_identity = np.allclose(T_matrix, np.eye(4), atol=1e-6)
        if is_identity:
            self.get_logger().warning('⚠️ 变换矩阵为单位矩阵，可能标定未完成或数据未正确保存')
        
        # 数据验证：检查平移向量是否为零（可能是默认值）
        is_zero_translation = np.allclose(t_vector, 0.0, atol=1e-6)
        if is_zero_translation:
            self.get_logger().warning('⚠️ 平移向量为零，可能标定未完成或数据未正确保存')
        
        # 获取评估数据
        evaluation = calibration_data.get('evaluation', {})
        method = calibration_data.get('method', 'SVD')
        calib_type = calibration_data.get('calibration_type', 'Eye-to-Hand')
        camera_height = calibration_data.get('camera_height', 550.0)
        
        # 构建YAML数据字典
        yaml_data = {}
        
        # 相机内参部分（参考ost.yaml格式）
        if image_size:
            yaml_data['image_width'] = int(image_size[0])
            yaml_data['image_height'] = int(image_size[1])
        
        yaml_data['camera_name'] = 'hand_eye_camera'
        
        if camera_matrix is not None:
            # 使用矩阵格式（二维数组）显示
            yaml_data['camera_matrix'] = camera_matrix.tolist()
            yaml_data['camera_matrix_info'] = {
                'rows': 3,
                'cols': 3,
                'description': '相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]'
            }
        
        if dist_coeffs is not None:
            yaml_data['distortion_model'] = 'plumb_bob'
            # 畸变系数保持一维数组格式（因为只有一行）
            yaml_data['distortion_coefficients'] = dist_coeffs.flatten().tolist()
            yaml_data['distortion_coefficients_info'] = {
                'count': len(dist_coeffs),
                'description': '畸变系数 (k1, k2, p1, p2, k3)'
            }
        
        # 手眼标定结果部分
        yaml_data['hand_eye_calibration'] = {
            'calibration_type': calib_type,
            'calibration_method': method,
            'calibration_date': timestamp,
            'camera_height_mm': float(camera_height),
            # 使用矩阵格式（二维数组）显示变换矩阵
            'transformation_matrix': T_matrix.tolist(),
            'transformation_matrix_info': {
                'rows': 4,
                'cols': 4,
                'description': '齐次变换矩阵：相机坐标系 → 机器人基座坐标系',
                'format': '[[R11, R12, R13, tx], [R21, R22, R23, ty], [R31, R32, R33, tz], [0, 0, 0, 1]]'
            },
            # 使用矩阵格式（二维数组）显示旋转矩阵
            'rotation_matrix': R_matrix.tolist(),
            'rotation_matrix_info': {
                'rows': 3,
                'cols': 3,
                'description': '旋转矩阵（3x3），无单位'
            },
            # 平移向量保持一维数组格式（列向量）
            'translation_vector': t_vector.tolist(),
            'translation_vector_info': {
                'rows': 3,
                'cols': 1,
                'unit': 'mm',
                'description': '平移向量 [tx, ty, tz]，单位：毫米'
            },
            'calibration_accuracy': {
                'mean_error_mm': float(evaluation.get('mean_error', 0.0)),
                'max_error_mm': float(evaluation.get('max_error', 0.0)),
                'min_error_mm': float(evaluation.get('min_error', 0.0)),
                'std_error_mm': float(evaluation.get('std_error', 0.0)),
                'data_point_count': int(evaluation.get('data_count', 0))
            }
        }
        
        # 使用yaml.dump生成YAML字符串
        yaml_content = yaml.dump(yaml_data, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        return yaml_content
    
    def _load_hand_eye_calibration_xml(self, xml_file: str) -> Dict:
        """
        加载手眼标定参数XML文件
        
        Args:
            xml_file: XML文件路径
            
        Returns:
            参数字典
        """
        try:
            import xml.etree.ElementTree as ET
            
            tree = ET.parse(xml_file)
            root = tree.getroot()
            
            result = {
                'success': True,
                'camera_intrinsic': {},
                'hand_eye_calibration': {}
            }
            
            # 读取相机内参
            camera_matrix_node = root.find('.//CameraMatrix/data')
            if camera_matrix_node is not None:
                data_str = camera_matrix_node.text.strip()
                data = list(map(float, data_str.split()))
                camera_matrix = np.array(data).reshape(3, 3)
                
                result['camera_intrinsic'] = {
                    'fx': float(camera_matrix[0, 0]),
                    'fy': float(camera_matrix[1, 1]),
                    'cx': float(camera_matrix[0, 2]),
                    'cy': float(camera_matrix[1, 2]),
                    'camera_matrix': camera_matrix.tolist()
                }
            
            # 读取畸变系数
            dist_coeffs_node = root.find('.//DistortionCoefficients/data')
            if dist_coeffs_node is not None:
                data_str = dist_coeffs_node.text.strip()
                data = list(map(float, data_str.split()))
                result['camera_intrinsic']['dist_coeffs'] = data
            
            # 读取图像尺寸
            image_size_node = root.find('.//ImageSize/data')
            if image_size_node is not None:
                size_str = image_size_node.text.strip()
                result['camera_intrinsic']['image_size'] = list(map(int, size_str.split()))
            
            # 读取手眼标定结果
            calib_type_node = root.find('.//CalibrationType')
            if calib_type_node is not None:
                result['hand_eye_calibration']['calibration_type'] = calib_type_node.text.strip()
            
            calib_method_node = root.find('.//CalibrationMethod')
            if calib_method_node is not None:
                result['hand_eye_calibration']['calibration_method'] = calib_method_node.text.strip()
            
            calib_date_node = root.find('.//CalibrationDate')
            if calib_date_node is not None:
                result['hand_eye_calibration']['calibration_date'] = calib_date_node.text.strip()
            
            camera_height_node = root.find('.//CameraHeight')
            if camera_height_node is not None:
                result['hand_eye_calibration']['camera_height'] = float(camera_height_node.text.strip())
            
            # 读取变换矩阵
            transform_matrix_node = root.find('.//TransformationMatrix/data')
            if transform_matrix_node is not None:
                data_str = transform_matrix_node.text.strip()
                data = list(map(float, data_str.split()))
                transform_matrix = np.array(data).reshape(4, 4)
                result['hand_eye_calibration']['transformation_matrix'] = transform_matrix.tolist()
            
            # 读取旋转矩阵
            rotation_matrix_node = root.find('.//RotationMatrix/data')
            if rotation_matrix_node is not None:
                data_str = rotation_matrix_node.text.strip()
                data = list(map(float, data_str.split()))
                rotation_matrix = np.array(data).reshape(3, 3)
                result['hand_eye_calibration']['rotation_matrix'] = rotation_matrix.tolist()
            
            # 读取平移向量
            translation_vector_node = root.find('.//TranslationVector/data')
            if translation_vector_node is not None:
                data_str = translation_vector_node.text.strip()
                data = list(map(float, data_str.split()))
                result['hand_eye_calibration']['translation_vector'] = data
            
            # 读取精度评估
            mean_error_node = root.find('.//MeanCalibrationError')
            if mean_error_node is not None:
                result['hand_eye_calibration']['mean_error'] = float(mean_error_node.text.strip())
            
            max_error_node = root.find('.//MaxCalibrationError')
            if max_error_node is not None:
                result['hand_eye_calibration']['max_error'] = float(max_error_node.text.strip())
            
            min_error_node = root.find('.//MinCalibrationError')
            if min_error_node is not None:
                result['hand_eye_calibration']['min_error'] = float(min_error_node.text.strip())
            
            std_error_node = root.find('.//StdCalibrationError')
            if std_error_node is not None:
                result['hand_eye_calibration']['std_error'] = float(std_error_node.text.strip())
            
            data_count_node = root.find('.//DataPointCount')
            if data_count_node is not None:
                result['hand_eye_calibration']['data_count'] = int(data_count_node.text.strip())
            
            self.get_logger().info(f'✅ 手眼标定参数加载成功')
            self.get_logger().info(f'   标定类型: {result["hand_eye_calibration"].get("calibration_type", "未知")}')
            self.get_logger().info(f'   标定方法: {result["hand_eye_calibration"].get("calibration_method", "未知")}')
            self.get_logger().info(f'   平均误差: {result["hand_eye_calibration"].get("mean_error", 0):.3f} mm')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ 解析手眼标定XML文件失败: {str(e)}')
            return {
                'success': False,
                'error': str(e)
            }


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

