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
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, CameraInfo
from demo_interface.msg import RobotStatus
from percipio_camera_interface.msg import CameraStatus, ImageData
from percipio_camera_interface.srv import SoftwareTrigger
import base64
from ament_index_python.packages import get_package_share_directory
from .camera_calibration_utils import CameraCalibrationUtils
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
        self.current_robot_pose = None
        self.current_robot_status = None  # 完整的机器人状态（包括is_online等）
        self.current_camera_status = None  # 相机状态
        self.current_point_cloud = None  # 当前点云数据（保留兼容性）
        self.current_depth_image = None  # 当前深度图（对齐到彩色图）
        self.current_camera_info = None  # 当前相机内参（从 CameraInfo 话题获取）
        self.bridge = CvBridge()
        
        # 相机标定工具
        self.calib_utils = CameraCalibrationUtils()
        
        # 角点检测结果
        self.detected_corners = None
        self.corners_3d = None
        self.pattern_size = (10, 7)  # 默认棋盘格大小
        
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
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        import json
        import time
        log_file.write(json.dumps({'id': 'log_depth_image_sub_created', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:109', 'message': '深度图订阅已创建', 'data': {'topic': depth_image_topic, 'sub_exists': self.depth_image_subscription is not None}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'I'}) + '\n')
        log_file.close()
        # #endregion
        self.get_logger().info(f'📡 已订阅 {depth_image_topic} 话题，等待深度图数据...')
        
        # 软触发客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/software_trigger')
        
        # 机器人运动控制服务客户端
        from demo_interface.srv import SetRobotPose
        self.set_robot_pose_client = self.create_client(SetRobotPose, '/set_robot_pose')
        
        # 使用固定话题名 'robot_status'，通过launch文件的remapping映射到实际话题
        # 这样可以通过remapping灵活映射到 /demo_robot_status 或其他话题
        robot_status_topic = 'robot_status'  # 固定话题名，通过remapping映射
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        import json
        import time
        log_file.write(json.dumps({'id': 'log_init_sub', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:102', 'message': '创建robot_status订阅', 'data': {'topic': robot_status_topic, 'msg_type': 'RobotStatus'}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'A'}) + '\n')
        log_file.close()
        # #endregion
        self.robot_status_subscription = self.create_subscription(
            RobotStatus,
            robot_status_topic,
            self.robot_status_callback,
            10
        )
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        log_file.write(json.dumps({'id': 'log_sub_created', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:115', 'message': 'robot_status订阅已创建', 'data': {'sub_exists': self.robot_status_subscription is not None}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'A'}) + '\n')
        log_file.close()
        # #endregion
        
        # 订阅相机状态
        self.camera_status_subscription = self.create_subscription(
            CameraStatus,
            '/camera_status',
            self.camera_status_callback,
            10
        )
        
        # 订阅相机内参（CameraInfo）
        # 参考 depth_z_reader 的实现，订阅相机发布的 CameraInfo 话题
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            image_qos
        )
        self.get_logger().info(f'📡 已订阅 {camera_info_topic} 话题，等待相机内参数据...')
        
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
                # #region agent log
                log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                import json
                import time
                log_file.write(json.dumps({'id': 'log_api_entry', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:242', 'message': 'API /api/robot_status被调用', 'data': {'current_robot_status_is_none': self.current_robot_status is None, 'current_robot_pose_is_none': self.current_robot_pose is None}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                log_file.close()
                # #endregion
                if self.current_robot_status is not None:
                    # #region agent log
                    log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                    try:
                        cart = self.current_robot_status.cartesian_position
                        log_file.write(json.dumps({'id': 'log_api_success_path', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:255', 'message': 'API返回成功路径', 'data': {'is_online': self.current_robot_status.is_online, 'position': {'x': cart.position.x, 'y': cart.position.y, 'z': cart.position.z}, 'orientation': {'x': cart.orientation.x, 'y': cart.orientation.y, 'z': cart.orientation.z, 'w': cart.orientation.w}}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                    except Exception as e:
                        log_file.write(json.dumps({'id': 'log_api_error_in_log', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:257', 'message': '日志记录时出错', 'data': {'error': str(e)}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                    log_file.close()
                    # #endregion
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
                        # #region agent log
                        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                        import traceback
                        log_file.write(json.dumps({'id': 'log_api_error_creating_response', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:288', 'message': '创建API响应时出错', 'data': {'error': str(e), 'traceback': traceback.format_exc()[:500]}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                        log_file.close()
                        # #endregion
                        self.get_logger().error(f'创建API响应时出错: {str(e)}')
                        import traceback
                        self.get_logger().error(traceback.format_exc())
                        return jsonify({'success': False, 'message': f'处理机器人状态数据时出错: {str(e)}'}), 500
                # #region agent log
                log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                log_file.write(json.dumps({'id': 'log_api_fail_path', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:295', 'message': 'API返回失败路径', 'data': {'reason': 'current_robot_status is None'}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                log_file.close()
                # #endregion
                return jsonify({'success': False, 'message': '无机器人状态数据'})
            except Exception as e:
                # #region agent log
                log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
                import traceback
                log_file.write(json.dumps({'id': 'log_api_unhandled_exception', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:300', 'message': 'API未处理异常', 'data': {'error': str(e), 'traceback': traceback.format_exc()[:500]}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'C'}) + '\n')
                log_file.close()
                # #endregion
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
                square_size = data.get('square_size', 15.0)  # mm
                
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '请先加载图像'})
                
                if self.calib_utils.camera_matrix is None:
                    return jsonify({'success': False, 'error': '请先加载相机参数'})
                
                # 检测角点
                corners = self.calib_utils.detect_checkerboard_corners(
                    self.current_image_raw, self.pattern_size
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
                
                # 绘制角点
                img_with_corners = self.calib_utils.draw_corners(
                    self.current_image_raw, corners, self.pattern_size
                )
                
                # 转换为base64
                _, buffer = cv2.imencode('.jpg', img_with_corners)
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
                
                # 深度误差检查（如果深度图可用）
                depth_error_result = None
                if self.current_depth_image is not None:
                    try:
                        depth_error_result = self.calib_utils.evaluate_depth_error(
                            corners, self.current_depth_image, self.pattern_size, square_size
                        )
                        if depth_error_result['success']:
                            self.get_logger().info(f'深度误差检查完成: 平均误差 {depth_error_result["depth_statistics"]["mean_error"]:.2f} mm')
                            
                            # 使用实际深度重新构建3D点（优先使用实际深度，无效时才用估计深度）
                            # 这样计算相邻角点距离和对角线距离时会更准确
                            depth_details = depth_error_result.get('depth_details', [])
                            if len(depth_details) == len(corners_2d):
                                points_3d_with_real_depth = []
                                for i, (corner_2d, point_3d_est, detail) in enumerate(zip(corners_2d, points_3d, depth_details)):
                                    # 优先使用实际深度，如果无效则使用估计深度
                                    actual_depth = detail.get('actual_depth', 0)
                                    estimated_depth = detail.get('estimated_depth', point_3d_est[2])
                                    
                                    # 使用实际深度（如果有效），否则使用估计深度
                                    z_to_use = actual_depth if actual_depth > 0 else estimated_depth
                                    
                                    # 使用实际深度重新计算3D坐标
                                    point_3d_real = self.calib_utils.pixel_to_camera_coords(
                                        corner_2d.reshape(1, 2), z_to_use
                                    )[0]
                                    points_3d_with_real_depth.append(point_3d_real)
                                
                                # 更新points_3d和corners_3d为使用实际深度的版本
                                points_3d = np.array(points_3d_with_real_depth)
                                self.corners_3d = points_3d
                                
                                # 重新计算相邻距离（使用实际深度）
                                distances = self.calib_utils.calculate_adjacent_distances(
                                    points_3d, self.pattern_size
                                )
                                
                                # 更新corners_data中的3D坐标和距离
                                corners_data = []
                                for i, (corner, point_3d, dist) in enumerate(zip(corners_2d, points_3d, distances)):
                                    corners_data.append({
                                        'index': i,
                                        'pixel_u': float(corner[0]),
                                        'pixel_v': float(corner[1]),
                                        'camera_x': float(point_3d[0]),
                                        'camera_y': float(point_3d[1]),
                                        'camera_z': float(point_3d[2]),  # 现在使用实际深度
                                        'adjacent_distance': float(dist)
                                    })
                                
                                self.get_logger().info('✅ 已使用实际深度重新构建3D点，距离计算更准确')
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
                square_size = data.get('square_size', 15.0)  # mm
                
                if self.current_image_raw is None:
                    return jsonify({'success': False, 'error': '请先加载图像'})
                
                if self.calib_utils.camera_matrix is None:
                    return jsonify({'success': False, 'error': '请先加载相机参数'})
                
                # 检测角点
                corners = self.calib_utils.detect_checkerboard_corners(
                    self.current_image_raw, self.pattern_size
                )
                
                if corners is None:
                    return jsonify({'success': False, 'error': '未检测到棋盘格角点'})
                
                # 使用solvePnP估计标定板姿态
                # 构建3D对象点（标定板坐标系）
                pattern_w, pattern_h = self.pattern_size
                obj_points = []
                for j in range(pattern_h):
                    for i in range(pattern_w):
                        obj_points.append([i * square_size, j * square_size, 0.0])
                obj_points = np.array(obj_points, dtype=np.float32)
                
                # 使用solvePnP求解
                corners_2d = corners.reshape(-1, 1, 2).astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners_2d,
                    self.calib_utils.camera_matrix,
                    self.calib_utils.dist_coeffs
                )
                
                if not success:
                    return jsonify({'success': False, 'error': 'solvePnP求解失败'})
                
                # 将旋转向量转换为旋转矩阵
                R_board2cam, _ = cv2.Rodrigues(rvec)
                
                # 旋转矩阵转四元数
                from scipy.spatial.transform import Rotation as R
                r = R.from_matrix(R_board2cam)
                quat = r.as_quat()  # [x, y, z, w]
                
                # tvec是标定板坐标系原点到相机坐标系原点的平移向量（在相机坐标系下）
                position = {
                    'x': float(tvec[0, 0]),
                    'y': float(tvec[1, 0]),
                    'z': float(tvec[2, 0])
                }
                
                orientation = {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                }
                
                return jsonify({
                    'success': True,
                    'position': position,
                    'orientation': orientation,
                    'translation_mm': {
                        'x': position['x'],
                        'y': position['y'],
                        'z': position['z']
                    }
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
                expected_size = data.get('expected_size', 15.0)  # mm
                
                if self.corners_3d is None:
                    return jsonify({'success': False, 'error': '请先提取角点'})
                
                if self.detected_corners is None:
                    return jsonify({'success': False, 'error': '请先提取角点'})
                
                # 计算最大距离和误差
                result = self.calib_utils.calculate_max_distance_error(
                    self.corners_3d, self.pattern_size, expected_size
                )
                
                # 深度误差检查（如果深度图数据可用）
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
                            
                            # 使用实际深度重新构建3D点并重新计算距离
                            depth_details = depth_error_result.get('depth_details', [])
                            corners_2d = self.detected_corners.reshape(-1, 2)
                            if len(depth_details) == len(corners_2d):
                                points_3d_with_real_depth = []
                                for i, (corner_2d, detail) in enumerate(zip(corners_2d, depth_details)):
                                    # 优先使用实际深度，无效时使用估计深度
                                    actual_depth = detail.get('actual_depth', 0)
                                    estimated_depth = detail.get('estimated_depth', self.corners_3d[i][2] if i < len(self.corners_3d) else 0)
                                    z_to_use = actual_depth if actual_depth > 0 else estimated_depth
                                    
                                    # 使用实际深度重新计算3D坐标
                                    point_3d_real = self.calib_utils.pixel_to_camera_coords(
                                        corner_2d.reshape(1, 2), z_to_use
                                    )[0]
                                    points_3d_with_real_depth.append(point_3d_real)
                                
                                # 更新corners_3d
                                self.corners_3d = np.array(points_3d_with_real_depth)
                                
                                # 重新计算距离（使用实际深度）
                                result = self.calib_utils.calculate_max_distance_error(
                                    self.corners_3d, self.pattern_size, expected_size
                                )
                                result['depth_error'] = depth_error_result
                                
                                self.get_logger().info('✅ 距离计算使用实际深度，结果更准确')
                    except Exception as e:
                        self.get_logger().warn(f'深度误差检查失败: {str(e)}')
                
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
        
        @self.app.route('/api/hand_eye/calibrate', methods=['POST'])
        def perform_hand_eye_calibration():
            """执行手眼标定 - 支持Eye-to-Hand和Eye-in-Hand两种方式"""
            try:
                data = request.get_json()
                calibration_type = data.get('calibration_type', 'eye-to-hand')  # 默认Eye-to-Hand
                pose_data = data.get('pose_data', [])
                
                self.get_logger().info('='*60)
                
                if calibration_type == 'eye-in-hand':
                    # Eye-in-Hand标定
                    calibration_method = data.get('calibration_method', 'corner-based')  # 默认角点法
                    if calibration_method == 'pose-based':
                        # 姿态法标定（已实现但暂时禁用）
                        # TODO: 启用姿态法时，将下面的return注释掉，启用实际的标定调用
                        return jsonify({
                            'success': False,
                            'error': '姿态法标定功能已实现但暂时禁用，请先使用角点法进行标定'
                        })
                        # return self._perform_eye_in_hand_pose_based_calibration(data)  # 启用时取消注释
                    else:
                        # 角点法标定（原有逻辑）
                        return self._perform_eye_in_hand_calibration(data)
                else:
                    # Eye-to-Hand标定（原有逻辑）
                    return self._perform_eye_to_hand_calibration(data)
                
            except Exception as e:
                self.get_logger().error(f'❌ 手眼标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        def _perform_eye_to_hand_calibration(self, data):
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
        
        def _perform_eye_in_hand_calibration(self, data):
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
                    T_camera2gripper, fixed_z = self._solve_eye_in_hand(shot_pose, pick_poses, camera_points_shot)
                    
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
        
        def _pose_to_transform_matrix(self, position, orientation_quat):
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
        
        def _solve_eye_in_hand(self, shot_pose, pick_poses, camera_points_shot, fixed_z=None):
            """
            Eye-in-Hand手眼标定核心算法
            基于hand_eye_calibration.py中的实现
            """
            n = len(pick_poses)
            
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
                    R_mat = r.as_matrix()
                
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
            
            r_init = R.from_matrix(R_init)
            axis_angle_init = r_init.as_rotvec()
            
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
                R_camera2gripper = r.as_matrix()
            
            # 构建变换矩阵
            T_camera2gripper = np.eye(4)
            T_camera2gripper[:3, :3] = R_camera2gripper
            T_camera2gripper[:3, 3] = translation
            
            mean_residual = np.mean(np.abs(result.fun))
            self.get_logger().info(f'   优化完成，平均残差: {mean_residual:.6f} mm')
            
            return T_camera2gripper, fixed_z
        
        def _perform_eye_in_hand_pose_based_calibration(self, data):
            """执行Eye-in-Hand标定 - 姿态法（Pose-based method，类似Tsai方法）
            
            姿态法原理：通过多组机器人运动，求解AX=XB问题
            - A: 机器人基座坐标系下的运动（从姿态1到姿态2）
            - X: 相机到末端执行器的变换（待求）
            - B: 相机坐标系下的运动（通过观察标定板计算）
            
            流程：
            1. 使用Tsai方法或SVD方法求解初始估计
            2. 使用非线性优化进一步优化结果
            """
            try:
                # 姿态法需要多组运动数据，每组包含两个姿态和对应的标定板观察
                motion_groups = data.get('motion_groups', [])
                
                self.get_logger().info(f'🤖 开始Eye-in-Hand手眼标定（姿态法）')
                self.get_logger().info(f'   标定场景: 相机安装在机器人末端')
                self.get_logger().info(f'   运动组数: {len(motion_groups)}')
                
                if len(motion_groups) < 2:
                    return jsonify({
                        'success': False, 
                        'error': f'数据不足，至少需要2组运动数据，当前只有{len(motion_groups)}组'
                    })
                
                # 检查相机标定参数
                if self.calib_utils.camera_matrix is None or self.calib_utils.dist_coeffs is None:
                    return jsonify({
                        'success': False, 
                        'error': '请先加载相机标定参数'
                    })
                
                # 构建运动矩阵
                A_list = []  # 机器人基座坐标系下的运动
                B_list = []  # 相机坐标系下的运动（通过标定板观察计算）
                
                for idx, motion in enumerate(motion_groups):
                    try:
                        pose1_data = motion.get('pose1')
                        pose2_data = motion.get('pose2')
                        board_pose1_data = motion.get('board_pose1')
                        board_pose2_data = motion.get('board_pose2')
                        
                        if not all([pose1_data, pose2_data, board_pose1_data, board_pose2_data]):
                            self.get_logger().warning(f'   运动组 #{idx+1} 数据不完整，已跳过')
                            continue
                        
                        # 构建变换矩阵
                        T_gripper1 = self._pose_to_transform_matrix(
                            {'x': pose1_data['robot_pos_x'], 'y': pose1_data['robot_pos_y'], 'z': pose1_data['robot_pos_z']},
                            {'x': pose1_data['robot_ori_x'], 'y': pose1_data['robot_ori_y'], 'z': pose1_data['robot_ori_z'], 'w': pose1_data['robot_ori_w']}
                        )
                        T_gripper2 = self._pose_to_transform_matrix(
                            {'x': pose2_data['robot_pos_x'], 'y': pose2_data['robot_pos_y'], 'z': pose2_data['robot_pos_z']},
                            {'x': pose2_data['robot_ori_x'], 'y': pose2_data['robot_ori_y'], 'z': pose2_data['robot_ori_z'], 'w': pose2_data['robot_ori_w']}
                        )
                        
                        T_board1 = self._pose_to_transform_matrix(
                            {'x': board_pose1_data['position']['x'], 'y': board_pose1_data['position']['y'], 'z': board_pose1_data['position']['z']},
                            {'x': board_pose1_data['orientation']['x'], 'y': board_pose1_data['orientation']['y'], 
                             'z': board_pose1_data['orientation']['z'], 'w': board_pose1_data['orientation']['w']}
                        )
                        T_board2 = self._pose_to_transform_matrix(
                            {'x': board_pose2_data['position']['x'], 'y': board_pose2_data['position']['y'], 'z': board_pose2_data['position']['z']},
                            {'x': board_pose2_data['orientation']['x'], 'y': board_pose2_data['orientation']['y'], 
                             'z': board_pose2_data['orientation']['z'], 'w': board_pose2_data['orientation']['w']}
                        )
                        
                        # 计算运动：A = T_gripper2 * T_gripper1^-1（基座坐标系）
                        T_gripper1_inv = np.linalg.inv(T_gripper1)
                        A = T_gripper2 @ T_gripper1_inv
                        
                        # 计算运动：B = T_board2^-1 * T_board1（相机坐标系）
                        T_board2_inv = np.linalg.inv(T_board2)
                        B = T_board2_inv @ T_board1
                        
                        A_list.append(A)
                        B_list.append(B)
                        
                        self.get_logger().info(f'   运动组 #{len(A_list)}: 已添加')
                        
                    except Exception as e:
                        self.get_logger().warning(f'   运动组 #{idx+1} 处理失败: {str(e)}')
                        continue
                
                if len(A_list) < 2:
                    return jsonify({
                        'success': False, 
                        'error': f'有效运动组不足，至少需要2组，当前只有{len(A_list)}组'
                    })
                
                # 执行姿态法标定（AX=XB问题）
                self.get_logger().info(f'   开始姿态法标定计算（AX=XB方法）...')
                self.get_logger().info(f'   使用 {len(A_list)} 组运动数据')
                
                try:
                    T_camera2gripper = self._solve_pose_based_hand_eye(A_list, B_list)
                    
                    # 计算误差（验证AX=XB约束）
                    errors = []
                    for i in range(len(A_list)):
                        left_side = A_list[i] @ T_camera2gripper
                        right_side = T_camera2gripper @ B_list[i]
                        error_matrix = left_side - right_side
                        pos_error = np.linalg.norm(error_matrix[:3, 3])
                        rot_error = np.linalg.norm(error_matrix[:3, :3], 'fro')
                        combined_error = pos_error + rot_error * 100.0
                        errors.append(combined_error)
                    
                    mean_error = np.mean(errors)
                    max_error = np.max(errors)
                    min_error = np.min(errors)
                    std_error = np.std(errors)
                    
                    self.get_logger().info('✅ Eye-in-Hand姿态法标定完成')
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
                        'method': 'Pose-Based (AX=XB)',
                        'calibration_type': 'Eye-in-Hand',
                        'transformation_matrix': T_camera2gripper.tolist(),
                        'rotation_matrix': R_cam2gripper.tolist(),
                        'translation_vector': t_cam2gripper.tolist(),
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
                            'data_count': len(A_list),
                            'errors_per_motion': [float(e) for e in errors]
                        },
                        'message': f'Eye-in-Hand姿态法标定成功，平均误差: {mean_error:.3f} mm'
                    })
                    
                except Exception as e:
                    self.get_logger().error(f'❌ 姿态法标定计算失败: {str(e)}')
                    import traceback
                    self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                    return jsonify({
                        'success': False, 
                        'error': f'标定计算失败: {str(e)}'
                    })
                    
            except Exception as e:
                self.get_logger().error(f'❌ Eye-in-Hand姿态法标定失败: {str(e)}')
                import traceback
                self.get_logger().error(f'   堆栈: {traceback.format_exc()}')
                return jsonify({'success': False, 'error': str(e)})
        
        def _solve_pose_based_hand_eye(self, A_list, B_list):
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
            
            # 提取旋转和平移
            Ra_list = [A[:3, :3] for A in A_list]
            ta_list = [A[:3, 3] for A in A_list]
            Rb_list = [B[:3, :3] for B in B_list]
            tb_list = [B[:3, 3] for B in B_list]
            
            # 步骤1：求解旋转部分（使用非线性优化）
            def rotation_residual(rx_params):
                """旋转残差函数"""
                angle = np.linalg.norm(rx_params)
                if angle < 1e-6:
                    Rx = np.eye(3)
                else:
                    axis = rx_params / angle
                    r = R.from_rotvec(axis * angle)
                    Rx = r.as_matrix()
                
                residual = []
                for i in range(n):
                    res = Ra_list[i] @ Rx - Rx @ Rb_list[i]
                    residual.extend(res.flatten())
                return np.array(residual)
            
            initial_rx = np.array([0.0, 0.0, 0.0])
            
            self.get_logger().info(f'   步骤1: 求解旋转部分（初始估计）...')
            result_rx = least_squares(rotation_residual, initial_rx, method='lm', 
                                     max_nfev=1000, ftol=1e-8, xtol=1e-8, verbose=0)
            
            # 提取旋转矩阵
            rx_params = result_rx.x
            angle = np.linalg.norm(rx_params)
            if angle < 1e-6:
                Rx = np.eye(3)
            else:
                axis = rx_params / angle
                r = R.from_rotvec(axis * angle)
                Rx = r.as_matrix()
            
            self.get_logger().info(f'   初始旋转求解完成，旋转角度: {np.degrees(angle):.3f} deg')
            
            # 步骤2：求解平移部分（线性系统）
            A_mat = []
            b_vec = []
            for i in range(n):
                A_block = np.eye(3) - Ra_list[i]
                b_block = Rx @ tb_list[i] - ta_list[i]
                A_mat.append(A_block)
                b_vec.append(b_block)
            
            A_mat = np.vstack(A_mat)
            b_vec = np.hstack(b_vec)
            
            tx, residuals, rank, s = np.linalg.lstsq(A_mat, b_vec, rcond=None)
            
            self.get_logger().info(f'   步骤2: 平移部分求解完成')
            
            # 步骤3：非线性优化（进一步优化结果）
            def full_residual(params):
                """完整约束的残差函数"""
                axis_angle = params[:3]
                translation = params[3:6]
                
                angle = np.linalg.norm(axis_angle)
                if angle < 1e-6:
                    R_mat = np.eye(3)
                else:
                    axis = axis_angle / angle
                    r = R.from_rotvec(axis * angle)
                    R_mat = r.as_matrix()
                
                T = np.eye(4)
                T[:3, :3] = R_mat
                T[:3, 3] = translation
                
                residual = []
                for i in range(n):
                    left = A_list[i] @ T
                    right = T @ B_list[i]
                    error = left - right
                    residual.extend(error.flatten())
                
                return np.array(residual)
            
            r_init = R.from_matrix(Rx)
            axis_angle_init = r_init.as_rotvec()
            initial_params = np.concatenate([axis_angle_init, tx])
            
            self.get_logger().info(f'   步骤3: 非线性优化（进一步优化）...')
            result_full = least_squares(full_residual, initial_params, method='lm',
                                       max_nfev=10000, ftol=1e-8, xtol=1e-8, verbose=0)
            
            # 提取优化后的结果
            axis_angle_opt = result_full.x[:3]
            translation_opt = result_full.x[3:6]
            
            angle_opt = np.linalg.norm(axis_angle_opt)
            if angle_opt < 1e-6:
                R_opt = np.eye(3)
            else:
                axis_opt = axis_angle_opt / angle_opt
                r_opt = R.from_rotvec(axis_opt * angle_opt)
                R_opt = r_opt.as_matrix()
            
            T_camera2gripper = np.eye(4)
            T_camera2gripper[:3, :3] = R_opt
            T_camera2gripper[:3, 3] = translation_opt
            
            mean_residual = np.mean(np.abs(result_full.fun))
            self.get_logger().info(f'   优化完成，平均残差: {mean_residual:.6f} mm')
            
            return T_camera2gripper
        
        @self.app.route('/api/hand_eye/save_calibration', methods=['POST'])
        def save_hand_eye_calibration():
            """生成手眼标定结果XML内容供下载"""
            try:
                data = request.get_json()
                
                # 检查是否有标定结果
                if self.hand_eye_calibration_data.get('transformation_matrix') is None:
                    return jsonify({
                        'success': False,
                        'error': '没有可保存的标定结果，请先完成标定'
                    })
                
                self.get_logger().info('='*60)
                self.get_logger().info(f'📥 生成手眼标定结果XML文件')
                
                # 构建XML内容
                xml_content = self._build_calibration_xml(data)
                
                self.get_logger().info(f'✅ XML内容已生成，准备下载')
                self.get_logger().info(f'   文件大小: {len(xml_content)} bytes')
                self.get_logger().info('='*60)
                
                return jsonify({
                    'success': True,
                    'xml_content': xml_content,
                    'message': '标定结果XML已生成'
                })
                
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
            
            # 详细日志：接收到ImageData
            self.get_logger().info('='*60)
            self.get_logger().info(f'📸 接收到ImageData消息')
            self.get_logger().info(f'   相机ID: {msg.camera_id}')
            self.get_logger().info(f'   时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            self.get_logger().info(f'   图像尺寸: {image_msg.width} x {image_msg.height}')
            self.get_logger().info(f'   图像编码: {image_msg.encoding}')
            self.get_logger().info(f'   数据大小: {len(image_msg.data)} bytes ({len(image_msg.data)/1024:.1f} KB)')
            
            # 检查编码类型
            if image_msg.encoding == 'jpeg' or image_msg.encoding == 'jpg':
                # JPEG编码，直接解码
                self.get_logger().info(f'   检测到JPEG编码，使用imdecode解码...')
                np_arr = np.frombuffer(bytes(image_msg.data), np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 其他编码，使用cv_bridge
                self.get_logger().info(f'   使用cv_bridge转换...')
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
            if image is not None:
                self.current_image = image
                self.current_image_raw = image.copy()  # 保存副本用于处理
                
                self.get_logger().info(f'✅ 图像转换成功!')
                self.get_logger().info(f'   OpenCV格式: {image.shape} (H x W x C)')
                self.get_logger().info(f'   数据类型: {image.dtype}')
                self.get_logger().info(f'   内存大小: {image.nbytes / 1024 / 1024:.2f} MB')
                self.get_logger().info(f'   像素范围: [{image.min()}, {image.max()}]')
                self.get_logger().info('='*60)
            else:
                self.get_logger().warn('⚠️ 图像转换结果为None')
        except Exception as e:
            self.get_logger().error(f'❌ 图像转换失败: {str(e)}')
            self.get_logger().error(f'   异常类型: {type(e).__name__}')
            import traceback
            self.get_logger().error(f'   堆栈信息: {traceback.format_exc()}')
    
    def robot_status_callback(self, msg):
        """机器人状态回调函数"""
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        import json
        import time
        log_file.write(json.dumps({'id': 'log_callback_entry', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:1090', 'message': 'robot_status_callback被调用', 'data': {'msg_type': type(msg).__name__, 'has_cartesian': hasattr(msg, 'cartesian_position')}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'B'}) + '\n')
        log_file.close()
        # #endregion
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        has_cartesian = hasattr(msg, 'cartesian_position')
        cartesian_info = {}
        if has_cartesian:
            cart = msg.cartesian_position
            cartesian_info = {'has_position': hasattr(cart, 'position'), 'has_orientation': hasattr(cart, 'orientation')}
            if hasattr(cart, 'position'):
                pos = cart.position
                cartesian_info['position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
            if hasattr(cart, 'orientation'):
                ori = cart.orientation
                cartesian_info['orientation'] = {'x': ori.x, 'y': ori.y, 'z': ori.z, 'w': ori.w}
        log_file.write(json.dumps({'id': 'log_callback_before_set', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:1093', 'message': '设置前检查消息内容', 'data': cartesian_info, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'B'}) + '\n')
        log_file.close()
        # #endregion
        self.current_robot_status = msg  # 保存完整的机器人状态
        self.current_robot_pose = msg.cartesian_position  # 保持兼容性
        # #region agent log
        log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
        log_file.write(json.dumps({'id': 'log_callback_after_set', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:1096', 'message': '设置后状态', 'data': {'current_robot_status_set': self.current_robot_status is not None, 'current_robot_pose_set': self.current_robot_pose is not None}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'B'}) + '\n')
        log_file.close()
        # #endregion
    
    def camera_status_callback(self, msg):
        """相机状态回调函数"""
        self.current_camera_status = msg
    
    def camera_info_callback(self, msg):
        """相机内参回调函数 - 从 CameraInfo 话题获取相机内参"""
        try:
            # 保存 CameraInfo 消息
            self.current_camera_info = msg
            
            # 提取相机内参矩阵 K (3x3)
            # K = [fx  0 cx]
            #     [ 0 fy cy]
            #     [ 0  0  1]
            camera_matrix = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ], dtype=np.float64)
            
            # 提取畸变系数
            dist_coeffs = np.array(msg.d, dtype=np.float64)
            
            # 更新到 calib_utils（如果还没有从文件加载）
            if self.calib_utils.camera_matrix is None:
                self.calib_utils.camera_matrix = camera_matrix
                self.calib_utils.dist_coeffs = dist_coeffs
                self.get_logger().info(
                    f'📷 从 CameraInfo 话题获取相机内参: '
                    f'fx={camera_matrix[0,0]:.2f}, fy={camera_matrix[1,1]:.2f}, '
                    f'cx={camera_matrix[0,2]:.2f}, cy={camera_matrix[1,2]:.2f}',
                    throttle_duration_sec=10.0
                )
            
            # 同时更新到 camera_calibration_data
            if self.camera_calibration_data['camera_matrix'] is None:
                self.camera_calibration_data['camera_matrix'] = camera_matrix
                self.camera_calibration_data['dist_coeffs'] = dist_coeffs
                
        except Exception as e:
            self.get_logger().error(f'处理 CameraInfo 消息失败: {str(e)}')
    
    def depth_image_callback(self, msg):
        """深度图回调函数 - 保存当前深度图用于深度误差检查"""
        try:
            # 将ROS图像消息转换为OpenCV格式（16位深度图）
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_depth_image = cv_image
            # #region agent log
            log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
            import json
            import time
            log_file.write(json.dumps({'id': 'log_depth_image_received', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:1633', 'message': '收到深度图数据', 'data': {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding, 'cv_shape': cv_image.shape if cv_image is not None else None, 'cv_dtype': str(cv_image.dtype) if cv_image is not None else None}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'A'}) + '\n')
            log_file.close()
            # #endregion
            self.get_logger().info(f'📦 收到深度图: {msg.width}x{msg.height}, 编码: {msg.encoding}', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {str(e)}')
            # #region agent log
            log_file = open('/home/mu/IVG/.cursor/debug.log', 'a')
            import json
            import time
            log_file.write(json.dumps({'id': 'log_depth_image_error', 'timestamp': int(time.time()*1000), 'location': 'hand_eye_calibration_node.py:1645', 'message': '深度图转换失败', 'data': {'error': str(e)}, 'sessionId': 'debug-session', 'runId': 'run1', 'hypothesisId': 'A'}) + '\n')
            log_file.close()
            # #endregion
    
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

