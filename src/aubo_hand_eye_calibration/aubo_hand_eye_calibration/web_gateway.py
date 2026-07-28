# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Local-only minimal HTTP gateway for the calibration ROS API."""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from aubo_msgs.action import RunHandEyeCalibration
from aubo_msgs.msg import IOState, JointStatus, RobotStatus
from aubo_msgs.srv import ActivateHandEyeCalibration
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .storage import default_storage_directory, load_candidate


# 权威关节顺序 (与 ros2_control xacro / io 控制器 joint_status 数组一致)
_JOINT_NAMES = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)


class WebGateway(Node):
    def __init__(self):
        super().__init__('hand_eye_web_gateway')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8088)
        # robot 区段 TF 坐标系 (与标定服务端参数同默认值)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wrist_frame', 'wrist3_Link')
        self.declare_parameter('camera_root_frame', 'camera_link')
        self._action = ActionClient(
            self, RunHandEyeCalibration,
            '/hand_eye_calibration_server/run')
        self._activate = self.create_client(
            ActivateHandEyeCalibration,
            '/hand_eye_calibration_server/activate')
        self._lock = threading.Lock()
        self._status = {'stage': 'starting', 'detail': 'waiting for server'}
        self._status_version = 0
        self._preview = None
        self._preview_times = []
        self._goal_pending = False
        self._goal_handle = None
        # robot 区段: 10Hz 定时器组包, 话题无数据时对应字段保持 None
        self._robot = {
            'pose': None,
            'camera_pose': None,
            'robot_status': None,
            'joints': None,
            'safety': None,
            'diagnostics': None,
        }
        self._robot_status_msg = None
        self._joint_status_msg = None
        self._io_state_msg = None
        self._diagnostics_msg = None
        self._joint_state_msg = None
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            '/hand_eye_calibration_server/status',
            self._status_callback,
            status_qos,
        )
        self.create_subscription(
            CompressedImage,
            '/hand_eye_calibration_server/preview/compressed',
            self._preview_callback,
            qos_profile_sensor_data,
        )
        # io 控制器状态话题 (mock/sim 无该控制器时回调不触发, 字段保持 None)
        self.create_subscription(
            RobotStatus, '/aubo_io_controller/robot_status',
            self._robot_status_callback, 10)
        self.create_subscription(
            JointStatus, '/aubo_io_controller/joint_status',
            self._joint_status_callback, 10)
        self.create_subscription(
            IOState, '/aubo_io_controller/io_states',
            self._io_state_callback, 10)
        self.create_subscription(
            DiagnosticArray, '/diagnostics',
            self._diagnostics_callback, 10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, qos_profile_sensor_data)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self._update_robot)
        self._http = None

    # ------------------------------------------------------------------
    # ROS 订阅与共享状态
    # ------------------------------------------------------------------
    def _status_callback(self, message):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            status = {'stage': 'unknown', 'detail': message.data}
        with self._lock:
            self._status = status
            self._status_version += 1

    def _preview_callback(self, message):
        now = time.monotonic()
        with self._lock:
            self._preview = bytes(message.data)
            self._preview_times.append(now)

    def _robot_status_callback(self, message):
        with self._lock:
            self._robot_status_msg = message

    def _joint_status_callback(self, message):
        with self._lock:
            self._joint_status_msg = message

    def _io_state_callback(self, message):
        with self._lock:
            self._io_state_msg = message

    def _diagnostics_callback(self, message):
        with self._lock:
            self._diagnostics_msg = message

    def _joint_state_callback(self, message):
        with self._lock:
            self._joint_state_msg = message

    def status(self):
        """SSE/GET /api/status 的全量快照: {calib, robot} 两区段。"""
        with self._lock:
            snapshot = {
                'calib': dict(self._status),
                'robot': dict(self._robot),
            }
            return snapshot, self._status_version

    def robot(self):
        with self._lock:
            return dict(self._robot)

    # ------------------------------------------------------------------
    # robot 区段组包 (10Hz 定时器, 任何失败都只落 None, 绝不抛异常)
    # ------------------------------------------------------------------
    def _tf_pose(self, parent, child):
        """base_frame→child 的 {xyz, rpy_deg, quat_xyzw}; 查不到返回 None。"""
        try:
            transform = self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        rpy = Rotation.from_quat(quaternion).as_euler(
            'xyz', degrees=True).tolist()
        return {
            'xyz': [translation.x, translation.y, translation.z],
            'rpy_deg': rpy,
            'quat_xyzw': quaternion,
        }

    def _update_robot(self):
        try:
            with self._lock:
                robot_status_msg = self._robot_status_msg
                joint_status_msg = self._joint_status_msg
                io_state_msg = self._io_state_msg
                diagnostics_msg = self._diagnostics_msg
                joint_state_msg = self._joint_state_msg

            base_frame = self.get_parameter('base_frame').value
            section = {
                'pose': self._tf_pose(
                    base_frame, self.get_parameter('wrist_frame').value),
                'camera_pose': self._tf_pose(
                    base_frame,
                    self.get_parameter('camera_root_frame').value),
                'robot_status': self._robot_status_section(robot_status_msg),
                'joints': self._joints_section(
                    joint_state_msg, joint_status_msg),
                'safety': self._safety_section(io_state_msg),
                'diagnostics': self._diagnostics_section(diagnostics_msg),
            }
            with self._lock:
                self._robot = section
                self._status_version += 1
        except Exception as error:  # 组包兜底: 状态展示绝不影响标定链路
            self.get_logger().warning(
                f'robot section update failed: {error}')

    @staticmethod
    def _to_int(value):
        # rosidl Python 的 int8/uint8 以单字节 bytes 传递, 需先解包
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder='big', signed=True)
        return int(value)

    @staticmethod
    def _robot_status_section(message):
        if message is None:
            return None
        return {
            'mode': WebGateway._to_int(message.mode),
            'e_stopped': WebGateway._to_int(message.e_stopped),
            'drives_powered': WebGateway._to_int(
                message.drives_powered),
            'motion_possible': WebGateway._to_int(
                message.motion_possible),
            'in_motion': WebGateway._to_int(message.in_motion),
            'in_error': WebGateway._to_int(message.in_error),
            'error_code': WebGateway._to_int(message.error_code),
        }

    @staticmethod
    def _joints_section(joint_state_msg, joint_status_msg):
        if joint_state_msg is None and joint_status_msg is None:
            return None
        # /joint_states 按名索引; joint_status 数组按权威关节顺序按下标
        positions = {}
        velocities = {}
        if joint_state_msg is not None:
            positions = dict(zip(
                joint_state_msg.name, joint_state_msg.position))
            if joint_state_msg.velocity:
                velocities = dict(zip(
                    joint_state_msg.name, joint_state_msg.velocity))

        def _column(msg, name, index):
            values = getattr(msg, name) if msg is not None else None
            if values is None or index >= len(values):
                return None
            return values[index]

        joints = []
        for index, name in enumerate(_JOINT_NAMES):
            joints.append({
                'name': name,
                'position': positions.get(name),
                'velocity': velocities.get(name),
                'current': _column(joint_status_msg, 'current', index),
                'temperature': _column(
                    joint_status_msg, 'temperature', index),
                'following_error': _column(
                    joint_status_msg, 'following_error', index),
                'error_code': _column(joint_status_msg, 'error_code', index),
            })
        return joints

    @staticmethod
    def _safety_section(io_state_msg):
        if io_state_msg is None:
            return None
        states = io_state_msg.safety_in_states
        # io 控制器约定: safety_in_states[0]=急停, [1]=防护停
        return {
            'estop': bool(states[0].state) if len(states) > 0 else None,
            'protective_stop': (
                bool(states[1].state) if len(states) > 1 else None),
        }

    @staticmethod
    def _diagnostics_section(diagnostics_msg):
        if diagnostics_msg is None:
            return None
        return [
            {
                'name': status.name,
                'level': WebGateway._to_int(status.level),
                'message': status.message,
            }
            for status in diagnostics_msg.status
        ]

    def preview(self):
        with self._lock:
            return self._preview

    def preview_fps(self, window=5.0):
        """最近 window 秒内的预览帧率与最后一帧距今年数。"""
        now = time.monotonic()
        with self._lock:
            recent = [t for t in self._preview_times if now - t <= window]
            self._preview_times = recent
            age = (now - recent[-1]) if recent else None
        return {
            'fps': round(len(recent) / window, 1),
            'last_frame_age_s': round(age, 1) if age is not None else None,
        }

    # ------------------------------------------------------------------
    # Action 桥接
    # ------------------------------------------------------------------
    def start(self, plan_only=False, return_to_start=True, method=''):
        if not self._action.server_is_ready():
            if not self._action.wait_for_server(timeout_sec=2.0):
                raise RuntimeError('calibration action server unavailable')
        with self._lock:
            # 占位在锁内同步完成, 杜绝连点造成重复 goal
            if self._goal_pending or self._goal_handle is not None:
                raise RuntimeError('a calibration request is already active')
            self._goal_pending = True
        goal = RunHandEyeCalibration.Goal()
        goal.plan_only = bool(plan_only)
        goal.return_to_start = bool(return_to_start)
        # 空串 = 服务端 solver_method 参数 (默认 auto)
        goal.method = str(method or '')
        future = self._action.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise RuntimeError('calibration goal rejected')
            with self._lock:
                self._goal_handle = goal_handle
                self._goal_pending = False
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result)
        except Exception as error:
            with self._lock:
                self._status = {'stage': 'failed', 'detail': str(error)}
                self._status_version += 1
                self._goal_handle = None
                self._goal_pending = False

    def _goal_result(self, future):
        try:
            result = future.result().result
            with self._lock:
                server_stage = self._status.get('stage', '')
            # 服务器已发布 cancelled 等终态时, 不覆盖其结果语义
            if server_stage in ('cancelled', 'complete', 'quality_failed',
                                'failed'):
                stage = None
            else:
                stage = 'complete' if result.success else 'failed'
            status = {
                'stage': stage,
                'detail': result.message,
                'candidate_id': result.candidate_id,
                'result_file': result.result_file,
                'metrics': {
                    'accepted_samples': result.accepted_samples,
                    'rejected_samples': result.rejected_samples,
                    'translation_rms_m': result.translation_rms_m,
                    'rotation_rms_deg': result.rotation_rms_deg,
                    'reprojection_rms_px': result.reprojection_rms_px,
                },
            }
        except Exception as error:
            status = {'stage': 'failed', 'detail': str(error)}
        with self._lock:
            if status['stage'] is not None:
                self._status = status
                self._status_version += 1
            self._goal_handle = None

    def cancel(self):
        with self._lock:
            goal_handle = self._goal_handle
        if goal_handle is None:
            raise RuntimeError('no active calibration goal')
        goal_handle.cancel_goal_async()

    def activate(self, candidate_id):
        if not self._activate.service_is_ready():
            if not self._activate.wait_for_service(timeout_sec=2.0):
                raise RuntimeError('activation service unavailable')
        request = ActivateHandEyeCalibration.Request()
        request.candidate_id = str(candidate_id)
        future = self._activate.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('activation request timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return {
            'success': True,
            'message': response.message,
            'active_result_file': response.active_result_file,
        }

    # ------------------------------------------------------------------
    # 候选结果查询
    # ------------------------------------------------------------------
    @staticmethod
    def _storage_dir():
        return default_storage_directory()

    def list_candidates(self):
        directory = self._storage_dir() / 'candidates'
        active_id = None
        active_path = self._storage_dir() / 'active.yaml'
        if active_path.is_file():
            try:
                with active_path.open(encoding='utf-8') as stream:
                    active_id = yaml.safe_load(stream).get('candidate_id')
            except (OSError, yaml.YAMLError, AttributeError):
                active_id = None
        entries = []
        if directory.is_dir():
            for path in sorted(directory.glob('*.yaml'), reverse=True):
                try:
                    _, document = load_candidate(path.stem)
                    entries.append({
                        'candidate_id': document['candidate_id'],
                        'created_at': document.get('created_at', ''),
                        'quality_passed': bool(
                            document.get('quality_passed', False)),
                        'method': document.get('method', ''),
                        'metrics': document.get('metrics', {}),
                        'failures': document.get('failures', []),
                        'active': document['candidate_id'] == active_id,
                    })
                except (OSError, KeyError, TypeError, ValueError,
                        yaml.YAMLError) as error:
                    entries.append({
                        'candidate_id': path.stem,
                        'error': str(error),
                    })
        return entries

    def candidate_detail(self, candidate_id):
        _, document = load_candidate(candidate_id)
        return document

    def active_detail(self):
        active_path = self._storage_dir() / 'active.yaml'
        if not active_path.is_file():
            return None
        with active_path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    # ------------------------------------------------------------------
    # HTTP 服务
    # ------------------------------------------------------------------
    def start_http(self):
        gateway = self
        index_path = Path(get_package_share_directory(
            'aubo_hand_eye_calibration')) / 'web' / 'index.html'

        class Handler(BaseHTTPRequestHandler):
            server_version = 'AuboHandEye/2.0'
            protocol_version = 'HTTP/1.1'

            def log_message(self, format_string, *args):
                gateway.get_logger().info(format_string % args)

            def _json(self, status, document):
                data = json.dumps(document).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _body(self):
                try:
                    length = int(self.headers.get('Content-Length') or '0')
                except ValueError:
                    return {}
                if not length:
                    return {}
                return json.loads(self.rfile.read(length).decode('utf-8'))

            def do_GET(self):
                path = urlparse(self.path).path
                try:
                    if path == '/':
                        data = index_path.read_bytes()
                        self.send_response(HTTPStatus.OK)
                        self.send_header(
                            'Content-Type', 'text/html; charset=utf-8')
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        self.wfile.write(data)
                    elif path == '/api/status':
                        status, _ = gateway.status()
                        self._json(HTTPStatus.OK, status)
                    elif path == '/api/robot':
                        self._json(HTTPStatus.OK, gateway.robot())
                    elif path == '/api/events':
                        self._events()
                    elif path == '/api/candidates':
                        self._json(HTTPStatus.OK, {
                            'candidates': gateway.list_candidates()})
                    elif path.startswith('/api/candidates/'):
                        candidate_id = path.rsplit('/', 1)[-1]
                        self._json(
                            HTTPStatus.OK,
                            gateway.candidate_detail(candidate_id))
                    elif path == '/api/active':
                        self._json(
                            HTTPStatus.OK,
                            {'active': gateway.active_detail()})
                    elif path == '/api/fps':
                        self._json(HTTPStatus.OK, gateway.preview_fps())
                    elif path == '/preview.jpg':
                        data = gateway.preview()
                        if data is None:
                            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE)
                            return
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        self.wfile.write(data)
                    elif path == '/preview.mjpeg':
                        self._mjpeg()
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except (OSError, KeyError, TypeError, ValueError,
                        yaml.YAMLError) as error:
                    self._json(HTTPStatus.BAD_REQUEST,
                               {'success': False, 'message': str(error)})

            def _events(self):
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last_version = -1
                last_beat = time.monotonic()
                try:
                    while True:
                        status, version = gateway.status()
                        if version != last_version:
                            last_version = version
                            payload = json.dumps(status)
                            self.wfile.write(
                                f'data: {payload}\n\n'.encode('utf-8'))
                            self.wfile.flush()
                            last_beat = time.monotonic()
                        elif time.monotonic() - last_beat > 10.0:
                            self.wfile.write(b': heartbeat\n\n')
                            self.wfile.flush()
                            last_beat = time.monotonic()
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _mjpeg(self):
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last = None
                try:
                    while True:
                        data = gateway.preview()
                        if data is not None and data is not last:
                            last = data
                            header = (
                                b'--frame\r\nContent-Type: image/jpeg\r\n'
                                + f'Content-Length: {len(data)}\r\n\r\n'
                                .encode('ascii'))
                            self.wfile.write(header)
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                path = urlparse(self.path).path
                try:
                    body = self._body()
                    if path in ('/api/preflight', '/api/plan'):
                        gateway.start(
                            plan_only=True,
                            method=body.get('method', ''),
                        )
                        result = {'success': True, 'message': 'plan requested'}
                    elif path == '/api/run':
                        gateway.start(
                            plan_only=False,
                            return_to_start=body.get('return_to_start', True),
                            method=body.get('method', ''),
                        )
                        result = {
                            'success': True,
                            'message': 'calibration requested',
                        }
                    elif path == '/api/cancel':
                        gateway.cancel()
                        result = {
                            'success': True,
                            'message': 'cancel requested',
                        }
                    elif path == '/api/activate':
                        result = gateway.activate(body.get('candidate_id', ''))
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    self._json(HTTPStatus.OK, result)
                except (json.JSONDecodeError, OSError, RuntimeError,
                        ValueError) as error:
                    self._json(
                        HTTPStatus.BAD_REQUEST,
                        {'success': False, 'message': str(error)},
                    )

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        if host not in ('127.0.0.1', 'localhost', '::1'):
            raise ValueError('Web UI is restricted to loopback interfaces')
        self._http = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(
            target=self._http.serve_forever,
            name='hand-eye-http',
            daemon=True,
        )
        thread.start()
        self.get_logger().info(f'Web UI: http://{host}:{port}')

    def destroy_node(self):
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebGateway()
    node.start_http()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
