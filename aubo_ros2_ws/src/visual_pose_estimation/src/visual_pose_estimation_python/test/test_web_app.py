from __future__ import annotations

import json
import sys
import tempfile
import threading
from unittest.mock import patch
from pathlib import Path

import cv2
import numpy as np
from fastapi.testclient import TestClient

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from visual_pose_estimation_python.web.app import create_app
from visual_pose_estimation_python.web.services import NativeWebService
from visual_pose_estimation_python.web.runtime_support import CAMERA_POSE_FIXED_ORIENTATION


class DummyRosBridge:
    def __init__(self):
        self.is_ready = True
        self.startup_error = None
        self.templates_dir = Path(tempfile.mkdtemp(prefix="vpe_templates_"))
        self.pose_list_dir = Path(tempfile.mkdtemp(prefix="vpe_pose_list_"))
        self.camera_pose_fixed_orientation = CAMERA_POSE_FIXED_ORIENTATION
        self.node = DummyNode()

    def start(self):
        return None

    def stop(self):
        return None

    def status(self):
        return {
            "ready": self.is_ready,
            "startup_error": self.startup_error,
            "bridge_module": "dummy",
        }


class DummyNode:
    def __init__(self):
        self.image_lock = threading.Lock()
        self.latest_depth_image = None
        self.latest_color_image = None
        self.depth_image_received = False
        self.color_image_received = False
        self.debug_last_capture_error = None
        self.preprocessor = DummyPreprocessor()
        self.feature_extractor = DummyFeatureExtractor()
        self.config_reader = DummyConfigReader()
        self.params_manager = DummyParamsManager()
        self._robot_status = {
            "joint_position_deg": [0, 0, 0, 0, 0, 0],
            "in_motion": False,
            "cartesian_position": {
                "position": {"x": 1, "y": 2, "z": 3},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
            },
        }

    def capture_image(self, camera_id, timeout=10.0):
        depth_image = np.full((4, 4), 42, dtype=np.uint16)
        color_image = np.full((4, 4, 3), 128, dtype=np.uint8)
        with self.image_lock:
            self.latest_depth_image = depth_image.copy()
            self.latest_color_image = color_image.copy()
            self.depth_image_received = True
            self.color_image_received = True
        return depth_image, color_image, None

    def estimate_pose(self, depth_image, color_image, object_id):
        return {
            "success": True,
            "success_num": 1,
            "object_id": object_id,
            "depth_len": len(depth_image),
            "color_len": len(color_image),
        }, None

    def list_templates(self, workpiece_id="", timeout=10.0):
        if workpiece_id:
            return {"workpiece_ids": [workpiece_id]}, None
        return {"workpiece_ids": ["demo_workpiece"]}, None

    def standardize_template(self, workpiece_id):
        return {"success": True, "workpiece_id": workpiece_id}, None

    def get_robot_status(self, timeout=5.0):
        return self._robot_status

    def move_to_pose(self, target_pose, use_joints=False, velocity_factor=0.08, acceleration_factor=0.05, timeout=180.0):
        if use_joints and isinstance(target_pose, list):
            self._robot_status["joint_position_deg"] = list(target_pose)
        return {"success": True, "message": "ok", "error_code": 0}, None

    def set_robot_io(self, io_type, io_index, value, timeout=10.0):
        return {"success": True, "message": f"{io_type}:{io_index}={value}", "error_code": 0}, None

    def call_execute_single_grasp(self, object_id, use_visual_estimation, timeout=300.0):
        class Point:
            x = 1.1
            y = 2.2
            z = 3.3

        class Orientation:
            x = 0.1
            y = 0.2
            z = 0.3
            w = 0.4

        class Result:
            success = True
            message = f"grasp:{object_id}:{use_visual_estimation}"
            final_position = Point()
            final_orientation = Orientation()

        return Result()

    def call_loop_grasp_control(self, start, timeout=10.0):
        class Result:
            success = True
            message = "started" if start else "stopped"

        return Result()

    def call_publish_grasps_loop_control(self, start, timeout=10.0):
        class Result:
            success = True
            message = "publish-started" if start else "publish-stopped"

        return Result()

    def call_run_gripper_swap(self, direction, timeout=None):
        class Result:
            success = True
            message = f"swap:{direction}"

        return Result()

    def notify_params_updated(self):
        return None


class DummyConfigReader:
    def __init__(self):
        self.params = {
            "binary_threshold_min": 0,
            "binary_threshold_max": 100,
            "component_min_area": 10,
            "component_max_area": 100000,
            "component_min_aspect_ratio": 0.3,
            "component_max_aspect_ratio": 4.0,
            "component_min_width": 60,
            "component_min_height": 60,
            "component_max_count": 3,
            "enable_zero_interp": True,
            "enable_smooth_edges": True,
            "smooth_edges_blur_sigma": 0,
            "use_rembg": False,
        }

    def load_debug_thresholds(self):
        return dict(self.params)


class DummyParamsManager:
    def __init__(self):
        self.config_path = "/tmp/debug_thresholds.json"
        self.values = {}

    def update(self, key, value):
        self.values[key] = value
        return True

    def save(self):
        return True


class DummyPreprocessor:
    def __init__(self):
        self.params = {}

    def set_parameters(self, params):
        self.params.update(params)

    def preprocess(self, depth_image, color_image, binary_threshold_min, binary_threshold_max):
        component = np.zeros(depth_image.shape, dtype=np.uint8)
        component[1:3, 1:3] = 255
        return [component], color_image.copy()


class DummyFeature:
    workpiece_center = (2.0, 2.0)
    workpiece_radius = 1.0
    valve_center = (2.0, 2.0)
    valve_radius = 0.5
    standardized_angle_deg = 45.0


class DummyFeatureExtractor:
    def __init__(self):
        self.params = {}

    def set_parameters(self, params):
        self.params.update(params)

    def extract_features(self, components, preprocessed_color):
        return [DummyFeature()]


def create_test_client():
    app = create_app()
    app.state.ros_bridge = DummyRosBridge()
    app.state.native_service = NativeWebService(app.state.ros_bridge)
    return TestClient(app)


def test_health_endpoint():
    with create_test_client() as client:
        response = client.get("/health")

    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert response.json()["ros_bridge_ready"] is True


def test_status_endpoint():
    with create_test_client() as client:
        response = client.get("/status")

    payload = response.json()
    assert response.status_code == 200
    assert payload["status"] == "online"
    assert payload["service"] == "visual_pose_estimation_fastapi_web"
    assert payload["ros_bridge"]["ready"] is True


def test_index_alias_exists():
    with create_test_client() as client:
        response = client.get("/index.html")

    assert response.status_code == 200
    assert "兼容界面" in response.text


def test_native_estimate_pose_route():
    with create_test_client() as client:
        response = client.post(
            "/api/estimate_pose",
            json={
                "object_id": "demo",
                "color_image": "data:image/jpeg;base64,abc",
                "depth_image": "data:image/png;base64,xyz",
            },
        )

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["object_id"] == "demo"


def test_native_capture_image_route():
    with create_test_client() as client:
        response = client.post("/api/capture_image", json={"camera_id": "cam_01"})

    payload = response.json()
    assert response.status_code == 200
    assert payload["success"] is True
    assert payload["depth_image_base64"].startswith("data:image/png;base64,")
    assert payload["color_image_base64"].startswith("data:image/jpeg;base64,")


def test_native_template_routes():
    with create_test_client() as client:
        templates_dir = client.app.state.ros_bridge.templates_dir
        pose_dir = templates_dir / "demo_workpiece" / "pose_1"
        pose_dir.mkdir(parents=True, exist_ok=True)

        image_path = pose_dir / "image.jpg"
        test_image = np.full((4, 4, 3), 200, dtype=np.uint8)
        cv2.imwrite(str(image_path), test_image)

        pose_json_path = pose_dir / "camera_pose.json"
        pose_json_path.write_text(json.dumps({"cartesian_position": {"position": {"x": 1}}}), encoding="utf-8")

        list_response = client.post("/api/list_templates", json={})
        read_response = client.post(
            "/api/read_template_pose",
            json={"workpiece_id": "demo_workpiece", "pose_id": "1", "pose_type": "camera_pose"},
        )
        image_response = client.get(
            "/api/get_template_image",
            params={"workpiece_id": "demo_workpiece", "pose_id": "1", "image_name": "image.jpg"},
        )
        ids_response = client.post("/api/list_workpiece_ids")

    assert list_response.status_code == 200
    assert list_response.json()["count"] == 1
    assert list_response.json()["templates"][0]["workpiece_id"] == "demo_workpiece"
    assert read_response.status_code == 200
    assert read_response.json()["pose_data"]["cartesian_position"]["position"]["x"] == 1
    assert image_response.status_code == 200
    assert image_response.headers["content-type"] == "image/jpeg"
    assert ids_response.status_code == 200
    assert ids_response.json()["workpiece_ids"] == ["demo_workpiece"]


def test_native_save_template_pose_and_standardize():
    with create_test_client() as client:
        save_response = client.post(
            "/api/save_template_pose",
            json={
                "workpiece_id": "demo_workpiece",
                "pose_id": "2",
                "pose_type": "camera_pose",
                "robot_status": {
                    "cartesian_position": {
                        "position": {"x": 1, "y": 2, "z": 3},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                    }
                },
            },
        )
        standardize_response = client.post("/api/standardize_template", json={"workpiece_id": "demo_workpiece"})

        saved_json = (
            client.app.state.ros_bridge.templates_dir / "demo_workpiece" / "pose_2" / "camera_pose.json"
        ).read_text(encoding="utf-8")

    saved_payload = json.loads(saved_json)
    assert save_response.status_code == 200
    assert save_response.json()["success"] is True
    assert saved_payload["cartesian_position"]["orientation"]["x"] == CAMERA_POSE_FIXED_ORIENTATION["orientation"]["x"]
    assert standardize_response.status_code == 200
    assert standardize_response.json()["workpiece_id"] == "demo_workpiece"


def test_native_robot_routes():
    with create_test_client() as client:
        status_response = client.post("/api/get_robot_status")
        pose_response = client.post(
            "/api/set_robot_pose",
            json={
                "target_pose": {
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "use_joints": False,
            },
        )
        io_response = client.post("/api/set_robot_io", json={"io_type": "DO", "io_index": 1, "value": 1})

    assert status_response.status_code == 200
    assert status_response.json()["robot_status"]["joint_position_deg"] == [0, 0, 0, 0, 0, 0]
    assert pose_response.status_code == 200
    assert pose_response.json()["success"] is True
    assert io_response.status_code == 200
    assert io_response.json()["success"] is True


def test_native_execute_pose_sequence_route():
    with create_test_client() as client:
        pose_list_dir = client.app.state.ros_bridge.pose_list_dir / "demo_folder"
        pose_list_dir.mkdir(parents=True, exist_ok=True)
        (pose_list_dir / "001.json").write_text(json.dumps({"joint_position_deg": [1, 2, 3, 4, 5, 6]}), encoding="utf-8")
        (pose_list_dir / "002.json").write_text(json.dumps({"joint_position_deg": [6, 5, 4, 3, 2, 1]}), encoding="utf-8")

        response = client.post("/api/execute_pose_sequence", json={"folder_name": "demo_folder"})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["skipped"] is False
    assert response.json()["total_files"] == 2


def test_native_grasp_routes():
    with create_test_client() as client:
        single_response = client.post(
            "/api/execute_single_grasp",
            json={"object_id": "demo_part", "use_visual_estimation": True},
        )
        loop_response = client.post("/api/loop_grasp_control", json={"data": True})
        publish_response = client.post("/api/publish_grasps_loop_control", json={"data": False})
        swap_response = client.post("/api/run_gripper_swap", json={"direction": "gripper0_to_gripper2"})

    assert single_response.status_code == 200
    assert single_response.json()["success"] is True
    assert single_response.json()["message"] == "grasp:demo_part:True"
    assert loop_response.status_code == 200
    assert loop_response.json()["message"] == "started"
    assert publish_response.status_code == 200
    assert publish_response.json()["message"] == "publish-stopped"
    assert swap_response.status_code == 200
    assert swap_response.json()["direction"] == "gripper0_to_gripper2"


def test_native_debug_routes():
    with create_test_client() as client:
        capture_response = client.post("/api/debug/capture", json={"camera_id": "cam_01"})
        images_response = client.post("/api/debug/get_images")
        update_response = client.post(
            "/api/debug/update_params",
            json={"param_name": "min-depth", "param_value": 12},
        )
        params_response = client.post("/api/debug/get_params")
        save_response = client.post("/api/debug/save_thresholds")
        features_response = client.post(
            "/api/save_debug_features",
            json={"session_id": "session_debug", "features": [{"id": 1, "ub": 2}]},
        )

    assert capture_response.status_code == 200
    assert capture_response.json()["success"] is True
    assert images_response.status_code == 200
    assert images_response.json()["has_images"] is True
    assert images_response.json()["stats"]["feature_count"] == 1
    assert update_response.status_code == 200
    assert update_response.json()["success"] is True
    assert params_response.status_code == 200
    assert params_response.json()["params"]["binary_threshold_min"] == 0
    assert save_response.status_code == 200
    assert save_response.json()["success"] is True
    assert features_response.status_code == 200
    assert features_response.json()["feature_count"] == 1


def test_exit_endpoint_schedules_shutdown():
    with patch("visual_pose_estimation_python.web.routers.system.schedule_exit") as schedule_exit_mock:
        with create_test_client() as client:
            response = client.post("/exit")

    assert response.status_code == 200
    assert response.json()["status"] == "success"
    schedule_exit_mock.assert_called_once_with()


def test_websocket_status_and_ping():
    with create_test_client() as client:
        with client.websocket_connect("/ws") as websocket:
            initial_message = websocket.receive_json()
            websocket.send_json({"action": "ping"})
            pong_message = websocket.receive_json()

    assert initial_message["type"] == "status"
    assert initial_message["payload"]["ros_bridge"]["ready"] is True
    assert pong_message == {"type": "pong"}
