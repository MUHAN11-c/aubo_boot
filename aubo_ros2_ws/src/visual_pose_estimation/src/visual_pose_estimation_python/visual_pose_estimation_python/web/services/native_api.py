from __future__ import annotations

import base64
import copy
import csv
import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from fastapi import HTTPException

from visual_pose_estimation_python.debug_visualizer import DebugVisualizer

from ..ros_bridge import RosBridgeManager
from ..runtime_support import CAMERA_POSE_FIXED_ORIENTATION, REMBG_AVAILABLE, get_rembg_processor, normalize_pose_rotation


LOGGER = logging.getLogger(__name__)


POSE_TYPE_TO_FILENAME = {
    "camera_pose": "camera_pose.json",
    "preparation_position": "preparation_position.json",
    "grab_position": "grab_position.json",
    "preplace_position": "preplace_position.json",
    "place_position": "place_position.json",
}


class NativeWebService:
    """FastAPI-native service layer for migrated core endpoints."""

    def __init__(self, ros_bridge: RosBridgeManager) -> None:
        self._ros_bridge = ros_bridge

    def capture_image(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        node = self._require_node()
        payload = payload or {}
        camera_id = payload.get("camera_id", "207000152740")

        depth_image, color_image, error_msg = node.capture_image(camera_id)
        if depth_image is None or color_image is None:
            raise HTTPException(status_code=500, detail=error_msg or "图像采集失败")

        _, depth_buffer = cv2.imencode(".png", depth_image)
        _, color_buffer = cv2.imencode(".jpg", color_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

        return {
            "success": True,
            "depth_image_base64": f"data:image/png;base64,{base64.b64encode(depth_buffer).decode('utf-8')}",
            "color_image_base64": f"data:image/jpeg;base64,{base64.b64encode(color_buffer).decode('utf-8')}",
        }

    def capture_template_image(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        workpiece_id = str(payload.get("workpiece_id", "")).strip()
        pose_id = str(payload.get("pose_id", "")).strip()
        camera_id = payload.get("camera_id", "DA3234363")

        if not workpiece_id or not pose_id:
            raise HTTPException(status_code=400, detail="工件ID和姿态ID不能为空")

        depth_image, color_image, error_msg = node.capture_image(camera_id)
        if depth_image is None or color_image is None:
            raise HTTPException(status_code=500, detail=error_msg or "模板图像采集失败")

        template_dir = self.templates_dir / workpiece_id / f"pose_{pose_id}"
        template_dir.mkdir(parents=True, exist_ok=True)

        color_image_path = template_dir / "original_image.jpg"
        depth_image_path = template_dir / "depth_image.png"

        ok_color = cv2.imwrite(str(color_image_path), color_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        ok_depth = cv2.imwrite(str(depth_image_path), depth_image)
        if not ok_color or not ok_depth:
            errors = []
            if not ok_color:
                errors.append("彩色图保存失败")
            if not ok_depth:
                errors.append("深度图保存失败")
            raise HTTPException(status_code=500, detail="，".join(errors))

        _, color_buffer = cv2.imencode(".jpg", color_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        image_base64 = base64.b64encode(color_buffer).decode("utf-8")
        return {
            "success": True,
            "image_path": str(color_image_path),
            "depth_image_path": str(depth_image_path),
            "image_base64": f"data:image/jpeg;base64,{image_base64}",
        }

    def estimate_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()

        depth_image = payload.get("depth_image") or payload.get("depth_image_base64") or ""
        color_image = payload.get("color_image") or payload.get("color_image_base64") or ""
        object_id = str(payload.get("object_id", "")).strip()

        if not depth_image or not color_image:
            raise HTTPException(status_code=400, detail="缺少输入图像（需要depth_image和color_image）")
        if not object_id:
            raise HTTPException(status_code=400, detail="缺少工件ID")

        if "," in depth_image:
            depth_image = depth_image.split(",", 1)[1]
        if "," in color_image:
            color_image = color_image.split(",", 1)[1]

        result, error = node.estimate_pose(depth_image, color_image, object_id)
        if error:
            raise HTTPException(status_code=500, detail=error)
        return result

    def list_templates(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        node = self._require_node()
        payload = payload or {}
        workpiece_id_filter = str(payload.get("workpiece_id", "")).strip()

        result, error = node.list_templates(workpiece_id=workpiece_id_filter, timeout=10.0)
        if error is not None:
            raise HTTPException(status_code=500, detail=error)

        templates_root = self.templates_dir
        if not templates_root.exists() or not templates_root.is_dir():
            raise HTTPException(status_code=500, detail=f"模板根目录不存在: {templates_root}")

        template_list = []
        for current_workpiece_id in result.get("workpiece_ids", []):
            workpiece_entry = templates_root / current_workpiece_id
            if not workpiece_entry.is_dir():
                continue
            for pose_entry in workpiece_entry.iterdir():
                if not pose_entry.is_dir() or not pose_entry.name.startswith("pose_"):
                    continue
                pose_id = pose_entry.name[5:]
                image_path = pose_entry / "image.jpg"
                if not image_path.is_file():
                    continue
                try:
                    with open(image_path, "rb") as file_obj:
                        image_base64 = base64.b64encode(file_obj.read()).decode("utf-8")
                except Exception as exc:
                    LOGGER.warning("无法读取模板图像 %s: %s", image_path, exc)
                    continue

                template_list.append(
                    {
                        "template_id": f"{current_workpiece_id}_{pose_id}",
                        "workpiece_id": current_workpiece_id,
                        "pose_id": pose_id,
                        "image_path": str(image_path),
                        "image_base64": f"data:image/jpeg;base64,{image_base64}",
                    }
                )

        return {"success": True, "templates": template_list, "count": len(template_list)}

    def list_workpiece_ids(self) -> dict[str, Any]:
        templates_dir = self.templates_dir
        workpiece_ids = []
        if templates_dir.exists() and templates_dir.is_dir():
            workpiece_ids = sorted(
                item.name for item in templates_dir.iterdir() if item.is_dir() and not item.name.startswith(".")
            )
        return {"success": True, "workpiece_ids": workpiece_ids, "count": len(workpiece_ids)}

    def read_template_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        workpiece_id = str(payload.get("workpiece_id", "")).strip()
        pose_id = str(payload.get("pose_id", "")).strip()
        pose_type = str(payload.get("pose_type", "")).strip()

        if not workpiece_id or not pose_id or not pose_type:
            raise HTTPException(status_code=400, detail="workpiece_id, pose_id, pose_type不能为空")
        if pose_type not in POSE_TYPE_TO_FILENAME:
            raise HTTPException(status_code=400, detail=f"不支持的姿态类型: {pose_type}")

        pose_dir_name = pose_id if pose_id.startswith("pose_") else f"pose_{pose_id}"
        json_path = self.templates_dir / workpiece_id / pose_dir_name / POSE_TYPE_TO_FILENAME[pose_type]
        if not json_path.exists():
            raise HTTPException(status_code=404, detail=f"模板文件不存在: {json_path}")

        with open(json_path, "r", encoding="utf-8") as file_obj:
            pose_data = json.load(file_obj)
        return {"success": True, "pose_data": pose_data}

    def save_template_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        workpiece_id = str(payload.get("workpiece_id", "")).strip()
        pose_id = str(payload.get("pose_id", "")).strip()
        pose_type = str(payload.get("pose_type", "")).strip()
        robot_status = payload.get("robot_status")

        if not workpiece_id or not pose_id or not pose_type:
            raise HTTPException(status_code=400, detail="工件ID、姿态ID和姿态类型不能为空")
        if robot_status is None:
            raise HTTPException(status_code=400, detail="机器人状态数据不能为空")
        if pose_type not in POSE_TYPE_TO_FILENAME:
            raise HTTPException(status_code=400, detail=f"无效的姿态类型: {pose_type}")

        normalized_status = normalize_pose_rotation(copy.deepcopy(robot_status))
        fixed = self._ros_bridge.camera_pose_fixed_orientation or CAMERA_POSE_FIXED_ORIENTATION
        if fixed and "cartesian_position" in normalized_status:
            cartesian_position = normalized_status["cartesian_position"]
            cartesian_position["orientation"] = dict(
                fixed.get("orientation", CAMERA_POSE_FIXED_ORIENTATION["orientation"])
            )
            cartesian_position["euler_orientation_rpy_rad"] = list(
                fixed.get(
                    "euler_orientation_rpy_rad",
                    CAMERA_POSE_FIXED_ORIENTATION["euler_orientation_rpy_rad"],
                )
            )
            cartesian_position["euler_orientation_rpy_deg"] = list(
                fixed.get(
                    "euler_orientation_rpy_deg",
                    CAMERA_POSE_FIXED_ORIENTATION["euler_orientation_rpy_deg"],
                )
            )

        template_dir = self.templates_dir / workpiece_id / f"pose_{pose_id}"
        template_dir.mkdir(parents=True, exist_ok=True)
        json_path = template_dir / POSE_TYPE_TO_FILENAME[pose_type]
        with open(json_path, "w", encoding="utf-8") as file_obj:
            json.dump(normalized_status, file_obj, indent=2, ensure_ascii=False)
            file_obj.flush()
            os.fsync(file_obj.fileno())

        return {"success": True, "file_path": str(json_path), "pose_type": pose_type}

    def standardize_template(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        workpiece_id = str(payload.get("workpiece_id", "")).strip()
        if not workpiece_id:
            raise HTTPException(status_code=400, detail="工件ID不能为空")

        result, error = node.standardize_template(workpiece_id)
        if error:
            raise HTTPException(status_code=500, detail=error)
        return result

    def get_template_image(self, workpiece_id: str, pose_id: str, image_name: str) -> tuple[bytes, str]:
        if not workpiece_id or not pose_id:
            raise HTTPException(status_code=400, detail="缺少workpiece_id或pose_id参数")

        image_path = self.templates_dir / workpiece_id / f"pose_{pose_id}" / image_name
        if not image_path.exists():
            raise HTTPException(status_code=404, detail="图像文件不存在")

        with open(image_path, "rb") as file_obj:
            image_data = file_obj.read()

        media_type = "image/png" if image_path.suffix.lower() == ".png" else "image/jpeg"
        return image_data, media_type

    def get_robot_status(self) -> dict[str, Any]:
        node = self._require_node()
        robot_status = node.get_robot_status()
        if robot_status is None:
            raise HTTPException(status_code=500, detail="未收到机器人状态数据")
        return {"success": True, "robot_status": robot_status}

    def set_robot_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        if "target_pose" not in payload:
            raise HTTPException(status_code=400, detail="缺少target_pose字段")

        target_pose = payload.get("target_pose", [])
        use_joints = payload.get("use_joints", False)
        velocity_factor = payload.get("velocity_factor", 0.08)
        acceleration_factor = payload.get("acceleration_factor", 0.05)
        try:
            timeout_sec = float(payload.get("timeout_sec", 180.0))
        except (TypeError, ValueError):
            timeout_sec = 180.0
        timeout_sec = max(5.0, min(timeout_sec, 600.0))

        if isinstance(target_pose, (list, tuple)):
            if len(target_pose) < 6:
                raise HTTPException(
                    status_code=400,
                    detail="target_pose必须包含至少6个元素 [x, y, z, qx, qy, qz] 或 7个元素 [x, y, z, qx, qy, qz, qw]",
                )
        elif isinstance(target_pose, dict):
            if "position" not in target_pose or "orientation" not in target_pose:
                raise HTTPException(status_code=400, detail="target_pose字典必须包含position和orientation字段")
        else:
            raise HTTPException(status_code=400, detail="target_pose格式错误，应为数组或字典")

        result, error = node.move_to_pose(
            target_pose,
            use_joints=use_joints,
            velocity_factor=velocity_factor,
            acceleration_factor=acceleration_factor,
            timeout=timeout_sec,
        )
        if error:
            raise HTTPException(status_code=500, detail=error)
        return result

    def set_robot_io(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        required_fields = {"io_type", "io_index", "value"}
        if not required_fields.issubset(payload):
            raise HTTPException(status_code=400, detail="缺少必需字段: io_type, io_index, value")

        result, error = node.set_robot_io(payload["io_type"], payload["io_index"], payload["value"])
        if error:
            raise HTTPException(status_code=500, detail=error)
        return result

    def execute_pose_sequence(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        folder_name = str(payload.get("folder_name", "")).strip()
        if not folder_name:
            raise HTTPException(status_code=400, detail="folder_name不能为空")

        config_dir = self.pose_list_dir / folder_name
        if not config_dir.exists() or not config_dir.is_dir():
            raise HTTPException(status_code=404, detail=f"文件夹不存在: {folder_name}")

        json_files = sorted(config_dir.glob("*.json"))
        if not json_files:
            raise HTTPException(status_code=404, detail=f"文件夹中没有找到JSON文件: {folder_name}")

        pose_files = []
        for json_file in json_files:
            try:
                with open(json_file, "r", encoding="utf-8") as file_obj:
                    pose_data = json.load(file_obj)
                if "joint_position_deg" in pose_data and len(pose_data["joint_position_deg"]) == 6:
                    pose_files.append({"filename": json_file.name, "joint_position_deg": pose_data["joint_position_deg"]})
                else:
                    LOGGER.warning("文件 %s 格式不正确，跳过", json_file.name)
            except Exception as exc:
                LOGGER.warning("读取文件 %s 失败: %s", json_file.name, exc)

        if not pose_files:
            raise HTTPException(status_code=400, detail="没有有效的姿态文件")

        robot_status = node.get_robot_status()
        if robot_status is None:
            raise HTTPException(status_code=500, detail="无法获取机器人状态")

        current_joints = robot_status.get("joint_position_deg", [])
        if len(current_joints) != 6:
            raise HTTPException(status_code=500, detail="机器人状态数据格式错误")

        last_pose = pose_files[-1]["joint_position_deg"]
        tolerance = 0.1
        is_at_last_pose = all(abs(current_joints[i] - last_pose[i]) < tolerance for i in range(6))
        if is_at_last_pose:
            return {
                "success": True,
                "skipped": True,
                "message": "当前已在最后一个点位，跳过执行",
                "current_joints": current_joints,
                "last_pose": last_pose,
                "total_files": len(pose_files),
            }

        def execute_poses():
            try:
                executed_count = 0
                failed_count = 0
                for pose_file in pose_files:
                    result, error = node.move_to_pose(
                        pose_file["joint_position_deg"],
                        use_joints=True,
                        velocity_factor=0.08,
                        acceleration_factor=0.05,
                        timeout=60.0,
                    )
                    if error:
                        failed_count += 1
                    elif result and result.get("success"):
                        executed_count += 1
                    else:
                        failed_count += 1

                    max_wait_time = 60.0
                    wait_start = time.time()
                    while time.time() - wait_start < max_wait_time:
                        status = node.get_robot_status()
                        if status and not status.get("in_motion", True):
                            break
                        time.sleep(0.5)

                LOGGER.info(
                    "姿态序列执行完成: 成功 %s/%s, 失败 %s",
                    executed_count,
                    len(pose_files),
                    failed_count,
                )
            except Exception:
                LOGGER.exception("执行姿态序列异常")

        execution_thread = threading.Thread(target=execute_poses, daemon=True)
        execution_thread.start()
        return {
            "success": True,
            "skipped": False,
            "message": "姿态序列执行已开始",
            "total_files": len(pose_files),
            "current_joints": current_joints,
            "last_pose": last_pose,
        }

    def execute_single_grasp(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        object_id = payload.get("object_id", "default")
        use_visual_estimation = payload.get("use_visual_estimation", True)
        try:
            timeout_sec = float(payload.get("timeout_sec", 300.0))
        except (TypeError, ValueError):
            timeout_sec = 300.0
        timeout_sec = max(30.0, min(timeout_sec, 900.0))

        result = node.call_execute_single_grasp(object_id, use_visual_estimation, timeout=timeout_sec)
        if result is None:
            raise HTTPException(status_code=500, detail="服务调用失败（超时或服务不可用）")
        return {
            "success": result.success,
            "message": result.message,
            "final_position": {
                "x": result.final_position.x,
                "y": result.final_position.y,
                "z": result.final_position.z,
            },
            "final_orientation": {
                "x": result.final_orientation.x,
                "y": result.final_orientation.y,
                "z": result.final_orientation.z,
                "w": result.final_orientation.w,
            },
        }

    def loop_grasp_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        start = payload.get("data", False)
        action = "启动" if start else "停止"
        result = node.call_loop_grasp_control(start)
        if result is None:
            raise HTTPException(status_code=500, detail=f"循环抓取{action}服务调用失败（超时或服务不可用）")
        return {"success": result.success, "message": result.message}

    def publish_grasps_loop_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        start = payload.get("data", False)
        action = "启动" if start else "停止"
        result = node.call_publish_grasps_loop_control(start)
        if result is None:
            raise HTTPException(
                status_code=500,
                detail=f"publish_grasps 循环{action}服务调用失败（超时或服务不可用）",
            )
        return {"success": result.success, "message": result.message}

    def run_gripper_swap(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        direction = str(payload.get("direction", "")).strip()
        if not direction:
            raise HTTPException(status_code=400, detail="direction 不能为空")

        timeout_sec = payload.get("timeout_sec", None)
        timeout = None if timeout_sec is None else float(timeout_sec)
        result = node.call_run_gripper_swap(direction, timeout=timeout)
        if result is None:
            raise HTTPException(status_code=500, detail="夹爪切换服务调用失败（超时或服务不可用）")
        return {"success": result.success, "message": result.message, "direction": direction}

    def save_debug_features(self, payload: dict[str, Any]) -> dict[str, Any]:
        session_id = str(payload.get("session_id", "")).strip()
        features = payload.get("features", [])
        if not session_id:
            raise HTTPException(status_code=400, detail="会话ID为空")

        debug_dir = Path.cwd() / "debug" / session_id
        debug_dir.mkdir(parents=True, exist_ok=True)
        csv_path = debug_dir / "features.csv"
        with open(csv_path, "w", newline="", encoding="utf-8") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(["ID", "ub", "vb", "rb", "us", "vs", "rs", "theta(rad)", "theta(deg)", "area"])
            for feature in features:
                writer.writerow(
                    [
                        feature.get("id", ""),
                        feature.get("ub", 0),
                        feature.get("vb", 0),
                        feature.get("rb", 0),
                        feature.get("us", 0),
                        feature.get("vs", 0),
                        feature.get("rs", 0),
                        feature.get("theta", 0),
                        feature.get("theta_deg", 0),
                        feature.get("area", 0),
                    ]
                )
        return {"success": True, "file_path": str(csv_path), "feature_count": len(features)}

    def debug_capture(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        node = self._require_node()
        payload = payload or {}
        camera_id = payload.get("camera_id", "207000152740")

        with node.image_lock:
            node.latest_depth_image = None
            node.latest_color_image = None
            node.depth_image_received = False
            node.color_image_received = False

        depth_img, color_img, error_msg = node.capture_image(camera_id=camera_id, timeout=10.0)
        if error_msg:
            try:
                node.debug_last_capture_error = error_msg
            except Exception:
                pass
            raise HTTPException(status_code=500, detail=error_msg)

        try:
            node.debug_last_capture_error = None
        except Exception:
            pass

        return {"success": True, "message": "图像采集成功"}

    def debug_get_images(self) -> dict[str, Any]:
        node = self._require_node()
        with node.image_lock:
            if node.latest_depth_image is None or node.latest_color_image is None:
                last_err = getattr(node, "debug_last_capture_error", None)
                return {"success": True, "has_images": False, "message": last_err if last_err else "等待图像..."}
            depth_image = node.latest_depth_image.copy()
            color_image = node.latest_color_image.copy()

        params = node.config_reader.load_debug_thresholds()
        if node.preprocessor:
            node.preprocessor.set_parameters(
                {
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
                }
            )
        if node.feature_extractor:
            node.feature_extractor.set_parameters(
                {
                    "component_min_area": params.get("component_min_area"),
                    "component_max_area": params.get("component_max_area"),
                    "component_min_aspect_ratio": params.get("component_min_aspect_ratio"),
                    "component_max_aspect_ratio": params.get("component_max_aspect_ratio"),
                    "component_min_width": params.get("component_min_width"),
                    "component_min_height": params.get("component_min_height"),
                }
            )

        if depth_image.shape[:2] != color_image.shape[:2]:
            color_image = cv2.resize(color_image, (depth_image.shape[1], depth_image.shape[0]))

        binary_threshold_min = int(params.get("binary_threshold_min", 0))
        binary_threshold_max = int(params.get("binary_threshold_max", 65535))

        if node.preprocessor is None:
            raise HTTPException(status_code=500, detail="算法模块未初始化")

        components, preprocessed_color = node.preprocessor.preprocess(
            depth_image, color_image, binary_threshold_min, binary_threshold_max
        )
        features_objs = node.feature_extractor.extract_features(components, preprocessed_color)
        features = []
        for feat in features_objs:
            features.append(
                {
                    "workpiece_center": list(feat.workpiece_center),
                    "workpiece_radius": float(feat.workpiece_radius),
                    "valve_center": list(feat.valve_center),
                    "valve_radius": float(feat.valve_radius),
                    "angle_deg": float(feat.standardized_angle_deg),
                }
            )

        use_rembg = bool(params.get("use_rembg", False)) and params.get("use_rembg") != 0
        if use_rembg and REMBG_AVAILABLE and preprocessed_color is not None:
            try:
                processor = get_rembg_processor()
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
                            if rembg_mask is not None and len(components) > 0:
                                components[0] = rembg_mask
            except Exception:
                LOGGER.exception("Rembg处理失败")

        visualizer = DebugVisualizer()
        debug_panel = visualizer.create_debug_panel(
            depth_image,
            color_image,
            components,
            features,
            binary_threshold_min,
            binary_threshold_max,
            preprocessed_color,
        )

        def encode_image(img):
            if img is None:
                return None
            ok, buffer = cv2.imencode(".jpg", img)
            if not ok:
                return None
            return "data:image/jpeg;base64," + base64.b64encode(buffer).decode("utf-8")

        return {
            "success": True,
            "has_images": True,
            "depth_image": encode_image(debug_panel.get("depth_display")),
            "color_image": encode_image(debug_panel.get("color_display")),
            "binary_image": encode_image(debug_panel.get("binary_display")),
            "preprocessed_image": encode_image(debug_panel.get("preprocessed_display")),
            "stats": {
                "component_count": len(components),
                "feature_count": len(features),
                "algorithm_source": "algorithm_module",
            },
            "features": features,
            "algorithm_info": {
                "source": "algorithm_module",
                "params_applied": "Debug参数已直接应用到算法",
                "note": "此预览使用与 /estimate_pose 完全相同的算法流程",
            },
        }

    def debug_update_params(self, payload: dict[str, Any]) -> dict[str, Any]:
        node = self._require_node()
        raw_key = payload.get("param_name")
        value = payload.get("param_value")
        normalized_key = raw_key.replace("-", "_") if raw_key else raw_key
        key_map = {
            "min_depth": "binary_threshold_min",
            "max_depth": "binary_threshold_max",
            "contour_min_area": "component_min_area",
            "contour_max_area": "component_max_area",
            "min_aspect": "component_min_aspect_ratio",
            "max_aspect": "component_max_aspect_ratio",
            "min_width": "component_min_width",
            "min_height": "component_min_height",
            "max_count": "component_max_count",
            "enable_smooth_edges": "enable_smooth_edges",
            "smooth_edges_blur_sigma": "smooth_edges_blur_sigma",
            "use_rembg": "use_rembg",
        }
        target_key = key_map.get(normalized_key, normalized_key)
        node.params_manager.update(target_key, value)
        node.notify_params_updated()
        return {"success": True}

    def debug_get_params(self) -> dict[str, Any]:
        node = self._require_node()
        params = node.config_reader.load_debug_thresholds()
        return {"success": True, "params": params}

    def debug_save_thresholds(self) -> dict[str, Any]:
        node = self._require_node()
        success = node.params_manager.save()
        if not success:
            raise HTTPException(status_code=500, detail="保存配置文件失败")
        node.config_reader.load_debug_thresholds()
        node.notify_params_updated()
        return {
            "success": True,
            "message": f"阈值已保存到配置文件: {node.params_manager.config_path} 并同步到算法模块",
        }

    @property
    def templates_dir(self) -> Path:
        return Path(self._ros_bridge.templates_dir)

    @property
    def pose_list_dir(self) -> Path:
        return Path(self._ros_bridge.pose_list_dir)

    def _require_node(self):
        node = self._ros_bridge.node
        if node is None:
            raise HTTPException(
                status_code=503,
                detail=self._ros_bridge.startup_error or "ROS2 bridge unavailable",
            )
        return node
