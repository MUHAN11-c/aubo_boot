#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false
"""
基于 MoveIt Python API（moveit_py）的运动控制器。

参考 MoveIt 官方教程的用法：
- MoveItPy + PlanningComponent
- set_start_state_to_current_state
- set_goal_state(...), plan(), execute()
"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.node import Node


DEFAULT_GROUP_NAME = "manipulator"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_EE_LINK = "tool_tcp"

ARC_PATH_MAX_RETRIES = 5
ARC_PATH_RETRY_DELAY_SEC = 2.0
ARC_PATH_INITIAL_DELAY_SEC = 0.2


@dataclass(frozen=True)
class CartesianSegment:
    axis: str
    offset: float


class GraspMotionControllerMoveIt2(Node):
    """
    与 moveit_arcline_demo.cpp 接口风格一致，但底层使用 moveit_py。

    注意：
    - moveit_py 不直接暴露 C++ MoveGroupInterface 的 computeCartesianPath 等价接口。
    - run_arc_path_sequence 采用“分段位姿目标”策略：每段保持姿态、到达下一段终点。
    """

    def __init__(self, node_name: str = "grasp_motion_controller_moveit2"):
        super().__init__(node_name)
        self.declare_parameter("group_name", DEFAULT_GROUP_NAME)
        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)
        self.declare_parameter("ee_link", DEFAULT_EE_LINK)
        self.declare_parameter("joint_names", [])

        self.group_name = str(self.get_parameter("group_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        joint_names_value = self.get_parameter("joint_names").value
        self.joint_names = list(joint_names_value) if isinstance(joint_names_value, (list, tuple)) else []
        self._request_seq = 0

        # 为 MoveItPy 显式提供规划配置，避免“Failed to load planning pipelines from parameter server”。
        config_dict = self._build_moveit_py_config()
        self._log_moveit_config(config_dict)
        self.get_logger().info(
            "[API] MoveItPy.__init__(node_name, config_dict) "
            f"node_name={node_name}_moveit_py"
        )
        self._moveit = MoveItPy(node_name=f"{node_name}_moveit_py", config_dict=config_dict)
        self.get_logger().info(
            "[API-RESULT] MoveItPy.__init__ -> ok"
        )
        self.get_logger().info(
            f"[API] MoveItPy.get_planning_component(planning_component_name={self.group_name})"
        )
        self._planning_component = self._moveit.get_planning_component(self.group_name)
        self.get_logger().info("[API-RESULT] MoveItPy.get_planning_component -> ok")
        self.get_logger().info(f"MoveItPy 初始化完成: group={self.group_name}")

    def _log_moveit_config(self, config: dict) -> None:
        pipelines = config.get("planning_pipelines", {})
        pipeline_names = pipelines.get("pipeline_names", [])
        default_pipeline = pipelines.get("default_planning_pipeline", "")
        plan_request = config.get("plan_request_params", {})
        controllers = config.get("controller_names", [])
        self.get_logger().info(
            "[MoveItPy config] "
            f"group={self.group_name}, base_frame={self.base_frame}, ee_link={self.ee_link}, "
            f"pipeline_names={pipeline_names}, default_pipeline={default_pipeline}, "
            f"plan_request_params={plan_request}, controllers={controllers}"
        )

    def _new_req_id(self, api_name: str) -> str:
        self._request_seq += 1
        return f"{api_name}-{self._request_seq:04d}"

    @staticmethod
    def _build_moveit_py_config() -> dict:
        """构建 MoveItPy 配置（基于 aubo_moveit_config，并补充 OMPL pipeline 关键参数）。"""
        config = MoveItConfigsBuilder("aubo_i5", package_name="aubo_moveit_config").to_moveit_configs().to_dict()
        config.setdefault("ompl", {})
        config["ompl"].update({
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        })
        config["planning_pipelines"] = {
            "pipeline_names": ["ompl"],
            "default_planning_pipeline": "ompl",
        }
        config["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "",
            "planning_attempts": 1,
            "planning_time": 5.0,
            "max_velocity_scaling_factor": 0.15,
            "max_acceleration_scaling_factor": 0.1,
        }

        # MoveItPy 在部分版本中会从 moveit_simple_controller_manager 命名空间读取控制器参数。
        if "controller_names" in config and "joint_trajectory_controller" in config:
            config["moveit_simple_controller_manager"] = {
                "controller_names": copy.deepcopy(config["controller_names"]),
                "joint_trajectory_controller": copy.deepcopy(config["joint_trajectory_controller"]),
            }
        return config

    def move_to_joints(
        self,
        joint_positions_rad: Iterable[float],
        velocity_factor: float = 0.15,
        acceleration_factor: float = 0.1,
        joint_names: list[str] | None = None,
    ) -> bool:
        """
        关节空间规划执行（moveit_py）。

        velocity_factor / acceleration_factor 在 moveit_py 中通常由参数文件配置，
        这里保留接口并记录日志，便于与 C++ 接口保持一致。
        """
        names = list(joint_names) if joint_names else list(self.joint_names)
        positions = [float(x) for x in joint_positions_rad]
        if not names:
            self.get_logger().error("move_to_joints 需要 joint_names（参数或函数入参）")
            return False
        if len(names) != len(positions):
            self.get_logger().error(
                f"move_to_joints 关节名/角数量不一致: names={len(names)}, positions={len(positions)}"
            )
            return False

        req_id = self._new_req_id("move_to_joints")
        robot_state = RobotState(self._moveit.get_robot_model())
        robot_state.joint_positions = {name: val for name, val in zip(names, positions)}
        self.get_logger().info(
            f"[{req_id}] [API] PlanningComponent.set_start_state_to_current_state()"
        )
        self._planning_component.set_start_state_to_current_state()
        self.get_logger().info(
            f"[{req_id}] [API-RESULT] PlanningComponent.set_start_state_to_current_state -> ok"
        )
        self.get_logger().info(
            f"[{req_id}] [API] PlanningComponent.set_goal_state(robot_state=RobotState)"
        )
        self._planning_component.set_goal_state(robot_state=robot_state)
        self.get_logger().info(
            f"[{req_id}] [API-RESULT] PlanningComponent.set_goal_state(robot_state) -> ok"
        )

        self.get_logger().info(
            f"[{req_id}] [move_to_joints] 开始规划执行: joints={len(names)}, vel={velocity_factor:.2f}, acc={acceleration_factor:.2f}, "
            f"joint_names={names}, joint_positions={[round(v, 6) for v in positions]}"
        )
        return self._plan_and_execute(req_id=req_id)

    def move_to_pose(
        self,
        target_pose: Pose,
        velocity_factor: float = 0.15,
        acceleration_factor: float = 0.1,
    ) -> bool:
        """位姿目标规划执行（moveit_py）。"""
        req_id = self._new_req_id("move_to_pose")
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.base_frame
        pose_goal.pose = target_pose

        self.get_logger().info(
            f"[{req_id}] [API] PlanningComponent.set_start_state_to_current_state()"
        )
        self._planning_component.set_start_state_to_current_state()
        self.get_logger().info(
            f"[{req_id}] [API-RESULT] PlanningComponent.set_start_state_to_current_state -> ok"
        )
        self.get_logger().info(
            f"[{req_id}] [API] PlanningComponent.set_goal_state(pose_stamped_msg=PoseStamped, pose_link={self.ee_link})"
        )
        self._planning_component.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=self.ee_link,
        )
        self.get_logger().info(
            f"[{req_id}] [API-RESULT] PlanningComponent.set_goal_state(pose_stamped_msg) -> ok"
        )
        self.get_logger().info(
            f"[{req_id}] [move_to_pose] 开始规划执行: frame={self.base_frame}, vel={velocity_factor:.2f}, acc={acceleration_factor:.2f}, "
            f"target_xyz=({target_pose.position.x:.6f}, {target_pose.position.y:.6f}, {target_pose.position.z:.6f}), "
            f"target_q=({target_pose.orientation.x:.6f}, {target_pose.orientation.y:.6f}, "
            f"{target_pose.orientation.z:.6f}, {target_pose.orientation.w:.6f})"
        )
        return self._plan_and_execute(req_id=req_id)

    def run_arc_path(self, axis: str, offset: float) -> bool:
        """单段路径封装：沿 axis 偏移 offset。"""
        return self.run_arc_path_sequence([CartesianSegment(axis=axis, offset=offset)])

    def run_arc_path_sequence(self, segments: list[CartesianSegment | dict]) -> bool:
        """
        多段路径执行。

        兼容 C++ 侧输入格式（axis+offset），每段都以当前姿态为目标姿态，
        只改变位置，依次执行到每个分段终点。
        """
        if not segments:
            self.get_logger().info("[run_arc_path_sequence] segments 为空，不运动")
            return True

        parsed_segments: list[CartesianSegment] = []
        for seg in segments:
            if isinstance(seg, CartesianSegment):
                parsed_segments.append(seg)
            else:
                parsed_segments.append(
                    CartesianSegment(axis=str(seg.get("axis", "")).lower(), offset=float(seg.get("offset", 0.0)))
                )

        if ARC_PATH_INITIAL_DELAY_SEC > 0:
            time.sleep(ARC_PATH_INITIAL_DELAY_SEC)

        curr = self.get_current_ee_pose()
        if curr is None:
            return False
        curr_ori = curr.orientation

        for idx, seg in enumerate(parsed_segments, start=1):
            target = _clone_pose(curr)
            target.orientation = curr_ori
            if seg.axis == "x":
                target.position.x += seg.offset
            elif seg.axis == "y":
                target.position.y += seg.offset
            elif seg.axis == "z":
                target.position.z += seg.offset
            else:
                self.get_logger().error(f"不支持的 axis={seg.axis}，仅支持 x/y/z")
                return False

            ok = False
            for attempt in range(1, ARC_PATH_MAX_RETRIES + 1):
                self.get_logger().info(
                    f"[run_arc_path_sequence] 段 {idx}/{len(parsed_segments)} 规划执行，尝试 {attempt}/{ARC_PATH_MAX_RETRIES}"
                )
                if self.move_to_pose(target):
                    ok = True
                    break
                if attempt < ARC_PATH_MAX_RETRIES:
                    time.sleep(ARC_PATH_RETRY_DELAY_SEC)
            if not ok:
                self.get_logger().error(f"第 {idx} 段失败: axis={seg.axis}, offset={seg.offset:+.4f}")
                return False
            curr = target

        self.get_logger().info("多段路径执行完成")
        return True

    def get_current_ee_pose(self) -> Pose | None:
        """直接通过 moveit_py 当前 RobotState 获取末端位姿。"""
        req_id = self._new_req_id("get_current_ee_pose")
        self.get_logger().info(f"[{req_id}] [API] MoveItPy.get_planning_scene_monitor()")
        planning_scene_monitor = self._moveit.get_planning_scene_monitor()
        self.get_logger().info(f"[{req_id}] [API-RESULT] MoveItPy.get_planning_scene_monitor -> ok")
        for attempt in range(1, 21):
            try:
                self.get_logger().info(f"[{req_id}] [API] PlanningSceneMonitor.read_only() attempt={attempt}")
                with planning_scene_monitor.read_only() as scene:
                    self.get_logger().info(f"[{req_id}] [API] RobotState.get_pose(link_name={self.ee_link})")
                    pose = scene.current_state.get_pose(self.ee_link)
                    self.get_logger().info(
                        f"[{req_id}] [API-RESULT] RobotState.get_pose -> success; "
                        f"attempt={attempt}, link={self.ee_link}, "
                        f"xyz=({pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}), "
                        f"q=({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
                        f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f})"
                    )
                    return pose
            except Exception as exc:
                self.get_logger().warn(f"[{req_id}] [API-RESULT] read_only/get_pose retry {attempt}/20, link={self.ee_link}, error={exc}")
                time.sleep(0.03)
        self.get_logger().error(f"[{req_id}] 获取当前末端位姿失败: moveit_py current_state link={self.ee_link}")
        return None

    def _plan_and_execute(self, req_id: str) -> bool:
        """按官方教程流程：plan() -> execute(trajectory)。"""
        plan_start = time.time()
        try:
            self.get_logger().info(f"[{req_id}] [API] PlanningComponent.plan() start, group={self.group_name}")
            plan_result = self._planning_component.plan()
            if not plan_result:
                self.get_logger().error(f"[{req_id}] [API-RESULT] PlanningComponent.plan -> empty")
                return False
            plan_elapsed = time.time() - plan_start
            has_trajectory = hasattr(plan_result, "trajectory") and plan_result.trajectory is not None
            self.get_logger().info(
                f"[{req_id}] [API-RESULT] PlanningComponent.plan -> success; "
                f"elapsed={plan_elapsed:.3f}s, has_trajectory={has_trajectory}"
            )
            exec_start = time.time()
            self.get_logger().info(
                f"[{req_id}] [API] MoveItPy.execute(group_name={self.group_name}, trajectory=RobotTrajectory, blocking=True)"
            )
            status = self._moveit.execute(self.group_name, plan_result.trajectory, blocking=True)
            exec_elapsed = time.time() - exec_start
            self.get_logger().info(
                f"[{req_id}] [API-RESULT] MoveItPy.execute -> status={status.status}, elapsed={exec_elapsed:.3f}s"
            )
            if bool(status):
                self.get_logger().info(f"[{req_id}] [RESULT] success=True")
                return True
            self.get_logger().error(f"[{req_id}] [RESULT] success=False, execute_status={status.status}")
            return False
        except Exception as exc:
            self.get_logger().error(f"[{req_id}] [RESULT] exception={exc}")
            return False


def _clone_pose(p: Pose) -> Pose:
    out = Pose()
    out.position.x = p.position.x
    out.position.y = p.position.y
    out.position.z = p.position.z
    out.orientation = p.orientation
    return out

