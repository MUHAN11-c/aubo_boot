#!/usr/bin/env python3
"""
视觉姿态估计主节点（Python / ROS2）

功能：
1. 初始化 ROS2 节点，声明全部参数
2. 使用 MultiThreadedExecutor 替代单线程 spin
3. 初始化所有算法组件和服务
"""

import logging
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .config_reader import ConfigReader
from .path_resolver import resolve_templates_root, resolve_web_paths
from .ros2_communication import ROS2Communication


class VisualPoseEstimationNode(Node):
    """视觉姿态估计 ROS2 节点"""

    def __init__(self):
        super().__init__("visual_pose_estimation_python")

        self._setup_logging()

        # ---- 声明全部参数（统一在此，避免重复 declare） ----
        self.declare_parameter("calib_file", "")
        self.declare_parameter("template_root", "")
        self.declare_parameter("capture_camera_id", "207000152740")
        self.declare_parameter("cache_image_max_age_sec", 8.0)
        self.declare_parameter("depth_image_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_scale", 0.00025)
        self.declare_parameter("depth_search_radius", 3)
        self.declare_parameter("gripper_opening_mm", 50.0)
        self.declare_parameter("gripper_length_mm", 100.0)

        # ---- 获取参数 ----
        calib_file = self.get_parameter("calib_file").value
        template_root = self.get_parameter("template_root").value

        if not template_root:
            template_root = str(resolve_templates_root())

        self.calib_file = calib_file
        self.template_root = template_root

        # ---- 延迟初始化 ----
        self._config_reader = ConfigReader()
        self._ros2_comm: ROS2Communication = None
        self._init_timer = self.create_timer(0.1, self._delayed_initialize)
        self.get_logger().info("视觉姿态估计节点正在初始化...")

    # ------------------------------------------------------------------
    def _setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format="[%(levelname)s] [%(name)s]: %(message)s",
        )

    def _delayed_initialize(self):
        self._init_timer.cancel()
        self.destroy_timer(self._init_timer)
        self._init_timer = None

        try:
            self._ros2_comm = ROS2Communication(self)
            if not self._ros2_comm.initialize(
                self._config_reader,
                self.template_root,
                self.calib_file,
            ):
                self.get_logger().error("ROS2 通信初始化失败")
                return

            self.get_logger().info("视觉姿态估计节点启动成功")
            self.get_logger().info(
                "阈值配置文件: debug_thresholds.json (通过 config_reader 加载)"
            )
            if self.calib_file:
                if Path(self.calib_file).exists():
                    self.get_logger().info(f"标定文件: {self.calib_file}")
                else:
                    self.get_logger().warning(
                        f"标定文件不存在: {self.calib_file}，将从标准路径查找"
                    )
            else:
                self.get_logger().info(
                    "标定文件: 未指定，将从标准路径查找"
                )
            self.get_logger().info(f"模板根目录: {self.template_root}")

        except Exception as e:
            self.get_logger().error(f"延迟初始化失败: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None
    try:
        node = VisualPoseEstimationNode()
        node.get_logger().info("开始运行视觉姿态估计节点...")

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"节点运行异常: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
