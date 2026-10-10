#!/usr/bin/env python3
"""
使用碰撞对象限制 MoveIt2 工作空间.

通过添加边界墙（walls）来限制机械臂的运动范围。
2026-09-30 自 aubo_boot aubo_moveit_config 移植（等价收敛：墙几何/话题/
墙 ID/CLI 与原版一致，交互性打印精简）；按 IVG/peach 隔离裁定迁入本包。

使用方法：
1. 确保 move_group 正在运行
2. python3 limit_workspace.py            # 用默认 workspace_limits.yaml
   python3 limit_workspace.py --x-min -0.5 --x-max 0.5 ...  # CLI 覆盖
   python3 limit_workspace.py --config my_config.yaml
   python3 limit_workspace.py --remove   # 移除边界墙
"""
import argparse
import os
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_LIMITS = {
    'x_min': -0.87, 'x_max': 0.87,
    'y_min': -0.87, 'y_max': 0.87,
    'z_min': -0.85, 'z_max': 1.10,
    'wall_thickness': 0.05,
}
WALL_IDS = [
    'workspace_wall_left', 'workspace_wall_right', 'workspace_wall_front',
    'workspace_wall_back', 'workspace_wall_top', 'workspace_wall_bottom',
]


class WorkspaceLimiter(Node):
    """通过添加边界碰撞对象来限制工作空间."""

    def __init__(self, workspace_limits=None):
        super().__init__('workspace_limiter')
        self.collision_object_publisher = self.create_publisher(
            CollisionObject, '/move_group/planning_scene_monitor', 10)
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene, '/move_group/publish_planning_scene', 10)
        time.sleep(1.0)
        self.get_logger().info('工作空间限制节点已启动')
        self.workspace_limits = workspace_limits or dict(DEFAULT_LIMITS)

    def _wall(self, wall_id, position, dimensions):
        obj = CollisionObject()
        obj.id = wall_id
        obj.header.frame_id = 'world'
        obj.header.stamp = self.get_clock().now().to_msg()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def publish_workspace_limits(self):
        """发布工作空间边界墙（左右/前后/顶，底墙默认禁用）."""
        lim = self.workspace_limits
        t = lim.get('wall_thickness', 0.05)
        height = lim['z_max'] - lim['z_min']
        cx = (lim['x_min'] + lim['x_max']) / 2
        cy = (lim['y_min'] + lim['y_max']) / 2
        dx = lim['x_max'] - lim['x_min']
        dy = lim['y_max'] - lim['y_min']
        enabled = lim.get('enabled_walls', {
            'left': True, 'right': True, 'front': True,
            'back': True, 'top': True, 'bottom': False,
        })

        walls = []
        if enabled.get('left', True):
            walls.append(self._wall(WALL_IDS[0],
                                    [lim['x_min'] - t / 2, cy, lim['z_min'] + height / 2],
                                    [t, dy, height]))
        if enabled.get('right', True):
            walls.append(self._wall(WALL_IDS[1],
                                    [lim['x_max'] + t / 2, cy, lim['z_min'] + height / 2],
                                    [t, dy, height]))
        if enabled.get('front', True):
            walls.append(self._wall(WALL_IDS[2],
                                    [cx, lim['y_max'] + t / 2, lim['z_min'] + height / 2],
                                    [dx, t, height]))
        if enabled.get('back', True):
            walls.append(self._wall(WALL_IDS[3],
                                    [cx, lim['y_min'] - t / 2, lim['z_min'] + height / 2],
                                    [dx, t, height]))
        if enabled.get('top', True):
            walls.append(self._wall(WALL_IDS[4],
                                    [cx, cy, lim['z_max'] + t / 2],
                                    [dx, dy, t]))
        if enabled.get('bottom', False):
            walls.append(self._wall(WALL_IDS[5],
                                    [cx, cy, lim['z_min'] - t / 2],
                                    [dx, dy, t]))

        self.get_logger().info(
            f"工作空间 X[{lim['x_min']:.2f},{lim['x_max']:.2f}] "
            f"Y[{lim['y_min']:.2f},{lim['y_max']:.2f}] "
            f"Z[{lim['z_min']:.2f},{lim['z_max']:.2f}] → {len(walls)} 面墙")

        for obj in walls:
            for _ in range(3):
                self.collision_object_publisher.publish(obj)
                time.sleep(0.1)
        time.sleep(1.0)

        # 备用通道：完整规划场景 diff（更可靠）
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = walls
        for _ in range(3):
            self.planning_scene_publisher.publish(scene)
            time.sleep(0.1)
        self.get_logger().info('边界墙已发布')

    def remove_workspace_limits(self):
        """移除全部边界墙."""
        for wall_id in WALL_IDS:
            obj = CollisionObject()
            obj.id = wall_id
            obj.header.frame_id = 'world'
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.operation = CollisionObject.REMOVE
            for _ in range(3):
                self.collision_object_publisher.publish(obj)
                time.sleep(0.1)
        self.get_logger().info('边界墙已移除')


def load_limits(args):
    limits = dict(DEFAULT_LIMITS)
    config = args.config
    if config is None:
        default_cfg = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'workspace_limits.yaml')
        if os.path.isfile(default_cfg):
            config = default_cfg
    if config and yaml is not None:
        with open(config, 'r') as f:
            data = yaml.safe_load(f) or {}
        limits.update({k: data[k] for k in DEFAULT_LIMITS if k in data})
        if 'enabled_walls' in data:
            limits['enabled_walls'] = data['enabled_walls']
    # CLI 覆盖
    for key in DEFAULT_LIMITS:
        val = getattr(args, key, None)
        if val is not None:
            limits[key] = val
    return limits


def main():
    ap = argparse.ArgumentParser(description='工作空间边界墙（MoveIt PlanningScene）')
    ap.add_argument('--config', default=None, help='YAML 配置文件')
    ap.add_argument('--remove', action='store_true', help='移除边界墙')
    ap.add_argument('--x-min', type=float, default=None)
    ap.add_argument('--x-max', type=float, default=None)
    ap.add_argument('--y-min', type=float, default=None)
    ap.add_argument('--y-max', type=float, default=None)
    ap.add_argument('--z-min', type=float, default=None)
    ap.add_argument('--z-max', type=float, default=None)
    ap.add_argument('--wall-thickness', type=float, default=None)
    args = ap.parse_args()

    rclpy.init()
    node = WorkspaceLimiter(None if args.remove else load_limits(args))
    try:
        if args.remove:
            node.remove_workspace_limits()
        else:
            node.publish_workspace_limits()
    finally:
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
