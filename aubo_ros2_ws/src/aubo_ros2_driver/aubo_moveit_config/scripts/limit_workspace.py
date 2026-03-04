#!/usr/bin/env python3
"""
使用碰撞对象限制 MoveIt2 工作空间
通过添加边界墙（walls）来限制机械臂的运动范围

使用方法：
1. 确保 MoveIt2 和 move_group 节点正在运行
2. 运行此脚本：python3 limit_workspace.py
3. 在 RViz2 中查看添加的边界墙
4. 机械臂将无法规划超出这些边界的路径

配置方式（按优先级）：
1. 命令行参数（最高优先级）
2. YAML配置文件（默认使用 workspace_limits.yaml）
3. 默认值（最低优先级）

示例：
  # 使用默认配置文件 workspace_limits.yaml（推荐）
  python3 limit_workspace.py
  
  # 使用命令行参数
  python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8
  
  # 使用其他配置文件
  python3 limit_workspace.py --config my_config.yaml
  
  # 移除边界墙
  python3 limit_workspace.py --remove
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time
import math
import argparse
import yaml
import os


class WorkspaceLimiter(Node):
    """通过添加边界碰撞对象来限制工作空间"""
    
    def __init__(self, workspace_limits=None):
        super().__init__('workspace_limiter')
        
        # 发布碰撞对象到规划场景监控话题
        self.collision_object_publisher = self.create_publisher(
            CollisionObject,
            '/move_group/planning_scene_monitor',
            10
        )
        
        # 发布规划场景话题（备用方法）
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            '/move_group/publish_planning_scene',
            10
        )
        
        # 等待发布者就绪
        time.sleep(1.0)
        
        self.get_logger().info('工作空间限制节点已启动')
        
        # 配置工作空间边界（单位：米，相对于 world 坐标系）
        # 如果提供了参数则使用，否则使用默认值
        if workspace_limits is None:
            self.workspace_limits = {
                'x_min': -0.8,   # X轴最小边界
                'x_max': 0.8,    # X轴最大边界
                'y_min': -0.8,   # Y轴最小边界
                'y_max': 0.8,    # Y轴最大边界
                'z_min': 0.0,    # Z轴最小边界（地面）
                'z_max': 1.2,    # Z轴最大边界
                'wall_thickness': 0.05  # 墙的厚度（米）
            }
        else:
            self.workspace_limits = workspace_limits
        
        # 发布边界墙
        self.publish_workspace_limits()
    
    def create_wall(self, wall_id, position, dimensions):
        """创建一面墙（碰撞对象）
        
        Args:
            wall_id: 墙的唯一标识符
            position: 墙的位置 [x, y, z]
            dimensions: 墙的尺寸 [length, width, height]
        
        Returns:
            CollisionObject: 碰撞对象
        """
        collision_object = CollisionObject()
        collision_object.id = wall_id
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        
        # 定义墙的形状（盒子）
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions  # [length, width, height]
        
        # 定义墙的位置和姿态
        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.w = 1.0  # 无旋转
        
        # 将形状和位置添加到碰撞对象
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
    
    def publish_workspace_limits(self):
        """发布工作空间边界墙"""
        limits = self.workspace_limits
        wall_thickness = limits['wall_thickness']
        height = limits['z_max'] - limits['z_min']
        
        # 获取启用的边界墙列表（默认为全部启用）
        enabled_walls = limits.get('enabled_walls', {
            'left': True,      # 左墙（X轴负方向）
            'right': True,     # 右墙（X轴正方向）
            'front': True,     # 前墙（Y轴正方向）
            'back': True,      # 后墙（Y轴负方向）
            'top': True,       # 顶墙（Z轴正方向）
            'bottom': False    # 底墙（Z轴负方向，默认禁用）
        })
        
        collision_objects = []
        
        # 1. 左墙（X轴负方向边界）
        if enabled_walls.get('left', True):
            left_wall = self.create_wall(
                "workspace_wall_left",
                [
                    limits['x_min'] - wall_thickness / 2,
                    (limits['y_min'] + limits['y_max']) / 2,
                    limits['z_min'] + height / 2
                ],
                [wall_thickness, limits['y_max'] - limits['y_min'], height]
            )
            collision_objects.append(left_wall)
        
        # 2. 右墙（X轴正方向边界）
        if enabled_walls.get('right', True):
            right_wall = self.create_wall(
                "workspace_wall_right",
                [
                    limits['x_max'] + wall_thickness / 2,
                    (limits['y_min'] + limits['y_max']) / 2,
                    limits['z_min'] + height / 2
                ],
                [wall_thickness, limits['y_max'] - limits['y_min'], height]
            )
            collision_objects.append(right_wall)
        
        # 3. 前墙（Y轴正方向边界）
        if enabled_walls.get('front', True):
            front_wall = self.create_wall(
                "workspace_wall_front",
                [
                    (limits['x_min'] + limits['x_max']) / 2,
                    limits['y_max'] + wall_thickness / 2,
                    limits['z_min'] + height / 2
                ],
                [limits['x_max'] - limits['x_min'], wall_thickness, height]
            )
            collision_objects.append(front_wall)
        
        # 4. 后墙（Y轴负方向边界）
        if enabled_walls.get('back', True):
            back_wall = self.create_wall(
                "workspace_wall_back",
                [
                    (limits['x_min'] + limits['x_max']) / 2,
                    limits['y_min'] - wall_thickness / 2,
                    limits['z_min'] + height / 2
                ],
                [limits['x_max'] - limits['x_min'], wall_thickness, height]
            )
            collision_objects.append(back_wall)
        
        # 5. 顶墙（Z轴正方向边界）
        if enabled_walls.get('top', True):
            top_wall = self.create_wall(
                "workspace_wall_top",
                [
                    (limits['x_min'] + limits['x_max']) / 2,
                    (limits['y_min'] + limits['y_max']) / 2,
                    limits['z_max'] + wall_thickness / 2
                ],
                [limits['x_max'] - limits['x_min'], limits['y_max'] - limits['y_min'], wall_thickness]
            )
            collision_objects.append(top_wall)
        
        # 6. 底墙（Z轴负方向边界）
        if enabled_walls.get('bottom', False):
            bottom_wall = self.create_wall(
                "workspace_wall_bottom",
                [
                    (limits['x_min'] + limits['x_max']) / 2,
                    (limits['y_min'] + limits['y_max']) / 2,
                    limits['z_min'] - wall_thickness / 2
                ],
                [limits['x_max'] - limits['x_min'], limits['y_max'] - limits['y_min'], wall_thickness]
            )
            collision_objects.append(bottom_wall)
        
        # 发布所有碰撞对象
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始发布工作空间边界墙')
        self.get_logger().info('工作空间限制:')
        self.get_logger().info(f'  X: [{limits["x_min"]:.2f}, {limits["x_max"]:.2f}] 米')
        self.get_logger().info(f'  Y: [{limits["y_min"]:.2f}, {limits["y_max"]:.2f}] 米')
        self.get_logger().info(f'  Z: [{limits["z_min"]:.2f}, {limits["z_max"]:.2f}] 米')
        self.get_logger().info(f'  墙厚度: {wall_thickness} 米')
        
        # 显示启用的边界墙
        enabled_list = []
        if enabled_walls.get('left', True):
            enabled_list.append('左墙')
        if enabled_walls.get('right', True):
            enabled_list.append('右墙')
        if enabled_walls.get('front', True):
            enabled_list.append('前墙')
        if enabled_walls.get('back', True):
            enabled_list.append('后墙')
        if enabled_walls.get('top', True):
            enabled_list.append('顶墙')
        if enabled_walls.get('bottom', False):
            enabled_list.append('底墙')
        
        self.get_logger().info(f'  启用的边界墙: {", ".join(enabled_list) if enabled_list else "无"}')
        self.get_logger().info('=' * 60)
        
        # 方法1：逐个发布碰撞对象
        for obj in collision_objects:
            self.get_logger().info(f'发布边界墙: {obj.id}')
            for i in range(3):  # 发布3次，确保被接收
                self.collision_object_publisher.publish(obj)
                time.sleep(0.1)
        
        # 等待一下，让move_group处理
        time.sleep(1.0)
        
        # 方法2：发布完整的规划场景（备用方法，更可靠）
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        planning_scene.world.collision_objects = collision_objects
        
        self.get_logger().info('发布完整规划场景（包含所有边界墙）')
        for i in range(3):  # 发布3次，确保被接收
            self.planning_scene_publisher.publish(planning_scene)
            time.sleep(0.2)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('工作空间边界墙发布完成！')
        self.get_logger().info(f'共发布了 {len(collision_objects)} 面墙')
        self.get_logger().info('请在 RViz2 中检查 Motion Planning 插件的 Scene Geometry 部分')
        self.get_logger().info('机械臂将无法规划超出这些边界的路径')
        self.get_logger().info('=' * 60)
    
    def remove_workspace_limits(self):
        """移除工作空间边界墙"""
        wall_ids = [
            "workspace_wall_left",
            "workspace_wall_right",
            "workspace_wall_front",
            "workspace_wall_back",
            "workspace_wall_top",
            "workspace_wall_bottom"
        ]
        
        self.get_logger().info('移除工作空间边界墙...')
        
        for wall_id in wall_ids:
            collision_object = CollisionObject()
            collision_object.id = wall_id
            collision_object.header.frame_id = "world"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.operation = CollisionObject.REMOVE
            
            self.get_logger().info(f'移除边界墙: {wall_id}')
            for i in range(3):
                self.collision_object_publisher.publish(collision_object)
                time.sleep(0.1)
        
        time.sleep(1.0)
        
        # 发布规划场景更新
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        
        for wall_id in wall_ids:
            remove_obj = CollisionObject()
            remove_obj.id = wall_id
            remove_obj.operation = CollisionObject.REMOVE
            planning_scene.world.collision_objects.append(remove_obj)
        
        for i in range(3):
            self.planning_scene_publisher.publish(planning_scene)
            time.sleep(0.2)
        
        self.get_logger().info('工作空间边界墙已移除')


def load_config_file(config_path):
    """从YAML配置文件加载工作空间限制
    
    Args:
        config_path: 配置文件路径
        
    Returns:
        dict: 工作空间限制字典，如果文件不存在或格式错误返回None
    """
    if not os.path.exists(config_path):
        print(f"警告: 配置文件不存在: {config_path}")
        return None
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 验证配置格式
        required_keys = ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max']
        if not all(key in config for key in required_keys):
            print(f"错误: 配置文件缺少必需的键: {required_keys}")
            return None
        
        # 设置默认墙厚度（如果未指定）
        if 'wall_thickness' not in config:
            config['wall_thickness'] = 0.05
        
        # 设置默认启用的边界墙（如果未指定）
        if 'enabled_walls' not in config:
            config['enabled_walls'] = {
                'left': True,
                'right': True,
                'front': True,
                'back': True,
                'top': True,
                'bottom': False
            }
        else:
            # 确保所有墙的选项都存在，缺失的设为默认值
            default_enabled = {
                'left': True,
                'right': True,
                'front': True,
                'back': True,
                'top': True,
                'bottom': False
            }
            for wall_name in default_enabled:
                if wall_name not in config['enabled_walls']:
                    config['enabled_walls'][wall_name] = default_enabled[wall_name]
        
        print(f"成功加载配置文件: {config_path}")
        return config
    except yaml.YAMLError as e:
        print(f"错误: 配置文件格式错误: {e}")
        return None
    except Exception as e:
        print(f"错误: 读取配置文件失败: {e}")
        return None


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='限制 MoveIt2 工作空间边界',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用默认值
  python3 limit_workspace.py
  
  # 使用命令行参数指定边界
  python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3
  
  # 使用配置文件
  python3 limit_workspace.py --config workspace_limits.yaml
  
  # 移除边界墙
  python3 limit_workspace.py --remove
        """
    )
    
    # 配置文件选项
    # 获取脚本所在目录，用于显示默认配置文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_config = os.path.join(script_dir, 'workspace_limits.yaml')
    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help=f'YAML配置文件路径（可选，未指定时自动使用: workspace_limits.yaml）'
    )
    
    # X轴边界
    parser.add_argument(
        '--x-min',
        type=float,
        default=None,
        help='X轴最小边界（米）'
    )
    parser.add_argument(
        '--x-max',
        type=float,
        default=None,
        help='X轴最大边界（米）'
    )
    
    # Y轴边界
    parser.add_argument(
        '--y-min',
        type=float,
        default=None,
        help='Y轴最小边界（米）'
    )
    parser.add_argument(
        '--y-max',
        type=float,
        default=None,
        help='Y轴最大边界（米）'
    )
    
    # Z轴边界
    parser.add_argument(
        '--z-min',
        type=float,
        default=None,
        help='Z轴最小边界（米）'
    )
    parser.add_argument(
        '--z-max',
        type=float,
        default=None,
        help='Z轴最大边界（米）'
    )
    
    # 墙厚度
    parser.add_argument(
        '--wall-thickness',
        type=float,
        default=None,
        help='边界墙的厚度（米，默认0.05）'
    )
    
    # 移除选项
    parser.add_argument(
        '--remove', '-r',
        action='store_true',
        help='移除所有边界墙'
    )
    
    return parser.parse_args()


def merge_limits(default_limits, config_limits=None, cmd_args=None):
    """合并默认值、配置文件和命令行参数
    
    优先级：命令行参数 > 配置文件 > 默认值
    
    Args:
        default_limits: 默认限制字典
        config_limits: 配置文件限制字典（可选）
        cmd_args: 命令行参数（可选）
        
    Returns:
        dict: 合并后的限制字典
    """
    limits = default_limits.copy()
    
    # 先应用配置文件的值
    if config_limits:
        for key in limits.keys():
            if key in config_limits:
                limits[key] = config_limits[key]
    
    # 再应用命令行参数的值（最高优先级）
    if cmd_args:
        if cmd_args.x_min is not None:
            limits['x_min'] = cmd_args.x_min
        if cmd_args.x_max is not None:
            limits['x_max'] = cmd_args.x_max
        if cmd_args.y_min is not None:
            limits['y_min'] = cmd_args.y_min
        if cmd_args.y_max is not None:
            limits['y_max'] = cmd_args.y_max
        if cmd_args.z_min is not None:
            limits['z_min'] = cmd_args.z_min
        if cmd_args.z_max is not None:
            limits['z_max'] = cmd_args.z_max
        if cmd_args.wall_thickness is not None:
            limits['wall_thickness'] = cmd_args.wall_thickness
    
    # 确保 enabled_walls 存在
    if 'enabled_walls' not in limits:
        limits['enabled_walls'] = {
            'left': True,
            'right': True,
            'front': True,
            'back': True,
            'top': True,
            'bottom': False
        }
    
    # 验证边界值的合理性
    if limits['x_min'] >= limits['x_max']:
        raise ValueError(f"X轴边界无效: x_min ({limits['x_min']}) >= x_max ({limits['x_max']})")
    if limits['y_min'] >= limits['y_max']:
        raise ValueError(f"Y轴边界无效: y_min ({limits['y_min']}) >= y_max ({limits['y_max']})")
    if limits['z_min'] >= limits['z_max']:
        raise ValueError(f"Z轴边界无效: z_min ({limits['z_min']}) >= z_max ({limits['z_max']})")
    if limits['wall_thickness'] <= 0:
        raise ValueError(f"墙厚度必须大于0: {limits['wall_thickness']}")
    
    return limits


def main(args=None):
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_config_path = os.path.join(script_dir, 'workspace_limits.yaml')
    
    # 解析命令行参数
    cmd_args = parse_arguments()
    
    # 如果只是移除边界墙
    if cmd_args.remove:
        rclpy.init(args=args)
        node = WorkspaceLimiter(workspace_limits={
            'x_min': -0.8, 'x_max': 0.8,
            'y_min': -0.8, 'y_max': 0.8,
            'z_min': 0.0, 'z_max': 1.2,
            'wall_thickness': 0.05
        })
        print("\n正在移除工作空间边界墙...")
        node.remove_workspace_limits()
        time.sleep(2.0)
        node.destroy_node()
        rclpy.shutdown()
        print("边界墙已移除！")
        return
    
    # 加载配置文件
    config_limits = None
    
    # 如果指定了配置文件，使用指定的
    if cmd_args.config:
        config_limits = load_config_file(cmd_args.config)
    # 否则尝试使用默认配置文件
    elif os.path.exists(default_config_path):
        config_limits = load_config_file(default_config_path)
        if config_limits:
            print(f"使用默认配置文件: {default_config_path}")
    
    # 默认限制值
    default_limits = {
        'x_min': -0.8,
        'x_max': 0.8,
        'y_min': -0.8,
        'y_max': 0.8,
        'z_min': 0.0,
        'z_max': 1.2,
        'wall_thickness': 0.05,
        'enabled_walls': {
            'left': True,
            'right': True,
            'front': True,
            'back': True,
            'top': True,
            'bottom': False
        }
    }
    
    # 合并所有配置源
    try:
        workspace_limits = merge_limits(default_limits, config_limits, cmd_args)
    except ValueError as e:
        print(f"错误: {e}")
        return
    
    # 显示最终配置
    print("\n" + "=" * 60)
    print("工作空间限制配置:")
    print(f"  X轴: [{workspace_limits['x_min']:.3f}, {workspace_limits['x_max']:.3f}] 米")
    print(f"  Y轴: [{workspace_limits['y_min']:.3f}, {workspace_limits['y_max']:.3f}] 米")
    print(f"  Z轴: [{workspace_limits['z_min']:.3f}, {workspace_limits['z_max']:.3f}] 米")
    print(f"  墙厚度: {workspace_limits['wall_thickness']:.3f} 米")
    print("=" * 60 + "\n")
    
    # 初始化ROS2节点并发布边界墙
    rclpy.init(args=args)
    node = WorkspaceLimiter(workspace_limits=workspace_limits)
    
    # 保持节点运行一段时间，确保边界墙被处理
    time.sleep(3.0)
    
    node.destroy_node()
    rclpy.shutdown()
    
    print("\n工作空间限制设置完成！")
    print("提示：")
    print("1. 在 RViz2 中查看 Motion Planning 插件的 Scene Geometry")
    print("2. 尝试规划超出边界的路径，应该会失败")
    print("3. 要移除边界墙，运行: python3 limit_workspace.py --remove")


if __name__ == '__main__':
    main()
