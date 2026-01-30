#!/usr/bin/env python3
"""
视觉姿态估计主节点

功能：
1. 初始化ROS2节点
2. 初始化所有组件（使用config_reader加载配置）
3. 启动服务
"""

import rclpy
from rclpy.node import Node
import logging
import sys
import traceback
from pathlib import Path
from .ros2_communication import ROS2Communication
from .config_reader import ConfigReader


class VisualPoseEstimationNode(Node):
    """视觉姿态估计ROS2节点类"""
    
    def __init__(self):
        """构造函数：初始化节点并声明参数"""
        super().__init__('visual_pose_estimation_python')
        
        # 配置日志
        self._setup_logging()
        
        # 声明参数
        self.declare_parameter('calib_file', '')
        self.declare_parameter('template_root', '')
        self.declare_parameter('debug', False)
        
        # 获取参数
        try:
            calib_file = self.get_parameter('calib_file').value
            template_root = self.get_parameter('template_root').value
            debug = self.get_parameter('debug').value
        except Exception as e:
            self.get_logger().warn(f'参数获取失败，使用默认值: {e}')
            calib_file = ''
            template_root = ''
            debug = False
        
        if not template_root:
            # 使用包内的templates目录
            current_file = Path(__file__).resolve()
            # visual_pose_estimation_python/visual_pose_estimation_python/main.py
            # 向上4级到 visual_pose_estimation_python 包目录，再向上2级到 visual_pose_estimation
            pkg_root = current_file.parents[4]  # visual_pose_estimation
            template_root = str(pkg_root / 'templates')
            
            if not Path(template_root).exists():
                # 备用路径
                template_root = '/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/templates'
        
        # 保存参数
        self.calib_file = calib_file
        self.template_root = template_root
        self.debug = debug
        
        # 延迟初始化（使用定时器确保对象已被shared_ptr管理）
        self.init_timer = self.create_timer(0.1, self._delayed_initialize)
        
        self.get_logger().info('视觉姿态估计节点正在初始化...')
    
    def _setup_logging(self):
        """配置日志"""
        # 配置Python日志
        logging.basicConfig(
            level=logging.INFO,
            format='[%(levelname)s] [%(name)s]: %(message)s'
        )
    
    def _delayed_initialize(self):
        """延迟初始化：在节点完全创建后执行初始化"""
        try:
            # 取消定时器，只执行一次
            self.init_timer.cancel()
            
            # 创建配置读取器（不再从YAML文件加载，使用默认配置）
            config_reader = ConfigReader()
            
            # 创建ROS2通信对象
            self.ros2_communication = ROS2Communication(self)
            
            # 初始化ROS2通信
            if not self.ros2_communication.initialize(
                config_reader,
                self.template_root,
                self.calib_file,
                self.debug
            ):
                self.get_logger().error('ROS2通信初始化失败')
                return
            
            self.get_logger().info('视觉姿态估计节点启动成功')
            self.get_logger().info(f'阈值配置文件: debug_thresholds.json (从config_reader加载)')
            if self.calib_file:
                # 检查文件是否存在
                if Path(self.calib_file).exists():
                    self.get_logger().info(f'标定文件: {self.calib_file}')
                else:
                    self.get_logger().warning(f'标定文件不存在: {self.calib_file}，将从标准路径查找')
            else:
                self.get_logger().info('标定文件: 未指定，将从标准路径查找（hand_eye_calibration/config/calibration_results/）')
            self.get_logger().info(f'模板根目录: {self.template_root}')
            self.get_logger().info(f'调试模式: {"开启" if self.debug else "关闭"}')
            
        except Exception as e:
            self.get_logger().error(f'延迟初始化失败: {e}')
    
    def __del__(self):
        """析构函数：节点关闭时调用"""
        self.get_logger().info('视觉姿态估计节点关闭')


def main(args=None):
    """主函数：程序入口
    
    Args:
        args: 命令行参数
    """
    # 初始化ROS2
    rclpy.init(args=args)
    
    try:
        # 创建节点实例
        node = VisualPoseEstimationNode()
        
        node.get_logger().info('开始运行视觉姿态估计节点...')
        
        # 运行节点，阻塞直到节点关闭
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as e:
        print(f'节点运行异常: {e}', file=sys.stderr)
        traceback.print_exc()
    finally:
        # 关闭ROS2
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
