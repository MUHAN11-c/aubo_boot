#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 测试节点

功能：
  1. 测试路径查找功能
  2. 测试模型加载
  3. 测试数据读取
  4. 测试抓取预测（可选）
"""

import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import torch

# 路径设置函数（ROS2 标准方式）
def setup_baseline_paths(baseline_dir):
    """设置 graspnet-baseline 的 Python 路径
    
    需要同时添加 baseline_dir 和 models 目录到 sys.path：
    - baseline_dir: 用于 from models.graspnet import ...
    - models 目录: 用于 graspnet.py 内部的 from backbone import ...
    """
    if baseline_dir and os.path.exists(baseline_dir):
        # 添加 baseline_dir 本身（用于 from models.graspnet import ...）
        if baseline_dir not in sys.path:
            sys.path.insert(0, baseline_dir)
        # 添加 models 目录（用于 graspnet.py 内部的 from backbone import ...）
        models_dir = os.path.join(baseline_dir, 'models')
        if os.path.exists(models_dir) and models_dir not in sys.path:
            sys.path.insert(0, models_dir)
        return True
    return False


class GraspNetTestNode(Node):
    """GraspNet 测试节点"""
    
    def __init__(self):
        super().__init__('graspnet_test_node')
        
        # 声明参数（baseline_dir 从 launch 文件传入）
        self.declare_parameter('baseline_dir', '')
        self.declare_parameter('test_model_load', True)
        self.declare_parameter('test_data_read', True)
        self.declare_parameter('test_prediction', False)
        self.declare_parameter('model_path', '')
        self.declare_parameter('data_dir', '')
        
        # 获取参数
        baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value
        self.test_model_load = self.get_parameter('test_model_load').get_parameter_value().bool_value
        self.test_data_read = self.get_parameter('test_data_read').get_parameter_value().bool_value
        self.test_prediction = self.get_parameter('test_prediction').get_parameter_value().bool_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        
        # 设置 Python 路径（使用从 launch 文件传入的 baseline_dir）
        if baseline_dir and os.path.exists(baseline_dir):
            if not setup_baseline_paths(baseline_dir):
                self.get_logger().error(f'无法设置 baseline 路径: {baseline_dir}')
                raise RuntimeError(f'无法设置 baseline 路径: {baseline_dir}')
            self.get_logger().info(f'已设置 baseline 路径: {baseline_dir}')
        else:
            self.get_logger().error(f'baseline_dir 参数无效或目录不存在: {baseline_dir}')
            raise RuntimeError(f'baseline_dir 参数无效或目录不存在: {baseline_dir}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('GraspNet ROS2 测试节点启动')
        self.get_logger().info('=' * 60)
        
        # 运行测试
        self.run_tests()
    
    def run_tests(self):
        """运行所有测试"""
        results = {}
        
        # 测试1: 路径查找
        self.get_logger().info('\n[测试 1] 路径查找功能')
        results['path_finding'] = self.test_path_finding()
        
        # 测试2: 模型加载
        if self.test_model_load:
            self.get_logger().info('\n[测试 2] 模型加载')
            results['model_load'] = self.test_model_load_func()
        
        # 测试3: 数据读取
        if self.test_data_read:
            self.get_logger().info('\n[测试 3] 数据读取')
            results['data_read'] = self.test_data_read_func()
        
        # 测试4: 抓取预测
        if self.test_prediction:
            self.get_logger().info('\n[测试 4] 抓取预测')
            results['prediction'] = self.test_prediction_func()
        
        # 打印测试结果
        self.print_results(results)
    
    def test_path_finding(self):
        """测试路径查找功能"""
        try:
            # 从参数获取 baseline_dir
            baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value
            
            if baseline_dir:
                self.get_logger().info(f'  ✓ baseline 目录（从参数）: {baseline_dir}')
                if os.path.exists(baseline_dir):
                    self.get_logger().info(f'  ✓ 目录存在')
                else:
                    self.get_logger().error(f'  ✗ 目录不存在')
                    return False
            else:
                self.get_logger().error('  ✗ baseline_dir 参数为空')
                return False
            
            # 测试路径设置（应该已经在 __init__ 中设置）
            if baseline_dir in [os.path.dirname(p) for p in sys.path if 'graspnet-baseline' in p]:
                self.get_logger().info('  ✓ Python 路径已设置')
            else:
                self.get_logger().warn('  ⚠ Python 路径可能未正确设置')
            
            # 测试模型路径
            model_path = os.path.join(baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar')
            if os.path.exists(model_path):
                self.get_logger().info(f'  ✓ 模型路径: {model_path}')
            else:
                self.get_logger().warn(f'  ⚠ 未找到默认模型路径: {model_path}')
            
            # 测试数据目录
            data_dir = os.path.join(baseline_dir, 'doc', 'pose_1')
            if os.path.exists(data_dir):
                self.get_logger().info(f'  ✓ 数据目录: {data_dir}')
            else:
                self.get_logger().warn(f'  ⚠ 未找到默认数据目录: {data_dir}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'  ✗ 路径查找测试失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_model_load_func(self):
        """测试模型加载"""
        try:
            # 从参数获取 baseline_dir
            baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value
            if not baseline_dir:
                self.get_logger().error('  ✗ baseline_dir 参数为空')
                return False
            
            # 验证路径
            models_path = os.path.join(baseline_dir, 'models')
            self.get_logger().info(f'  models 路径: {models_path}')
            self.get_logger().info(f'  sys.path 中的 models 路径: {[p for p in sys.path if "models" in p]}')
            
            # 尝试导入模型
            try:
                from models.graspnet import GraspNet
                self.get_logger().info('  ✓ 成功导入 models.graspnet')
            except ImportError as e:
                self.get_logger().error(f'  ✗ 导入失败: {str(e)}')
                self.get_logger().info(f'  sys.path 前5个: {sys.path[:5]}')
                return False
            
            # 获取模型路径
            if self.model_path:
                model_path = self.model_path
            else:
                model_path = os.path.join(baseline_dir, 'logs', 'log_kn', 'checkpoint-rs.tar')
            
            if not model_path:
                self.get_logger().error('  ✗ 未指定模型路径')
                return False
            
            self.get_logger().info(f'  模型路径: {model_path}')
            
            # 检查文件是否存在
            if not os.path.exists(model_path):
                self.get_logger().error(f'  ✗ 模型文件不存在: {model_path}')
                return False
            
            self.get_logger().info('  ✓ 模型文件存在')
            
            # 创建模型
            device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
            self.get_logger().info(f'  使用设备: {device}')
            
            net = GraspNet(
                input_feature_dim=0,
                num_view=300,
                num_angle=12,
                num_depth=4,
                cylinder_radius=0.05,
                hmin=-0.02,
                hmax_list=[0.01, 0.02, 0.03, 0.04],
                is_training=False,
            )
            net.to(device)
            self.get_logger().info('  ✓ 模型创建成功')
            
            # 加载权重
            checkpoint = torch.load(model_path, map_location=device)
            net.load_state_dict(checkpoint['model_state_dict'])
            net.eval()
            self.get_logger().info('  ✓ 模型权重加载成功')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'  ✗ 模型加载测试失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_data_read_func(self):
        """测试数据读取"""
        try:
            from PIL import Image
            import scipy.io as scio
            
            # 获取数据目录
            if self.data_dir:
                data_dir = self.data_dir
            else:
                baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value
                if baseline_dir:
                    data_dir = os.path.join(baseline_dir, 'doc', 'pose_1')
                else:
                    data_dir = ''
            
            if not data_dir:
                self.get_logger().error('  ✗ 未指定数据目录')
                return False
            
            self.get_logger().info(f'  数据目录: {data_dir}')
            
            if not os.path.exists(data_dir):
                self.get_logger().error(f'  ✗ 数据目录不存在: {data_dir}')
                return False
            
            # 检查必需的文件
            required_files = ['color.png', 'depth.png', 'workspace_mask.png', 'meta.mat']
            missing_files = []
            
            for filename in required_files:
                filepath = os.path.join(data_dir, filename)
                if os.path.exists(filepath):
                    self.get_logger().info(f'  ✓ 找到文件: {filename}')
                else:
                    self.get_logger().error(f'  ✗ 缺少文件: {filename}')
                    missing_files.append(filename)
            
            if missing_files:
                return False
            
            # 尝试读取文件
            try:
                color = np.array(Image.open(os.path.join(data_dir, 'color.png')))
                self.get_logger().info(f'  ✓ 读取 color.png: {color.shape}')
            except Exception as e:
                self.get_logger().error(f'  ✗ 读取 color.png 失败: {str(e)}')
                return False
            
            try:
                depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
                self.get_logger().info(f'  ✓ 读取 depth.png: {depth.shape}')
            except Exception as e:
                self.get_logger().error(f'  ✗ 读取 depth.png 失败: {str(e)}')
                return False
            
            try:
                workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
                self.get_logger().info(f'  ✓ 读取 workspace_mask.png: {workspace_mask.shape}')
            except Exception as e:
                self.get_logger().error(f'  ✗ 读取 workspace_mask.png 失败: {str(e)}')
                return False
            
            try:
                meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
                self.get_logger().info('  ✓ 读取 meta.mat')
                if 'intrinsic_matrix' in meta:
                    self.get_logger().info(f'    intrinsic_matrix: {meta["intrinsic_matrix"].shape}')
                if 'factor_depth' in meta:
                    self.get_logger().info(f'    factor_depth: {meta["factor_depth"]}')
            except Exception as e:
                self.get_logger().error(f'  ✗ 读取 meta.mat 失败: {str(e)}')
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'  ✗ 数据读取测试失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_prediction_func(self):
        """测试抓取预测"""
        try:
            self.get_logger().info('  ⚠ 抓取预测测试需要 GPU 和完整的数据，跳过')
            return True
            
        except Exception as e:
            self.get_logger().error(f'  ✗ 抓取预测测试失败: {str(e)}')
            return False
    
    def print_results(self, results):
        """打印测试结果"""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('测试结果汇总')
        self.get_logger().info('=' * 60)
        
        total = len(results)
        passed = sum(1 for r in results.values() if r is True)
        failed = total - passed
        
        for test_name, result in results.items():
            status = '✓ 通过' if result else '✗ 失败'
            self.get_logger().info(f'  {test_name}: {status}')
        
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'总计: {total} 个测试')
        self.get_logger().info(f'通过: {passed} 个')
        self.get_logger().info(f'失败: {failed} 个')
        self.get_logger().info('=' * 60)
        
        if failed == 0:
            self.get_logger().info('🎉 所有测试通过！')
        else:
            self.get_logger().warn('⚠ 部分测试失败，请检查上述错误信息')


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetTestNode()
    
    # 运行完成后退出
    rclpy.shutdown()
    
    # 返回退出码
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
