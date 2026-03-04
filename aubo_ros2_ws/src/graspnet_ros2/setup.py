import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'graspnet_ros2'

# 收集 graspnet-baseline 目录下的文件
def collect_baseline_data_files():
    """收集 graspnet-baseline 目录下的数据文件"""
    baseline_dir = os.path.join(os.path.dirname(__file__), 'graspnet-baseline')
    data_files = []
    
    if not os.path.exists(baseline_dir):
        return data_files
    
    # 排除的目录和文件
    exclude_dirs = {'.git', '__pycache__', '.pytest_cache', 'dist', 'build', '*.egg-info', '.ipynb_checkpoints'}
    exclude_extensions = {'.pyc', '.pyo', '.pyd', '.so', '.egg'}
    
    # 遍历目录收集文件
    for root, dirs, files in os.walk(baseline_dir):
        # 过滤排除的目录
        dirs[:] = [d for d in dirs if d not in exclude_dirs and not d.startswith('.')]
        
        # 收集文件
        collected_files = []
        for filename in files:
            # 跳过排除的文件
            if any(filename.endswith(ext) for ext in exclude_extensions):
                continue
            if filename.startswith('.'):
                continue
            
            filepath = os.path.join(root, filename)
            relpath = os.path.relpath(filepath, baseline_dir)
            collected_files.append(filepath)
        
        if collected_files:
            # 计算目标目录
            rel_dir = os.path.relpath(root, baseline_dir)
            if rel_dir == '.':
                dest_dir = os.path.join('share', package_name, 'graspnet-baseline')
            else:
                dest_dir = os.path.join('share', package_name, 'graspnet-baseline', rel_dir)
            
            data_files.append((dest_dir, collected_files))
    
    return data_files

# 获取 baseline 数据文件
baseline_data_files = collect_baseline_data_files()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # 配置文件（RViz、参数等）
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.rviz') + glob('config/*.yaml') + glob('config/*.yml') + glob('config/*.json')),
    ] + baseline_data_files,
    install_requires=[
        'setuptools',
        # 注意：numpy, torch, open3d, scipy, Pillow 等依赖
        # 应该在 conda 环境（ros2_env）中预先安装
        # 这里不列出以避免包管理冲突
    ],
    zip_safe=False,  # 设置为 False 以支持数据文件访问
    maintainer='mu',
    maintainer_email='2155413529@qq.com',
    description='GraspNet ROS2 功能包：6自由度抓取位姿预测',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_saver = graspnet_ros2.image_saver:main',
            'graspnet_node = graspnet_ros2.graspnet_node:main',
            'graspnet_demo_node = graspnet_ros2.graspnet_demo_node:main',
            'graspnet_test_node = graspnet_ros2.test_node:main',
            'publish_grasps_client = graspnet_ros2.publish_grasps_client:main',
            'grasp_move_service = graspnet_ros2.grasp_move_service:main',
            'grasp_move_client = graspnet_ros2.grasp_move_client:main',
        ],
    },
)
