from setuptools import setup
import os
from glob import glob

package_name = 'visual_pose_estimation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # 使用 scripts 参数安装 Python 脚本
    scripts=[
        'scripts/trigger_depth.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='基于单目相机模板匹配的工件三维抓取姿态估计算法 - C++ ROS2实现',
    license='MIT',
    tests_require=['pytest'],
)
