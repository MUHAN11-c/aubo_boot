# 尽量屏蔽系统 pkg_resources 的 DeprecationWarning（若仍出现，构建时使用：
#   PYTHONWARNINGS=ignore::DeprecationWarning colcon build --packages-select aubo_robot_simulator_ros2）
import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning, module='pkg_resources')

from setuptools import setup
import os
from glob import glob

package_name = 'aubo_robot_simulator_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaosong',
    maintainer_email='834031119@qq.com',
    description='ROS2 trajectory interpolator for aubo: joint_path_command -> moveItController_cmd',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aubo_robot_simulator_node = aubo_robot_simulator_ros2.aubo_robot_simulator_node:main',
        ],
    },
)
