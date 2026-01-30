from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'visual_pose_estimation_python'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'configs'),
            glob('configs/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mu',
    maintainer_email='mu@example.com',
    description='Python implementation of visual pose estimation for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_pose_estimation_node = visual_pose_estimation_python.main:main',
        ],
    },
)
