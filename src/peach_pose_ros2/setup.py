from glob import glob
from setuptools import find_packages, setup

package_name = 'peach_pose_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='wjz@example.com',
    description='PeachPose ROS 2 perception node for AUBO E5 workspace.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'peach_pose_node = peach_pose_ros2.peach_pose_node:main',
            'dataset_replayer = peach_pose_ros2.dataset_replayer:main',
        ],
    },
)
