from setuptools import setup
import os
from glob import glob

package_name = 'vision_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/video_*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mu',
    maintainer_email='2155413529@qq.com',
    description='视觉感知功能包：MediaPipe Holistic + YOLOv8 OBB',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe_holistic_node = vision_perception.mediapipe_holistic_node:main',
            'yolo_obb_node = vision_perception.yolo_obb_node:main',
            'video_publisher_node = vision_perception.video_publisher_node:main',
        ],
    },
)
