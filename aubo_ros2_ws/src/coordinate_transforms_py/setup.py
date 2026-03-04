from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'coordinate_transforms_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    extras_require={'opencv': ['opencv-python'], 'viz': ['matplotlib']},
    zip_safe=False,
    maintainer='mu',
    maintainer_email='2155413529@qq.com',
    description='Python coordinate transforms (core + ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coord_tf_demo_node = coordinate_transforms_py.ros2.coord_tf_demo_node:main',
            'run_core_demo = coordinate_transforms_py.run_core_demo:main',
            'run_visualization_demo = coordinate_transforms_py.run_visualization_demo:main',
            'run_opencv_process_demo = coordinate_transforms_py.run_opencv_process_demo:main',
        ],
    },
)
