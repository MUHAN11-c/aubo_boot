from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'serial_imu'


def _resolve_python():
    """工作区 aubo_py3.12 存在则用作 console_scripts shebang."""
    ws_venv = Path(__file__).resolve().parents[2] / 'aubo_py3.12' / 'bin' / 'python'
    if ws_venv.exists():
        return str(ws_venv)
    return sys.executable


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='USB serial IMU driver publishing sensor_msgs/Imu.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'serial_imu_node = serial_imu.imu_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
