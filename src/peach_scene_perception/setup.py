from glob import glob
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'peach_scene_perception'


def _resolve_python():
    """
    解析节点要用的 Python 解释器（console_scripts shebang 指向它）.

    工作区 `aubo_py3.12` venv 存在则用之，否则回退构建解释器。
    经 options.build_scripts.executable 写入 console_scripts 启动器的
    shebang（ROS 2 官方文档 Using Python Packages with ROS 2 的做法）；
    路径在构建期解析，源码不写死绝对路径，工作区迁移后重建即自动修正。
    """
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
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='PeachPose ROS 2 perception node for AUBO E5 workspace.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_scene_perception_node = peach_scene_perception.peach_pose_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
