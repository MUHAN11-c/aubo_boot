import os
from pathlib import Path
import sys

from setuptools import find_packages, setup


def _venv_python() -> str:
    """工作区 aubo_py3.12 存在则用作 console_scripts shebang（torch/open3d）."""
    ws_venv = Path(__file__).resolve().parents[2] / 'aubo_py3.12' / 'bin' / 'python'
    if ws_venv.exists():
        return str(ws_venv)
    return sys.executable


package_name = 'peach_harvester'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', f) for f in sorted(os.listdir('launch'))
          if f.endswith('.launch.py')]),
        (os.path.join('share', package_name, 'config'),
         [os.path.join('config', f) for f in sorted(os.listdir('config'))
          if f.endswith(('.yaml', '.param.yaml'))]),
        (os.path.join('share', package_name, 'model'),
         [os.path.join('model', f) for f in sorted(os.listdir('model'))
          if not f.startswith('.')]),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='套袋桃采收大脑：vision（看+建）与 supervisor（批+编排）一包两进程。',
    license='BSD-3-Clause',
    # colcon test 的 pytest 探测只认 extras_require（legacy tests_require 会静默
    # 退回 unittest 且 0 测试判失败；peach_bringup 同款写法）。
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'peach_harvester = peach_harvester.brain:main',
            'peach_scene_perception_node = '
            'peach_harvester.vision.scene_perception.scene_perception_node:main',
            'peach_target_reconstruction_node = '
            'peach_harvester.vision.target_reconstruction.target_reconstruction_node:main',
            'peach_supervisor = '
            'peach_harvester.supervisor.executor_node:main',
            'peach_lifecycle_manager = '
            'peach_harvester.supervisor.lifecycle_manager:main',
        ],
    },
    options={'build_scripts': {'executable': _venv_python()}},
)
