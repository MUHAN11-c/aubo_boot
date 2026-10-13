from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'sim_perception'


def _resolve_python():
    """工作区 venv 存在则用作 console_scripts shebang（torch/open3d 走 venv）."""
    ws_venv = Path(__file__).resolve().parents[3] / 'aubo_py3.12' / 'bin' / 'python'
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
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Sim-sub-workspace snapshot of the peach vision stack (perception + reconstruction).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'sim_scene_perception_node = '
            'sim_perception.vision.scene_perception.scene_perception_node:main',
            'sim_target_reconstruction_node = '
            'sim_perception.vision.target_reconstruction.target_reconstruction_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
