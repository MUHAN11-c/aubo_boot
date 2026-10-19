from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'sim_worlds'


def _resolve_python():
    """工作区 venv 存在则用作 console_scripts shebang."""
    ws_venv = Path(__file__).resolve().parents[3] / 'aubo_py3.12' / 'bin' / 'python'
    if ws_venv.exists():
        return str(ws_venv)
    return sys.executable


def _files(directory: str) -> list:
    """目录下的一级文件（跳过子目录，setuptools 只拷普通文件）."""
    return [str(path) for path in sorted(Path(directory).glob('*'))
            if path.is_file()]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), _files('worlds')),
        (os.path.join('share', package_name, 'worlds', 'textures'),
         _files('worlds/textures')),
        (os.path.join('share', package_name, 'maps'), _files('maps')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Deterministic bagged-peach orchard scene generation (SDF + manifest + maps).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'generate_orchard = sim_worlds.cli:main',
            'generate_field = sim_worlds.field_gt:main',
            'scene_preview = sim_worlds.preview:main',
            'orchard_tour = sim_worlds.tour_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
