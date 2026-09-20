from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'peach_observability'


def _resolve_python():
    """工作区 venv 存在则用作 console_scripts shebang."""
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
        (os.path.join('share', package_name), ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         [os.path.join('config', f) for f in sorted(os.listdir('config'))
          if f.endswith('.yaml')]),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach observability and rosbag2 recording.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_observability = peach_observability.observability_node:main',
            'peach_bag_report = peach_observability.bag_report:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
