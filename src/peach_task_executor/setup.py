from glob import glob
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'peach_task_executor'


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
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach harvest task executor (explicit RunHarvest).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_task_executor = peach_task_executor.executor_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
