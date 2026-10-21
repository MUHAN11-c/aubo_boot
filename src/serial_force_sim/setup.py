from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

package_name = 'serial_force_sim'


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
        (os.path.join('share', package_name), ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'doc'), glob('doc/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='5-point force CoP closed-loop alignment simulation (mock arm).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'fake_force = serial_force_sim.fake_force:main',
            'cop_web = serial_force_sim.cop_web:main',
            'cop_control = serial_force_sim.cop_control:main',
            'cop_trials = serial_force_sim.trial_runner:main',
            'cop_analyze = serial_force_sim.analysis:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
