from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup

if len(sys.argv) < 2 or sys.argv[1] != 'clean':
    from generate_parameter_library_py.setup_helper import generate_parameter_module
    generate_parameter_module(
        'target_reconstruction_parameters',
        'config/target_reconstruction_parameters.yaml',
    )

package_name = 'peach_target_reconstruction'


def _venv_python():
    """Prefer workspace aubo_py3.12 for torch/open3d entry points."""
    ws_venv = Path(__file__).resolve().parents[3] / 'aubo_py3.12' / 'bin' / 'python'
    if ws_venv.exists():
        return str(ws_venv)
    return sys.executable


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml', 'README.md']),
]
if os.path.isdir('launch'):
    data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))
if os.path.isdir('config'):
    data_files.append((os.path.join('share', package_name, 'config'), glob('config/*.yaml')))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach target reconstruction lifecycle node.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
            'peach_target_reconstruction_node = '
            'peach_target_reconstruction.target_reconstruction_node:main',
        ]},
    options={'build_scripts': {'executable': _venv_python()}},
)
