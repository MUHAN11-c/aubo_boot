from glob import glob
import contextlib
import io
import os
from pathlib import Path
import sys

from generate_parameter_library_py.generate_python_module import (
    run as write_parameter_module)
from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

package_name = 'peach_perception'


def _generate_parameter_module(module_name, yaml_file):
    """写入 install（colcon argv）以及源码包目录.

    仅写 install 时，若 PYTHONPATH 把 src/peach_perception 放在前面，
    运行会找不到生成的 *_parameters 模块。
    colcon 刮 setup 元数据时带 --dry-run，此时不得向 stdout 打生成日志。
    """
    generate_parameter_module(module_name, yaml_file)
    if '--dry-run' in sys.argv:
        return
    dest = Path(__file__).resolve().parent / package_name / f'{module_name}.py'
    with contextlib.redirect_stdout(io.StringIO()):
        write_parameter_module(str(dest), yaml_file)


# 参数库官方装载链：config/<节点>_parameters.yaml 为声明/类型/默认值/描述/
# 校验的单一事实源，构建期生成 peach_perception 包内 *_parameters 模块。
_generate_parameter_module(
    'scene_perception_parameters',
    'config/scene_perception_parameters.yaml')
_generate_parameter_module(
    'target_reconstruction_parameters',
    'config/target_reconstruction_parameters.yaml')


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
        (os.path.join('share', package_name), ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach scene observation and single-target reconstruction.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_scene_perception_node = '
            'peach_perception.scene_perception.scene_perception_node:main',
            'peach_target_reconstruction_node = '
            'peach_perception.target_reconstruction.target_reconstruction_node:main',
            'peach_bag_baseline = '
            'peach_perception.scene_perception.offline.bag_baseline:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
)
