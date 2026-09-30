from glob import glob
import os
from pathlib import Path
import sys

from setuptools import setup

package_name = 'peach2_perception'


def _venv_python() -> str:
    # The node imports torch/ultralytics, which only exist in the workspace venv; the installed
    # entry script must therefore start with the venv interpreter (the venv sees system
    # site-packages, so rclpy/cv_bridge still resolve to the Jazzy apt copies).
    for parent in Path(__file__).resolve().parents:
        venv = parent / 'aubo_py3.12' / 'bin' / 'python'
        if venv.exists():
            return str(venv)
    return sys.executable


setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='wjz@example.com',
    description='Peach v2 single-frame bag perception (YOLO + MobileSAM + landmarks + tracking).',
    license='BSD-3-Clause',
    options={'build_scripts': {'executable': _venv_python()}},
    entry_points={
        'console_scripts': [
            'perception_node = peach2_perception.perception_node:main',
        ],
    },
)
