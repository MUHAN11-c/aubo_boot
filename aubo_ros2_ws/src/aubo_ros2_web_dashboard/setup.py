"""
ament_python 包定义：
  aubo_ros2_web_dashboard：占位包名（resource/ 索引）；
  ivg_gateway：FastAPI 网关源码，位于 gateway/ivg_gateway，随 colcon 装入 Python 路径；
  data_files：launch、package.xml、静态 web 资源安装到 share。
"""
import os
from collections import defaultdict
from glob import glob

from setuptools import setup

PKG = 'aubo_ros2_web_dashboard'
WEB_ROOT = os.path.join('web', 'ros2_web_bridge_demo')


def web_data_files():
    by_dest = defaultdict(list)
    for root, _, files in os.walk(WEB_ROOT):
        for name in files:
            src = os.path.join(root, name)
            rel = os.path.relpath(src, WEB_ROOT)
            dest_dir = os.path.join('share', PKG, 'web', 'ros2_web_bridge_demo', os.path.dirname(rel))
            by_dest[dest_dir].append(src)
    return sorted(by_dest.items())


setup(
    name=PKG,
    version='0.2.0',
    packages=[PKG, 'ivg_gateway'],
    # ivg_gateway 物理路径在 gateway 子目录，与 ament 包根并列
    package_dir={'ivg_gateway': os.path.join('gateway', 'ivg_gateway')},
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PKG}']),
        (f'share/{PKG}', ['package.xml']),
        (f'share/{PKG}/launch', glob('launch/*.py')),
    ]
    + web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IVG',
    maintainer_email='maintainer@example.com',
    description='rosbridge + official RobotWebTools ros2-web-bridge web demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
