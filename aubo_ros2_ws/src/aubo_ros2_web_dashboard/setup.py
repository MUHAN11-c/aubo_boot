"""
ament_python 包定义：
  aubo_ros2_web_dashboard：占位包名（resource/ 索引）；
  data_files：launch、package.xml、静态 web 资源安装到 share。
"""
import os
from collections import defaultdict
from glob import glob

from setuptools import setup

PKG = 'aubo_ros2_web_dashboard'
# HTTP 文档根（与 launch 中 threaded_static_server --directory 一致）
WEB_ROOT = os.path.join('web', 'public')
WEB_INSTALL_SUBDIR = ('web', 'public')


def web_data_files():
    by_dest = defaultdict(list)
    for root, _, files in os.walk(WEB_ROOT):
        for name in files:
            src = os.path.join(root, name)
            rel = os.path.relpath(src, WEB_ROOT)
            dest_dir = os.path.join('share', PKG, *WEB_INSTALL_SUBDIR, os.path.dirname(rel))
            by_dest[dest_dir].append(src)
    return sorted(by_dest.items())


setup(
    name=PKG,
    version='0.2.0',
    packages=[PKG],
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
    description='IVG web dashboard: rosbridge_suite (roslib/WebSocket) + static RobotWebTools demo UI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ivg_threaded_static_server = aubo_ros2_web_dashboard.threaded_static_server:main',
        ],
    },
)
