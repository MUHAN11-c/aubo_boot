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
    description='rosbridge + official RobotWebTools ros2-web-bridge web demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
