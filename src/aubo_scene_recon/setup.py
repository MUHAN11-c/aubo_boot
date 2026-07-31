from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'aubo_scene_recon'
script = Path('scripts/recon_fusion_node')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        # venv wrapper（不用 console_scripts，避免系统 python3 缺 open3d）
        ('lib/' + package_name, [str(script)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Eye-in-hand scene reconstruction with Open3D.',
    license='BSD-3-Clause',
)
