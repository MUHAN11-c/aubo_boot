from glob import glob
import os
import sys

from setuptools import find_packages, setup

if len(sys.argv) < 2 or sys.argv[1] != 'clean':
    from generate_parameter_library_py.setup_helper import generate_parameter_module
    generate_parameter_module(
        'flag_bridge_parameters', 'config/flag_bridge_parameters.yaml')
    generate_parameter_module(
        'autostart_parameters', 'config/autostart_parameters.yaml')

package_name = 'peach_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach harvest stack bringup.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_lifecycle_flag_bridge = '
            'peach_bringup.lifecycle_flag_bridge:main',
            'peach_autostart_client = peach_bringup.autostart_client:main',
        ],
    },
)
