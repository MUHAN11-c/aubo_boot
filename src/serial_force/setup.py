from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'serial_force'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name),
         ['package.xml', 'LICENSE', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'doc'),
         glob('doc/*.md') + glob('doc/*.png')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='USB serial 5-point force sensor driver and simulation tools.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'serial_force_node = serial_force.force_node:main',
            'force_sim_node = serial_force.force_sim_node:main',
            'force_cop_control = serial_force.cop_control:main',
            'force_cop_trials = serial_force.cop_trials:main',
            'force_cop_analyze = serial_force.cop_analysis:main',
        ],
    },
)
