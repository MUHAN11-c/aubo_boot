from glob import glob
import os

from setuptools import setup

package_name = 'sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.config')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Sim sub-workspace bringup: gz world spawn, bridges, Nav2, perception launches.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'orchard_harvest = sim_bringup.orchard_harvest:main',
            'odom_tf_relay = sim_bringup.odom_tf_relay:main',
        ],
    },
)
