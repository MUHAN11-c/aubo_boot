from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'peach_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'peach_photo_pose_init = peach_bringup.photo_pose_init:main',
        ],
    },
)
