from glob import glob

from setuptools import find_packages, setup

package_name = 'peach_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach bagging mission orchestration (P0 skeleton).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mission_node = peach_motion.mission_node:main',
        ],
    },
)
