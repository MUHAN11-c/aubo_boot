from glob import glob
import os

from setuptools import setup

package_name = 'peach2_observability'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='wjz@example.com',
    description='Peach v2 read-only observability (HTTP 8091, session bag).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach2_observability = peach2_observability.observability_node:main',
        ],
    },
)
