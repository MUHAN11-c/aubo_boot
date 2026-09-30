import os

from setuptools import setup

package_name = 'peach2_core'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='wjz@example.com',
    description='Peach v2 pure-Python core: landmarks, tracker, fusion, tool budget.',
    license='BSD-3-Clause',
)
