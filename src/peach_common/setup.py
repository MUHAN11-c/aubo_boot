import os

from setuptools import setup

package_name = 'peach_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Peach shared param/QoS plumbing: single source since W1.',
    license='BSD-3-Clause',
)
