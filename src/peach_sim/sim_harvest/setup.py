from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sim_harvest'
setup(
    name=package_name, version='0.2.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml', 'LICENSE', 'README.md']),
                (os.path.join('share', package_name, 'doc'), glob('doc/*.md'))],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='wjz', maintainer_email='2155413529@qq.com',
    description='Nav2 mission and MoveIt/Gazebo harvesting actions.', license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'grasp_node = sim_harvest.grasp_node:main',
        'base_guard = sim_harvest.base_guard:main',
        'mission_node = sim_harvest.mission_node:main',
        'run_harvest = sim_harvest.run_harvest:main']},
)
