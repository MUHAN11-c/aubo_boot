from glob import glob
import os

from setuptools import setup

package_name = 'sim_perception'

setup(
    name=package_name,
    version='0.1.0',
    # The historical vision snapshot remains in source for reference only.
    # Runtime behavior belongs to the existing Peach capability packages.
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'README.md', 'LICENSE']),
        (os.path.join('share', package_name, 'config'), [
            'config/scene_perception.yaml', 'config/target_reconstruction.yaml']),
        (os.path.join('share', package_name, 'doc'), glob('doc/*.md')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Simulation parameter overlays and thin Peach vision entry points.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'sim_scene_perception_node = '
            'sim_perception.scene_node:main',
            'sim_target_reconstruction_node = '
            'sim_perception.reconstruction_node:main',
        ],
    },
)
