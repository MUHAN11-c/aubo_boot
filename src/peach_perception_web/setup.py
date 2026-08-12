from glob import glob

from setuptools import find_packages, setup


package_name = 'peach_perception_web'


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
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Harvest task center for perception, reconstruction and motion.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'peach_perception_web = peach_perception_web.gateway:main',
        ],
    },
)
