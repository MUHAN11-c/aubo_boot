from glob import glob

from setuptools import find_packages, setup

package_name = 'aubo_hand_eye_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='Automatic eye-in-hand calibration for AUBO E5.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'calibration_server = aubo_hand_eye_calibration.server:main',
            'extrinsics_publisher = aubo_hand_eye_calibration.extrinsics_publisher:main',
            'web_gateway = aubo_hand_eye_calibration.web_gateway:main',
        ],
    },
)
