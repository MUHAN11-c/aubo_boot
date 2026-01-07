from setuptools import setup
import os
from glob import glob

package_name = 'hand_eye_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
        (os.path.join('share', package_name, 'web/static'), glob('web/static/*.*')),
        (os.path.join('share', package_name, 'web/templates'), glob('web/templates/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'config/calibrationdata'), glob('config/calibrationdata/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='单目相机手眼标定工具',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_eye_calibration_node = hand_eye_calibration.hand_eye_calibration_node:main',
            'image_data_converter_node = hand_eye_calibration.image_data_converter_node:main',
        ],
    },
)

