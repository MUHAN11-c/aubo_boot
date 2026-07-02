from setuptools import setup

package_name = 'ivg_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mu',
    maintainer_email='mu@example.com',
    description='IVG2.0 shared utilities: math, IO pin definitions, robot constants, TF cache',
    license='MIT',
    entry_points={},
)
