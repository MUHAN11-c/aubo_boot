from setuptools import find_packages, setup

package_name = 'peach_common_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjz',
    maintainer_email='2155413529@qq.com',
    description='桃子采摘链路公共纯核库（零 ROS import，ros 子包除外）。',
    license='BSD-3-Clause',
    # 纯库包：无 console_scripts，不需要 venv 解释器重定向
    entry_points={},
)
