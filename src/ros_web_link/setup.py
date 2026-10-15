import os

from setuptools import find_packages, setup

package_name = 'ros_web_link'


def _web_data_files():
    """收集 web/public/js 下的通信层 JS 资产."""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web', 'public')
    files = []
    for root, _, filenames in os.walk(base):
        for f in filenames:
            full = os.path.join(root, f)
            rel = os.path.relpath(full, base)
            dest_dir = os.path.dirname(rel)
            if dest_dir:
                dest = 'share/' + package_name + '/web/public/' + dest_dir
            else:
                dest = 'share/' + package_name + '/web/public'
            # setup() 要求相对路径（相对于 setup.py 所在目录）
            files.append((dest, [os.path.join('web/public', rel)]))
    return files


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/defaults.yaml']),
        ('share/' + package_name + '/launch', ['launch/transport.launch.xml',
                                               'launch/official_demo.launch.py',
                                               'launch/sim_demo.launch.py',
                                               'launch/panda_moveit_web.launch.py']),
    ] + _web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhan11',
    maintainer_email='wangxiaoyun@iscas.ac.cn',
    description='Web-ROS2 transport layer: Python gateway + JS client library',
    license='Apache-2.0',
    # tests_require 在 setuptools>=72 被忽略（colcon 依赖它探测 pytest runner），
    # 用 extras_require.test 同义表达——与 aubo_ros2_web_dashboard 同款
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ros_web_link_gateway = ros_web_link.cli:main',
            'rwl_comm_fixture = ros_web_link.comm_fixture:main',
        ],
    },
)
