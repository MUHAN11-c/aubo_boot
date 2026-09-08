from glob import glob

from setuptools import find_packages, setup

package_name = 'graspnet_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 模型权重（12MB，GraspNet-baseline checkpoint）安装到 share 供节点默认加载
        ('share/' + package_name + '/models', glob('models/*.tar')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=False,
    maintainer='mu',
    maintainer_email='2155413529@qq.com',
    description=(
        'GraspNet 6-DOF grasp detection and execution '
        '(point cloud to grasp poses via MoveIt), pure-torch backend.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'graspnet_demo_points_node = graspnet_ros2.graspnet_node:main',
            'publish_grasps_client = graspnet_ros2.publish_grasps_client:main',
        ],
    },
)
