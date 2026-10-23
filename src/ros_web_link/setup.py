import os

from setuptools import find_packages, setup

package_name = 'ros_web_link'


_SKIP_DIRS = {'__pycache__', 'node_modules', '.pytest_cache', '.mimosa', '.v2c'}


def _data_tree(base_rel: str, dest_sub: str):
    """收集一棵源码树到 share/<pkg>/<dest_sub>（跳过缓存/依赖目录）."""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), base_rel)
    files = []
    for root, dirs, filenames in os.walk(base):
        dirs[:] = sorted(d for d in dirs if d not in _SKIP_DIRS)
        for f in sorted(filenames):
            if f.endswith('.pyc'):
                continue
            full = os.path.join(root, f)
            rel = os.path.relpath(full, base)
            dest_dir = os.path.dirname(rel)
            dest = 'share/' + package_name + '/' + dest_sub
            if dest_dir:
                dest += '/' + dest_dir
            # setup() 要求相对路径（相对于 setup.py 所在目录）
            files.append((dest, [os.path.join(base_rel, rel)]))
    return files


def _web_data_files():
    """收集 web/public 下的通信层 JS/console/示例资产."""
    return _data_tree('web/public', 'web/public')


setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'CHANGELOG.rst']),
        ('share/' + package_name + '/config', ['config/defaults.yaml']),
        ('share/' + package_name + '/launch', ['launch/transport.launch.xml',
                                               'launch/monitor.launch.py',
                                               'launch/official_demo.launch.py',
                                               'launch/sim_demo.launch.py',
                                               'launch/panda_moveit_web.launch.py']),
    ] + _web_data_files() + _data_tree('examples', 'examples') \
      + _data_tree('doc', 'doc'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhan11',
    maintainer_email='wangxiaoyun@iscas.ac.cn',
    description='Independent Web-ROS2 gateway + JS client + monitor console with API examples',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ros_web_link_gateway = ros_web_link.cli:main',
            'rwl_comm_fixture = ros_web_link.comm_fixture:main',
        ],
    },
)
