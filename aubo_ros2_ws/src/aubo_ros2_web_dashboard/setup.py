"""
ament_python 包定义（目录与 ``ros2 pkg create --build-type ament_python`` 对齐）：

- ``resource/<pkg>/``：ament 索引标记
- ``<pkg>/``：可导入的 Python 包（网关代码）
- ``launch/``：ROS 2 Launch 描述
- ``test/``：pytest（``colcon test`` / ``pytest``）
- ``web/public/``：静态资源，安装到 ``share/<pkg>/web/public/``
- ``docs/``：架构与维护用 Markdown（可选安装到 share）
"""
import os #路径操作
from collections import defaultdict #字典操作
from glob import glob #文件操作

from setuptools import find_packages, setup #包管理

# 与 package.xml <name>、resource/ 下标记文件名一致
PKG = 'aubo_ros2_web_dashboard'
# 静态站点源目录（相对「含本 setup.py 的包根」）；安装目标为 share/<PKG>/web/public/
# （与 launch 中 get_package_share_directory(...) + web/public 一致）。仅当 setuptools/colcon
# 在该包根为当前工作目录执行时，os.walk(WEB_ROOT) 才能找到文件；独立脚本勿照搬此相对路径。
WEB_ROOT = os.path.join('web', 'public')
WEB_INSTALL_SUBDIR = ('web', 'public')


def web_data_files():
	"""遍历 web/public，生成 data_files 列表，保留子目录结构（js/css/vendor 等）。"""
	by_dest = defaultdict(list)
	for root, _, files in os.walk(WEB_ROOT):
		for name in files:
			src = os.path.join(root, name)
			rel = os.path.relpath(src, WEB_ROOT)
			dest_dir = os.path.join('share', PKG, *WEB_INSTALL_SUBDIR, os.path.dirname(rel))
			by_dest[dest_dir].append(src)
	return sorted(by_dest.items())


setup(
	name=PKG,
	# 与 package.xml <version> 对齐
	version='0.4.0',
	# 含 aubo_ros2_web_dashboard.gateway 等子包；不把顶层 test/ 当作 Python 包收录
	packages=find_packages(where='.', include=['aubo_ros2_web_dashboard*']),
	data_files=[
		# ament 索引：标记本包已安装
		('share/ament_index/resource_index/packages', [f'resource/{PKG}']),
		(f'share/{PKG}', ['package.xml']),
		(f'share/{PKG}/launch', glob('launch/*.py')),
		(f'share/{PKG}/docs', glob('docs/*.md')),
	]
	+ web_data_files(),
	# ROS 侧依赖见 package.xml；此处为 pip 侧（FastAPI 网关进程）
	install_requires=[
		'setuptools',
		'fastapi>=0.100.0',
		'uvicorn[standard]>=0.22.0',
		'httpx>=0.25.0',
		'websockets>=12.0',
	],
	zip_safe=True,
	maintainer='IVG',
	maintainer_email='maintainer@example.com',
	description='IVG web dashboard: rosbridge + FastAPI static gateway for web/public (RobotWebTools / roslibjs)',
	license='Apache-2.0',
	# 与 pytest.ini（testpaths=test）配合；官方示例多用 extras_require['test']，此处沿用 tests_require
	tests_require=['pytest'],
	entry_points={
		# 实际安装路径由 setup.cfg 的 install_scripts 指向 lib/<pkg>/
		'console_scripts': [
			'ivg_fastapi_static_gateway = aubo_ros2_web_dashboard.fastapi_static_gateway:main',
			'ivg_pointcloud_web_throttle = aubo_ros2_web_dashboard.pointcloud_web_throttle:main',
		],
	},
)
