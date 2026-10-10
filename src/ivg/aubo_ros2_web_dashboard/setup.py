"""
ament_python 安装配置.

遵循 ROS 2 ament_python 规范：
  - data_files 安装到 share/<pkg>/ 下
  - console_scripts 注册 CLI 入口
2026-09-30 移植适配：robotwebtools 前端库已内嵌 web/public（原仓为指向
独立仓的符号链接），删除其单独 data_files 收集逻辑。
"""
import os
from collections import defaultdict
from glob import glob

from setuptools import find_packages, setup

PKG = "aubo_ros2_web_dashboard"
WEB_ROOT = os.path.join("web", "public")


def _web_data_files():
    """递归收集 web/public 下所有文件，映射到 share/<pkg>/web/public/ 目录."""
    by_dest = defaultdict(list)
    for root, _, files in os.walk(WEB_ROOT):
        for name in files:
            src = os.path.join(root, name)
            rel = os.path.relpath(src, WEB_ROOT)
            dest = os.path.join("share", PKG, "web", "public", os.path.dirname(rel))
            by_dest[dest].append(src)
    return sorted(by_dest.items())


setup(
    name=PKG,
    version="0.4.0",
    packages=find_packages(where=".", include=["aubo_ros2_web_dashboard*"]),
    python_requires=">=3.10",
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.py")),
        (f"share/{PKG}/config", glob("config/*.yaml")),
    ] + _web_data_files(),
    install_requires=[
        "setuptools",
    ],
    zip_safe=False,
    maintainer="muhan11",
    maintainer_email="wangxiaoyun@iscas.ac.cn",
    description="IVG Web Dashboard: FastAPI gateway + ROSLIB.js frontend for ROS 2 robot control",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "ivg_fastapi_static_gateway = aubo_ros2_web_dashboard.gateway.cli:main",
        ],
    },
)
