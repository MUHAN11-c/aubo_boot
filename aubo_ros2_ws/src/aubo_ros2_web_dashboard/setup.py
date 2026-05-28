"""ament_python 安装配置。

遵循 ROS 2 ament_python 规范：
  - data_files 安装到 share/<pkg>/ 下
  - console_scripts 注册 CLI 入口
  - install_requires 声明 Python 依赖
"""
import os
from collections import defaultdict
from glob import glob

from setuptools import find_packages, setup

PKG = "aubo_ros2_web_dashboard"
WEB_ROOT = os.path.join("web", "public")
WEB_INSTALL_SUBDIR = ("web", "public")


def _web_data_files():
    """递归收集 web/public 下所有文件，映射到 share/<pkg>/web/public/ 目录。"""
    by_dest = defaultdict(list)
    for root, _, files in os.walk(WEB_ROOT):
        for name in files:
            src = os.path.join(root, name)
            rel = os.path.relpath(src, WEB_ROOT)
            dest = os.path.join("share", PKG, *WEB_INSTALL_SUBDIR, os.path.dirname(rel))
            by_dest[dest].append(src)
    return sorted(by_dest.items())


def _robotwebtools_data_files():
    """部署 RobotWebTools runtime JS 资产到 web/public/js/robotwebtools/。

    源目录: ../robotwebtools/runtime_js_assets/
    目标:    share/<pkg>/web/public/js/robotwebtools/

    colcon build 时自动执行。若源目录不存在则优雅降级（空列表）。
    开发阶段也可手动创建符号链接:
      ln -s ../../../robotwebtools/runtime_js_assets web/public/js/robotwebtools
    """
    rwt_src = os.path.join(os.path.dirname(__file__), "..", "robotwebtools", "runtime_js_assets")
    if not os.path.isdir(rwt_src):
        return []
    by_dest = defaultdict(list)
    for root, _, files in os.walk(rwt_src):
        for name in files:
            src = os.path.join(root, name)
            rel = os.path.relpath(src, rwt_src)
            dest = os.path.join("share", PKG, "web", "public", "js", "robotwebtools", os.path.dirname(rel))
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
        (f"share/{PKG}/docs", glob("docs/*.md")),
    ] + _web_data_files() + _robotwebtools_data_files(),
    install_requires=[
        "setuptools",
        "fastapi>=0.100.0",
        "uvicorn[standard]>=0.22.0",
        "httpx>=0.25.0",
        "websockets>=12.0",
        "pyyaml>=6.0",
    ],
    zip_safe=False,
    maintainer="IVG",
    maintainer_email="maintainer@example.com",
    description="IVG Web Dashboard: FastAPI gateway + ROSLIB.js frontend for ROS 2 robot control",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "ivg_fastapi_static_gateway = aubo_ros2_web_dashboard.fastapi_static_gateway:main",
        ],
    },
)
