"""
ament_python 包安装配置：Python 包、launch、文档与 RWT 前端静态文件。

前端文件通过 dist_data_files 安装到 share/aubo_ros2_web_dashboard/web/dist/，运行时用 ament_index 解析。
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aubo_ros2_web_dashboard"


def dist_data_files() -> list[tuple[str, list[str]]]:
    """将 frontend/dist 下所有文件按相对路径安装到 share/<包名>/web/dist/。"""
    base = "frontend/dist"
    if not os.path.isdir(base):
        return []
    files = [p for p in glob(os.path.join(base, "**", "*"), recursive=True) if os.path.isfile(p)]
    grouped: dict[str, list[str]] = {}
    for path in files:
        rel = os.path.relpath(path, base)
        install_dir = os.path.join("share", package_name, "web", "dist", os.path.dirname(rel))
        grouped.setdefault(install_dir, []).append(path)
    return sorted(grouped.items())


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "doc"), glob("docs/*.md")),
    ]
    + dist_data_files(),
    install_requires=["setuptools", "fastapi", "uvicorn[standard]", "httpx>=0.27"],
    zip_safe=True,
    maintainer="mu",
    maintainer_email="mu@example.com",
    description="ROS2 web dashboard via rosbridge + FastAPI + HTML/CSS/JS frontend",
    license="Apache-2.0",
    tests_require=["pytest", "httpx"],
    entry_points={
        "console_scripts": [
            "web_dashboard = aubo_ros2_web_dashboard.web.server:main",
        ],
    },
)
