import os
from glob import glob

from setuptools import find_packages, setup

package_name = "visual_pose_estimation_python"


def package_files(directory: str, patterns: list[str]) -> list[tuple[str, list[str]]]:
    data_files: list[tuple[str, list[str]]] = []
    for pattern in patterns:
        matched_files = [
            path for path in glob(os.path.join(directory, "**", pattern), recursive=True) if os.path.isfile(path)
        ]
        if not matched_files:
            continue

        grouped: dict[str, list[str]] = {}
        for path in matched_files:
            install_dir = os.path.join("share", package_name, os.path.dirname(path))
            grouped.setdefault(install_dir, []).append(path)
        data_files.extend(sorted(grouped.items()))
    return data_files

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=(
        [
            ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
            (os.path.join("share", package_name), ["package.xml"]),
            (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
            (
                os.path.join("share", package_name, "web_ui"),
                glob("web_ui/*.html") + glob("web_ui/*.md") + glob("web_ui/*.txt") + glob("web_ui/*.sh"),
            ),
        ]
        + package_files("web_ui/configs", ["*.json", "*.yaml", "*.xml", "*.md"])
        + package_files("web_ui/scripts", ["*.py", "*.js"])
        + package_files("web_ui/static", ["*.html", "*.css", "*.js"])
        + package_files("web_ui/styles", ["*.css"])
        + package_files("web_ui/assets", ["*"])
        + package_files("web_ui/docs", ["*.md"])
        + package_files("web_ui/tools", ["*.py", "*.sh", "*.md", "*.html"])
    ),
    install_requires=[
        "setuptools",
        "fastapi",
        "uvicorn",
    ],
    zip_safe=False,
    maintainer="mu",
    maintainer_email="mu@example.com",
    description="Python implementation of visual pose estimation for ROS2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "visual_pose_estimation_node = visual_pose_estimation_python.main:main",
            "visual_pose_estimation_web = visual_pose_estimation_python.web.server:main",
        ],
    },
)
