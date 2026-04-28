import os
from glob import glob
from setuptools import find_packages, setup

PKG = "coffee_latte_demo"

setup(
    name=PKG,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.launch.py")),
        (f"share/{PKG}/config", glob("config/*.yaml")),
        (f"share/{PKG}/meshes/visual", glob("meshes/visual/*.stl")),
        (f"share/{PKG}/meshes/collision", glob("meshes/collision/*.stl")),
        (f"share/{PKG}/web/public", glob("web/public/*.html")),
        (f"share/{PKG}/web/public/css", glob("web/public/css/*.css")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Coffee Latte Demo Package",
    license="BSD",
    entry_points={
        "console_scripts": [
            "latte_node = coffee_latte_demo.latte_node:main",
        ],
    },
)
