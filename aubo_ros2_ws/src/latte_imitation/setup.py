import os
from glob import glob
from setuptools import find_packages, setup

package_name = "latte_imitation"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/resource/original",
         glob("resource/original/*.parquet")),
        *[
            (f"share/{package_name}/resource/cartesian/{arm}",
             glob(f"resource/cartesian/{arm}/*.npz"))
            for arm in ["left", "right"]
        ],
        (f"share/{package_name}/resource/heart",
         glob("resource/heart/*.npz")),
        (f"share/{package_name}/resource/heart",
         glob("resource/heart/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Latte Art Imitation — RM65 to AUBO E5 trajectory retargeting (SPOT + Isaac Teleop + MoveIt2)",
    license="BSD",
    entry_points={
        "console_scripts": [
            "latte_imitation_node = latte_imitation.trajectory_pipeline:main",
        ],
    },
)
