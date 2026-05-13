"""graspnet-baseline 轻量 pip 包 — 使 models/utils/dataset 可通过标准 import 使用。

IVG2.0 专用：pointnet2 / knn / graspnetAPI 各自独立 pip install -e .，此处排除以避免冲突。
"""
from setuptools import setup, find_packages
import os
import sys

# 确保 models/ 在 sys.path 中以兼容上游代码的绝对导入 (from backbone import ...)
# 上游 models/*.py 使用 "from backbone import" 而非 "from .backbone import"
_here = os.path.dirname(os.path.abspath(__file__))
_models_dir = os.path.join(_here, 'models')
if _models_dir not in sys.path:
    sys.path.insert(0, _models_dir)

setup(
    name='graspnet-baseline',
    version='0.0.0',
    packages=find_packages(
        exclude=['pointnet2', 'pointnet2.*', 'knn', 'knn.*',
                 'graspnetAPI', 'graspnetAPI.*',
                 'build', 'dist', '*.egg-info', 'logs', 'doc'],
    ),
    install_requires=[
        'numpy',
        'torch',
        'open3d',
        'scipy',
        'Pillow',
    ],
    description='GraspNet-1Billion baseline models for IVG2.0',
)
