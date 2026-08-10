"""pytest 收集控制：open3d 缺失时跳过依赖 open3d 的测试文件."""

import importlib.util

# colcon test 走系统 python3（无 open3d）；完整套件按 AGENTS.md 第 6 节经
# 工作区 venv 运行，open3d 存在时照常全量执行。cloud_builder 反投影与
# geometry_refiner 法线已换 open3d 官方实现（懒加载），凡经 cloud_builder
# 建云的测试文件一并跳过。
collect_ignore = []
if importlib.util.find_spec('open3d') is None:
    collect_ignore.append('test_cloud_builder.py')
    collect_ignore.append('test_multiview_alignment.py')
    collect_ignore.append('test_tsdf_volume.py')
    collect_ignore.append('test_geometry_refiner.py')
    collect_ignore.append('test_icp_refiner.py')
