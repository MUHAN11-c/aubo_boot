"""pytest 收集控制：open3d 缺失时跳过融合后端测试."""

import importlib.util

# colcon test 走系统 python3（无 open3d）；完整套件按 AGENTS.md 第 6 节经
# 工作区 venv 运行，open3d 存在时照常全量执行。
collect_ignore = []
if importlib.util.find_spec('open3d') is None:
    collect_ignore.append('test_pc_utils.py')
