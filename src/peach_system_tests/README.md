# peach_system_tests

隔离域 `launch_testing` 与 mock 故障矩阵。不含真机、不自动 `RunHarvest`。

## 测什么

- `test_preflight.py`：启动预检零 ROS（argv basename，不误伤编辑器路径）
- `test_mock_launch.py`：isolated launch_testing 起 mock `harvest_system`；使能默认关、无自动 goal；`/joint_states` 关节序；lifecycle Active

## 构建 / 测试

```bash
colcon test --packages-select peach_system_tests
```

`colcon test` 绿 ≠ 套袋验收。BSD-3-Clause。
