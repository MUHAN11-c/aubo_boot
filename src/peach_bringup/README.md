# peach_bringup

整栈组合入口。Include 只读 `aubo_e5_bringup`，再拉感知 / 技能 / 调度 / 观测。**不自动 RunHarvest。**

## 公有入口

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

`peach_executor/harvest_system.launch.py` 薄转发到本包。预检 `peach_bringup.preflight.running_stack_pids`（argv0 或 `.../lib/<pkg>/<node>`，不是 colcon 参数名）。

## 构建 / 测试 / 许可

随工作区 `colcon build --packages-select peach_bringup`。零 ROS 预检：`peach_system_tests/test/test_preflight.py`。BSD-3-Clause。
