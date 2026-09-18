# peach_bringup

整栈组合入口。Include 只读 `aubo_e5_bringup`，再拉大脑（`peach_harvester` brain：感知+重建+调度一进程三节点）/ 技能 / 观测。**不自动 RunHarvest。**

## 公有入口

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

bag 回放另加 `use_sim_time:=true`，并先 `ros2 bag play --clock`。

`peach_harvester/launch/harvest_system.launch.py` 薄转发到本包。预检 `peach_bringup.preflight.running_stack_pids`（argv0 或 `.../lib/<pkg>/<node>`，不是 colcon 参数名）。参数走本包 `yaml_params.py`（与 harvester / vegetation 同文副本，不合并）。

## 构建 / 测试 / 许可

随工作区 `colcon build --packages-select peach_bringup`。零 ROS 预检：`peach_system_tests/test/test_preflight.py`。BSD-3-Clause。
