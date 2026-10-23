# peach_system_tests 架构

## 职责

手工复核包：冻结相机回放、有限时探针和场景查询。自动检查只剩官方 ament lint。

## 源码地图

`CMakeLists.txt`：官方 `ament_lint_auto`。`scripts/`：有限时驱动、探针、场景查询和冻结回放。`launch/`、`config/`：回放组合与专用 overlay。

## 入口、输入输出与参数

不进入产品 launch。安装脚本用 `ros2 run peach_system_tests`，参数先看 `--help`。`run_live_camera_mock.py` 默认托管RViz，使用bringup同一应用显示配置；`--no-rviz`用于无头排障。业务配置来自产品默认或 `--task-params-file`，与real同源，不生成高速/放宽超时或跳重建专用overlay，不自动ACK。任何模型/抓取失败或恢复等待停止后续；PICK_ALL按全部目标终局验收。模型、动作、soak与清理分别记录实际结果，历史短测数据见testing.md，不视为当前统一参数通过。

## 生命周期、并发与失败

手工起栈时自己选一个空闲 `ROS_DOMAIN_ID`。Jazzy controller_manager spawner 的锁位于 `$ROS_HOME/locks`，不随 DDS 域隔离；不要删其他会话的锁。runner 拥有的进程必须收口，仍须核对残留。runner 非零、skip 及 action 结果未知要分别看日志；不能把进程退出 0 直接记为采摘成功。

## 验证与限制

`colcon test --packages-select peach_system_tests` 只跑官方 lint。接线与现场验收见 [测试指导](testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。跑完清本轮进程；真机运动或 SetIO 需另授权，驱动只读，numpy 固定 1.26.4。lint 通过不证明现场精度、功能安全或剪切成功。
