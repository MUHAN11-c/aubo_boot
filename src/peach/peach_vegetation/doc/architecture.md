# peach_vegetation 架构

## 职责

独立 GPU 枝叶二维分割；不写 PlanningScene，不发运动，不进入 harvest_system。

## 源码地图

`peach_vegetation/`：节点、参数、Frangi/ExG 分割核；`config/`：部署 YAML；`launch/vegetation.launch.py`：输入 remap；`test/`：规则与纯核测试。

## 入口、输入输出与参数

`ros2 launch peach_vegetation vegetation.launch.py`；输入相对名 image（默认 remap 彩色 raw）；输出 leaf_mask/branch_mask/overlay/status 及 diagnostics，详细类型见 [io.md](io.md)。参数经 peach_common.yaml_params.attach，规则在 params.py。

## 生命周期、并发与失败

Lifecycle 节点由 main 自激活，不加入系统管理器；互斥推理回调组配合双线程 executor，忙时丢帧并计数。device=auto 可回退 CPU，显式 CUDA 无设备则 configure 失败。相机重启后订阅可能停滞，诊断 STALE 时重启本节点；不通过另建同名节点修复。

## 验证与限制

`colcon test --packages-select peach_vegetation --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
