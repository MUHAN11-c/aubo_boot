# peach_harvester

v1 采摘大脑：场景感知产生观测，目标重建产生模型与 `GraspDecision`，调度选择目标并编排批次。默认 `peach_harvester` 可执行程序在同一进程承载三个节点；`peach_scene_obstacles` 是独立进程，按 Survey 结果生成场景障碍快照。

本包不直接写关节或工具 IO。跨包契约在 `peach_interfaces`；臂侧的运动和接触许可检查由 `peach_arm` 承担。生产仍使用 v1，v2 孵化栈在 `src/peach2/`。

## 入口与源码

- `peach_harvester.brain:main`：默认三节点入口。
- `vision/scene_perception/`、`vision/target_reconstruction/`：感知、重建及其纯计算核；各自也有独立 console script。
- `vision/scene_obstacles/`：场景快照节点及几何核。
- `supervisor/`：批次 FSM、选果、编排与账本；`peach_supervisor` 可独立启动。
- `config/`：本包节点部署 YAML、停位几何及工具相关配置；共享参数函数直接导入 `peach_common`。
- `model/`：随仓模型资产；`test/`：纯核、部署参数与集成约束测试。共享参数函数的通用测试在 `peach_common/test/`。

公有 ROS API 的名称、消息类型和 QoS 以 [接口清单](../peach_interfaces/config/interface_manifest.yaml) 为准；本包接口说明在 [doc/io.md](doc/io.md)。内部 Python 算法模块不承诺跨包稳定 API。

## 构建、运行与测试

在工作区根目录，按 [v1 测试指导](../peach_system_tests/doc/testing.md) 配置 Jazzy 与 `aubo_py3.12`（numpy 1.26.4），然后执行：

```bash
colcon build --packages-up-to peach_harvester
source install/setup.bash
colcon test --packages-select peach_common peach_harvester
colcon test-result --verbose
```

整栈入口是 `ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock camera_enabled:=false`。相机关闭时不验证真实感知。所有测试或程序结束后须停栈并检查残留；真机运动或 SetIO 须另有人授权。

## 文档与许可

架构和源码职责见 [doc/architecture.md](doc/architecture.md)，系统组合见 [peach_bringup/doc/architecture.md](../peach_bringup/doc/architecture.md)。这些 Markdown 随源码维护，无独立文档生成步骤。

包元数据声明 BSD-3-Clause，许可文本见 [工作区 LICENSE](../../../LICENSE)。模型资产的来源和约束见各资产附带说明。
