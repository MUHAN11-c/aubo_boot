# sim_perception

仿真的视觉参数覆盖与兼容入口。运行时直接复用现行的
`peach_scene_perception` 和 `peach_target_reconstruction`，不再安装或运行
本包历史 `sim_perception/vision/` 业务副本。目录外能力包、模型和配置只读引用。

- `sim_scene_perception_node` → `peach_scene_perception.scene_perception_node.main`
- `sim_target_reconstruction_node` →
  `peach_target_reconstruction.target_reconstruction_node.main`

两个节点保持 `peach_scene_perception_node`、`peach_target_reconstruction_node`
名称和现行相机/感知接口。入口只透传 `args`，生命周期、线程、诊断、TF 与取消
实现由共享能力包维护。Python 解释器由构建环境确定；构建及启动使用工作区既有
`aubo_py3.12`，保持 `numpy==1.26.4`，本包不安装第三方库。

## 配置装配

`sim_bringup/launch/perception.launch.py` 按以下顺序装配：

1. 对应共享能力包 `config/scene_perception.yaml` 或
   `config/target_reconstruction.yaml` 的完整配置。
2. `peach_common.grasp_standoffs` 与 `peach_common.tool_profiles` 的工具覆盖。
3. 本包同名 YAML 的仿真覆盖，再注入本次运行目录与统一仿真时钟。

本包 YAML 是 **overlay**，不能脱离共享完整配置当作另一份完整 schema。
`ParameterFile(..., allow_substs=True)` 负责展开包路径和环境引用。
权重来自 `peach_scene_perception/model/`，不再引用已经变为元包的
`peach_harvester/model/`。目标重建的掩膜时间容差遵守现行 schema 的 0.5 s
上限，当前仿真覆盖为 0.45 s；不得照搬历史 3 s 设置。

重建 `session.root_dir` 必须是 `peach_sim` 内的绝对运行目录。
本包覆盖要求项目运行器提供 `PEACH_SIM_RUN_DIRECTORY`，launch 最终还应覆盖为
本次 `run_directory/vision`；缺环境时拒绝展开，不回退目录外的工作区 `runs/`。

## 模式与验收

联合入口的 `perception:=vision` 使用真实 RGB-D 推理链；
`ground_truth` 模式使用明确标识的仿真目标，不启动视觉能力。
导航/抓放真值模式通过不能证明视觉精度。运行方式、独立 ROS domain、Gazebo
partition 与收尾由 `sim_bringup` 项目运行器统一管理。

官方检查：`colcon test --packages-select sim_perception`。
flake8/pep257 只检查本包实际安装的薄适配、打包与检查入口；历史 `vision/`
保留供追溯，不随包安装，不再作为本包运行接口或当前验收对象。
包级设计与接口见 [architecture](doc/architecture.md)、[io](doc/io.md)。

BSD-3-Clause；模型与共享能力各自遵守原有许可。
