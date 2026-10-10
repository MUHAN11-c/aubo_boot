# `peach_sim` 架构（工作区级仿真，活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` / `docs/io.md` 的 sim 相关节拆写入包。独立栈：不进 `harvest_system` / lifecycle / 运行 launch；系统级架构在 [src/peach/peach_bringup/doc/architecture.md](../peach/peach_bringup/doc/architecture.md)。

> 2026-09-30 套袋专项 v10：纠正 v9 厚盒底和撑满侧壁，改为薄封边、内果局部鼓起和下部余纸，独立三袋模型含原尺寸内果/果梗/扎丝；新增正侧面检查。仅套袋资产更新，既有整园/field/五光照仍属 v9 历史结果，不是 v10 验收。命令与证据见 [peach_sim README](../README.md)。

> 2026-09-24 果园外观重建：当前 Blender 入口为 `src/peach/peach_sim/reconstruction/`，不复用旧模型/贴图/随机布局。RGB-D 约束局部可见袋面；整树、遮挡部分与树行是标注清楚的推断。旧 Gazebo SDF/GT 仍属旧管线，尚未接入此次新场景；无 ROS IDL、运动或驱动变更。数据口径、离线命令与验收边界见 [peach_sim README](../README.md)。根目录 `blender_orchard/` 观感预览链已于 2026-09-28 并入 `src/peach/peach_sim/reconstruction/`（袋面折痕、程序贴图、PeachDataSet 先验统计迁入；数据口径以 priors 分位为准），果园外观建模收敛为单管线；光照/叶遮挡/枝干扰为受控变量，多视角感知验证矩阵与基线回归门见 README。

## 包职责与文件树

peach_sim/                 # Gazebo Harmonic 果园仿真（场景+巡园+离线 Blender 线；独立栈，不进 harvest_system/lifecycle）
  peach_sim/{scene,cli,preview,params,textures,nav_map,tour_core,tour_node}.py   # scene=确定性果园纯核；nav_map/tour_core=地图/巡园纯核，tour_node=NavigateToPose 节点
  launch/{orchard_sim,orchard_nav}.launch.py   # 仿真+工位（mobile:=true 解锚可驾驶）/ Nav2 巡园（AMCL+确定性图或 slam:=True）
  config/{orchard,orchard_bridge,nav2_params}.yaml   # 场景 schema / gz↔ROS 六路桥 / Nav2 全参
  maps/orchard_map.{pgm,yaml}   # 确定性栅格图（生成器产物入库）
  blender/{build_assets,export_field_to_gz}.py   # 离线 Blender 4.5 无头导出（Blender 自带 Python，勿进 aubo_py3.12/numpy 环境）：工位 GLB 网格 / 重建去草整园→查看 GLB（决策 0039）
  reconstruction/               # Blender 外观重建 + 感知验证矩阵管线（离线门；见 v1 测试指导「peach_sim 仿真」A 与包 README）
  worlds/urdf/meshes/textures/  # 两份 SDF 世界（peach_orchard=程序化锚定 GT；field_no_grass=重建桥接纯视觉查看，无桥无 TF）/ 履带+臂 xacro（mobile 化）/ GLB 网格 / PBR 贴图
  test/{test_params,test_scene,test_textures,test_nav_config,test_nav_map,test_tour_core,test_urdf_mobile}.py

## 物理仿真现状（原 architecture §8 行）

| 关节名顺序 | URDF / `controllers.yaml` / 透传 goal 按 MUST 六关节序；`/joint_states` 由 `joint_state_broadcaster` 发布，name 数组常见字母序（`foreArm` 在 `shoulder` 前），消费者必须按名字对齐，禁止按下标当 MUST 序 | 透传点按下标拧腕；launch_testing 只断言六名存在 |
