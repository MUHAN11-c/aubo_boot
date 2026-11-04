# 参考来源

本页记录本轮重构/测试实际参考的官方规范、源码和项目内记录。只列与本项目决策/验证有关的条目；未落地的方向标为“待验证”，不当成已实现。

## 官方 ROS 2 / Nav2 / MoveIt

| 主题 | 参考 | 用于 |
|---|---|---|
| 生命周期节点 | ROS 2 design: <https://design.ros2.org/articles/node_lifecycle.html> | LifecycleNode、configure/activate/cleanup、托管栈 |
| Executor 并发 | ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html> | scene 单线程 executor + worker；reconstruction/supervisor 多线程 |
| QoS | ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html> | 传感器/状态/事件 QoS，depth/durability 选择 |
| launch / launch_ros | ROS 2 Jazzy Launch: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html> | 参数文件、overlay、include、condition |
| Nav2 lifecycle manager | Nav2: <https://docs.nav2.org/configuration/packages/configuring-lifecycle-manager.html> | 托管名单、STARTUP/PAUSE/RESUME、is_active |
| Bond | ROS 2 design: <https://design.ros2.org/articles/bond.html> | `bond_timeout` 语义；当前默认 0 关闭 |
| generate_parameter_library | <https://github.com/PickNikRobotics/generate_parameter_library> / <https://docs.ros.org/en/jazzy/p/generate_parameter_library/> | GPL schema、ParamListener、参数描述/约束 |
| GPL Python 生成器（本机） | `/opt/ros/jazzy/lib/python3.12/site-packages/generate_parameter_library_py/`；Debian `ros-jazzy-generate-parameter-library-py 0.7.6-1noble.20260810.122749` | 生成结构/ParamListener、整数 IntegerRange 缺陷复现；这是本机实际版本 |
| diagnostic_updater | <https://docs.ros.org/en/jazzy/p/diagnostic_updater/> | `/diagnostics`、节点健康聚合 |
| rosbag2 录制 | <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html> | 会话 bag 语义、`ros2 bag info` |
| rosbag2 MCAP | <https://docs.ros.org/en/jazzy/p/rosbag2_storage_mcap/> | 单一 MCAP writer |
| RViz2 用户指南 | <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html> | 显示配置、Fixed Frame、Image/PointCloud 负载 |
| MoveIt 2 RViz 可视化 | <https://moveit.picknik.ai/main/doc/how_to_guides/visualizing/visualizing_in_rviz.html> | RobotModel、MotionPlanning 面板与显示插件 |
| FastAPI | <https://fastapi.tiangolo.com/> | 同源 API、Pydantic/线程池 |
| FastAPI concurrency | <https://fastapi.tiangolo.com/async/> / `fastapi.concurrency.run_in_threadpool` | 长调试动作不阻塞事件循环 |
| Uvicorn | <https://www.uvicorn.org/> | 后台 HTTP 服务、socket 绑定 |
| Starlette concurrency | <https://www.starlette.io/concurrency/> | sync backend 调用线程池语义 |

## 项目内规范与代码

| 参考 | 用途 |
|---|---|
| `AGENTS.md` / `CLAUDE.md` | MUST/DEFAULT/KEEP/UNWIND、测试边界、禁止真机未授权运动/SetIO |
| `src/peach/peach_system_tests/doc/testing.md` | 既有短测/A-B/4-slot/10×30×1800 记录与验收口径 |
| `src/peach/peach_bringup/doc/architecture.md`、`io.md`、`design.md` | 系统进程、数据流、FSM、动作/服务/话题、RViz 应用视图 |
| `src/peach/peach_arm/doc/architecture.md`、`io.md` | 接触/套入/工具、ExecutionAuthority、SetIO、completion level |
| `src/peach/peach_observability/doc/architecture.md`、`io.md` | HTTP、bag、自检、参数面板、Web 行为 |
| `src/peach/peach_interfaces/config/interface_manifest.yaml` | 图名/QoS/生产消费方唯一清单 |
| `src/peach/peach_interfaces/msg/FailureCode.msg` | 跨包失败码 |
| `src/serial_force/doc/geometry.md` | 五点膜片坐标、tcp z=−72.29 mm、CoP 公式 |
| `src/serial_force/scripts/force_web.html`（git 历史） | 旧力页 CAD 底图/五点点位/镜像视图设计 |
| `src/serial_force/web/face_fruit.png`、`face_full.png` | 力页 CAD 果实侧/整机底图 |
| `src/serial_force/doc/force_pads_on_tool.png` | 四点 CAD 位置联图 |
| `src/peach/peach_bringup/config/harvest.rviz` | 原应用 RViz 显示集合 |
| `src/peach/peach_bringup/config/harvest_app.rviz` | 去 MotionPlanning 崩溃面板后的应用视图 |
| `aubo_e5_moveit_config`（工作区） | MoveIt robot_description/controllers/mock 配置 |
| `docs/architecture/00-12`、`docs/adr/0001-0006` | 本轮架构决策与验证矩阵 |

## 历史测试记录（文档中可查）

| 记录 | 位置 | 结果口径 |
|---|---|---|
| 2026-10-16 5 轮 | `testing.md` 记录（该 runs 目录当前工作区已不在） | 5/5 L6，短测 |
| 2026-10-21 3 轮 | `runs/live_mock_unified_rviz_20261021_1813/report.json` | 3/3 L6，RViz 托管 |
| A/B 全重建 / skip | `runs/AB_A_full_20261021_1829` / `runs/AB_B_skip_20261021_1832` | A 33.3%（2/6），B 100%（8/8），均 L6 |
| 4-slot ×2 | `runs/goal_skip_4slot_20261022_0908` / `..._b_20261022_0915` | 8/8 L6；第二次 slot4 camera false |
| 10×30×1800 | `testing.md` 记录 | **未通过**：6/10 L6，4 质量跳过，soak 约21min，RViz 退出 |
| Web/RViz 单轮 | `runs/web_rviz_102246/ledger.json` + `runs/peach_refactor_20261027/web_review/rviz_complete.png` | L6，33.456 s，tool=false |
| 自动 runner 回归 | `runs/peach_refactor_20261027/full_regression2` | target_1/target_2 记录不一致，判失败 |

## GPL 版本与“官方/主流”判定

- **官方发行版中的包**：本机 `generate_parameter_library` / `_py` 为 ROS Jazzy apt 包，版本 **0.7.6**（`ros-jazzy-generate-parameter-library 0.7.6-1noble.20260912.163311`，Python 包 0.7.6-1noble.20260810.122749）。
- **不是 ROS REP/核心标准**：它是 PickNik 维护的 ROS 2 生态参数生成库，Nav2 等常用；ROS 2 官方参数 API 仍是 `rclpy`/`rclcpp` 的 `declare_parameter` + `ParameterDescriptor`。
- **不是上游最新**：截至本页记录，上游 `main` 的 `package.xml` 为 **1.3.1**（<https://raw.githubusercontent.com/PickNikRobotics/generate_parameter_library/main/generate_parameter_library/package.xml>）。本机 0.7.6 是 Jazzy 发行版快照，不能用“本机版本=最新”表述。
- **Python 生成器成熟度低于 C++**：本机 0.7.6 生成整数 `IntegerRange.from_value = 1.0`，`ros2 param describe` 触发 `rcl_interfaces` 的 `PyLong_Check` 断言；项目已用 Python 回调规避。升级 GPL 后必须重新验证该缺陷是否仍存在，不能把 workaround 当成永久标准。
- 结论：**主流 ROS 2 生态工具 = 是；ROS 官方发行包 = 是（Jazzy 0.7.6）；ROS 核心/REP 标准 = 不是；上游最新 = 不是。**

## 未作为已实现参考

- TensorRT/Orin 实测：仅有 x86 RTX 3090 数据；Orin 目标机未接入。
- rosbridge/foxglove：明确不作为控制面参考。
- `src/peach_sim`：环境仿真，不修改、不纳入本重构验收。

## 引用原则

- 官方规范用于解释“为什么这样设计”，不是复制代码。
- 项目内活文档/清单优先于历史说明；冲突时以源码和 manifest 为准。
- 历史短测不会替代 10×30×1800；L6 不会写成 L7。
