# 桃子视觉两包分层架构设计（参数层 / 抽象接口层 / 数据层 / 编排主节点）

> 2026-08-10（v2：经 GitHub 成熟项目源码实证修正）。范围：`peach_pose_ros2`（单帧感知）、
> `peach_reconstruction_ros2`（连续 TSDF 重建）。原独立 fusion 方案已撤销，
> 质量门和 refined 输出收敛到重建包，避免重复状态机和话题转发。
> 目标：把"事实分层"正式化为"契约分层"——层间依赖单向、每层可独立测试、实现可替换。

## 1. 参考范式（GitHub 源码实证，2026-08-10 调研）

| 项目 | 实证事实 | 采用的启示 |
|---|---|---|
| **py_trees / py_trees_ros** | py_trees 的 package.xml 零 ROS 依赖（本质是 pip 包，ament 构建只是 shim）；py_trees_ros 壳继承核心类、`setup(node)` 注入 ROS（"delay ROS artifacts"注释）；msg⇄纯类型转换独立成 `conversions.py` | **纯核零 ROS import** + 壳可厚但厚的只能是通信胶水 + 构造期不碰 ROS 资源 |
| **nav2** | nav2_core 是无 src 的纯接口包：每类插件一个 ABC 头 + 独立 exceptions 头；`ParameterHandler<ParamsT>` 集中装载参数成结构体（validate/update 双回调）；Costmap2DROS 是独立数据层库（显式 start/stop/pause/resume）；controller_server 编排层 main.cpp 915B、成员零算法 | 接口用 **ABC**（不用 Protocol）；参数集中装载成 dataclass；数据持有者专职化、显式生命周期；编排层成员零算法 |
| **yolo_ros**（Python 感知包社区形态） | msgs / ros / bringup 三包；`type_to_model = {"YOLO": YOLO, ...}` 字符串→类显式注册表选实现；GPU 资源挂生命周期回调 | Python 侧实现发现用**显式注册表字典**即可，不引 pluginlib/entry_points |
| **moveit_configs_utils** | `@dataclass(slots=True) MoveItConfigs` 集中装载 YAML 成结构体 | YAML 权威哲学下唯一成熟先例：builder 装载成 dataclass（且必须配双向同步测试） |
| **demo_nodes_py（官方）** | declare 即镜像实例属性 + pre/on/post 三段参数回调 | 动态改参规范（本阶段参数为启动期静态，只用 declare+load） |
| **image_pipeline** | 组件即节点，懒订阅（有订阅者才建上游订阅） | 可选加分项：无人订阅 debug_image 时跳过绘制（后续优化，不本期） |

反面教材（实证）：yolo_ros 的 detect_3d_node.py 48.7KB 巨文件、ultralytics_ros
构造即加载模型 + 每帧 get_parameter、parse_* 转换挂节点类（不可离线测试）。

## 2. 四层定义（修正版）

### 2.1 参数层（每包 `params.py`）

- 形态（nav2 ParameterHandler 的 Python 版）：**frozen dataclass** + `declare(node)`（集中
  declare 全部参数与中文 descriptor）+ `from_node(node)`（集中读取/解析/校验 → dataclass）。
- 权威源哲学：**维持本仓约定**（AGENTS.md §7：`config/*.yaml` 为权威默认值源，代码
  declare 逐一对齐）。实证显示 6/7 项目用代码权威，但本仓已选定 YAML 权威 +
  moveit_configs 先例要求配**双向同步单元测试**——本期补上该测试（yaml 默认值 ==
  declare 默认值逐项比对）。
- 参数为启动期静态装载（不做动态改参回调；如需后续加 on_set 校验）。
- 禁止：`__init__` 堆十几个 declare（yolo_ros 式下限）、每帧 get_parameter。

### 2.2 抽象接口层（每包纯核子包内 `interfaces.py`）

- 用 **abc.ABC**（实证：nav2_core/py_trees 全用 ABC，不用 Protocol）。
- 签名模式：一个 workhorse 纯数据方法（numpy/dataclass 进 → dataclass 出，无副作用，
  不碰 ROS）；重资源装载（模型加载）显式 `load()/close()` 钩子（对标 nav2 生命周期
  四件套的轻量版，现有懒加载语义不变）。
- 实现发现：显式注册表字典（yolo_ros 先例），如
  `POSE_ESTIMATORS = {'bag': RobustBagPosePipeline, 'fruit': RobustFruitPosePipeline}`。
- **peach_pose_ros2**：`Detector`（detect(rgb)→list[det]）、`Segmenter`
  （segment(rgb,bbox)→mask|None）、`PoseEstimator`（estimate(obs,…)→CandidateResult）。
  InferenceEngine 同时满足 Detector+Segmenter（保持一类）。
- **peach_reconstruction_ros2**：`FrameStore`、`CloudBuilder`、`VolumeFusion`
  （对齐 scene_recon FusionBackend 形状）、`GeometryRefiner`。

### 2.3 数据层（拆两类，立规为主）

- **工作数据持有者**（对标 Costmap2DROS/Blackboard）：专职类、显式生命周期、
  由编排节点持有并注入算法——perception 的 `TargetRegistry`（目标级）、
  reconstruction 的 `FrameCollector`（批级帧栈）已是此形态，保持。
- **持久化**：`session_io.py`（重建 session 落盘）、`hand_eye/` 外参、
  `validation_runs/`——纯函数模块，保持。
- 硬规则（写进 README/AGENTS 固化）：① 数据对象零 ROS import；② 生命周期分级
  （帧级/目标级/批级/会话级）；③ ROS 消息只在编排边界出现。

### 2.4 编排主节点

- 只做：declare/load 参数（一行）→ 建数据持有者 → 按注册表实例化算法 →
  接线（订阅/服务/发布/timer）→ 回调 = conversions 转换 → 调 workhorse → 发布。
- msg⇄纯类型只在 `conversions.py`（py_trees_ros 先例，感知已有；重建侧保持现状）。
- 量化参照：nav2 main.cpp 915B、ControllerServer 成员零算法；我们的节点可以厚，
  但厚的只能是 ROS 胶水（QoS/闩锁/节流日志），不是业务逻辑。

## 3. 层间依赖规则（单向）

```
params.py ──► 编排主节点 ──► 纯核子包（interfaces.py ABC + 实现 + 数据持有者）
                 │                 ▲
                 └── conversions.py（msg⇄纯类型，只被编排层用）
```
纯核子包零 ROS import（import guard 单测强制）；conversions 不反向依赖节点。

## 4. 落地改动清单（小步、零行为变化）

| # | 改动 | 包 | 性质 |
|---|---|---|---|
| 1 | 新建 `params.py`：frozen dataclass + declare(node) + from_node(node)；节点一行装载 | 两包 | 搬迁 |
| 2 | 新建纯核 `interfaces.py`：ABC + 注册表；现有实现签名对齐（不齐微调） | 两包 | 新增契约 |
| 3 | 新测试：yaml↔declare 双向同步、参数装载（默认/非法值）、纯核 import guard（零 rclpy） | 两包 | 新增测试 |
| 4 | README 软件框架节改四层图 + 层间规则 | 两包 | 文档 |
| 5 | AGENTS.md 第 8 节补分层约定（父代理执行） | 仓库 | 文档 |
| 6 | 连续 TSDF 的 ICP/质量门保持纯核，节点只编排 | 重建包 | 已实施 |

明确不做：不改消息定义、不动算法实现、不改 topic/service 面、不引新依赖、
不做 lifecycle/动态改参/懒订阅（后续可选）。

## 5. 验收

- colcon test lint 全绿；venv pytest 全过（含新增 params/interfaces/import-guard 用例）
- 回放冒烟行为与改动前一致
- import 图无环；纯核子包零 ROS import（测试强制）
