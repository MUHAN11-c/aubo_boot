# peach_pose_ros2 — PeachPose 桃姿 ROS 2 感知包

## 简介

订阅 Percipio RGB-D，运行 YOLO + MobileSAM + 实测深度几何管线，发布抓取参考候选
与 RViz Marker。**只发参考位姿，不发送运动指令。**

## 当前开发范围：套袋桃

当前项目的开发、参数整定和真机验收均以**套袋桃**为主线。在线链路重点使用
`peach_bag` 类别、袋体掩膜、袋底—袋颈方向以及面向下游重建的稳定目标身份。
`peach_nobag` 裸桃分支仅作为研究与接口兼容能力保留，尚未纳入当前真机抓取
验收；其参考位姿不得绕过重建质量门，或直接作为现阶段工具接触的放行依据。

从零逐行读懂本包（启动链 → 参数 → 节点 → 管线 → 输出话题）见
**[TUTORIAL.md](TUTORIAL.md)**（零基础教程）。

本包与连续局部重建的职责边界、接口映射、目标切换、真机全流程和验收方法见
**[桃子首帧感知与连续局部重建联动说明](../../docs/peach_pose_reconstruction_integration.md)**。

## 使用方法

### 依赖（`aubo_py3.12`，**GPU**）

必须保持 `numpy==1.26.4`（与系统 `cv2` / `cv_bridge` 兼容），禁止抬升到
numpy 2.x。YOLO / MobileSAM 走 **CUDA**（本机 RTX 3090，`torch 2.13.0+cu130`）。

依赖全部锁定在仓库根 `requirements.txt`（torch / torchvision / ultralytics /
ultralytics-thop 等）：

```bash
# 从仓库根目录安装
aubo_py3.12/bin/pip install -r requirements.txt

aubo_py3.12/bin/python -c "
import numpy, torch, ultralytics
from cv_bridge import CvBridge
assert numpy.__version__.startswith('1.26')
assert torch.cuda.is_available(), torch.__version__
print(numpy.__version__, torch.__version__, torch.cuda.get_device_name(0))
"
```

注意：ultralytics 会把 `opencv-python` 作为依赖带进 venv（本机已验证与
numpy 1.26.4 / cv_bridge 共存）；**不要手动安装/升级 opencv-python 或
scipy**（会 shadow 系统版）。若 `import cv2` / `cv_bridge` 段错误，按
`requirements.txt` 头注释卸载 venv 内 opencv-python，回退 apt 的
python3-opencv。

图像编解码统一走 **cv_bridge**（`bgr8` / `passthrough` uint16 mm / `mono8`）。

重力参数：`gravity_hint_xyz` 为逗号分隔 `"x,y,z"`；空串表示 `None`（算法默认相机系 +Y）。
`gravity_mode` 选重力来源：`fixed`（默认，仅用 `gravity_hint_xyz`）或 `tf`（由本帧
TF 旋转反推相机系重力，`output_frame` 系重力约定 `[0,0,-1]`，只乘旋转不加平移；
TF 不可用的帧回退 `gravity_hint_xyz`）。

### 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_pose_msgs peach_pose_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 运行

#### 无相机（数据集回放冒烟）

数据集回放工具已独立为 `tools/peach_dataset_replayer.py`（不随 colcon 构建，
与包内模块无依赖）。用法：

```bash
# 终端 1：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py

# 终端 2：另开终端回放数据集
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <数据集根> [--limit N] [--loop] [--rate 0.5]
```

启动前请确认无残留：`pgrep -af 'peach_pose_ros2|peach_dataset_replayer'`（多实例同时加载 YOLO/SAM 可能导致段错误）。

`--dataset` 缺省为工作区 `src/peach_pose_ros2/data/dataset`（软链到 peach_canopy），
推断失败会报错。回放内参用本机 Percipio（脚本内置 `K_PERCIPIO`，与
`color_camera_info.yaml` 一致）；此时 `depth_scale_unit` 应为 1.0（数据集深度
是真毫米）——节点参数仅在启动时读取，请改 `config/peach_pose.yaml` 后再启动。

#### 真相机 RGB-D

```bash
# 终端 1：显式 RGB-D 封装（同时强制开启普通点云）
ros2 launch percipio_camera percipio_rgbd.launch.py

# 终端 2：感知节点
ros2 launch peach_pose_ros2 peach_pose.launch.py
```

也可直接与 bringup 并用。当前 `percipio_camera.launch.py` 默认已开启深度、
深度到 color 配准和彩色点云，因此 `camera_enabled:=true` 可直接满足本包输入。
`percipio_rgbd.launch.py` 额外强制开启普通点云，适合单独调试；若选择另起该
launch，bringup 侧必须给 `camera_enabled:=false`，避免重复打开同一设备：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
# 另开终端：ros2 launch percipio_camera percipio_rgbd.launch.py
```

### 参数

32 个参数全部带中文 `ParameterDescriptor`（`ros2 param describe /peach_pose_node
<参数>` 可查），权威值在 `config/peach_pose.yaml`（launch 全量加载）。常用：

| 参数 | 默认 | 说明 |
|---|---|---|
| `color_topic` / `depth_topic` / `camera_info_topic` | /camera/color/image_raw 等 | RGB-D 输入（深度须 registration 对齐；uint16 原始值或 32FC1 米制） |
| `camera_optical_frame` | camera_color_optical_frame | 手眼链所挂光学系；空串用深度图 header.frame_id |
| `output_frame` | base_link | 输出坐标系；空串保持相机系 |
| `tf_timeout_sec` | 0.5 | TF 查询超时 (s) |
| `depth_scale_unit` | 0.25 | uint16 原始深度：raw × 本值 = 毫米（Percipio 常见 0.25）；数据集回放设 1.0；32FC1 米制深度下不生效 |
| `sync_slop_s` | 0.05 | RGB-D 近似同步允差 (s)；时间戳偏差超 80% 允差时 WARN 节流提示 |
| `min_detection_conf` / `yolo_conf` | 0.3 / 0.3 | 入管线置信度下限 / YOLO 推理阈值 |
| `detection_dedup_ios` | 0.6 | 重叠检测框去重：IoS（交集/较小框面积）≥ 本值判同一目标，保留大框（跨类生效）；≥1.0 关闭 |
| `publish_debug_image` | false | debug 叠加图开关；阶段 H（2.13）起生产档默认关——关闭时零序列化零发布，现场调图显式置 true |
| `publish_masks` / `publish_detection_cloud` | true | 掩膜画布 / 检测框点云输出开关 |
| `pipeline.locked_only_segmentation` | true | 阶段 H（2.13-E1）：锁定后 SAM 只对锁定集目标的检测框推理（记忆锚点经 TF 反投影识别锁定框；锁定前/开关关/TF 不可用帧全量）；segment_ms 分项 EMA 随之回落 |
| `detection_cloud_stride` | 2 | 检测框点云降采样步长（>1 减轻 RViz 负载） |
| `yolo_model_path` / `sam_model_path` | "" | 空串 = 包内 model/best.pt、model/mobile_sam.pt |
| `model_version` / `calibration_version` | 见 yaml | 随结果发布的模型/内外参版本标识（可追溯） |
| `gravity_hint_xyz` | "" | 重力方向提示 "x,y,z"（相机系）；空串 = 算法默认 +Y |
| `gravity_mode` | fixed | 重力来源：`fixed`=仅用 gravity_hint_xyz；`tf`=由本帧 TF 旋转反推相机系重力 |
| `tool.*`（8 个） | 见 yaml | 刀具几何：内径/插入深/刀刃距/入口 standoff/余量/版本号 |
| `target_memory.*`（8 个） | 见下节 | 目标身份记忆：世界系匹配跨帧复用 target_id |
| `harvest.*`（4 个） | 见「全局目标计划」节 | 收齐式窗口锁定与多维优先级 |

### 目标身份记忆（跨帧稳定 target_id）

每帧检测原本只赋帧内序号 `target_{i}`（i 为帧内序号），同一物理桃子消失再
出现后 ID 全变。开启 `target_memory.enable`（默认开）后，节点在 TF 变换之后、
组消息之前把每个候选拿到**世界系**（`output_frame`，默认 `base_link`）做
最近邻匹配。空间锚点取**检测框前景点云的中位质心**（`points_centroid`，
比袋底/袋颈等端点抗掩膜抖动），缺失时依次回退袋底 → position → 入口点。
匹配规则：**同类（class_id 相同）且欧氏距离 ≤ `target_memory.match_radius_m`
的最近历史目标**命中则复用其 `target_id`，并把位置/轴/直径按
`target_memory.position_ema`（α）做 EMA 平滑（轴先做 ± 符号对齐再 EMA 归一化；
直径 0 视为无效观测不参与平滑）；未命中发新 ID（`target_0/target_1/…`
单调计数，不复用已消亡序号）。已确认目标不删除，表容量超
`target_memory.max_targets` 时按最久未见淘汰。抗扰三件套（均为
未命中才生效的保护，常态相邻目标不误并）：

- **恢复匹配①（同类放宽）**：正常匹配未命中时半径 × `recovery_scale`
  再匹配一次，吸收锚点跳变
- **恢复匹配②（跨类收紧）**：仍半径内但允许跨类，吸收 bag/nobag 翻类；
  命中只复用 ID、不改表项类别
- **确认机制**：新目标累计命中 ≥ `confirm_frames` 帧才转正长期记录；
  未确认目标连续超 `tentative_ttl_frames` 帧未再命中即清除（瞬时误检
  不留记录；按帧计、帧率以运行状态为准，低帧率/卡顿不误清确认进度）；
  匹配时已确认表项优先于更近的未确认表项

语义边界：

- 记忆只影响**身份与位置平滑**；三态（ACCEPT/REOBSERVE/REJECT）仍逐帧独立
  计算，`last_status` 仅记录不参与匹配
- 同帧去重：一帧内已命中的表项不再参与后续匹配，两个候选不会撞同一表项
- **`tf_unavailable` 帧**（TF 彻底失败、几何退回相机系）跳过注册：保留帧内
  序号 ID 并加 `target_untracked` 诊断标记，避免相机系坐标污染世界系表；
  `tf_stale` 帧（按时间戳查询失败回退最新 TF）仍正常匹配/注册
- 几何失败帧（无质心/袋底/位置/入口锚点）无法定位，保留帧内序号不参与匹配
- 匹配结果经 `target_new` / `target_matched` 诊断标记随 candidate/fitting
  发布；Marker 文字同步显示稳定 ID；注册表规模随启动日志与周期日志
  （10s 节流）报告

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_memory.enable` | true | 身份记忆开关；false 回到帧内序号 |
| `target_memory.match_radius_m` | 0.06 | 匹配半径 (m)：同类且距离 ≤ 本值的最近目标命中 |
| `target_memory.max_targets` | 50 | 目标表容量上限；超限按最久未见淘汰 |
| `target_memory.position_ema` | 0.3 | 命中后位置/轴/直径 EMA 系数 α(0,1]，越大越跟随新观测 |
| `target_memory.recovery_scale` | 3.5 | 恢复匹配半径倍率（同类，未命中才启用，21cm 覆盖跨视角锚点偏差）；1.0=关闭 |
| `target_memory.cross_class_recovery` | true | 恢复匹配允许跨类（半径不放大，命中不改表项类别） |
| `target_memory.confirm_frames` | 3 | 累计命中 ≥ 本值帧数才转正长期记录；1=立即确认 |
| `target_memory.tentative_ttl_frames` | 5 | 未确认目标存活时限（帧），连续超本帧数未再命中即清除；按帧计，帧率以运行状态为准 |

### 真机联调前置（本包不运动）

1. 手眼 `hand_eye/active.yaml` 已激活，重启 `extrinsics_publisher`
2. 内参一律本机 Percipio：订阅 `/camera/color/camera_info`（`color_camera_info.yaml` 棋盘标定）；改标定后同步 `peach_pose/offline/config.py` 的 `K_PERCIPIO` 与 YAML `calibration_version`
3. 深度 `depth_scale_unit:=0.25`（与点云 Z 一致）；registration 打开；本包不发运动

### 单测

```bash
cd src/peach_pose_ros2
PYTHONPATH=peach_pose_ros2:$PYTHONPATH \
  /home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python -m pytest test/ -q
```

98 例业务测试（候选/拟合/袋果双管线/球精化/校验/深度归一化/TF 变换契约与锚点/
目标身份注册表（含恢复匹配与确认机制）/全局计划收齐式锁定与多维优先级/
检测框去重/参数层同步与装载/接口层契约/纯核 import guard）
+ flake8/pep257 lint。

### 全局目标计划与运行数据查询

节点对全局目标采用**收齐式窗口锁定**：`~/reset_global_targets`（或启动）后
进入收齐窗口，逐帧把已确认目标（含 REOBSERVE，不限于可选择目标）并入累积集
（同 ID 后者覆盖，锁定时取最新一帧质量量）。满足「累计 ≥
`harvest.min_collect_frames` 帧且连续 `harvest.lock_settle_frames` 帧无新增
确认 ID」即关闭窗口；超过 `harvest.max_collect_s` 秒未静止也强制关闭兜底。
窗口关闭时对累积集一次性排序、按 `target_memory.max_targets` 截断并锁定——
**空集也锁定**（`target_set_locked=true`、`target_count=0`）。锁定后新出现的
ID 不再入集，靠近期间目标暂时遮挡或丢失不会改变选中 ID，只有显式调用重置
服务才开始下一轮收齐。优先级排序键：安全状态（ACCEPT 先于 REOBSERVE）→
相机距离（先近后远，先清外围减少碰枝/遮挡）→ base 系高度（先低后高，避免摘
高处时碰落低处果；高度取袋底 `bag_bottom` 的 base 系 Z，几何缺失按
质心→position→入口点回退）→ 置信度 → 稳定 ID（确定性 tie-break）。逐目标
消息 `/peach/perception/target_observations` 同时携带稳定 ID、优先级、三维
结果、2D 结果、拟合诊断和以 `depth.header.stamp` 标记的独立 mono8 掩膜。

| 参数 | 默认 | 说明 |
|---|---|---|
| `harvest.min_collect_frames` | 10 | 收齐窗口最少累积帧数：达到后才允许按静止条件关闭窗口锁定 |
| `harvest.lock_settle_frames` | 5 | 连续无新增确认 ID 的帧数，与最少帧数联合判定目标集合已稳定 |
| `harvest.max_collect_s` | 25.0 | 收齐窗口最长时长 (s)，超时强制关闭兜底（空集也锁定）；须 ≥ min_collect_frames/相机FPS+settle 余量 |
| `harvest.priority_prefer_lower_first` | true | 优先级启用高度键（先低后高）；false 则高度不参与排序 |

```bash
ros2 topic echo /peach/perception/harvest_state
ros2 service call /peach_pose_node/query_harvest_state std_srvs/srv/Trigger "{}"
# 外部抓取执行器确认成功后推进到固定优先级的下一目标
ros2 service call /peach_pose_node/complete_selected_target std_srvs/srv/Trigger "{}"
# 整轮结束或需要放弃计划时，才重新确定目标数量
ros2 service call /peach_pose_node/reset_global_targets std_srvs/srv/Trigger "{}"
```

每轮数据默认写入工作区 `harvest_runs/harvest_*/`，也可用环境变量
`AUBO_HARVEST_DATA_DIR` 改根目录：

```text
manifest.yaml                 # 固定目标清单、优先级、模型/标定版本
events.jsonl                  # 感知与重建按时间追加的完整事件链
latest_perception.json        # 感知最新状态
latest_reconstruction.json    # 重建最新状态
masks/<stamp_ns>_<id>.png     # 选中目标的逐帧掩膜
```

### 与 `peach_reconstruction_ros2` 联动

本包输出的是“发现与接近初值”，重建包输出的是“多视角精化结果”。联动时：

1. 本包通过 `/peach/perception/target_observations` 发布当前选中 ID、逐目标掩膜、
   初始三维候选、优先级、跟踪状态和质量标志。
2. 掩膜使用 `depth.header.stamp`，重建包只接受同时间戳的深度、掩膜和精确 TF。
3. 上层以 `/peach/perception/initial_pose` 规划低速接近，不将其当作最终抓取结果。
4. 重建 finalize 后，上层优先使用 `/peach/reconstruction/refined_pose`，并把
   `/peach/reconstruction/grasp_decision` 作为视觉许可门。
5. 实际抓取和撤离成功后，上层才调用 `complete_selected_target` 推进下一目标。

重建候选选择器对非空 `selected_target_id` 执行严格身份硬绑定：该 ID 暂时
不可用时等待或拒绝本帧，不会切换到其他更高质量目标。运行时仍应监测感知
`selected_target_id` 与重建 `diagnostics.target_id` 一致；不一致说明存在陈旧进程或
跨版本节点，应立即 reset 并重启相关节点。完整约束见上方跨包联动文档。

## 执行逻辑

### 一帧数据的完整管线（`_on_rgbd` 回调）

```text
RGB(bgr8) + 深度(16UC1) + CameraInfo   ApproximateTime 同步 (slop=sync_slop_s,
  QoS RELIABLE，与数据集回放/相机驱动对齐)
  → cv_bridge 转图；深度归一化为 uint16 毫米（uint16：raw × depth_scale_unit；
    32FC1 米制 ×1000，此时 depth_scale_unit 不生效）；尺寸/CameraInfo 一致性校验
  → YOLO 检测（yolo_conf），低于 min_detection_conf 的框丢弃
  → 逐目标 MobileSAM 分割 → hybrid_dilated 前景
      = SAM 掩膜 ∩ 膨胀后的实测深度连通域（SAM 缺失显式 REOBSERVE +
      mask_unavailable，禁止静默深度回退）
  → 按检测类别分流几何管线：
      class 0 袋桃 → RobustBagPosePipeline（圆柱 RANSAC 拟合袋轴）
      class 1 裸桃 → RobustFruitPosePipeline（球拟合 + 梗腔定轴）
  → 刀具几何门控（tool.*：内径/插入深/安全余量）→ 三态
      ACCEPT=0 / REOBSERVE=1 / REJECT=2
  → 几何经 TF 变到 output_frame（默认 base_link，依赖
      hand_eye_extrinsics_publisher；按 depth.header.stamp 查询失败回退最新 TF 并给本帧
      candidate/fitting 打 tf_stale；彻底失败退回相机系并告警 + 打
      tf_unavailable，不静默用错系）
  → 目标身份记忆（target_memory.*）：世界系最近邻匹配/注册，帧内序号
      target_{i} 改写为跨帧稳定 ID；tf_unavailable 帧跳过并打
      target_untracked（详见「目标身份记忆」小节）
  → HarvestPlan：收齐窗口逐帧累积确认目标，窗口关闭（帧数+静止判据，
      超时兜底）一次性锁定数量和固定优先级（空集也锁定）；
      生成 selected_target_id 与 PLANNED / WAITING_QUALITY / SELECTED / HARVESTED 状态
  → HarvestDataStore：写 latest_perception.json、events.jsonl 和选中目标逐帧掩膜
  → 发布候选 / 2D / 拟合诊断 / 检测 / 掩膜 / Marker / debug 图 / 检测框点云
      + target_observations / harvest_state
```

内参始终取本机 `/camera/color/camera_info`（不做 FOV 推导回退，避免与标定
不一致）；几何先在相机光心系求解，再按需变到输出系。

## 软件框架

### 四层架构（参数层 / 接口层 / 数据层 / 编排层）

按 `docs/superpowers/specs/2026-08-10-peach-layered-architecture.md` 正式化的
契约分层，层间依赖单向：

```text
params.py（参数层）──► peach_pose_node.py（编排层）──► peach_pose/（纯核：
                            │                    interfaces.py ABC + 实现 + 数据层）
                            └── conversions.py（msg⇄纯类型，只被编排层用）
```

| 层 | 模块 | 职责 |
|---|---|---|
| 参数层 | `params.py` | `PeachPoseParams` frozen dataclass：`declare(node)` 集中声明 41 参数 + `from_node(node)` 集中读取/解析/校验；YAML 为权威源（双向同步测试强制） |
| 接口层 | `peach_pose/interfaces.py` | abc.ABC：`Detector`/`Segmenter`/`PoseEstimator` + `POSE_ESTIMATORS` 注册表（pipeline 底部显式登记；candidates 类别路由走注册表） |
| 数据层 | `peach_pose/contracts.py`、`target_registry.py`、`harvest_plan.py`、`harvest_data.py` | 纯数据合约、稳定身份表、固定采摘计划和运行数据持有者；零 ROS import |
| 编排层 | `peach_pose_node.py` | 只做：参数一行装载 → 建数据持有者 → 按注册表实例化算法 → 接线（订阅/发布/TF）→ 回调编排 → 发布 |
| 编排层工具 | `tf_utils.py` / `conversions.py` / `visualization.py` / `cloud_utils.py` | TF 工具、msg⇄纯类型转换、RViz/debug 绘制、检测框点云（只被编排层用） |
| 离线工具 | `peach_pose/offline/` | 离线数据集评估子包（不参与在线管线）：`e2e_validate.py` CLI、`validation.py`、`config.py`、`sphere_ref.py` |

层间规则：纯核子包 `peach_pose/`（含 `offline/` 与 `interfaces.py`）**零 ROS
import**（`test_pure_core.py` AST 扫描强制）；`conversions.py` 不反向依赖
节点；实现发现走显式注册表字典，不引 pluginlib/entry_points；参数启动期
静态装载，禁止每帧 `get_parameter`。

### 节点与入口

单节点 `peach_pose_node`，标准 console_scripts 入口；setup.py 经
`options.build_scripts.executable` 把启动器 shebang 指到 venv 解释器
（构建期解析：工作区 `aubo_py3.12/bin/python` 存在则用之，否则回退构建
解释器；约定同 `AGENTS.md` 第 8 节）。
launch 无任何解释器参数，标准 `Node()` 启动。

### 话题

| 方向 | 话题 | 类型 |
|------|------|------|
| sub | `/camera/color/image_raw` | `sensor_msgs/Image` bgr8 |
| sub | `/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1 mm（须 registration） |
| sub | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| pub | `/peach/perception/initial_pose` | `peach_pose_msgs/BagGraspCandidateArray`（3D 抓取参考候选，主输出） |
| pub | `/peach/perception/axis` | `geometry_msgs/Vector3Stamped`（最优候选平移方向，ACCEPT 优先；无候选不发布） |
| pub | `/peach/perception/single_cloud` | `sensor_msgs/PointCloud2` xyz+rgb（检测框内深度反投影） |
| pub | `/peach/perception/detections` | `vision_msgs/Detection2DArray` |
| pub | `/peach/perception/masks` | `sensor_msgs/Image` mono8 |
| pub | `/peach/perception/diagnostics` | `peach_pose_msgs/BagFittingArray`（拟合诊断指标） |
| pub | `/peach/perception/markers` | `visualization_msgs/MarkerArray` |
| pub | `/peach/perception/debug_image` | `sensor_msgs/Image` bgr8（debug 叠加图） |
| pub | `/peach/perception/target_observations` | `peach_pose_msgs/PeachTargetObservationArray`（稳定 ID、计划、逐目标结果与深度时间戳掩膜；`collecting_count`/`pending_count` 发现进度摘要——锁定前 observations 恒空，进度看这两字段） |
| pub | `/peach/perception/harvest_state` | `std_msgs/String`（JSON，全局计划快照，transient-local；含同名 `collecting_count`/`pending_count` 键与 `timing` 子对象——detect/segment/geometry/total 各段耗时 EMA 毫秒 + 实测 fps） |

发布面为 `/peach/perception/*` 单套规范话题（A5 起旧 `~/` 组已删除；2D 候选
不再单独成话题，随 `target_observations` 的 `candidate_2d` 字段下发）。
frame_id 约定：三维结果（候选/拟合/Marker/检测点云）为
`output_frame`（TF 失败的帧退回相机系并打 `tf_unavailable`）；图像平面数据
（2D 候选/detections/整幅 masks/debug_image）使用图像坐标语义。三维结果的
header.stamp 与 TF 查询统一使用 `depth.header.stamp`；普通图像平面调试结果保持
RGB 时间戳；`target_observations` 内逐目标 mask 特意使用深度时间戳，供重建精确匹配。

### 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `/peach_pose_node/query_harvest_state` | `std_srvs/srv/Trigger` | 返回本轮 run ID、锁定数量、优先级、完成集合和 selected ID |
| `/peach_pose_node/complete_selected_target` | `std_srvs/srv/Trigger` | 抓取与撤离确认后标记当前目标完成并推进 |
| `/peach_pose_node/reset_global_targets` | `std_srvs/srv/Trigger` | 放弃旧计划并开始新的全局观察轮次 |

三态：`ACCEPT=0` / `REOBSERVE=1` / `REJECT=2`。SAM 缺失显式 `mask_unavailable`，禁止静默回退。

### 模型

`model/best.pt`（约 20 MB）、`model/mobile_sam.pt`（约 41 MB），从
peach_canopy 复制；`yolo_model_path` / `sam_model_path` 留空即按包内
默认路径加载。
