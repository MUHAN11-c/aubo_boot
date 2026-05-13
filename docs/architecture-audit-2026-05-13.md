# Architecture.md 对照审查报告

> 审查时间: 2026-05-13 | 基准文档: `docs/architecture.md`

---

## 一、决策逐条对照

| # | 架构决策 | 当前状态 | 评估 |
|---|---------|---------|------|
| 1 | Web 框架统一为 FastAPI | `hand_eye_calibration` 仍用 Flask :8080，其余已 FastAPI | ⚠️ 待迁移 |
| 2 | 前端选 Vue 3 + TS | 当前原生 JS + Web Components，`aubo_ros2_web_dashboard/web/public/` | ❌ 未开始 |
| 3 | ivg_utils 零 ROS 依赖 | **已验证** — 仅依赖 numpy，无 rclpy/sensor_msgs 导入 | ✅ 合规 |
| 4 | 感知层包间仅通过 ROS 2 接口通信 | `graspnet_ros2` 通过 `sys.path` 导入 `graspnet-baseline` 内部模块（models/backbone等）| ⚠️ 部分违规 |
| 5 | Web 层通过 rosbridge 桥接 | 已验证 — 网关 8090 代理 rosbridge 9090 | ✅ 合规 |
| 6 | 保留 SDK 双连接架构 | `conn_control_` + `conn_status_` 架构不变 | ✅ 合规 |
| 7 | **移除 MoveIt 2 本地源码复刻** | `moveit_ros_planning/`、`moveit_ros_planning_interface/`、`moveit_ros_visualization/` 仍在 src/ 中（v2.5.9，与 apt 同版本，无本地补丁） | ❌ 未执行 |
| 8 | IO 引脚语义统一为命名常量 | `ivg_utils/io.py` 已定义 `IO_GRIPPER=6`, `IO_QUICK_SWAP=7`，但各 worker 仍用硬编码数字 | ⚠️ 定义完毕，未全面使用 |
| 9 | **GraspNet sys.path 改为标准依赖** | pointnet2/knn/graspnetAPI 已 `pip install -e .`；但 `models/`、`utils/`、`dataset/` 仍通过 `sys.path.insert` 导入 | ⚠️ 部分完成 |
| 10 | Python 3.10 → 3.12 | 当前 3.10.12，等待 "L" Turtle 迁移 | ⏳ 待定 |
| 11 | Humble → "L" Turtle | 当前 ROS 2 Humble | ⏳ 待定 |
| 12-14 | 仿真/Docker/VLA | 后续参考 | ⏳ 远期 |

---

## 二、硬编码路径

### 🔴 关键（运行时代码）

| 文件 | 行号 | 内容 |
|------|------|------|
| `hand_eye_calibration/hand_eye_calibration/hand_eye_calibration_node.py` | 1594, 1992 | `os.path.join('/home/mu/IVG', ...)` — 旧工作空间路径，**运行时创建目录会失败** |

### 🟡 中等（构建产物）

- `aubo_driver_ros2/lib/lib64/deps/` 下 `.la` / `.pc` 文件 — 15+ 处嵌入构建路径
- `tool_changer/urdf/*.urdf` — 自动生成注释含绝对路径

### 🟢 信息性（文档示例）

- 10+ README 文件中 `cd /home/mu/IVG2.0/...` shell 示例
- `aubo_moveit_config/README.md` 14 处硬编码路径
- `robotwebtools/README.md` 11 处硬编码路径

---

## 三、sys.path 运行时注入（决策 #9）

| 文件 | 行 | 路径 | 严重度 |
|------|-----|------|--------|
| `graspnet_ros2/graspnet_demo_points_node.py` | 60-63 | 4 处 `sys.path.insert/append` (models/dataset/utils) | 🔴 生产代码 |
| `visual_pose_estimation_python/test/test_web_app.py` | 16 | | 🟡 测试 |
| `latte_imitation/scripts/visualize_latte_trajectory.py` | 33 | | 🟡 脚本 |
| `aubo_ros2_web_dashboard/launch/web_dashboard.launch.py` | 30 | | 🟡 launch |
| `hand_eye_calibration/tests/*.py` | 4 处 | | 🟢 测试 |

已修复（本次审查前）：pointnet2/knn/graspnetAPI 三个包已改为 `pip install -e .`。

待修复：`graspnet-baseline` 的 `models/`、`utils/`、`dataset/` 仍为 `sys.path` 导入，未作为独立包安装。

---

## 四、代码重复

### 四元数/旋转矩阵函数

| 函数 | 重复次数 | 分布 |
|------|---------|------|
| `quaternion_to_rotation_matrix` | 6 处 | `hand_eye_calibration`(4), `visual_pose_estimation`(2) |
| `rotation_matrix_to_quaternion` | 9 处 | `hand_eye_calibration`(5), `visual_pose_estimation`(4) |
| `rotation_matrix_to_euler_rpy` | 3 处 | `visual_pose_estimation`(3) |
| `normalize_angle_to_180` | 1 处 | `visual_pose_estimation`(1) |

`ivg_utils/math.py` 已提供上述全部 4 个函数，但调用方本地仍有私有副本未删除。

### YOLO 节点

| 文件 | 类 | 共享基类 |
|------|-----|---------|
| `vision_perception/yolo_detect_node.py` | `YoloDetectNode(Node)` | ❌ |
| `vision_perception/yolo_obb_node.py` | `YoloOBBNode(Node)` | ❌ |
| `vision_perception/yolo_track_node.py` | `YoloTrackNode(Node)` | ❌ |

3 个节点各自继承 `rclpy.Node`，无共享基类。模型加载、CV Bridge、结果发布模式高度相似。

### 单体文件

`hand_eye_calibration_node.py` — **5945 行**（架构文档声称 5958 行，偏差 ~0.2%），需拆分为 6 个模块。

---

## 五、MoveIt 2 本地复刻（决策 #7）

| 目录 | 版本 | 与 apt 版本差异 | 可移除？ |
|------|------|---------------|---------|
| `src/moveit_ros_planning/` | 2.5.9 | 仅 `package.xml` URL 字符串差异（`moveit.ai` vs `moveit.ros.org`），无实质代码补丁 | ✅ 是 |
| `src/moveit_ros_planning_interface/` | 2.5.9 | 同上 | ✅ 是 |
| `src/moveit_ros_visualization/` | 2.5.9 | 同上 + 3 个 PNG 图标压缩差异 | ✅ 是 |

外部依赖（`demo_driver`、`tool_changer`）通过标准 `find_package()` / `<depend>` 引用，移除本地副本后 CMake 会从 `/opt/ros/humble/share/` 找到相同版本的 apt 包。**可释放 ~3.7MB 磁盘空间。**

---

## 六、ivg_utils 使用现状

`ivg_utils` **零 ROS 依赖已验证** ✅

### 使用 ivg_utils 的包

| 包 | 导入内容 |
|---|---------|
| `hand_eye_calibration` | `quaternion_to_rotation_matrix`（4 文件） |
| `visual_pose_estimation` | 通过 `math_utils.py` 垫片全量重导出 |

### 未使用 ivg_utils 的包

`graspnet_ros2`、`coffee_latte_demo`、`latte_imitation`、`tool_changer`、`vision_perception`、`camport_ros2`

> 注：这些包目前未发现自己的重复数学函数，不需要强制引入 ivg_utils。但如果后续添加旋转/四元数运算，应优先使用 `ivg_utils.math` 而非本地实现。

---

## 七、包命名一致性

### 命名模式分布

| 模式 | 数量 | 包 |
|------|------|-----|
| `ivg_*` (目标前缀) | **1** | `ivg_utils` |
| `aubo_*` (硬件层，可接受) | 5 | `aubo_description`, `aubo_driver_ros2`, `aubo_moveit_config`, `aubo_msgs`, `aubo_ros2_web_dashboard` |
| `demo_*` (待改名) | 2 | `demo_driver`, `demo_interface` |
| `*_interface` (待合并) | 3 | `demo_interface`, `tool_changer_interface`, `interface` |
| `*_demo` (教学包为主) | 4 | `coffee_latte_demo`, 3 个教程包 |
| 其他 (待改名) | 8 | `graspnet_ros2`, `vision_perception`, `visual_pose_estimation_python`, `hand_eye_calibration`, `latte_imitation`, `tool_changer` + 3 个 MoveIt (待移除) |
| 教程 (无关) | 3 | `myrobot_description`, `xarm_description`, `xarm_moveit_config` |

### 接口包碎片化

架构 §3 要求 `demo_interface` + `tool_changer_interface` → 合并为 `ivg_service_interfaces`。此外 `visual_pose_estimation/interface`（裸包名 `interface`）语义模糊，也应合并。

---

## 八、优先级建议

### 🔴 高优先级（应在下次启动前修复）

1. **MoveIt 本地复刻移除**（决策 #7）— 3 个 `moveit_ros_*` 目录加 `COLCON_IGNORE` 或直接删除
2. **硬编码路径修复**（`hand_eye_calibration_node.py` 1594/1992 行）— `/home/mu/IVG` → 可配置目录或 `$HOME`

### 🟡 中优先级（下一个迭代）

3. **GraspNet sys.path 完全消除**（决策 #9）— `models/` + `utils/` + `dataset/` 改为标准包导入
4. **setup.py 幽灵入口点已清理** ✅（本次审查前完成）
5. **代码重复清理** — `hand_eye_calibration` 和 `visual_pose_estimation` 的私有数学函数 → 统一调用 `ivg_utils.math`
6. **YOLO 基类提取** — 3 个节点提取 `YoloBaseNode`

### 🟢 低优先级（架构重构阶段）

7. **包命名统一** — 8 个项目包改为 `ivg_*` 前缀
8. **接口包合并** — 3 个 `*_interface` → `ivg_service_interfaces`
9. **Flask → FastAPI 迁移**（手眼标定）
10. **Vue 3 前端重写**
11. **`hand_eye_calibration_node.py` 拆分** — 5945 行 → 6 模块

---

## 九、本次审查已修复项

以下问题在本次审查前/审查中已完成修复：

- ✅ pointnet2/knn/graspnetAPI `pip install -e .`（消除 3 个 CUDA 扩展的 sys.path 依赖）
- ✅ `graspnet_ros2` 幽灵 `console_scripts` 入口点清理
- ✅ `graspnet_ros2` 冗余文件删除（21 个文件）
- ✅ `start_aubo_new_driver.sh` 文档同步（`LD_LIBRARY_PATH`、依赖安装步骤）
- ✅ `DEPLOYMENT.md` / `aubo_ros2_ws/README.md` / 项目 `README.md` 同步更新
- ✅ `ivg_utils` 零 ROS 依赖已验证
