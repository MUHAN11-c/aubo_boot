# latte_imitation — 拉花轨迹模仿学习包

从 [ridxm/latte-pour-demos](https://huggingface.co/datasets/ridxm/latte-pour-demos) 数据集提取双臂拉花末端笛卡尔轨迹，
通过 Aubo E5 IK 转换为关节轨迹并发送到机械臂执行。

## 架构

```
原始 parquet (RM65 关节角)       笛卡尔 npz (末端位姿，预计算)
        │                                │
        │  [一次性: RM65 FK]               │ ← 直接加载
        └──────────→ ────────────────────┘
                         │
                         ▼
                CartesianTrajectory.load()
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
     visualize (matplotlib)    ROS2 Node
                               │
                          debug: PoseStamped + Path
                          action: Aubo E5 IK → JointTrajectory → Action
```

## 模块

| 文件 | 职责 |
|------|------|
| `trajectory.py` | `CartesianTrajectory` 类：加载/保存 npz，统计（路径长、速度），导出 ROS2 消息 |
| `robot_model.py` | PyKDL FK + DLS IK（action 模式用） |
| `trajectory_publisher.py` | ROS2 节点：加载 npz → 发布位姿 → Aubo IK → Action 执行 |
| `scripts/visualize_latte_trajectory.py` | 离线播放器：40 条轨迹叠加 + 动画回放 |

## 数据文件

```
resource/
├── original/               ← 40 个原始 14D parquet（RM65 关节角，保留）
├── cartesian/
│   ├── left/               ← 40 个左臂末端笛卡尔 npz (400×3 positions + 400×4 orientations)
│   └── right/              ← 40 个右臂末端笛卡尔 npz
└── trajectory_overview.png ← 40 条右臂轨迹叠加图
```

每个 npz 包含：
- `positions` (400, 3) — 末端 XYZ 位置 (m)
- `orientations` (400, 4) — 四元数 xyzw
- `timestamps` (400,) — 从 0 开始，dt=0.05s
- `dt` — 时间步长
- `episode_idx` — episode 编号

## 使用方法

### 构建

```bash
# 必须在 workspace 根目录运行 colcon
cd aubo_ros2_ws
colcon build --packages-select latte_imitation
source install/setup.bash
```

### 离线可视化（不依赖 ROS2）

```bash
cd src/latte_imitation
python3 scripts/visualize_latte_trajectory.py                 # 右臂拉花轨迹（默认）
python3 scripts/visualize_latte_trajectory.py --arm left      # 左臂持杯轨迹
python3 scripts/visualize_latte_trajectory.py --speed 2.0     # 2 倍速
```

键盘：`[ ]` 切 episode | 空格 播放/暂停 | `← →` 逐帧 | `↑ ↓` 变速 | `a` 叠加 | `r` 重置

### ROS2 Debug 模式（发布末端位姿）

```bash
ros2 launch latte_imitation replay_trajectory.launch.py
ros2 launch latte_imitation replay_trajectory.launch.py episode_idx:=5 arm:=left

# 查看发布的轨迹
ros2 topic echo /latte_imitation/ee_path
```

### ROS2 Action 模式（机械臂执行）

```bash
ros2 launch latte_imitation replay_trajectory.launch.py \
    mode:=action episode_idx:=0 arm:=right speed_scale:=0.5

# 批量执行 40 条
for ep in $(seq 0 39); do
    ros2 launch latte_imitation replay_trajectory.launch.py \
        mode:=action episode_idx:=$ep speed_scale:=1.0
    sleep 22
done
```

### Python API

```python
from latte_imitation.trajectory import CartesianTrajectory

# 加载全部 40 条
carts = CartesianTrajectory.load_all("resource", "right")
for ep, cart in carts.items():
    print(f"Ep{ep}: {cart.path_length():.2f}m, {cart.num_frames} frames")

# 加载单条
cart = CartesianTrajectory.load("resource/cartesian/right/episode_000000.npz")
print(cart.start)   # [-0.34, 0.028, 0.521]
print(cart.end)     # [-0.34, 0.010, 0.585]

# 导出 ROS2 消息
path_msg = cart.to_ros2_path()
pose_msg = cart.to_pose_stamped(10)
```

## 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `episode_idx` | 0 | episode 编号 (0-39) |
| `arm` | right | left / right |
| `speed_scale` | 1.0 | 播放速度倍率 |
| `mode` | debug | debug=PoseStamped+Path, action=IK+执行 |
| `pos_only` | true | IK 仅匹配位置（忽略姿态） |

## CartesianTrajectory API

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `load(path)` | `CartesianTrajectory` | 从 npz 加载单条 |
| `load_all(dir, arm)` | `OrderedDict[int, CartesianTrajectory]` | 加载目录下全部 npz |
| `save(path)` | — | 保存为 npz |
| `path_length()` | float | 累计路径长 (m) |
| `velocity_profile()` | ndarray (T-1,) | 帧间瞬时速度 (m/s) |
| `segment(i, j)` | `CartesianTrajectory` | 切片子轨迹 |
| `to_pose(idx)` | `Pose` | 单帧 ROS2 Pose |
| `to_pose_stamped(idx)` | `PoseStamped` | 单帧 PoseStamped |
| `to_ros2_path(step=5)` | `Path` | 完整 ROS2 Path |

属性: `positions` (T,3), `orientations` (T,4), `timestamps` (T,), `dt`, `start`, `end`, `num_frames`

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/latte_imitation/ee_pose` | PoseStamped | 当前帧末端位姿 |
| `/latte_imitation/ee_path` | Path | 完整轨迹（每 5 帧采样） |
| `/latte_imitation/aubo_ee_pose` | PoseStamped | IK 验证位姿（action 模式） |
| `/latte_imitation/joint_path_command` | JointTrajectory | 关节轨迹（action 模式） |

## 数据集信息

- 来源：[ridxm/latte-pour-demos](https://huggingface.co/datasets/ridxm/latte-pour-demos)
- 40 个 episode，每个 400 帧 @ 20fps（20 秒）
- 14D 关节角：左臂 dims 0-6 + 右臂 dims 7-13 + 夹爪 dim 13
- 右臂（拉花臂）末端路径长约 1.53m，分四阶段：进杯 → 调整 → 核心拉花 → 退杯

### 14 维关节分解

| 维度 | 所属 | 运动特征 |
|------|------|---------|
| dim 0-5 | 左臂 6 关节 | 几乎静止（持杯） |
| dim 6 | 左臂夹爪 | 恒定 0.948 |
| dim 7-12 | **右臂 6 关节** | **拉花运动** |
| dim 13 | 右臂夹爪 | 恒定 0.991 |

### 笛卡尔末端轨迹（右臂，RM65 FK）

| 轴 | 范围 (m) | 跨度 (mm) |
|---|---|---|
| X | [-0.365, -0.153] | 212 |
| Y | [-0.094, 0.028] | 121 |
| Z | [0.173, 0.585] | 413 |

- 直线位移 67mm，总路径 **1.53m**
- 40 条轨迹帧间 std 均值仅 0.044m，高度一致

## 数据流详解

### 数据生成（一次性）

原始 parquet 是 RM65 关节角，不可直接用于 Aubo E5。通过 RM65 FK 预计算生成笛卡尔 npz：

```
resource/original/ (40 个 parquet, 14D 关节角)
    │
    │  RM65 FK (robot_model.py)
    ▼
resource/cartesian/left/  (40 个 npz, 3D 位置 + 4D 姿态)
resource/cartesian/right/ (40 个 npz)
```

之后所有模块只加载笛卡尔 npz，不再依赖 parquet 和 RobotModel（RM65）。

### 节点执行流程

```
CartesianTrajectory.load(npz)
    │
    ▼
[debug] → to_ros2_path() → /latte_imitation/ee_path
    │
[action] → _pose_to_kdl_frame() → Aubo IK → JointTrajectory → Action
    │
    ├─ IK 成功帧: 正常执行
    └─ IK 失败帧: 跳过，不包含在 JointTrajectory 中
```

### 设计决策

| 决策 | 原因 |
|------|------|
| 只存笛卡尔 npz | 关节角跨机械臂不可复用，笛卡尔位姿是通用的 |
| 保留原始 parquet | 可追溯数据来源，需要时可重新 FK 计算 |
| pos_only IK | RM65 和 Aubo E5 姿态差异大，只匹配位置避免 IK 发散 |
| 不包含 retarget | 数据集是在 cup 上方的相对运动，无需 workspace 映射 |
| `load_all` 返回 OrderedDict | 按 episode 编号排序，遍历顺序可预测 |

## YOLO26 训练与推理

### 环境

| 项目 | 详情 |
|------|------|
| Ultralytics 版本 | **8.4.49**（pip 安装，`pip3 show ultralytics`） |
| 源码参考 | `yolov26_src/` — 只作参考，实际训练推理走 pip 安装的包 |
| 模型权重 | `yolo26n.pt` (5.3MB), `yolo26m.pt` (44.3MB), `yolo26x.pt` (113MB) |
| 数据集 | `datasets/coco128/` (128 张, 7.5MB, 仅功能验证) |
| GPU | RTX 3090 24GB, CUDA 13.2, PyTorch 2.12.0 |

> `yolov26_src/` 是 ultralytics 官方源码副本，**未 `pip install -e .` 到环境中**。`yolov6/` 同。训练推理脚本走系统安装的 `ultralytics==8.4.49`。
>
> 确认当前版本：`python3 -c "import ultralytics; print(ultralytics.__version__)"` → 8.4.49

### YOLO26 模型规格

| 模型 | 参数量 | GFLOPs | 权重大小 | RTX 3090 推理 |
|------|--------|--------|---------|-------------|
| yolo26n | 2.4M | 5.4 | 5.3MB | 1.0ms |
| yolo26m | 20.4M | 68.2 | 44.3MB | 2.9ms |
| yolo26x | 59.0M | 209.5 | 113.2MB | 不可训练 (OOM) |

核心特性：NMS-Free 端到端推理、MuSGD 优化器、移除 DFL、CPU 推理速度提升 43%。

### 训练

```bash
# 基础训练（脚本自动使用包目录下的 .pt 权重）
python3 scripts/train_yolo26.py

# 完整参数示例
python3 scripts/train_yolo26.py \
    --model m \
    --data datasets/coco128.yaml \
    --epochs 300 \
    --batch 64 \
    --device 0 \
    --half \
    --project runs/yolo26 \
    --name my_experiment
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | n | 模型规模 n/s/m/l/x |
| `--data` | datasets/coco128.yaml | 数据集 yaml |
| `--epochs` | 300 | 训练轮数 |
| `--batch` | -1 (自动) | batch size |
| `--imgsz` | 640 | 输入尺寸 |
| `--device` | 0 | GPU 编号 / cpu |
| `--half` | False | FP16 半精度 |
| `--cache` | False | 数据集缓存到 RAM |
| `--resume` | False | 从 checkpoint 恢复 |

### 推理

```bash
# 单图推理
python3 scripts/infer_yolo26.py --source image.jpg

# 目录批量
python3 scripts/infer_yolo26.py --source images/ --save-txt

# 摄像头实时
python3 scripts/infer_yolo26.py --source 0 --show

# 指定模型和阈值
python3 scripts/infer_yolo26.py --model m --source img.jpg --conf 0.5 --save-crop
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--source` | (必填) | 图片/目录/摄像头编号 |
| `--model` | n | 模型规模 n/s/m/l/x |
| `--weight` | — | 自定义权重路径（覆盖 `--model`） |
| `--conf` | 0.25 | 置信度阈值 |
| `--iou` | 0.7 | NMS IOU 阈值 |
| `--show` | False | 实时显示结果 |
| `--save-txt` | False | 保存 YOLO 格式 txt |
| `--save-crop` | False | 裁剪保存检测目标 |

### GPU 训练基准（300 epochs × COCO128）

#### yolo26n（轻量基线）

```
yolo train model=yolo26n.pt data=coco128.yaml epochs=300 device=0
```

| 指标 | 数值 |
|------|------|
| Batch size | 16 (默认) |
| **GPU 显存** | **4.26 GB (18%)** |
| GPU 利用率 | ~40% |
| 训练速度 | 12-13 it/s |
| 总训练时间 | **0.094 小时 (5.6 分钟)** |
| 最佳 mAP50 | 0.901 |

#### yolo26m（压榨 GPU）

```
yolo train model=yolo26m.pt data=coco128.yaml epochs=300 batch=64 device=0 workers=4
```

| 指标 | 数值 |
|------|------|
| Batch size | **64** |
| **GPU 显存** | **17.9 GB (73%)** |
| GPU 利用率 | **100%** |
| GPU 功耗 | **337W / 350W** |
| 总训练时间 | **0.249 小时 (15 分钟)** |
| 最佳 mAP50 | 0.977 |

### 性能瓶颈分析

#### 1. 数据集过小

COCO128 仅 128 张图，几分钟跑完，mAP 虚高无参考价值。真实训练需用完整 COCO（118K 张）或自定义工业数据集。

#### 2. yolo26x 显存溢出

yolo26x (59M 参数, 209 GFLOPs) 在当前环境中无法训练：

| 尝试配置 | 结果 | 原因 |
|----------|------|------|
| batch=48, imgsz=640 | CUDA OOM | 模型 + AMP 超出 24GB |
| batch=24 (auto-reduce) | 仍然 OOM | 同上 |

RTX 3090 24GB 训练 yolo26x 的方案：`--half` (FP16)、`--cache` (缓存到 RAM)、减小 `--imgsz 480`、或多卡/云 GPU。

#### 3. 僵尸进程显存泄漏

OOM 崩溃后 PyTorch DataLoader worker 进程可能残留 CUDA context 占用显存：

```bash
fuser -v /dev/nvidia* | grep pt_data_worker | awk '{print $2}' | xargs kill -9
```

#### 4. 模型 vs 显存（实测）

| 模型 | batch=16 显存 | 最大可用 batch | 推荐 batch |
|------|-------------|---------------|-----------|
| yolo26n | ~4.3 GB | ~128 | 16-64 |
| yolo26m | ~13 GB | ~64 | 32-64 |
| yolo26x | — | <16 (不可用) | GPU 不足 |

### 依赖

- numpy, PyKDL, urdf_parser_py
- ROS2 Humble: rclpy, trajectory_msgs, geometry_msgs, nav_msgs, control_msgs, ament_index_python
- 可视化: matplotlib
- YOLO26: ultralytics>=8.4.0, torch>=1.8.0
