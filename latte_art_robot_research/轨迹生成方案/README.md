# 轨迹生成方案

## 四种技术路线对比

| 方案 | 原理 | 数据需求 | 泛化能力 | 实现难度 | 计算需求 | 适用场景 |
|------|------|---------|---------|---------|---------|---------|
| 经典运动学+插值 | D-H → IK → 样条插值 | 无 | 低 | ⭐⭐ | 低 | 固定图案重复拉花 |
| 模仿学习 (VLA) | 示教 → VLA模型 → 动作生成 | 40+ episodes | 高 | ⭐⭐⭐⭐ | 高(GPU) | 变化图案拉花 |
| Transformer 生成 | 语言/图像 → Transformer → 轨迹 | 大量训练数据 | 很高 | ⭐⭐⭐⭐⭐ | 很高(GPU) | 自然语言交互式拉花 |
| 贝塞尔+抗晃荡 | 路径参数化 + 约束优化 | 无 | 中 | ⭐⭐⭐ | 中 | 精确控制的定制图案 |

---

## 方案一：经典运动学 + 轨迹插值

### 流程
```
D-H参数建模 → 正向运动学 → 工作空间分析 (Monte Carlo)
    → 定义末端轨迹点 → 样条/贝塞尔插值 → 逆运动学 → 关节角轨迹
```

### 工具链
- **运动学**: KDL (ROS2 内置) / Trac-IK / IKFast
- **仿真**: CoppeliaSim / Gazebo (Ignition) + ROS2
- **插值**: CubicSpline / B-Spline / Bezier (scipy)

### 轨迹定义伪代码
```python
# 定义拉花图案的路径点
pattern_points = [
    (x1, y1),  # 起点
    (x2, y2),  # 推注点
    ...
]
# 添加高度剖面
z_profile = [z_mix, z_draw, z_draw, ..., z_finish]
# 3D 轨迹
trajectory = [(x, y, z) for (x,y), z in zip(pattern_points, z_profile)]
# 样条插值平滑
from scipy.interpolate import CubicSpline
cs = CubicSpline(waypoint_indices, trajectory)
smooth_traj = cs(t_dense)
# MoveIt2 执行
move_group.compute_cartesian_path(smooth_traj)
```

### 优点
- 可解释性强，轨迹精确可控
- 不需要训练数据
- 计算量小，可在嵌入式设备运行

### 缺点
- 需要手动为每种图案编写轨迹
- 缺乏柔性，难以适应变化（如杯子位置偏移）

---

## 方案二：模仿学习 (VLA)

### 流程
```
示教采集 → 多模态数据(关节角 + RGB视频) → VLA模型训练 → flow matching 生成动作
```

### 核心组件

#### 示教采集
- 硬件: 示教背心 / 主从遥操作 / kinesthetic teaching
- 数据格式: LeRobot v3.0 (Parquet + MP4)
- 建议量: 每个图案 30-50 个 episode

#### 模型: Pi0 (Physical Intelligence)
```python
# 基于 Pi0 的拉花轨迹生成
# 输入: 多视角图像 + 机器人状态
# 输出: action chunk (20步关节角)
# 模型: PaliGemma (VLM) + flow matching head
```

#### 训练
- 预训练模型: Pi0/Pi0.5 (无需从头训练)
- Fine-tune: 在自己的示教数据上微调
- 框架: LeRobot (HuggingFace)

### 关键项目
- `latte-art-robot`: Pi0 + ROS2 + RM65 双臂
- HuggingFace 数据集: `ridxm/latte-pour-demos`

### 优点
- 无需显式设计轨迹，模型自动学习
- 可泛化到新图案、新杯子位置
- 可适应环境变化

### 缺点
- 需要高质量示教设备
- 模型推理需要 GPU
- 行为可解释性差

---

## 方案三：Transformer 生成轨迹

### 流程
```
自然语言 / 图像图案 → 编码器 → Transformer 编码器-解码器 → 3D 轨迹点序列
```

### 代表性工作
- **LATTE** (arXiv:2208.02918): BERT+CLIP → Transformer → 轨迹
  - 输入: "画一个心形拉花，大一点"
  - 输出: 3D 轨迹点序列
- **PathFormer** (arXiv:2510.20161): 3D 网格约束 Transformer
  - 3-grid 编码 (what/when/where)
  - 测试于 xArm Lite 6, 成功率 86.7%

### 优点
- 最自然的交互方式（说话就能改变图案）
- 可处理复杂约束

### 缺点
- 需要大量训练数据
- 计算资源需求高
- 当前为研究阶段，工程化不成熟

---

## 方案四：贝塞尔曲线 + 抗晃荡优化（推荐入门路线）

### 流程
```
拉花图案 → 轮廓提取 → 2D路径点 → 贝塞尔曲线拟合
    → 添加高度剖面 Z(t) → 添加摆动叠加 X_osc(t)
    → 速度剖面(抗晃荡约束) → MoveIt2 CartesianPath
```

### 1. 图案到路径点
```python
import cv2
import numpy as np

def pattern_to_waypoints(image_path, num_points=100):
    """从拉花图案图像提取路径点"""
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    # 边缘检测
    edges = cv2.Canny(img, 50, 150)
    # 轮廓提取
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
    main_contour = max(contours, key=cv2.contourArea)
    # 采样路径点
    indices = np.linspace(0, len(main_contour)-1, num_points, dtype=int)
    return main_contour[indices].reshape(-1, 2)
```

### 2. 贝塞尔曲线拟合
```python
from scipy.interpolate import splprep, splev

def fit_bezier_trajectory(waypoints_2d, smoothness=0.5):
    """贝塞尔/样条拟合平滑轨迹"""
    tck, u = splprep([waypoints_2d[:, 0], waypoints_2d[:, 1]], s=smoothness)
    u_new = np.linspace(0, 1, 200)
    x_smooth, y_smooth = splev(u_new, tck)
    return np.column_stack([x_smooth, y_smooth])
```

### 3. 抗晃荡速度剖面
```python
def anti_sloshing_velocity(distance, v_max=0.05, a_max=0.1, jerk_max=0.5):
    """
    计算满足抗晃荡约束的 S 曲线速度剖面
    - 通过限制 jerk（加加速度）来平滑速度变化
    - 避免液体晃动导致奶泡图案变形
    """
    # S曲线参数 (7段式)
    t_j = a_max / jerk_max          # jerk时间
    t_a = v_max / a_max - t_j       # 加速度段时间
    # ... 计算完整 S 曲线速度剖面
    return velocity_profile
```

### 4. MoveIt2 集成
```python
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetCartesianPath

def execute_latte_art_trajectory(move_group, trajectory_xyz):
    """通过 MoveIt2 执行笛卡尔轨迹"""
    waypoints = xyz_to_poses(trajectory_xyz)
    # compute_cartesian_path 返回成功率
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        eef_step=0.001,     # 末端步长 1mm
        jump_threshold=0.0  # 禁用跳跃
    )
    if fraction >= 0.95:    # 路径规划成功率 >= 95%
        move_group.execute(plan)
    return fraction
```

### 优点
- 精确可控，适合定制图案
- 无需训练数据
- 计算量小
- 可以逐步加入约束

### 缺点
- 每种新图案需要参数化定义
- 对杯子位置偏移需要额外处理

---

## 推荐实施步骤 (方案四 → 方案二)

```
第一阶段 (快速验证):
  贝塞尔轨迹 + MoveIt2 仿真 → Gazebo 中完成拉花动作
  
第二阶段 (加入约束):
  + 抗晃荡速度剖面 + 力控反馈 → 实际倒奶验证
  
第三阶段 (智能化):
  引入 Pi0 模仿学习 → 从示教数据学习新图案
```
