# 机械臂咖啡拉花方案

## 总体架构

```
网络视频 → YOLO OBB 跟踪拉花缸壶嘴 → 2D像素轨迹 → 单应矩阵2D→3D
    → 轨迹平滑(B-spline/低通滤波) → IK → JointTrajectory
    → /execute_trajectory → Aubo E5 执行
```

## 阶段划分

### Phase 1: 视频 → 2D像素轨迹 (当前阶段)
- 目标检测/跟踪拉花缸(牛奶缸/milk pitcher)
- 提取壶嘴(spout)像素轨迹
- 输出: 时间序列 (x,y) 像素坐标

### Phase 2: 2D → 3D 映射
- 相机内参标定
- 相机→机器人手眼标定
- 桌面平面单应矩阵
- 输出: 机器人基坐标系下 3D waypoints

### Phase 3: 轨迹 → 执行
- 轨迹平滑 + 降采样
- IK求解(使用 /get_ik 或 KDL)
- 通过 /execute_trajectory 发送
- 实时速度/加速度控制

---

## Phase 1 详细设计

### 1.1 拉花缸检测方案

| 方案 | 模型 | 优点 | 缺点 |
|------|------|------|------|
| A. COCO预训练检测 | YOLO (cup/bottle类) | 零训练,开箱即用 | cup≠pitcher, 精度可能不够 |
| B. 传统CV颜色分割 | HSV阈值+轮廓 | 不锈钢反光特征明显,零训练 | 光照敏感 |
| C. 微调YOLO检测 | 标注50-100张图fine-tune | 精度高,鲁棒 | 需要标注数据 |
| D. OBB旋转框 | YOLO OBB fine-tune | 倾斜拉花缸更准确 | 需要旋转框标注 |

**推荐先A+B混合快速验证, 需要精度时再C+D。**

### 1.2 壶嘴提取

在检测框内:
- 框内Canny边缘检测
- 找到最底部的尖端点(pitcher spout通常指向斜下方)
- 或用GrabCut/轮廓分析提取壶嘴

### 1.3 轨迹后处理

- 时间序列平滑: 指数移动平均(EMA) α=0.3
- 异常值剔除: 检测框突然跳变时丢弃
- 降采样: 30fps → 10-15Hz (机械臂执行频率)

---

## 开源项目参考

### 最相关
| 项目 | Stars | 关键技术 | 适配度 |
|------|-------|----------|--------|
| ridxm/latte-art-robot | 2 | ROS2+π0 VLA+LeRobot,双臂拉花 | 高-执行层可改造 |
| Pavan-43/Autonomous-Barista-Robot | 1 | ROS2+MoveIt2+UR5,6种咖啡 | 高-架构参考 |
| hyeonguk65/robotArm_bartender | 0 | ROS2+Doosan+YOLO+LLM,真机倒饮料 | 高-视觉管线 |
| UBC-OpenRobotics/6DoF-Arm | 1 | ROS2+6DOF+YOLO,杯具识别 | 中-视觉部分 |
| mouse826612011/DMP_Simulation | 12 | DMP+PyBullet+Panda臂 | 中-轨迹学习 |

### 仿真/研究
| 项目 | Stars | 关键技术 |
|------|-------|----------|
| medevilunknown/Water-pouring-robot-simulation | 0 | MuJoCo+YOLOv8+Llama3+倒水 |
| AswinRetnakumar/Pouring_Project | 0 | PyBullet+Kuka+RL倒水 |
| akihikoy/lfd_sim | 1 | ROS1+ODE+LfD倒水仿真 |

### 模仿学习框架
| 项目 | Stars | 关键技术 |
|------|-------|----------|
| MarkFzp/mobile-aloha | 4400 | ACT+双臂移动操作 |
| MarkFzp/act-plus-plus | 3600 | ACT+Diffusion Policy |
| Skylark0924/Rofunc | 709 | LfD全流程(GMR/TPGMM/LQT) |

### 视觉感知
| 项目 | Stars | 关键技术 |
|------|-------|----------|
| hetolin/PourIt | 29 | ICCV2023,弱监督液体感知,ROS1 |

---

## 模型/数据集现状

### 现有模型
- `yolo26n-obb.pt` (本地): DOTA遥感15类, **不可用于拉花**
- COCO预训练模型: 有cup/bottle/bowl/spoon类, 但无milk pitcher
- **无公开拉花缸检测数据集或预训练模型**

### 策略
1. 先用COCO "cup"/"bottle"类做粗糙检测验证pipeline
2. 同时用HSV颜色分割(不锈钢=高亮度低饱和度)做壶嘴定位
3. 如果精度不够, 标注50-100张图微调YOLO
