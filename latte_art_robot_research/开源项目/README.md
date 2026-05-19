# 开源项目

## 1. latte-art-robot ⭐ 最推荐

- **链接**: https://github.com/ridxm/latte-art-robot
- **语言**: Python (83.9%), Docker
- **星标**: 2
- **活动**: Physical AI Hack 2026 参赛项目

### 技术栈
- **机械臂**: OpenDroid R2D3 + Realman RM65 双臂
- **软件**: ROS2 + Pi0 (Physical Intelligence VLA 模型) + LeRobot
- **感知**: 三摄像头系统 (640x480@20Hz)
- **控制器**: Flow matching 动作生成 + temporal ensembling 平滑

### 轨迹方法
模仿学习路线：
1. 通过示教背心采集运动示教数据（40 个 episode）
2. 使用 flow matching 生成动作序列
3. Action chunk size = 20, temporal ensembling 平滑输出

### 数据集
- HuggingFace: `ridxm/latte-pour-demos`
- 格式: LeRobot v3.0 (Parquet + MP4)

### 为什么是最佳参考
- **唯一使用 ROS2 的拉花项目**
- 双臂协作架构（倒奶 + 拉花）
- 数据公开，可复现

---

## 2. Automated-Latte-Art-with-Using-ABB-s-YuMi

- **链接**: https://github.com/TranThanhTuan2509/Automated-Latte-Art-with-Using-ABB-s-YuMi
- **语言**: MATLAB
- **星标**: 5

### 技术栈
- **机械臂**: ABB YuMi 双臂 (14 DOF)
- **仿真**: V-REP / CoppeliaSim
- **计算**: Maple (D-H 参数) + MATLAB (运动学)

### 轨迹方法
经典运动学路线：
1. 建立 D-H 参数表
2. 正向/逆向运动学求解
3. Monte Carlo 工作空间分析
4. 位置/速度/加速度轨迹生成

### 特点
- 双臂协作（一臂取浓缩咖啡，一臂倒奶泡）
- 附带 PDF 报告（越南语）
- 运动学推导完整，可移植到 ROS2

---

## 3. Latte-Art-Robot (angrocki)

- **链接**: https://github.com/angrocki/Latte-Art-Robot
- **语言**: Python (57.1%), C++ (42.9%)
- **星标**: 0

### 技术栈
- G-code 控制（CNC 式运动）
- 含 UI 界面和传感器模块
- **无 ROS**

### 轨迹方法
- G-code 驱动，类似 3D 打印机/CNC 机床
- 将拉花图案转换为 G-code 指令序列

### 参考价值
- G-code 转机械臂轨迹的思路可借鉴
- UI 界面设计参考

---

## 4. starbots_coffee_dispenser

- **链接**: https://github.com/kuralme/starbots_coffee_dispenser
- **语言**: Python, Docker
- **星标**: 3

### 技术栈
- ROS2 + Behavior Trees + OpenCV + Docker
- 机器人咖啡机（**非拉花**，但 ROS2 架构参考价值高）

### 参考价值
- ROS2 Behavior Tree 架构设计
- 咖啡制作流程的状态机/行为树建模

---

## 5. coffee_bot

- **链接**: https://github.com/LAV2000/coffee_bot
- **星标**: 1
- **技术栈**: ROS2
- **用途**: Coffee Robot using ROS2

---

## 6. robotics_brewbuddy

- **链接**: https://github.com/Bendemeurichy/robotics_brewbuddy
- **星标**: 1
- **技术栈**: ROS2 + TurtleBot
- **用途**: TurtleBot 送咖啡

---

## 项目对比总结

| 项目 | ROS | 轨迹方法 | 仿真 | 实际硬件 | 推荐度 |
|------|-----|---------|------|---------|--------|
| latte-art-robot | ROS2 ✅ | 模仿学习 (Pi0) | ❌ | RM65 双臂 | ⭐⭐⭐ |
| Automated-Latte-Art-YuMi | ❌ | 运动学+插值 | CoppeliaSim ✅ | ABB YuMi | ⭐⭐⭐ |
| Latte-Art-Robot (angrocki) | ❌ | G-code | ❌ | 未知 | ⭐⭐ |
| starbots_coffee | ROS2 ✅ | Behavior Tree | ❌ | 未知 | ⭐⭐ |
