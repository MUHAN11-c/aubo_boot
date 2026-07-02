# ROS2 机械臂咖啡拉花轨迹建立 -- 资料汇总

本文件夹整理了 ROS2 机械臂咖啡拉花轨迹建立相关的所有调研资料，按主题分类存放。

## 目录结构

| 文件夹 | 内容 |
|--------|------|
| `论文资料/` | 学术论文索引与摘要，含直接相关论文和轨迹规划基础论文 |
| `开源项目/` | GitHub 开源项目详细信息、技术栈、轨迹方法分析 |
| `轨迹生成方案/` | 四种技术路线详解 + Python 轨迹生成参考代码 |
| `拉花图案与动作/` | 人类拉花动作分解 → 机械臂轨迹参数映射 |
| `ROS2实现参考/` | MoveIt2、ros2_control、笛卡尔规划等 ROS2 实现参考 |
| `视频资料/` | B站/YouTube 拉花教程视频 + **完整 29 分钟英文字幕转写** |
| `拉花图案与动作/` | **双源验证**: Sunergos + latteartguide + Clive Coffee + PDG → 机械臂轨迹参数映射 |
| `轨迹生成方案/` | 四种技术路线 + **多阶段精确轨迹模型** Python 代码 (已用视频参数校正) |

## 核心结论

目前**没有现成的开源 ROS2 拉花轨迹专用库**，但有以下实现路线：

1. **快速原型**：MoveIt2 笛卡尔路径规划 + 多阶段轨迹模型 (融合→推云→回拉+摆幅递减→收拢→划穿)
2. **高质量效果**：Pi0/LeRobot 模仿学习路线，参考 `latte-art-robot` 项目
3. **关键难点**：抗晃荡速度规划 + 奶泡流量控制 (pitch角动态调整) + 液面高度实时检测 (<0.5mm)

## 多源教程交叉验证的关键参数

| 参数 | Sunergos | latteartguide | Clive Coffee | **机械臂建议** |
|------|----------|---------------|-------------|--------------|
| 融合高度 | 7.6cm+ | — | 10-12cm | **8-10cm** |
| 成形高度 | 0.6cm内 | 抵杯沿 | 贴液面 | **0.3-0.5cm** |
| 收尾高度 | 7.6cm+ | 竖直抬升 | 数英寸上 | **7-8cm** |
| 杯子倾斜 | 30-45° | — | — | **30-40°** |
| Rosetta 摆幅 | — | **递减** | — | A0→0 linear ramp |

## 参考资料

- [latte-art-robot (Pi0 + ROS2)](https://github.com/ridxm/latte-art-robot)
- [Automated-Latte-Art-YuMi (MATLAB + CoppeliaSim)](https://github.com/TranThanhTuan2509/Automated-Latte-Art-with-Using-ABB-s-YuMi)
- [B站: 牛奶科学、奶泡和拉花专业教程](https://www.bilibili.com/video/BV1uE411g7MJ/)
- 数据集: HuggingFace `ridxm/latte-pour-demos`
