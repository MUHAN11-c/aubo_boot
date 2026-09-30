# 论文资料

## 一、直接相关论文（咖啡拉花 / 液体操作）

### 1. LATTE: LAnguage Trajectory TransformEr
- **来源**: arXiv:2208.02918, Bucker et al. 2022
- **核心方法**: BERT + CLIP 编码用户意图 → Transformer 编码器-解码器 → 直接生成 3D 轨迹
- **亮点**: 支持自然语言交互修改轨迹（如"把轨迹往左移一点"）
- **与本项目关系**: 提供了一种端到端轨迹生成思路，自然语言 → 拉花轨迹

### 2. Pi0: A Vision-Language-Action Flow Model for General Robot Control
- **来源**: arXiv:2410.24164, RSS 2025, Physical Intelligence
- **核心方法**: Flow matching VLA 模型，预训练 VLM (PaliGemma)，支持双臂/单臂操作
- **亮点**: 已实际用于咖啡拉花，是 `latte-art-robot` 项目的核心模型
- **与本项目关系**: 模仿学习路线的基础模型，需要示教数据 fine-tune

### 3. RoboPaint: From Human Demonstration to Any Robot and Any View
- **来源**: arXiv:2602.05325, Fan et al. 2026
- **核心方法**: 人类示教 → 仿真重映射 → 部署到任意机器人
- **硬件**: 3 RGB-D + 11 RGB 摄像头 + 触觉手套
- **结果**: 基于 Pi0.5，成功率 80%
- **与本项目关系**: 示教数据跨机器人迁移的关键技术

### 4. MiDiGap: Discrete-Time Gaussian Process Mixtures for Robot Policy Learning
- **来源**: arXiv:2505.03296, von Hartz et al. 2025
- **核心方法**: 5 个演示即可学习，CPU 上分钟级训练
- **亮点**: 以 making coffee 为长时域任务示例，轨迹成本降低 67%
- **与本项目关系**: 少样本学习，适合快速部署新拉花图案

### 5. Learning Compositional Models of Robot Skills for Task and Motion Planning
- **来源**: IJRR 2021, Wang et al.
- **核心方法**: 以 making coffee 为主要示例任务，输出关节角路径
- **与本项目关系**: 咖啡制作任务的运动规划参考

### 6. Explainable Hierarchical Imitation Learning for Robotic Drink Pouring
- **来源**: IEEE Trans. 2021, Zhang et al., 59 次引用
- **核心方法**: 分层模仿学习用于倒饮料
- **与本项目关系**: 倒奶阶段的轨迹参考，分层架构可借鉴

## 二、轨迹规划核心技术论文

### 7. PathFormer: Transformer with 3D Grid Constraints
- **来源**: arXiv:2510.20161, Alanazi et al. 2025
- **核心方法**: 3D 网格约束 Transformer (what/when/where 编码)，端到端成功率 86.7%
- **测试平台**: xArm Lite 6
- **与本项目关系**: 深度轨迹生成的最新方法

### 8. Time-Optimal Handling of Liquids (抗晃荡轨迹)
- **来源**: U. Bologna PhD Thesis 2023, Di Leva
- **核心方法**: 时间最优 + 加速度/jerk 约束 → 液体防晃荡轨迹规划
- **与本项目关系**: **拉花倒奶的关键技术** -- 防止液体晃动导致图案破坏

### 9. One-shot Video Imitation via PSAG
- **来源**: arXiv:2408.12674, Wang et al. 2024
- **核心方法**: 从单段视频学习，包含 Pouring Liquid 任务，可泛化到新物体
- **与本项目关系**: 从拉花教学视频中直接学习轨迹的可能性

### 10. Hierarchical Neural Dynamic Policies
- **来源**: arXiv:2107.05627, Bahl et al. 2021
- **核心方法**: 模仿 + 强化学习，包含 pouring 动态任务
- **与本项目关系**: 动态倒液的分层策略

### 11. Intelligent Master-Slave Collaborative Robot for Cafeteria Service
- **来源**: Robotics and Autonomous Systems 2022, Gao et al.
- **核心方法**: 餐厅服务机器人的快速平滑轨迹规划
- **与本项目关系**: 实际部署的餐饮服务机器人轨迹参考

### 12. MIME: Multiple Interactions Made Easy
- **来源**: arXiv:1810.07121, Sharma et al. 2018
- **核心方法**: 8260 个人类-机器人示教数据集，包含 pouring 任务
- **与本项目关系**: 最大的 pouring 示教数据集，可用于预训练

## 三、推荐阅读优先级

| 优先级 | 论文 | 理由 |
|--------|------|------|
| ⭐⭐⭐ | Di Leva 2023 (抗晃荡) | 拉花成败的关键技术 |
| ⭐⭐⭐ | Pi0 (arXiv:2410.24164) | 学习路线的核心模型 |
| ⭐⭐⭐ | LATTE (arXiv:2208.02918) | 最接近"语言→拉花轨迹"目标的论文 |
| ⭐⭐ | PathFormer (arXiv:2510.20161) | 最新深度轨迹生成方法 |
| ⭐⭐ | RoboPaint (arXiv:2602.05325) | 示教迁移到新机器人 |
| ⭐ | MiDiGap (arXiv:2505.03296) | 少样本快速学习 |
