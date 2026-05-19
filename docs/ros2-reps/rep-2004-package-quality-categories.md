# REP 2004 -- Package Quality Categories

- **Status**: Active
- **Type**: Informational
- **Source**: https://www.ros.org/reps/rep-2004.html

## 质量等级概览

| 等级 | 定位 | 示例 |
|------|------|------|
| Level 1 | 生产系统必需 | rclcpp, urdf, tf2 |
| Level 2 | 高质量通用方案，可向 L1 演进 | - |
| Level 3 | 开发/工具用 | ros2cli, rviz, rqt |
| Level 4 | Demo/教程/实验 | demo_nodes_cpp |
| Level 5 | 默认等级，不满足 L4 | - |

## Level 1 要求摘要

### 版本策略
- 必须有版本策略（如 semver）
- 版本 >= 1.0.0（稳定版）
- 明确声明的公开 API
- API/ABI 稳定性策略
- 在一个 ROS 发行版内保持 API/ABI 稳定

### 变更控制
- 所有代码变更通过 PR/MR
- 贡献者来源确认（DCO/CLA）
- 同行评审策略
- CI 策略
- 文档策略

### 文档
- 每个功能有文档
- 公开 API 每个条目有文档
- 声明许可证
- 版权声明
- 质量声明文档（QUALITY_DECLARATION.md）

### 测试
- 系统测试覆盖所有功能文档
- 覆盖所有公开 API 的测试
- 代码覆盖率跟踪和策略
- 性能测试（如适用）
- 代码风格 + 静态分析

### 依赖
- 直接 ROS 依赖不低于同一等级
- 非 ROS 依赖需论证等效质量

### 平台支持
- 支持所有 Tier 1 平台（见 REP 2000）

### 安全
- 漏洞披露策略

完整内容见：https://www.ros.org/reps/rep-2004.html
