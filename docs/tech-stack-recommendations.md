# IVG2.0 技术栈选型推荐

> 原则：**前沿 × 广泛使用 × 生态抗衰退** — GitHub Stars / 下载量 / 学术占比 / 社区采用率 均为硬指标喵~

## 完整技术栈对照表

| 层级 | 当前选型 | 推荐选型 | 推荐理由（量化依据） | 优先级 | 备注 |
|------|----------|----------|---------------------|--------|------|
| **深度学习框架** | PyTorch ✅ | **PyTorch** 保持 | 学术论文 55%+ 占比，HuggingFace 85%+ 模型 native，158k Stars，LLM/VLA 生态全部基于 PyTorch | — | 无需变更喵~ |
| **ROS 2** | Humble (2022 LTS) | Humble → **"L" Turtle (2026 LTS)** | Humble 市场占比 48.53%，2027.05 EOL；"L" Turtle 预计 2026.05 发布，支持至 2031 年 | 中 | 等 2026 Q3 生态稳定后迁移喵~ |
| **VLA 基座模型** | 无 | **OpenVLA** (1.1k+ Stars) | MIT 许可证完全开源，训练代码+权重全公开，被 OFT/FAST/RT-VLA 等大量工作作为基座，生态最广 | 高 | 学院派基石模型，备选 OpenPI π₀ (9.4k Stars) 喵~ |
| **VLA 实验平台** | 无 | **StarVLA** (2.2k+ Stars) | 统一 backbone × action header 自由组合，支持 8+ Benchmark，港科大+开源社区 | 高 | 用于多模型公平对比喵~ |
| **机器人学习框架** | 无 | **LeRobot** (24k Stars) | ICLR 2026 接收，HuggingFace 背书收购 Pollen Robotics，200万+轨迹段数据集超 Google Open-X+RT-1 总和，硬件→训练→部署闭环 | 高 | 端到端平台，NVIDIA 合作加速喵~ |
| **LLM 推理（服务端）** | 无 | **vLLM** (74k Stars) | PagedAttention + Continuous Batching，50并发 920 tok/s，OpenAI 兼容 API，AWS/Databricks/IBM 生产采用 | 中 | 生产环境默认选择喵~ |
| **LLM 推理（边缘端）** | 无 | **llama.cpp** (100k Stars) | C++ 纯实现无依赖，CUDA/ROCm/Metal/Vulkan/WebAssembly 全覆盖，GGUF 格式行业标准 | 中 | 机器人端侧推理，CPU/ARM 可用喵~ |
| **LLM 本地开发** | 无 | **Ollama** (120k Stars) | 一句命令启动，52M 月下载量，开发者体验第一 | 低 | ⚠️ 仅开发/原型用，不可上生产（5并发即崩）喵~ |
| **机器人仿真（AI/视觉）** | mock_components | **NVIDIA Isaac Sim** | GPU 加速 PhysX 5，万倍实时并行仿真，光线追踪渲染，Isaac Lab 框架，2025 年开源 Apache 2.0，中国人形机器人公司标配 | 中 | 需 NVIDIA RTX GPU，备选 MuJoCo 喵~ |
| **机器人仿真（RL研究）** | — | **MuJoCo + MJX** | 接触动力学业界最佳，MJX (JAX GPU) 70× 加速，RL 论文学术标准，DeepMind 维护 Apache 2.0 开源 | 中 | 轻量级，无需高端 GPU 喵~ |
| **ROS 2 集成仿真** | — | **Gazebo Harmonic** | ROS 2 集成最深，最大社区和插件生态，LTS 至 2028 | 低 | 仅用于 ROS 2 集成测试喵~ |
| **前端框架** | 原生 JS + Web Components | **Vue 3 + TypeScript** | 52.8k Stars，npm 843万周下载，开发者保留率 87%（React 75%），中国市场 ~50% 渗透率，学习曲线平缓，Pinia 状态管理 80% 采用率，Vue 3 采用率 82%，渐进式迁移友好 | 高 | 国内大厂（阿里/腾讯/百度）广泛使用，团队中文生态适配好喵~ |
| **后端框架** | FastAPI + Flask 并存 | **FastAPI 统一**（Litestar 观察） | FastAPI 92k⭐ / 450万日下载 / 38%开发者采用率 / OpenAI+Anthropic+MS+Netflix 生产使用 / AI 生态(LangChain/LlamaIndex/HF)默认后端；Flask 同步老旧应统一。Litestar (5.9k⭐) 技术更优(2×吞吐/10×省内存/团队维护/内置JWT+CSRF+限流)但生态太小，列为长期观察对象 | 中 | 先统一 Flask→FastAPI；若3年后 Litestar 达临界质量(15k+⭐)且 AI 生态跟上再评估迁移喵~ |
| **容器化** | 无 | **Docker + Compose** | 96% 容器评估组织采用，ROS/ROS 2 官方镜像丰富，nvidia-docker GPU 透传成熟 | 高 | 单机开发/部署最优解喵~ |
| **容器编排** | 无 | **K3s**（多机器人时） | CNCF 沙箱项目，82% 生产采用 K8s，K3s 是轻量版（~500MB，5分钟安装），适配 Jetson/ARM 边缘设备 | 低 | 集群规模 < 5 机器人用 Compose 即可喵~ |
| **MLOps 实验追踪** | 无 | **MLflow** (自托管) | 2025 年 57% 采用率 (+15pp)，Apache 2.0 开源，16M+ 月下载，端到端 MLOps 平台 | 中 | W&B 被 CoreWeave 收购后存在厂商锁定风险喵~ |
| **数据版本控制** | 无 | **DVC** | Git 原生集成，S3/GCS/Azure 后端存大文件，与 MLflow 互补 | 中 | MLflow + DVC = 完整开源 MLOps 喵~ |
| **Python** | 3.10 | 3.10 → **3.12**（随 ROS 2 迁移） | 3.11 性能提升 10-60%，3.12 额外 5%+，ROS 2 Humble 绑定 3.10 | 低 | 跟随 ROS 2 大版本一同升级喵~ |
| **MoveIt 2** | 本地复刻源码 (2.5.9) | **apt 安装** 版本 | 本地复刻导致升级困难，应验证无本地补丁后移除 | 低 | 需先调查是否有未合并的本地补丁喵~ |
| **构建系统** | colcon (ament) ✅ | **colcon** 保持 | ROS 2 标准构建工具，无需变更 | — | 无需变更喵~ |
| **数据录制** | rosbag2 ✅ | **rosbag2** 保持 | ROS 2 标准数据录制，无需变更 | — | 无需变更喵~ |

## 优先级实施路径

```
🔴 本周可做:
  1. 移除 moveit_ros_* 本地源码复刻（验证无本地补丁后删除）
  2. 统一 Flask → FastAPI（手眼标定 Web UI）

🟡 本月可做:
  3. 引入 Docker + Compose 开发环境
  4. 前端 React + TypeScript 渐进式迁移（先做一个面板试点）
  5. 引入 MLflow + DVC

🟢 本季度可做:
  6. 集成 LeRobot，开始采集操作数据集
  7. 搭建 vLLM 推理服务
  8. 引入 OpenVLA + StarVLA 实验框架

🔵 2026 Q3-Q4:
  9. ROS 2 Humble → "L" Turtle LTS 迁移
 10. Isaac Sim / MuJoCo 引入（视 RL 训练需求）
```

---

---

## 附录：Python 后端框架深度对比

### 核心指标

| 维度 | FastAPI | Litestar | Django Ninja | BlackSheep |
|------|---------|----------|--------------|------------|
| GitHub Stars | **92k** | 5.9k | ~7k | ~1.5k |
| PyPI 日下载 | **450万** | 增长中 | 少量 | 极少 |
| 维护模式 | ⚠️ 单人 (tiangolo) | ✅ **团队维护** | ⚠️ 单人 | 社区 |
| 开发者采用率 | **38%** | < 1% | < 1% | < 0.1% |
| 生产用户 | OpenAI/Anthropic/MS/Netflix/Uber | 少数 | 依赖 Django | 极少 |

### 性能基准 (Uvicorn 4 workers, 1000并发)

| 框架 | 吞吐量 | 平均延迟 | 内存(100路由) |
|------|--------|----------|--------------|
| BlackSheep | 7,271 req/s | 130ms | — |
| Litestar | 4,743 req/s | 192ms | **57 MB** |
| FastAPI | 2,488 req/s | 398ms | 116 MB |

### 功能内置 vs 外挂

| 功能 | FastAPI | Litestar |
|------|---------|----------|
| JWT 认证 | ❌ 第三方 | ✅ 内置 |
| CSRF 防护 | ❌ 第三方 | ✅ 内置 |
| 频率限制 | ❌ 第三方 | ✅ 内置 |
| 会话管理 | ❌ 第三方 | ✅ 内置 |
| Prometheus 指标 | ❌ 第三方 | ✅ 内置 |
| 缓存 | ❌ 第三方 | ✅ 内置 |
| DTO 层 | ❌ 无 | ✅ 内置 |
| OpenAPI | 3.0 | **3.1** |

### 最终推荐

- **当前**：FastAPI 保持不变（已有代码 + AI 生态绑定 + 社区海量）
- **Flask 手眼标定**：迁移到 FastAPI 统一技术栈
- **Litestar**：列为长期观察对象——若 3 年内达 15k+ Stars 且 AI 生态跟上，可评估迁移
- **Django Ninja**：不推荐（单人维护，已出现社区 fork `Django Shinobi`）

> 来源：[Rollbar 2026](https://rollbar.com/blog/python-backend-frameworks/) | [Better Stack](https://betterstack.com/community/guides/scaling-python/litestar-vs-fastapi/) | [Uvik 2026](https://uvik.net/blog/python-api-framework/) | [FastAPI Deconstructed](https://discuss.whatever.social/r/Python/comments/1ifu2sv/fastapi_deconstructed_anatomy_of_a_modern_asgi/)

---

*最后更新: 2026-05-12*
*基于：GitHub Stars、npm 下载量、学术论文占比、Stack Overflow 调查、CNCF 报告、ROS Metrics Report 2025*
