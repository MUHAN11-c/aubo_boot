# 套袋桃采摘机器人 · 项目设计书

本目录为工程**项目设计书**交付（对照 GB/T 8567 概要设计、ISO/IEC/IEEE 42010 体系结构描述、ROS 2 / Autoware / Nav2 / UR Driver 主流画法）。不是 `docs/` 三份活文档的替代件：现行运行快照仍以 `docs/architecture.md`、`docs/io.md`、`docs/testing.md` 为准。

| 文件 | 对应系统 | 内容 |
|------|----------|------|
| [peach_v1_套袋桃采摘机器人项目设计书.md](peach_v1_套袋桃采摘机器人项目设计书.md) | 现行 `src/peach_*` 采摘核（v1） | 需求、硬件/软件架构、三剪切手、安全、**真相机+仿真全流程抓取**测试 |
| [peach_v2_套袋桃采摘机器人项目设计书.md](peach_v2_套袋桃采摘机器人项目设计书.md) | `src/peach2/*` 重构栈 + 后续已落地优化 | 重写原则、包切分、RSS 预算、BT+MTC、命令门、已实施优化 |
| [v1_三末端混合测试/](v1_三末端混合测试/) | v1 附录试验 | 三末端 HIL：真实立体相机在环 + mock 臂一次完整抓取，**墙钟 ≤ 120 s** |

硬件基线两边相同：**现场主控 NVIDIA Jetson Orin NX 16GB**；计算压力大时增加 **Rockchip RK3588** 做采集/预处理分流。开发与本轮 HIL 在工控机（Ubuntu 24.04 + ROS 2 Jazzy + RTX 3090）上跑 mock 臂，相机走实验室 Percipio 链路。
